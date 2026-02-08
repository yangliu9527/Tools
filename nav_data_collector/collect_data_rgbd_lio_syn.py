#!/usr/bin/env python3
import os
import argparse

import rclpy
from rclpy.node import Node

import numpy as np
import transforms3d.quaternions as tq

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

import message_filters
import tf2_ros


class RGBDLidarMatrixSync(Node):
    def __init__(self, output_dir: str):
        super().__init__('rgbd_lidar_matrix_sync')

        # =============================
        # Output directory
        # =============================
        self.output_dir = os.path.expanduser(output_dir)
        self.rgb_dir = os.path.join(self.output_dir, 'rgb')
        self.depth_dir = os.path.join(self.output_dir, 'depth')
        self.pose_file = os.path.join(self.output_dir, 'pose.txt')

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        self.pose_fp = open(self.pose_file, 'a')

        self.get_logger().info(f'Data will be saved to: {self.output_dir}')

        # =============================
        # Frame definitions
        # =============================
        self.map_frame = 'map'
        self.lidar_frame = 'lidar'
        self.camera_frame = 'camera'

        # =============================
        # TF
        # =============================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # =============================
        # CV bridge
        # =============================
        self.bridge = CvBridge()

        # =============================
        # RGB / Depth sync
        # =============================
        rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw'
        )
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw'
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.callback)

        # =============================
        # Lidar -> Camera extrinsic (4x4)
        # =============================
        self.T_lidar_camera = np.array([
            [1.0, 0.0, 0.0, 0.10],
            [0.0, 1.0, 0.0, 0.00],
            [0.0, 0.0, 1.0, 0.20],
            [0.0, 0.0, 0.0, 1.00],
        ], dtype=np.float64)

        self.T_ic = np.array(
            [[-0.01658374, -0.06612253,  0.99769604, -0.08455519],
             [ 1.00364341, -0.05075506,  0.0200464,   0.1859044 ],
             [-0.05195012,  1.0016635,   0.06484486, -1.10611519],
             [ 0.          ,  0.          ,  0.          , 1.        ]], dtype=np.float64)

        self.get_logger().info('RGB-D + pose data collection started')

    def callback(self, rgb_msg: Image, depth_msg: Image):
        # =============================
        # Timestamp (ROS time)
        # =============================
        stamp = rgb_msg.header.stamp
        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
        ros_time = rclpy.time.Time.from_msg(stamp)

        # =============================
        # Lookup map -> lidar TF
        # =============================
        try:
            tf_map_lidar = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lidar_frame,
                ros_time,
                timeout=rclpy.duration.Duration(seconds=0.03)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # =============================
        # Compute map -> camera
        # =============================
        T_map_imu = self.tf_to_matrix(tf_map_lidar)
        T_map_camera = T_map_imu @ self.T_ic

        # =============================
        # Save RGB
        # =============================
        rgb_img = self.bridge.imgmsg_to_cv2(
            rgb_msg, desired_encoding='bgr8'
        )
        cv2.imwrite(
            os.path.join(self.rgb_dir, f'{timestamp}.png'),
            rgb_img
        )

        # =============================
        # Save Depth (with encoding check)
        # =============================
        depth_encoding = depth_msg.encoding
        depth_img = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        if depth_encoding == '16UC1':
            # already in millimeters
            depth_to_save = depth_img.astype(np.uint16)

        elif depth_encoding == '32FC1':
            # meters -> millimeters
            depth_mm = depth_img * 1000.0

            # 防止 NaN / inf / 负值
            depth_mm = np.nan_to_num(depth_mm, nan=0.0, posinf=0.0, neginf=0.0)
            depth_mm[depth_mm < 0] = 0

            depth_to_save = depth_mm.astype(np.uint16)

        else:
            self.get_logger().warn(
                f'Unsupported depth encoding: {depth_encoding}'
            )
            return

        cv2.imwrite(
            os.path.join(self.depth_dir, f'{timestamp}.png'),
            depth_to_save
        )


        # =============================
        # Save Pose
        # =============================
        t = T_map_camera[0:3, 3]
        R = T_map_camera[0:3, 0:3]
        qw, qx, qy, qz = tq.mat2quat(R)

        self.pose_fp.write(
            f"{timestamp} "
            f"{t[0]} {t[1]} {t[2]} "
            f"{qx} {qy} {qz} {qw}\n"
        )
        self.pose_fp.flush()

        self.get_logger().info(f'Saved frame @ {timestamp}')

    @staticmethod
    def tf_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        R = tq.quat2mat([q.w, q.x, q.y, q.z])

        T = np.eye(4, dtype=np.float64)
        T[0:3, 0:3] = R
        T[0:3, 3] = [t.x, t.y, t.z]
        return T

    def destroy_node(self):
        self.pose_fp.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='RGB-D + LiDAR pose data collector'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        default='~/rgbd_lio_dataset',
        help='Output directory for rgb/depth/pose'
    )
    args = parser.parse_args()

    rclpy.init()
    node = RGBDLidarMatrixSync(args.output_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
