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

        self.lio_pose_file = os.path.join(self.output_dir, 'lio-poses.txt')
        self.cam_pose_file = os.path.join(self.output_dir, 'camera-poses.txt')

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        self.lio_pose_fp = open(self.lio_pose_file, 'a')
        self.cam_pose_fp = open(self.cam_pose_file, 'a')

        self.get_logger().info(f'Data will be saved to: {self.output_dir}')

        # =============================
        # Frame definitions
        # =============================
        self.map_frame = 'map'
        self.lio_frame = 'lio'
        self.camera_frame = 'camera'

        # =============================
        # TF
        # =============================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # =============================
        # Pose saving timer
        # =============================
        self.timer = self.create_timer(
            0.02, self.pose_timer_callback
        )  # 50Hz

        # 防止重复位姿
        self.last_pose_stamp = None

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

        self.sync.registerCallback(self.rgbd_callback)

        # =============================
        # Extrinsic
        # =============================
        self.T_cl = np.array([
            [0.03109787, -0.0627963,  0.99754175, -0.20589067],
            [-0.0627963,  0.04941931,  0.99680206, -0.10058281],
            [0.99754175, -0.02801317,  0.06423174, -1.1367896],
            [0.0,         0.0,         0.0,         1.0]
        ], dtype=np.float64)

        self.T_lb = np.array([
            [1.0, 0.0, 0.0, 0.011],
            [0.0, 1.0, 0.0, 0.02329],
            [0.0, 0.0, 1.0, -0.04412],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # T_bc = (T_cl^-1 * T_lb)^-1
        self.T_bc = np.linalg.inv(np.linalg.inv(self.T_cl) @ self.T_lb)

        self.get_logger().info('RGB-D + pose data collection started')

    # =========================================================
    # RGBD callback (only save image)
    # =========================================================
    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):

        stamp = rgb_msg.header.stamp
        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"

        # -----------------------------
        # RGB
        # -----------------------------
        rgb_img = self.bridge.imgmsg_to_cv2(
            rgb_msg, desired_encoding='bgr8'
        )

        cv2.imwrite(
            os.path.join(self.rgb_dir, f'{timestamp}.png'),
            rgb_img
        )

        # -----------------------------
        # Depth
        # -----------------------------
        depth_encoding = depth_msg.encoding

        depth_img = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        if depth_encoding == '16UC1':

            depth_to_save = depth_img.astype(np.uint16)

        elif depth_encoding == '32FC1':

            depth_mm = depth_img * 1000.0
            depth_mm = np.nan_to_num(
                depth_mm, nan=0.0, posinf=0.0, neginf=0.0
            )
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

        self.get_logger().info(f'Saved RGBD @ {timestamp}')

    # =========================================================
    # Pose timer callback
    # =========================================================
    def pose_timer_callback(self):

        try:

            tf_map_lio = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lio_frame,
                rclpy.time.Time()
            )

        except Exception:
            return

        stamp = tf_map_lio.header.stamp

        # =============================
        # Skip duplicated pose
        # =============================
        if self.last_pose_stamp is not None:

            if (stamp.sec == self.last_pose_stamp.sec and
                stamp.nanosec == self.last_pose_stamp.nanosec):
                return

        self.last_pose_stamp = stamp

        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"

        T_map_lio = self.tf_to_matrix(tf_map_lio)

        # -----------------------------
        # save LIO pose
        # -----------------------------
        self.write_pose(
            self.lio_pose_fp,
            timestamp,
            T_map_lio
        )

        # -----------------------------
        # compute camera pose
        # -----------------------------
        T_map_cam = T_map_lio @ self.T_bc

        self.write_pose(
            self.cam_pose_fp,
            timestamp,
            T_map_cam
        )

    # =========================================================
    # Pose write helper
    # =========================================================
    def write_pose(self, fp, timestamp, T):

        t = T[0:3, 3]
        R = T[0:3, 0:3]

        qw, qx, qy, qz = tq.mat2quat(R)

        fp.write(
            f"{timestamp} "
            f"{t[0]} {t[1]} {t[2]} "
            f"{qx} {qy} {qz} {qw}\n"
        )

        fp.flush()

    # =========================================================
    @staticmethod
    def tf_to_matrix(tf_msg: TransformStamped) -> np.ndarray:

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        R = tq.quat2mat(
            [q.w, q.x, q.y, q.z]
        )

        T = np.eye(4)

        T[0:3, 0:3] = R
        T[0:3, 3] = [t.x, t.y, t.z]

        return T

    # =========================================================
    def destroy_node(self):

        self.lio_pose_fp.close()
        self.cam_pose_fp.close()

        super().destroy_node()


# =============================================================
def main():

    parser = argparse.ArgumentParser(
        description='RGB-D + LiDAR pose data collector'
    )

    parser.add_argument(
        '--output_dir',
        type=str,
        default='~/rgbd_lio_dataset',
        help='Output directory'
    )

    args = parser.parse_args()

    rclpy.init()

    node = RGBDLidarMatrixSync(
        args.output_dir
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()