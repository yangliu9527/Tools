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
from nav_msgs.msg import Odometry

import message_filters


class RGBDLidarMatrixSync(Node):
    def __init__(self, output_dir: str):
        super().__init__('rgbd_lidar_matrix_sync')

        # =============================
        # Output directory
        # =============================
        self.output_dir = os.path.expanduser(output_dir)
        self.rgb_dir = os.path.join(self.output_dir, 'rgb')
        self.depth_dir = os.path.join(self.output_dir, 'depth')
        self.pose_file = os.path.join(self.output_dir, 'poses.txt')

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        self.pose_fp = open(self.pose_file, 'a')
        self.get_logger().info(f'Data will be saved to: {self.output_dir}')

        # =============================
        # CV bridge
        # =============================
        self.bridge = CvBridge()

        # =============================
        # Subscribers
        # =============================
        rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        # depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        pose_sub = message_filters.Subscriber(self, Odometry, 'lightning/odometry')  # 假设 LIO 发布的是 Odometry

        # =============================
        # Approximate synchronizer
        # =============================
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, pose_sub],
            queue_size=50,
            slop=0.02
        )
        self.sync.registerCallback(self.callback)

        # =============================
        # Extrinsics
        # =============================
        #D455
        # self.T_cl = np.array([
        # [0.03109787, -0.0627963,  0.99754175, -0.20589067],
        # [0.99680206, -0.0627963,  0.04941931,  -0.10058281],
        # [-0.02801317, 0.99754175, 0.06423174, -1.1367896],
        # [0.0,         0.0,         0.0,         1.0]
        # ], dtype=np.float64)

        # 336L
        self.T_cl = np.array([
        [0.09629, 0.98253, 0.15929, -0.33069],
        [0-0.55413, -0.08002, 0.82858,  0.58819],
        [0.82684, -0.16805, 0.53674, -0.17265],
        [0.0,         0.0,         0.0,         1.0]
        ], dtype=np.float64)

        self.T_lb = np.array([
            [1.0, 0.0, 0.0, 0.011],
            [0.0, 1.0, 0.0, 0.02329],
            [0.0, 0.0, 1.0, -0.04412],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        self.T_bc = np.linalg.inv(self.T_cl @ self.T_lb)

        # self.T_bc = np.array(
        #     [[-0.01658374, -0.06612253,  0.99769604, -0.08455519],
        #      [ 1.00364341, -0.05075506,  0.0200464,   0.1859044 ],
        #      [-0.05195012,  1.0016635,   0.06484486, -1.10611519],
        #      [ 0.          ,  0.          ,  0.          , 1.        ]], dtype=np.float64)

        self.get_logger().info('RGB-D + LIO pose data collection started (buffered sync)')

    def callback(self, rgb_msg: Image, depth_msg: Image, pose_msg: Odometry):
        # =============================
        # Timestamp
        # =============================
        stamp = rgb_msg.header.stamp
        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"

        # =============================
        # LIO pose -> camera
        # =============================
        # 从 odom_msg 提取 pose
        t = pose_msg.pose.pose.position
        q = pose_msg.pose.pose.orientation

        # 转成 4x4 矩阵
        R = tq.quat2mat([q.w, q.x, q.y, q.z])
        T_wb = np.eye(4)
        T_wb[0:3, 0:3] = R
        T_wb[0:3, 3] = [t.x, t.y, t.z]
        T_wc = T_wb @ self.T_bc


        # =============================
        # Save RGB
        # =============================
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.rgb_dir, f'{timestamp}.png'), rgb_img)

        # =============================
        # Save Depth
        # =============================
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_img = np.nan_to_num(depth_img, nan=0.0, posinf=0.0, neginf=0.0)
        depth_img[depth_img < 0] = 0
        if depth_msg.encoding == '32FC1':
            depth_img = (depth_img * 1000).astype(np.uint16)
        elif depth_msg.encoding == '16UC1':
            depth_img = depth_img.astype(np.uint16)
        cv2.imwrite(os.path.join(self.depth_dir, f'{timestamp}.png'), depth_img)

        # =============================
        # Save Pose
        # =============================
        t = T_wc[0:3, 3]
        R = T_wc[0:3, 0:3]
        qw, qx, qy, qz = tq.mat2quat(R)
        self.pose_fp.write(f"{timestamp} {t[0]} {t[1]} {t[2]} {qx} {qy} {qz} {qw}\n")
        self.pose_fp.flush()

        self.get_logger().info(f'Saved frame @ {timestamp}')


    def destroy_node(self):
        self.pose_fp.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='RGB-D + LIO pose data collector')
    parser.add_argument('--output_dir', type=str, default='~/rgbd_lio_dataset', help='Output directory')
    args = parser.parse_args()

    rclpy.init()
    node = RGBDLidarMatrixSync(args.output_dir)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()