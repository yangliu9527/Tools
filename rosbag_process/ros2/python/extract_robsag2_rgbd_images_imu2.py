#!/usr/bin/env python3
import os
import argparse
import numpy as np
import cv2

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, Imu


def write_txt(f, data):
    for i, line in enumerate(data):
        f.write(" ".join(map(str, line)))
        if i != len(data) - 1:
            f.write("\n")
    f.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract RGB / Depth / IMU from rosbag2 with frame skipping"
    )
    parser.add_argument(
        "--bag_path",
        type=str,
        required=True,
        help="rosbag2 文件夹路径（不带 .db3 后缀）"
    )
    parser.add_argument(
        "--output_path",
        type=str,
        required=True,
        help="输出目录（直接在该目录下生成 image/depth/txt）"
    )
    parser.add_argument(
        "--skip",
        type=int,
        default=1,
        help="跳帧数：每 skip 帧保存 1 帧（>=1）"
    )
    parser.add_argument(
        "--rgb_topic",
        type=str,
        default="/camera/rgb/image_raw",
        help="RGB 图像话题（默认 /camera/rgb/image_raw）"
    )
    parser.add_argument(
        "--depth_topic",
        type=str,
        default="/camera/depth/image_raw",
        help="Depth 图像话题（默认 /camera/depth/image_raw）"
    )
    parser.add_argument(
        "--imu_topic",
        type=str,
        default="/jetbot_camera/imu",
        help="IMU 话题（默认 /jetbot_camera/imu）"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    assert args.skip >= 1, "skip 必须 >= 1"

    bridge = CvBridge()

    rgb_topic = args.rgb_topic
    depth_topic = args.depth_topic
    imu_topic = args.imu_topic

    # 输出路径
    rgb_imgs_save_path = os.path.join(args.output_path, 'image')
    depth_imgs_save_path = os.path.join(args.output_path, 'depth')
    timestamps_save_path = os.path.join(args.output_path, 'times.txt')
    imu_save_path = os.path.join(args.output_path, 'imu.txt')

    os.makedirs(rgb_imgs_save_path, exist_ok=True)
    os.makedirs(depth_imgs_save_path, exist_ok=True)

    print("Bag path   :", args.bag_path)
    print("Output path:", args.output_path)
    print("RGB topic  :", rgb_topic)
    print("Depth topic:", depth_topic)
    print("IMU topic  :", imu_topic)
    print("Frame skip :", args.skip)

    # 初始化 reader
    reader = SequentialReader()
    storage_options = StorageOptions(
        uri=args.bag_path,
        storage_id='sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # topic -> type
    topic_types = {
        t.name: t.type for t in reader.get_all_topics_and_types()
    }

    rgb_images = {}
    depth_images = {}
    imu_data = []
    timestamps_data = []

    depth_factor = 1000.0
    time_tolerance = 0.01

    print("Reading bag file...")
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic not in topic_types:
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        if not hasattr(msg, 'header'):
            continue

        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if topic == rgb_topic:
            rgb_images[ts] = msg
        elif topic == depth_topic:
            depth_images[ts] = msg
        elif topic == imu_topic:
            imu_data.append([
                ts,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

    print("Synchronizing and saving images...")
    saved_count = 0

    for idx, (rgb_time, rgb_msg) in enumerate(sorted(rgb_images.items())):
        if not depth_images:
            break

        # 跳帧
        if idx % args.skip != 0:
            continue

        closest_depth_time = min(
            depth_images.keys(),
            key=lambda t: abs(t - rgb_time)
        )

        if abs(rgb_time - closest_depth_time) > time_tolerance:
            continue

        depth_msg = depth_images[closest_depth_time]

        rgb_cv_image = bridge.imgmsg_to_cv2(
            rgb_msg, desired_encoding='passthrough'
        )
        depth_cv_image = bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        if depth_cv_image.dtype == np.float32:
            depth_cv_image = (depth_cv_image * depth_factor).astype(np.uint16)

        cv2.imwrite(
            os.path.join(rgb_imgs_save_path, f"{saved_count:06d}.png"),
            rgb_cv_image
        )
        cv2.imwrite(
            os.path.join(depth_imgs_save_path, f"{saved_count:06d}_depth.png"),
            depth_cv_image
        )

        timestamps_data.append([rgb_time])
        saved_count += 1

    with open(timestamps_save_path, 'w+') as f:
        write_txt(f, timestamps_data)

    if imu_data:
        with open(imu_save_path, 'w+') as f:
            write_txt(f, imu_data)

    print(f"Finished saving {saved_count} image pairs "
          f"and {len(imu_data)} IMU messages.")


if __name__ == "__main__":
    main()