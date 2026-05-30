#!/usr/bin/env python3
"""
Extract RGB images from a rosbag2 at a fixed frame or time interval.

This is a RGB-only variant intended for YOLO dataset preparation.  It streams
the bag and does not require depth/IMU topics.
"""

import argparse
import os
from typing import List, Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract RGB images from rosbag2 by frame skip or time interval."
    )
    parser.add_argument(
        "--bag_path",
        type=str,
        required=True,
        help="rosbag2 文件夹路径，即包含 metadata.yaml 的目录，不是单独 .db3 文件",
    )
    parser.add_argument(
        "--output_path",
        type=str,
        required=True,
        help="图片输出目录，例如 YOLO 数据集的 images/train",
    )
    parser.add_argument(
        "--rgb_topic",
        type=str,
        default="/camera/rgb/image_raw",
        help="RGB 图像话题",
    )

    interval_group = parser.add_mutually_exclusive_group()
    interval_group.add_argument(
        "--skip",
        type=int,
        default=1,
        help="每 skip 帧保存 1 帧；例如 --skip 10 表示每 10 帧取 1 帧",
    )
    interval_group.add_argument(
        "--interval_sec",
        type=float,
        default=None,
        help="按时间间隔保存；例如 --interval_sec 0.5 表示约每 0.5 秒取 1 帧",
    )

    parser.add_argument(
        "--format",
        choices=("jpg", "png"),
        default="jpg",
        help="保存格式。YOLO 训练通常用 jpg 更省空间",
    )
    parser.add_argument(
        "--jpeg_quality",
        type=int,
        default=95,
        help="JPG 质量 0-100",
    )
    parser.add_argument(
        "--png_compression",
        type=int,
        default=3,
        help="PNG 压缩等级 0-9",
    )
    parser.add_argument(
        "--encoding",
        type=str,
        default="bgr8",
        help="cv_bridge 输出编码。用 cv2.imwrite 保存彩色图时建议保持默认 bgr8",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="rgb",
        help="输出图片文件名前缀",
    )
    parser.add_argument(
        "--start_index",
        type=int,
        default=0,
        help="输出图片编号起始值",
    )
    parser.add_argument(
        "--storage_id",
        type=str,
        default="sqlite3",
        help="rosbag2 storage id，常见为 sqlite3 或 mcap",
    )
    parser.add_argument(
        "--timestamps_file",
        type=str,
        default=None,
        help=(
            "时间戳文件路径；默认写到 output_path/timestamps.txt。"
            "如果传相对路径，则相对于 output_path"
        ),
    )
    return parser.parse_args()


def get_msg_stamp_sec(msg, bag_time_ns: int) -> float:
    """Prefer message header stamp; fall back to rosbag storage timestamp."""
    if hasattr(msg, "header"):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            return stamp
    return bag_time_ns * 1e-9


def to_cv_image(msg, topic_type: str, bridge: CvBridge, encoding: str):
    """Convert sensor_msgs/Image or sensor_msgs/CompressedImage to cv image."""
    if topic_type == "sensor_msgs/msg/CompressedImage":
        try:
            return bridge.compressed_imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception:
            # Fallback for systems where cv_bridge compressed conversion is absent.
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # sensor_msgs/msg/Image, or a compatible type.
    return bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)


def imwrite_params(fmt: str, jpeg_quality: int, png_compression: int) -> List[int]:
    if fmt == "jpg":
        return [cv2.IMWRITE_JPEG_QUALITY, int(np.clip(jpeg_quality, 0, 100))]
    return [cv2.IMWRITE_PNG_COMPRESSION, int(np.clip(png_compression, 0, 9))]


def should_save_by_interval(
    stamp: float,
    rgb_index: int,
    skip: int,
    interval_sec: Optional[float],
    last_saved_stamp: Optional[float],
) -> bool:
    if interval_sec is None:
        return rgb_index % skip == 0

    if last_saved_stamp is None:
        return True
    return (stamp - last_saved_stamp) >= interval_sec


def main():
    args = parse_args()
    if args.skip < 1:
        raise ValueError("--skip 必须 >= 1")
    if args.interval_sec is not None and args.interval_sec <= 0:
        raise ValueError("--interval_sec 必须 > 0")

    os.makedirs(args.output_path, exist_ok=True)
    if args.timestamps_file is None:
        timestamps_path = os.path.join(args.output_path, "timestamps.txt")
    elif os.path.isabs(args.timestamps_file):
        timestamps_path = args.timestamps_file
    else:
        timestamps_path = os.path.join(args.output_path, args.timestamps_file)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag_path, storage_id=args.storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.rgb_topic not in topic_types:
        available = "\n".join(f"  {name}: {typ}" for name, typ in sorted(topic_types.items()))
        raise RuntimeError(f"找不到 RGB 话题: {args.rgb_topic}\n可用话题:\n{available}")

    rgb_type = topic_types[args.rgb_topic]
    msg_type = get_message(rgb_type)
    bridge = CvBridge()

    print("Bag path   :", args.bag_path)
    print("Output path:", args.output_path)
    print("RGB topic  :", args.rgb_topic)
    print("RGB type   :", rgb_type)
    if args.interval_sec is None:
        print("Interval   : every", args.skip, "RGB frames")
    else:
        print("Interval   :", args.interval_sec, "sec")

    rgb_index = 0
    saved_count = 0
    last_saved_stamp: Optional[float] = None
    save_params = imwrite_params(args.format, args.jpeg_quality, args.png_compression)
    timestamps: List[Tuple[str, float]] = []

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        if topic != args.rgb_topic:
            continue

        msg = deserialize_message(data, msg_type)
        stamp = get_msg_stamp_sec(msg, bag_time_ns)

        if not should_save_by_interval(
            stamp, rgb_index, args.skip, args.interval_sec, last_saved_stamp
        ):
            rgb_index += 1
            continue

        cv_image = to_cv_image(msg, rgb_type, bridge, args.encoding)
        filename = f"{args.prefix}_{args.start_index + saved_count:06d}.{args.format}"
        output_file = os.path.join(args.output_path, filename)
        if not cv2.imwrite(output_file, cv_image, save_params):
            raise RuntimeError(f"保存图片失败: {output_file}")

        timestamps.append((filename, stamp))
        saved_count += 1
        last_saved_stamp = stamp
        rgb_index += 1

    with open(timestamps_path, "w", encoding="utf-8") as f:
        for filename, stamp in timestamps:
            f.write(f"{filename} {stamp:.9f}\n")

    print(f"Finished. Read {rgb_index} RGB frames, saved {saved_count} images.")
    print("Timestamps:", timestamps_path)


if __name__ == "__main__":
    main()
