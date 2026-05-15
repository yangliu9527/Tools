#!/usr/bin/env python3
import os
import re
import argparse
from pathlib import Path

def extract_ros_timestamp_str(filename: str):
    """
    从文件名中提取 ROS 时间戳字符串（格式：seconds.nanoseconds）
    例如：'1712345678.123456789.png' -> '1712345678.123456789'
    如果不匹配，返回 None
    """
    stem = Path(filename).stem
    # 严格匹配：整数.整数（至少1位数字）
    if re.fullmatch(r'\d+\.\d+', stem):
        seconds, nanoseconds = stem.split('.')
        # 纳秒部分标准化为9位（补零或截断）
        nanoseconds = nanoseconds.ljust(9, '0')[:9]
        return f"{seconds}.{nanoseconds}"
    return None

def main():
    parser = argparse.ArgumentParser(
        description="从文件夹中提取文件名中的 ROS 时间戳（仅时间戳），排序后写入文本文件。"
    )
    parser.add_argument(
        '--input_dir',
        type=str,
        required=True,
        help='包含带时间戳文件的输入目录（例如：/path/to/images）'
    )
    parser.add_argument(
        '--output_file',
        type=str,
        required=True,
        help='输出的时间戳列表文件（例如：timestamps.txt）'
    )
    
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    if not input_dir.is_dir():
        raise FileNotFoundError(f"输入目录不存在: {input_dir}")

    timestamp_strs = []

    for file_path in input_dir.iterdir():
        if file_path.is_file():
            ts_str = extract_ros_timestamp_str(file_path.name)
            if ts_str is not None:
                timestamp_strs.append(ts_str)

    # 转为 float 排序（确保数值顺序正确）
    try:
        sorted_timestamps = sorted(timestamp_strs, key=float)
    except ValueError as e:
        print(f"警告：发现无效时间戳，跳过排序错误: {e}")
        sorted_timestamps = sorted(timestamp_strs)  # 回退到字典序（不推荐）

    # 写入输出文件（每行一个时间戳）
    with open(args.output_file, 'w') as f:
        for ts in sorted_timestamps:
            f.write(ts + '\n')

    print(f"成功提取并排序 {len(sorted_timestamps)} 个时间戳")
    print(f"结果已保存至: {args.output_file}")

if __name__ == '__main__':
    main()