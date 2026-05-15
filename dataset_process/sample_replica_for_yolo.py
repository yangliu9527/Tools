#!/usr/bin/env python3
import os
import argparse
import shutil
from pathlib import Path

def sample_images(dataset_root, seq_ids, frame_interval, output_dir):
    """
    从指定序列的results文件夹中按帧间隔采样frame开头的图片
    
    Args:
        dataset_root (str): 数据集根路径
        seq_ids (list): 序列名称列表，如["room0", "room1"]
        frame_interval (int): 帧间隔
        output_dir (str): 输出保存路径
    """
    # 创建输出目录
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # 处理指定的序列
    for seq_id in seq_ids:
        # 构建数据集路径
        dataset_dir = Path(dataset_root) / seq_id
        
        # 构建results文件夹路径
        results_dir = dataset_dir / "results"
        
        # 收集所有frame开头的jpg图片
        frame_files = []
        for file in results_dir.iterdir():
            if file.name.startswith("frame") and file.name.endswith(".jpg"):
                # 提取帧编号
                try:
                    frame_num = int(file.name[5:11])  # 提取frame后面的六位数字
                    frame_files.append((frame_num, file))
                except ValueError:
                    pass
        
        # 按帧编号排序
        frame_files.sort(key=lambda x: x[0])
        
        # 按帧间隔采样
        sampled_files = frame_files[::frame_interval]
        
        # 复制采样的图片到输出目录
        for frame_num, file in sampled_files:
            # 构建输出文件名，包含数据集名称
            output_filename = f"{seq_id}_{file.name}"
            output_file = output_path / output_filename
            shutil.copy2(file, output_file)
            print(f"已复制: {file.name} -> {output_filename}")
        
        print(f"{seq_id}: 共找到 {len(frame_files)} 张图片，采样 {len(sampled_files)} 张")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="从指定序列的results文件夹中采样frame开头的图片")
    parser.add_argument("--dataset_root", help="数据集根路径")
    parser.add_argument("--frame_interval", type=int, help="帧间隔")
    parser.add_argument("--output_dir", help="输出保存路径")
    
    args = parser.parse_args()
    
    # 在这里指定要处理的序列
    seq_ids = ["room0", "room1", "office0", "office1", "office2", "office3", "office4", "room2"]
    
    sample_images(args.dataset_root, seq_ids, args.frame_interval, args.output_dir)
