#!/usr/bin/env python3
"""
视频四宫格拼接工具
将四个视频按顺序拼成2x2网格
"""

import cv2
import numpy as np
import argparse
import sys
import os
from pathlib import Path

def resize_video_frame(frame, target_width, target_height):
    """
    将视频帧调整到指定大小
    
    Args:
        frame: 输入帧
        target_width: 目标宽度
        target_height: 目标高度
    
    Returns:
        调整大小后的帧
    """
    return cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)

def create_grid_frame(frames, grid_cols=2, grid_rows=2):
    """
    将四个帧拼成网格
    
    Args:
        frames: 包含四个帧的列表，顺序为 [左上, 右上, 左下, 右下]
        grid_cols: 网格列数（固定为2）
        grid_rows: 网格行数（固定为2）
    
    Returns:
        拼接后的网格帧
    """
    if len(frames) != 4:
        raise ValueError(f"需要4个帧，但只提供了{len(frames)}个")
    
    # 获取每个帧的尺寸（假设所有帧尺寸相同）
    h, w = frames[0].shape[:2]
    
    # 创建第一行：左上 + 右上
    top_row = np.hstack((frames[0], frames[1]))
    
    # 创建第二行：左下 + 右下
    bottom_row = np.hstack((frames[2], frames[3]))
    
    # 垂直拼接两行
    grid = np.vstack((top_row, bottom_row))
    
    return grid

def get_video_properties(cap):
    """
    获取视频的基本属性
    
    Args:
        cap: VideoCapture对象
    
    Returns:
        (fps, total_frames) 元组
    """
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    return fps, total_frames

def merge_four_videos(video_paths, output_path, target_width, target_height):
    """
    将四个视频拼成四宫格
    
    Args:
        video_paths: 四个视频文件路径的列表
        output_path: 输出视频路径
        target_width: 每个小画面的目标宽度
        target_height: 每个小画面的目标高度
    """
    # 检查是否提供了4个视频
    if len(video_paths) != 4:
        print(f"错误：需要4个视频文件，但只提供了{len(video_paths)}个")
        return False
    
    # 检查所有视频文件是否存在
    for i, path in enumerate(video_paths):
        if not os.path.exists(path):
            print(f"错误：视频文件不存在 - {path}")
            return False
    
    # 打开所有视频
    caps = []
    for path in video_paths:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print(f"错误：无法打开视频文件 - {path}")
            # 关闭已打开的视频
            for c in caps:
                c.release()
            return False
        caps.append(cap)
    
    # 获取视频属性（使用第一个视频的fps作为输出fps）
    fps_info = [get_video_properties(cap) for cap in caps]
    fps = fps_info[0][0]  # 使用第一个视频的fps
    min_frames = min([frames for _, frames in fps_info])  # 使用最短的视频长度
    
    print(f"视频信息：")
    for i, (path, (f, frames)) in enumerate(zip(video_paths, fps_info)):
        print(f"  视频{i+1}: {os.path.basename(path)} - {frames}帧, {f:.2f}fps")
    print(f"输出fps: {fps:.2f}")
    print(f"处理帧数: {min_frames}")
    print(f"每个小画面尺寸: {target_width}x{target_height}")
    print(f"输出视频尺寸: {target_width*2}x{target_height*2}")
    
    # 初始化视频写入器
    output_size = (target_width * 2, target_height * 2)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用MP4编码
    out = cv2.VideoWriter(output_path, fourcc, fps, output_size)
    
    if not out.isOpened():
        print(f"错误：无法创建输出视频文件 - {output_path}")
        for cap in caps:
            cap.release()
        return False
    
    print("开始处理视频...")
    frame_count = 0
    
    try:
        # 逐帧处理
        while frame_count < min_frames:
            frames = []
            
            # 从每个视频读取一帧
            for i, cap in enumerate(caps):
                ret, frame = cap.read()
                
                if not ret:
                    # 如果某个视频提前结束，使用最后一帧或黑帧
                    print(f"警告：视频{i+1}在第{frame_count}帧结束，使用黑帧填充")
                    black_frame = np.zeros((target_height, target_width, 3), dtype=np.uint8)
                    frames.append(black_frame)
                else:
                    # 调整帧大小
                    resized_frame = resize_video_frame(frame, target_width, target_height)
                    frames.append(resized_frame)
            
            # 创建四宫格
            grid_frame = create_grid_frame(frames)
            
            # 写入输出视频
            out.write(grid_frame)
            
            frame_count += 1
            
            # 显示进度
            if frame_count % 30 == 0 or frame_count == min_frames:
                progress = (frame_count / min_frames) * 100
                print(f"  进度: {progress:.1f}% ({frame_count}/{min_frames})", end='\r')
        
        print(f"\n处理完成！共处理 {frame_count} 帧")
        
    except KeyboardInterrupt:
        print("\n用户中断处理")
    except Exception as e:
        print(f"\n处理过程中出现错误: {e}")
        return False
    finally:
        # 释放所有资源
        for cap in caps:
            cap.release()
        out.release()
    
    print(f"视频已保存到: {output_path}")
    return True

def main():
    parser = argparse.ArgumentParser(
        description='将四个视频拼接成四宫格（2x2网格）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python video_grid.py -v video1.mp4 video2.mp4 video3.mp4 video4.mp4 -o output.mp4 -w 640 -h 360
  
  python video_grid.py -v vid1.mp4 vid2.mp4 vid3.mp4 vid4.mp4 -o result.mp4 --width 960 --height 540
        """
    )
    
    parser.add_argument('-v', '--videos', 
                        nargs=4, 
                        required=True,
                        metavar=('VIDEO1', 'VIDEO2', 'VIDEO3', 'VIDEO4'),
                        help='四个视频文件路径（按左上、右上、左下、右下顺序）')
    
    parser.add_argument('-o', '--output', 
                        required=True,
                        help='输出视频文件路径（例如：output.mp4）')
    
    parser.add_argument('-w', '--width', 
                        type=int, 
                        required=True,
                        help='每个小画面的目标宽度（像素）')
    
    parser.add_argument('-ht', '--height', 
                        type=int, 
                        required=True,
                        help='每个小画面的目标高度（像素）')
    
    args = parser.parse_args()
    
    # 验证宽度和高度
    if args.width <= 0 or args.height <= 0:
        print("错误：宽度和高度必须为正数")
        sys.exit(1)
    
    # 检查输出文件扩展名
    output_ext = os.path.splitext(args.output)[1].lower()
    if output_ext not in ['.mp4', '.avi', '.mov', '.mkv']:
        print(f"警告：输出文件扩展名 '{output_ext}' 可能不被支持，建议使用 .mp4")
    
    # 执行视频拼接
    success = merge_four_videos(args.videos, args.output, args.width, args.height)
    
    if success:
        print("\n✓ 四宫格视频创建成功！")
        sys.exit(0)
    else:
        print("\n✗ 视频创建失败！")
        sys.exit(1)

if __name__ == "__main__":
    main()
