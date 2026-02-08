#!/usr/bin/env python3
"""
ROS2 Bag 时间戳校正脚本（修正版）
严格保持话题-消息对应关系，只修改时间戳
"""

import os
import argparse
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import (
    StorageOptions,
    ConverterOptions,
    SequentialReader,
    SequentialWriter
)
import shutil
from rosidl_runtime_py.utilities import get_message
import sys

class BagTimestampFixer:
    def __init__(self):
        self.reader = None
        self.writer = None
        self.processed_count = 0
        self.skipped_count = 0
        self.msg_type_cache = {}
        
    def get_message_class(self, msg_type):
        """动态获取消息类"""
        if msg_type in self.msg_type_cache:
            return self.msg_type_cache[msg_type]
        
        try:
            msg_class = get_message(msg_type)
            self.msg_type_cache[msg_type] = msg_class
            return msg_class
        except Exception as e:
            print(f"警告: 无法加载消息类型 {msg_type}: {e}")
            return None
    
    def has_header_stamp(self, msg):
        """检查消息是否有header.stamp"""
        try:
            # 标准消息类型
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                return True
            # TF消息特殊处理
            if hasattr(msg, 'transforms') and len(msg.transforms) > 0:
                if hasattr(msg.transforms[0], 'header'):
                    return True
            return False
        except Exception:
            return False
    
    def get_header_stamp(self, msg):
        """提取header.stamp时间戳"""
        try:
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                return msg.header.stamp
            if hasattr(msg, 'transforms') and len(msg.transforms) > 0:
                return msg.transforms[0].header.stamp
        except Exception as e:
            print(f"提取header.stamp时出错: {e}")
        return None
    
    def create_new_storage_path(self, original_path):
        """创建新的存储路径"""
        if original_path.endswith('/'):
            original_path = original_path[:-1]
        return f"{original_path}_timestamp_fixed"
    
    def fix_bag_timestamps(self, input_bag_path, output_bag_path=None):
        """主修复函数"""
        if output_bag_path is None:
            output_bag_path = self.create_new_storage_path(input_bag_path)
        
        if os.path.exists(output_bag_path):
            response = input(f"输出路径 {output_bag_path} 已存在。是否覆盖? (y/N): ")
            if response.lower() != 'y':
                print("操作已取消")
                return False
            shutil.rmtree(output_bag_path)
        
        print(f"开始处理bag文件...")
        print(f"输入路径: {input_bag_path}")
        print(f"输出路径: {output_bag_path}")
        
        # 初始化ROS2
        rclpy.init()
        
        try:
            # 初始化读取器
            storage_options = StorageOptions(uri=input_bag_path, storage_id='sqlite3')
            converter_options = ConverterOptions('', '')
            
            self.reader = SequentialReader()
            self.reader.open(storage_options, converter_options)
            
            # 获取所有话题信息
            all_topics = self.reader.get_all_topics_and_types()
            print("发现的话题列表:")
            for topic_meta in all_topics:
                print(f"  - {topic_meta.name} (类型: {topic_meta.type})")
            
            # 初始化写入器
            writer_storage_options = StorageOptions(uri=output_bag_path, storage_id='sqlite3')
            self.writer = SequentialWriter()
            self.writer.open(writer_storage_options, converter_options)
            
            # 注册所有话题到写入器 - 保持原话题名称
            topic_type_map = {}
            for topic_metadata in all_topics:
                try:
                    self.writer.create_topic(topic_metadata)
                    topic_type_map[topic_metadata.name] = topic_metadata.type
                    print(f"已注册话题: {topic_metadata.name}")
                except Exception as e:
                    print(f"注册话题 {topic_metadata.name} 时出错: {e}")
                    return False
            
            self.processed_count = 0
            self.skipped_count = 0
            topic_stats = {}
            
            print("\n开始处理消息...")
            
            # 逐条处理消息
            while self.reader.has_next():
                try:
                    # 读取下一条消息
                    topic, data, recorded_timestamp = self.reader.read_next()
                    
                    # 更新统计
                    if topic not in topic_stats:
                        topic_stats[topic] = 0
                    
                    # 获取该话题的消息类型
                    if topic not in topic_type_map:
                        print(f"警告: 话题 {topic} 未注册，跳过")
                        self.skipped_count += 1
                        continue
                    
                    msg_type = topic_type_map[topic]
                    msg_class = self.get_message_class(msg_type)
                    
                    if msg_class is None:
                        # 无法识别的消息类型，使用原始时间戳
                        self.writer.write(topic, data, recorded_timestamp)
                        self.skipped_count += 1
                        topic_stats[topic] += 1
                        continue
                    
                    try:
                        # 反序列化消息
                        msg = deserialize_message(data, msg_class)
                        
                        new_timestamp = recorded_timestamp  # 默认使用原始时间戳
                        
                        if self.has_header_stamp(msg):
                            header_stamp = self.get_header_stamp(msg)
                            if header_stamp is not None:
                                # 使用header.stamp作为新时间戳
                                new_timestamp = int(header_stamp.sec * 1e9 + header_stamp.nanosec)
                                self.processed_count += 1
                            else:
                                self.skipped_count += 1
                        else:
                            self.skipped_count += 1
                        
                        # 关键修正：消息数据保持不变，写入原话题名称，只改时间戳
                        self.writer.write(topic, data, new_timestamp)
                        topic_stats[topic] += 1
                        
                    except Exception as e:
                        print(f"处理消息失败 (话题: {topic}): {e}")
                        # 出错时使用原始时间戳
                        self.writer.write(topic, data, recorded_timestamp)
                        self.skipped_count += 1
                        topic_stats[topic] += 1
                    
                    # 进度显示
                    total_messages = self.processed_count + self.skipped_count
                    if total_messages % 100 == 0:
                        print(f"已处理 {total_messages} 条消息...")
                        
                except Exception as e:
                    print(f"读取消息时出错: {e}")
                    continue
            
            # 输出详细统计
            print(f"\n处理完成! 详细统计:")
            print(f"成功校正时间戳: {self.processed_count} 条消息")
            print(f"跳过处理: {self.skipped_count} 条消息")
            print("各话题消息数量:")
            for topic, count in topic_stats.items():
                status = "✓ 已校正" if count > 0 else "⚠ 无消息"
                print(f"  - {topic}: {count} 条消息 ({status})")
            
            print(f"\n输出已保存至: {output_bag_path}")
            return True
            
        except Exception as e:
            print(f"处理bag文件时发生错误: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            # 正确清理资源
            if hasattr(self, 'reader') and self.reader is not None:
                del self.reader
                self.reader = None
            if hasattr(self, 'writer') and self.writer is not None:
                del self.writer
                self.writer = None
            rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(
        description='ROS2 Bag时间戳校正：将录制时间戳替换为header.stamp'
    )
    parser.add_argument('input_bag', help='输入的ROS2 bag文件路径（目录）')
    parser.add_argument('-o', '--output', help='输出的bag文件路径（目录）')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input_bag):
        print(f"错误: 输入路径 {args.input_bag} 不存在")
        return 1
    
    # 检查是否是ROS2 bag目录
    if not os.path.isdir(args.input_bag):
        print(f"错误: {args.input_bag} 不是目录")
        return 1
    
    fixer = BagTimestampFixer()
    
    try:
        success = fixer.fix_bag_timestamps(args.input_bag, args.output)
        
        if success:
            print("\n✅ 时间戳校正完成!")
            print("✅ 所有消息都保持在原话题下")
            print("✅ 只修改了录制时间戳，消息内容完全不变")
            return 0
        else:
            print("\n❌ 处理失败")
            return 1
            
    except KeyboardInterrupt:
        print("\n操作被用户中断")
        return 1
    except Exception as e:
        print(f"程序执行错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    main()