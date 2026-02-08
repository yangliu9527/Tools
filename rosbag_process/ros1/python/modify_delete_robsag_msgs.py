import rospy
import rosbag
import cv2
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge
import os
import numpy as np


# 打开 rosbag 文件
input_bag_path = '/home/zhiyu/DataSet/SelfCollected/LYR_AGV/1_12/bag/data1_checked.bag'  # 替换为你的 rosbag 文件路径
input_bag = rosbag.Bag(input_bag_path, 'r')

# 设置保存图像的文件夹路径
output_bag_path = '/home/zhiyu/DataSet/SelfCollected/LYR_AGV/1_12/bag/data1_checked'
output_bag= rosbag.Bag(output_bag_path , 'w')

topics_ts_to_modify = {}
topics_ts_to_modify["/camera_array/cam0/image_raw"] = {1736667435.800276041:1736667435.900278091, 1736667435.900278091:1736667435.800276041, 1736667543.200393915:1736667543.300395966,1736667543.300395966:1736667543.200393915}
topics_ts_to_modify["/camera_array/cam1/image_raw"] = {1741156010.999747038:1736667435.900278091, 1736667435.900278091:1736667435.800276041, 1736667543.200393915:1736667543.300395966,1736667543.300395966:1736667543.200393915}

print("Reading bag file...")
for topic, msg, t in input_bag.read_messages():
    #print(f"reading {topic} at {t.to_sec()}")
    ts = msg.header.stamp.to_sec()
    if(topic in topics_ts_to_modify):
        if(ts in topics_ts_to_modify[topic]):
            new_msg = msg
            new_msg.header.stamp = rospy.rostime.Time.from_sec(topics_ts_to_modify[topic][ts])
            print(f"modify one msg stamp {ts} to {topics_ts_to_modify[topic][ts]}")
        else:
            output_bag.write(topic, msg, t)
    else:
        output_bag.write(topic, msg, t)

output_bag.close()

    
    
    
