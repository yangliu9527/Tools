import rosbag
import sys
import os
import numpy as np
from sensor_msgs.msg import Imu, Image
import rospy
from geometry_msgs.msg import Vector3
import cv2 as cv
from cv_bridge import CvBridge
import argparse

#to write data(list format) into .txt file
def write_txt(f, data):
    num = len(data)
    for i in range(0,num):
        line = data[i]
        num2 = len(line)
        for j in range(0,num2):
            value = line[j]
            if(j!=num2-1):
                f.write(str(value)+" ")
            else:
                f.write(str(value))
        if(i!=num-1):
            f.write("\n")
    f.close()

parser = argparse.ArgumentParser()
parser.add_argument('--imu_file_path', type=str)
parser.add_argument('--bag_output_filepath', type=str)
parser.add_argument('--imu_output_txtpath', type=str,default=None)


# prepare files
args = parser.parse_args()
imu_file = open(args.imu_file_path)

# parameters
imu_freq = 100
imu_head = '#RAWIMUXA'

# output parameters
rosbag_output = rosbag.Bag(args.bag_output_filepath, 'w')
imu_topic_name = 'imu'

#################### Process IMU ##################
imu_data_txt = []
imu_ts_data = imu_file.readlines()
# data for carlib
for ts_imu_line in imu_ts_data:
    ts_imu_line = ts_imu_line.split(',')
    # check data flag
    if (not (imu_head in ts_imu_line[0])):
        continue
    # check valid flag
    valid_flag = int(ts_imu_line[3])
    if valid_flag == 0:
        continue

    # add imu point
    time_stamp = float(ts_imu_line[2])

    # create a new IMU point
    imu_point = []
    imu_point.append(time_stamp)

    up_deltav = float(ts_imu_line[4])
    up_deltav = up_deltav*400/(pow(2, 31))
    up_a = up_deltav*imu_freq
    imu_point.append(up_a)

    forward_deltav = float(ts_imu_line[5])
    forward_deltav = forward_deltav*400/(pow(2, 31))
    forward_a = forward_deltav*imu_freq
    imu_point.append(forward_a)

    right_deltav = float(ts_imu_line[6])
    right_deltav = right_deltav*400/(pow(2, 31))
    right_a = right_deltav*imu_freq
    imu_point.append(right_a)

    #from angle to rad
    k = np.pi/180.0
    up_deltaw = float(ts_imu_line[7])
    up_deltaw = up_deltaw * 2160/(pow(2, 31))
    up_w = k*up_deltaw*imu_freq
    imu_point.append(up_w)

    forward_deltaw = float(ts_imu_line[8])
    forward_deltaw = forward_deltaw * 2160/(pow(2, 31))
    forward_w = k*forward_deltaw*imu_freq
    imu_point.append(forward_w)

    right_deltaw = float(ts_imu_line[9])
    right_deltaw = right_deltaw * 2160/(pow(2, 31))
    right_w = k*right_deltaw*imu_freq
    imu_point.append(right_w)

    print(imu_point)
    imu_data_txt.append(imu_point)

    # create IMU ros msg
    imu_rosmsg = Imu()
    angular_v = Vector3()
    linear_a = Vector3()

    # read timestamp and transform to ROS timestamp
    ts = imu_point[0]
    #ts = ts+315964800+604800*2258-8*3600
    imu_ts_ros = rospy.rostime.Time.from_sec(ts)
    imu_rosmsg.header.stamp = imu_ts_ros

    # acc
    linear_a.x = imu_point[1]
    linear_a.y = imu_point[2]
    linear_a.z = imu_point[3]

    # gyro
    angular_v.x = imu_point[4]
    angular_v.y = imu_point[5]
    angular_v.z = imu_point[6]

    imu_rosmsg.angular_velocity = angular_v
    imu_rosmsg.linear_acceleration = linear_a

    rosbag_output.write(imu_topic_name, imu_rosmsg, imu_ts_ros)

rosbag_output.close()
print("imu rosbag generated.")

if(args.imu_output_txtpath):
    imu_output_txtfile = open(args.imu_output_txtpath,"w")
    write_txt(imu_output_txtfile,imu_data_txt)