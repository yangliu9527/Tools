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
parser.add_argument('--img_files_path', type=str,)
parser.add_argument('--img_timestamps_path', type=str)
parser.add_argument('--imu_file_path', type=str)
parser.add_argument('--bag_output_filepath', type=str)
parser.add_argument('--imu_output_txtpath', type=str,default=None)


# prepare files
args = parser.parse_args()
img_files_names = os.listdir(args.img_files_path)
img_times_file = open(args.img_timestamps_path, 'r')
imu_file = open(args.imu_file_path)

# parameters
sample_freq = 4 #HZ
imu_freq = 100 #HZ
img_freq = 20 #HZ
img_head = '#CAMERA'
imu_head = '#RAWIMUXA'

# output parameters
rosbag_output = rosbag.Bag(args.bag_output_filepath, 'w')
imu_topic_name = 'imu'
img_topic_name = 'camera/gray'


#################### Process Images ##################
# establish the correspondences between id and file name
img_num = len(img_files_names)
id_fname = {}
for img_name in img_files_names:
    originl_name = img_name
    img_name = img_name.split("-")
    # get img id corresponding to the line index of the timestamps file
    img_id = int((img_name[-1].split('.'))[0])
    print('img_id='+str(img_id)+", name = "+originl_name)
    id_fname[img_id] = originl_name

max_img_ts = 0.0
min_img_ts = sys.float_info.max
# write img msg into rosbag
sample_period = 1.0/sample_freq
time_cnt = sample_period
last_ts = 0.0
img_times = img_times_file.readlines()
for id, ts_line in enumerate(img_times):
    ts_line = ts_line.split(",")
    if (not id_fname.has_key(id)):
        continue
    if (not (img_head in ts_line[0])):
        continue

    

    # read timestamp and transform to ROS timestamp
    ts = float(ts_line[2])
    time_cnt = time_cnt+(ts-last_ts)
    last_ts = ts
    if(time_cnt<sample_period):
        continue
    else:
        time_cnt = 0.0

    if (ts > max_img_ts):
        max_img_ts = ts
    if (ts < min_img_ts):
        min_img_ts = ts
    #ts = ts+315964800+604800*2258-8*3600
    img_ts_ros = rospy.rostime.Time.from_sec(ts)

    # read img and transform to ROS msg
    img_name = id_fname[id]
    img_path = args.img_files_path+"/"+img_name
    print(img_path)
    img_cv = cv.imread(img_path, -1)
    cv.imshow('im', img_cv)
    cv.waitKey(10)
    br = CvBridge()
    img_rosmsg = br.cv2_to_imgmsg(img_cv)

    # write img info
    img_rosmsg.header.stamp = img_ts_ros
    img_rosmsg.header.frame_id = "camera"
    img_rosmsg.encoding = "mono8"

    # write into rosbag
    rosbag_output.write(img_topic_name, img_rosmsg, img_ts_ros)


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
    if (time_stamp < min_img_ts-5.0):
        continue
    if (time_stamp > max_img_ts+5.0):
        continue

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
print("cam-imu rosbag generated.")

if(args.imu_output_txtpath):
    imu_output_txtfile = open(args.imu_output_txtpath,"w")
    write_txt(imu_output_txtfile,imu_data_txt)