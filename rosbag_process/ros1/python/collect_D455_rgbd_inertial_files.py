#encoding=utf-8

import rosbag
import sys
import os
import numpy as np
from sensor_msgs.msg import Imu, Image
import rospy
from geometry_msgs.msg import Vector3
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import argparse

parser = argparse.ArgumentParser()
# parser.add_argument("-t","--topics",dest="topics",nargs='+',help="topics to collect")
parser.add_argument('--output_path', type=str,default="./")

args = parser.parse_args()
output_path = args.output_path

if (not os.path.exists(output_path)):
        os.mkdir(output_path)

rgb_path = output_path+"/rgb/"
depth_path = output_path+"/depth/"

if (not os.path.exists(rgb_path)):
        os.mkdir(rgb_path)

if (not os.path.exists(depth_path)):
        os.mkdir(depth_path)

imu_file_path = output_path+"/imu.txt"
rgb_ts_file_path = output_path+"/rgb.txt"
depth_ts_file_path = output_path+"/depth.txt"

imu_file = open(imu_file_path,"w+")
rgb_ts_file = open(rgb_ts_file_path,"w+")
depth_ts_file = open(depth_ts_file_path,"w+")

def rgb_callback(imgmsg):
    try:
        bridge = CvBridge()
        im_bgr = bridge.imgmsg_to_cv2(imgmsg, "passthrough")
        im_rgb = cv.cvtColor(im_bgr, cv.COLOR_BGR2RGB)
        ts = imgmsg.header.stamp
        seconds = ts.secs
        nanoseconds = ts.nsecs
        sec_nsec = str(seconds)+"."+str(nanoseconds)
        print(sec_nsec)
        # cv.imshow("rgb", im_rgb)
        # cv.waitKey(10)
        cv.imwrite(rgb_path+sec_nsec+".png", im_rgb)
        line_to_write = sec_nsec+" "+"rgb/"+sec_nsec+".png\n"
        rgb_ts_file.writelines(line_to_write)
    except CvBridgeError as e:
        print(e)

def depth_callback(imgmsg):
    try:
        bridge = CvBridge()
        im_depth = bridge.imgmsg_to_cv2(imgmsg, "16UC1")
        ts = imgmsg.header.stamp
        seconds = ts.secs
        nanoseconds = ts.nsecs
        sec_nsec = str(seconds)+"."+str(nanoseconds)
        print(sec_nsec)
        # cv.imshow("depth", im_depth)
        # cv.waitKey(10)
        cv.imwrite(depth_path+sec_nsec+".png", im_depth)
        line_to_write = sec_nsec+" "+"depth/"+sec_nsec+".png\n"
        depth_ts_file.writelines(line_to_write)
    except CvBridgeError as e:
        print(e)


def d455_collector():
    rospy.init_node("collect d455 node")
    rospy.loginfo("collect d455 rgbd and inertial data")
    rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    d455_collector()

