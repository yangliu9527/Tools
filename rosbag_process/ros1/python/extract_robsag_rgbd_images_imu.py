#import rospy
import rosbag
import cv2
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge
import os
import numpy as np

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


#rospy.init_node('rosbag_image_reader')


bridge = CvBridge()


bag_file = '/home/zhiyu/DataSet/ISAAC_NAV_DATASET/collected_data/jetbot_10234403_1.bag'  
bag = rosbag.Bag(bag_file, 'r')


output_folder = 'output_images'
os.makedirs(output_folder, exist_ok=True)

save_path = "/home/zhiyu/DataSet/ISAAC_NAV_DATASET/collected_data/jetbot_10234403_1/"
rgb_imgs_save_path = f"{save_path}/image/"
depth_imgs_save_path = f"{save_path}/depth/"
timestamps_save_path = f"{save_path}/times.txt"
imu_save_path = f"{save_path}/imu.txt"


rgb_topic = '/jetbot_camera/rgb'
depth_topic = '/jetbot_camera/depth'
imu_topic = '/h1_camera/imu'


rgb_counter = 0
depth_counter = 0
imu_counter = 0

# timestamps
time_tolerance = 0.01

# dict
rgb_images = {}
depth_images = {}
timestamps_data = []
imu_data = []

depth_factor = 1000.0

#=======================================Extracting RosBag=====================================
print("Reading bag file...")
for topic, msg, t in bag.read_messages():
    print(f"reading {topic} at {t.to_sec()}")
    ts = msg.header.stamp.to_sec()
    if topic == rgb_topic:
        rgb_images[ts] = msg
    elif topic == depth_topic:
        depth_images[ts] = msg
    elif topic == imu_topic:
        angv_x = msg.angular_velocity.x
        angv_y = msg.angular_velocity.y
        angv_z = msg.angular_velocity.z
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        imu_point = [ts, acc_x,acc_y,acc_z,angv_x,angv_y,angv_z]
        imu_data.append(imu_point)
        imu_counter+=1

        
        

print("Synchronizing and saving images...")
synced_count = 0
for rgb_time, rgb_msg in rgb_images.items():
    closest_depth_time = min(depth_images.keys(), key=lambda t: abs(t - rgb_time))
    if abs(rgb_time - closest_depth_time) <= time_tolerance:
        depth_msg = depth_images[closest_depth_time]

        print(f"associated timestamp: rgb {rgb_time}, depth {closest_depth_time}")

        rgb_cv_image = bridge.imgmsg_to_cv2(rgb_msg)
        rgb_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2RGB) 
        depth_cv_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if depth_cv_image.dtype == np.float32:
            depth_cv_image = (depth_cv_image * depth_factor).astype(np.uint16)  


        rgb_image_path = os.path.join(rgb_imgs_save_path , f"{synced_count:06d}.png")
        depth_image_path = os.path.join(depth_imgs_save_path, f"{synced_count:06d}_depth.png")
        cv2.imwrite(rgb_image_path, rgb_cv_image)
        cv2.imwrite(depth_image_path, depth_cv_image)
        timestamps_data.append([rgb_time])
        synced_count += 1


bag.close()

timestamps_file = open(timestamps_save_path,"w+")
write_txt(timestamps_file, timestamps_data)

if(len(imu_data)!=0):
    imu_file=open(imu_save_path,"w+")
    write_txt(imu_file, imu_data)

timestamps_file.close()
print(f"Finished saving {synced_count} stereo images. {imu_counter} imu datas.")

#=======================================Extracted RosBag=====================================
    
    
