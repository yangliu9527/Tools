import os
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, Imu

def write_txt(f, data):
    for i, line in enumerate(data):
        f.write(" ".join(map(str, line)))
        if i != len(data) - 1:
            f.write("\n")
    f.close()

bridge = CvBridge()

# 输入路径
bag_path = '/home/brain/LYCodes/habitat-data-collector/outputs/hm3d/00829-QaLdnwvtxbs_1/'
seq_name = "rosbag2"
output_path = os.path.join('/home/brain/DataSet/sam3_to_train_yolo/hm3d/00829-QaLdnwvtxbs',seq_name)
# topics
rgb_topic = '/camera/rgb/image_raw'
depth_topic = '/camera/depth/image_raw'
imu_topic = '/jetbot_camera/imu'



# bag_path = '/home/brain/DataSet/SelfCollected/turtlebot2/rosbag2/'
# seq_name = "association_test1"
# output_path = os.path.join('/home/brain/DataSet/SelfCollected/turtlebot2/files/',seq_name)
# rgb_topic = '/realsense/rgb/decompressed'
# depth_topic = '/realsense/depth/decompressed'
# imu_topic = '/jetbot_camera/imu'

bag_file = os.path.join(bag_path, seq_name)  # 不带 .db3 后缀
rgb_imgs_save_path = os.path.join(output_path, 'image')
depth_imgs_save_path = os.path.join(output_path, 'depth')
timestamps_save_path = os.path.join(output_path, 'times.txt')
imu_save_path = os.path.join(output_path, 'imu.txt')

print(rgb_imgs_save_path)
os.makedirs(rgb_imgs_save_path, exist_ok=True)
os.makedirs(depth_imgs_save_path, exist_ok=True)



# 初始化 reader
reader = SequentialReader()
storage_options = StorageOptions(uri=bag_file, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage_options, converter_options)

# 记录数据
rgb_images = {}
depth_images = {}
imu_data = []
timestamps_data = []

depth_factor = 1000.0
time_tolerance = 0.01

# topic类型映射
topic_types = {}
for topic_info in reader.get_all_topics_and_types():
    topic_types[topic_info.name] = topic_info.type

print("Reading bag file...")
while reader.has_next():
    (topic, data, t) = reader.read_next()
    msg_type = get_message(topic_types[topic])
    msg = deserialize_message(data, msg_type)
    if hasattr(msg, 'header'):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    else:
        continue

    if topic == rgb_topic:
        rgb_images[ts] = msg
    elif topic == depth_topic:
        depth_images[ts] = msg
    elif topic == imu_topic:
        imu_point = [
            ts,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]
        imu_data.append(imu_point)

print("Synchronizing and saving images...")
synced_count = 0
for rgb_time, rgb_msg in rgb_images.items():
    if len(depth_images) == 0:
        break
    closest_depth_time = min(depth_images.keys(), key=lambda t: abs(t - rgb_time))
    if abs(rgb_time - closest_depth_time) <= time_tolerance:
        depth_msg = depth_images[closest_depth_time]

        # 转换图像
        rgb_cv_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        # rgb_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_BGR2RGB)

        depth_cv_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        if depth_cv_image.dtype == np.float32:
            depth_cv_image = (depth_cv_image * depth_factor).astype(np.uint16)

        # 保存
        rgb_image_path = os.path.join(rgb_imgs_save_path, f"{synced_count:06d}.png")
        depth_image_path = os.path.join(depth_imgs_save_path, f"{synced_count:06d}_depth.png")
        cv2.imwrite(rgb_image_path, rgb_cv_image)
        cv2.imwrite(depth_image_path, depth_cv_image)
        timestamps_data.append([rgb_time])
        synced_count += 1

# 保存时间戳与IMU
with open(timestamps_save_path, 'w+') as f:
    write_txt(f, timestamps_data)

if imu_data:
    with open(imu_save_path, 'w+') as f:
        write_txt(f, imu_data)

print(f"Finished saving {synced_count} image pairs and {len(imu_data)} IMU messages.")

