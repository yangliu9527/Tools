import rospy
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

# 初始化 ROS 节点
rospy.init_node('rosbag_image_reader')

# 创建 CvBridge 对象，用于转换 ROS 图像消息到 OpenCV 格式
bridge = CvBridge()

# 打开 rosbag 文件
bag_file = 'your_rosbag_file.bag'  # 替换为你的 rosbag 文件路径
bag = rosbag.Bag(bag_file, 'r')

# 设置保存图像的文件夹路径
output_folder = 'output_images'
os.makedirs(output_folder, exist_ok=True)


save_path = "/home/zhiyu/DataSet/SelfCollected/LYR_AGV/12_27/files"
left_imgs_save_path = f"{save_path}/image_2"
right_imgs_save_path = f"{save_path}/image_3"
timestamps_save_path = f"{save_path}/times.txt"

# 设置左右图像的主题名（根据你的实际情况修改）
left_topic = '/camera_array/cam0/image_raw'   # 左目图像的主题
right_topic = '/camera_array/cam1/image_raw'  # 右目图像的主题

# 设置图像的帧数计数器
left_counter = 0
right_counter = 0
counter = 0

# timestamps
first_ts = -1.0



# 遍历 rosbag 中的消息
for topic, msg, t in bag.read_messages():
    # 处理左目图像
    if topic == left_topic:
        try:
            # 使用 CvBridge 转换为 OpenCV 图像
            left_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 保存图像
            left_filename = os.path.join(left_imgs_save_path, f"left_{left_counter:04d}.png")
            cv2.imwrite(left_filename, left_img)

            print(f"Saved left image: {left_filename}")
            left_counter += 1

            

        except Exception as e:
            print(f"Error converting left image: {e}")

    # 处理右目图像
    if topic == right_topic:
        try:
            # 使用 CvBridge 转换为 OpenCV 图像
            right_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 保存图像
            right_filename = os.path.join(right_imgs_save_path, f"right_{right_counter:04d}.png")
            cv2.imwrite(right_filename, right_img)

            print(f"Saved right image: {right_filename}")
            right_counter += 1

        except Exception as e:
            print(f"Error converting right image: {e}")
    


# 关闭 rosbag
bag.close()

print("Finished saving images.")
