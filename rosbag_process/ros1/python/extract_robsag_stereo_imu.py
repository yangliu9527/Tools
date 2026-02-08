import rospy
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

def read_yaml_file(file_path):
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Cannot open the YAML file: {file_path}")

    params = {}
    
    # 读取摄像头参数
    params["Camera"] = {
        "type": fs.getNode("Camera.type").string(),
        "width": int(fs.getNode("Camera.width").real()),
        "height": int(fs.getNode("Camera.height").real()),
        "fps": int(fs.getNode("Camera.fps").real()),
        "RGB": int(fs.getNode("Camera.RGB").real())
    }

    # 读取左目相机参数
    params["Camera1"] = {
        "fx": fs.getNode("Camera1.fx").real(),
        "fy": fs.getNode("Camera1.fy").real(),
        "cx": fs.getNode("Camera1.cx").real(),
        "cy": fs.getNode("Camera1.cy").real(),
        "k1": fs.getNode("Camera1.k1").real(),
        "k2": fs.getNode("Camera1.k2").real(),
        "p1": fs.getNode("Camera1.p1").real(),
        "p2": fs.getNode("Camera1.p2").real(),
    }

    # 读取右目相机参数
    params["Camera2"] = {
        "fx": fs.getNode("Camera2.fx").real(),
        "fy": fs.getNode("Camera2.fy").real(),
        "cx": fs.getNode("Camera2.cx").real(),
        "cy": fs.getNode("Camera2.cy").real(),
        "k1": fs.getNode("Camera2.k1").real(),
        "k2": fs.getNode("Camera2.k2").real(),
        "p1": fs.getNode("Camera2.p1").real(),
        "p2": fs.getNode("Camera2.p2").real(),
    }

    # 读取深度和双目变换矩阵
    params["Stereo"] = {
        "ThDepth": fs.getNode("Stereo.ThDepth").real(),
        "T_c1_c2": np.linalg.inv(fs.getNode("Stereo.T_c1_c2").mat())
    }

    params["IMU"] = {
        "T_b_c1": fs.getNode("IMU.T_b_c1").mat()
    }


    
    fs.release()
    return params

# 计算双目矫正参数并应用校正的函数
def compute_rectification_maps(yaml_path):
    # 读取参数
    params = read_yaml_file(yaml_path)

    # 提取参数
    camera1 = params["Camera1"]
    camera2 = params["Camera2"]
    stereo = params["Stereo"]
    width = params["Camera"]["width"]
    height = params["Camera"]["height"]

    K1 = np.array([[camera1["fx"], 0, camera1["cx"]], [0, camera1["fy"], camera1["cy"]], [0, 0, 1]])
    D1 = np.array([camera1["k1"], camera1["k2"], camera1["p1"], camera1["p2"]])

    K2 = np.array([[camera2["fx"], 0, camera2["cx"]], [0, camera2["fy"], camera2["cy"]], [0, 0, 1]])
    D2 = np.array([camera2["k1"], camera2["k2"], camera2["p1"], camera2["p2"]])

    R = np.array(stereo["T_c1_c2"][0:3, 0:3], dtype=np.float64)  # 确保 R 是 np.float64 类型
    T = np.array(stereo["T_c1_c2"][0:3, 3], dtype=np.float64)    # 确保 T 是 np.float64 类型


    print(f"K1 \n= {K1}")
    print(f"K1 \n= {D1}")
    print(f"K1 \n= {K2}")
    print(f"K1 \n= {D2}")
    print(f"R \n= {R}")
    print(f"t \n= {T}")

    image_size = (width, height)

    # 计算校正变换
    R1, R2, P1, P2, Q, valid_roi1, valid_roi2 = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, T, cv2.CALIB_ZERO_DISPARITY, alpha=-1)

    print(f"P1 \n= {P1}")

    # 计算校正映射
    map1_x, map1_y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
    map2_x, map2_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)

    b = P1[0,3]/P1[0,0]-P2[0,3]/P2[0,0]

    #矫正后的IMU-相机外参
    T_b_c1 = params["IMU"]["T_b_c1"]
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T_b_c1_new = T_b_c1.dot(np.linalg.inv(T1))

    return [map1_x, map1_y, map2_x, map2_y, P1, b, T_b_c1_new]


# 初始化 ROS 节点
#rospy.init_node('rosbag_image_reader')

# 创建 CvBridge 对象，用于转换 ROS 图像消息到 OpenCV 格式
bridge = CvBridge()

# 打开 rosbag 文件
bag_file = '/home/zhiyu/DataSet/SelfCollected/LYR_AGV/rosbag/0305/playground_cut_20250305.bag'  # 替换为你的 rosbag 文件路径
bag = rosbag.Bag(bag_file, 'r')

# 设置保存图像的文件夹路径
output_folder = 'output_images'
os.makedirs(output_folder, exist_ok=True)

params_path = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Config/LYR-AGV/LYR-AGV-20250318.yaml"
save_path = "/home/zhiyu/DataSet/SelfCollected/LYR_AGV/playground_cut_20250305"
left_imgs_save_path = f"{save_path}/image_2"
if(not os.path.exists(left_imgs_save_path)):
        os.system(f'mkdir {left_imgs_save_path}')
right_imgs_save_path = f"{save_path}/image_3"
if(not os.path.exists(right_imgs_save_path)):
        os.system(f'mkdir {right_imgs_save_path}')
timestamps_save_path = f"{save_path}/times.txt"
imu_save_path = f"{save_path}/imu.txt"

# 设置左右图像的主题名（根据你的实际情况修改）
left_topic = '/camera_array/cam0/image_raw'   # 左目图像的主题
right_topic = '/camera_array/cam1/image_raw'  # 右目图像的主题
imu_topic = '/livox/imu'

# 是否矫正
stereo_rectify = True
M1l_M2l_M1r_M2r_P_b_Tbc = []
if(stereo_rectify):
    M1l_M2l_M1r_M2r_P_b_Tbc = compute_rectification_maps(params_path)


# print(M1l_M2l_M1r_M2R)


# 设置图像的帧数计数器
left_counter = 0
right_counter = 0
imu_counter = 0

# timestamps
time_tolerance = 0.01

# dict
left_images = {}
right_images = {}
timestamps_data = []
imu_data = []

#=======================================Extracting RosBag=====================================
# 遍历 rosbag 中的消息
print("Reading bag file...")
for topic, msg, t in bag.read_messages():
    print(f"reading {topic} at {t.to_sec()}")
    ts = msg.header.stamp.to_sec()
    # 处理左目图像
    if topic == left_topic:
        left_images[ts] = msg
    # 处理右目图像
    elif topic == right_topic:
        right_images[ts] = msg
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
for left_time, left_msg in left_images.items():
    closest_right_time = min(right_images.keys(), key=lambda t: abs(t - left_time))
    if abs(left_time - closest_right_time) <= time_tolerance:
        # 获取右目图像消息
        right_msg = right_images[closest_right_time]

        print(f"associated timestamp: left {left_time}, right {closest_right_time}")

        # 转换为OpenCV格式图像
        left_cv_image = bridge.imgmsg_to_cv2(left_msg, "bgr8")
        right_cv_image = bridge.imgmsg_to_cv2(right_msg, "bgr8")

        if(stereo_rectify):
            rectified_left = cv2.remap(left_cv_image, M1l_M2l_M1r_M2r_P_b_Tbc[0], M1l_M2l_M1r_M2r_P_b_Tbc[1], interpolation=cv2.INTER_LINEAR)
            rectified_right = cv2.remap(right_cv_image, M1l_M2l_M1r_M2r_P_b_Tbc[2], M1l_M2l_M1r_M2r_P_b_Tbc[3], interpolation=cv2.INTER_LINEAR)
            left_cv_image = rectified_left
            right_cv_image = rectified_right


        # 保存图像
        left_image_path = os.path.join(left_imgs_save_path , f"{synced_count:06d}.png")
        right_image_path = os.path.join(right_imgs_save_path, f"{synced_count:06d}.png")
        cv2.imwrite(left_image_path, left_cv_image)
        cv2.imwrite(right_image_path, right_cv_image)
        timestamps_data.append([left_time])
        synced_count += 1


# 关闭 rosbag
bag.close()

timestamps_file = open(timestamps_save_path,"w+")
write_txt(timestamps_file, timestamps_data)

if(len(imu_data)!=0):
    imu_file=open(imu_save_path,"w+")
    write_txt(imu_file, imu_data)

timestamps_file.close()
print(f"Finished saving {synced_count} stereo images. {imu_counter} imu datas.")

#=======================================Extracted RosBag=====================================

if(stereo_rectify):
    print(f"After rectification, P = \n {M1l_M2l_M1r_M2r_P_b_Tbc[4]}, baseline = {M1l_M2l_M1r_M2r_P_b_Tbc[5]}\n, new Tbc = \n{M1l_M2l_M1r_M2r_P_b_Tbc[6]}.")
    
    
