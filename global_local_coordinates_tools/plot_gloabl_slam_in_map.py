import numpy as np
import folium
from scipy.spatial.transform import Rotation
from pyproj import Proj, Transformer
import matplotlib.pyplot as plt

def load_se3_poses(file_path):
    """加载SE3格式位姿文件（每行12个元素：3x4矩阵平展）"""
    poses = np.loadtxt(file_path)
    return [np.vstack([pose.reshape(3,4), [0,0,0,1]]) for pose in poses]

def umeyama_alignment(source, target):
    """修正后的6DoF Umeyama对齐算法"""
    assert source.ndim == 2 and target.ndim == 2, "输入必须是二维矩阵"
    assert source.shape[1] == target.shape[1], "点数量必须相同"
    
    # 中心化
    src_mean = np.mean(source, axis=1, keepdims=True)
    tgt_mean = np.mean(target, axis=1, keepdims=True)
    src_centered = source - src_mean
    tgt_centered = target - tgt_mean
    
    # 计算协方差矩阵
    H = src_centered @ tgt_centered.T
    
    # SVD分解
    U, S_vec, Vt = np.linalg.svd(H)
    S = np.diag(S_vec)  # 将奇异值转换为对角矩阵
    
    # 计算旋转矩阵
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # 计算缩放因子 (修正点1)
    scale = np.trace(S) / np.trace(src_centered @ src_centered.T)
    
    # 计算平移向量
    t = tgt_mean - scale * R @ src_mean
    
    return np.vstack([np.hstack([scale*R, t]), [0,0,0,1]])

def calculate_alignment_error(slam_aligned, nav_local):
    """计算对齐后的位置误差"""
    errors = []
    for p_slam, p_nav in zip(slam_aligned, nav_local):
        error = np.linalg.norm(p_slam[:3,3] - p_nav[:3,3])
        errors.append(error)
    
    plt.figure(figsize=(10,4))
    plt.plot(errors)
    plt.title('局部坐标系对齐误差')
    plt.ylabel('误差(m)')
    plt.xlabel('帧序号')
    plt.grid()
    plt.savefig('alignment_error.png')
    plt.show()
    
    print(f"最大误差: {np.max(errors):.2f}m")
    print(f"平均误差: {np.mean(errors):.2f}m")

def convert_enu_to_wgs84(enu_points, origin_lla):
    """ENU坐标转WGS84（修正投影参数）"""
    # 定义ENU局部坐标系（使用OMERC投影模拟）
    enu_crs = {
        'proj': 'omerc',
        'lat_0': origin_lla[0],
        'lonc': origin_lla[1],
        'alpha': 90,  # 旋转角度，使Y轴指向北
        'gamma': 0,
        'k': 1,
        'ellps': 'WGS84',
        'h': origin_lla[2],
        'units': 'm'
    }
    
    # 创建坐标转换器
    transformer = Transformer.from_crs(enu_crs, "EPSG:4326")  # WGS84
    
    # 执行坐标转换
    lats, lons, alts = transformer.transform(
        enu_points[:,0], 
        enu_points[:,1], 
        enu_points[:,2]
    )
    
    return np.column_stack([lats, lons, alts])

def visualize_trajectories(nav_global, slam_global, output_path):
    """可视化全局轨迹"""
    map_center = [np.mean(nav_global[:,0]), np.mean(nav_global[:,1])]
    
    m = folium.Map(location=map_center, zoom_start=17,
                  tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
                  attr='Google Satellite')
    
    # 绘制组合导航轨迹
    folium.PolyLine(
        nav_global[:,:2].tolist(),
        #color='#1E90FF',
        color='#FF4500',
        weight=6,
        opacity=0.8,
        tooltip='组合导航轨迹'
    ).add_to(m)
    
    # 绘制SLAM轨迹
    # folium.PolyLine(
    #     slam_global[:,:2].tolist(),
    #     color='#FF4500',
    #     weight=3,
    #     opacity=0.6,
    #     dash_array='5,5',
    #     tooltip='SLAM对齐轨迹'
    # ).add_to(m)
    
    # 添加起终点标记
    folium.Marker(
        nav_global[0,:2].tolist(),
        icon=folium.Icon(color='green', icon='flag'),
        tooltip=f"起点\n纬度: {nav_global[0,0]:.6f}\n经度: {nav_global[0,1]:.6f}"
    ).add_to(m)
    
    folium.Marker(
        nav_global[-1,:2].tolist(),
        icon=folium.Icon(color='blue', icon='stop'),
        tooltip=f"终点\n纬度: {nav_global[-1,0]:.6f}\n经度: {nav_global[-1,1]:.6f}"
    ).add_to(m)
    
    m.save(output_path)
    return m

if __name__ == "__main__":
    # 输入文件路径配置（根据实际文件格式修改列索引）
    SLAM_POSES_PATH = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV/results/xinghu_cut_20250305_est.txt"       # SLAM局部位姿
    NAV_LOCAL_PATH = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV/ground_truth/xinghu_cut_20250305_gt.txt"        # 组合导航局部位姿 
    NAV_GLOBAL_PATH = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV/ground_truth/xinghu_cut_20250305_gt_global.txt"          # 组合导航全局轨迹（新格式）
    
    # 加载全局数据（假设新格式直接为纬度、经度、高度）
    # 如果数据有其他列，需调整列索引。例如：
    # 若格式为 [其他列, lat, lon, alt, ...]，则改为 nav_global = np.loadtxt(NAV_GLOBAL_PATH)[:,1:4]
    nav_global = np.loadtxt(NAV_GLOBAL_PATH)[:,:3]  # 提取前三列: lat, lon, alt
    
    # 加载局部位姿数据
    slam_poses = load_se3_poses(SLAM_POSES_PATH)
    nav_local_poses = load_se3_poses(NAV_LOCAL_PATH)
    
    # 验证数据维度
    print(f"全局数据维度: {nav_global.shape} (应为N×3)")
    print(f"SLAM位姿数量: {len(slam_poses)}")
    print(f"导航局部位姿数量: {len(nav_local_poses)}")
    
    # 步骤1：执行Umeyama对齐
    source = np.array([pose[:3,3] for pose in slam_poses]).T
    target = np.array([pose[:3,3] for pose in nav_local_poses]).T
    T_align = umeyama_alignment(source, target)
    
    # 应用对齐变换
    slam_aligned = [T_align @ pose for pose in slam_poses]
    
    # 步骤2：转换到全局坐标系
    origin_lla = nav_global[0]  # 使用第一个导航点作为ENU原点
    enu_points = np.array([pose[:3,3] for pose in slam_aligned])
    slam_global_lla = convert_enu_to_wgs84(enu_points, origin_lla)

    calculate_alignment_error(slam_aligned, nav_local_poses)
    
    # 步骤3：可视化
    output_html = "trajectory.html"
    visualize_trajectories(nav_global, slam_global_lla, output_html)
    print(f"可视化结果已保存至：{output_html}")
















