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
    S = np.diag(S_vec)
    
    # 计算旋转矩阵
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # 计算缩放因子
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

def convert_ned_to_wgs84(ned_points, origin_lla):
    """NED坐标转WGS84（直接处理北东地坐标系）"""
    # 定义投影参数（北东地坐标系）
    proj_params = {
        'proj': 'tmerc',
        'lat_0': origin_lla[0],    # 原点纬度
        'lon_0': origin_lla[1],     # 原点经度
        'k': 1,
        'ellps': 'WGS84',
        'units': 'm',
        'axis': 'neu'  # 使用北东天坐标轴顺序
    }
    
    transformer = Transformer.from_proj(
        Proj(**proj_params),
        Proj('EPSG:4326')  # WGS84
    )
    
    # 转换坐标（注意坐标顺序）
    # 输入应为北方向、东方向、地方向（高度取负）
    lats, lons = transformer.transform(
        ned_points[:,0],  # 北方向坐标（对应纬度增加）
        ned_points[:,1],  # 东方向坐标（对应经度增加）
    )
    alts = origin_lla[2] - ned_points[:,2]  # 地方向取反得到高度
    
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
        color='#1E90FF',
        weight=3,
        opacity=0.8,
        tooltip='组合导航轨迹'
    ).add_to(m)
    
    # 绘制SLAM轨迹
    if len(slam_global) > 0:
        folium.PolyLine(
            slam_global[:,:2].tolist(),
            color='#FF4500',
            weight=3,
            opacity=0.6,
            dash_array='5,5',
            tooltip='SLAM对齐轨迹'
        ).add_to(m)
    
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
    # 输入文件路径配置
    SLAM_POSES_PATH = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV/results/xinghu_cut_20250305_est.txt"       # SLAM局部位姿
    NAV_LOCAL_PATH = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV/ground_truth/xinghu_cut_20250305_gt.txt"        # 组合导航局部位姿 
    NAV_GLOBAL_PATH = "/home/zhiyu/LYCodes/ORB_SLAM3_Dev/Evaluation/LYR-AGV/ground_truth/xinghu_cut_20250305_gt_global.txt"          # 组合导航全局轨迹（新格式）
    
    # 加载全局数据
    nav_global = np.loadtxt(NAV_GLOBAL_PATH)[:,:3]  # 纬度、经度、高度
    
    # 加载局部位姿数据（保持原始坐标系）
    slam_poses = load_se3_poses(SLAM_POSES_PATH)
    nav_local_poses = load_se3_poses(NAV_LOCAL_PATH)  # 确保为NED坐标系

    # 步骤1：执行Umeyama对齐（直接在NED坐标系中进行）
    source = np.array([pose[:3,3] for pose in slam_poses]).T
    target = np.array([pose[:3,3] for pose in nav_local_poses]).T
    T_align = umeyama_alignment(source, target)
    
    # 应用对齐变换
    slam_aligned = [T_align @ pose for pose in slam_poses]
    
    # 步骤2：转换到全局坐标系
    origin_lla = nav_global[0]  # 使用组合导航起点作为NED原点
    ned_points = np.array([pose[:3,3] for pose in slam_aligned])
    slam_global_lla = convert_ned_to_wgs84(ned_points, origin_lla)

    # 验证坐标范围
    print("SLAM轨迹纬度范围:", np.min(slam_global_lla[:,0]), np.max(slam_global_lla[:,0]))
    print("SLAM轨迹经度范围:", np.min(slam_global_lla[:,1]), np.max(slam_global_lla[:,1]))
    
    #calculate_alignment_error(slam_aligned, nav_local_poses)
    
    # 步骤3：可视化
    output_html = "aligned_trajectory.html"
    visualize_trajectories(nav_global, slam_global_lla, output_html)
    print(f"可视化结果已保存至：{output_html}")