#!/usr/bin/env python3
import os
import argparse
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R


def parse_ros_time_to_ns(time_str):
    """
    将 ROS 时间戳字符串转为整数纳秒。
    支持:
      - "1700000000.123456789" (标准 ROS time)
      - "1700000000123456789"   (整数纳秒)
    """
    if '.' in time_str:
        parts = time_str.split('.')
        if len(parts) != 2:
            raise ValueError("Invalid float-like timestamp")
        sec = parts[0]
        nanosec = parts[1].ljust(9, '0')[:9]  # 补齐或截断到9位
        return int(sec) * 1_000_000_000 + int(nanosec)
    else:
        # 假设是整数纳秒（可能来自某些数据集）
        return int(time_str)


def read_poses(pose_file):
    """
    读取 pose.txt，返回 [(ts_ns: int, trans, quat)]，按时间排序
    """
    poses = []
    with open(pose_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            ts_str = parts[0]
            try:
                ts_ns = parse_ros_time_to_ns(ts_str)
            except Exception as e:
                print(f"Warning: skipping invalid timestamp '{ts_str}': {e}")
                continue
            trans = np.array([float(x) for x in parts[1:4]], dtype=np.float64)
            quat = np.array([float(x) for x in parts[4:8]], dtype=np.float64)  # qx, qy, qz, qw
            poses.append((ts_ns, trans, quat))
    poses.sort(key=lambda x: x[0])
    return poses


def get_image_timestamps_with_names(img_dir):
    """
    返回 [(ts_ns: int, original_name: str)] 列表，按时间排序
    original_name 是文件名去掉 .png 的部分（用于后续读图）
    """
    ts_name_list = []
    for fname in os.listdir(img_dir):
        if not fname.endswith('.png'):
            continue
        ts_str = fname[:-4]
        try:
            ts_ns = parse_ros_time_to_ns(ts_str)
            ts_name_list.append((ts_ns, ts_str))
        except Exception as e:
            continue
    ts_name_list.sort(key=lambda x: x[0])
    return ts_name_list


def find_closest_image_name(pose_ts_ns, img_ts_name_list, time_tol_ns):
    """
    在 img_ts_name_list 中找与 pose_ts_ns 最近的图像（时间差 <= time_tol_ns）
    返回原始文件名字符串（如 "1700000000.123456789"），若无则返回 None
    """
    if not img_ts_name_list:
        return None

    ts_arr = np.array([item[0] for item in img_ts_name_list])
    idx = np.searchsorted(ts_arr, pose_ts_ns)

    best_diff = float('inf')
    best_name = None

    # 检查 idx-1 和 idx 位置
    for i in [idx - 1, idx]:
        if 0 <= i < len(ts_arr):
            diff = abs(pose_ts_ns - ts_arr[i])
            if diff <= time_tol_ns and diff < best_diff:
                best_diff = diff
                best_name = img_ts_name_list[i][1]

    return best_name


def depth_to_point_cloud(depth_img, rgb_img, intrinsic, trans, quat, pixel_step=1):
    h, w = depth_img.shape
    fx, fy, cx, cy = intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]

    u = np.arange(0, w, pixel_step)
    v = np.arange(0, h, pixel_step)
    u_grid, v_grid = np.meshgrid(u, v)
    u_grid = u_grid.astype(np.float64)
    v_grid = v_grid.astype(np.float64)

    depth_sampled = depth_img[::pixel_step, ::pixel_step]
    rgb_sampled = rgb_img[::pixel_step, ::pixel_step]

    valid = (depth_sampled > 0) & (depth_sampled < 10000)
    z = depth_sampled.astype(np.float64) / 1000.0  # mm → m

    x = (u_grid - cx) * z / fx
    y = (v_grid - cy) * z / fy

    points_cam = np.stack((x, y, z), axis=-1)
    colors = rgb_sampled.astype(np.float64) / 255.0

    points_cam = points_cam[valid]
    colors = colors[valid]

    if points_cam.size == 0:
        return None, None

    rot = R.from_quat(quat).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans

    points_hom = np.hstack([points_cam, np.ones((points_cam.shape[0], 1))])
    points_world = (T @ points_hom.T).T[:, :3]

    return points_world, colors


def main(dataset_path, frame_skip=1, pixel_step=1, time_tol_sec=0.05):
    rgb_dir = os.path.join(dataset_path, 'rgb')
    depth_dir = os.path.join(dataset_path, 'depth')
    pose_file = os.path.join(dataset_path, 'pose.txt')

    if not os.path.exists(rgb_dir):
        raise FileNotFoundError(f"RGB directory not found: {rgb_dir}")
    if not os.path.exists(depth_dir):
        raise FileNotFoundError(f"Depth directory not found: {depth_dir}")
    if not os.path.exists(pose_file):
        raise FileNotFoundError(f"Pose file not found: {pose_file}")

    # 读取位姿（整数纳秒）
    poses = read_poses(pose_file)

    # 读取图像时间戳 + 原始文件名
    rgb_list = get_image_timestamps_with_names(rgb_dir)
    depth_list = get_image_timestamps_with_names(depth_dir)

    if not rgb_list or not depth_list:
        raise RuntimeError("No valid images found in rgb/ or depth/")

    # 构建 depth 文件名集合（用于 O(1) 查找）
    depth_names_set = set(name for _, name in depth_list)

    print(f"Loaded {len(poses)} poses, {len(rgb_list)} RGB images, {len(depth_list)} depth images.")

    # 相机内参（请根据你的设备调整）
    intrinsic = np.array([[385.2762145996094, 0.0, 316.4674072265625],
                          [0.0, 384.7712097167969, 255.935546875],
                          [0.0, 0.0, 1.0]])

    all_points = []
    all_colors = []
    processed_count = 0

    # 时间容差转为纳秒
    time_tol_ns = int(time_tol_sec * 1_000_000_000)

    for i, (pose_ts_ns, trans, quat) in enumerate(poses):
        if i % frame_skip != 0:
            continue

        # 找最邻近的 RGB 图像（返回原始文件名）
        matched_name = find_closest_image_name(pose_ts_ns, rgb_list, time_tol_ns)
        if matched_name is None:
            continue

        # 检查 depth 是否存在同名文件
        if matched_name not in depth_names_set:
            continue

        rgb_path = os.path.join(rgb_dir, f"{matched_name}.png")
        depth_path = os.path.join(depth_dir, f"{matched_name}.png")

        if not os.path.exists(rgb_path) or not os.path.exists(depth_path):
            continue

        rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if rgb is None or depth is None:
            continue

        if rgb.shape[:2] != depth.shape[:2]:
            continue

        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        points, colors = depth_to_point_cloud(depth, rgb, intrinsic, trans, quat, pixel_step=pixel_step)

        if points is not None:
            all_points.append(points)
            all_colors.append(colors)
            processed_count += 1

        if processed_count % 10 == 0:
            print(f"Processed {processed_count} frames")

    if not all_points:
        raise RuntimeError("No valid point clouds generated!")

    final_points = np.vstack(all_points)
    final_colors = np.vstack(all_colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(final_points)
    pcd.colors = o3d.utility.Vector3dVector(final_colors)

    # 可选：体素下采样
    pcd = pcd.voxel_down_sample(voxel_size=0.01)

    output_path = os.path.join(dataset_path, "global_pointcloud.pcd")
    o3d.io.write_point_cloud(output_path, pcd)

    print(f"\n✅ Global point cloud saved to: {output_path}")
    print(f"Total poses: {len(poses)}, Processed frames: {processed_count}")
    print(f"Time tolerance: {time_tol_sec:.3f} sec ({time_tol_ns} ns)")
    print(f"Pixel step: {pixel_step} → ～1/{pixel_step**2} pixels per frame")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate global point cloud from RGB-D + poses with precise timestamp matching.")
    parser.add_argument("--dataset_path", type=str, required=True,
                        help="Path to dataset folder containing rgb/, depth/, and pose.txt")
    parser.add_argument("--frame_skip", type=int, default=5,
                        help="Process every N-th pose (default: 5)")
    parser.add_argument("--pixel_step", type=int, default=3,
                        help="Subsample pixels with step size (default: 3 → use ～1/9 pixels)")
    parser.add_argument("--time_tol", type=float, default=0.001,
                        help="Max allowed time difference in seconds (default: 0.05 = 50ms)")

    args = parser.parse_args()
    main(
        dataset_path=args.dataset_path,
        frame_skip=args.frame_skip,
        pixel_step=args.pixel_step,
        time_tol_sec=args.time_tol
    )