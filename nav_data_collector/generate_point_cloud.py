import os
import argparse
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R


def read_poses(pose_file):
    """
    读取 pose.txt 文件，返回字典 {timestamp: (translation, quaternion)}
    """
    poses = {}
    with open(pose_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            ts = parts[0]
            trans = np.array([float(x) for x in parts[1:4]], dtype=np.float64)
            quat = np.array([float(x) for x in parts[4:8]], dtype=np.float64)  # qx, qy, qz, qw
            poses[ts] = (trans, quat)
    return poses


def depth_to_point_cloud(depth_img, rgb_img, intrinsic, trans, quat, pixel_step=1):
    """
    将单帧深度图 + RGB + 位姿转换为世界坐标系下的彩色点云
    支持 pixel_step 跳像素（例如 step=4 表示每4个像素取1个）
    """
    h, w = depth_img.shape
    fx, fy, cx, cy = intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]

    # 跳像素：只取子采样后的像素坐标
    u = np.arange(0, w, pixel_step)
    v = np.arange(0, h, pixel_step)
    u_grid, v_grid = np.meshgrid(u, v)
    u_grid = u_grid.astype(np.float64)
    v_grid = v_grid.astype(np.float64)

    # 对应的深度和RGB也需按相同位置采样
    depth_sampled = depth_img[::pixel_step, ::pixel_step]
    rgb_sampled = rgb_img[::pixel_step, ::pixel_step]

    # 有效深度掩码
    valid = (depth_sampled > 0)
    # z = depth_sampled.astype(np.float64) / 1000.0  # 转换为米
    z = depth_sampled.astype(np.float64) / 5208.0  # 转换为米

    x = (u_grid - cx) * z / fx
    y = (v_grid - cy) * z / fy

    points_cam = np.stack((x, y, z), axis=-1)  # [H', W', 3]
    colors = rgb_sampled.astype(np.float64) / 255.0

    points_cam = points_cam[valid]
    colors = colors[valid]

    if points_cam.size == 0:
        return None, None

    # 构建变换矩阵：相机坐标系 → 世界坐标系
    rot = R.from_quat(quat).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans

    points_hom = np.hstack([points_cam, np.ones((points_cam.shape[0], 1))])
    
    points_world = (T @ points_hom.T).T[:, :3]

    return points_world, colors


def main(dataset_path, frame_skip=1, pixel_step=1):
    rgb_dir = os.path.join(dataset_path, 'images')
    depth_dir = os.path.join(dataset_path, 'depth')
    pose_file = os.path.join(dataset_path, 'CameraTrajectory.txt')

    if not os.path.exists(rgb_dir):
        raise FileNotFoundError(f"RGB directory not found: {rgb_dir}")
    if not os.path.exists(depth_dir):
        raise FileNotFoundError(f"Depth directory not found: {depth_dir}")
    if not os.path.exists(pose_file):
        raise FileNotFoundError(f"Pose file not found: {pose_file}")

    poses = read_poses(pose_file)
    timestamps = list(poses.keys())

    # 相机内参（请根据你的设备调整）
    # D455
    # intrinsic = np.array([[385.2762145996094, 0.0, 316.4674072265625],
    #                       [0.0, 384.7712097167969, 255.935546875],
    #                       [0.0, 0.0, 1.0]])
    
    # 336L
    # intrinsic = np.array([[367.25384521484375, 0.0, 318.5082092285156],
    #                       [0.0, 367.1114196777344, 244.2189483642578],
    #                       [0.0, 0.0, 1.0]])
    
    # TUM2
    intrinsic = np.array([[520.908620, 0.0, 325.141442],
                          [0.0, 521.007327, 249.701764],
                          [0.0, 0.0, 1.0]])
    

    all_points = []
    all_colors = []

    total_frames = len(timestamps)
    processed_count = 0

    for i, ts in enumerate(timestamps):
        # 跳帧：只处理满足条件的帧
        if i % frame_skip != 0:
            continue

        trans, quat = poses[ts]
        rgb_path = os.path.join(rgb_dir, f"{ts}.png")
        depth_path = os.path.join(depth_dir, f"{ts}.png")

        if not os.path.exists(rgb_path) or not os.path.exists(depth_path):
            print(f"Skipping timestamp {ts}: missing image(s)")
            continue

        rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if rgb is None or depth is None:
            print(f"Failed to load images for timestamp {ts}")
            continue

        if rgb.shape[:2] != depth.shape[:2]:
            print(f"Size mismatch for timestamp {ts}, skipping")
            continue

        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        points, colors = depth_to_point_cloud(depth, rgb, intrinsic, trans, quat, pixel_step=pixel_step)

        if points is not None:
            all_points.append(points)
            all_colors.append(colors)
            processed_count += 1

        if processed_count % 10 == 0:
            print(f"Processed {processed_count} frames (total skipped: {i+1 - processed_count})")

    if not all_points:
        raise RuntimeError("No valid point clouds generated!")

    final_points = np.vstack(all_points)
    final_colors = np.vstack(all_colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(final_points)
    pcd.colors = o3d.utility.Vector3dVector(final_colors)

    # 可选：再做一次体素下采样（进一步精简）
    pcd = pcd.voxel_down_sample(voxel_size=0.01)

    output_path = os.path.join(dataset_path, "global_pointcloud.pcd")
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Global point cloud saved to: {output_path}")
    print(f"Total input frames: {total_frames}, Processed frames: {processed_count}")
    print(f"Pixel step: {pixel_step} → approx. 1/{pixel_step**2} pixels used per frame")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate global point cloud from RGB-D + poses with subsampling.")
    parser.add_argument("--dataset_path", type=str, required=True,
                        help="Path to the dataset folder containing rgb/, depth/, and pose.txt")
    parser.add_argument("--frame_skip", type=int, default=5,
                        help="Process every N-th frame (default: 1 = no skip)")
    parser.add_argument("--pixel_step", type=int, default=3,
                        help="Subsample pixels with step size (default: 1 = no skip; e.g., 4 → use 1/16 pixels)")

    args = parser.parse_args()

    main(args.dataset_path, frame_skip=args.frame_skip, pixel_step=args.pixel_step)