import os
import shutil
import argparse


# -----------------------------
# 读取文件（来自 TUM 官方）
# -----------------------------
def read_file_list(filename):
    with open(filename) as f:
        data = f.read()

    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    parsed = [
        [v.strip() for v in line.split(" ") if v.strip() != ""]
        for line in lines
        if len(line) > 0 and line[0] != "#"
    ]

    parsed = [(float(l[0]), l[1:]) for l in parsed if len(l) > 1]
    return dict(parsed)


# -----------------------------
# 官方 associate（核心）
# -----------------------------
def associate(first_list, second_list, offset, max_difference):
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())

    potential_matches = [
        (abs(a - (b + offset)), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - (b + offset)) < max_difference
    ]

    potential_matches.sort()

    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))

    matches.sort()
    return matches


# -----------------------------
# 主程序
# -----------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Generate TUM-style dataset with proper RGB-Depth-GT association (NO interpolation)"
    )
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--max_diff", type=float, default=0.02)
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--debug", action="store_true")

    args = parser.parse_args()

    dataset_dir = args.input
    output_dir = args.output

    rgb_file = os.path.join(dataset_dir, "rgb.txt")
    depth_file = os.path.join(dataset_dir, "depth.txt")
    gt_file = os.path.join(dataset_dir, "groundtruth.txt")

    rgb_out = os.path.join(output_dir, "images")
    depth_out = os.path.join(output_dir, "depth")

    os.makedirs(rgb_out, exist_ok=True)
    os.makedirs(depth_out, exist_ok=True)

    # -----------------------------
    # 读取数据
    # -----------------------------
    print("Loading files...")
    rgb_list = read_file_list(rgb_file)
    depth_list = read_file_list(depth_file)
    gt_list = read_file_list(gt_file)

    # -----------------------------
    # Step1: RGB ↔ Depth
    # -----------------------------
    print("Associating RGB and Depth...")
    rgb_depth_matches = associate(rgb_list, depth_list, args.offset, args.max_diff)

    # 构造中间结构
    rgbd_dict = {}
    for t_rgb, t_depth in rgb_depth_matches:
        rgbd_dict[t_rgb] = {
            "rgb": rgb_list[t_rgb][0],
            "depth": depth_list[t_depth][0],
        }

    # -----------------------------
    # Step2: RGB ↔ GT
    # -----------------------------
    print("Associating RGB and Groundtruth...")
    rgb_gt_matches = associate(rgbd_dict, gt_list, args.offset, args.max_diff)

    # -----------------------------
    # 输出文件
    # -----------------------------
    times_file = open(os.path.join(output_dir, "times.txt"), "w")
    pose_file = open(os.path.join(output_dir, "CameraTrajectory.txt"), "w")

    valid_count = 0

    for t_rgb, t_gt in rgb_gt_matches:
        rgb_rel = rgbd_dict[t_rgb]["rgb"]
        depth_rel = rgbd_dict[t_rgb]["depth"]

        rgb_path = os.path.join(dataset_dir, rgb_rel)
        depth_path = os.path.join(dataset_dir, depth_rel)

        filename = os.path.basename(rgb_rel)

        # 拷贝
        shutil.copy(rgb_path, os.path.join(rgb_out, filename))
        shutil.copy(depth_path, os.path.join(depth_out, filename))

        # 写时间
        times_file.write(f"{t_rgb:.6f}\n")

        # 写位姿（直接使用GT，不插值）
        pose = gt_list[t_gt]  # tx ty tz qx qy qz qw

        pose_file.write(
            f"{t_rgb:.6f} "
            f"{pose[0]} {pose[1]} {pose[2]} "
            f"{pose[3]} {pose[4]} {pose[5]} {pose[6]}\n"
        )

        valid_count += 1

        if args.debug:
            print(f"[MATCH] RGB {t_rgb:.6f} ↔ GT {t_gt:.6f} (dt={abs(t_rgb - t_gt):.6f})")

    times_file.close()
    pose_file.close()

    print("\n==============================")
    print(f"Finished. Valid frames: {valid_count}")
    print(f"Output: {output_dir}")
    print("==============================")


if __name__ == "__main__":
    main()