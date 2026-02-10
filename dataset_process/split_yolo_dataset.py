import os
import shutil
import random
import argparse
from pathlib import Path

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}

def main(img_dir, txt_dir, train_ratio, out_dir, seed=42):
    random.seed(seed)

    img_dir = Path(img_dir)
    txt_dir = Path(txt_dir)
    out_dir = Path(out_dir)

    # 输出目录
    img_train = out_dir / "images" / "train"
    img_val   = out_dir / "images" / "val"
    lbl_train = out_dir / "labels" / "train"
    lbl_val   = out_dir / "labels" / "val"

    for d in [img_train, img_val, lbl_train, lbl_val]:
        d.mkdir(parents=True, exist_ok=True)

    # 收集有效图像（必须有对应 txt）
    pairs = []
    for img_path in img_dir.iterdir():
        if img_path.suffix.lower() not in IMG_EXTS:
            continue
        txt_path = txt_dir / (img_path.stem + ".txt")
        if txt_path.exists():
            pairs.append((img_path, txt_path))

    if not pairs:
        raise RuntimeError("未找到任何 图像-标签 对，请检查路径或命名是否一致")

    random.shuffle(pairs)

    n_train = int(len(pairs) * train_ratio)
    train_pairs = pairs[:n_train]
    val_pairs = pairs[n_train:]

    def copy_pairs(pairs, img_dst, lbl_dst):
        for img_path, txt_path in pairs:
            shutil.copy2(img_path, img_dst / img_path.name)
            shutil.copy2(txt_path, lbl_dst / txt_path.name)

    copy_pairs(train_pairs, img_train, lbl_train)
    copy_pairs(val_pairs, img_val, lbl_val)

    print(f"总样本数: {len(pairs)}")
    print(f"训练集: {len(train_pairs)}")
    print(f"验证集: {len(val_pairs)}")
    print(f"数据集已生成于: {out_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True, help="图像文件夹路径")
    parser.add_argument("--labels", required=True, help="txt 标签文件夹路径")
    parser.add_argument("--train_ratio", type=float, default=0.7, help="训练集比例 (0~1)")
    parser.add_argument("--out", required=True, help="输出数据集路径")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")

    args = parser.parse_args()
    main(args.images, args.labels, args.train_ratio, args.out, args.seed)
