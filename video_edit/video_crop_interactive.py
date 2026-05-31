#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式视频裁切脚本。

功能：
1. 命令行读取输入视频路径；
2. 显示视频首帧，可在窗口中移动鼠标查看原始像素坐标与 BGR/RGB 颜色值；
3. 支持鼠标拖拽选择轴对齐矩形，或鼠标点击 4 个顶点做透视裁切；
4. 也支持命令行直接传入矩形坐标或 4 个顶点坐标；
5. 将选中区域逐帧裁切/透视变换后输出为新视频，输出分辨率即选区大小。

注意：本脚本使用 OpenCV 写视频，不会保留原视频音频轨。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np


Point = Tuple[int, int]
Rect = Tuple[int, int, int, int]  # x, y, w, h


def import_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "未找到 OpenCV。请先安装：\n"
            "  pip install opencv-python\n"
            "如果服务器无 GUI，可尝试：pip install opencv-python-headless "
            "（但 headless 版本不能使用交互窗口）。"
        ) from exc
    return cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="显示视频首帧，交互选择矩形区域并输出裁切后的视频。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("video", help="输入视频路径")
    parser.add_argument("-o", "--output", help="输出视频路径；不指定则自动生成")
    parser.add_argument(
        "--mode",
        choices=("rect", "points"),
        default="rect",
        help=(
            "选择模式：rect=拖拽轴对齐矩形；"
            "points=点击/输入 4 个顶点并做透视裁切"
        ),
    )
    parser.add_argument(
        "--rect",
        nargs=4,
        type=int,
        metavar=("X1", "Y1", "X2", "Y2"),
        help="跳过交互，直接使用两个对角点坐标裁切轴对齐矩形",
    )
    parser.add_argument(
        "--points",
        nargs=8,
        type=float,
        metavar=("X1", "Y1", "X2", "Y2", "X3", "Y3", "X4", "Y4"),
        help=(
            "跳过交互，直接使用 4 个顶点做透视裁切；"
            "顶点可按顺时针/逆时针输入，脚本会自动排序"
        ),
    )
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="OpenCV VideoWriter fourcc 编码，例如 mp4v、XVID、MJPG、avc1",
    )
    parser.add_argument(
        "--display-scale",
        type=float,
        help=(
            "首帧窗口显示缩放比例。不指定时，按 max-display-width/height "
            "自动缩小；鼠标显示的仍是原始视频像素坐标"
        ),
    )
    parser.add_argument(
        "--max-display-width",
        type=int,
        default=1600,
        help="自动显示缩放时的最大窗口宽度",
    )
    parser.add_argument(
        "--max-display-height",
        type=int,
        default=900,
        help="自动显示缩放时的最大窗口高度",
    )
    parser.add_argument(
        "--show-every",
        type=int,
        default=30,
        help="处理时每隔多少帧打印一次进度；<=0 表示不打印",
    )
    return parser.parse_args()


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def normalize_rect_from_corners(
    p1: Point,
    p2: Point,
    frame_width: int,
    frame_height: int,
) -> Rect:
    x1 = clamp(int(round(p1[0])), 0, frame_width - 1)
    y1 = clamp(int(round(p1[1])), 0, frame_height - 1)
    x2 = clamp(int(round(p2[0])), 0, frame_width - 1)
    y2 = clamp(int(round(p2[1])), 0, frame_height - 1)

    left, right = sorted((x1, x2))
    top, bottom = sorted((y1, y2))
    # Python 切片右边界不包含，因此宽高用坐标差 + 1，更符合“两个像素点都包含”的直觉。
    w = right - left + 1
    h = bottom - top + 1
    if w <= 1 or h <= 1:
        raise ValueError(f"矩形太小：x={left}, y={top}, w={w}, h={h}")
    return left, top, w, h


def choose_display_scale(
    frame_width: int,
    frame_height: int,
    display_scale: Optional[float],
    max_display_width: int,
    max_display_height: int,
) -> float:
    if display_scale is not None:
        if display_scale <= 0:
            raise ValueError("--display-scale 必须大于 0")
        return display_scale

    scale = min(
        1.0,
        max_display_width / frame_width if max_display_width > 0 else 1.0,
        max_display_height / frame_height if max_display_height > 0 else 1.0,
    )
    return max(scale, 1e-6)


def source_to_display(point: Point, scale: float) -> Point:
    return int(round(point[0] * scale)), int(round(point[1] * scale))


def display_to_source(point: Point, scale: float, frame_width: int, frame_height: int) -> Point:
    x = int(round(point[0] / scale))
    y = int(round(point[1] / scale))
    return clamp(x, 0, frame_width - 1), clamp(y, 0, frame_height - 1)


@dataclass
class SelectionState:
    mode: str
    frame: np.ndarray
    scale: float
    cv2: object
    cursor: Optional[Point] = None
    dragging: bool = False
    drag_start: Optional[Point] = None
    drag_end: Optional[Point] = None
    rect: Optional[Rect] = None
    points: list[Point] = field(default_factory=list)
    confirmed: bool = False
    cancelled: bool = False
    message: str = ""

    @property
    def frame_height(self) -> int:
        return int(self.frame.shape[0])

    @property
    def frame_width(self) -> int:
        return int(self.frame.shape[1])

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        cv2 = self.cv2
        sx, sy = display_to_source((x, y), self.scale, self.frame_width, self.frame_height)
        self.cursor = (sx, sy)

        if self.mode == "rect":
            if event == cv2.EVENT_LBUTTONDOWN:
                self.dragging = True
                self.drag_start = (sx, sy)
                self.drag_end = (sx, sy)
                self.rect = None
                self.message = "Dragging: release left button, then press c/Enter/Space to confirm."
            elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
                self.drag_end = (sx, sy)
            elif event == cv2.EVENT_LBUTTONUP and self.dragging:
                self.dragging = False
                self.drag_end = (sx, sy)
                if self.drag_start is not None:
                    try:
                        self.rect = normalize_rect_from_corners(
                            self.drag_start,
                            self.drag_end,
                            self.frame_width,
                            self.frame_height,
                        )
                        x0, y0, w, h = self.rect
                        self.message = (
                            f"Selected rect: x={x0}, y={y0}, w={w}, h={h}; "
                            "press c/Enter/Space to confirm, r to reset."
                        )
                    except ValueError as exc:
                        self.rect = None
                        self.message = str(exc)

        elif self.mode == "points":
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(self.points) < 4:
                    self.points.append((sx, sy))
                    if len(self.points) < 4:
                        self.message = f"Added point {len(self.points)}; keep clicking."
                    else:
                        self.message = "Selected 4 points; press c/Enter/Space to confirm, u to undo."
            elif event == cv2.EVENT_RBUTTONDOWN:
                if self.points:
                    self.points.pop()
                    self.message = "Undid last point."

    def reset(self) -> None:
        self.dragging = False
        self.drag_start = None
        self.drag_end = None
        self.rect = None
        self.points.clear()
        self.confirmed = False
        self.message = "Reset."


def put_text_lines(cv2, image: np.ndarray, lines: Sequence[str]) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 1
    line_height = 22
    pad = 8

    # 半透明背景
    overlay = image.copy()
    max_width = 0
    for line in lines:
        size, _ = cv2.getTextSize(line, font, font_scale, thickness)
        max_width = max(max_width, size[0])
    box_w = min(image.shape[1], max_width + pad * 2)
    box_h = min(image.shape[0], line_height * len(lines) + pad * 2)
    cv2.rectangle(overlay, (0, 0), (box_w, box_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, image, 0.45, 0, image)

    y = pad + 15
    for line in lines:
        cv2.putText(image, line, (pad, y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        y += line_height


def render_selection(state: SelectionState) -> np.ndarray:
    cv2 = state.cv2
    frame = state.frame.copy()

    if state.mode == "rect":
        rect = state.rect
        if state.drag_start is not None and state.drag_end is not None:
            try:
                rect = normalize_rect_from_corners(
                    state.drag_start,
                    state.drag_end,
                    state.frame_width,
                    state.frame_height,
                )
            except ValueError:
                rect = None
        if rect is not None:
            x, y, w, h = rect
            cv2.rectangle(frame, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{w}x{h}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
    else:
        pts = state.points
        for idx, point in enumerate(pts):
            cv2.circle(frame, point, 5, (0, 255, 255), -1)
            cv2.putText(
                frame,
                str(idx + 1),
                (point[0] + 6, point[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        if len(pts) >= 2:
            for p1, p2 in zip(pts, pts[1:]):
                cv2.line(frame, p1, p2, (0, 255, 255), 2)
            if len(pts) == 4:
                cv2.line(frame, pts[-1], pts[0], (0, 255, 255), 2)
                try:
                    ordered = order_points(np.asarray(pts, dtype=np.float32))
                    out_w, out_h = perspective_output_size(ordered)
                    cv2.putText(
                        frame,
                        f"{out_w}x{out_h}",
                        pts[0],
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                except ValueError:
                    pass

    if abs(state.scale - 1.0) > 1e-9:
        display_w = max(1, int(round(state.frame_width * state.scale)))
        display_h = max(1, int(round(state.frame_height * state.scale)))
        interpolation = cv2.INTER_AREA if state.scale < 1 else cv2.INTER_LINEAR
        frame = cv2.resize(frame, (display_w, display_h), interpolation=interpolation)

    lines: list[str] = []
    if state.cursor is not None:
        x, y = state.cursor
        b, g, r = state.frame[y, x].tolist()
        lines.append(f"cursor: x={x}, y={y} | BGR=({b},{g},{r}) RGB=({r},{g},{b})")
    else:
        lines.append("cursor: move mouse into image to show x/y and BGR/RGB")

    if state.mode == "rect":
        lines.append("rect mode: left-drag select | c/Enter/Space confirm | i input coords | r reset | q/Esc quit")
    else:
        lines.append("points mode: left-click 4 corners | right-click/u undo | c/Enter/Space confirm | i input 8 nums | r reset | q/Esc quit")
    if state.message:
        lines.append(state.message)
    put_text_lines(cv2, frame, lines[:4])
    return frame


def parse_numbers_from_stdin(prompt: str, expected_count: int) -> Optional[list[float]]:
    try:
        raw = input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        return None
    if not raw:
        return None
    # 兼容 "x1,y1 x2,y2 ..." 和 "x1 y1 x2 y2 ..."
    raw = raw.replace(",", " ")
    parts = raw.split()
    if len(parts) != expected_count:
        print(f"需要 {expected_count} 个数字，但收到 {len(parts)} 个。", file=sys.stderr)
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        print("输入中包含非数字。", file=sys.stderr)
        return None


def interactive_select(
    cv2,
    frame: np.ndarray,
    mode: str,
    display_scale: Optional[float],
    max_display_width: int,
    max_display_height: int,
) -> tuple[str, Rect | np.ndarray]:
    frame_height, frame_width = frame.shape[:2]
    scale = choose_display_scale(
        frame_width,
        frame_height,
        display_scale,
        max_display_width,
        max_display_height,
    )
    state = SelectionState(mode=mode, frame=frame, scale=scale, cv2=cv2)
    state.message = (
        f"Source frame: {frame_width}x{frame_height}; display scale: {scale:.4g}."
    )

    win_name = "Select crop on first frame"
    cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win_name, state.mouse_callback)

    while True:
        cv2.imshow(win_name, render_selection(state))
        key = cv2.waitKey(20) & 0xFF
        if key == 255:
            continue
        if key in (ord("q"), 27):  # q or Esc
            state.cancelled = True
            break
        if key in (ord("r"),):
            state.reset()
        elif key in (ord("u"), 8, 127) and state.mode == "points":
            if state.points:
                state.points.pop()
                state.message = "Undid last point."
        elif key in (ord("i"),):
            if state.mode == "rect":
                nums = parse_numbers_from_stdin("请输入矩形两个对角点 x1 y1 x2 y2：", 4)
                if nums is not None:
                    try:
                        state.rect = normalize_rect_from_corners(
                            (int(round(nums[0])), int(round(nums[1]))),
                            (int(round(nums[2])), int(round(nums[3]))),
                            frame_width,
                            frame_height,
                        )
                        x, y, w, h = state.rect
                        state.message = f"Input rect: x={x}, y={y}, w={w}, h={h}. Press c to confirm."
                    except ValueError as exc:
                        state.message = str(exc)
            else:
                nums = parse_numbers_from_stdin("请输入 4 个顶点 x1 y1 x2 y2 x3 y3 x4 y4：", 8)
                if nums is not None:
                    pts = [(int(round(nums[i])), int(round(nums[i + 1]))) for i in range(0, 8, 2)]
                    pts = [
                        (clamp(x, 0, frame_width - 1), clamp(y, 0, frame_height - 1))
                        for x, y in pts
                    ]
                    state.points = pts
                    state.message = "Input 4 points. Press c to confirm."
        elif key in (ord("c"), ord("\r"), ord("\n"), ord(" ")):
            if state.mode == "rect":
                if state.rect is not None:
                    state.confirmed = True
                    break
                state.message = "Please drag/input a valid rectangle first."
            else:
                if len(state.points) == 4:
                    try:
                        pts = order_points(np.asarray(state.points, dtype=np.float32))
                        # 校验宽高
                        perspective_output_size(pts)
                        state.confirmed = True
                        break
                    except ValueError as exc:
                        state.message = str(exc)
                else:
                    state.message = f"Points mode needs 4 points; current: {len(state.points)}."

    cv2.destroyWindow(win_name)
    if state.cancelled or not state.confirmed:
        raise SystemExit("已取消。")

    if state.mode == "rect":
        assert state.rect is not None
        return "rect", state.rect

    return "points", order_points(np.asarray(state.points, dtype=np.float32))


def order_points(pts: np.ndarray) -> np.ndarray:
    """将 4 个点排序为 top-left, top-right, bottom-right, bottom-left。

    用“相对中心点的角度”排序，比常见的 x+y / y-x 排序更能处理菱形、
    大角度旋转矩形等情况。
    """
    if pts.shape != (4, 2):
        raise ValueError("需要形状为 (4, 2) 的点数组")

    pts = pts.astype(np.float32)
    if len({(float(x), float(y)) for x, y in pts}) != 4:
        raise ValueError("4 个点中存在重复点，请重新选择。")

    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    rect = pts[np.argsort(angles)]  # 图像坐标系下通常为 TL, TR, BR, BL 的顺序

    # 旋转数组，让最接近“左上角”的点作为第一个点。
    start = int(np.argmin(rect[:, 0] + rect[:, 1]))
    rect = np.roll(rect, -start, axis=0)

    # 面积太小表示点几乎共线，无法形成有效四边形。
    x = rect[:, 0]
    y = rect[:, 1]
    area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    if area <= 1.0:
        raise ValueError("4 个点几乎共线或面积太小，无法形成有效矩形/四边形。")
    return rect


def perspective_output_size(ordered_pts: np.ndarray) -> tuple[int, int]:
    tl, tr, br, bl = ordered_pts
    width_top = float(np.linalg.norm(tr - tl))
    width_bottom = float(np.linalg.norm(br - bl))
    height_right = float(np.linalg.norm(br - tr))
    height_left = float(np.linalg.norm(bl - tl))

    out_w = int(round(max(width_top, width_bottom)))
    out_h = int(round(max(height_left, height_right)))
    if out_w <= 1 or out_h <= 1:
        raise ValueError(f"透视裁切尺寸太小：{out_w}x{out_h}")
    return out_w, out_h


def sanitize_codec(codec: str) -> str:
    if len(codec) != 4:
        raise ValueError("--codec 必须是 4 个字符的 FourCC，例如 mp4v")
    return codec


def default_output_path(input_path: str, selection_kind: str, size: tuple[int, int]) -> str:
    path = Path(input_path)
    stem = path.stem
    suffix = path.suffix or ".mp4"
    out_suffix = suffix if suffix.lower() in {".mp4", ".avi", ".mov", ".mkv"} else ".mp4"
    w, h = size
    return str(path.with_name(f"{stem}_{selection_kind}_crop_{w}x{h}{out_suffix}"))


def open_video_or_die(cv2, video_path: str):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise SystemExit(f"无法打开视频：{video_path}")
    return cap


def get_fps(cap) -> float:
    fps = float(cap.get(5))  # cv2.CAP_PROP_FPS == 5
    if not math.isfinite(fps) or fps <= 1e-6:
        fps = 30.0
    return fps


def make_writer(cv2, output_path: str, codec: str, fps: float, size: tuple[int, int]):
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(output_path, fourcc, fps, size)
    if not writer.isOpened():
        raise SystemExit(
            f"无法创建输出视频：{output_path}\n"
            f"可尝试更换扩展名或编码，例如：--codec XVID -o output.avi"
        )
    return writer


def read_video_size(cv2, video_path: str) -> Optional[tuple[int, int]]:
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        return None
    try:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    finally:
        cap.release()
    if w <= 0 or h <= 0:
        return None
    return w, h


def crop_video_rect(
    cv2,
    video_path: str,
    output_path: str,
    rect: Rect,
    codec: str,
    show_every: int,
) -> int:
    x, y, w, h = rect
    cap = open_video_or_die(cv2, video_path)
    fps = get_fps(cap)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    writer = make_writer(cv2, output_path, codec, fps, (w, h))

    frame_idx = 0
    written = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            crop = frame[y : y + h, x : x + w]
            if crop.shape[0] != h or crop.shape[1] != w:
                # 理论上不会发生，除非视频中途分辨率变化。
                if crop.size == 0:
                    crop = np.zeros((h, w, 3), dtype=frame.dtype)
                else:
                    crop = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
            writer.write(crop)
            written += 1
            frame_idx += 1
            if show_every > 0 and (written == 1 or written % show_every == 0):
                suffix = f"/{total}" if total > 0 else ""
                print(f"\r处理中：{written}{suffix} 帧", end="", flush=True)
    finally:
        cap.release()
        writer.release()
    if show_every > 0:
        print()
    return written


def crop_video_points(
    cv2,
    video_path: str,
    output_path: str,
    ordered_pts: np.ndarray,
    codec: str,
    show_every: int,
) -> tuple[int, tuple[int, int]]:
    out_w, out_h = perspective_output_size(ordered_pts)
    dst = np.asarray(
        [[0, 0], [out_w - 1, 0], [out_w - 1, out_h - 1], [0, out_h - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(ordered_pts.astype(np.float32), dst)

    cap = open_video_or_die(cv2, video_path)
    fps = get_fps(cap)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    writer = make_writer(cv2, output_path, codec, fps, (out_w, out_h))

    written = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            warped = cv2.warpPerspective(frame, matrix, (out_w, out_h))
            writer.write(warped)
            written += 1
            if show_every > 0 and (written == 1 or written % show_every == 0):
                suffix = f"/{total}" if total > 0 else ""
                print(f"\r处理中：{written}{suffix} 帧", end="", flush=True)
    finally:
        cap.release()
        writer.release()
    if show_every > 0:
        print()
    return written, (out_w, out_h)


def main() -> None:
    args = parse_args()
    cv2 = import_cv2()

    video_path = args.video
    if not os.path.exists(video_path):
        raise SystemExit(f"输入视频不存在：{video_path}")

    codec = sanitize_codec(args.codec)

    cap = open_video_or_die(cv2, video_path)
    ok, first_frame = cap.read()
    cap.release()
    if not ok or first_frame is None:
        raise SystemExit("无法读取视频首帧。")

    frame_h, frame_w = first_frame.shape[:2]

    selection_kind: str
    selection: Rect | np.ndarray
    if args.rect is not None and args.points is not None:
        raise SystemExit("--rect 和 --points 不能同时使用。")
    if args.rect is not None:
        selection_kind = "rect"
        selection = normalize_rect_from_corners(
            (args.rect[0], args.rect[1]),
            (args.rect[2], args.rect[3]),
            frame_w,
            frame_h,
        )
    elif args.points is not None:
        raw_pts = np.asarray(args.points, dtype=np.float32).reshape(4, 2)
        raw_pts[:, 0] = np.clip(raw_pts[:, 0], 0, frame_w - 1)
        raw_pts[:, 1] = np.clip(raw_pts[:, 1], 0, frame_h - 1)
        selection_kind = "points"
        selection = order_points(raw_pts)
        # 校验尺寸
        perspective_output_size(selection)
    else:
        selection_kind, selection = interactive_select(
            cv2=cv2,
            frame=first_frame,
            mode=args.mode,
            display_scale=args.display_scale,
            max_display_width=args.max_display_width,
            max_display_height=args.max_display_height,
        )

    if selection_kind == "rect":
        x, y, w, h = selection  # type: ignore[misc]
        output_size = (w, h)
        print(f"最终矩形：x={x}, y={y}, w={w}, h={h}")
    else:
        pts = selection  # type: ignore[assignment]
        out_w, out_h = perspective_output_size(pts)
        output_size = (out_w, out_h)
        print("最终 4 点（排序后：TL, TR, BR, BL）：")
        for name, point in zip(("TL", "TR", "BR", "BL"), pts):
            print(f"  {name}: ({point[0]:.1f}, {point[1]:.1f})")
        print(f"输出尺寸：{out_w}x{out_h}")

    output_path = args.output or default_output_path(video_path, selection_kind, output_size)
    if os.path.abspath(output_path) == os.path.abspath(video_path):
        raise SystemExit("输出路径不能与输入视频相同。")

    if selection_kind == "rect":
        written = crop_video_rect(
            cv2=cv2,
            video_path=video_path,
            output_path=output_path,
            rect=selection,  # type: ignore[arg-type]
            codec=codec,
            show_every=args.show_every,
        )
    else:
        written, _ = crop_video_points(
            cv2=cv2,
            video_path=video_path,
            output_path=output_path,
            ordered_pts=selection,  # type: ignore[arg-type]
            codec=codec,
            show_every=args.show_every,
        )

    print(f"完成：写入 {written} 帧 -> {output_path}")
    actual_size = read_video_size(cv2, output_path)
    if actual_size is not None and actual_size != output_size:
        print(
            "警告：输出文件实际分辨率为 "
            f"{actual_size[0]}x{actual_size[1]}，与选区 {output_size[0]}x{output_size[1]} 不一致。"
        )
        print(
            "这通常是因为 mp4v/XVID/MJPG 等编码器会要求偶数宽高。"
            "如需严格保留奇数尺寸，可选择偶数大小的框，或尝试："
            " --codec \"DIB \" -o output.avi（文件会较大）。"
        )
    print("提示：OpenCV 输出不会保留原视频音频。")


if __name__ == "__main__":
    main()
