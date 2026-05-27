#!/usr/bin/env python3
"""
generate_dataset.py
PC에서만 실행 — 물리적 출력 없이 YOLOv8 학습용 합성 데이터셋 자동 생성

사용법:
    pip install opencv-python numpy requests
    python generate_dataset.py

생성 결과: datasets/apriltag/ 폴더에 YOLO 포맷 데이터셋
"""

import cv2
import numpy as np
import os
import random
import urllib.request
from pathlib import Path
import shutil
import math

# ══════════════════════════════════════════════════════════════════
# 설정
# ══════════════════════════════════════════════════════════════════
NUM_IMAGES   = 2000        # 생성할 이미지 총 수 (최소 500, 권장 2000)
IMG_SIZE     = 640         # YOLOv8 입력 크기 (픽셀)
TAG_IDS      = list(range(10))  # 학습할 AprilTag ID 목록 (0~9)
TRAIN_RATIO  = 0.85        # train/val 분리 비율
OUTPUT_DIR   = Path("datasets/apriltag")
TAG_CACHE    = Path("datasets/tag_src")  # 다운받은 원본 PNG 캐시

SEED = 42
random.seed(SEED)
np.random.seed(SEED)

# ══════════════════════════════════════════════════════════════════
# Step 1: AprilTag 원본 이미지 다운로드 (tag36h11 패밀리)
# ══════════════════════════════════════════════════════════════════
def download_tag_images():
    TAG_CACHE.mkdir(parents=True, exist_ok=True)
    base_url = (
        "https://raw.githubusercontent.com/"
        "AprilRobotics/apriltag-imgs/master/tag36h11"
    )
    tag_imgs = {}

    for tid in TAG_IDS:
        fname = f"tag36_11_{tid:05d}.png"
        local = TAG_CACHE / fname

        if not local.exists():
            url = f"{base_url}/{fname}"
            print(f"  다운로드: {fname} ...", end=" ")
            try:
                urllib.request.urlretrieve(url, local)
                print("완료")
            except Exception as e:
                print(f"실패({e}) → 패턴 생성으로 대체")
                _make_fallback_tag(tid, local)

        img = cv2.imread(str(local), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            tag_imgs[tid] = img

    return tag_imgs


def _make_fallback_tag(tid: int, path: Path):
    """다운로드 실패 시 간단한 흑백 격자 패턴으로 대체"""
    size = 80
    img = np.ones((size, size), dtype=np.uint8) * 255
    # 외곽 검정 테두리
    cv2.rectangle(img, (0, 0), (size - 1, size - 1), 0, 8)
    # 내부 무작위 셀 (tid 기반 seed)
    rng = np.random.default_rng(tid)
    for r in range(2, 7):
        for c in range(2, 7):
            if rng.random() > 0.5:
                x1, y1 = c * 10, r * 10
                img[y1:y1 + 10, x1:x1 + 10] = 0
    cv2.imwrite(str(path), img)


# ══════════════════════════════════════════════════════════════════
# Step 2: 복도 벽 배경 생성 (실내 벽 색조)
# ══════════════════════════════════════════════════════════════════
WALL_COLORS_BGR = [
    (220, 220, 215),  # 연한 회백색
    (200, 210, 200),  # 연두빛 회색
    (215, 200, 185),  # 베이지
    (230, 225, 210),  # 아이보리
    (190, 195, 200),  # 푸른빛 회색
    (180, 180, 175),  # 중간 회색
]


def make_corridor_background(h: int, w: int) -> np.ndarray:
    """복도 배경: 단색 벽 + 선택적 그림자 그라디언트"""
    base_color = random.choice(WALL_COLORS_BGR)
    bg = np.full((h, w, 3), base_color, dtype=np.uint8)

    # 밝기 그라디언트 (조명 효과 모사)
    if random.random() > 0.4:
        grad = np.linspace(0.85, 1.05, w, dtype=np.float32)
        grad = np.clip(grad, 0, 1)
        bg = (bg * grad[np.newaxis, :, np.newaxis]).clip(0, 255).astype(np.uint8)

    # 수직 선 (복도 기둥/문 틀 모사)
    if random.random() > 0.6:
        x = random.randint(w // 4, 3 * w // 4)
        cv2.line(bg, (x, 0), (x, h), (160, 160, 155), random.randint(1, 4))

    return bg


# ══════════════════════════════════════════════════════════════════
# Step 3: 태그를 다양한 각도로 warp해서 배경에 붙이기
# ══════════════════════════════════════════════════════════════════
def paste_tag_with_perspective(
    bg: np.ndarray,
    tag_gray: np.ndarray,
) -> tuple[np.ndarray, tuple[float, float, float, float]] | None:
    """
    태그를 임의의 원근 변환(perspective warp) + 크기로 배경에 합성.
    Returns: (합성된 이미지, (cx_norm, cy_norm, w_norm, h_norm)) 또는 None
    """
    H, W = bg.shape[:2]

    # ── 태그 크기 (거리 모사): 이미지 넓이의 5%~30% ──────────────
    tag_base = random.randint(int(W * 0.05), int(W * 0.30))
    tag_gray_resized = cv2.resize(tag_gray, (tag_base, tag_base))

    # ── 3채널 변환 + 흰색 여백 추가 (실물 인쇄 여백 모사) ─────────
    margin = max(4, tag_base // 8)
    tag_padded_size = tag_base + 2 * margin
    tag_padded = np.ones((tag_padded_size, tag_padded_size), dtype=np.uint8) * 255
    tag_padded[margin:margin + tag_base, margin:margin + tag_base] = tag_gray_resized
    tag_color = cv2.cvtColor(tag_padded, cv2.COLOR_GRAY2BGR)
    th, tw = tag_color.shape[:2]

    # ── 원근 변환 파라미터 ────────────────────────────────────────
    # 복도 주행 중 옆에서 보이는 각도 모사: 수평 기울기 0~75°
    horiz_angle = random.uniform(0, 75)   # 수평 비틀림 (가장 중요)
    vert_angle  = random.uniform(-15, 15) # 수직 비틀림 (약간)

    # 원근 변환 행렬 계산
    pts_src = np.float32([
        [0,  0],
        [tw, 0],
        [tw, th],
        [0,  th]
    ])

    # 수평 기울기: 오른쪽(또는 왼쪽)에서 보는 것처럼
    horiz_shrink = math.cos(math.radians(horiz_angle))
    vert_shrink  = math.cos(math.radians(abs(vert_angle)))

    side = random.choice([-1, 1])  # 왼쪽 or 오른쪽에서 보기
    if side == 1:  # 오른쪽에서 보기 → 왼쪽이 좁아짐
        new_w_left  = max(4, int(tw * horiz_shrink))
        new_w_right = tw
    else:          # 왼쪽에서 보기 → 오른쪽이 좁아짐
        new_w_left  = tw
        new_w_right = max(4, int(tw * horiz_shrink))

    top_shift    = int(th * (1 - vert_shrink) / 2) if vert_angle < 0 else 0
    bottom_shift = int(th * (1 - vert_shrink) / 2) if vert_angle > 0 else 0

    pts_dst = np.float32([
        [tw - new_w_left,  top_shift],
        [new_w_right,      top_shift],
        [new_w_right,      th - bottom_shift],
        [tw - new_w_left,  th - bottom_shift],
    ])

    # x 좌표가 음수가 되지 않도록 shift
    min_x = min(pts_dst[:, 0])
    if min_x < 0:
        pts_dst[:, 0] -= min_x
    out_w = int(pts_dst[:, 0].max()) + 1
    out_h = int(pts_dst[:, 1].max()) + 1

    M = cv2.getPerspectiveTransform(pts_src, pts_dst)
    warped = cv2.warpPerspective(
        tag_color, M, (out_w, out_h),
        flags=cv2.INTER_LINEAR,
        borderValue=(255, 255, 255)
    )

    wh, ww = warped.shape[:2]
    if wh == 0 or ww == 0:
        return None

    # ── 배경 위 무작위 위치에 붙이기 ──────────────────────────────
    max_x = W - ww
    max_y = H - wh
    if max_x <= 0 or max_y <= 0:
        return None

    px = random.randint(0, max_x)
    py = random.randint(0, max_y)

    result = bg.copy()
    roi = result[py:py + wh, px:px + ww]

    # 태그 영역(흰 배경 제외)을 알파 마스크로 합성
    gray_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_warped, 250, 255, cv2.THRESH_BINARY_INV)
    mask_inv = cv2.bitwise_not(mask)

    bg_part  = cv2.bitwise_and(roi,    roi,    mask=mask_inv)
    tag_part = cv2.bitwise_and(warped, warped, mask=mask)
    combined = cv2.add(bg_part, tag_part)
    result[py:py + wh, px:px + ww] = combined

    # ── YOLO bbox 계산 (정규화) ────────────────────────────────────
    cx = (px + ww / 2) / W
    cy = (py + wh / 2) / H
    bw = ww / W
    bh = wh / H

    return result, (cx, cy, bw, bh)


# ══════════════════════════════════════════════════════════════════
# Step 4: 노이즈 / 블러 어그멘테이션
# ══════════════════════════════════════════════════════════════════
def augment(img: np.ndarray) -> np.ndarray:
    # 밝기/대비 조정
    alpha = random.uniform(0.7, 1.3)   # 대비
    beta  = random.randint(-30, 30)    # 밝기
    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    # 가우시안 블러 (카메라 포커스 부족 모사)
    if random.random() > 0.5:
        k = random.choice([3, 5])
        img = cv2.GaussianBlur(img, (k, k), 0)

    # 가우시안 노이즈
    if random.random() > 0.5:
        noise = np.random.normal(0, random.uniform(5, 20), img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    return img


# ══════════════════════════════════════════════════════════════════
# Step 5: 메인 생성 루프
# ══════════════════════════════════════════════════════════════════
def setup_dirs():
    for split in ["train", "val"]:
        (OUTPUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)


def write_yaml():
    yaml_content = f"""# AprilTag YOLOv8 데이터셋
path: {OUTPUT_DIR.resolve().as_posix()}
train: images/train
val:   images/val

nc: 1
names:
  0: apriltag
"""
    (OUTPUT_DIR / "apriltag.yaml").write_text(yaml_content, encoding="utf-8")
    print(f"  YAML 저장: {OUTPUT_DIR / 'apriltag.yaml'}")


def generate():
    print("=" * 60)
    print(" AprilTag 합성 데이터셋 생성 시작")
    print(f" 목표: {NUM_IMAGES}장, 태그 ID: {TAG_IDS}")
    print("=" * 60)

    print("\n[1/4] 출력 폴더 생성...")
    setup_dirs()

    print("\n[2/4] AprilTag 원본 이미지 준비...")
    tag_imgs = download_tag_images()
    if not tag_imgs:
        raise RuntimeError("태그 이미지를 하나도 로드하지 못했습니다.")
    print(f"  사용 가능한 태그: {list(tag_imgs.keys())}")

    print(f"\n[3/4] 합성 이미지 {NUM_IMAGES}장 생성 중...")
    n_train = int(NUM_IMAGES * TRAIN_RATIO)
    generated = 0
    skip = 0

    for i in range(NUM_IMAGES):
        split = "train" if i < n_train else "val"
        tid   = random.choice(list(tag_imgs.keys()))
        bg    = make_corridor_background(IMG_SIZE, IMG_SIZE)

        # 배경에만 태그를 붙이는 경우 10% (Hard negative 역할)
        if random.random() < 0.10:
            img_out = augment(bg)
            label   = ""   # no object
        else:
            result = paste_tag_with_perspective(bg, tag_imgs[tid])
            if result is None:
                skip += 1
                continue
            img_out, (cx, cy, bw, bh) = result
            img_out = augment(img_out)
            label = f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n"

        fname = f"syn_{i:05d}"
        cv2.imwrite(
            str(OUTPUT_DIR / "images" / split / f"{fname}.jpg"),
            img_out,
            [cv2.IMWRITE_JPEG_QUALITY, 92]
        )
        with open(OUTPUT_DIR / "labels" / split / f"{fname}.txt", "w") as f:
            f.write(label)

        generated += 1
        if (i + 1) % 200 == 0:
            print(f"  {i + 1}/{NUM_IMAGES} 완료...")

    print(f"\n  생성 완료: {generated}장 (스킵: {skip}장)")
    print(f"  train: {n_train}장  /  val: {NUM_IMAGES - n_train}장")

    print("\n[4/4] YAML 파일 작성...")
    write_yaml()

    print("\n" + "=" * 60)
    print(" 데이터셋 생성 완료!")
    print(f" 경로: {OUTPUT_DIR.resolve()}")
    print("=" * 60)
    print("\n다음 단계:")
    print("  python train_yolo.py")
    print("=" * 60)


if __name__ == "__main__":
    generate()
