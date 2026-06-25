#!/usr/bin/env python3
"""웹 대시보드(http://localhost:8090) 매장 지도 PNG 생성.

1206_sim_1.pgm(벽/장애물) + 2026_amr_layout.json(테이블·문·은폐처)을 합쳐
storagy_llm/web/1206_top.png 를 만든다. app.js 좌표 보정과 동일한 해상도를 쓴다.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

MAP_PGM = PKG_ROOT / 'src/storagy/map/1206_sim_1.pgm'
LAYOUT_JSON = PKG_ROOT / 'src/storagy/worlds/2026_amr_layout.json'
OUT_PNG = PKG_ROOT / 'src/storagy_llm/web/1206_top.png'

# app.js MAP 상수와 동일
RES = 0.05
ORIGIN_X = -5.0
ORIGIN_Y = -3.8
PGM_W = 200
PGM_H = 150
IMG_W = 858
IMG_H = 645

# style.css 팔레트 (BGR)
C_FREE = (238, 245, 250)       # #faf5ee
C_UNKNOWN = (215, 228, 237)    # #ede4d7
C_WALL = (41, 50, 61)          # #3d3229
C_TABLE = (24, 90, 139)        # brown table top
C_TABLE_EDGE = (18, 64, 100)
C_CHAIR = (66, 112, 58)        # green-ish chairs
C_DOOR = (37, 191, 64)         # green door frame
C_HIDE = (54, 54, 58)          # hideout cabinet
C_ORIGIN = (81, 154, 78)       # spawn zone


def world_to_img(x: float, y: float) -> tuple[int, int]:
    px = int(round((x - ORIGIN_X) / RES * (IMG_W / PGM_W)))
    py = int(round(IMG_H - (y - ORIGIN_Y) / RES * (IMG_H / PGM_H)))
    return px, py


def desk_rect(x: float, y: float, sx: float, sy: float) -> np.ndarray:
    hx, hy = sx / 2.0, sy / 2.0
    corners = [
        world_to_img(x - hx, y - hy),
        world_to_img(x + hx, y - hy),
        world_to_img(x + hx, y + hy),
        world_to_img(x - hx, y + hy),
    ]
    return np.array(corners, dtype=np.int32)


def draw_label(img: np.ndarray, text: str, x: float, y: float) -> None:
    px, py = world_to_img(x, y)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thick = 2
    (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
    cv2.putText(
        img, text, (px - tw // 2, py + th // 2), font, scale,
        (255, 255, 255), thick + 2, cv2.LINE_AA)
    cv2.putText(
        img, text, (px - tw // 2, py + th // 2), font, scale,
        C_WALL, thick, cv2.LINE_AA)


def pgm_to_topview(pgm: np.ndarray) -> np.ndarray:
    """Occupancy PGM → 대시보드용 컬러 탑뷰."""
    img = np.zeros((PGM_H, PGM_W, 3), dtype=np.uint8)
    img[pgm >= 250] = C_WALL
    img[(pgm > 0) & (pgm < 250)] = C_UNKNOWN
    img[pgm == 0] = C_FREE
    return cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_NEAREST)


def generate(layout: dict) -> np.ndarray:
    pgm = cv2.imread(str(MAP_PGM), cv2.IMREAD_GRAYSCALE)
    if pgm is None:
        raise FileNotFoundError(MAP_PGM)

    img = pgm_to_topview(pgm)

    # 테이블 + 의자
    for desk in layout.get('desks', []):
        cv2.fillPoly(img, [desk_rect(desk['x'], desk['y'], desk['size_x'], desk['size_y'])], C_TABLE)
        cv2.polylines(img, [desk_rect(desk['x'], desk['y'], desk['size_x'], desk['size_y'])],
                      True, C_TABLE_EDGE, 2, cv2.LINE_AA)
        draw_label(img, desk.get('label', desk['name']), desk['x'], desk['y'])
        for chair in desk.get('chairs', []):
            cx, cy = world_to_img(chair['x'], chair['y'])
            cv2.circle(img, (cx, cy), 5, C_CHAIR, -1, cv2.LINE_AA)

    # 진입문
    door = layout['entry_door']
    dw = door.get('width_m', 0.85)
    p1 = world_to_img(door['x'], door['y'] - dw / 2)
    p2 = world_to_img(door['x'], door['y'] + dw / 2)
    cv2.line(img, p1, p2, C_DOOR, 6, cv2.LINE_AA)
    draw_label(img, 'DOOR', door['x'] - 0.35, door['y'])

    # 은폐처
    hide = layout['hideout']
    hx, hy = hide['x'], hide['y']
    cv2.fillPoly(img, [desk_rect(hx, hy, 0.30, 1.20)], C_HIDE)
    draw_label(img, 'HIDE', hx, hy)

    # 로봇 스폰(origin)
    origin = layout['origin']
    ox, oy = origin['x'], origin['y']
    cv2.circle(img, world_to_img(ox, oy), 10, C_ORIGIN, 2, cv2.LINE_AA)
    draw_label(img, 'SPAWN', ox, oy - 0.35)

    return img


def main() -> int:
    if not LAYOUT_JSON.is_file():
        print(f'ERROR: layout not found: {LAYOUT_JSON}', file=sys.stderr)
        return 1
    layout = json.loads(LAYOUT_JSON.read_text(encoding='utf-8'))
    img = generate(layout)
    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT_PNG), img)
    print(f'Dashboard map: {OUT_PNG} ({IMG_W}x{IMG_H})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
