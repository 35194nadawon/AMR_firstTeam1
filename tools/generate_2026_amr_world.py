#!/usr/bin/env python3
"""Gazebo 월드 생성: 1206_2.dae 밑판 + 1206_sim_1 맵 기반 책상 4개(T1~T4) + 진입문.

Nav2 static map(1206_sim_1.pgm)에 layout과 동일한 테이블·의자·hideout footprint를 stamp.

사용:
  python3 tools/generate_2026_amr_world.py
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
MAP_1206 = PKG_ROOT / 'src/storagy/map/1206_sim_1.pgm'
WALLS_PGM = PKG_ROOT / 'src/storagy/map/1206_sim_1_walls.pgm'
DEFAULT_META = PKG_ROOT / 'src/storagy/worlds/2026_amr_layout.json'
DEFAULT_SDF = PKG_ROOT / 'src/storagy/worlds/2026_amr.sdf'
POINTS_YAML = PKG_ROOT / 'src/storagy_llm/params/points.yaml'
SRC_DAE = PKG_ROOT / 'src/storagy/meshes/1206_2.dae'
SIM_DAE = PKG_ROOT / 'src/storagy/meshes/1206_2_sim.dae'
DAE_NS = 'http://www.collada.org/2005/11/COLLADASchema'

# 1206_2.dae 중앙 소형 테이블 2개(Cube_016/017) — brown box T2/T3와 중복
DAE_REMOVE_NODES = {'Cube_016', 'Cube_017'}
DAE_REMOVE_GEOMS = {'Cube_028-mesh', 'Cube_029-mesh'}

RES = 0.05
ORIGIN_1206 = (-5.0, -3.8)
OCCUPIED = 0
FREE = 254

# hideout_cabinet SDF collision box
HIDEOUT_SIZE_X = 0.30
HIDEOUT_SIZE_Y = 1.20

ROOM_1206 = {'x': [-4.8, 4.8], 'y': [-3.5, 3.5]}
HIDEOUT_1206 = {'x': -4.55, 'y': -3.0}
ENV_MESH_POSE = (-4.65, -3.80, 0.5)

DESK_H = 0.75
DESK_DEPTH = 0.65

CHAIR_SEAT = 0.42
CHAIR_SEAT_H = 0.04
CHAIR_BACK_H = 0.38
CHAIR_LEG_H = 0.43
CHAIR_GAP = 0.12

# T1 -x면: 진입문(-4.47, 0.12) 쪽 통로 — 의자 2개 배치 안 함
CHAIR_SKIP_SIDES = {'t1': {'nx'}}

# T2/T3 동쪽 통로를 북↔남으로 배회 (Nav2 /scan 장애물로 회피) — waypoints는 layout에서 계산
# YOLO 오탐·배회 actor 간섭을 줄이려면 기본 off. 필요 시 --walking-person
ENABLE_WALKING_PERSON = False

WALKING_PERSON = {
    'name': 'walking_person',
    'skin': 'walk.dae',
    'z': 0.875,
    'delay_start': 5.0,
}


def walking_person_waypoints(layout: dict) -> list[tuple[float, float, float, float]]:
    """T2/T3 +x(의자) 동쪽 통로에서 남↔북 왕복."""
    by_name = {d['name']: d for d in layout['desks']}
    t2, t3 = by_name['t2'], by_name['t3']
    px_chairs = [
        c['x'] for d in (t2, t3)
        for c in d.get('chairs', []) if '_px' in c['id']
    ]
    walk_x = round(max(px_chairs) + 0.15, 2)
    y_south = round(t3['y'] - t3['size_y'] / 2 + 0.40, 2)
    y_north = round(t2['y'] + t2['size_y'] / 2 - 0.40, 2)
    return [
        (0.0, walk_x, y_south, math.pi / 2),
        (35.0, walk_x, y_north, math.pi / 2),
        (37.0, walk_x, y_north, -math.pi / 2),
        (72.0, walk_x, y_south, -math.pi / 2),
        (74.0, walk_x, y_south, math.pi / 2),
    ]

# 청사진 기준 T1~T4 배치 (origin/dev_hide Gazebo 월드와 동일)
BASE_BLUEPRINT_DESKS = [
    {'name': 't1', 'label': 'T1', 'x': -3.20, 'y': 0.45, 'size_x': 0.65, 'size_y': 2.20},
    {'name': 't2', 'label': 'T2', 'x': -1.00, 'y': 1.25, 'size_x': 0.65, 'size_y': 1.80},
    {'name': 't3', 'label': 'T3', 'x': -1.00, 'y': -0.90, 'size_x': 0.65, 'size_y': 1.80},
    {'name': 't4', 'label': 'T4', 'x': 3.20, 'y': -0.50, 'size_x': 0.65, 'size_y': 2.20},
]

# 세로 길이·두께 동일 비율 확대 (T2/T3 간격은 겹침 없이 자동 벌림)
TABLE_LENGTH_SCALE = 0.95
TABLE_PAIR_EDGE_GAP = 1.40   # T2–T3 가장자리 간격


def _pair_half_sep(size_y_t2: float, size_y_t3: float) -> float:
    return ((size_y_t2 + size_y_t3) / 2.0 + TABLE_PAIR_EDGE_GAP) / 2.0

CHAIR_COLORS = {
    't1': (0.55, 0.68, 0.38),
    't2': (0.78, 0.72, 0.48),
    't3': (0.82, 0.62, 0.58),
    't4': (0.58, 0.70, 0.42),
}

# T1~T4 검색 영역 (--from-map 옵션용, 맵 윤곽선 자동 추정)
TABLE_REGIONS = {
    'T1': {'x': (-3.5, -2.2), 'y': (0.5, 2.5)},
    'T2': {'x': (-0.5, 2.5), 'y': (2.8, 3.5)},
    'T3': {'x': (0.5, 2.8), 'y': (-3.6, -2.5)},
    'T4': {'x': (3.5, 4.9), 'y': (-1.8, 0.5)},
}


def ensure_sim_dae() -> str:
    """DAE에서 중앙 소형 테이블 2개 제거한 1206_2_sim.dae 생성."""
    ET.register_namespace('', DAE_NS)
    tree = ET.parse(SRC_DAE)
    root = tree.getroot()

    scene = root.find(f'.//{{{DAE_NS}}}visual_scene')
    if scene is not None:
        for node in list(scene.findall(f'{{{DAE_NS}}}node')):
            if node.get('id') in DAE_REMOVE_NODES:
                scene.remove(node)

    lib_geom = root.find(f'.//{{{DAE_NS}}}library_geometries')
    if lib_geom is not None:
        for geom in list(lib_geom.findall(f'{{{DAE_NS}}}geometry')):
            if geom.get('id') in DAE_REMOVE_GEOMS:
                lib_geom.remove(geom)

    SIM_DAE.parent.mkdir(parents=True, exist_ok=True)
    tree.write(SIM_DAE, encoding='utf-8', xml_declaration=True)
    return '1206_2_sim.dae'


def chairs_for_desk(spec: dict) -> list[dict]:
    """세로 테이블 가로면(±x)에만 의자 2개씩 — 긴 변(y) 방향 나란히."""
    tx, ty = spec['x'], spec['y']
    hw = spec['size_x'] / 2.0
    hl = spec['size_y'] / 2.0
    dist = CHAIR_GAP + CHAIR_SEAT / 2.0
    along = max(hl - CHAIR_SEAT / 2 - 0.08, hl * 0.45)
    sides = (
        ('px', tx + hw + dist, math.pi),
        ('nx', tx - hw - dist, 0.0),
    )
    chairs: list[dict] = []
    skip = CHAIR_SKIP_SIDES.get(spec['name'], set())
    for prefix, cx, yaw in sides:
        if prefix in skip:
            continue
        for i, y_off in enumerate((along, -along), start=1):
            chairs.append({
                'id': f"{spec['name']}_{prefix}{i}",
                'x': round(cx, 2),
                'y': round(ty + y_off, 2),
                'yaw_rad': round(yaw, 4),
            })
    return chairs


def px2world(px: float, py: float, height: int) -> tuple[float, float]:
    wx = ORIGIN_1206[0] + px * RES
    wy = ORIGIN_1206[1] + (height - py) * RES
    return wx, wy


def world2px(wx: float, wy: float, height: int) -> tuple[int, int]:
    px = int(round((wx - ORIGIN_1206[0]) / RES))
    py = int(round(height - (wy - ORIGIN_1206[1]) / RES))
    return px, py


def stamp_rect(
    img: np.ndarray,
    cx: float,
    cy: float,
    sx: float,
    sy: float,
    yaw: float = 0.0,
    value: int = OCCUPIED,
) -> None:
    """월드 좌표 축정렬/회전 사각형을 PGM에 stamp."""
    h, _w = img.shape
    px, py = world2px(cx, cy, h)
    bw = max(1, int(round(sx / RES)))
    bh = max(1, int(round(sy / RES)))
    angle_deg = -math.degrees(yaw)
    box = cv2.boxPoints(((px, py), (bw, bh), angle_deg)).astype(np.int32)
    cv2.fillConvexPoly(img, box, int(value))


def _clear_region_world(
    img: np.ndarray,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    value: int = FREE,
) -> None:
    h, w = img.shape
    for py in range(h):
        for px in range(w):
            wx, wy = px2world(px, py, h)
            if x0 <= wx <= x1 and y0 <= wy <= y1:
                img[py, px] = value


def _clear_spawn_corridor(img: np.ndarray, layout: dict) -> None:
    """진입문~스폰 왼쪽 legacy 벽 돌출(원형 inflation) 제거 — hideout 위쪽 통로만."""
    door = layout['entry_door']
    origin = layout['origin']
    dw = door.get('width_m', 0.85)
    hide = layout['hideout']
    hide_top = hide['y'] + HIDEOUT_SIZE_Y / 2 + 0.05

    _clear_region_world(
        img, door['x'] - 0.55, door['x'] + 0.15,
        door['y'] - dw / 2 - 0.15, door['y'] + dw / 2 + 0.15,
    )
    _clear_region_world(
        img, origin['x'] - 0.55, origin['x'] + 0.55,
        origin['y'] - 0.55, origin['y'] + 0.55,
    )
    # 왼쪽 벽면 legacy mesh 돌출 — RViz 하단 좌측 원형 장애물
    _clear_region_world(
        img, -4.95, -3.85, hide_top, origin['y'] + 0.55,
    )
    # 왼쪽 하단 legacy 원형 노이즈 4점 (1206 mesh artifact)
    _clear_region_world(img, -4.05, -3.05, -2.75, -1.85)


def _clear_main_corridor(img: np.ndarray, layout: dict) -> None:
    """메인 동서 통로(y≈origin) legacy mesh occupied 제거."""
    oy = layout['origin']['y']
    room = layout['base']['room_1206']
    _clear_region_world(img, -4.95, room['x'][1] - 0.2, oy - 0.38, oy + 0.38)


def _unknown_to_free_in_room(img: np.ndarray, layout: dict, margin: float = 0.2) -> None:
    """실내 unknown(205) → free — RViz 청록 노이즈·costmap unknown 제거."""
    room = layout['base']['room_1206']
    h, w = img.shape
    x0, x1 = room['x'][0] + margin, room['x'][1] - margin
    y0, y1 = room['y'][0] + margin, room['y'][1] - margin
    for py in range(h):
        for px in range(w):
            wx, wy = px2world(px, py, h)
            if x0 <= wx <= x1 and y0 <= wy <= y1 and img[py, px] == 205:
                img[py, px] = FREE


def denoise_static_map(img: np.ndarray, layout: dict, min_blob: int = 8) -> None:
    """고립된 소형 occupied/unknown blob 제거 (벽 대형 blob은 유지)."""
    _unknown_to_free_in_room(img, layout)
    combined = np.isin(img, [0, 205]).astype(np.uint8)
    num, labels, stats, _ = cv2.connectedComponentsWithStats(combined, 8)
    for i in range(1, num):
        if stats[i, cv2.CC_STAT_AREA] < min_blob:
            img[labels == i] = FREE


def refresh_walls_base(layout: dict, map_path: Path = MAP_1206) -> np.ndarray:
    """walls.pgm 재생성 — 통로 clearing + denoise."""
    if WALLS_PGM.is_file():
        walls = cv2.imread(str(WALLS_PGM), cv2.IMREAD_GRAYSCALE)
    else:
        src = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
        if src is None:
            raise RuntimeError(f'cannot read {map_path}')
        walls = extract_walls_base(src, layout)
    _clear_main_corridor(walls, layout)
    _clear_spawn_corridor(walls, layout)
    denoise_static_map(walls, layout)
    WALLS_PGM.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(WALLS_PGM), walls)
    return walls


def extract_walls_base(img: np.ndarray, layout: dict) -> np.ndarray:
    """벽만 남긴 PGM — legacy 테이블 blob·가구 footprint 제거."""
    walls = img.copy()
    for reg in TABLE_REGIONS.values():
        _clear_region_world(walls, reg['x'][0], reg['x'][1], reg['y'][0], reg['y'][1])
    pad = 0.12
    for desk in layout['desks']:
        stamp_rect(
            walls, desk['x'], desk['y'],
            desk['size_x'] + pad * 2, desk['size_y'] + pad * 2,
            desk.get('yaw_rad', 0.0), FREE)
        for chair in desk.get('chairs', []):
            stamp_rect(
                walls, chair['x'], chair['y'],
                CHAIR_SEAT + pad * 2, CHAIR_SEAT + pad * 2,
                chair.get('yaw_rad', 0.0), FREE)
    hide = layout['hideout']
    stamp_rect(
        walls, hide['x'], hide['y'],
        HIDEOUT_SIZE_X + pad * 2, HIDEOUT_SIZE_Y + pad * 2, 0.0, FREE)
    _clear_spawn_corridor(walls, layout)
    _clear_main_corridor(walls, layout)
    return walls


def sync_nav_map(walls: np.ndarray, layout: dict) -> np.ndarray:
    """가제보 layout → Nav2 static map (벽 + 테이블·의자·hideout)."""
    out = walls.copy()
    _clear_spawn_corridor(out, layout)
    _clear_main_corridor(out, layout)

    hide = layout['hideout']
    stamp_rect(out, hide['x'], hide['y'], HIDEOUT_SIZE_X, HIDEOUT_SIZE_Y, 0.0, OCCUPIED)

    for desk in layout['desks']:
        stamp_rect(
            out, desk['x'], desk['y'],
            desk['size_x'], desk['size_y'],
            desk.get('yaw_rad', 0.0), OCCUPIED)
        for chair in desk.get('chairs', []):
            stamp_rect(
                out, chair['x'], chair['y'],
                CHAIR_SEAT, CHAIR_SEAT,
                chair.get('yaw_rad', 0.0), OCCUPIED)

    denoise_static_map(out, layout)
    return out


def write_nav_map(walls: np.ndarray, layout: dict, out_path: Path) -> None:
    stamped = sync_nav_map(walls, layout)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), stamped):
        raise RuntimeError(f'PGM write failed: {out_path}')


def ensure_walls_base(source: np.ndarray, layout: dict) -> np.ndarray:
    return refresh_walls_base(layout, MAP_1206)


def detect_entry_door(img: np.ndarray) -> dict:
    """왼쪽 벽 진입문 — free 공간 중 좌측(x<-4.25) 중앙대."""
    h, _ = img.shape
    free = img == 254
    fy, fx = np.where(free)
    wx = ORIGIN_1206[0] + fx * RES
    wy = ORIGIN_1206[1] + (h - fy) * RES
    mask = (wx < -4.25) & (wx > -4.75) & (wy > -1.5) & (wy < 2.0)
    if mask.sum() < 10:
        mask = (wx < -4.2) & (wy > -0.8) & (wy < 1.2)
    cx = float(wx[mask].mean())
    cy = float(wy[mask].mean())
    return {
        'name': 'entry_door',
        'x': round(cx, 2),
        'y': round(cy, 2),
        'width_m': 0.85,
        'yaw_rad': 1.5708,
    }


def _largest_component_bbox(region_mask: np.ndarray) -> tuple[float, float, float, float]:
    """영역 마스크에서 가장 큰 connected component의 축정렬 bbox."""
    n, _labels, stats, centroids = cv2.connectedComponentsWithStats(
        region_mask.astype(np.uint8), connectivity=8,
    )
    if n <= 1:
        raise RuntimeError('occupied connected component를 찾지 못했습니다.')
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    px, py, pw, ph, _area = stats[idx]
    cx, cy = centroids[idx]
    h = region_mask.shape[0]
    wx_c = ORIGIN_1206[0] + cx * RES
    wy_c = ORIGIN_1206[1] + (h - cy) * RES
    xspan = pw * RES
    yspan = ph * RES
    return float(wx_c), float(wy_c), float(xspan), float(yspan)


def scaled_blueprint_specs(scale: float = TABLE_LENGTH_SCALE) -> list[dict]:
    """비율 유지 확대 + T2/T3를 T1–T4 중간에 대칭 배치."""
    by_name = {d['name']: dict(d) for d in BASE_BLUEPRINT_DESKS}
    for spec in by_name.values():
        spec['size_x'] = round(spec['size_x'] * scale, 2)
        spec['size_y'] = round(spec['size_y'] * scale, 2)

    t1, t2, t3, t4 = by_name['t1'], by_name['t2'], by_name['t3'], by_name['t4']
    center_x = (t1['x'] + t4['x']) / 2.0
    center_y = (t1['y'] + t4['y']) / 2.0
    half_sep = _pair_half_sep(t2['size_y'], t3['size_y'])

    t2['x'] = t3['x'] = round(center_x, 2)
    t2['y'] = round(center_y + half_sep, 2)
    t3['y'] = round(center_y - half_sep, 2)
    return list(by_name.values())


def blueprint_desks(scale: float = TABLE_LENGTH_SCALE) -> list[dict]:
    """청사진 주석 기준 T1~T4 (세로 배치, 비율 확대)."""
    desks: list[dict] = []
    for spec in scaled_blueprint_specs(scale):
        chairs = chairs_for_desk(spec)
        desks.append({
            'name': spec['name'],
            'label': spec['label'],
            'x': spec['x'],
            'y': spec['y'],
            'size_x': spec['size_x'],
            'size_y': spec['size_y'],
            'size_z': DESK_H,
            'yaw_rad': 0.0,
            'orientation': 'vertical',
            'chairs': chairs,
        })
    return desks


def detect_tables(img: np.ndarray) -> list[dict]:
    """1206 맵 occupied 윤곽선에서 T1~T4 중심·크기·방향(최대 CC bbox) 추정."""
    h, w = img.shape
    occ = img == 0

    desks: list[dict] = []
    for name, reg in TABLE_REGIONS.items():
        x0, x1 = reg['x']
        y0, y1 = reg['y']
        region_mask = np.zeros((h, w), dtype=np.uint8)
        for py in range(h):
            for px in range(w):
                if not occ[py, px]:
                    continue
                wx, wy = px2world(px, py, h)
                if x0 <= wx <= x1 and y0 <= wy <= y1:
                    region_mask[py, px] = 1
        if region_mask.sum() < 5:
            raise RuntimeError(f'{name} 영역에서 occupied 픽셀을 찾지 못했습니다.')

        cx, cy, xspan, yspan = _largest_component_bbox(region_mask)

        if xspan >= yspan:
            size_x = round(xspan, 2)
            size_y = round(max(yspan, DESK_DEPTH), 2)
            orientation = 'horizontal'
        else:
            size_x = round(max(xspan, DESK_DEPTH), 2)
            size_y = round(yspan, 2)
            orientation = 'vertical'

        desks.append({
            'name': name.lower(),
            'label': name,
            'x': round(cx, 2),
            'y': round(cy, 2),
            'size_x': size_x,
            'size_y': size_y,
            'size_z': DESK_H,
            'yaw_rad': 0.0,
            'orientation': orientation,
        })
    return desks


def build_layout(img: np.ndarray, *, from_map: bool = False,
                 table_scale: float = TABLE_LENGTH_SCALE) -> dict:
    door = detect_entry_door(img)
    mesh_uri = ensure_sim_dae()
    desks = detect_tables(img) if from_map else blueprint_desks(table_scale)
    return {
        'base': {
            'mesh': mesh_uri,
            'mesh_uri': mesh_uri,
            'model': 'environment_1206_2',
            'pose': list(ENV_MESH_POSE),
            'nav_map': '1206_sim_1.yaml',
            'room_1206': ROOM_1206,
        },
        'entry_door': door,
        'origin': {
            'x': round(door['x'] + 0.55, 2),
            'y': door['y'],
            'qz': 0.0,
            'qw': 1.0,
        },
        'hideout': {
            'x': HIDEOUT_1206['x'],
            'y': HIDEOUT_1206['y'],
            'qz': 0.0,
            'qw': 1.0,
        },
        'desks': desks,
    }


def chair_model_sdf(desk: dict, chair: dict) -> str:
    """간단한 의자(좌판+등받이+4다리)."""
    x, y, yaw = chair['x'], chair['y'], chair['yaw_rad']
    cr, cg, cb = CHAIR_COLORS.get(desk['name'], (0.6, 0.6, 0.6))
    leg = 0.04
    half = CHAIR_SEAT / 2
    leg_z = CHAIR_LEG_H / 2
    seat_z = CHAIR_LEG_H + CHAIR_SEAT_H / 2
    back_z = CHAIR_LEG_H + CHAIR_BACK_H / 2
    return f"""    <model name="chair_{chair['id']}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="seat_col">
          <pose>0 0 {seat_z:.3f} 0 0 0</pose>
          <geometry><box><size>{CHAIR_SEAT} {CHAIR_SEAT} {CHAIR_SEAT_H}</size></box></geometry>
        </collision>
        <collision name="back_col">
          <pose>{-half + 0.03:.3f} 0 {back_z:.3f} 0 0 0</pose>
          <geometry><box><size>0.05 {CHAIR_SEAT} {CHAIR_BACK_H}</size></box></geometry>
        </collision>
        <visual name="seat">
          <pose>0 0 {seat_z:.3f} 0 0 0</pose>
          <geometry><box><size>{CHAIR_SEAT} {CHAIR_SEAT} {CHAIR_SEAT_H}</size></box></geometry>
          <material><ambient>{cr*0.6:.2f} {cg*0.6:.2f} {cb*0.6:.2f} 1</ambient><diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1</diffuse></material>
        </visual>
        <visual name="back">
          <pose>{-half + 0.03:.3f} 0 {back_z:.3f} 0 0 0</pose>
          <geometry><box><size>0.05 {CHAIR_SEAT} {CHAIR_BACK_H}</size></box></geometry>
          <material><ambient>{cr*0.5:.2f} {cg*0.5:.2f} {cb*0.5:.2f} 1</ambient><diffuse>{cr*0.85:.2f} {cg*0.85:.2f} {cb*0.85:.2f} 1</diffuse></material>
        </visual>
        <visual name="leg_fl"><pose>{half - leg:.3f} {half - leg:.3f} {leg_z:.3f} 0 0 0</pose><geometry><box><size>{leg} {leg} {CHAIR_LEG_H}</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual>
        <visual name="leg_fr"><pose>{half - leg:.3f} {-half + leg:.3f} {leg_z:.3f} 0 0 0</pose><geometry><box><size>{leg} {leg} {CHAIR_LEG_H}</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual>
        <visual name="leg_bl"><pose>{-half + leg:.3f} {half - leg:.3f} {leg_z:.3f} 0 0 0</pose><geometry><box><size>{leg} {leg} {CHAIR_LEG_H}</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual>
        <visual name="leg_br"><pose>{-half + leg:.3f} {-half + leg:.3f} {leg_z:.3f} 0 0 0</pose><geometry><box><size>{leg} {leg} {CHAIR_LEG_H}</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material></visual>
      </link>
    </model>
"""


def desk_model_sdf(desk: dict) -> str:
    x, y = desk['x'], desk['y']
    sx, sy, sz = desk['size_x'], desk['size_y'], desk['size_z']
    zc = sz / 2.0
    return f"""    <model name="table_{desk['name']}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <pose>0 0 {zc:.3f} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="top">
          <pose>0 0 {sz - 0.02:.3f} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} 0.04</size></box></geometry>
          <material><ambient>0.30 0.18 0.10 1</ambient><diffuse>0.55 0.35 0.18 1</diffuse></material>
        </visual>
        <visual name="body">
          <pose>0 0 {zc:.3f} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz - 0.04:.2f}</size></box></geometry>
          <material><ambient>0.25 0.15 0.08 1</ambient><diffuse>0.40 0.25 0.12 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def desk_and_chairs_sdf(desk: dict) -> str:
    parts = [desk_model_sdf(desk)]
    for chair in desk.get('chairs', []):
        parts.append(chair_model_sdf(desk, chair))
    return ''.join(parts)


def walking_person_actor_sdf(layout: dict) -> str:
    """Gazebo actor — walk.dae 스킨, 충돌 박스는 라이다(/scan)에 잡혀 Nav2가 회피."""
    wp = WALKING_PERSON
    waypoints = '\n'.join(
        f'''          <waypoint>
            <time>{t:.1f}</time>
            <pose>{x:.2f} {y:.2f} {wp['z']:.3f} 0 0 {yaw:.4f}</pose>
          </waypoint>'''
        for t, x, y, yaw in walking_person_waypoints(layout)
    )
    return f'''
    <!-- 배회하는 사람 (로봇 라이다 → Nav2 장애물 회피) -->
    <actor name="{wp['name']}">
      <link name="link">
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.166667</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.166667</iyy><iyz>0</iyz><izz>0.166667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.5 0.5 1.5</size></box></geometry>
        </collision>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
      <skin>
        <filename>{wp['skin']}</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>{wp['skin']}</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>{wp['delay_start']:.1f}</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
{waypoints}
        </trajectory>
      </script>
    </actor>'''


# T1~T3 목표 x (스폰 -3.92에서 +x 이격, idempotent)
DESK_TARGET_X = {'t1': -2.40, 't2': 0.50, 't3': 0.50}


def refresh_desk_chairs(desk: dict) -> None:
    desk['chairs'] = chairs_for_desk(desk)


def apply_desk_spawn_shifts(layout: dict) -> None:
    """T1~T3 책상·의자 x를 스폰 이격 목표 위치로 설정."""
    for desk in layout['desks']:
        tx = DESK_TARGET_X.get(desk['name'])
        if tx is None:
            continue
        desk['x'] = tx
        refresh_desk_chairs(desk)


def generate_tactile_path(layout: dict) -> str:
    """비충돌형 노란색 점자 블록 시각 경로 (Gazebo 전용, Nav2 맵 미반영)."""
    points: list[tuple[float, float]] = []
    origin = layout['origin']
    oy = origin['y']
    by_name = {d['name']: d for d in layout['desks']}
    t1x = by_name['t1']['x']
    t23x = by_name.get('t2', by_name.get('t3', {'x': 0.0}))['x']
    t4 = by_name.get('t4', {'x': 3.2, 'y': -0.5})

    def add_line(p1: tuple[float, float], p2: tuple[float, float], step: float = 0.3) -> None:
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 0.01:
            points.append((x1, y1))
            return
        n_steps = int(dist / step)
        for i in range(n_steps + 1):
            t = i / max(1, n_steps)
            points.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))

    add_line((origin['x'] + 0.02, oy), (2.4, oy), step=0.35)
    add_line((t1x, oy), (t1x, by_name['t1']['y'] + 0.25), step=0.3)
    add_line((t23x, oy), (t23x, 0.65), step=0.3)
    add_line((t23x, oy), (t23x, -0.70), step=0.3)
    add_line((2.4, oy), (t4['x'] - 0.2, t4['y']), step=0.3)

    unique_points: list[tuple[float, float]] = []
    for p in points:
        if not any(math.hypot(p[0] - u[0], p[1] - u[1]) < 0.15 for u in unique_points):
            unique_points.append(p)

    sdf_parts = []
    for idx, (px, py) in enumerate(unique_points):
        sdf_parts.append(f"""    <model name="tactile_block_{idx}">
      <static>true</static>
      <pose>{px:.3f} {py:.3f} 0.012 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>0.25 0.25 0.003</size></box></geometry>
          <material>
            <ambient>0.95 0.75 0.06 1.0</ambient>
            <diffuse>0.95 0.75 0.06 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>""")
    return '\n'.join(sdf_parts)


def generate_sdf(layout: dict, *, enable_walking_person: bool = ENABLE_WALKING_PERSON) -> str:
    ex, ey, ez = layout['base']['pose']
    door = layout['entry_door']
    hide = layout['hideout']
    mesh_uri = layout['base'].get('mesh_uri', '1206_2_sim.dae')
    desks_xml = '\n'.join(desk_and_chairs_sdf(d) for d in layout['desks'])
    tactile_xml = generate_tactile_path(layout)

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="world_2026_amr">
    <physics name="8ms" type="ode">
      <max_step_size>0.008</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>125</real_time_update_rate>
    </physics>

    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>
    <plugin name="gz::sim::systems::Actor" filename="gz-sim-actor-system"/>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.478950</latitude_deg>
      <longitude_deg>19.057785</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>1</static>
      <pose>0 0 0.015 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- 강사님 제공 1206_2.dae 밑판 + T1~T4 책상 + 진입문 -->
    <model name="environment_1206_2">
      <static>true</static>
      <pose>{ex} {ey} {ez} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://storagy/meshes/{mesh_uri}</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://storagy/meshes/{mesh_uri}</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

{desks_xml}
    <model name="entry_door_frame">
      <static>true</static>
      <pose>{door['x']} {door['y']} 0 0 0 1.5708</pose>
      <link name="link">
        <visual name="lintel">
          <pose>0 0 2.05 0 0 0</pose>
          <geometry><box><size>{door['width_m']} 0.08 0.12</size></box></geometry>
          <material><ambient>0.1 0.55 0.2 1</ambient><diffuse>0.15 0.75 0.25 1</diffuse></material>
        </visual>
        <visual name="jamb_l">
          <pose>-0.38 0 1.0 0 0 0</pose>
          <geometry><box><size>0.08 0.08 2.0</size></box></geometry>
          <material><ambient>0.1 0.55 0.2 1</ambient><diffuse>0.15 0.75 0.25 1</diffuse></material>
        </visual>
        <visual name="jamb_r">
          <pose>0.38 0 1.0 0 0 0</pose>
          <geometry><box><size>0.08 0.08 2.0</size></box></geometry>
          <material><ambient>0.1 0.55 0.2 1</ambient><diffuse>0.15 0.75 0.25 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="entry_beacon_zone">
      <static>true</static>
      <pose>{layout['origin']['x']} {layout['origin']['y']} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="zone">
          <geometry><box><size>0.9 0.9 0.02</size></box></geometry>
          <material>
            <ambient>0.1 0.45 0.85 0.35</ambient>
            <diffuse>0.2 0.55 0.95 0.35</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="hideout_cabinet">
      <static>true</static>
      <pose>{hide['x']} {hide['y']} 0 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <pose>0 0 0.30 0 0 0</pose>
          <geometry><box><size>0.30 1.20 0.60</size></box></geometry>
        </collision>
        <visual name="vis">
          <pose>0 0 0.30 0 0 0</pose>
          <geometry><box><size>0.30 1.20 0.60</size></box></geometry>
          <material><ambient>0.18 0.18 0.20 1</ambient><diffuse>0.32 0.32 0.36 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://aruco_0</uri>
      <name>hideout_aruco</name>
      <pose>{hide['x'] + 0.17:.2f} {hide['y']} 0.35 0 1.5708 0</pose>
    </include>
{tactile_xml}
{walking_person_actor_sdf(layout) if enable_walking_person else ''}
  </world>
</sdf>
"""


# T1~T3 Nav2 goal: 메인 통로/분기점 (스폰 근처 가구·inflation 회피, table4는 의자 접근)
MIN_GOAL_DIST_FROM_ORIGIN = 2.0
def _desk_by_name(layout: dict, name: str) -> dict:
    for d in layout['desks']:
        if d['name'] == name:
            return d
    raise KeyError(name)


CORRIDOR_NAV_GOALS = {
    't1': lambda layout: (
        round(layout['origin']['x'] + MIN_GOAL_DIST_FROM_ORIGIN, 2),
        layout['origin']['y'],
    ),
    't2': lambda layout: (_desk_by_name(layout, 't2')['x'], 0.95),
    't3': lambda layout: (_desk_by_name(layout, 't3')['x'], -1.0),
}


def nav_goal_for_desk(
    desk: dict,
    img: np.ndarray | None = None,
    layout: dict | None = None,
) -> tuple[float, float, float, float]:
    """Nav2 goal — T1~T3는 스폰에서 먼 통로 접근점, T4는 +x 의자 앞 free 셀."""
    name = desk['name']
    if layout is not None and name in CORRIDOR_NAV_GOALS:
        gx, gy = CORRIDOR_NAV_GOALS[name](layout)
    else:
        px_chairs = [c for c in desk.get('chairs', []) if '_px' in c['id']]
        if px_chairs:
            gx = sum(c['x'] for c in px_chairs) / len(px_chairs)
            gy = desk['y']
        else:
            gx, gy = desk['x'], desk['y']
    if layout is not None:
        ox, oy = layout['origin']['x'], layout['origin']['y']
        dist = math.hypot(gx - ox, gy - oy)
        if dist < MIN_GOAL_DIST_FROM_ORIGIN:
            s = MIN_GOAL_DIST_FROM_ORIGIN / max(dist, 1e-6)
            gx = ox + (gx - ox) * s
            gy = oy + (gy - oy) * s
    if img is not None:
        gx, gy = snap_to_free(img, gx, gy)
    return round(gx, 2), round(gy, 2), 1.0, 0.0


def nav_goal_for_hideout(layout: dict, img: np.ndarray) -> tuple[float, float, float, float]:
    """hideout 캐비넷(occupied) 앞 room 쪽 free 셀."""
    hide = layout['hideout']
    gx, gy = snap_to_free(img, hide['x'], hide['y'] + 0.8)
    return gx, gy, hide.get('qz', 0.0), hide.get('qw', 1.0)


def is_nav_free(img: np.ndarray, px: int, py: int) -> bool:
    h, w = img.shape
    if not (0 <= px < w and 0 <= py < h):
        return False
    return int(img[py, px]) >= FREE


def snap_to_free(img: np.ndarray, wx: float, wy: float, max_cells: int = 16) -> tuple[float, float]:
    """seed 주변 BFS로 Nav2 static map free(254+) 셀에 스냅."""
    from collections import deque

    h = img.shape[0]
    spx, spy = world2px(wx, wy, h)
    if is_nav_free(img, spx, spy):
        return round(wx, 2), round(wy, 2)
    q: deque[tuple[int, int, int]] = deque([(spx, spy, 0)])
    seen = {(spx, spy)}
    while q:
        x, y, depth = q.popleft()
        if depth > 0 and is_nav_free(img, x, y):
            fx, fy = px2world(x, y, h)
            return round(fx, 2), round(fy, 2)
        if depth >= max_cells:
            continue
        for dx, dy in (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, 1), (1, -1), (-1, -1),
        ):
            nx, ny = x + dx, y + dy
            if (nx, ny) not in seen:
                seen.add((nx, ny))
                q.append((nx, ny, depth + 1))
    return round(wx, 2), round(wy, 2)


def run_dashboard_map() -> None:
    dash = Path(__file__).resolve().parent / 'generate_dashboard_map.py'
    subprocess.run([sys.executable, str(dash)], check=False)


def sync_gazebo_from_layout(
    layout_path: Path = DEFAULT_META,
    map_path: Path = MAP_1206,
    sdf_path: Path = DEFAULT_SDF,
    apply_shifts: bool = True,
    enable_walking_person: bool = ENABLE_WALKING_PERSON,
) -> int:
    """layout → Gazebo SDF + Nav2 PGM + points.yaml 전체 동기화."""
    if not layout_path.is_file():
        print(f'ERROR: layout not found: {layout_path}', file=sys.stderr)
        return 1

    layout = json.loads(layout_path.read_text(encoding='utf-8'))
    if apply_shifts:
        apply_desk_spawn_shifts(layout)
    for desk in layout['desks']:
        if 'chairs' not in desk or not desk['chairs']:
            refresh_desk_chairs(desk)

    img = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'ERROR: cannot read {map_path}', file=sys.stderr)
        return 1

    layout_path.write_text(
        json.dumps(layout, indent=2, ensure_ascii=False) + '\n', encoding='utf-8',
    )
    sdf_path.write_text(
        generate_sdf(layout, enable_walking_person=enable_walking_person),
        encoding='utf-8',
    )
    walls = ensure_walls_base(img, layout)
    write_nav_map(walls, layout, map_path)
    sync_points_yaml(layout)
    run_dashboard_map()

    print(f'[sync-gazebo] layout: {layout_path}')
    print(f'  sdf:    {sdf_path}')
    print(f'  navmap: {map_path}')
    for d in layout['desks']:
        if d['name'] in DESK_TARGET_X:
            print(f"  {d['label']}: ({d['x']}, {d['y']})")
    return 0


def sync_from_layout(
    layout_path: Path = DEFAULT_META,
    map_path: Path = MAP_1206,
) -> int:
    """Pull/merge로 받은 layout.json 기준 Nav2 PGM·points·대시보드만 동기화 (SDF/layout 덮어쓰지 않음)."""
    if not layout_path.is_file():
        print(f'ERROR: layout not found: {layout_path}', file=sys.stderr)
        return 1

    layout = json.loads(layout_path.read_text(encoding='utf-8'))
    img = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'ERROR: cannot read {map_path}', file=sys.stderr)
        return 1

    walls = ensure_walls_base(img, layout)
    write_nav_map(walls, layout, map_path)
    sync_points_yaml(layout)
    run_dashboard_map()

    print(f'[sync-from-layout] navmap: {map_path}  (from {layout_path.name})')
    print(f'  walls:  {WALLS_PGM}')
    print(f'  sdf:    unchanged')
    print(f'  layout: unchanged')
    return 0


def sync_points_yaml(layout: dict, path: Path = POINTS_YAML, map_path: Path = MAP_1206) -> None:
    """LLM Nav2 goal(points.yaml)을 layout 접근 좌표와 동기화."""
    import yaml

    img = cv2.imread(str(map_path), cv2.IMREAD_GRAYSCALE)
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    name_map = {'t1': 'table1', 't2': 'table2', 't3': 'table3', 't4': 'table4'}
    ox, oy = layout['origin']['x'], layout['origin']['y']
    data['places']['origin'] = {'x': ox, 'y': oy, 'qz': 0.0, 'qw': 1.0}
    door = layout['entry_door']
    data['places']['entry_door'] = {
        'x': door['x'], 'y': door['y'], 'qz': 0.0, 'qw': 1.0,
    }
    if img is not None:
        hx, hy, hqz, hqw = nav_goal_for_hideout(layout, img)
        data['places']['hideout'] = {'x': hx, 'y': hy, 'qz': hqz, 'qw': hqw}
    for desk in layout['desks']:
        key = name_map[desk['name']]
        x, y, qz, qw = nav_goal_for_desk(desk, img, layout)
        data['places'][key] = {'x': x, 'y': y, 'qz': qz, 'qw': qw}
    path.write_text(
        '# 1206_2 밑판 + 청사진 주석 T1~T4 (worlds/2026_amr_layout.json)\n'
        '# table* 좌표 = 테이블 앞 접근점(맵 free), 중심 아님\n'
        + yaml.dump(data, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=MAP_1206)
    parser.add_argument('--meta', type=Path, default=DEFAULT_META)
    parser.add_argument('--sdf', type=Path, default=DEFAULT_SDF)
    parser.add_argument('--from-map', action='store_true',
                        help='맵 윤곽선 자동 추정 (기본: 청사진 BLUEPRINT_DESKS)')
    parser.add_argument('--table-scale', type=float, default=TABLE_LENGTH_SCALE,
                        help=f'테이블 크기 배율 (기본 {TABLE_LENGTH_SCALE})')
    parser.add_argument(
        '--from-layout', action='store_true',
        help='기존 layout.json 기준 Nav2 PGM·points·대시보드만 동기화 (SDF/layout 덮어쓰지 않음)',
    )
    parser.add_argument(
        '--sync-gazebo', action='store_true',
        help='layout → Gazebo SDF + Nav2 PGM + points (T1~T3 스폰 이격 shift 포함)',
    )
    parser.add_argument(
        '--walking-person', action='store_true',
        help='Gazebo 배회 actor(walking_person) 포함 (기본: 제외)',
    )
    args = parser.parse_args()

    enable_walking = args.walking_person or ENABLE_WALKING_PERSON

    if args.sync_gazebo:
        return sync_gazebo_from_layout(
            args.meta, args.map, args.sdf, enable_walking_person=enable_walking,
        )

    if args.from_layout:
        return sync_from_layout(args.meta, args.map)

    img = cv2.imread(str(args.map), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'ERROR: cannot read {args.map}', file=sys.stderr)
        return 1

    layout = build_layout(img, from_map=args.from_map, table_scale=args.table_scale)
    walls = ensure_walls_base(img, layout)
    write_nav_map(walls, layout, args.map)
    sync_points_yaml(layout)
    args.meta.parent.mkdir(parents=True, exist_ok=True)
    args.meta.write_text(json.dumps(layout, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    args.sdf.write_text(
        generate_sdf(layout, enable_walking_person=enable_walking),
        encoding='utf-8',
    )

    run_dashboard_map()

    print('1206_2.dae base + 4 tables (blueprint'
          f', scale={args.table_scale})' if not args.from_map
          else '1206_2.dae base + 4 tables (map auto)')
    print(f"  door:  ({layout['entry_door']['x']}, {layout['entry_door']['y']})")
    for d in layout['desks']:
        n_ch = len(d.get('chairs', []))
        print(
            f"  {d['label']}: ({d['x']}, {d['y']}) "
            f"{d['size_x']}x{d['size_y']}m {d['orientation']}  chairs={n_ch}"
        )
    print(f'  mesh:   {layout["base"].get("mesh_uri")}')
    print(f'  walls:  {WALLS_PGM}')
    print(f'  navmap: {args.map}  (Gazebo furniture synced)')
    print(f'  layout: {args.meta}')
    print(f'  sdf:    {args.sdf}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
