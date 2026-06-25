#!/usr/bin/env python3
"""Gazebo ?붾뱶 ?앹꽦: 1206_2.dae 諛묓뙋 + 1206_sim_1 留?湲곕컲 梨낆긽 4媛?T1~T4) + 吏꾩엯臾?

?ъ슜:
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
DEFAULT_META = PKG_ROOT / 'src/storagy/worlds/2026_amr_layout.json'
DEFAULT_SDF = PKG_ROOT / 'src/storagy/worlds/2026_amr.sdf'
SRC_DAE = PKG_ROOT / 'src/storagy/meshes/1206_2.dae'
SIM_DAE = PKG_ROOT / 'src/storagy/meshes/1206_2_sim.dae'
DAE_NS = 'http://www.collada.org/2005/11/COLLADASchema'

# 1206_2.dae 以묒븰 ?뚰삎 ?뚯씠釉?2媛?Cube_016/017) ??brown box T2/T3? 以묐났
DAE_REMOVE_NODES = {'Cube_016', 'Cube_017'}
DAE_REMOVE_GEOMS = {'Cube_028-mesh', 'Cube_029-mesh'}

RES = 0.05
ORIGIN_1206 = (-5.0, -3.8)

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

# T1 -x硫? 吏꾩엯臾?-4.47, 0.12) 履??듬줈 ???섏옄 2媛?諛곗튂 ????CHAIR_SKIP_SIDES = {'t1': {'nx'}}

# T2/T3 ?숈そ ?듬줈瑜?遺곣넄?⑥쑝濡?諛고쉶 (Nav2 /scan ?μ븷臾쇰줈 ?뚰뵾)
WALKING_PERSON = {
    'name': 'walking_person',
    'skin': 'walk.dae',
    'z': 0.875,
    'delay_start': 5.0,
    'waypoints': [
        (0.0, 1.20, -2.20, math.pi / 2),
        (35.0, 1.20, 2.20, math.pi / 2),
        (37.0, 1.20, 2.20, -math.pi / 2),
        (72.0, 1.20, -2.20, -math.pi / 2),
        (74.0, 1.20, -2.20, math.pi / 2),
    ],
}

# 泥?궗吏?湲곗? T1~T4 諛곗튂 (scale=1.0 湲곗? ?ш린쨌醫뚰몴)
BASE_BLUEPRINT_DESKS = [
    {'name': 't1', 'label': 'T1', 'x': -2.75, 'y': 0.45, 'size_x': 0.65, 'size_y': 2.20},
    {'name': 't2', 'label': 'T2', 'x': -1.00, 'y': 1.25, 'size_x': 0.65, 'size_y': 1.80},
    {'name': 't3', 'label': 'T3', 'x': -1.00, 'y': -0.90, 'size_x': 0.65, 'size_y': 1.80},
    {'name': 't4', 'label': 'T4', 'x': 2.75, 'y': -0.50, 'size_x': 0.65, 'size_y': 2.20},
]

# ?몃줈 湲몄씠쨌?먭퍡 ?숈씪 鍮꾩쑉 ?뺣? (T2/T3 媛꾧꺽? 寃뱀묠 ?놁씠 ?먮룞 踰뚮┝)
TABLE_LENGTH_SCALE = 1.25
TABLE_PAIR_EDGE_GAP = 0.35   # T2?밫3 媛?μ옄由?媛꾧꺽


def _pair_half_sep(size_y_t2: float, size_y_t3: float) -> float:
    return ((size_y_t2 + size_y_t3) / 2.0 + TABLE_PAIR_EDGE_GAP) / 2.0

CHAIR_COLORS = {
    't1': (0.55, 0.68, 0.38),
    't2': (0.78, 0.72, 0.48),
    't3': (0.82, 0.62, 0.58),
    't4': (0.58, 0.70, 0.42),
}

# T1~T4 寃???곸뿭 (--from-map ?듭뀡?? 留??ㅺ낸???먮룞 異붿젙)
TABLE_REGIONS = {
    'T1': {'x': (-3.5, -2.2), 'y': (0.5, 2.5)},
    'T2': {'x': (-0.5, 2.5), 'y': (2.8, 3.5)},
    'T3': {'x': (0.5, 2.8), 'y': (-3.6, -2.5)},
    'T4': {'x': (3.5, 4.9), 'y': (-1.8, 0.5)},
}


def ensure_sim_dae() -> str:
    """DAE?먯꽌 以묒븰 ?뚰삎 ?뚯씠釉?2媛??쒓굅??1206_2_sim.dae ?앹꽦."""
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
    """?몃줈 ?뚯씠釉?媛濡쒕㈃(짹x)?먮쭔 ?섏옄 2媛쒖뵫 ??湲?蹂(y) 諛⑺뼢 ?섎???"""
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


def detect_entry_door(img: np.ndarray) -> dict:
    """?쇱そ 踰?吏꾩엯臾???free 怨듦컙 以?醫뚯륫(x<-4.25) 以묒븰?."""
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
    """?곸뿭 留덉뒪?ъ뿉??媛????connected component??異뺤젙??bbox."""
    n, _labels, stats, centroids = cv2.connectedComponentsWithStats(
        region_mask.astype(np.uint8), connectivity=8,
    )
    if n <= 1:
        raise RuntimeError('occupied connected component瑜?李얠? 紐삵뻽?듬땲??')
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
    """鍮꾩쑉 ?좎? ?뺣? + T2/T3瑜?T1?밫4 以묎컙???移?諛곗튂."""
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
    """泥?궗吏?二쇱꽍 湲곗? T1~T4 (?몃줈 諛곗튂, 鍮꾩쑉 ?뺣?)."""
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
    """1206 留?occupied ?ㅺ낸?좎뿉??T1~T4 以묒떖쨌?ш린쨌諛⑺뼢(理쒕? CC bbox) 異붿젙."""
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
            raise RuntimeError(f'{name} ?곸뿭?먯꽌 occupied ?쎌???李얠? 紐삵뻽?듬땲??')

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
    """媛꾨떒???섏옄(醫뚰뙋+?깅컺??4?ㅻ━)."""
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


def walking_person_actor_sdf() -> str:
    """Gazebo actor ??walk.dae ?ㅽ궓, 異⑸룎 諛뺤뒪???쇱씠??/scan)???≫? Nav2媛 ?뚰뵾."""
    wp = WALKING_PERSON
    waypoints = '\n'.join(
        f'''          <waypoint>
            <time>{t:.1f}</time>
            <pose>{x:.2f} {y:.2f} {wp['z']:.3f} 0 0 {yaw:.4f}</pose>
          </waypoint>'''
        for t, x, y, yaw in wp['waypoints']
    )
    return f'''
    <!-- 諛고쉶?섎뒗 ?щ엺 (濡쒕큸 ?쇱씠????Nav2 ?μ븷臾??뚰뵾) -->
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


def generate_tactile_path() -> str:
    """鍮꾩땐?뚰삎 ?몃????먯옄 釉붾줉 ?쒓컖 寃쎈줈 ?앹꽦."""
    points = []

    def add_line(p1, p2, step=0.3):
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

    # Main corridor trunk
    add_line((-3.9, 0.12), (2.4, 0.12), step=0.35)

    # T1 branch (goes north to T1)
    add_line((-2.75, 0.12), (-2.75, 0.70), step=0.3)

    # T2 branch (goes north to T2 edge)
    add_line((0.0, 0.12), (0.0, 0.65), step=0.3)

    # T3 branch (goes south to T3 edge)
    add_line((0.0, 0.12), (0.0, -0.70), step=0.3)

    # T4 branch (goes to T4)
    add_line((2.4, 0.12), (2.75, -0.50), step=0.3)

    # Deduplicate points that are too close
    unique_points = []
    for p in points:
        if not any(math.hypot(p[0] - u[0], p[1] - u[1]) < 0.15 for u in unique_points):
            unique_points.append(p)

    # Format to visual-only SDF models
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


def generate_sdf(layout: dict) -> str:
    ex, ey, ez = layout['base']['pose']
    door = layout['entry_door']
    hide = layout['hideout']
    mesh_uri = layout['base'].get('mesh_uri', '1206_2_sim.dae')
    desks_xml = '\n'.join(desk_and_chairs_sdf(d) for d in layout['desks'])
    tactile_xml = generate_tactile_path()

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

    <!-- 媛뺤궗???쒓났 1206_2.dae 諛묓뙋 + T1~T4 梨낆긽 + 吏꾩엯臾?-->
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
{walking_person_actor_sdf()}
  </world>
</sdf>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=MAP_1206)
    parser.add_argument('--meta', type=Path, default=DEFAULT_META)
    parser.add_argument('--sdf', type=Path, default=DEFAULT_SDF)
    parser.add_argument('--from-map', action='store_true',
                        help='留??ㅺ낸???먮룞 異붿젙 (湲곕낯: 泥?궗吏?BLUEPRINT_DESKS)')
    parser.add_argument('--table-scale', type=float, default=TABLE_LENGTH_SCALE,
                        help=f'?뚯씠釉??ш린 諛곗쑉 (湲곕낯 {TABLE_LENGTH_SCALE})')
    args = parser.parse_args()

    img = cv2.imread(str(args.map), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'ERROR: cannot read {args.map}', file=sys.stderr)
        return 1

    layout = build_layout(img, from_map=args.from_map, table_scale=args.table_scale)
    args.meta.parent.mkdir(parents=True, exist_ok=True)
    args.meta.write_text(json.dumps(layout, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    args.sdf.write_text(generate_sdf(layout), encoding='utf-8')

    dash = Path(__file__).resolve().parent / 'generate_dashboard_map.py'
    subprocess.run([sys.executable, str(dash)], check=False)

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
    print(f'  layout: {args.meta}')
    print(f'  sdf:    {args.sdf}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
