#!/usr/bin/env python3
"""
data_loader.py - 数据加载模块
加载卫星参数、任务参数、access 窗口
"""
import os, json, csv
from collections import defaultdict
from config import DATA_PATH


def parse_intervals(s: str) -> list:
    """解析JSON格式时间区间字符串为列表

    Args:
        s: JSON 格式字符串，如 "[[0, 26889], [37691, 86399]]"

    Returns:
        区间列表 [(a, b), ...]，解析失败返回空列表
    """
    if not s or not s.strip():
        return []
    try:
        return [(int(a), int(b)) for a, b in json.loads(s)]
    except Exception:
        return []


def load_satellites(path: str = None) -> dict:
    """加载卫星参数 CSV

    Args:
        path: CSV 文件路径，默认 data/satellite.csv

    Returns:
        卫星参数字典 {name: {type, left_roll, right_roll, startup_min, ...}}
    """
    if path is None:
        path = os.path.join(DATA_PATH, 'satellite.csv')
    sats = {}
    with open(path, 'r', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            n = r['satellite_name']
            sats[n] = {
                'name': n,
                'type': r['satellite_type'],
                'left_roll': float(r['left_roll'] or 0),
                'right_roll': float(r['right_roll'] or 0),
                'startup_min': float(r['startup_min'] or 5),
                'startup_max': float(r['startup_max'] or 60),
                'orbit_max': float(r['orbit_max'] or 600),
                'trantime_0': float(r['trantime_0'] or 20),
                'trantime_10': float(r['trantime_10'] or 40),
                'trantime_20': float(r['trantime_20'] or 60),
                'trantime_sar': float(r['trantime_sar'] or 20),
                'trantime_cc': float(r['trantime_cc'] or 20),
                'weather': int(r['weather'] or 1),
                'sunlight': float(r['sunlight'] or 10),
                'reversal_time': float(r.get('reversal_time', 0) or 0),
                'lf': _parse_optional_float(r, 'left_forbidden'),
                'rf': _parse_optional_float(r, 'right_forbidden'),
                'swath': float(r.get('swath_width', 10) or 10),
                'sar_width': float(r.get('sar_width', 10) or 10),
            }
    return sats


def load_missions(path: str = None) -> dict:
    """加载任务参数 CSV

    Args:
        path: CSV 文件路径，默认 data/mission.csv

    Returns:
        任务参数字典 {name: {type, score, validity, frequency, ...}}
    """
    if path is None:
        path = os.path.join(DATA_PATH, 'mission.csv')
    miss = {}
    with open(path, 'r', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            n = r['mission_name']
            miss[n] = {
                'name': n,
                'type': r['mission_type'],
                'score': float(r['score'] or 0),
                'validity': parse_intervals(r.get('validity', '')),
                'weather': parse_intervals(r.get('weather', '')),
                'sunlight': parse_intervals(r.get('sunlight', '')),
                'frequency': int(r.get('frequency', 0) or 0),
                'time_interval': float(r.get('time_interval', 0) or 0),
                'comm_min': float(r.get('comm_min', 0) or 0),
                'comm_max': float(r.get('comm_max', 0) or 0),
            }
    return miss


def load_access(path: str = None) -> tuple:
    """加载 access.csv，分离观测窗口和通信窗口

    Args:
        path: CSV 文件路径，默认 data/access.csv

    Returns:
        (obs_acc, stn_acc)
        obs_acc: {sat: {mission: [(time, roll), ...]}}
        stn_acc: {sat: {station: [(start, end), ...]}}
    """
    if path is None:
        path = os.path.join(DATA_PATH, 'access.csv')
    obs_acc = defaultdict(lambda: defaultdict(list))   # sat -> mission -> [(time, roll), ...]
    stn_acc = defaultdict(lambda: defaultdict(list))   # sat -> station -> [(start, end), ...]
    with open(path, 'r', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            sat, ms, stn_s = r['satellite_name'], r['mission_name'], r['station_name']
            t = int(r['start_time'])
            roll_s = (r.get('roll_angle', '') or '').strip()
            roll = float(roll_s) if roll_s else 0.0
            if stn_s and stn_s.strip():
                stn_acc[sat][stn_s].append((t, int(r['end_time'])))
            else:
                obs_acc[sat][ms].append((t, roll))
    for d in [obs_acc, stn_acc]:
        for sat in d:
            for k in d[sat]:
                d[sat][k].sort()
    return obs_acc, stn_acc


def load_all():
    """一次性加载全部数据

    Returns:
        (sats, miss, obs_acc, stn_acc)
        sats: 卫星参数字典
        miss: 任务参数字典
        obs_acc: 观测 access 数据
        stn_acc: 地面站 access 数据
    """
    print("Loading data...")
    sats = load_satellites()
    miss = load_missions()
    print("Loading access...")
    obs_acc, stn_acc = load_access()
    return sats, miss, obs_acc, stn_acc


def _parse_optional_float(row: dict, key: str) -> float | None:
    """解析可选浮点字段，空值返回 None

    Args:
        row: CSV 行字典
        key: 字段名

    Returns:
        浮点值或 None
    """
    v = row.get(key, '').strip()
    if not v:
        return None
    return float(v)
