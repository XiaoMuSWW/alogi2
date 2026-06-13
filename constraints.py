#!/usr/bin/env python3
"""
constraints.py - 约束检查模块
对应 PDF section 2.14 的 9 个 status 检查
"""
from config import ORBIT_PERIOD, STATIONS, COMM_RATIO_MIN, COMM_RATIO_MAX


# ============================================================
# Status 2: 侧摆角约束
# ============================================================
def check_roll_strict(sat: dict, roll: float) -> bool:
    """严格侧摆角检查（点目标 p）
    左边界 ≤ roll ≤ 右边界，且 SAR 不在禁飞区
    """
    if not (sat['left_roll'] <= roll <= sat['right_roll']):
        return False
    return not _in_forbidden(sat, roll)


def clamp_roll_area(sat: dict, roll: float) -> float:
    """区域任务侧摆角夹持（a / ap / aa）
    将超出卫星范围的 roll 夹持到最近有效边界，避开禁飞区
    """
    if sat['left_roll'] <= roll <= sat['right_roll']:
        if _in_forbidden(sat, roll):
            return _nearest_valid(sat, roll)
        return roll
    clamped = max(sat['left_roll'], min(sat['right_roll'], roll))
    if _in_forbidden(sat, clamped):
        return _nearest_valid(sat, clamped)
    return clamped


# ============================================================
# Status 3 + Status 6: 开机时间
# ============================================================
def obs_duration(sat: dict, mission: dict) -> int:
    """计算观测窗口时长
    - SAR 点目标(p/ap)：固定 10 秒（Status 6）
    - 区域目标(a/aa)：使用 startup_max 最大化覆盖面积（上限60s）
    - 光学点目标(p/ap)：使用 startup_min，最小 5 秒
    """
    mt = mission['type']
    # SAR 点目标 / 追加点：固定 10 秒
    if sat['type'] == 'sar' and mt in ('p', 'ap'):
        return 10
    # 区域目标 / 动目标：使用完整 startup_max
    if mt in ('a', 'aa'):
        return max(int(sat.get('startup_max', 60)), 5)
    # 光学点目标 / 追加点：最小开机时间
    return max(int(sat['startup_min']), 5)


def check_startup_max(sat: dict, dur: int) -> bool:
    """检查观测时长是否 ≤ startup_max（Status 3 最长开机时间约束）"""
    return dur <= int(sat.get('startup_max', 60))


# ============================================================
# Status 4: 窗口重叠检查
# ============================================================
def trans_time(sat: dict, r1: float | None, r2: float) -> int:
    """姿态转换时间：根据侧摆角差值区间计算
    - 光学卫星：差值 0-10° → trantime_0, 10-20° → trantime_10, 20°+ → trantime_20
    - SAR 卫星：统一 trantime_sar
    """
    if r1 is None:
        return 0
    if sat['type'] == 'sar':
        return int(sat['trantime_sar'])
    diff = abs(r2 - r1)
    if diff <= 10:
        return int(sat['trantime_0'])
    elif diff <= 20:
        return int(sat['trantime_10'])
    else:
        return int(sat['trantime_20'])


def need_reversal(sat: dict, r1: float | None, r2: float) -> bool:
    """SAR 反转判断：两次侧摆角符号相反，穿越禁飞区"""
    if sat['type'] != 'sar' or r1 is None:
        return False
    if sat['lf'] is None:
        return False
    lf, rf = sat['lf'], sat['rf']
    return (r1 <= lf and r2 >= rf) or (r1 >= rf and r2 <= lf)


def sat_timeline_gap(sat: dict, last_roll: float | None, new_roll: float) -> int:
    """同一卫星两次任务之间的最小间隔 = 姿态转换 + 反转 + 测控/数传转换"""
    tt = trans_time(sat, last_roll, new_roll)
    rev = sat['reversal_time'] if need_reversal(sat, last_roll, new_roll) else 0
    cc = int(sat.get('trantime_cc', 0) or 0)
    return int(tt + rev + cc)


def check_sat_insertion(sat: dict, timeline: list, ul_start: int, sat_end: int,
                        new_roll: float) -> bool:
    """检查新窗口是否能插入卫星时间线
    timeline: [(ul_start, sat_end, roll, mission), ...] 按时间升序
    对前驱和后继分别计算 transition gap（含 SAR 反转 + 测控/数传转换）
    返回 True = 冲突，不可插入
    """
    if not timeline:
        return False

    lo, hi = 0, len(timeline)
    while lo < hi:
        mid = (lo + hi) // 2
        if timeline[mid][0] < ul_start:
            lo = mid + 1
        else:
            hi = mid
    ins_pos = lo

    if ins_pos > 0:
        pred = timeline[ins_pos - 1]
        pred_end = pred[1]
        pred_roll = pred[2]
        gap = sat_timeline_gap(sat, pred_roll, new_roll)
        if pred_end + gap >= ul_start:
            return True

    if ins_pos < len(timeline):
        succ = timeline[ins_pos]
        succ_start = succ[0]
        succ_roll = succ[2]
        gap = sat_timeline_gap(sat, new_roll, succ_roll)
        if sat_end + gap >= succ_start:
            return True

    return False


def check_station_conflict(booked: list, t_start: int, t_end: int,
                           new_sat: str = None, sats: dict = None) -> bool:
    """检查地面站是否忙
    booked: [(start, end, sat_name), ...]
    不同卫星需要 trantime_cc 转换时间间隙
    返回 True = 冲突
    """
    for bs, be, booked_sat in booked:
        if new_sat and sats and booked_sat != new_sat:
            cc = int(sats.get(booked_sat, {}).get('trantime_cc', 0) or 0)
            if t_start <= be + cc and bs <= t_end + cc:
                return True
        else:
            if t_start <= be and bs <= t_end:
                return True
    return False


# ============================================================
# Status 7: 轨道累计时间（每 5400s 周期）
# ============================================================
def check_orbit_sum(orbit_sum: dict, sat_key: str, obs_start: int,
                    dur: int, orbit_max: float) -> bool:
    """检查当前观测是否超出该轨道周期的累计上限"""
    p_idx = obs_start // ORBIT_PERIOD
    cum = orbit_sum.get(sat_key, {}).get(p_idx, 0)
    return cum + dur <= orbit_max


# ============================================================
# Status 8: 任务本身约束（有效窗口、天气、光照）
# ============================================================
def check_mission_window(mi: dict, os_: int, oe: int) -> bool:
    """检查任务的时间窗口有效性"""
    if not (_in_interval(os_, mi['validity']) and
            _in_interval(oe, mi['validity'])):
        return False
    if mi['weather'] and not _in_interval(os_, mi['weather']):
        return False
    return True


def check_sunlight(sat: dict, mi: dict, t: int) -> bool:
    """光学卫星光照检查（SAR 不检查）"""
    if sat['type'] != 'opt':
        return True
    if not mi['sunlight']:
        return True
    return _in_interval(t, mi['sunlight'])


def check_time_interval(prev_obs: list, new_start: int, time_int_h: float) -> bool:
    """同一任务两次观测的时间间隔检查（单位：小时）"""
    if not prev_obs or time_int_h <= 0:
        return True
    last_end = prev_obs[-1][3]
    return new_start - last_end >= time_int_h * 3600


# ============================================================
# Status 5: 数传/成像时间比
# ============================================================
def check_comm_ratio(mi: dict, obs_dur: int, dl_dur: int) -> bool:
    """检查数传/成像时间比 ∈ [comm_min, comm_max]
    比值 = dl_dur / obs_dur
    优先使用任务级 comm_min/comm_max，否则使用全局 COMM_RATIO_MIN/MAX（2.0~2.5）
    """
    cmin = mi.get('comm_min', 0) or COMM_RATIO_MIN
    cmax = mi.get('comm_max', 0) or COMM_RATIO_MAX
    if obs_dur <= 0:
        return False
    ratio = dl_dur / obs_dur
    if ratio < cmin - 1e-9:
        return False
    if ratio > cmax + 1e-9:
        return False
    return True


def calc_dl_dur(obs_dur: int, mi: dict = None) -> int:
    """动态计算下行数传时长 = 成像时间 × 最小时间比
    优先使用任务级 comm_min，否则使用全局 COMM_RATIO_MIN
    """
    cmin = COMM_RATIO_MIN
    if mi:
        mcmin = mi.get('comm_min', 0)
        if mcmin and mcmin > 0:
            cmin = mcmin
    return max(int(obs_dur * cmin), 1)


# ============================================================
# 内部辅助函数
# ============================================================
def _in_forbidden(sat: dict, roll: float) -> bool:
    """判断 roll 是否在 SAR 禁飞区"""
    if sat['lf'] is None or sat['rf'] is None:
        return False
    return sat['lf'] <= roll <= sat['rf']


def _nearest_valid(sat: dict, roll: float) -> float:
    """回退到最近的合法侧摆角（禁飞区外）"""
    candidates = [sat['left_roll'], sat['right_roll']]
    if sat['lf'] is not None:
        candidates.extend([sat['lf'], sat['rf']])
    valid = [v for v in candidates
             if sat['left_roll'] <= v <= sat['right_roll']
             and not _in_forbidden(sat, v)]
    valid.sort(key=lambda v: abs(v - roll))
    return valid[0] if valid else roll


def _in_interval(t: int, intervals: list) -> bool:
    """判断时间点 t 是否在任意区间内"""
    if not intervals:
        return True
    return any(a <= t <= b for a, b in intervals)
