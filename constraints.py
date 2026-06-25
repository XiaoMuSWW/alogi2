#!/usr/bin/env python3
"""
constraints.py - 约束检查模块
对应 PDF section 2.14 的 9 个 status 检查
"""
from config import ORBIT_PERIOD, COMM_RATIO_MIN, COMM_RATIO_MAX

# =========================================================
# Status 1，9:检查由时间窗口计算完成，无需约束
# =========================================================


# ============================================================
# Status 2: 侧摆角约束
# ============================================================
def check_roll_strict(sat: dict, roll: float) -> bool:
    """严格侧摆角检查（点目标 p / ap）

    Args:
        sat: 卫星参数字典，含 left_roll / right_roll / lf / rf
        roll: 目标侧摆角（度）

    Returns:
        True 如果左边界 ≤ roll ≤ 右边界 且 roll 不在 SAR 禁飞区
    """
    if not (sat['left_roll'] <= roll <= sat['right_roll']):
        return False
    return not _in_forbidden(sat, roll)


def clamp_roll_area(sat: dict, roll: float) -> float:
    """侧摆角夹持（适用于所有任务类型 p/ap/a/aa）

    将超出卫星范围的 roll 夹持到最近有效边界，避开禁飞区。
    若原始侧摆角已在有效范围内且不在禁飞区，直接返回原值。

    Args:
        sat: 卫星参数字典，含 left_roll / right_roll / lf / rf
        roll: 原始侧摆角（度）

    Returns:
        夹持后的有效侧摆角（度）
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
# Status 3 + Status 6: 开机时间约束
# ============================================================
def check_startup(sat: dict, dur: int) -> bool:
    """检查观测时长是否在 [startup_min, startup_max] 范围内（Status 3）

    Args:
        sat: 卫星参数字典，含 startup_min / startup_max
        dur: 观测时长（秒）

    Returns:
        True 如果 startup_min ≤ dur ≤ startup_max
    """
    return dur >= int(sat.get('startup_min', 5)) and dur <= int(sat.get('startup_max', 60))


def obs_duration(sat: dict, mission: dict) -> int:
    """计算观测窗口时长

    - SAR 点目标(p/ap)：固定 10 秒（Status 6）
    - 区域目标(a/aa)：使用 startup_max 最大化覆盖面积（上限60s）
    - 光学点目标(p/ap)：使用 startup_min，最小 5 秒

    Args:
        sat: 卫星参数字典，含 type / startup_min / startup_max
        mission: 任务参数字典，含 type

    Returns:
        观测时长（秒）
    """
    mt = mission['type']
    # SAR 点目标 / 追加点：固定 10 秒
    if sat['type'] == 'sar' and mt in ('p', 'ap'):
        return 10
    # 区域目标 / 追加区域目标：使用完整 startup_max
    if mt in ('a', 'aa'):
        return max(int(sat.get('startup_max', 60)), 5)
    # 光学点目标 / 追加点：最小开机时间
    return max(int(sat['startup_min']), 5)


# ============================================================
# Status 4: 窗口重叠检查
# ============================================================
def _comm_type(sat: dict, base_mode: bool, is_end: bool) -> str:
    """推断卫星时间线上任务边界的通信类型

    任务始终以 DL（数传）结束；起始类型取决于模式：
    - base_mode: OBS ∥ DL，起始包含 DL
    - wave_mode: UL → OBS ∥ DL，起始为 UL（测控）

    Args:
        sat: 卫星参数字典（保留以便扩展）
        base_mode: True=基线模式, False=波次模式
        is_end: True=任务末尾类型, False=任务起始类型

    Returns:
        'DL' | 'UL' | 'OBS'
    """
    if is_end:
        return 'DL'  # 所有任务末尾均为数传
    return 'DL' if base_mode else 'UL'


def sat_timeline_gap(sat: dict, last_roll: float | None, new_roll: float,
                     same_comm_type: bool = False) -> int:
    """同一卫星两次任务之间的最小间隔

    gap = 姿态转换时间 + SAR反转时间 + 测控/数传转换时间

    当两次任务边界通信类型相同时（如 DL→DL），省略测控/数传转换时间。

    Args:
        sat: 卫星参数字典，含 type / reversal_time / trantime_cc / trantime_sar / lf / rf
        last_roll: 前一次观测的侧摆角，None 表示无前驱
        new_roll: 新观测的侧摆角（度）
        same_comm_type: 前后边界通信类型是否相同，True 时跳过 trantime_cc

    Returns:
        最小间隔时间（秒）
    """
    tt = trans_time(sat, last_roll, new_roll)
    rev = sat['reversal_time'] if need_reversal(sat, last_roll, new_roll) else 0
    cc = 0 if same_comm_type else int(sat.get('trantime_cc', 0) or 0)
    return int(tt + rev + cc)


def trans_time(sat: dict, r1: float | None, r2: float) -> int:
    """姿态转换时间：根据侧摆角差值区间计算

    - 光学卫星：差值 0-10° → trantime_0, 10-20° → trantime_10, 20°+ → trantime_20
    - SAR 卫星：统一 trantime_sar

    Args:
        sat: 卫星参数字典，含 type / trantime_0 / trantime_10 / trantime_20 / trantime_sar
        r1: 起始侧摆角（度），None 表示无前驱
        r2: 目标侧摆角（度）

    Returns:
        姿态转换时间（秒）
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
    """SAR 反转判断：两次侧摆角符号相反且穿越禁飞区时需要反转

    Args:
        sat: 卫星参数字典，含 type / lf / rf
        r1: 起始侧摆角（度），None 表示无前驱
        r2: 目标侧摆角（度）

    Returns:
        True 如果 SAR 卫星侧摆角穿越禁飞区需要反转
    """
    if sat['type'] != 'sar' or r1 is None:
        return False
    if sat['lf'] is None:
        return False
    lf, rf = sat['lf'], sat['rf']
    return (r1 <= lf and r2 >= rf) or (r1 >= rf and r2 <= lf)


# ============================================================
# Status 5: 数传/成像时间比
# ============================================================
def check_comm_ratio(mi: dict, obs_dur: int, dl_dur: int) -> bool:
    """检查数传/成像时间比 ∈ [comm_min, comm_max]

    比值 = dl_dur / obs_dur
    优先使用任务级 comm_min/comm_max，否则使用全局 COMM_RATIO_MIN/MAX（2.0~2.5）

    Args:
        mi: 任务参数字典，含 comm_min / comm_max
        obs_dur: 观测（成像）时长（秒）
        dl_dur: 下行数传时长（秒）

    Returns:
        True 如果时间比在合法范围内
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

    Args:
        obs_dur: 观测时长（秒）
        mi: 任务参数字典，含 comm_min，可为 None

    Returns:
        下行数传时长（秒），最小为 1
    """
    cmin = COMM_RATIO_MIN
    if mi:
        mcmin = mi.get('comm_min', 0)
        if mcmin and mcmin > 0:
            cmin = mcmin
    return max(int(obs_dur * cmin), 1)


def _is_type_switch(t1: str, t2: str) -> bool:
    """判断两个任务类型是否为点↔区域切换

    点目标: p / ap，区域目标: a / aa

    Args:
        t1: 第一个任务的类型
        t2: 第二个任务的类型

    Returns:
        True 如果一个为点目标、另一个为区域目标
    """
    point = {'p', 'ap'}
    area = {'a', 'aa'}
    return (t1 in point and t2 in area) or (t1 in area and t2 in point)


def check_sat_insertion(sat: dict, timeline: list, ul_start: int, sat_end: int,
                        new_roll: float, new_type: str = None,
                        miss: dict = None, start_with_ul: bool = False) -> bool:
    """检查新窗口是否能插入卫星时间线

    对前驱和后继分别计算 transition gap（含 SAR 反转 + 测控/数传转换）。
    点↔区域任务切换时，至少需满足 trantime_sar 任务切换时间。
    前后边界通信类型相同时，省略 trantime_cc。

    Args:
        sat: 卫星参数字典，含 type / lf / rf / reversal_time / trantime_cc / trantime_sar
        timeline: 已占用时间线，[(ul_start, sat_end, roll, mission_name, start_with_ul?), ...] 按时间升序
        ul_start: 新窗口的卫星占用起始时间（秒）
        sat_end: 新窗口的卫星占用结束时间（秒）
        new_roll: 新窗口的侧摆角（度）
        new_type: 新任务类型 (p/ap/a/aa)，用于点↔区域切换检查
        miss: 任务字典 {name: {type, ...}}，用于查前后任务类型
        start_with_ul: 新任务是否以 UL 起始（False=基线方案以DL起始, True=波次方案以UL起始）

    Returns:
        True = 冲突，不可插入；False = 可插入
    """
    if not timeline:
        return False

    trantime_sar = int(sat.get('trantime_sar', 20))
    # 新任务的通信起始类型
    new_start_comm = 'DL' if not start_with_ul else 'UL'

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
        # 前驱末尾永远是 DL，新任务起始由 start_with_ul 决定
        pred_start_with_ul = pred[4] if len(pred) > 4 else True  # 向后兼容
        pred_end_comm = 'DL'
        same_comm = (pred_end_comm == new_start_comm)
        gap = sat_timeline_gap(sat, pred_roll, new_roll, same_comm_type=same_comm)
        # 点↔区域切换: 至少需要 trantime_sar 任务切换时间
        if new_type and miss:
            pred_mn = pred[3] if len(pred) > 3 else None
            if pred_mn:
                pred_type = miss.get(pred_mn, {}).get('type', '')
                if _is_type_switch(pred_type, new_type):
                    gap = max(gap, trantime_sar)
        if pred_end + gap >= ul_start:
            return True

    if ins_pos < len(timeline):
        succ = timeline[ins_pos]
        succ_start = succ[0]
        succ_roll = succ[2]
        # 后继起始类型从时间线读取，新任务末尾永远是 DL
        succ_start_with_ul = succ[4] if len(succ) > 4 else True  # 向后兼容
        succ_start_comm = 'DL' if not succ_start_with_ul else 'UL'
        same_comm = ('DL' == succ_start_comm)  # 新末尾 DL == 后继起始？
        gap = sat_timeline_gap(sat, new_roll, succ_roll, same_comm_type=same_comm)
        # 点↔区域切换: 至少需要 trantime_sar 任务切换时间
        if new_type and miss:
            succ_mn = succ[3] if len(succ) > 3 else None
            if succ_mn:
                succ_type = miss.get(succ_mn, {}).get('type', '')
                if _is_type_switch(succ_type, new_type):
                    gap = max(gap, trantime_sar)
        if sat_end + gap >= succ_start:
            return True

    return False


def check_station_conflict(booked: list, t_start: int, t_end: int,
                           new_sat: str = None, sats: dict = None) -> bool:
    """检查地面站时间是否冲突

    不同卫星需要 trantime_cc 转换时间间隙。

    Args:
        booked: 已占用列表，[(start, end, sat_name), ...]
        t_start: 新占用起始时间（秒）
        t_end: 新占用结束时间（秒）
        new_sat: 新占用卫星名
        sats: 卫星参数字典，含 trantime_cc

    Returns:
        True = 冲突；False = 可占用
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
# Status 7: 单圈累计时间
# ============================================================
def check_orbit_sum(orbit_sum: dict, sat_key: str, obs_start: int,
                    dur: int, orbit_max: float) -> bool:
    """检查当前观测是否超出该轨道周期的累计上限

    Args:
        orbit_sum: 轨道累计字典 {sat_name: {period_index: cum_time}}
        sat_key: 卫星名
        obs_start: 观测起始时间（秒）
        dur: 观测时长（秒）
        orbit_max: 轨道周期内最大累计成像时间（秒）

    Returns:
        True 如果累计时间未超上限
    """
    p_idx = obs_start // ORBIT_PERIOD
    cum = orbit_sum.get(sat_key, {}).get(p_idx, 0)
    return cum + dur <= orbit_max


# ============================================================
# Status 8: 任务的有效窗口，天气，光照
# ============================================================
def check_mission_window(mi: dict, os_: int, oe: int) -> bool:
    """检查任务的时间窗口有效性

    观测起始和结束时间都必须落在任务的有效窗口内，
    且起始时间需满足天气窗口限制。

    Args:
        mi: 任务参数字典，含 validity / weather
        os_: 观测起始时间（秒）
        oe: 观测结束时间（秒）

    Returns:
        True 如果窗口有效
    """
    if not (_in_interval(os_, mi['validity']) and
            _in_interval(oe, mi['validity'])):
        return False
    if mi['weather'] and not _in_interval(os_, mi['weather']):
        return False
    return True


def check_sunlight(sat: dict, mi: dict, t: int) -> bool:
    """光学卫星光照检查（SAR 卫星始终通过）

    Args:
        sat: 卫星参数字典，含 type
        mi: 任务参数字典，含 sunlight（光照时间窗口）
        t: 观测时刻（秒）

    Returns:
        True 如果满足光照条件
    """
    if sat['type'] != 'opt':
        return True
    if not mi['sunlight']:
        return True
    return _in_interval(t, mi['sunlight'])


def check_time_interval(prev_obs: list, new_start: int, time_int_h: float) -> bool:
    """同一任务两次观测的时间间隔检查

    Args:
        prev_obs: 已调度的观测列表，每项格式为 (sn, stn, os_, oe, roll, ul, dl)
        new_start: 新观测起始时间（秒）
        time_int_h: 要求的最小时间间隔（小时），≤0 表示无限制

    Returns:
        True 如果满足时间间隔要求
    """
    if not prev_obs or time_int_h <= 0:
        return True
    last_end = prev_obs[-1][3]
    return new_start - last_end >= time_int_h * 3600




def _in_forbidden(sat: dict, roll: float) -> bool:
    """判断 roll 是否在 SAR 禁飞区

    Args:
        sat: 卫星参数字典，含 lf（禁飞区左边界）/ rf（禁飞区右边界）
        roll: 侧摆角（度）

    Returns:
        True 如果 lf ≤ roll ≤ rf（在禁飞区内）
    """
    if sat['lf'] is None or sat['rf'] is None:
        return False
    return sat['lf'] <= roll <= sat['rf']


def _nearest_valid(sat: dict, roll: float) -> float:
    """回退到最近的合法侧摆角（禁飞区外）

    Args:
        sat: 卫星参数字典，含 left_roll / right_roll / lf / rf
        roll: 原始侧摆角（度）

    Returns:
        最近的有效侧摆角（度）
    """
    candidates = [sat['left_roll'], sat['right_roll']]
    if sat['lf'] is not None:
        candidates.extend([sat['lf'], sat['rf']])
    valid = [v for v in candidates
             if sat['left_roll'] <= v <= sat['right_roll']
             and not _in_forbidden(sat, v)]
    valid.sort(key=lambda v: abs(v - roll))
    return valid[0] if valid else roll


def _in_interval(t: int, intervals: list) -> bool:
    """判断时间点 t 是否在任意区间内

    Args:
        t: 时间点（秒）
        intervals: 区间列表，[(a, b), ...]，空列表表示始终有效

    Returns:
        True 如果 t 在某个区间内（或 intervals 为空）
    """
    if not intervals:
        return True
    return any(a <= t <= b for a, b in intervals)
