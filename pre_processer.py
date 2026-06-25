from collections import defaultdict
from config import ORBIT_PERIOD, CLAMP_TOLERANCE_P, CLAMP_TOLERANCE_A
from constraints import (
    clamp_roll_area, obs_duration,
    check_mission_window, check_sunlight
)

def expand_windows(obs_acc: dict, sats: dict, miss: dict, offset: int = 0) -> dict:
    """将瞬时 access 点扩展为最小开机时长窗口，并过滤静态约束

    Args:
        obs_acc: 观测 access 数据 {sat: {mission: [(t, roll), ...]}}
        sats: 卫星参数字典 {name: {...}}
        miss: 任务参数字典 {name: {...}}
        offset: 时间偏移（秒），波次场景下只处理 [offset, 86400) 内的窗口

    Returns:
        expanded[sat][mission] = [(os_, oe, roll), ...]
    """
    expanded = defaultdict(lambda: defaultdict(list))
    for sn, mw in obs_acc.items():
        sa = sats.get(sn)
        if not sa:
            continue
        for mn, wlist in mw.items():
            mi = miss.get(mn)
            if not mi:
                continue
            dur = obs_duration(sa, mi)
            half = dur // 2
            for t, roll in wlist:
                if t < offset:
                    continue
                # 侧摆角处理：按任务类型使用不同容差
                clamped = clamp_roll_area(sa, roll)
                # 点目标 0.1° / 区域目标 1° 夹持容差
                tol = CLAMP_TOLERANCE_P if mi['type'] in ('p', 'ap') else CLAMP_TOLERANCE_A
                if abs(clamped - roll) > tol:
                    continue
                use_roll = clamped
                # 时间窗口构建
                os_ = max(t - half, offset)
                oe = os_ + dur - 1
                if oe > 86399:
                    oe = 86399
                    os_ = oe - dur + 1
                if os_ < offset:
                    continue
                # 任务约束
                if not check_mission_window(mi, os_, oe):
                    continue
                if not check_sunlight(sa, mi, os_):
                    continue
                expanded[sn][mn].append((os_, oe, use_roll))

    #后过滤：剔除候选窗口不足以完成任务的 mission
    all_missions = set()
    for sn in expanded:
        all_missions.update(expanded[sn].keys())
    for mn in all_missions:
        mi = miss.get(mn)
        if not mi:
            continue
        freq = mi['frequency'] if mi['frequency'] > 0 else 1
        total = sum(len(expanded.get(s, {}).get(mn, [])) for s in expanded)
        if total < freq:
            for s in list(expanded.keys()):
                if mn in expanded[s]:
                    del expanded[s][mn]

    return expanded

def filter_state_before(state: dict, boundary: int) -> dict:
    """过滤继承状态：仅保留在 boundary 之前完全结束的资源占用

    波次继承规则：前驱时间段 [0, boundary) 内的已分配资源被锁定，
    波次任务只能在 [boundary, 86400) 内自由调度，可覆盖基础方案在
    该时段内的原分配。

    Args:
        state: 调度状态字典，含 sat_tl / orbit_sum / stn_booked / sched
        boundary: 时间边界（秒），保留 end < boundary 的条目

    Returns:
        过滤后的状态深拷贝
    """
    from copy import deepcopy
    filtered = deepcopy(state)

    # sat_tl: 保留 sat_end < boundary 的条目（在前驱时段内完全结束）
    for sn in list(filtered['sat_tl'].keys()):
        filtered['sat_tl'][sn] = [
            e for e in filtered['sat_tl'][sn] if e[1] < boundary
        ]
        if not filtered['sat_tl'][sn]:
            del filtered['sat_tl'][sn]

    # stn_booked: 保留 end < boundary 的条目
    for stn in list(filtered['stn_booked'].keys()):
        filtered['stn_booked'][stn] = [
            e for e in filtered['stn_booked'][stn] if e[1] < boundary
        ]
        if not filtered['stn_booked'][stn]:
            del filtered['stn_booked'][stn]

    # orbit_sum: 保留 boundary 之前的轨道周期累计
    boundary_period = boundary // ORBIT_PERIOD
    for sn in list(filtered['orbit_sum'].keys()):
        filtered['orbit_sum'][sn] = {
            p: v for p, v in filtered['orbit_sum'][sn].items()
            if p < boundary_period
        }
        if not filtered['orbit_sum'][sn]:
            del filtered['orbit_sum'][sn]

    # sched: 保留 oe < boundary 的条目（前驱时段内完成的观测）
    for mn in list(filtered['sched'].keys()):
        filtered['sched'][mn] = [
            e for e in filtered['sched'][mn] if e[3] < boundary
        ]
        if not filtered['sched'][mn]:
            del filtered['sched'][mn]

    return filtered
