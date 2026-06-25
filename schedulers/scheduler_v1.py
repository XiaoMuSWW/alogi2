#!/usr/bin/env python3
"""
scheduler.py - 调度算法核心
窗口扩展 → 约束过滤 → 任务分配
"""
from collections import defaultdict
from config import UL_DUR, ORBIT_PERIOD, STATIONS
from constraints import (
    check_startup,
    check_sat_insertion, check_station_conflict,
    check_orbit_sum, check_comm_ratio, calc_dl_dur, check_time_interval,
)

def schedule_missions(sat_names: list, mission_names: set,
                      sats: dict, miss: dict, expanded: dict,
                      stn_acc: dict, base_mode: bool = True,
                      sat_tl: dict = None, orbit_sum: dict = None,
                      stn_booked: dict = None, sched: dict = None) -> tuple:
    """任务优先贪心调度

    base_mode=True:  初始方案，仅 OBS→DL（成像同时数传）
    base_mode=False: 动态波次方案，UL→OBS→DL（数传在成像后完成）

    可选参数 sat_tl / orbit_sum / stn_booked / sched 用于波次状态继承。

    Args:
        sat_names: 参与调度的卫星名列表
        mission_names: 参与调度的任务名集合
        sats: 卫星参数字典 {name: {...}}
        miss: 任务参数字典 {name: {...}}
        expanded: 展开后的观测窗口 {sat: {mission: [(os_, oe, roll), ...]}}
        stn_acc: 地面站 access {sat: {station: [(start, end), ...]}}
        base_mode: True=初始方案, False=波次方案
        sat_tl: 继承的卫星时间线（仅波次）
        orbit_sum: 继承的轨道累计（仅波次）
        stn_booked: 继承的地面站占用（仅波次）
        sched: 继承的调度结果（仅波次）

    Returns:
        (entries_list, state_dict)
        entries_list: 调度条目列表，含 satellite_name / mission_name / start_time 等
        state_dict: 调度状态，含 sat_tl / orbit_sum / stn_booked / sched
    """
    # 状态初始化,支持波次继承
    if sat_tl is None:
        sat_tl = defaultdict(list)
    if orbit_sum is None:
        orbit_sum = defaultdict(lambda: defaultdict(float))
    if stn_booked is None:
        stn_booked = defaultdict(list)
    if sched is None:
        sched = defaultdict(list)

    # 收集所有候选窗口
    all_cands = []
    for sn in sat_names:
        for mn in mission_names:
            if mn not in expanded.get(sn, {}):
                continue
            mi = miss[mn]
            for os_, oe, roll in expanded[sn][mn]:
                dur = oe - os_ + 1
                dl_dur = calc_dl_dur(dur, mi)
                total_cost = dur + dl_dur  # OBS + DL 总时间消耗
                all_cands.append({
                    'sn': sn, 'mn': mn, 'os': os_, 'oe': oe, 'roll': roll,
                    'score': mi['score'],
                    'dur': dur,
                    'dl_dur': dl_dur,
                    'score_ratio': mi['score'] / max(total_cost, 1),  # 单位时间收益
                    'freq': mi['frequency'] if mi['frequency'] > 0 else 1,
                    'time_int': mi['time_interval'],
                })
    if not all_cands:
        state = {
            'sat_tl': sat_tl, 'orbit_sum': orbit_sum,
            'stn_booked': stn_booked, 'sched': sched,
        }
        return [], state

    # 按单位时间收益(OBS+DL)降序、时间升序
    all_cands.sort(key=lambda c: (-c['score_ratio'], c['os']))

    # 索引
    miss_idx = defaultdict(list)
    for idx, c in enumerate(all_cands):
        miss_idx[c['mn']].append(idx)

    # 按分数降序遍历任务
    for mn in sorted(miss_idx.keys(), key=lambda m: -miss[m]['score']):
        mi = miss[mn]
        needed = mi['frequency'] if mi['frequency'] > 0 else 1
        if needed <= 0:
            continue

        # 两轮调度：第一轮直接贪心，第二轮重试失败候选
        failed = []
        for idx in miss_idx[mn]:
            if len(sched.get(mn, [])) >= needed:
                break
            ok, entry, _ = _try_schedule(
                all_cands[idx], sats, miss, sat_tl, orbit_sum, stn_booked, sched, stn_acc, base_mode
            )
            if ok:
                _commit_schedule(entry, mn, miss, sat_tl,
                                 orbit_sum, stn_booked, sched, base_mode)
            else:
                if len(failed) < 5:
                    failed.append(all_cands[idx])

        # # 第二轮：重试失败的候选
        # for fc in failed:
        #     if len(sched.get(mn, [])) >= needed:
        #         break
        #     ok2, entry2, _ = _try_schedule(
        #         fc, sats, miss, sat_tl, orbit_sum, stn_booked, sched, stn_acc, base_mode
        #     )
        #     if ok2:
        #         _commit_schedule(entry2, mn, miss, sat_tl,
        #                          orbit_sum, stn_booked, sched, base_mode)

    # 构建输出（仅当前 mission_names，不含继承状态中的历史条目）
    entries = []
    for mn in mission_names:
        wins = sched.get(mn, [])
        if not wins:
            continue
        for sn, stn, os_, oe, roll, ul_info, dl_info in wins:
            ul_s, ul_e = ul_info
            dl_s, dl_e = dl_info
            # 上行（仅波次方案）
            if not base_mode:
                entries.append({
                    'satellite_name': sn, 'mission_name': mn, 'station_name': stn,
                    'start_time': ul_s, 'end_time': ul_e, 'roll_angle': None,
                })
            # 观测
            entries.append({
                'satellite_name': sn, 'mission_name': mn, 'station_name': '',
                'start_time': os_, 'end_time': oe, 'roll_angle': roll,
            })
            # 下行
            entries.append({
                'satellite_name': sn, 'mission_name': mn, 'station_name': stn,
                'start_time': dl_s, 'end_time': dl_e, 'roll_angle': None,
            })

    entries.sort(key=lambda e: (e['satellite_name'], e['start_time']))

    # 返回 当前条目 + 状态（供波次继承）
    state = {
        'sat_tl': sat_tl, 'orbit_sum': orbit_sum,
        'stn_booked': stn_booked, 'sched': sched,
    }
    return entries, state


def _commit_schedule(entry: dict, mn: str, miss: dict, sat_tl: dict,
                     orbit_sum: dict, stn_booked: dict,
                     sched: dict, base_mode: bool = True) -> None:
    """将成功调度的观测写入状态

    Args:
        entry: 调度条目，含 sn / os / oe / roll / stn / ul_s / ul_e / dl_s / dl_e
        mn: 任务名
        miss: 任务参数字典
        sat_tl: 卫星时间线（会被修改）
        orbit_sum: 轨道累计（会被修改）
        stn_booked: 地面站占用（会被修改）
        sched: 调度结果（会被修改）
        base_mode: True=初始方案, False=波次方案
    """
    sn = entry['sn']
    os_ = entry['os']
    oe = entry['oe']
    dur = oe - os_ + 1
    p_idx = os_ // ORBIT_PERIOD

    ul_s = entry.get('ul_s', os_ - UL_DUR)
    ul_e = entry.get('ul_e', ul_s + UL_DUR - 1)
    dl_s = entry.get('dl_s', os_)
    dl_e = entry.get('dl_e', dl_s + calc_dl_dur(dur, miss[mn]) - 1)

    # 卫星占用到 DL 结束，地面站覆盖 DL（base_mode）或 UL+DL（wave_mode）
    sat_end = dl_e
    stn_end = dl_e

    start_with_ul = not base_mode
    sat_tl[sn].append((ul_s, sat_end, entry['roll'], mn, start_with_ul))
    sat_tl[sn].sort(key=lambda x: x[0])
    # 轨道累计只计算成像时间
    orbit_sum[sn][p_idx] = orbit_sum[sn].get(p_idx, 0) + dur
    # 地面站占用范围
    stn_booked[entry['stn']].append((ul_s, stn_end, sn))
    stn_booked[entry['stn']].sort()

    sched[mn].append((
        sn, entry['stn'], os_, oe, entry['roll'],
        (ul_s, ul_e),
        (dl_s, dl_e),
    ))


def _try_schedule(c: dict, sats: dict, miss: dict,
                  sat_tl: dict, orbit_sum: dict, stn_booked: dict,
                  sched: dict, stn_acc: dict, base_mode: bool = True) -> tuple:
    """尝试调度单个候选窗口，逐项检查约束

    base_mode=True:  初始方案 OBS→DL（成像同时数传）
    base_mode=False: 波次方案 UL→OBS→DL 串行

    Args:
        c: 候选窗口，含 sn / mn / os / oe / roll / dur / dl_dur / score 等
        sats: 卫星参数字典
        miss: 任务参数字典
        sat_tl: 卫星时间线 {sat: [(ul_s, sat_end, roll, mn), ...]}
        orbit_sum: 轨道累计 {sat: {period_idx: cum_time}}
        stn_booked: 地面站占用 {stn: [(start, end, sat), ...]}
        sched: 已调度结果 {mn: [(sn, stn, os_, oe, roll, ul_info, dl_info), ...]}
        stn_acc: 地面站 access {sat: {stn: [(start, end), ...]}}
        base_mode: True=初始方案, False=波次方案

    Returns:
        (ok, entry_dict, _unused)
        ok: True 如果调度成功
        entry_dict: 成功时的调度条目，失败时为空字典
    """
    sn = c['sn']
    mn = c['mn']
    sa = sats[sn]
    mi = miss[mn]
    dur = c['oe'] - c['os'] + 1
    dl_dur = calc_dl_dur(dur, mi)

    # 窗口计算
    if base_mode:
        # 初始方案: OBS→DL，无 UL。DL 可与 OBS 同时开始
        ul_s = c['os']             # 卫星占用起点 = OBS 开始
        ul_e = c['os']             # 无独立 UL
        dl_s = c['os']             # DL ≥ OBS 开始（可同时）
        dl_e = dl_s + dl_dur - 1
        sat_occ_end = dl_e
        stn_range_start = dl_s
        stn_range_end = dl_e       # 地面站仅覆盖 DL
    else:
        # 波次方案: UL→OBS 串行, OBS∥DL (DL 可与 OBS 同时开始)
        ul_s = c['os'] - UL_DUR
        ul_e = ul_s + UL_DUR - 1
        dl_s = c['os']             # DL ≥ OBS 开始（可同时）
        dl_e = dl_s + dl_dur - 1
        sat_occ_end = dl_e
        stn_range_start = ul_s
        stn_range_end = dl_e       # 地面站覆盖 UL+DL

    # 开机时间检查
    if not check_startup(sa, dur):
        return False, {}, False

    # 时间范围检查
    t_start = c['os'] if base_mode else ul_s
    if t_start < 0 or sat_occ_end > 86399:
        return False, {}, False

    # 完整操作须在同一可用窗口
    validity = mi['validity']
    if validity:
        v_start = c['os'] if base_mode else ul_s
        in_one_window = any(a <= v_start and dl_e <= b for a, b in validity)
        if not in_one_window:
            return False, {}, False

    # 累计轨道运行时间
    if not check_orbit_sum(orbit_sum, sn, c['os'], dur, sa['orbit_max']):
        return False, {}, False

    # 香菱最小时间间隔
    if mi['type'] in ('p', 'ap'):
        prev = sched.get(mn, [])
        if not check_time_interval(prev, c['os'], mi['time_interval']):
            return False, {}, False

    # 数传/成像时间比
    if not check_comm_ratio(mi, dur, dl_dur):
        return False, {}, False

    # 卫星时间线插入检查（含点↔区域任务切换 & 测控/数传同型跳过）
    st_list = sat_tl.get(sn, [])
    start_with_ul = not base_mode
    if check_sat_insertion(sa, st_list, ul_s, sat_occ_end, c['roll'],
                           mi['type'], miss, start_with_ul=start_with_ul):
        return False, {}, False

    # 地面站可用性检查
    best_stn = None
    for stn in STATIONS:
        covered = any(ws <= stn_range_start and we >= stn_range_end
                      for ws, we in stn_acc.get(sn, {}).get(stn, []))
        if not covered:
            continue
        if check_station_conflict(stn_booked.get(stn, []), stn_range_start, stn_range_end, sn, sats):
            continue
        best_stn = stn
        break

    if best_stn is None:
        return False, {}, False

    return True, {
        'sn': sn, 'stn': best_stn, 'os': c['os'], 'oe': c['oe'], 'roll': c['roll'],
        'ul_s': ul_s, 'ul_e': ul_e, 'dl_s': dl_s, 'dl_e': dl_e, 'base_mode': base_mode,
    }, False
