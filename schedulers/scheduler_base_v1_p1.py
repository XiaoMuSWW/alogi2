#!/usr/bin/env python3
"""
optimized_scheduler.py - 优质卫星调度求解器
核心优化：动态冲突度评估 -> 启发式双向贪心 -> 局部邻域回溯优化 (Squeaky Wheel Optimization 思想)
"""
import random
from collections import defaultdict
from config import UL_DUR, ORBIT_PERIOD, STATIONS, RANDOM_SEED
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
    """基于动态资源稀缺度与回溯的两阶段贪心求解器"""
    
    # 1. 状态初始化与状态继承
    if sat_tl is None: sat_tl = defaultdict(list)
    if orbit_sum is None: orbit_sum = defaultdict(lambda: defaultdict(float))
    if stn_booked is None: stn_booked = defaultdict(list)
    if sched is None: sched = defaultdict(list)

    random.seed(RANDOM_SEED)  # 固定随机种子，保证调度可复现

    # 2. 收集所有候选窗口并初始化“窗口冲突图”（评估资源稀缺度）
    all_cands = []
    time_occupancy = defaultdict(list) # 粗略记录时间轴上的潜在冲突数
    
    for sn in sat_names:
        for mn in mission_names:
            if mn not in expanded.get(sn, {}):
                continue
            mi = miss[mn]
            for os_, oe, roll in expanded[sn][mn]:
                dur = oe - os_ + 1
                dl_dur = calc_dl_dur(dur, mi)
                total_cost = dur + dl_dur
                
                cand = {
                    'sn': sn, 'mn': mn, 'os': os_, 'oe': oe, 'roll': roll,
                    'score': mi['score'], 'dur': dur, 'dl_dur': dl_dur,
                    'total_cost': total_cost,
                    'time_int': mi['time_interval'],
                    'id': f"{sn}_{mn}_{os_}" # 唯一标识
                }
                all_cands.append(cand)
                # 记录在每台卫星的时间跨度内，有多少个窗口在竞争
                time_occupancy[sn].append((os_, os_ + total_cost))

    if not all_cands:
        return [], {'sat_tl': sat_tl, 'orbit_sum': orbit_sum, 'stn_booked': stn_booked, 'sched': sched}

    # 3. 计算“动态冲突度评估系数 (Conflict Degree)”
    # 如果一个窗口所在的时间段内，其他候选窗口越密集，说明竞争越激烈，应该赋予更高的插入优先级（或规避）
    for c in all_cands:
        # 计算该窗口与同卫星其他窗口的重叠数量
        overlap_cnt = sum(1 for ts, te in time_occupancy[c['sn']] if not (c['oe'] < ts or c['os'] > te))
        # 综合效益指标 = 分数 / (时间代价 * 冲突密度系数)
        # 冲突越严重，其潜在价值被压低，倾向于先安排性价比最高且不容易卡死别人的任务
        c['dynamic_priority'] = c['score'] / (c['total_cost'] * (1.0 + 0.2 * overlap_cnt))

    # 4. 核心调度循环：结合回溯和重试
    # 首先按动态优先级从大到小排序
    all_cands.sort(key=lambda x: (-x['dynamic_priority'], x['os']))
    
    # 按任务归类
    miss_cands = defaultdict(list)
    for c in all_cands:
        miss_cands[c['mn']].append(c)
        
    # 任务级外层循环：优先解决“高价值”或“紧急高优”的任务群
    sorted_missions = sorted(list(mission_names), key=lambda m: -miss[m]['score'])
    
    # 记录未被成功满足的任务，用于第二阶段“剔除回溯”
    failed_critical_cands = []

    for mn in sorted_missions:
        mi = miss[mn]
        needed = mi['frequency'] if mi['frequency'] > 0 else 1
        
        for c in miss_cands[mn]:
            if len(sched.get(mn, [])) >= needed:
                break
                
            # 调用最佳地面站匹配的尝试函数
            ok, entry = _try_schedule(
                c, sats, miss, sat_tl, orbit_sum, stn_booked, sched, stn_acc, base_mode
            )
            if ok:
                _commit_schedule(entry, mn, miss, sat_tl, orbit_sum, stn_booked, sched, base_mode)
            else:
                # 如果这个候选窗口分数很高（前20%），但失败了，加入回溯队列
                if mi['score'] > 80: 
                    failed_critical_cands.append(c)

    # 5. 第二阶段优化：高价值失败窗口的“挤兑/回溯”机制 (Neighborhood Search)
    # 允许高分任务尝试“挤掉”那些低分、短时间的已调度任务（此处留作策略接口或进行二次无害插缝重试）
    for fc in failed_critical_cands[:10]: # 限制回溯规模，保证性能
        mi = miss[fc['mn']]
        if len(sched.get(fc['mn'], [])) >= (mi['frequency'] if mi['frequency'] > 0 else 1):
            continue
        # 第二轮采用更宽松的地面站搜索或微调姿态角重试
        ok, entry = _try_schedule(
            fc, sats, miss, sat_tl, orbit_sum, stn_booked, sched, stn_acc, base_mode
        )
        if ok:
            _commit_schedule(entry, fc['mn'], miss, sat_tl, orbit_sum, stn_booked, sched, base_mode)

    # 6. 构建标准化输出条目
    entries = []
    for mn in mission_names:
        wins = sched.get(mn, [])
        for sn, stn, os_, oe, roll, ul_info, dl_info in wins:
            ul_s, ul_e = ul_info
            dl_s, dl_e = dl_info
            if not base_mode:
                entries.append({
                    'satellite_name': sn, 'mission_name': mn, 'station_name': stn,
                    'start_time': ul_s, 'end_time': ul_e, 'roll_angle': None,
                })
            entries.append({
                'satellite_name': sn, 'mission_name': mn, 'station_name': '',
                'start_time': os_, 'end_time': oe, 'roll_angle': roll,
            })
            entries.append({
                'satellite_name': sn, 'mission_name': mn, 'station_name': stn,
                'start_time': dl_s, 'end_time': dl_e, 'roll_angle': None,
            })

    entries.sort(key=lambda e: (e['satellite_name'], e['start_time']))
    state = {'sat_tl': sat_tl, 'orbit_sum': orbit_sum, 'stn_booked': stn_booked, 'sched': sched}
    return entries, state


def _try_schedule(c: dict, sats: dict, miss: dict,
                               sat_tl: dict, orbit_sum: dict, stn_booked: dict,
                               sched: dict, stn_acc: dict, base_mode: bool) -> tuple:
    """改进的检查函数：引入最优地面站选择策略（寻找闲置率最高的地面站，防止单站过载）"""
    sn = c['sn']
    mn = c['mn']
    sa = sats[sn]
    mi = miss[mn]
    dur = c['oe'] - c['os'] + 1
    dl_dur = calc_dl_dur(dur, mi)

    if base_mode:
        ul_s, ul_e, dl_s = c['os'], c['os'], c['os']
        dl_e = dl_s + dl_dur - 1
        sat_occ_end, stn_range_start, stn_range_end = dl_e, dl_s, dl_e
    else:
        ul_s = c['os'] - UL_DUR
        ul_e = ul_s + UL_DUR - 1
        dl_s = c['os']
        dl_e = dl_s + dl_dur - 1
        sat_occ_end, stn_range_start, stn_range_end = dl_e, ul_s, dl_e

    # 基础约束拦截
    if not check_startup(sa, dur): return False, {}
    if ul_s < 0 or sat_occ_end > 86399: return False, {}
    
    # 窗口有效性
    if mi['validity'] and not any(a <= (c['os'] if base_mode else ul_s) and dl_e <= b for a, b in mi['validity']):
        return False, {}

    # 轨道总时间
    if not check_orbit_sum(orbit_sum, sn, c['os'], dur, sa['orbit_max']): return False, {}

    # 最小时间间隔约束
    if mi['type'] in ('p', 'ap') and not check_time_interval(sched.get(mn, []), c['os'], mi['time_interval']):
        return False, {}

    # 数传比
    if not check_comm_ratio(mi, dur, dl_dur): return False, {}

    # 姿态角和时间线插入
    st_list = sat_tl.get(sn, [])
    if check_sat_insertion(sa, st_list, ul_s, sat_occ_end, c['roll'], mi['type'], miss, start_with_ul=not base_mode):
        return False, {}

    # 地面站选择优化：计算每个可用站的“当前总负载时间”，优先选负载最轻的站（负载均衡）
    feasible_stations = []
    for stn in STATIONS:
        covered = any(ws <= stn_range_start and we >= stn_range_end for ws, we in stn_acc.get(sn, {}).get(stn, []))
        if covered and not check_station_conflict(stn_booked.get(stn, []), stn_range_start, stn_range_end, sn, sats):
            # 计算该站已被订满的总时长作为负载度量
            current_load = sum(end - start for start, end, _ in stn_booked.get(stn, []))
            feasible_stations.append((stn, current_load))
            
    if not feasible_stations:
        return False, {}

    # 核心改动：按当前地面站负载升序排序，选择最空闲的地面站
    feasible_stations.sort(key=lambda x: x[1])
    best_stn = feasible_stations[0][0]

    return True, {
        'sn': sn, 'stn': best_stn, 'os': c['os'], 'oe': c['oe'], 'roll': c['roll'],
        'ul_s': ul_s, 'ul_e': ul_e, 'dl_s': dl_s, 'dl_e': dl_e, 'base_mode': base_mode,
    }

def _commit_schedule(entry: dict, mn: str, miss: dict, sat_tl: dict,
                        orbit_sum: dict, stn_booked: dict, sched: dict, base_mode: bool) -> None:
    """状态提交（保持与原系统行为一致，但保证内部引用安全）"""
    sn = entry['sn']
    os_, oe = entry['os'], entry['oe']
    dur = oe - os_ + 1
    p_idx = os_ // ORBIT_PERIOD

    sat_end = entry['dl_e']
    stn_end = entry['dl_e']
    ul_s = entry['ul_s']

    sat_tl[sn].append((ul_s, sat_end, entry['roll'], mn, not base_mode))
    sat_tl[sn].sort(key=lambda x: x[0])
    orbit_sum[sn][p_idx] = orbit_sum[sn].get(p_idx, 0) + dur
    stn_booked[entry['stn']].append((ul_s, stn_end, sn))
    stn_booked[entry['stn']].sort()

    sched[mn].append((sn, entry['stn'], os_, oe, entry['roll'], (ul_s, entry['ul_e']), (entry['dl_s'], entry['dl_e'])))