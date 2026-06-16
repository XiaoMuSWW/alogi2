"""
scheduler_v3.py - 基于 CP‑SAT 的最优调度器，使用贪心法完成关于地面站的排班
传入CP-SAT的约束中剔除了地面站,加快求解速度的同时提升了解的质量
"""
from collections import defaultdict
from ortools.sat.python import cp_model

from config import UL_DUR, ORBIT_PERIOD, STATIONS
from constraints import (
    calc_dl_dur, sat_timeline_gap, check_station_conflict,
)

# ================================================================
# CP‑SAT 参数配置
# ================================================================
MAX_TIME_SECONDS = 30      # 单次求解最大运行时间（秒）
SCORE_SCALE = 100          # 将浮点分数放大为整数，以适配 CP‑SAT
NUM_WORKERS = 4            # 并行搜索线程数


def schedule_missions(sat_names, mission_names, sats, miss, expanded,
                      stn_acc, base_mode=True, sat_tl=None, orbit_sum=None,
                      stn_booked=None, sched=None):
    """
    CP‑SAT 最优调度主函数

    base_mode=True  : 基线模式，仅包含 OBS → DL
    base_mode=False : 波次模式，包含 UL → OBS → DL

    算法流程：
    阶段 1：收集候选观测窗口并进行预筛选
    阶段 2：构建 CP‑SAT 模型（变量、约束、目标函数）
    阶段 3：调用求解器并获取结果
    阶段 4：地面站后处理（贪心分配）
    阶段 5：生成最终调度条目

    返回：
        (entries_list, state_dict)
    """
    # ============================================================
    # 阶段 0：状态初始化
    # ============================================================
    if sat_tl is None:
        sat_tl = defaultdict(list)
    if orbit_sum is None:
        orbit_sum = defaultdict(lambda: defaultdict(float))
    if stn_booked is None:
        stn_booked = defaultdict(list)
    if sched is None:
        sched = defaultdict(list)

    # 统计每个任务已被安排的次数
    sched_count = defaultdict(int)
    for mn in mission_names:
        sched_count[mn] = len(sched.get(mn, []))

    # ============================================================
    # 阶段 1：候选观测窗口收集与预筛选
    # ============================================================
    all_cands = []           # 所有可行候选窗口
    miss_idx = defaultdict(list)  # 任务名 -> 候选索引列表

    for sn in sat_names:
        sa = sats.get(sn)
        if not sa:
            continue

        for mn in mission_names:
            if mn not in expanded.get(sn, {}):
                continue

            mi = miss[mn]
            for os_, oe, roll in expanded[sn][mn]:
                dur = oe - os_ + 1
                dl_dur = calc_dl_dur(dur, mi)

                # 计算卫星占用时间窗
                if base_mode:
                    ul_s = os_
                    sat_occ_end = os_ + dl_dur - 1
                else:
                    ul_s = os_ - UL_DUR
                    sat_occ_end = os_ + dl_dur - 1

                # ---------- 预筛选：卫星时间线冲突 ----------
                st_list = sat_tl.get(sn, [])
                conflict = False

                # 二分查找插入位置
                lo, hi = 0, len(st_list)
                while lo < hi:
                    mid = (lo + hi) // 2
                    if st_list[mid][0] < ul_s:
                        lo = mid + 1
                    else:
                        hi = mid
                ins = lo

                # 检查前驱任务
                if ins > 0:
                    pred = st_list[ins - 1]
                    gap = sat_timeline_gap(sa, pred[2], roll)
                    if pred[1] + gap >= ul_s:
                        conflict = True

                # 检查后继任务
                if not conflict and ins < len(st_list):
                    succ = st_list[ins]
                    gap = sat_timeline_gap(sa, roll, succ[2])
                    if sat_occ_end + gap >= succ[0]:
                        conflict = True

                if conflict:
                    continue

                # ---------- 预筛选：轨道累计时长 ----------
                p_idx = os_ // ORBIT_PERIOD
                cum = orbit_sum.get(sn, {}).get(p_idx, 0)
                if cum + dur > sa.get('orbit_max', 600):
                    continue

                # ---------- 预筛选：任务频次限制 ----------
                freq = mi['frequency'] if mi['frequency'] > 0 else 1
                if sched_count[mn] >= freq:
                    continue

                # ---------- 预筛选：p / ap 任务最小时间间隔 ----------
                if mi['type'] in ('p', 'ap') and mi.get('time_interval', 0) > 0:
                    prev_obs = sched.get(mn, [])
                    if prev_obs:
                        last_end = prev_obs[-1][3]
                        if os_ - last_end < mi['time_interval'] * 3600:
                            continue

                idx = len(all_cands)
                all_cands.append(dict(
                    sn=sn, mn=mn, os=os_, oe=oe, roll=roll,
                    dur=dur, dl_dur=dl_dur,
                    score=mi['score'],
                    type=mi['type'],
                    ul_s=ul_s, sat_occ_end=sat_occ_end,
                ))
                miss_idx[mn].append(idx)

    if not all_cands:
        state = dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                     stn_booked=stn_booked, sched=sched)
        return [], state

    # ============================================================
    # 阶段 2：构建 CP‑SAT 模型
    # ============================================================
    model = cp_model.CpModel()

    # 决策变量：x[i] = 1 表示选择第 i 个候选窗口
    x = {}
    for idx in range(len(all_cands)):
        x[idx] = model.NewBoolVar(f'x_{idx}')

    # ---- 目标函数：最大化总评分 ----
    objective_terms = []
    for idx, c in enumerate(all_cands):
        weight = int(round(c['score'] * SCORE_SCALE))
        objective_terms.append(x[idx] * weight)
    model.Maximize(sum(objective_terms))

    # ---- 约束 1：任务频次上限 ----
    for mn in mission_names:
        mi = miss[mn]
        freq = mi['frequency'] if mi['frequency'] > 0 else 1
        remaining = freq - sched_count.get(mn, 0)
        if remaining <= 0:
            continue
        idxs = miss_idx.get(mn, [])
        if idxs:
            model.Add(sum(x[i] for i in idxs) <= remaining)

    # ---- 约束 2：卫星时间线互斥 ----
    sat_cand_idxs = defaultdict(list)
    for idx, c in enumerate(all_cands):
        sat_cand_idxs[c['sn']].append(idx)

    for sn, idxs in sat_cand_idxs.items():
        idxs_sorted = sorted(idxs, key=lambda i: all_cands[i]['ul_s'])
        sa = sats[sn]

        # 最大可能姿态切换时间
        max_gap = max(
            int(sa.get('trantime_sar', 20)) +
            int(sa.get('reversal_time', 0)) +
            int(sa.get('trantime_cc', 20)),
            int(sa.get('trantime_20', 60)) +
            int(sa.get('trantime_cc', 20)),
        )

        for i_pos in range(len(idxs_sorted)):
            idx_i = idxs_sorted[i_pos]
            ci = all_cands[idx_i]

            for j_pos in range(i_pos + 1, len(idxs_sorted)):
                idx_j = idxs_sorted[j_pos]
                cj = all_cands[idx_j]

                # 提前退出：即使最宽松的间隔也不可能冲突
                if ci['sat_occ_end'] + max_gap < cj['ul_s']:
                    break

                gap_ij = sat_timeline_gap(sa, ci['roll'], cj['roll'])
                gap_ji = sat_timeline_gap(sa, cj['roll'], ci['roll'])

                i_before_j = ci['sat_occ_end'] + gap_ij < cj['ul_s']
                j_before_i = cj['sat_occ_end'] + gap_ji < ci['ul_s']

                if not i_before_j and not j_before_i:
                    model.Add(x[idx_i] + x[idx_j] <= 1)

    # ---- 约束 3：轨道累计观测时长 ----
    orbit_bins = defaultdict(lambda: defaultdict(list))
    for idx, c in enumerate(all_cands):
        p_idx = c['os'] // ORBIT_PERIOD
        orbit_bins[c['sn']][p_idx].append(idx)

    for sn, periods in orbit_bins.items():
        orbit_max = sats[sn].get('orbit_max', 600)
        inherited_usage = orbit_sum.get(sn, {})

        for p_idx, idxs in periods.items():
            effective_max = orbit_max - inherited_usage.get(p_idx, 0)
            if effective_max <= 0:
                for i in idxs:
                    model.Add(x[i] == 0)
            else:
                model.Add(
                    sum(all_cands[i]['dur'] * x[i] for i in idxs)
                    <= int(effective_max)
                )

    # ---- 约束 4：p / ap 任务最小时间间隔 ----
    for mn in mission_names:
        mi = miss[mn]
        if mi['type'] not in ('p', 'ap'):
            continue

        time_int_h = mi.get('time_interval', 0)
        if time_int_h <= 0:
            continue

        time_int_s = int(time_int_h * 3600)
        idxs = miss_idx.get(mn, [])
        idxs_sorted = sorted(idxs, key=lambda i: all_cands[i]['os'])

        for i_pos in range(len(idxs_sorted)):
            idx_i = idxs_sorted[i_pos]
            for j_pos in range(i_pos + 1, len(idxs_sorted)):
                idx_j = idxs_sorted[j_pos]
                if all_cands[idx_j]['os'] - all_cands[idx_i]['os'] >= time_int_s:
                    break
                model.Add(x[idx_i] + x[idx_j] <= 1)

    # ============================================================
    # 阶段 3：求解
    # ============================================================
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = MAX_TIME_SECONDS
    solver.parameters.num_workers = NUM_WORKERS
    solver.parameters.log_search_progress = False

    status = solver.Solve(model)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        state = dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                     stn_booked=stn_booked, sched=sched)
        return [], state

    # 提取被选中的候选窗口
    selected = [
        idx for idx in range(len(all_cands))
        if solver.Value(x[idx]) == 1
    ]

    # ============================================================
    # 阶段 4：地面站后处理（贪心分配）
    # ============================================================
    selected.sort(key=lambda i: -all_cands[i]['score'])

    committed = []
    for idx in selected:
        c = all_cands[idx]
        sn, mn = c['sn'], c['mn']
        sa = sats[sn]
        mi = miss[mn]
        dur = c['dur']
        dl_dur = c['dl_dur']

        # 重新计算各阶段时间窗
        if base_mode:
            ul_s = c['os']
            ul_e = c['os']
            dl_s = c['os']
            dl_e = dl_s + dl_dur - 1
            sat_occ_end = dl_e
            stn_range_start = dl_s
            stn_range_end = dl_e
        else:
            ul_s = c['os'] - UL_DUR
            ul_e = ul_s + UL_DUR - 1
            dl_s = c['os']             # DL >= OBS start (concurrent)
            dl_e = dl_s + dl_dur - 1
            sat_occ_end = dl_e
            stn_range_start = ul_s
            stn_range_end = dl_e

        # 再次检查卫星时间线冲突
        st_list = sat_tl.get(sn, [])
        conflict = False

        lo, hi = 0, len(st_list)
        while lo < hi:
            mid = (lo + hi) // 2
            if st_list[mid][0] < ul_s:
                lo = mid + 1
            else:
                hi = mid
        ins = lo

        if ins > 0:
            pred = st_list[ins - 1]
            gap = sat_timeline_gap(sa, pred[2], c['roll'])
            if pred[1] + gap >= ul_s:
                conflict = True

        if not conflict and ins < len(st_list):
            succ = st_list[ins]
            gap = sat_timeline_gap(sa, c['roll'], succ[2])
            if sat_occ_end + gap >= succ[0]:
                conflict = True

        if conflict:
            continue

        # 地面站分配
        best_stn = None
        for stn in STATIONS:
            covered = any(
                ws <= stn_range_start and we >= stn_range_end
                for ws, we in stn_acc.get(sn, {}).get(stn, [])
            )
            if not covered:
                continue
            if check_station_conflict(
                stn_booked.get(stn, []),
                stn_range_start,
                stn_range_end,
                sn,
                sats
            ):
                continue
            best_stn = stn
            break

        if best_stn is None:
            continue

        # 提交到状态
        entry = dict(
            sn=sn, stn=best_stn, os=c['os'], oe=c['oe'], roll=c['roll'],
            ul_s=ul_s, ul_e=ul_e, dl_s=dl_s, dl_e=dl_e,
            base_mode=base_mode,
        )

        # 更新卫星时间线
        sat_tl[sn].append((ul_s, sat_occ_end, c['roll'], mn))
        sat_tl[sn].sort(key=lambda e: e[0])

        # 更新轨道累计时长
        p_idx = c['os'] // ORBIT_PERIOD
        orbit_sum[sn][p_idx] = orbit_sum[sn].get(p_idx, 0) + dur

        # 更新地面站占用
        stn_booked[best_stn].append((ul_s, sat_occ_end, sn))
        stn_booked[best_stn].sort()

        # 更新任务调度记录
        sched[mn].append((
            sn, best_stn, c['os'], c['oe'], c['roll'],
            (ul_s, ul_e), (dl_s, dl_e),
        ))
        committed.append((entry, mn))

    # ============================================================
    # 阶段 5：生成最终输出条目
    # ============================================================
    entries = []
    for mn in mission_names:
        wins = sched.get(mn, [])
        if not wins:
            continue

        for sn, stn, os_, oe, roll, ul_info, dl_info in wins:
            ul_s, ul_e = ul_info
            dl_s, dl_e = dl_info

            # 上行链路（仅波次模式）
            if not base_mode:
                entries.append(dict(
                    satellite_name=sn,
                    mission_name=mn,
                    station_name=stn,
                    start_time=ul_s,
                    end_time=ul_e,
                    roll_angle=None
                ))

            # 观测任务
            entries.append(dict(
                satellite_name=sn,
                mission_name=mn,
                station_name="",
                start_time=os_,
                end_time=oe,
                roll_angle=roll
            ))

            # 下行链路
            entries.append(dict(
                satellite_name=sn,
                mission_name=mn,
                station_name=stn,
                start_time=dl_s,
                end_time=dl_e,
                roll_angle=None
            ))

    entries.sort(key=lambda e: (e["satellite_name"], e["start_time"]))

    state = dict(
        sat_tl=sat_tl,
        orbit_sum=orbit_sum,
        stn_booked=stn_booked,
        sched=sched
    )
    return entries, state