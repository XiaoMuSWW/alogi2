"""
scheduler_v2.py - 基于 CP‑SAT 的最优调度器

使用 Google OR‑Tools 的 CP‑SAT 求解器，对卫星观测任务进行全局最优（或近似最优）调度，
替代 scheduler.py 中原有的贪心启发式算法。
地面站约束已纳入 CP‑SAT 模型内部，消除后处理阶段的可行解丢失。

相比 v1 的主要优势：
- 全局优化，而非按时间顺序的局部贪心决策
- 同时考虑所有候选观测窗口（卫星时间线 + 地面站分配）
- 在给定时间限制内可获得可证明的最优性界
"""
from collections import defaultdict
from ortools.sat.python import cp_model

from config import UL_DUR, ORBIT_PERIOD, STATIONS
from constraints import calc_dl_dur, sat_timeline_gap


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
    CP‑SAT 最优调度主函数（含地面站约束）

    base_mode=True  : 基线模式，仅包含 OBS → DL
    base_mode=False : 波次模式，包含 UL → OBS → DL

    算法流程：
    阶段 1：收集候选观测窗口并进行预筛选（卫星时间线 + 地面站）
    阶段 2：构建 CP‑SAT 模型（变量、约束、目标函数）
    阶段 3：调用求解器并获取结果（含地面站分配）
    阶段 4：生成最终调度条目

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

                # 计算时间窗
                if base_mode:
                    ul_s = os_
                    sat_occ_end = os_ + dl_dur - 1
                    stn_start = os_
                    stn_end = os_ + dl_dur - 1
                else:
                    ul_s = os_ - UL_DUR
                    sat_occ_end = os_ + dl_dur - 1
                    stn_start = os_ - UL_DUR
                    stn_end = os_ + dl_dur - 1

                # ---------- 预筛选：卫星时间线冲突 ----------
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
                    gap = sat_timeline_gap(sa, pred[2], roll)
                    if pred[1] + gap >= ul_s:
                        conflict = True
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

                # ---------- 预筛选：地面站可用性 ----------
                stn_opts = []  # 可用地面站索引列表
                for g_idx, stn in enumerate(STATIONS):
                    covered = any(
                        ws <= stn_start and we >= stn_end
                        for ws, we in stn_acc.get(sn, {}).get(stn, [])
                    )
                    if not covered:
                        continue
                    # 检查与继承状态的地面站冲突
                    stn_conflict = False
                    for bs, be, booked_sat in stn_booked.get(stn, []):
                        if booked_sat != sn:
                            cc = int(sats.get(booked_sat, {}).get('trantime_cc', 0) or 0)
                            if stn_start <= be + cc and bs <= stn_end + cc:
                                stn_conflict = True
                                break
                        else:
                            if stn_start <= be and bs <= stn_end:
                                stn_conflict = True
                                break
                    if not stn_conflict:
                        stn_opts.append(g_idx)

                # 至少需要一个可用地面站
                if not stn_opts:
                    continue

                idx = len(all_cands)
                all_cands.append(dict(
                    sn=sn, mn=mn, os=os_, oe=oe, roll=roll,
                    dur=dur, dl_dur=dl_dur,
                    score=mi['score'],
                    type=mi['type'],
                    ul_s=ul_s, sat_occ_end=sat_occ_end,
                    stn_start=stn_start, stn_end=stn_end,
                    stn_opts=stn_opts,
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

    # ---- 地面站分配变量：y[i][g] = 1 表示候选 i 使用地面站 g ----
    y = {}
    for idx, c in enumerate(all_cands):
        for g in c['stn_opts']:
            y[(idx, g)] = model.NewBoolVar(f'y_{idx}_{g}')

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

    # ---- 约束 5：地面站通道约束 ----
    # 5a. 通道选择：被选中的候选必须分配到恰好一个可用地面站
    for idx, c in enumerate(all_cands):
        if c['stn_opts']:
            model.Add(sum(y[(idx, g)] for g in c['stn_opts']) == x[idx])

    # 5b. 地面站时间互斥
    for g_idx, stn in enumerate(STATIONS):
        stn_cands = [
            idx for idx, c in enumerate(all_cands)
            if g_idx in c['stn_opts']
        ]
        # 按地面站占用起始时间排序
        stn_cands.sort(key=lambda i: all_cands[i]['stn_start'])

        for i_pos in range(len(stn_cands)):
            idx_i = stn_cands[i_pos]
            ci = all_cands[idx_i]

            for j_pos in range(i_pos + 1, len(stn_cands)):
                idx_j = stn_cands[j_pos]
                cj = all_cands[idx_j]

                # 提前退出：窗口完全不重叠
                if ci['stn_end'] < cj['stn_start']:
                    break

                # 计算地面站间隔
                if ci['sn'] == cj['sn']:
                    cc_i = 0
                    cc_j = 0
                else:
                    cc_i = int(sats.get(ci['sn'], {}).get('trantime_cc', 20))
                    cc_j = int(sats.get(cj['sn'], {}).get('trantime_cc', 20))

                i_before_j = ci['stn_end'] + cc_i < cj['stn_start']
                j_before_i = cj['stn_end'] + cc_j < ci['stn_start']

                if not i_before_j and not j_before_i:
                    # 两者在 g 上冲突：不能同时分配到此站
                    model.Add(y[(idx_i, g_idx)] + y[(idx_j, g_idx)] <= 1)

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

    # ============================================================
    # 阶段 4：提取解并提交状态
    # ============================================================
    # 按分数降序提交,高价值任务优先
    selected = [
        idx for idx in range(len(all_cands))
        if solver.Value(x[idx]) == 1
    ]
    selected.sort(key=lambda i: -all_cands[i]['score'])

    for idx in selected:
        c = all_cands[idx]
        sn, mn = c['sn'], c['mn']
        sa = sats[sn]
        mi = miss[mn]
        dur, dl_dur = c['dur'], c['dl_dur']

        # 确定分配的地面站
        assigned_stn = None
        for g in c['stn_opts']:
            if solver.Value(y[(idx, g)]) == 1:
                assigned_stn = STATIONS[g]
                break
        if assigned_stn is None:
            continue  # 不应该发生（模型正确时）

        # 计算各阶段时间窗
        if base_mode:
            ul_s = c['os']
            ul_e = c['os']
            dl_s = c['os']
            dl_e = dl_s + dl_dur - 1
            sat_occ_end = dl_e
        else:
            ul_s = c['os'] - UL_DUR
            ul_e = ul_s + UL_DUR - 1
            dl_s = c['os']
            dl_e = dl_s + dl_dur - 1
            sat_occ_end = dl_e

        # 提交到卫星时间线
        sat_tl[sn].append((ul_s, sat_occ_end, c['roll'], mn))
        sat_tl[sn].sort(key=lambda e: e[0])

        # 更新轨道累计时长
        p_idx = c['os'] // ORBIT_PERIOD
        orbit_sum[sn][p_idx] = orbit_sum[sn].get(p_idx, 0) + dur

        # 更新地面站占用
        stn_booked[assigned_stn].append((c['stn_start'], c['stn_end'], sn))
        stn_booked[assigned_stn].sort()

        # 更新任务调度记录
        sched[mn].append((
            sn, assigned_stn, c['os'], c['oe'], c['roll'],
            (ul_s, ul_e), (dl_s, dl_e),
        ))

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

            if not base_mode:
                entries.append(dict(
                    satellite_name=sn, mission_name=mn, station_name=stn,
                    start_time=ul_s, end_time=ul_e, roll_angle=None,
                ))
            entries.append(dict(
                satellite_name=sn, mission_name=mn, station_name="",
                start_time=os_, end_time=oe, roll_angle=roll,
            ))
            entries.append(dict(
                satellite_name=sn, mission_name=mn, station_name=stn,
                start_time=dl_s, end_time=dl_e, roll_angle=None,
            ))

    entries.sort(key=lambda e: (e["satellite_name"], e["start_time"]))

    state = dict(
        sat_tl=sat_tl, orbit_sum=orbit_sum,
        stn_booked=stn_booked, sched=sched,
    )
    return entries, state
