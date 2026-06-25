"""
scheduler_v3.py — CP-SAT 调度器（地面站约束内化 + 热启动）

与旧 v3 的关键区别：
  旧 v3：地面站约束从 CP-SAT 中剥离，用贪心后处理分配
         → 卫星 MIP 不知地面站冲突，选出的候选可能在站端不可行

  新 v3：地面站约束直接纳入 CP-SAT 模型（y[i][g] 变量 + 互斥约束）
         → 求解器同时决定卫星排班和地面站分配
         → 不会选出站端不可行的组合
         → 配合 v1 贪心热启动保证解 >= v1
"""
from collections import defaultdict
from copy import deepcopy

from ortools.sat.python import cp_model

from config import (UL_DUR, ORBIT_PERIOD, STATIONS,
                     CP_SAT_MAX_TIME_SECONDS, CP_SAT_NUM_WORKERS,
                     CP_SAT_SCORE_SCALE)
from constraints import calc_dl_dur, sat_timeline_gap


def schedule_missions(sat_names, mission_names, sats, miss, expanded,
                      stn_acc, base_mode=True, sat_tl=None, orbit_sum=None,
                      stn_booked=None, sched=None):
    """CP-SAT 调度（地面站约束在模型内 + v1 热启动）

    返回：(entries_list, state_dict)
    """
    # ============================================================
    # stg 0：状态初始化
    # ============================================================
    if sat_tl is None: sat_tl = defaultdict(list)
    if orbit_sum is None: orbit_sum = defaultdict(lambda: defaultdict(float))
    if stn_booked is None: stn_booked = defaultdict(list)
    if sched is None: sched = defaultdict(list)

    sched_count = defaultdict(int)
    for mn in mission_names:
        sched_count[mn] = len(sched.get(mn, []))

    # ============================================================
    # stg 1：候选收集（含地面站兼容性预计算）
    # ============================================================
    all_cands, miss_idx = _collect_candidates(
        sat_names, mission_names, sats, miss, expanded, stn_acc,
        base_mode, sat_tl, orbit_sum, stn_booked, sched, sched_count)

    if not all_cands:
        return [], dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                        stn_booked=stn_booked, sched=sched)

    # ============================================================
    # stg 1.5：v1 贪心热启动
    # ============================================================
    greedy_hints = _run_greedy_hints(
        sat_names, mission_names, sats, miss, expanded, stn_acc, base_mode,
        deepcopy(sat_tl), deepcopy(orbit_sum),
        deepcopy(stn_booked), deepcopy(sched), all_cands)

    # ============================================================
    # stg 2：构建 CP-SAT 模型（含地面站约束）
    # ============================================================
    model = cp_model.CpModel()
    n = len(all_cands)

    # ---- 变量 ----
    x = {}       # x[i] = 1 选候选 i
    y = {}       # y[i][g] = 1 候选 i 用地面站 g
    for idx, c in enumerate(all_cands):
        x[idx] = model.NewBoolVar(f'x_{idx}')
        if idx in greedy_hints:
            model.AddHint(x[idx], 1)
        for g in c['stn_opts']:
            y[(idx, g)] = model.NewBoolVar(f'y_{idx}_{g}')

    # ---- 目标 ----
    obj = []
    for idx, c in enumerate(all_cands):
        obj.append(x[idx] * int(round(c['score'] * CP_SAT_SCORE_SCALE)))
    model.Maximize(sum(obj))

    # ---- con1：频次 ----
    for mn in mission_names:
        mi = miss[mn]
        freq = mi['frequency'] if mi['frequency'] > 0 else 1
        remaining = freq - sched_count.get(mn, 0)
        if remaining <= 0: continue
        idxs = miss_idx.get(mn, [])
        if idxs:
            model.Add(sum(x[i] for i in idxs) <= remaining)

    # ---- con2：卫星互斥 ----
    sat_cand_idxs = defaultdict(list)
    for idx, c in enumerate(all_cands):
        sat_cand_idxs[c['sn']].append(idx)

    for sn, idxs in sat_cand_idxs.items():
        idxs_sorted = sorted(idxs, key=lambda i: all_cands[i]['ul_s'])
        sa = sats[sn]
        max_gap = max(
            int(sa.get('trantime_sar', 20)) + int(sa.get('reversal_time', 0)) +
            int(sa.get('trantime_cc', 20)),
            int(sa.get('trantime_20', 60)) + int(sa.get('trantime_cc', 20)))

        for i_pos in range(len(idxs_sorted)):
            idx_i = idxs_sorted[i_pos]; ci = all_cands[idx_i]
            for j_pos in range(i_pos + 1, len(idxs_sorted)):
                idx_j = idxs_sorted[j_pos]; cj = all_cands[idx_j]
                if ci['sat_occ_end'] + max_gap < cj['ul_s']: break
                gap_ij = sat_timeline_gap(sa, ci['roll'], cj['roll'])
                gap_ji = sat_timeline_gap(sa, cj['roll'], ci['roll'])
                if ci['sat_occ_end'] + gap_ij >= cj['ul_s'] and \
                   cj['sat_occ_end'] + gap_ji >= ci['ul_s']:
                    model.Add(x[idx_i] + x[idx_j] <= 1)

    # ---- con3：轨道累计 ----
    orbit_bins = defaultdict(lambda: defaultdict(list))
    for idx, c in enumerate(all_cands):
        orbit_bins[c['sn']][c['os'] // ORBIT_PERIOD].append(idx)
    for sn, periods in orbit_bins.items():
        orbit_max = sats[sn].get('orbit_max', 600)
        inherited = orbit_sum.get(sn, {})
        for p_idx, idxs in periods.items():
            eff = orbit_max - inherited.get(p_idx, 0)
            if eff <= 0:
                for i in idxs: model.Add(x[i] == 0)
            else:
                model.Add(sum(all_cands[i]['dur'] * x[i] for i in idxs) <= int(eff))

    # ---- con4：p/ap 时间间隔 ----
    for mn in mission_names:
        mi = miss[mn]
        if mi['type'] not in ('p', 'ap'): continue
        ti = mi.get('time_interval', 0)
        if ti <= 0: continue
        tis = int(ti * 3600)
        idxs = sorted(miss_idx.get(mn, []), key=lambda i: all_cands[i]['os'])
        for i_pos in range(len(idxs)):
            idx_i = idxs[i_pos]
            for j_pos in range(i_pos + 1, len(idxs)):
                idx_j = idxs[j_pos]
                if all_cands[idx_j]['os'] - all_cands[idx_i]['os'] >= tis: break
                model.Add(x[idx_i] + x[idx_j] <= 1)

    # ---- con5：地面站通道 ----
    # 5a. 选中必须分配恰好一个站
    for idx, c in enumerate(all_cands):
        if c['stn_opts']:
            model.Add(sum(y[(idx, g)] for g in c['stn_opts']) == x[idx])

    # 5b. 站互斥
    for g_idx in range(len(STATIONS)):
        stn_cands = [idx for idx, c in enumerate(all_cands)
                     if g_idx in c['stn_opts']]
        stn_cands.sort(key=lambda i: all_cands[i]['stn_start'])
        for i_pos in range(len(stn_cands)):
            idx_i = stn_cands[i_pos]; ci = all_cands[idx_i]
            for j_pos in range(i_pos + 1, len(stn_cands)):
                idx_j = stn_cands[j_pos]; cj = all_cands[idx_j]
                if ci['stn_end'] < cj['stn_start']: break
                cc_i = 0 if ci['sn'] == cj['sn'] else int(sats[ci['sn']].get('trantime_cc', 20))
                cc_j = 0 if ci['sn'] == cj['sn'] else int(sats[cj['sn']].get('trantime_cc', 20))
                if ci['stn_end'] + cc_i >= cj['stn_start'] and \
                   cj['stn_end'] + cc_j >= ci['stn_start']:
                    model.Add(y[(idx_i, g_idx)] + y[(idx_j, g_idx)] <= 1)

    # ============================================================
    # stg 3：求解
    # ============================================================
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = CP_SAT_MAX_TIME_SECONDS
    solver.parameters.num_workers = CP_SAT_NUM_WORKERS
    solver.parameters.log_search_progress = False

    status = solver.Solve(model)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        if greedy_hints:
            return _fallback_greedy(greedy_hints, all_cands, sats, miss,
                                     stn_acc, base_mode, sat_tl, orbit_sum,
                                     stn_booked, sched, mission_names)
        return [], dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                        stn_booked=stn_booked, sched=sched)

    # ============================================================
    # stg 4：从求解器直接提取结果（无后处理）
    # ============================================================
    selected = [idx for idx in range(n) if solver.Value(x[idx]) == 1]

    for idx in selected:
        c = all_cands[idx]
        sn, mn = c['sn'], c['mn']

        # 直接从 y 变量读取地面站
        assigned_stn = None
        for g in c['stn_opts']:
            if solver.Value(y[(idx, g)]) == 1:
                assigned_stn = STATIONS[g]; break
        if assigned_stn is None:
            continue  # 模型正确时不应发生

        if base_mode:
            ul_s, ul_e = c['os'], c['os']
            dl_s, dl_e = c['os'], c['os'] + c['dl_dur'] - 1
            sat_occ_end = dl_e
        else:
            ul_s = c['os'] - UL_DUR
            ul_e = ul_s + UL_DUR - 1
            dl_s, dl_e = c['os'], c['os'] + c['dl_dur'] - 1
            sat_occ_end = dl_e

        sat_tl[sn].append((ul_s, sat_occ_end, c['roll'], mn))
        sat_tl[sn].sort(key=lambda e: e[0])
        orbit_sum[sn][c['os'] // ORBIT_PERIOD] = \
            orbit_sum[sn].get(c['os'] // ORBIT_PERIOD, 0) + c['dur']
        stn_booked[assigned_stn].append((c['stn_start'], c['stn_end'], sn))
        stn_booked[assigned_stn].sort()
        sched[mn].append((sn, assigned_stn, c['os'], c['oe'], c['roll'],
                          (ul_s, ul_e), (dl_s, dl_e)))

    # ============================================================
    # stg 5：输出
    # ============================================================
    entries = _build_entries(sched, mission_names, base_mode)
    return entries, dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                         stn_booked=stn_booked, sched=sched)


# ====================================================================
# 候选收集
# ====================================================================
def _collect_candidates(sat_names, mission_names, sats, miss, expanded,
                         stn_acc, base_mode, sat_tl, orbit_sum, stn_booked,
                         sched, sched_count):
    all_cands = []; miss_idx = defaultdict(list)
    for sn in sat_names:
        sa = sats.get(sn)
        if not sa: continue
        for mn in mission_names:
            if mn not in expanded.get(sn, {}): continue
            mi = miss[mn]
            for os_, oe, roll in expanded[sn][mn]:
                dur = oe - os_ + 1; dl_dur = calc_dl_dur(dur, mi)
                if base_mode:
                    ul_s = os_; se = os_ + dl_dur - 1
                    ss, se2 = os_, os_ + dl_dur - 1
                else:
                    ul_s = os_ - UL_DUR; se = os_ + dl_dur - 1
                    ss, se2 = ul_s, se
                # 继承状态冲突
                tl = sat_tl.get(sn, [])
                lo, hi = 0, len(tl)
                while lo < hi:
                    mid = (lo + hi) // 2
                    if tl[mid][0] < ul_s: lo = mid + 1
                    else: hi = mid
                conflict = False
                if lo > 0 and tl[lo-1][1] + sat_timeline_gap(sa, tl[lo-1][2], roll) >= ul_s:
                    conflict = True
                if not conflict and lo < len(tl) and \
                   se + sat_timeline_gap(sa, roll, tl[lo][2]) >= tl[lo][0]:
                    conflict = True
                if conflict: continue
                # 轨道
                p = os_ // ORBIT_PERIOD
                if orbit_sum.get(sn, {}).get(p, 0) + dur > sa.get('orbit_max', 600):
                    continue
                # 频次
                freq = mi['frequency'] if mi['frequency'] > 0 else 1
                if sched_count[mn] >= freq: continue
                # 时间间隔
                if mi['type'] in ('p', 'ap') and mi.get('time_interval', 0) > 0:
                    prev = sched.get(mn, [])
                    if prev and os_ - prev[-1][3] < mi['time_interval'] * 3600:
                        continue
                # 地面站兼容性
                stn_opts = []
                for g_idx, stn in enumerate(STATIONS):
                    if not any(ws <= ss and we >= se2
                               for ws, we in stn_acc.get(sn, {}).get(stn, [])):
                        continue
                    conflict2 = False
                    for bs, be, bsat in stn_booked.get(stn, []):
                        if bsat != sn:
                            cc = int(sats.get(bsat, {}).get('trantime_cc', 0) or 0)
                            if ss <= be + cc and bs <= se2 + cc:
                                conflict2 = True; break
                        elif ss <= be and bs <= se2:
                            conflict2 = True; break
                    if not conflict2: stn_opts.append(g_idx)
                if not stn_opts: continue

                idx = len(all_cands)
                all_cands.append(dict(sn=sn, mn=mn, os=os_, oe=oe, roll=roll,
                                      dur=dur, dl_dur=dl_dur, score=mi['score'],
                                      type=mi['type'], ul_s=ul_s, sat_occ_end=se,
                                      stn_start=ss, stn_end=se2, stn_opts=stn_opts))
                miss_idx[mn].append(idx)
    return all_cands, miss_idx


# ====================================================================
# 热启动
# ====================================================================
def _run_greedy_hints(sat_names, mission_names, sats, miss, expanded,
                       stn_acc, base_mode, sat_tl, orbit_sum, stn_booked,
                       sched, all_cands):
    try:
        from schedulers.scheduler_v1 import schedule_missions as v1
    except ImportError:
        return set()
    try:
        _, state = v1(sat_names, mission_names, sats, miss, expanded,
                      stn_acc, base_mode, sat_tl=sat_tl, orbit_sum=orbit_sum,
                      stn_booked=stn_booked, sched=sched)
    except Exception:
        return set()

    lookup = {}
    for idx, c in enumerate(all_cands):
        key = (c['sn'], c['mn'], c['os'], c['oe'], round(c['roll'], 6))
        lookup.setdefault(key, []).append(idx)
    hints = set()
    for mn in state.get('sched', {}):
        for obs in state['sched'][mn]:
            sn, _, os_, oe, roll, _, _ = obs
            key = (sn, mn, os_, oe, round(roll, 6))
            for idx in lookup.get(key, []):
                hints.add(idx); break
    return hints


def _fallback_greedy(hints, all_cands, sats, miss, stn_acc, base_mode,
                      sat_tl, orbit_sum, stn_booked, sched, mission_names):
    """CP-SAT 无解时回退贪心"""
    from schedulers.scheduler_v1 import schedule_missions as v1
    # 重建 expanded
    expanded2 = defaultdict(lambda: defaultdict(list))
    for idx in hints:
        c = all_cands[idx]
        expanded2[c['sn']][c['mn']].append((c['os'], c['oe'], c['roll']))
    entries, state = v1(set(c['sn'] for c in all_cands), set(c['mn'] for c in all_cands),
                         sats, miss, expanded2, stn_acc, base_mode,
                         sat_tl=sat_tl, orbit_sum=orbit_sum,
                         stn_booked=stn_booked, sched=sched)
    return entries, state


# ====================================================================
# 输出
# ====================================================================
def _build_entries(sched, mission_names, base_mode):
    entries = []
    for mn in mission_names:
        for sn, stn, os_, oe, roll, ul_info, dl_info in sched.get(mn, []):
            ul_s, ul_e = ul_info; dl_s, dl_e = dl_info
            if not base_mode:
                entries.append(dict(satellite_name=sn, mission_name=mn,
                                    station_name=stn, start_time=ul_s,
                                    end_time=ul_e, roll_angle=None))
            entries.append(dict(satellite_name=sn, mission_name=mn,
                                station_name="", start_time=os_,
                                end_time=oe, roll_angle=roll))
            entries.append(dict(satellite_name=sn, mission_name=mn,
                                station_name=stn, start_time=dl_s,
                                end_time=dl_e, roll_angle=None))
    entries.sort(key=lambda e: (e["satellite_name"], e["start_time"]))
    return entries
