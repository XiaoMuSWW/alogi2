"""
scheduler_v2.py — CP-SAT 卫星排班 + 地面站贪心后处理

设计思路（原 v3 方案）:
  阶段 1:收集候选（仅卫星端预筛选，不含地面站兼容性检查）
  阶段 2:CP-SAT 求解卫星排班（x[i] 变量，无视地面站）
  阶段 3:对 CP-SAT 选中的候选按分数降序贪心分配地面站
          → 分不到站的候选被丢弃
  阶段 4:输出

  优势:模型小（仅 x 变量），求解快
  代价:CP-SAT 不知地面站冲突，选出的组合可能站端不可行，
        后处理丢弃率约 5-15%
"""
from collections import defaultdict
from ortools.sat.python import cp_model

from config import (UL_DUR, ORBIT_PERIOD, STATIONS,
                     CP_SAT_MAX_TIME_SECONDS, CP_SAT_NUM_WORKERS,
                     CP_SAT_SCORE_SCALE)
from constraints import (
    calc_dl_dur, sat_timeline_gap, check_station_conflict,
)


def schedule_missions(sat_names, mission_names, sats, miss, expanded,
                      stn_acc, base_mode=True, sat_tl=None, orbit_sum=None,
                      stn_booked=None, sched=None):
    """CP-SAT 卫星排班 + 地面站贪心后处理

    返回:(entries_list, state_dict)
    """
    # ============================================================
    # stg 0:状态初始化
    # ============================================================
    if sat_tl is None: sat_tl = defaultdict(list)
    if orbit_sum is None: orbit_sum = defaultdict(lambda: defaultdict(float))
    if stn_booked is None: stn_booked = defaultdict(list)
    if sched is None: sched = defaultdict(list)

    sched_count = defaultdict(int)
    for mn in mission_names:
        sched_count[mn] = len(sched.get(mn, []))

    # ============================================================
    # stg 1:候选收集（仅卫星端预筛选）
    # ============================================================
    all_cands = []
    miss_idx = defaultdict(list)

    for sn in sat_names:
        sa = sats.get(sn)
        if not sa: continue
        for mn in mission_names:
            if mn not in expanded.get(sn, {}): continue
            mi = miss[mn]
            for os_, oe, roll in expanded[sn][mn]:
                dur = oe - os_ + 1
                dl_dur = calc_dl_dur(dur, mi)
                if base_mode:
                    ul_s = os_; se = os_ + dl_dur - 1
                else:
                    ul_s = os_ - UL_DUR; se = os_ + dl_dur - 1

                # ---- 继承状态:卫星时间线 ----
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

                # ---- 继承状态:轨道累计 ----
                p = os_ // ORBIT_PERIOD
                if orbit_sum.get(sn, {}).get(p, 0) + dur > sa.get('orbit_max', 600):
                    continue

                # ---- 频次 ----
                freq = mi['frequency'] if mi['frequency'] > 0 else 1
                if sched_count[mn] >= freq: continue

                # ---- 时间间隔 ----
                if mi['type'] in ('p', 'ap') and mi.get('time_interval', 0) > 0:
                    prev = sched.get(mn, [])
                    if prev and os_ - prev[-1][3] < mi['time_interval'] * 3600:
                        continue

                idx = len(all_cands)
                all_cands.append(dict(
                    sn=sn, mn=mn, os=os_, oe=oe, roll=roll,
                    dur=dur, dl_dur=dl_dur, score=mi['score'],
                    type=mi['type'], ul_s=ul_s, sat_occ_end=se))
                miss_idx[mn].append(idx)

    if not all_cands:
        return [], dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                        stn_booked=stn_booked, sched=sched)

    # ============================================================
    # stg 2:CP-SAT 卫星排班（仅 x 变量，无地面站）
    # ============================================================
    model = cp_model.CpModel()
    n = len(all_cands)

    x = {}
    for idx in range(n):
        x[idx] = model.NewBoolVar(f'x_{idx}')

    # ---- 目标 ----
    obj = []
    for idx, c in enumerate(all_cands):
        obj.append(x[idx] * int(round(c['score'] * CP_SAT_SCORE_SCALE)))
    model.Maximize(sum(obj))

    # ---- con1:频次 ----
    for mn in mission_names:
        mi = miss[mn]
        freq = mi['frequency'] if mi['frequency'] > 0 else 1
        remaining = freq - sched_count.get(mn, 0)
        if remaining <= 0: continue
        idxs = miss_idx.get(mn, [])
        if idxs:
            model.Add(sum(x[i] for i in idxs) <= remaining)

    # ---- con2:卫星互斥 ----
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

    # ---- con3:轨道累计 ----
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

    # ---- con4:p/ap 时间间隔 ----
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

    # ============================================================
    # stg 3:求解
    # ============================================================
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = CP_SAT_MAX_TIME_SECONDS
    solver.parameters.num_workers = CP_SAT_NUM_WORKERS
    solver.parameters.log_search_progress = False

    status = solver.Solve(model)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return [], dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                        stn_booked=stn_booked, sched=sched)

    selected = [idx for idx in range(n) if solver.Value(x[idx]) == 1]

    # ============================================================
    # stg 4:地面站贪心后处理
    # ============================================================
    selected.sort(key=lambda i: -all_cands[i]['score'])

    for idx in selected:
        c = all_cands[idx]
        sn, mn = c['sn'], c['mn']
        sa = sats[sn]

        if base_mode:
            ul_s, ul_e = c['os'], c['os']
            dl_s, dl_e = c['os'], c['os'] + c['dl_dur'] - 1
            sat_occ_end = dl_e
            stn_s, stn_e = dl_s, dl_e
        else:
            ul_s = c['os'] - UL_DUR
            ul_e = ul_s + UL_DUR - 1
            dl_s, dl_e = c['os'], c['os'] + c['dl_dur'] - 1
            sat_occ_end = dl_e
            stn_s, stn_e = ul_s, dl_e

        # ---- 卫星时间线复检 ----
        tl = sat_tl.get(sn, [])
        lo, hi = 0, len(tl)
        while lo < hi:
            mid = (lo + hi) // 2
            if tl[mid][0] < ul_s: lo = mid + 1
            else: hi = mid
        conflict = False
        if lo > 0 and tl[lo-1][1] + sat_timeline_gap(sa, tl[lo-1][2], c['roll']) >= ul_s:
            conflict = True
        if not conflict and lo < len(tl) and \
           sat_occ_end + sat_timeline_gap(sa, c['roll'], tl[lo][2]) >= tl[lo][0]:
            conflict = True
        if conflict: continue

        # ---- 地面站贪心分配 ----
        best_stn = None
        for stn in STATIONS:
            covered = any(ws <= stn_s and we >= stn_e
                          for ws, we in stn_acc.get(sn, {}).get(stn, []))
            if not covered: continue
            if check_station_conflict(stn_booked.get(stn, []),
                                       stn_s, stn_e, sn, sats):
                continue
            best_stn = stn; break

        if best_stn is None: continue

        # ---- 提交 ----
        sat_tl[sn].append((ul_s, sat_occ_end, c['roll'], mn))
        sat_tl[sn].sort(key=lambda e: e[0])
        orbit_sum[sn][c['os'] // ORBIT_PERIOD] = \
            orbit_sum[sn].get(c['os'] // ORBIT_PERIOD, 0) + c['dur']
        stn_booked[best_stn].append((stn_s, stn_e, sn))
        stn_booked[best_stn].sort()
        sched[mn].append((sn, best_stn, c['os'], c['oe'], c['roll'],
                          (ul_s, ul_e), (dl_s, dl_e)))

    # ============================================================
    # stg 5:输出
    # ============================================================
    entries = _build_entries(sched, mission_names, base_mode)
    return entries, dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                         stn_booked=stn_booked, sched=sched)


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
