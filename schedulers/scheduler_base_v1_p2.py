#!/usr/bin/env python3
"""
scheduler_base_v1_p2.py — 基于随机优先级多起点构造的调度求解器

与 v1/p1(贪心)、v2/v3/v4(CP-SAT) 思路根本不同：

  贪心(v1/p1)：按固定 score_ratio 优先级一次性构造，同输入→同输出
  CP-SAT(v2/v3/v4)：0-1 离散空间全局搜索，短时受限

  本求解器：随机优先级多起点构造(GRASP 风格）
  1. 每轮生成一个候选的随机排列（优先级），按此排列贪心构造
  2. 构造过程：对排列中的每个候选，若约束允许则插入
  3. 大量随机排列 → 不同的构造顺序 → 不同的可行解
  4. 取所有轮次中的最优解
  5. 最后用破坏-修复对最优解做局部改进

  本质：通过随机化构造顺序来探索解空间，避免单一排序的偏差。
  完全不依赖其他求解器，不依赖任何贪心启发式排序。
"""
import random
import time
from collections import defaultdict
from copy import deepcopy

from config import UL_DUR, ORBIT_PERIOD, STATIONS, RANDOM_SEED
from constraints import calc_dl_dur, sat_timeline_gap, check_station_conflict

# ================================================================
# 参数
# ================================================================
MAX_TIME = 20.0               # 最大 wall-clock 时间（秒）
LNS_DESTROY_MIN = 3          # 最终局部改进：破坏最少移除数
LNS_DESTROY_MAX = 12         # 最终局部改进：破坏最多移除数
LNS_STAG = 30                # 最终局部改进：停滞上限


def schedule_missions(sat_names, mission_names, sats, miss, expanded,
                      stn_acc, base_mode=True, sat_tl=None, orbit_sum=None,
                      stn_booked=None, sched=None):
    """随机优先级多起点构造 + 局部改进"""
    random.seed(RANDOM_SEED)

    all_cands = _build_candidates(sat_names, mission_names, sats, miss,
                                   expanded, base_mode)
    if not all_cands:
        if sat_tl is None: sat_tl = defaultdict(list)
        if orbit_sum is None: orbit_sum = defaultdict(lambda: defaultdict(float))
        if stn_booked is None: stn_booked = defaultdict(list)
        if sched is None: sched = defaultdict(list)
        return [], dict(sat_tl=sat_tl, orbit_sum=orbit_sum,
                        stn_booked=stn_booked, sched=sched)

    # ---- Phase 1：随机优先级多起点构造 ----
    best = None
    t0 = time.time()
    n = len(all_cands)
    runs = 0

    while time.time() - t0 < MAX_TIME * 0.85:  # 留 15% 时间给局部改进
        # 生成候选的随机排列
        order = list(range(n))
        random.shuffle(order)

        # 按此随机排列贪心构造
        sol = _random_order_construct(order, all_cands, sats, miss,
                                       stn_acc, base_mode)
        runs += 1

        if best is None or sol['score'] > best['score']:
            best = sol

    # ---- Phase 2：对最优解做破坏-修复局部改进 ----
    best = _lns_refine(best, all_cands, sats, miss, stn_acc, base_mode,
                        time_budget=MAX_TIME - (time.time() - t0))

    # ---- 输出 ----
    entries = _build_entries(best, base_mode)
    state = dict(sat_tl=best['sat_tl'], orbit_sum=best['orbit_sum'],
                 stn_booked=best['stn_booked'], sched=best['sched'])
    return entries, state


# ====================================================================
# 候选构建
# ====================================================================
def _build_candidates(sat_names, mission_names, sats, miss, expanded, base_mode):
    all_cands = []
    for sn in sat_names:
        if sn not in sats: continue
        for mn in mission_names:
            if mn not in expanded.get(sn, {}): continue
            mi = miss[mn]
            for os_, oe, roll in expanded[sn][mn]:
                dur = oe - os_ + 1
                dl_dur = calc_dl_dur(dur, mi)
                if base_mode:
                    ul_s, se = os_, os_ + dl_dur - 1
                    ss, se2 = os_, os_ + dl_dur - 1
                else:
                    ul_s, se = os_ - UL_DUR, os_ + dl_dur - 1
                    ss, se2 = ul_s, se
                all_cands.append(dict(
                    sn=sn, mn=mn, os=os_, oe=oe, roll=roll,
                    dur=dur, dl_dur=dl_dur, score=mi['score'],
                    type=mi['type'], ul_s=ul_s, sat_occ_end=se,
                    stn_start=ss, stn_end=se2))
    return all_cands


# ====================================================================
# 随机排列贪心构造
# ====================================================================
def _random_order_construct(order, ac, sats, miss, stn_acc, base_mode):
    """按给定排列顺序贪心构造解：能插就插，不回溯"""
    sol = dict(scheduled=set(), sat_tl=defaultdict(list),
               orbit_sum=defaultdict(lambda: defaultdict(float)),
               stn_booked=defaultdict(list), sched=defaultdict(list), score=0)
    sched_cnt = defaultdict(int)

    for idx in order:
        c = ac[idx]
        mn = c['mn']; mi = miss[mn]
        freq = mi['frequency'] if mi['frequency'] > 0 else 1
        if sched_cnt[mn] >= freq: continue
        if mi['type'] in ('p', 'ap') and mi.get('time_interval', 0) > 0:
            prev = sol['sched'].get(mn, [])
            if prev and c['os'] - prev[-1][3] < mi['time_interval'] * 3600:
                continue
        ok, stn = _check(c, sol, sats, miss, stn_acc)
        if ok:
            _commit(c, stn, sol, ac, base_mode)
            sched_cnt[mn] += 1

    sol['score'] = sum(ac[i]['score'] for i in sol['scheduled'])
    return sol


# ====================================================================
# LNS 局部改进
# ====================================================================
def _lns_refine(best, ac, sats, miss, stn_acc, base_mode, time_budget):
    if time_budget <= 0: return best
    stag = 0; t0 = time.time()
    while stag < LNS_STAG and time.time() - t0 < time_budget:
        work = deepcopy(best)
        removed = _destroy(work, ac)
        if len(removed) < LNS_DESTROY_MIN: stag += 1; continue
        added = _repair(work, ac, sats, miss, stn_acc, base_mode)
        delta = (sum(ac[i]['score'] for i in added) -
                 sum(ac[i]['score'] for i in removed))
        if delta > 0:
            work['score'] += delta
            if work['score'] > best['score']: best = work; stag = 0
            else: stag += 1
        else: stag += 1
    return best


# ====================================================================
# 约束检查
# ====================================================================
def _check(c, sol, sats, miss, stn_acc):
    sn, mn = c['sn'], c['mn']; sa = sats[sn]; mi = miss[mn]
    freq = mi['frequency'] if mi['frequency'] > 0 else 1
    if len(sol['sched'].get(mn, [])) >= freq: return False, None
    p = c['os'] // ORBIT_PERIOD
    if sol['orbit_sum'][sn].get(p, 0) + c['dur'] > sa.get('orbit_max', 600):
        return False, None
    if mi['type'] in ('p', 'ap') and mi.get('time_interval', 0) > 0:
        prev = sol['sched'].get(mn, [])
        if prev and c['os'] - prev[-1][3] < mi['time_interval'] * 3600:
            return False, None
    tl = sol['sat_tl'].get(sn, [])
    lo, hi = 0, len(tl)
    while lo < hi:
        mid = (lo + hi) // 2
        if tl[mid][0] < c['ul_s']: lo = mid + 1
        else: hi = mid
    if lo > 0 and tl[lo-1][1] + sat_timeline_gap(sa, tl[lo-1][2], c['roll']) >= c['ul_s']:
        return False, None
    if lo < len(tl) and c['sat_occ_end'] + sat_timeline_gap(sa, c['roll'], tl[lo][2]) >= tl[lo][0]:
        return False, None
    for stn in STATIONS:
        if not any(ws <= c['stn_start'] and we >= c['stn_end']
                   for ws, we in stn_acc.get(sn, {}).get(stn, [])): continue
        if not check_station_conflict(sol['stn_booked'].get(stn, []),
                                       c['stn_start'], c['stn_end'], sn, sats):
            return True, stn
    return False, None


# ====================================================================
# 状态操作
# ====================================================================
def _key(c): return (c['sn'], c['mn'], c['os'], c['oe'], round(c['roll'], 6))

def _idx(c, ac):
    k = _key(c)
    for i, cc in enumerate(ac):
        if _key(cc) == k: return i
    return -1

def _commit(c, stn, sol, ac, base_mode):
    sn, mn = c['sn'], c['mn']
    sol['sat_tl'][sn].append((c['ul_s'], c['sat_occ_end'], c['roll'], mn, not base_mode))
    sol['sat_tl'][sn].sort(key=lambda e: e[0])
    sol['orbit_sum'][sn][c['os'] // ORBIT_PERIOD] = \
        sol['orbit_sum'][sn].get(c['os'] // ORBIT_PERIOD, 0) + c['dur']
    sol['stn_booked'][stn].append((c['stn_start'], c['stn_end'], sn))
    sol['stn_booked'][stn].sort()
    ul_e = c['os'] if base_mode else c['os'] - UL_DUR + UL_DUR - 1
    dl_s, dl_e = c['os'], c['os'] + c['dl_dur'] - 1
    sol['sched'][mn].append((sn, stn, c['os'], c['oe'], c['roll'],
                             (c['ul_s'], ul_e), (dl_s, dl_e)))
    sol['scheduled'].add(_idx(c, ac))

def _remove(c, sol, ac):
    sn, mn = c['sn'], c['mn']
    stn = None
    for obs in sol['sched'].get(mn, []):
        if obs[0] == sn and obs[2] == c['os'] and obs[3] == c['oe']:
            stn = obs[1]; break
    tl = sol['sat_tl'].get(sn, [])
    for i, e in enumerate(tl):
        if e[0] == c['ul_s'] and e[2] == c['roll'] and e[3] == mn:
            tl.pop(i); break
    if not tl: del sol['sat_tl'][sn]
    p = c['os'] // ORBIT_PERIOD
    sol['orbit_sum'][sn][p] = max(0, sol['orbit_sum'][sn].get(p, 0) - c['dur'])
    if stn:
        bk = sol['stn_booked'].get(stn, [])
        for i, (s, e, sat) in enumerate(bk):
            if sat == sn and s == c['stn_start']: bk.pop(i); break
        if stn in sol['stn_booked'] and not sol['stn_booked'][stn]:
            del sol['stn_booked'][stn]
    sl = sol['sched'][mn]
    for i, obs in enumerate(sl):
        if obs[0] == sn and obs[2] == c['os'] and obs[3] == c['oe']:
            sl.pop(i); break
    if mn in sol['sched'] and not sol['sched'][mn]: del sol['sched'][mn]
    sol['scheduled'].discard(_idx(c, ac))


# ====================================================================
# 破坏与修复
# ====================================================================
def _destroy(sol, ac):
    if not sol['scheduled']: return []
    n = min(random.randint(LNS_DESTROY_MIN, LNS_DESTROY_MAX), len(sol['scheduled']))
    sorted_idx = sorted(sol['scheduled'], key=lambda i: ac[i]['score'])
    weights = [1.0 / (p + 1) for p in range(len(sorted_idx))]
    tw = sum(weights); probs = [w / tw for w in weights]
    removed = []
    for _ in range(n):
        if not sorted_idx: break
        r = random.random(); cum = 0; pick = 0
        for pi, p in enumerate(probs[:len(sorted_idx)]):
            cum += p
            if r <= cum: pick = pi; break
        idx = sorted_idx.pop(pick); probs.pop(pick)
        _remove(ac[idx], sol, ac); removed.append(idx)
    return removed

def _repair(sol, ac, sats, miss, stn_acc, base_mode):
    uns = [i for i in range(len(ac)) if i not in sol['scheduled']]
    random.shuffle(uns)  # 随机顺序修复，避免贪心偏差
    added = []
    for idx in uns:
        ok, stn = _check(ac[idx], sol, sats, miss, stn_acc)
        if ok:
            _commit(ac[idx], stn, sol, ac, base_mode); added.append(idx)
            if len(added) >= LNS_DESTROY_MAX * 3: break
    return added


# ====================================================================
# 输出
# ====================================================================
def _build_entries(sol, base_mode):
    entries = []
    for mn, wins in sol['sched'].items():
        for sn, stn, os_, oe, roll, ul_info, dl_info in wins:
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
