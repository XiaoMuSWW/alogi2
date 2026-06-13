#!/usr/bin/env python3
"""
main.py - 卫星调度算法执行入口
用法:
  python main.py                    # 处理场景 001
  python main.py --scenario 042     # 处理场景 042
  python main.py --all              # 处理全部 300 个场景
  python main.py --evaluate         # 处理 + 评测
"""
import os, json, time, argparse, subprocess
from config import DATA_PATH, SCENARIO_PATH, RESULT_PATH, EVAL_EXE
from data_loader import load_all
from scheduler import expand_windows, schedule_missions
from output_writer import write_csv


def process(scenario_id: str, sats: dict, miss: dict, obs_acc: dict, stn_acc: dict) -> None:
    """处理单个场景：基础 + 子场景波次"""
    sc_dir = os.path.join(SCENARIO_PATH, scenario_id)
    out_dir = os.path.join(RESULT_PATH, scenario_id)

    with open(os.path.join(sc_dir, f'{scenario_id}.json')) as f:
        base = json.load(f)
    base_sats = base['satellite_names']
    base_miss = set(base.get('mission_names', []))

    expanded_full = expand_windows(obs_acc, sats, miss, offset=0)

    # ---------- 基础场景 (OBS → DL) ----------
    entries_base, _ = schedule_missions(
        base_sats, base_miss, sats, miss, expanded_full, stn_acc, base_mode=True
    )
    write_csv(entries_base, os.path.join(out_dir, f'{scenario_id}.csv'))
    n_base = sum(1 for e in entries_base if not e.get('station_name'))

    print(f"\n{'='*60}")
    print(f"  {scenario_id}: {len(base_sats)} sats, {len(base_miss)} missions")
    print(f"  Base: {n_base} obs ({len(entries_base)} rows) | ")
    

    # ---------- 波次场景 (UL → OBS → DL) ----------
    _process_waves(scenario_id, sc_dir, out_dir, 1, 2, 28800,
                   base_sats, sats, miss, obs_acc, stn_acc)
    _process_waves(scenario_id, sc_dir, out_dir, 2, 11, 7200,
                   base_sats, sats, miss, obs_acc, stn_acc)


def _process_waves(scenario_id: str, sc_dir: str, out_dir: str,
                   wave_type: int, wave_count: int, wave_interval: int,
                   base_sats: list, sats: dict, miss: dict,
                   obs_acc: dict, stn_acc: dict) -> None:
    """处理一种波次类型的所有波次（独立调度，无状态继承）"""

    for wi in range(1, wave_count + 1):
        sf = f'-{wave_type}-{wi}'
        wo = wi * wave_interval
        wf = os.path.join(sc_dir, f'{scenario_id}{sf}.json')
        if not os.path.exists(wf):
            continue

        with open(wf) as f:
            wsc = json.load(f)
        wms = set(wsc.get('mission_names', []))

        if not wms:
            write_csv([], os.path.join(out_dir, f'{scenario_id}{sf}.csv'))
            continue

        # 为该波次扩展窗口（仅 t >= wo 的窗口，独立调度）
        wave_expanded = expand_windows(obs_acc, sats, miss, offset=wo)
        new_entries, _ = schedule_missions(
            base_sats, wms, sats, miss, wave_expanded, stn_acc,
            base_mode=False,
        )
        write_csv(new_entries, os.path.join(out_dir, f'{scenario_id}{sf}.csv'))
        n_wo = sum(1 for e in new_entries if not e.get('station_name'))
        print(f"  {sf}: {n_wo} obs | ")


def evaluate() -> None:
    """调用评测工具"""
    cmd = [EVAL_EXE, '-d', DATA_PATH, '-i', SCENARIO_PATH, '-o', RESULT_PATH]
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(EVAL_EXE))
    print(r.stdout[:3000])
    if r.stderr:
        print("ERR:", r.stderr[-500:])


def main():
    p = argparse.ArgumentParser(description='卫星调度算法')
    p.add_argument('--scenario', type=str, default='001', help='场景编号')
    p.add_argument('--all', action='store_true', help='处理全部 300 个场景')
    p.add_argument('--evaluate', action='store_true', help='处理完成后评测')
    args = p.parse_args()

    t0 = time.time()
    sats, miss, obs_acc, stn_acc = load_all()

    if args.all:
        scenario_dirs = sorted(
            d for d in os.listdir(SCENARIO_PATH)
            if os.path.isdir(os.path.join(SCENARIO_PATH, d))
        )
        for sid in scenario_dirs:
            process(sid, sats, miss, obs_acc, stn_acc)
    else:
        process(args.scenario, sats, miss, obs_acc, stn_acc)

    print(f"\nTime: {time.time() - t0:.2f}s")

    if args.evaluate:
        evaluate()


if __name__ == '__main__':
    main()
