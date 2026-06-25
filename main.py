#!/usr/bin/env python3
"""
main.py - 卫星调度算法执行入口
用法:
  python main.py                    # 处理场景 001
  python main.py --scenario 042     # 处理场景 042
  python main.py --all              # 处理全部 300 个场景
  python main.py --evaluate         # 处理 + 评测
"""
import os, json, time, argparse, subprocess, csv
from typing import List, Dict
from config import DATA_PATH, SCENARIO_PATH, RESULT_PATH, EVAL_EXE
from data_loader import load_all
from schedulers.scheduler_v2 import (schedule_missions)
from output_writer import write_csv

from pre_processer import (
    expand_windows,
    filter_state_before
)

# 存储各场景调度耗时: key = "{scenario_id}_{variant}", value = seconds
# variant: "base" | "1" | "2"
_timing: Dict[str, float] = {}


def process(scenario_id: str, sats: dict, miss: dict, obs_acc: dict, stn_acc: dict) -> None:
    """处理单个场景：基础方案 + 子场景波次

    Args:
        scenario_id: 场景编号（如 "001"）
        sats: 卫星参数字典
        miss: 任务参数字典
        obs_acc: 观测 access 数据
        stn_acc: 地面站 access 数据
    """
    sc_dir = os.path.join(SCENARIO_PATH, scenario_id)
    out_dir = os.path.join(RESULT_PATH, scenario_id)

    with open(os.path.join(sc_dir, f'{scenario_id}.json')) as f:
        base = json.load(f)
    base_sats = base['satellite_names']
    base_miss = set(base.get('mission_names', []))

    expanded_full = expand_windows(obs_acc, sats, miss, offset=0)

    # ---------- 基础场景 (OBS → DL) ----------
    t_base_start = time.time()
    entries_base, base_state = schedule_missions(
        base_sats, base_miss, sats, miss, expanded_full, stn_acc, base_mode=True
    )
    t_base = time.time() - t_base_start
    _timing[f"{scenario_id}_base"] = t_base

    write_csv(entries_base, os.path.join(out_dir, f'{scenario_id}.csv'))
    n_base = sum(1 for e in entries_base if not e.get('station_name'))

    print(f"\n{'='*60}")
    print(f"  {scenario_id}: {len(base_sats)} sats, {len(base_miss)} missions")
    print(f"  Base: {n_base} obs ({len(entries_base)} rows) | {t_base:.2f}s")

    # ---------- 波次场景 (UL → OBS → DL) ----------
    _process_waves(scenario_id, sc_dir, out_dir, 1, 2, 28800,
                   base_sats, sats, miss, obs_acc, stn_acc, base_state)
    _process_waves(scenario_id, sc_dir, out_dir, 2, 11, 7200,
                   base_sats, sats, miss, obs_acc, stn_acc, base_state)


def _process_waves(scenario_id: str, sc_dir: str, out_dir: str,
                   wave_type: int, wave_count: int, wave_interval: int,
                   base_sats: list, sats: dict, miss: dict,
                   obs_acc: dict, stn_acc: dict, base_state: dict) -> None:
    """处理一种波次类型的所有波次

    继承规则: 锁定基础方案在 [0, wave_offset) 内已完成的占用，
    波次新任务在 [wave_offset, 86400) 内自由调度。

    Args:
        scenario_id: 场景编号
        sc_dir: 场景 JSON 目录
        out_dir: 输出目录
        wave_type: 波次类型（1 或 2）
        wave_count: 波次数量
        wave_interval: 波次间隔（秒）
        base_sats: 基础场景卫星列表
        sats: 卫星参数字典
        miss: 任务参数字典
        obs_acc: 观测 access 数据
        stn_acc: 地面站 access 数据
        base_state: 基础方案调度状态
    """
    t_wave_start = time.time()
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

        # 继承基础方案状态：取消 wave_offset 之后的原占用，
        # 保留之前的锁定资源，波次任务在 [wave_offset, 86400) 内自由追加
        inherited = filter_state_before(base_state, wo)

        wave_expanded = expand_windows(obs_acc, sats, miss, offset=wo)

        new_entries, _ = schedule_missions(
            base_sats, wms, sats, miss, wave_expanded, stn_acc,
            base_mode=False,
            sat_tl=inherited['sat_tl'],
            orbit_sum=inherited['orbit_sum'],
            stn_booked=inherited['stn_booked'],
            sched=inherited['sched'],
        )
        write_csv(new_entries, os.path.join(out_dir, f'{scenario_id}{sf}.csv'))
        n_wo = sum(1 for e in new_entries if not e.get('station_name'))
        print(f"  {sf}: {n_wo} obs ")

    t_wave = time.time() - t_wave_start
    _timing[f"{scenario_id}_{wave_type}"] = t_wave
    print(f"  Wave {wave_type}: {t_wave:.2f}s")


def parse_results(stdout: str) -> List[Dict[str, float]]:
    """解析评测工具的标准输出，提取结果行

    Args:
        stdout: 评测工具的标准输出字符串

    Returns:
        结果列表 [{scene, point_score, region_score, sat_constraint_penalty, total_score}, ...]
    """
    results = []
    lines = stdout.strip().splitlines()

    total = 0        # 总行数（排除空行）
    skipped_header = 0  # 跳过：表头/中文字段
    skipped_parts = 0   # 跳过：字段数 < 5
    skipped_parse = 0   # 跳过：float 解析失败

    for line in lines:
        if not line.strip():
            continue

        total += 1

        # 跳过表头或非数据行
        if any(kw in line for kw in ("场景", "点得分", "约束分", "总分", "==========")):
            skipped_header += 1
            continue

        parts = line.split()
        if len(parts) < 5:
            skipped_parts += 1
            # 打印被跳过的短行便于排查
            if total <= 20 or skipped_parts <= 5:
                print(f"  [DEBUG] 跳过短行 ({len(parts)}字段): {line[:100]}")
            continue

        try:
            results.append({
                "scene": parts[0],
                "point_score": float(parts[1]),
                "region_score": float(parts[2]),
                "sat_constraint_penalty": float(parts[3]),
                "total_score": float(parts[4]),
            })
        except ValueError:
            skipped_parse += 1
            if skipped_parse <= 5:
                print(f"  [DEBUG] 跳过解析失败行: {line[:100]}")
            continue

    print(f"  parse_results: 总行={total} 表头跳过={skipped_header} "
          f"短行跳过={skipped_parts} 解析失败={skipped_parse} "
          f"有效结果={len(results)}")
    return results


def write_eval_csv(results: List[Dict[str, float]], output_path: str) -> None:
    """将评测结果写入 CSV（含调度耗时）

    Args:
        results: 评测结果列表
        output_path: 输出 CSV 文件路径
    """
    if not results:
        print("未解析到任何结果，不写入 CSV")
        return

    fieldnames = [
        "scene",
        "point_score",
        "region_score",
        "sat_constraint_penalty",
        "total_score",
        "time_s",
    ]

    with open(output_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)

    print(f"已写入 CSV:{output_path}(共 {len(results)} 条)")


def evaluate() -> None:
    """调用评测工具并保存结果为 CSV（含各场景调度耗时）"""
    cmd = [EVAL_EXE, '-d', DATA_PATH, '-i', SCENARIO_PATH, '-o', RESULT_PATH]

    r = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        cwd=os.path.dirname(EVAL_EXE)
    )

    # 保存原始 stdout 以便排查
    raw_log = os.path.join(RESULT_PATH, "evaluation_raw_output.txt")
    with open(raw_log, "w", encoding="utf-8") as f:
        f.write(r.stdout)
        if r.stderr:
            f.write("\n\n=== STDERR ===\n")
            f.write(r.stderr)
    print(f"原始输出已保存: {raw_log}")

    print(r.stdout[:2000])
    if len(r.stdout) > 2000:
        print(f"... (共 {len(r.stdout)} 字符，截断显示)")

    results = parse_results(r.stdout)

    # 统计场景覆盖率
    scene_ids = set()
    for row in results:
        sid = row["scene"].rsplit("-", 1)[0]
        scene_ids.add(sid)
    print(f"  evaluate: 覆盖 {len(scene_ids)} 个场景，共 {len(results)} 条评测结果")

    # 拼接调度耗时: scene "001-0"→_timing["001_base"], "001-1"→_timing["001_1"], ...
    _VARIANT_MAP = {"0": "base", "1": "1", "2": "2"}
    for row in results:
        scene = row["scene"]
        parts = scene.rsplit("-", 1)
        sid = parts[0]                             # "001"
        variant = parts[1] if len(parts) > 1 else "0"  # "0"/"1"/"2"
        key = f"{sid}_{_VARIANT_MAP.get(variant, variant)}"
        row["time_s"] = round(_timing.get(key, 0.0), 2)

    write_eval_csv(results, os.path.join(RESULT_PATH, "evaluation_summary.csv"))



def main():
    """主入口：解析参数并执行调度"""
    p = argparse.ArgumentParser(description='卫星调度算法')
    p.add_argument('--scenario', type=str, default='001', help='场景编号')
    p.add_argument('--all', action='store_true', help='处理全部场景')
    p.add_argument('--evaluate', action='store_true', help='处理完成后评测')
    args = p.parse_args()

    t0 = time.time()
    sats, miss, obs_acc, stn_acc = load_all()

    if args.all:
        scenario_dirs = sorted(
            d for d in os.listdir(SCENARIO_PATH)
            if os.path.isdir(os.path.join(SCENARIO_PATH, d))
        )
        print(f"共 {len(scenario_dirs)} 个场景待处理")
        ok_count = 0
        for i, sid in enumerate(scenario_dirs, 1):
            try:
                process(sid, sats, miss, obs_acc, stn_acc)
                ok_count += 1
            except Exception as e:
                print(f"  [ERROR] 场景 {sid} 处理失败: {e}")
            if i % 50 == 0:
                print(f"\n--- 进度: {i}/{len(scenario_dirs)} (成功 {ok_count}) ---\n")
        print(f"\n处理完成: {ok_count}/{len(scenario_dirs)} 个场景成功")
    else:
        process(args.scenario, sats, miss, obs_acc, stn_acc)

    print(f"\nTime: {time.time() - t0:.2f}s")

    if args.evaluate:
        evaluate()


if __name__ == '__main__':
    main()
