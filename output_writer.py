#!/usr/bin/env python3
"""
output_writer.py - CSV 输出模块
"""
import os, csv


def write_csv(entries: list, path: str) -> None:
    """将调度结果写入 CSV 文件"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=[
            'satellite_name', 'mission_name', 'station_name',
            'start_time', 'end_time', 'roll_angle',
        ])
        w.writeheader()
        for e in entries:
            ra = e['roll_angle']
            w.writerow({
                'satellite_name': e['satellite_name'],
                'mission_name': e['mission_name'],
                'station_name': e.get('station_name', ''),
                'start_time': e['start_time'],
                'end_time': e['end_time'],
                'roll_angle': '{:.6f}'.format(ra) if ra is not None else '',
            })
