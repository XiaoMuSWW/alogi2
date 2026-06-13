#!/usr/bin/env python3
"""
config.py - 项目配置：路径 + 全局常量
"""
import os

# === 路径配置 ===
PROJECT_PATH = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(PROJECT_PATH, 'data')
SCENARIO_PATH = os.path.join(DATA_PATH, 'scenario')
RESULT_PATH = os.path.join(PROJECT_PATH, 'result')
EVAL_EXE = os.path.join(PROJECT_PATH, 'evaluation_pub.exe')

# === 全局常量 ===
STATIONS = ['gs001', 'gs002', 'gs003', 'gs004', 'gs005']
ORBIT_PERIOD = 5400       # 轨道周期 90分钟（秒）
UL_DUR = 10               # 测控时间窗固定时长（秒），end-start+1=10
DL_DUR = 10               # 数传下行持续时间（秒，兜底值，实际由 calc_dl_dur 动态计算）
COMM_RATIO_MIN = 2.0      # 数传/成像 最小时间比
COMM_RATIO_MAX = 2.5      # 数传/成像 最大时间比
