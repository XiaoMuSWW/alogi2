#!/usr/bin/env python3
"""
config.py - 项目配置：路径 + 全局常量
"""
import os

# === 路径配置 ===
PROJECT_PATH = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(PROJECT_PATH, 'data')
RESULT_PATH = os.path.join(PROJECT_PATH, 'result')


SCENARIO_PATH = os.path.join(DATA_PATH, 'scenario')


EVAL_EXE = os.path.join(PROJECT_PATH, 'evaluation_pub.exe')

# === 全局常量 ===

# 所有地面站的量是相同的

STATIONS = ['gs001', 'gs002', 'gs003', 'gs004', 'gs005']

COMM_RATIO_MIN = 2.0      # 数传/成像 最小时间比
COMM_RATIO_MAX = 2.5      # 数传/成像 最大时间比


ORBIT_PERIOD = 5400       # 轨道周期 90分钟（秒）
UL_DUR = 10               # 测控时间窗固定时长（秒），end-start+1=10
DL_DUR = 10               # 数传下行持续时间（未定义状态）

CLAMP_TOLERANCE_P = 0.1
CLAMP_TOLERANCE_A = 1     # 区域目标侧摆角夹持最大容许偏移（度）

# === CP-SAT 求解器参数 ===
CP_SAT_MAX_TIME_SECONDS = 30    # 单次求解最大时间（秒）
CP_SAT_NUM_WORKERS = 8          # 并行搜索线程数（4→8，更多策略覆盖）
CP_SAT_SCORE_SCALE = 100        # 浮点分数→整数放大倍数
CP_SAT_STATION_MIP_TIMEOUT = 2  # 地面站分配 MIP 超时（秒，5→2，<0.5s 即可解完）
CP_SAT_MAX_FEEDBACK_ITERATIONS = 3  # 反馈回路最大轮次（2→3，更多机会消解站冲突）
CP_SAT_GAP_FILL_LIMIT = 300     # 间隙填补最多尝试候选数（200→300）

# === 通用随机种子 ===
RANDOM_SEED = 42             # 全局随机种子，保证调度可复现
