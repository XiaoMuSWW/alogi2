# 多星多任务调度优化系统

基于动态时间窗口的对地观测卫星调度算法，实现 120 颗卫星（60 光学 + 60 SAR）对 805 个任务的协同观测规划。

## 环境配置

```bash
# 从 environment.yml 创建 conda 环境
conda env create -f environment.yml

# 激活环境
conda activate alogi2
```

## 前置步骤（必须先运行）

在运行调度之前，必须先生成轨道数据和时间窗口：

```bash
# 1. 轨道外推 — 从 TLE 生成 120 颗卫星的轨道数据 (.ipc)
./orbit_propagator.exe -d data/

# 2. 时间窗口计算 — 基于轨道数据计算卫星-任务-地面站 access 窗口 (access.csv)
./time_window.exe -d data/
```

> **注意**: `data/access.csv` 和 `data/orbit/` 目录由上述工具生成，不包含在 git 仓库中。

## 快速开始

```bash
# 处理场景 001
python main.py --scenario 001

# 处理 + 外部评测
python main.py --scenario 001 --evaluate

# 批量处理全部 300 个场景
python main.py --all
```

## 项目结构

```
alogi2/
├── config.py            # 路径 + 全局常量 (UL_DUR, ORBIT_PERIOD, COMM_RATIO 等)
├── data_loader.py       # 数据加载 (卫星参数 / 任务参数 / access 窗口)
├── constraints.py       # 约束检查 (Status 2-9，对应 PDF Section 2.14)
├── scheduler.py         # 调度核心 (窗口扩展 → 贪心调度 → 得分率统计)
├── output_writer.py     # CSV 输出
├── main.py              # CLI 入口 + 场景处理流水线
├── environment.yml      # Conda 环境配置
├── data/                # 输入数据
│   ├── satellite.csv    # 120 颗卫星参数
│   ├── mission.csv      # 805 个任务参数
│   ├── tle.txt          # TLE 轨道根数
│   ├── access.csv       # 预计算访问窗口 (由 time_window.exe 生成)
│   ├── orbit/           # 120 个 .ipc 轨道数据文件 (由 orbit_propagator.exe 生成)
│   └── scenario/        # 300 个测试场景 (001-300)
├── result/              # 调度结果输出 (由 main.py 生成)
├── orbit_propagator.exe # 轨道外推工具
├── time_window.exe      # 时间窗口计算工具
└── evaluation_pub.exe   # 评测工具
```

## 模块依赖

```
config.py
 ├── data_loader.py → config
 ├── constraints.py → config
 ├── output_writer.py
 └── scheduler.py → config + constraints
      └── main.py → config + data_loader + scheduler + output_writer
```

## 算法概要

### 两种调度模式

| 模式 | 阶段 | CSV 行/观测 | 适用场景 |
|------|------|------------|---------|
| **base_mode** | OBS → DL（无测控） | 2 | 初始方案 (001.csv) |
| **wave_mode** | UL → OBS → DL 串行 | 3 | 动态波次 (001-1-1.csv 等) |

### 约束检查（9 项 Status）

| Status | 约束 | 说明 |
|--------|------|------|
| 1 | 数据格式 | 卫星/任务/地面站名称校验 |
| 2 | 侧摆角 | left_roll ≤ roll ≤ right_roll，禁飞区 |
| 3 | 开机时间 | startup_min ≤ dur ≤ startup_max |
| 4 | 时间窗重叠 | 卫星/地面站冲突检测 (含姿态转换+反转+CC) |
| 5 | 数传/成像比 | comm_min ≤ dl_dur/obs_dur ≤ comm_max |
| 6 | SAR 特有 | 侧摆反转 + 点目标固定 10s |
| 7 | 轨道累计 | 每 5400s 圈累计 ≤ orbit_max(600s) |
| 8 | 任务约束 | validity / weather / sunlight / time_interval |
| 9 | 覆盖范围 | 由评测工具计算 |

### 候选排序

按 `score / (OBS_dur + DL_dur)` 降序排列，单位时间收益高的任务优先调度。

### 动态波次

- **Type 1**（8h 间隔，2 波）：在 8h / 16h 追加新任务
- **Type 2**（2h 间隔，11 波）：在 2h / 4h / ... / 22h 追加新任务
- 每波次独立调度，评测工具按后缀时间范围拼接评分

## 关键 API

```python
from scheduler import schedule_missions, calc_score_rate

# 调度
entries, state = schedule_missions(
    sat_names, mission_names, sats, miss, expanded, stn_acc,
    base_mode=True  # OBS→DL；False = UL→OBS→DL
)

# 得分率统计
result = calc_score_rate(entries, mission_names, miss)
print(result['weighted_rate'])   # 加权得分率
print(result['mission_rate'])    # 任务覆盖率
print(result['per_mission'])     # 逐任务明细
```

## 评测结果（场景 001）

```
001-0: 任务=633.6  效率=46.3  约束=-1   总分=678.9
001-1: 任务=499.1  效率=47.0  约束=-1   总分=545.1
001-2: 任务=108.9  效率=41.3  约束=-4   总分=146.2
```
