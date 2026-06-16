# 多星多任务调度优化系统

基于动态时间窗口的对地观测卫星调度算法，实现 60 颗光学 + 60 颗 SAR 卫星对 805 个任务的协同观测规划。

提供两种求解器：
- **V1** (`scheduler.py`)：任务优先贪心调度
- **V2** (`scheduler_v2.py`)：基于 Google OR-Tools CP-SAT 的全局最优调度

## 环境配置

```bash
# 从 environment.yml 创建 conda 环境
conda env create -f environment.yml

# 激活环境
conda activate alogi2
```

## 前置步骤（必须先运行）

```bash
# 1. 轨道外推 — 从 TLE 生成 120 颗卫星的轨道数据 (.ipc)
./orbit_propagator.exe -d data/

# 2. 时间窗口计算 — 基于轨道数据计算卫星-任务-地面站 access 窗口 (access.csv)
./time_window.exe -d data/
```

> `data/access.csv` 和 `data/orbit/` 由上述工具生成，不包含在 git 仓库中。

## 快速开始

```bash
# === V1 贪心调度 ===
python main.py --scenario 001           # 处理场景 001
python main.py --scenario 001 --evaluate # 处理 + 评测
python main.py --all                     # 批量处理全部 300 个场景

# === V2 CP-SAT 最优调度 ===
python main.py --scenario 001 --v2       # 使用 V2 处理场景 001
python main.py --all --v2                # 使用 V2 处理全部场景
```

## 项目结构

```
alogi2/
├── config.py              # 路径 + 全局常量
├── data_loader.py         # 数据加载（卫星/任务/access）
├── constraints.py         # 约束检查模块（Status 2-8）
├── window_processer.py    # 窗口扩展 + 波次状态过滤（共享模块）
├── scheduler.py           # V1 贪心调度核心
├── scheduler_v2.py        # V2 CP-SAT 最优调度核心
├── output_writer.py       # CSV 输出
├── main.py                # CLI 入口 + 场景处理流水线
├── environment.yml        # Conda 环境配置
├── docs/
│   └── constraints_formal.md  # 形式化数学约束文档
├── data/                  # 输入数据
│   ├── satellite.csv      # 120 颗卫星参数
│   ├── mission.csv        # 805 个任务参数
│   ├── tle.txt            # TLE 轨道根数
│   ├── access.csv         # 预计算访问窗口
│   ├── orbit/             # 120 个 .ipc 轨道数据文件
│   └── scenario/          # 300 个测试场景 (001-300)
├── result/                # 调度结果输出
├── orbit_propagator.exe   # 轨道外推工具
├── time_window.exe        # 时间窗口计算工具
└── evaluation_pub.exe     # 评测工具
```

## 模块依赖

```
config.py
 ├── data_loader.py → config
 ├── constraints.py → config
 ├── window_processer.py → config + constraints
 ├── scheduler.py → config + constraints
 ├── scheduler_v2.py → config + constraints + scheduler (expand_windows, filter_state_before)
 ├── output_writer.py
 └── main.py → config + data_loader + window_processer + scheduler[_v2] + output_writer
```

## 算法概要

### 任务类型

| 类型 | 含义 | 侧摆角处理 | 观测时长 |
|------|------|-----------|---------|
| `p` | 点目标 | 严格检查 | SAR→10s，光学→startup_min |
| `ap` | 追加点目标 | 严格检查 | SAR→10s，光学→startup_min |
| `a` | 区域目标 | 夹持（±3°容差） | startup_max |
| `aa` | 追加区域目标 | 夹持（±3°容差） | startup_max |

### 两种调度模式

| 模式 | 时序 | CSV 行/观测 | 卫星占用窗 | 地面站占用窗 |
|------|------|-----------|----------|-----------|
| **base_mode** | OBS ∥ DL | 2 | [os, os+dl_dur-1] | [os, os+dl_dur-1] |
| **wave_mode** | UL→OBS, OBS ∥ DL | 3 | [os-10, os+dl_dur-1] | [os-10, os+dl_dur-1] |

### 约束体系

| Status | 约束 | 实现 |
|--------|------|------|
| 2 | 侧摆角 | `check_roll_strict` / `clamp_roll_area`，SAR 禁飞区过滤 |
| 3 | 开机时间 | startup_min ≤ dur ≤ startup_max |
| 4 | 时间线互斥 | `sat_timeline_gap` = 姿态转换 + SAR反转 + 测控转换 |
| 5 | 数传/成像比 | comm_min ≤ dl_dur/dur ≤ comm_max (默认 2.0~2.5) |
| 6 | SAR 特性 | 反转时间；点目标固定 10s；禁飞区穿越检测 |
| 7 | 轨道累计 | 每 5400s 周期累计 ≤ orbit_max(600s) |
| 8 | 任务窗口 | validity / weather / sunlight / time_interval |
| — | 地面站 | 覆盖检查 + 互斥（同星 0 gap，异星 trantime_cc gap） |
| — | 点↔区域切换 | 类型切换时至少需 trantime_sar 间隔 |

### V1 贪心调度 (`scheduler.py`)

- 候选排序：按 `score / (dur + dl_dur)` 单位时间收益降序
- 任务处理顺序：按 `score` 降序
- 两轮调度：第一轮贪心 + 第二轮重试失败候选
- 波次继承：`filter_state_before` 锁定前驱时段资源

### V2 CP-SAT 最优调度 (`scheduler_v2.py`)

- **决策变量**：$x_i \in \{0,1\}$（候选窗口选择），$y_{i,g} \in \{0,1\}$（地面站分配）
- **目标**：$\max \sum \text{score} \cdot x_i$
- **约束**：任务频次、卫星时间线互斥、轨道累积、时间间隔、地面站通道+互斥
- **求解器**：OR-Tools CP-SAT，30s 时限，4 线程并行
- **地面站**：纳入 CP-SAT 模型内部，非后处理
- 形式化数学文档见 [docs/constraints_formal.md](docs/constraints_formal.md)

### 动态波次

| 类型 | 间隔 | 波次数 | 追加时机 |
|------|------|--------|---------|
| Type 1 | 8h | 2 波 | 8h, 16h |
| Type 2 | 2h | 11 波 | 2h, 4h, …, 22h |

每波次独立调度，继承基础方案在 `[0, wave_offset)` 内的锁定资源。

## 评测结果

### 场景 001（样例）

| 版本 | 场景 | 点得分 | 区域得分 | 约束扣分 | 总分 |
|------|------|--------|---------|---------|------|
| V1 贪心 | 001-0 | 633.6 | 46.3 | -1.0 | 678.9 |
| V1 贪心 | 001-1 | 498.3 | 47.0 | -1.0 | 544.3 |
| V1 贪心 | 001-2 | 108.9 | 41.3 | -4.0 | 146.2 |
| **V2 CP-SAT** | **001-0** | **759.0** | **44.6** | **-3.0** | **800.6** |
| **V2 CP-SAT** | **001-1** | **563.9** | **19.5** | **-3.0** | **580.4** |
| **V2 CP-SAT** | **001-2** | **166.4** | **5.9** | **-0.3** | **172.0** |

### 多场景对比（V1 vs V2 CP-SAT，仅基础方案观测数）

| 场景 | V1 贪心 | V2 CP-SAT | 提升 |
|------|---------|-----------|------|
| 001 | 293 | 320 | +9.2% |
| 002 | 275 | 314 | +14.2% |
| 003 | 246 | 281 | +14.2% |

## 参考文献

- [OR-Tools CP-SAT Guide](https://developers.google.com/optimization/cp/cp_solver)
- 形式化约束：[docs/constraints_formal.md](docs/constraints_formal.md)
