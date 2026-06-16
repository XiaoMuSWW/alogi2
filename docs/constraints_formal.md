# 卫星调度问题形式化约束

## 1. 符号定义

### 集合与索引

| 符号              | 含义                                                             |
| ----------------- | ---------------------------------------------------------------- |
| $\mathcal{S}$   | 卫星集合，下标$s$，共 60 颗（光学 60 颗 + SAR 60 颗 = 120 颗） |
| $\mathcal{M}$   | 任务集合，下标$m$                                              |
| $\mathcal{C}$   | 候选观测窗口集合，下标$i$                                      |
| $\mathcal{C}_s$ | 卫星$s$ 上的候选集                                             |
| $\mathcal{C}_m$ | 任务$m$ 的候选集                                               |
| $\mathcal{G}$   | 地面站集合                                                       |
| $\mathcal{P}$   | 轨道周期集合，$p \in \{0, 1, \dots, 15\}$（每 5400s 一个周期） |

### 任务类型

| 类型   | 含义         | 侧摆角处理                             |
| ------ | ------------ | -------------------------------------- |
| `p`  | 点目标       | `check_roll_strict` 严格检查         |
| `ap` | 追加点目标   | `check_roll_strict` 严格检查         |
| `a`  | 区域目标     | `clamp_roll_area` 夹持（±3° 容差） |
| `aa` | 追加区域目标 | `clamp_roll_area` 夹持（±3° 容差） |

### 卫星属性

| 参数                                             | 含义                                             |
| ------------------------------------------------ | ------------------------------------------------ |
| $\text{type}_s \in \{\text{opt}, \text{sar}\}$ | 卫星类型                                         |
| $\theta_s^L, \theta_s^R$                       | 侧摆角左/右边界                                  |
| $\theta_s^{\text{lf}}, \theta_s^{\text{rf}}$   | SAR 禁飞区左/右边界                              |
| $\tau_s^{\text{min}}, \tau_s^{\text{max}}$     | 最短/最长开机时间（startup_min / startup_max）   |
| $\tau_s^{\text{orbit}}$                        | 轨道周期最大累计成像时间（orbit_max）            |
| $\tau_s^{[0]}, \tau_s^{[10]}, \tau_s^{[20]}$   | 光学卫星姿态转换时间（0-10° / 10-20° / 20°+） |
| $\tau_s^{\text{sar}}$                          | SAR 卫星姿态转换时间（trantime_sar）             |
| $\tau_s^{\text{rev}}$                          | SAR 反转时间（reversal_time）                    |
| $\tau_s^{\text{cc}}$                           | 测控/数传转换时间（trantime_cc）                 |

### 任务属性

| 参数                     | 含义                                                      |
| ------------------------ | --------------------------------------------------------- |
| $\text{score}_m$       | 单次观测得分                                              |
| $f_m$                  | 最大观测次数（frequency），$f_m \le 0$ 时取 $f_m = 1$ |
| $\Delta_m$             | p/ap 任务最小时间间隔（小时），0 表示无限制               |
| $V_m = \{(a_k, b_k)\}$ | 有效时间窗口集合                                          |
| $W_m = \{(a_k, b_k)\}$ | 天气窗口集合                                              |
| $L_m = \{(a_k, b_k)\}$ | 光照窗口集合（仅光学卫星）                                |

---

## 2. 候选窗口生成（预过滤）

每个候选 $i$ 对应一个三元组 $(s_i, m_i, [\text{os}_i, \text{oe}_i], \theta_i)$，生成时通过以下静态约束过滤：

### 2.1 侧摆角约束（Status 2）

**点目标 (p/ap)**：

$$
\theta_s^L \le \theta_i \le \theta_s^R \quad \land \quad \theta_i \notin [\theta_s^{\text{lf}}, \theta_s^{\text{rf}}]
$$

**区域目标 (a/aa)**：将 $\theta_i$ 夹持到最近有效边界：

$$
\theta_i' = \text{clamp}(\theta_i, \theta_s^L, \theta_s^R, \theta_s^{\text{lf}}, \theta_s^{\text{rf}})
$$

若 $|\theta_i' - \theta_i| > 3.0°$，丢弃该候选。

### 2.2 观测时长（Status 3 & 6）

| 条件             | 时长$\text{dur}_i$    |
| ---------------- | ----------------------- |
| SAR ∧ (p ∨ ap) | 10s                     |
| a ∨ aa          | $\tau_s^{\text{max}}$ |
| opt ∧ (p ∨ ap) | $\tau_s^{\text{min}}$ |

满足开机约束：$\tau_s^{\text{min}} \le \text{dur}_i \le \tau_s^{\text{max}}$

### 2.3 时间窗口（Status 8）

$$
\text{os}_i \in V_m \;\land\; \text{oe}_i \in V_m \;\land\; \text{os}_i \in W_m \;\land\; (\text{type}_s = \text{sar} \lor \text{os}_i \in L_m)
$$

| 子句 | 含义 |
|------|------|
| $\text{os}_i \in V_m$ | 观测开始时刻必须落在任务 $m$ 的有效时间窗口内 |
| $\text{oe}_i \in V_m$ | 观测结束时刻必须落在任务 $m$ 的有效时间窗口内 |
| $\text{os}_i \in W_m$ | 观测开始时刻必须满足任务 $m$ 的天气窗口要求（无天气限制时 $W_m = \emptyset$，始终通过） |
| $\text{type}_s = \text{sar} \lor \text{os}_i \in L_m$ | **光照条件**：SAR 卫星不受光照限制；光学卫星的观测时刻必须落在光照窗口 $L_m$ 内 |

> **注意**：有效窗口、天气窗口、光照窗口均为若干不相交区间的并集，即 $V_m = \{(a_1,b_1), (a_2,b_2), \dots\}$。$\text{os}_i \in V_m$ 当且仅当 $\exists (a,b) \in V_m: a \le \text{os}_i \le b$。

---

## 3. 决策变量

$$
x_i \in \{0, 1\}, \quad \forall i \in \mathcal{C}
$$

$x_i = 1$ 表示选择候选窗口 $i$。

---

## 4. 目标函数

$$
\max \sum_{i \in \mathcal{C}} \text{score}_{m_i} \cdot x_i
$$

> CP-SAT 要求整数系数，故在CP-SAT实际实现中乘以 $K = 100$ 后取整：$w_i = \lfloor \text{score}_{m_i} \cdot 100 + 0.5 \rfloor$

---

## 5. 约束条件

### 5.1 任务频次约束

$$
\forall m \in \mathcal{M}: \quad \sum_{i \in \mathcal{C}_m} x_i \le f_m
$$

### 5.2 卫星时间线互斥约束

对卫星 $s$ 上的任意两个候选 $i, j \in \mathcal{C}_s$，定义：

- 卫星占用窗口：$[\text{ul}_i,\; \text{sat\_end}_i]$，其中

  - 基线模式：$\text{ul}_i = \text{os}_i,\; \text{sat\_end}_i = \text{os}_i + \text{dl\_dur}_i - 1$
  - 波次模式：$\text{ul}_i = \text{os}_i - 10,\; \text{sat\_end}_i = \text{os}_i + \text{dl\_dur}_i - 1$
- 最小间隔：

  $$
  \text{gap}_{ij} = \tau^{\text{trans}}(\text{type}_s, \theta_i, \theta_j) + \tau^{\text{rev}}_s \cdot \mathbb{1}[\text{cross\_forbidden}(\theta_i, \theta_j)] + \tau_s^{\text{cc}}
  $$

  其中姿态转换时间：

  $$
  \tau^{\text{trans}} = \begin{cases}
  \tau_s^{\text{sar}} & \text{type}_s = \text{sar} \\
  \tau_s^{[0]} & |\theta_i - \theta_j| \le 10° \\
  \tau_s^{[10]} & 10° < |\theta_i - \theta_j| \le 20° \\
  \tau_s^{[20]} & |\theta_i - \theta_j| > 20°
  \end{cases}
  $$

  SAR 反转条件：

  $$
  \text{cross\_forbidden}(\theta_i, \theta_j) \iff (\theta_i \le \theta_s^{\text{lf}} \land \theta_j \ge \theta_s^{\text{rf}}) \lor (\theta_i \ge \theta_s^{\text{rf}} \land \theta_j \le \theta_s^{\text{lf}})
  $$

若两个候选在两种顺序下均无法共存，则互斥：

$$
\Big(\text{sat\_end}_i + \text{gap}_{ij} \ge \text{ul}_j\Big) \;\land\; \Big(\text{sat\_end}_j + \text{gap}_{ji} \ge \text{ul}_i\Big) \implies x_i + x_j \le 1
$$

等价地，定义冲突图 $\mathcal{G}_s^{\text{conf}} = (\mathcal{C}_s, \mathcal{E}_s)$：

$$
\mathcal{E}_s = \left\{(i,j) \;\middle|\; \begin{aligned} &\text{sat\_end}_i + \text{gap}_{ij} \ge \text{ul}_j \\ \land\;&\text{sat\_end}_j + \text{gap}_{ji} \ge \text{ul}_i \end{aligned}\right\}
$$

$$
\forall (i,j) \in \mathcal{E}_s: \quad x_i + x_j \le 1
$$

### 5.3 轨道累计时长约束

$$
\forall s \in \mathcal{S},\; \forall p \in \mathcal{P}: \quad \sum_{\substack{i \in \mathcal{C}_s \\ \lfloor \text{os}_i / 5400 \rfloor = p}} \text{dur}_i \cdot x_i \le \tau_s^{\text{orbit}}
$$

### 5.4 最小时间间隔约束（p/ap 任务）

$$
\forall m \in \{p, ap\}, \;\forall i,j \in \mathcal{C}_m: \quad |\text{os}_i - \text{os}_j| < \Delta_m \cdot 3600 \implies x_i + x_j \le 1
$$

### 5.5 点↔区域任务切换约束

在时间线插入检查中，若新任务 $i$ 与前后已调度任务类型不同（点 ↔ 区域），最小间隔至少为 $\tau_s^{\text{sar}}$：

$$
\text{gap}_{ij}' = \max\left(\text{gap}_{ij},\; \tau_s^{\text{sar}} \cdot \mathbb{1}[\text{type}(m_i) \neq \text{type}(m_j)]\right)
$$

---

## 6. 地面站约束（后处理阶段）

CP-SAT 模型求解后，对选中候选按分数降序进行地面站贪心分配。

### 6.1 地面站覆盖

候选 $i$ 可使用地面站 $g$ 当且仅当：

$$
\exists (t_s, t_e) \in \text{Access}(s_i, g): \quad t_s \le \text{stn\_start}_i \;\land\; t_e \ge \text{stn\_end}_i
$$

其中地面站占用窗口：

- 基线模式：$[\text{stn\_start}_i,\; \text{stn\_end}_i] = [\text{os}_i,\; \text{os}_i + \text{dl\_dur}_i - 1]$
- 波次模式：$[\text{stn\_start}_i,\; \text{stn\_end}_i] = [\text{os}_i - 10,\; \text{os}_i + \text{dl\_dur}_i - 1]$

### 6.2 地面站互斥

地面站 $g$ 上两个占用不冲突当且仅当：

$$
\text{end}_k + \tau_{s_k}^{\text{cc}} \cdot \mathbb{1}[s_k \neq s_j] < \text{start}_j \;\lor\; \text{end}_j + \tau_{s_j}^{\text{cc}} \cdot \mathbb{1}[s_j \neq s_k] < \text{start}_k
$$

### 6.3 数传/成像时间比（Status 5）

$$
\forall \text{观测} i: \quad \text{comm\_min}_{m_i} \le \frac{\text{dl\_dur}_i}{\text{dur}_i} \le \text{comm\_max}_{m_i}
$$

默认 $\text{comm\_min}_m = 2.0$，$\text{comm\_max}_m = 2.5$。

---

## 7. CP-SAT 求解参数

| 参数                    | 值  | 说明             |
| ----------------------- | --- | ---------------- |
| `max_time_in_seconds` | 30  | 单次求解时间上限 |
| `num_workers`         | 4   | 并行搜索线程数   |
| `SCORE_SCALE`         | 100 | 浮点分数放大倍数 |

---

## 8. 算法复杂度

| 阶段        | 复杂度                               | 说明                                    |
| ----------- | ------------------------------------ | --------------------------------------- |
| 候选收集    | $O(n)$                             | $n = \vert\mathcal{C}\vert$，顺序遍历 |
| 冲突图构建  | $O(n \cdot \overline{d})$          | $\overline{d}$ 为每颗星平均冲突度     |
| CP-SAT 求解 | NP-hard                              | 分支定界 + 约束传播，单次 30s 时限      |
| 地面站分配  | $O(k \cdot \vert\mathcal{G}\vert)$ | $k$ 为 CP-SAT 选中的候选数，贪心分配  |
