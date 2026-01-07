# PyVRP 算法流程与架构文档

本文档详细描述了 PyVRP 的整体架构、各文件的主要作用、调用关系以及完整的算法执行流程。

## 目录

1. [整体架构概述](#整体架构概述)
2. [Python 接口层详细说明](#python-接口层详细说明)
3. [C++ 核心数据结构层](#c-核心数据结构层)
4. [C++ 搜索层](#c-搜索层)
5. [完整调用流程](#完整调用流程)
6. [关键数据结构说明](#关键数据结构说明)

---

## 整体架构概述

PyVRP 采用**分层架构**，分为 Python 接口层和 C++ 核心层：

- **Python 层**：负责问题建模、参数配置、算法流程控制、统计信息收集和结果展示
- **C++ 层**：提供高性能的数据结构、解表示、局部搜索算子和成本评估
- **核心算法**：基于**迭代局部搜索（Iterated Local Search, ILS）**，结合自适应惩罚机制、扰动策略和粒度邻域

### 架构图

```
用户代码
    ↓
Python 接口层 (Model.py, solve.py, IteratedLocalSearch.py, ...)
    ↓
C++ 核心层 (_pyvrp: ProblemData, Solution, LocalSearch, CostEvaluator, ...)
    ↓
算法执行 (ILS主循环 → 局部搜索 → 算子评估 → 解更新)
    ↓
结果返回 (Result: best solution + statistics)
```

---

## Python 接口层详细说明

### 1. `__init__.py` - 模块导出入口

**主要作用：**
- 统一导出 PyVRP 的主要类和函数，提供简洁的 API
- 配置日志系统，将 `"pyvrp"` 日志输出到标准输出

**导出的主要类/函数：**
- **建模相关**：`Model`, `Edge`, `Profile`
- **求解相关**：`solve`, `SolveParams`, `IteratedLocalSearch`, `IteratedLocalSearchParams`
- **惩罚管理**：`PenaltyManager`, `PenaltyParams`
- **结果与统计**：`Result`, `Statistics`
- **C++ 核心类型**：`ProblemData`, `Solution`, `Route`, `Trip`, `Client`, `Depot`, `VehicleType` 等
- **工具函数**：`read`, `read_solution`, `minimise_fleet`, `show_versions`

**调用关系：**
- 用户通过 `from pyvrp import Model, solve, ...` 导入所需功能
- 所有导入都经过此文件统一管理

---

### 2. `Model.py` - Python 端建模接口

**主要类：**

#### `Edge` 类
- **作用**：表示两个位置之间的一条边
- **属性**：
  - `frm`, `to`: 起始和目标位置（`Client` 或 `Depot` 对象）
  - `distance`, `duration`: 边的距离和持续时间（整数）
- **验证**：不允许负值，自环的距离和持续时间必须为 0

#### `Profile` 类
- **作用**：路线配置（routing profile），定义一套完整的距离和持续时间矩阵
- **用途**：可以为不同车辆类型定义不同的道路网络（如卡车、汽车、自行车）
- **属性**：
  - `edges`: 边列表，定义该配置的所有边
  - `name`: 配置名称

#### `Model` 类
- **作用**：面向用户的建模接口，提供逐步构建 VRP 问题的方法

**核心方法：**

1. **添加元素方法：**
   - `add_client(...)`: 添加客户
     - 参数：坐标、配送量/拾取量、服务时间、时间窗、释放时间、奖赏、是否必需、客户组等
   - `add_client_group(required, name)`: 添加客户组（支持互斥组）
   - `add_depot(...)`: 添加仓库（坐标、时间窗）
   - `add_profile(name)`: 添加路线配置
   - `add_edge(frm, to, distance, duration, profile=None)`: 添加边
     - 如果 `profile=None`，作为基础边（所有配置共享）
     - 如果指定 `profile`，只覆盖该配置的对应边
   - `add_vehicle_type(...)`: 添加车辆类型
     - 参数：可用数量、容量、起止仓库、固定成本、班次时间窗、最大距离/时间、成本系数、配置索引、重装载相关参数等

2. **问题数据生成：**
   - `data(missing_value=MAX_VALUE) -> ProblemData`
     - 构建基础距离和持续时间矩阵（用 `missing_value` 填充缺失边）
     - 为每个 `Profile` 创建配置特定的矩阵（基于基础矩阵，用 profile 的边覆盖）
     - 如果没有 profile，自动创建一个基于基础矩阵的默认 profile
     - 返回 C++ `ProblemData` 对象

3. **求解方法：**
   - `solve(stop, seed, collect_stats, display, params, missing_value) -> Result`
     - 内部调用 `data()` 生成 `ProblemData`
     - 调用 `pyvrp.solve.solve(...)` 执行求解

**调用关系：**
```
用户代码
  → Model.add_xxx()  (逐步构建问题)
  → Model.data()     (生成 ProblemData)
  → Model.solve()    (调用 solve.py 的 solve 函数)
```

**辅助结构：**
- `_idx_by_id(item, container)`: 通过对象身份（`is`）而非值相等查找索引，避免对象比较陷阱

---

### 3. `read.py` - VRPLIB 文件读取

**主要函数：**

#### `read(where, round_func="none") -> ProblemData`
- **作用**：从 VRPLIB 格式文件读取问题实例
- **流程**：
  1. 使用 `vrplib.read_instance(where)` 读取文件
  2. 创建 `_InstanceParser` 解析数据
  3. 创建 `_ProblemDataBuilder` 构建 `ProblemData`
  4. 返回构建好的 `ProblemData` 对象

**辅助类：**

#### `_InstanceParser` 类
- **作用**：解析 VRPLIB 数据，拾取各种字段
- **主要方法**：
  - `num_locations`, `num_depots`, `num_clients`, `num_vehicles`: 获取基本计数
  - `coords()`, `demands()`, `backhauls()`, `service_times()`, `time_windows()`, `release_times()`, `prizes()`: 获取客户相关数据
  - `capacities()`, `vehicles_depot()`, `max_distances()`, `shift_durations()`, `reload_depots()`, `allowed_clients()`: 获取车辆相关数据
  - `edge_weight()`: 获取边权重矩阵
  - `mutually_exclusive_groups()`: 获取互斥客户组
- **舍入处理**：使用 `round_func` 对所有数值进行统一舍入（支持 `"round"`, `"trunc"`, `"dimacs"`, `"exact"`, `"none"`）

#### `_ProblemDataBuilder` 类
- **作用**：从解析器数据构建 C++ `ProblemData` 对象
- **主要方法**：
  - `_depots()`: 构建仓库列表（验证仓库索引必须连续）
  - `_clients()`: 构建客户列表（处理配送量、拾取量、时间窗、奖赏、客户组等）
  - `_vehicle_types()`: 构建车辆类型列表（按属性分组车辆）
  - `_distance_matrices()`: 构建距离矩阵列表（处理 VRPB 约束、允许客户限制等）
  - `_groups()`: 构建客户组列表
  - `_allowed2profile()`: 将允许客户集合映射到配置索引

#### `read_solution(where, data) -> Solution`
- **作用**：从 VRPLIB 格式文件读取解
- **流程**：
  1. 使用 `vrplib.read_solution(...)` 读取解文件
  2. 根据车辆类型名称中的编码恢复车辆类型索引
  3. 将每条 route 按仓库位置分割成多个 `Trip`
  4. 构建 `Route` 和 `Solution` 对象

**调用关系：**
```
用户代码
  → read("instance.vrp")           (读取问题)
  → read_solution("sol.sol", data) (读取解，可选)
  → solve(data, ...)               (求解)
```

---

### 4. `solve.py` - Python 层统一求解入口

#### `SolveParams` 类
- **作用**：封装所有求解参数
- **主要属性：**
  - `ils`: `IteratedLocalSearchParams`（重启频率、接受准则参数等）
  - `penalty`: `PenaltyParams`（惩罚更新频率、上下界、增减因子等）
  - `neighbourhood`: `NeighbourhoodParams`（粒度邻域参数）
  - `node_ops`: 节点算子类型列表（如 `[Relocate, Swap, ...]`）
  - `route_ops`: 路径算子类型列表（如 `[RouteExchange, ...]`）
  - `display_interval`: 进度显示间隔（秒）
  - `perturbation`: `PerturbationParams`（扰动次数范围）
  - `initial_solution`: 初始解（可选，默认自动生成）

- **类方法：**
  - `from_file(loc)`: 从 TOML 文件加载参数配置

#### `solve(data, stop, seed, collect_stats, display, params) -> Result`
- **作用**：求解 VRP 问题的主入口函数
- **详细流程：**

1. **初始化随机数生成器：**
   ```python
   rng = RandomNumberGenerator(seed=seed)
   ```

2. **计算邻域结构：**
   ```python
   neighbours = compute_neighbours(data, params.neighbourhood)
   ```
   - 为每个客户计算粒度邻域（有限个最近邻居）
   - 返回 `Neighbours` 结构供 C++ `SearchSpace` 使用

3. **创建扰动管理器：**
   ```python
   perturbation = PerturbationManager(params.perturbation)
   ```

4. **创建局部搜索对象（C++）：**
   ```python
   ls = LocalSearch(data, rng, neighbours, perturbation)
   ```

5. **注册节点算子：**
   ```python
   for node_op in params.node_ops:
       if node_op.supports(data):
           ls.add_node_operator(node_op(data))
   ```
   - 检查每个算子是否支持当前问题数据
   - 如果支持，创建算子实例并注册到局部搜索

6. **注册路径算子：**
   ```python
   for route_op in params.route_ops:
       if route_op.supports(data):
           ls.add_route_operator(route_op(data))
   ```

7. **初始化惩罚管理器：**
   ```python
   pm = PenaltyManager.init_from(data, params.penalty)
   ```
   - 根据问题数据自动计算初始惩罚值
   - 使惩罚值与问题的规模相匹配

8. **生成初始解：**
   ```python
   init = params.initial_solution
   if init is None:
       init = ls.search(Solution(data, []), pm.max_cost_evaluator())
   ```
   - 如果未提供初始解，从空解开始
   - 使用最大惩罚值强制生成可行解

9. **创建并运行迭代局部搜索：**
   ```python
   algo = IteratedLocalSearch(data, pm, rng, ls, init, params.ils)
   return algo.run(stop, collect_stats, display, params.display_interval)
   ```

**调用关系：**
```
用户代码 / Model.solve()
  → solve(data, ...)
    → IteratedLocalSearch.run()
      → LocalSearch (C++) 的 search() / intensify()
        → NodeOperator / RouteOperator (C++)
```

---

### 5. `IteratedLocalSearch.py` - 迭代局部搜索主算法

#### `IteratedLocalSearchParams` 数据类
- **参数说明：**
  - `num_iters_no_improvement`: 无改进迭代次数阈值，达到后触发重启（默认 20000）
  - `initial_accept_weight`: 初始接受权重，控制接受准则的严格程度（范围 [0,1]，默认 1）
  - `history_length`: 历史记录长度，用于计算接受阈值（默认 500）

#### `IteratedLocalSearch` 类
- **成员变量：**
  - `_data`: `ProblemData`（问题数据）
  - `_pm`: `PenaltyManager`（惩罚管理器）
  - `_rng`: `RandomNumberGenerator`（随机数生成器）
  - `_search`: `SearchMethod`（搜索方法，实际是 C++ `LocalSearch`）
  - `_init`: `Solution`（初始解）
  - `_params`: `IteratedLocalSearchParams`（参数）

#### `run()` 方法 - ILS 主循环

**详细流程：**

1. **初始化：**
   ```python
   print_progress = ProgressPrinter(display, display_interval)
   history = History(size=history_length)
   stats = Statistics(collect_stats=collect_stats)
   start = time.perf_counter()
   iters = iters_no_improvement = 0
   best = current = self._init
   ```

2. **主循环：** `while not stop(cost_eval.cost(best))`
   
   **每次迭代执行：**
   
   a. **更新计数器：**
      ```python
      iters += 1
      iters_no_improvement += 1
      ```
   
   b. **检查重启条件：**
      ```python
      if iters_no_improvement == num_iters_no_improvement:
          print_progress.restart()
          history.clear()
          current = best  # 重置为最优解
          iters_no_improvement = 0
      ```
   
   c. **获取当前成本评估器：**
      ```python
      cost_eval = self._pm.cost_evaluator()
      ```
      - 惩罚值可能在上次迭代中已更新
   
   d. **执行局部搜索：**
      ```python
      candidate = self._search(current, cost_eval)
      ```
      - 调用 C++ `LocalSearch` 的 `operator()`
      - 内部执行扰动 + 节点级搜索 + 路径级搜索
      - 返回改进后的候选解
   
   e. **注册候选解到惩罚管理器：**
      ```python
      self._pm.register(candidate)
      ```
      - 记录解的可行性统计
      - 可能触发惩罚值更新
   
   f. **更新最优解：**
      ```python
      if cost_eval.cost(candidate) < cost_eval.cost(best):
          best = candidate
          iters_no_improvement = 0
      ```
      - 使用无惩罚成本比较（仅目标函数值）
   
   g. **记录历史：**
      ```python
      cand_cost = cost_eval.penalised_cost(candidate)
      history.append(cand_cost)
      ```
   
   h. **接受准则（基于历史的自适应阈值）：**
      ```python
      weight = initial_accept_weight
      if stop.fraction_remaining() is not None:
          weight *= fraction  # 根据剩余时间调整权重
      
      best_weight = (1 - weight) * history.min()
      mean_weight = weight * history.mean()
      
      if cand_cost <= best_weight + mean_weight:
          current = candidate  # 接受候选解
      ```
      - 阈值 = `(1-weight) * 历史最小值 + weight * 历史平均值`
      - 权重越大，接受越宽松（探索更多）
      - 随着时间推移，权重减小（更保守）
   
   i. **收集统计信息：**
      ```python
      stats.collect(current, candidate, best, cost_eval)
      print_progress.iteration(stats)
      ```

3. **结束处理：**
   ```python
   runtime = time.perf_counter() - start
   res = Result(best, stats, iters, runtime)
   print_progress.end(res)
   return res
   ```

#### `History` 辅助类
- **作用**：管理最近候选解成本值的历史记录
- **实现**：使用固定大小的 NumPy 数组作为循环缓冲区
- **方法：**
  - `append(value)`: 添加新的成本值（覆盖最旧的值）
  - `min()`: 返回历史最小值（忽略 NaN）
  - `mean()`: 返回历史平均值（忽略 NaN）
  - `clear()`: 清空历史记录

**调用关系：**
```
IteratedLocalSearch.run()
  → LocalSearch (C++) operator()
    → PerturbationManager.perturb()
    → LocalSearch.search()      (节点级搜索)
    → LocalSearch.intensify()    (路径级搜索)
  → PenaltyManager.register()
  → Statistics.collect()
```

---

### 6. `PenaltyManager.py` - 惩罚项管理与自适应调节

#### `PenaltyParams` 数据类
- **参数说明：**
  - `solutions_between_updates`: 每注册多少解后更新一次惩罚（默认 500）
  - `penalty_increase`: 惩罚增加系数，当可行解不足时使用（默认 1.25）
  - `penalty_decrease`: 惩罚减少系数，当可行解过多时使用（默认 0.85）
  - `target_feasible`: 目标可行解比例（默认 0.90）
  - `feas_tolerance`: 可行比例容差（默认 0.05）
  - `min_penalty`, `max_penalty`: 惩罚值上下界（默认 0.1, 100000.0）

#### `PenaltyManager` 类
- **成员变量：**
  - `_penalties`: NumPy 数组，存储当前惩罚值 `[load_dims..., tw_penalty, dist_penalty]`
  - `_feas_lists`: 每个惩罚维度一个列表，记录最近解的可行性

#### 核心方法：

1. **`init_from(data, params) -> PenaltyManager`（类方法）：**
   - **作用**：根据问题数据自动计算初始惩罚值
   - **计算流程：**
     ```
     1. 计算所有车辆类型和配置下的最小边成本矩阵
     2. 计算平均边成本、平均距离、平均持续时间
     3. 计算平均负载（取配送量和拾取量的最大值）
     4. 初始惩罚 = 平均边成本 / 平均约束值
        - init_load = avg_cost / max(avg_load, 1)
        - init_tw = avg_cost / max(avg_duration, 1)
        - init_dist = avg_cost / max(avg_distance, 1)
     ```
   - **原理**：使单位约束违反的惩罚成本 ≈ 单位边成本，平衡目标函数和惩罚项

2. **`register(sol)` 方法：**
   - **作用**：注册解的可行性，可能触发惩罚值更新
   - **流程：**
     ```
     1. 检查解的可行性维度：
        - 每个负载维度是否超载
        - 是否有时间扭曲
        - 是否超距
     
     2. 对每个惩罚维度：
        a. 将可行性添加到 _feas_lists[idx]
        b. 如果列表长度达到 solutions_between_updates：
           - 计算可行性百分比
           - 调用 _compute() 更新惩罚值
           - 清空列表
     ```

3. **`_compute(penalty, feas_percentage) -> float` 方法：**
   - **作用**：根据当前惩罚值和可行性百分比计算新惩罚值
   - **逻辑：**
     ```
     diff = target_feasible - feas_percentage
     
     if abs(diff) < feas_tolerance:
         return penalty  # 无需更新
     
     if diff > 0:  # 可行解不足
         new_penalty = penalty_increase * penalty
     else:  # 可行解过多
         new_penalty = penalty_decrease * penalty
     
     return clip(new_penalty, min_penalty, max_penalty)
     ```

4. **`cost_evaluator() -> CostEvaluator`：**
   - 使用当前惩罚值创建 C++ `CostEvaluator`

5. **`max_cost_evaluator() -> CostEvaluator`：**
   - 使用最大惩罚值创建 C++ `CostEvaluator`（用于强制可行性）

**调用关系：**
```
IteratedLocalSearch.run()
  → PenaltyManager.register(candidate)
    → _register()  (记录可行性)
    → _compute()   (可能更新惩罚值)
  → PenaltyManager.cost_evaluator()
    → CostEvaluator (C++)  (用于评估解)
```

---

### 7. `Statistics.py` - 统计信息收集

#### `_Datum` 数据类
- **作用**：存储单次迭代的数据点
- **字段：**
  - `current_cost`, `current_feas`: 当前解的惩罚成本和可行性
  - `candidate_cost`, `candidate_feas`: 候选解的惩罚成本和可行性
  - `best_cost`, `best_feas`: 最优解的惩罚成本和可行性

#### `Statistics` 类
- **成员变量：**
  - `runtimes`: 每次迭代的运行时间列表
  - `num_iterations`: 迭代次数计数器
  - `data`: `_Datum` 对象列表
  - `_collect_stats`: 是否收集统计信息的标志

- **主要方法：**
  - `collect(current, candidate, best, cost_evaluator)`: 收集一次迭代的统计信息
  - `from_csv(where, ...)`: 从 CSV 文件读取统计信息
  - `to_csv(where, ...)`: 将统计信息写入 CSV 文件
  - `__iter__()`: 迭代器，遍历所有数据点

**调用关系：**
```
IteratedLocalSearch.run()
  → Statistics.collect(current, candidate, best, cost_eval)
    → 记录运行时间、成本、可行性等信息
```

---

### 8. `Result.py` - 求解结果封装

#### `Result` 数据类
- **成员变量：**
  - `best`: `Solution`（最优解）
  - `stats`: `Statistics`（统计信息）
  - `num_iterations`: 迭代次数
  - `runtime`: 运行时间（秒）

- **主要方法：**
  - `cost() -> float`: 返回最优解的目标函数值（不可行返回 `inf`）
  - `is_feasible() -> bool`: 返回最优解是否可行
  - `summary() -> str`: 返回格式化的结果摘要
  - `__str__() -> str`: 返回完整的结果字符串（摘要 + 路线详情）

**调用关系：**
```
IteratedLocalSearch.run()
  → Result(best, stats, iters, runtime)
    → 用户访问 result.best, result.cost(), result.summary() 等
```

---

## C++ 核心数据结构层

### 1. `ProblemData` (`pyvrp/cpp/ProblemData.*`)

**作用：** 存储 VRP 问题的所有数据

**主要结构：**

- **嵌套结构：**
  - `Client`: 客户数据（坐标、配送量、拾取量、服务时间、时间窗、释放时间、奖赏、是否必需、客户组索引）
  - `Depot`: 仓库数据（坐标、时间窗）
  - `VehicleType`: 车辆类型数据（可用数量、容量、起止仓库、固定成本、班次时间窗、最大距离/时间、成本系数、配置索引、重装载参数等）
  - `ClientGroup`: 客户组数据（客户索引列表、是否必需、是否互斥）

- **核心数据：**
  - `clients_`: 客户列表
  - `depots_`: 仓库列表
  - `vehicleTypes_`: 车辆类型列表
  - `groups_`: 客户组列表
  - `dists_`: 距离矩阵列表（每个配置一个）
  - `durs_`: 持续时间矩阵列表（每个配置一个）

- **辅助数据：**
  - `centroid_`: 客户位置的中心点
  - `numVehicles_`: 车辆总数
  - `numLoadDimensions_`: 负载维度数量
  - `hasTimeWindows_`: 是否存在时间窗约束

**主要方法：**
- 访问器：`clients()`, `depots()`, `vehicleTypes()`, `groups()`
- 矩阵访问：`distanceMatrix(profile)`, `durationMatrix(profile)`
- 计数：`numClients()`, `numDepots()`, `numVehicles()`, `numProfiles()`, `numLoadDimensions()`
- 位置访问：`location(idx)` - 返回 `Location` 联合体（客户或仓库）

---

### 2. 解表示层

#### `Trip` (`pyvrp/cpp/Trip.*`)
- **作用**：表示一次行程（从起始仓库到结束仓库的一段路径）
- **数据：**
  - `visits_`: 客户访问序列
  - `distance_`: 总距离
  - `delivery_`, `pickup_`, `load_`: 配送量、拾取量、负载曲线（每个维度）
  - `excessLoad_`: 超载量（每个维度）
  - `travel_`, `service_`: 旅行时间、服务时间
  - `release_`: 释放时间（最早可出发时间）
  - `prizes_`: 收集的奖赏
  - `centroid_`: 客户位置中心点
  - `vehicleType_`, `startDepot_`, `endDepot_`: 车辆类型和起止仓库索引

#### `Route` (`pyvrp/cpp/Route.*`)
- **作用**：表示一条完整的车辆路径（由多个 `Trip` 组成）
- **数据：**
  - `trips_`: `Trip` 列表
  - `schedule_`: `ScheduledVisit` 列表（时间安排）
  - `distance_`, `distanceCost_`, `excessDistance_`: 距离相关统计
  - `delivery_`, `pickup_`, `excessLoad_`: 负载相关统计（每个维度）
  - `duration_`, `overtime_`, `durationCost_`, `timeWarp_`: 时间相关统计
  - `travel_`, `service_`: 旅行时间、服务时间
  - `startTime_`, `slack_`: 开始时间、松弛时间
  - `prizes_`: 收集的奖赏
  - `centroid_`: 路径中心点
  - `vehicleType_`, `startDepot_`, `endDepot_`: 车辆类型和起止仓库

- **关键方法：**
  - `makeSchedule()`: 计算时间安排（考虑时间窗、等待时间、时间扭曲）
  - `validate()`: 验证路径的一致性
  - `update()`: 更新路径统计信息（在修改后调用）

#### `Solution` (`pyvrp/cpp/Solution.*`)
- **作用**：表示一个完整的 VRP 解（多辆车的路径集合）
- **数据：**
  - `routes_`: `Route` 列表
  - `neighbours_`: 邻居关系（每个客户的前驱和后继）
  - `numClients_`: 访问的客户数
  - `numMissingClients_`: 缺失的必需客户数
  - `distance_`, `distanceCost_`: 总距离和距离成本
  - `duration_`, `overtime_`, `durationCost_`: 总持续时间和时间成本
  - `excessDistance_`, `excessLoad_`, `timeWarp_`: 约束违反统计
  - `fixedVehicleCost_`: 固定车辆成本
  - `prizes_`, `uncollectedPrizes_`: 收集和未收集的奖赏
  - `isGroupFeas_`: 是否满足客户组约束

- **关键方法：**
  - `evaluate()`: 评估解，计算所有统计信息
  - `makeNeighbours()`: 构建邻居关系
  - `isFeasible()`: 检查解是否可行

---

### 3. `CostEvaluator` (`pyvrp/cpp/CostEvaluator.*`)

**作用：** 统一计算解的成本（目标函数 + 惩罚项）

**惩罚项：**
- 负载惩罚：`loadPenalties_[dim] * excessLoad[dim]`
- 时间扭曲惩罚：`twPenalty_ * timeWarp`
- 距离惩罚：`distPenalty_ * excessDistance`

**主要方法：**
- `penalisedCost(solution/route/trip)`: 返回惩罚成本（目标函数 + 惩罚项）
- `cost(solution/route/trip)`: 返回成本（可行解返回惩罚成本，不可行返回 `max`）
- `deltaCost(proposal)`: 计算增量成本（用于算子评估）

**调用关系：**
```
PenaltyManager.cost_evaluator()
  → CostEvaluator(load_penalties, tw_penalty, dist_penalty)
    → LocalSearch / NodeOperator / RouteOperator 使用
      → 评估解和移动的成本
```

---

## C++ 搜索层

### 1. `LocalSearch` (`pyvrp/cpp/search/LocalSearch.*`)

**作用：** 执行局部搜索，应用节点和路径算子改进解

**成员变量：**
- `data`: `ProblemData` 引用
- `solution_`: 当前解（内部表示，支持快速修改）
- `searchSpace_`: `SearchSpace` 对象（管理邻域和 promising 标记）
- `perturbationManager_`: `PerturbationManager` 引用
- `nodeOps`, `routeOps`: 节点和路径算子列表
- `lastTestedNodes`, `lastTestedRoutes`, `lastUpdated`: 用于避免重复评估

**核心方法：**

1. **`operator()(solution, costEvaluator) -> Solution`：**
   - **流程：**
     ```
     1. loadSolution(solution)  (加载解到内部表示)
     2. perturbationManager_.perturb(solution_, searchSpace_, costEvaluator)
        (执行扰动)
     3. while True:
          a. search(costEvaluator)      (节点级搜索)
          b. intensify(costEvaluator)   (路径级搜索)
          c. 如果 intensify 没有改进，退出循环
     4. return solution_.unload()  (返回改进后的解)
     ```

2. **`search(costEvaluator)` 方法（节点级搜索）：**
   - **流程：**
     ```
     1. markRequiredMissingAsPromising()  (标记缺失的必需客户)
     2. 迭代直到 searchCompleted_ == True:
         a. 遍历 promising clients (按 clientOrder):
            - applyOptionalClientMoves()  (插入/删除可选客户)
            - applyGroupMoves()           (处理互斥客户组)
            - applyDepotRemovalMove()     (删除不必要的重装载仓库)
            - 遍历邻居客户，应用 NodeOperator:
              * evaluate() 计算增量成本
              * 如果改进，apply() 执行移动，update() 更新
            - applyEmptyRouteMoves()      (利用空路径)
     ```

3. **`intensify(costEvaluator)` 方法（路径级搜索）：**
   - **流程：**
     ```
     迭代直到 searchCompleted_ == True:
       遍历路径对 (U, V) (按 routeOrder):
         应用 RouteOperator:
           * evaluate() 计算增量成本
           * 如果改进，apply() 执行移动，update() 更新
     ```

4. **`update(U, V)` 方法：**
   - 更新受影响的路径统计信息
   - 更新 `lastUpdated` 时间戳
   - 通知 route operators 更新缓存

**调用关系：**
```
IteratedLocalSearch.run()
  → LocalSearch.operator()(current, cost_eval)
    → PerturbationManager.perturb()
    → LocalSearch.search()
      → NodeOperator.evaluate() / apply()
    → LocalSearch.intensify()
      → RouteOperator.evaluate() / apply()
```

---

### 2. `SearchSpace` (`pyvrp/cpp/search/SearchSpace.*`)

**作用：** 管理搜索空间，包括粒度邻域和 promising 标记

**成员变量：**
- `neighbours_`: 每个客户的粒度邻域（有限个最近邻居）
- `promising_`: 位集，标记哪些客户当前"有希望"（值得重点搜索）
- `clientOrder_`: 客户搜索顺序（可 shuffle）
- `routeOrder_`: 路径搜索顺序（可 shuffle）
- `vehTypeOrder_`: 车辆类型顺序（可 shuffle）

**主要方法：**
- `isPromising(client)`: 检查客户是否 promising
- `markPromising(client/node)`: 标记客户为 promising（包括邻居）
- `markAllPromising()` / `unmarkAllPromising()`: 标记所有/取消所有
- `neighboursOf(client)`: 返回客户的邻居列表
- `shuffle(rng)`: 随机打乱搜索顺序

**调用关系：**
```
solve()
  → compute_neighbours(data, neighbourhood)
    → SearchSpace.setNeighbours(neighbours)
      → LocalSearch 使用 SearchSpace 限制搜索范围
```

---

### 3. `PerturbationManager` (`pyvrp/cpp/search/PerturbationManager.*`)

**作用：** 对解执行扰动，帮助跳出局部最优

**成员变量：**
- `params_`: `PerturbationParams`（最小/最大扰动次数）
- `numPerturbations_`: 当前扰动次数（在范围内随机）

**核心方法：**

**`perturb(solution, searchSpace, costEvaluator)`：**
- **流程：**
  ```
  1. searchSpace.unmarkAllPromising()  (清空 promising 标记)
  2. 初始化 perturbed 位集（记录已处理的客户）
  3. 遍历客户（按 clientOrder）：
     a. 确定动作：如果客户在解中 → REMOVE，否则 → INSERT
     b. 对客户执行动作：
        - REMOVE: 从路径中删除客户
        - INSERT: 插入客户到最佳位置
     c. 标记客户为 perturbed
     d. 对客户的邻居执行相同动作
     e. 如果达到 numPerturbations_，停止
  4. 标记受影响的客户为 promising
  ```

**调用关系：**
```
LocalSearch.operator()()
  → PerturbationManager.perturb()
    → 修改 solution（删除/插入客户）
    → 更新 searchSpace.promising
```

---

### 4. 局部搜索算子

#### `LocalSearchOperator` (`pyvrp/cpp/search/LocalSearchOperator.h`)
- **作用**：算子基类，定义统一接口
- **接口：**
  - `evaluate(...) -> deltaCost`: 评估移动的增量成本
  - `apply(...)`: 执行移动
  - `update(Route*)`: 更新算子内部缓存（可选）
  - `statistics()`: 返回统计信息（评估次数、应用次数）

#### `NodeOperator` / `RouteOperator`
- **作用**：节点级和路径级算子的基类
- **常见算子：**
  - **节点级**：`Relocate`（重定位）、`Swap`（交换）、`2Opt`（2-opt）、`Exchange`（交换）等
  - **路径级**：`RouteExchange`（路径交换）、`Cross`（交叉）等

**调用关系：**
```
LocalSearch.search() / intensify()
  → NodeOperator / RouteOperator.evaluate()
    → 计算增量成本（使用 CostEvaluator）
  → 如果改进：
     → NodeOperator / RouteOperator.apply()
       → 修改 Route / Solution
     → LocalSearch.update()
       → RouteOperator.update()  (更新缓存)
```

---

## 完整调用流程

### 整体流程图

```
用户代码
    │
    ├─→ 方式A: Model 建模
    │   Model.add_xxx()
    │   Model.data() → ProblemData
    │   Model.solve() ─┐
    │                   │
    └─→ 方式B: 读取文件 │
        read("file.vrp") → ProblemData
        solve(data, ...) ─┘
                        │
                        ↓
            solve.py::solve()
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ↓               ↓               ↓
    RandomNumber    compute_      Perturbation
    Generator      neighbours     Manager
        │               │               │
        └───────────────┼───────────────┘
                        │
                        ↓
            LocalSearch (C++)
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ↓               ↓               ↓
    NodeOperator   RouteOperator   SearchSpace
        │               │               │
        └───────────────┼───────────────┘
                        │
                        ↓
        PenaltyManager.init_from()
                        │
                        ↓
        IteratedLocalSearch.run()
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ↓               ↓               ↓
    History        Statistics      ProgressPrinter
        │               │               │
        └───────────────┼───────────────┘
                        │
                        ↓
                    Result
                        │
                        ↓
                    用户访问
```

### 详细执行流程

#### 阶段 1：问题建模与数据准备

1. **用户选择建模方式：**
   - **方式 A**：使用 `Model` 类逐步构建
     ```python
     model = Model()
     model.add_depot(x, y, ...)
     model.add_client(x, y, delivery=[...], ...)
     model.add_vehicle_type(...)
     data = model.data()  # 或直接 model.solve(...)
     ```
   - **方式 B**：从文件读取
     ```python
     data = read("instance.vrp")
     ```

2. **构建 C++ ProblemData：**
   - Python 层收集所有数据
   - 构建距离和持续时间矩阵
   - 调用 C++ `ProblemData` 构造函数
   - C++ 层验证数据一致性

#### 阶段 2：求解初始化

1. **调用 `solve(data, stop, seed, ...)`：**

2. **初始化组件：**
   ```python
   rng = RandomNumberGenerator(seed)
   neighbours = compute_neighbours(data, params.neighbourhood)
   perturbation = PerturbationManager(params.perturbation)
   ls = LocalSearch(data, rng, neighbours, perturbation)
   ```

3. **注册算子：**
   ```python
   for node_op in params.node_ops:
       if node_op.supports(data):
           ls.add_node_operator(node_op(data))
   
   for route_op in params.route_ops:
       if route_op.supports(data):
           ls.add_route_operator(route_op(data))
   ```

4. **初始化惩罚管理器：**
   ```python
   pm = PenaltyManager.init_from(data, params.penalty)
   # 内部计算初始惩罚值
   ```

5. **生成初始解：**
   ```python
   if params.initial_solution is None:
       init = ls.search(Solution(data, []), pm.max_cost_evaluator())
   ```

6. **创建 ILS 对象：**
   ```python
   algo = IteratedLocalSearch(data, pm, rng, ls, init, params.ils)
   ```

#### 阶段 3：迭代局部搜索主循环

**每次迭代执行：**

1. **检查停止条件：**
   ```python
   while not stop(cost_eval.cost(best)):
   ```

2. **检查重启条件：**
   ```python
   if iters_no_improvement == num_iters_no_improvement:
       current = best  # 重启
       history.clear()
   ```

3. **获取成本评估器：**
   ```python
   cost_eval = pm.cost_evaluator()  # 使用当前惩罚值
   ```

4. **执行局部搜索（C++）：**
   ```python
   candidate = ls(current, cost_eval)
   ```
   
   **内部流程：**
   ```
   a. loadSolution(current)
   b. perturbationManager.perturb()
      - 随机选择客户及其邻居
      - 执行 REMOVE / INSERT 操作
      - 更新 promising 标记
   c. while True:
      - search(cost_eval)  # 节点级搜索
        * 遍历 promising clients
        * 应用 NodeOperator
        * 如果改进，执行移动
      - intensify(cost_eval)  # 路径级搜索
        * 遍历路径对
        * 应用 RouteOperator
        * 如果改进，执行移动
      - 如果 intensify 没有改进，退出
   d. return solution_.unload()
   ```

5. **注册候选解：**
   ```python
   pm.register(candidate)
   ```
   - 记录可行性统计
   - 可能更新惩罚值

6. **更新最优解：**
   ```python
   if cost_eval.cost(candidate) < cost_eval.cost(best):
       best = candidate
   ```

7. **接受准则：**
   ```python
   history.append(cand_cost)
   weight = adjust_weight(...)
   threshold = (1-weight) * history.min() + weight * history.mean()
   if cand_cost <= threshold:
       current = candidate
   ```

8. **收集统计：**
   ```python
   stats.collect(current, candidate, best, cost_eval)
   print_progress.iteration(stats)
   ```

#### 阶段 4：结果返回

1. **构建结果对象：**
   ```python
   runtime = time.perf_counter() - start
   res = Result(best, stats, iters, runtime)
   ```

2. **用户访问结果：**
   ```python
   result.cost()           # 目标函数值
   result.is_feasible()   # 是否可行
   result.summary()        # 文本摘要
   result.best             # Solution 对象
   result.stats            # Statistics 对象
   ```

---

## 关键数据结构说明

### 1. 解的内部表示（C++ LocalSearch）

**`Solution` 的内部表示：**
- `nodes`: 节点数组，每个节点包含：
  - `client()`: 客户索引
  - `route()`: 所属路径指针
  - `idx()`: 在路径中的索引
  - `prev()` / `next()`: 前驱/后继节点指针
- `routes`: 路径数组

**优势：**
- 支持 O(1) 的节点访问和路径遍历
- 支持快速插入/删除操作
- 算子可以高效评估增量成本

### 2. 粒度邻域（Granular Neighbourhood）

**概念：**
- 不是所有客户对都值得尝试交换
- 只为每个客户保留有限个"最近邻居"
- 大幅减少搜索空间

**实现：**
- `SearchSpace.neighbours_[client]`: 客户索引列表
- 通常使用距离或成本作为相似度度量
- 在 `compute_neighbours()` 中预先计算

### 3. Promising 标记机制

**概念：**
- 不是所有客户都需要在每次迭代中搜索
- 只标记"有希望改进"的客户为 promising
- 进一步减少搜索开销

**标记时机：**
- 解被修改后，相关客户及其邻居标记为 promising
- 扰动后，受影响的客户标记为 promising
- 缺失的必需客户标记为 promising

### 4. 增量成本评估

**原理：**
- 算子不需要重新计算整个解的成本
- 只计算移动导致的成本变化（delta cost）
- 使用 proposal 结构（轻量级的解片段）

**优势：**
- 大幅提升评估速度
- 允许快速拒绝不改进的移动

### 5. 自适应惩罚机制

**原理：**
- 惩罚值不是固定的，而是根据搜索历史动态调整
- 如果可行解太少 → 增加惩罚（促进可行性）
- 如果可行解太多 → 减少惩罚（增加探索）

**实现：**
- `PenaltyManager` 维护每个维度的可行性历史
- 定期（每 `solutions_between_updates` 次）更新惩罚值
- 使用目标可行比例（`target_feasible`）作为调节目标

---

## 总结

PyVRP 采用**迭代局部搜索（ILS）**作为核心算法，结合以下关键技术：

1. **自适应惩罚机制**：动态调整约束违反惩罚，平衡可行性和探索性
2. **粒度邻域**：限制搜索范围，提高效率
3. **Promising 标记**：只搜索有希望的区域
4. **增量成本评估**：快速评估移动的成本变化
5. **扰动策略**：跳出局部最优
6. **多算子组合**：节点级和路径级算子协同工作

整个系统通过 Python 层提供友好的接口，C++ 层提供高性能的实现，实现了易用性和效率的平衡。

