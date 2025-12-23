# FILO-2优化技术代码实现示例

## 目录
1. [构想1：选择性顶点缓存（SVC）实现](#构想1选择性顶点缓存svc实现)
2. [构想3：SWAP*顺序搜索与剪枝](#构想3swap顺序搜索与剪枝)
3. [性能对比基准](#性能对比基准)

---

## 构想1：选择性顶点缓存（SVC）实现

### 方案A：轻量级SVC（推荐首先实施）

#### 步骤1：修改LocalSearch.h

在`LocalSearch`类的private部分添加：

```cpp
// 在LocalSearch.h的private部分（约第122行后）添加：

/* SELECTIVE VERTEX CACHE (SVC) FOR FILO-2 OPTIMIZATION */
std::unordered_set<int> svcNodes;           // 被修改的节点集合
bool useSVC;                                 // 是否启用SVC
int svcExpansionLevel;                       // SVC扩展级别：0=仅修改节点，1=+直接邻居，2=+二级邻居

// SVC管理函数
void addToSVC(int nodeIndex);                // 将节点加入SVC
void addToSVCWithNeighbors(int nodeIndex, int level);  // 将节点及其邻居加入SVC
void clearSVC();                             // 清空SVC
void initializeSVCForLoop();                 // 每轮循环开始时初始化SVC
```

#### 步骤2：修改LocalSearch.cpp - 添加SVC管理函数

在`LocalSearch.cpp`末尾添加（约第1020行后）：

```cpp
void LocalSearch::addToSVC(int nodeIndex)
{
    if (!useSVC) return;
    svcNodes.insert(nodeIndex);
}

void LocalSearch::addToSVCWithNeighbors(int nodeIndex, int level)
{
    if (!useSVC) return;
    
    svcNodes.insert(nodeIndex);
    
    if (level >= 1) {
        // 添加直接邻居（基于correlatedVertices）
        for (int neighbor : params.correlatedVertices[nodeIndex]) {
            svcNodes.insert(neighbor);
        }
    }
    
    if (level >= 2) {
        // 添加二级邻居
        for (int neighbor : params.correlatedVertices[nodeIndex]) {
            for (int neighbor2 : params.correlatedVertices[neighbor]) {
                svcNodes.insert(neighbor2);
            }
        }
    }
}

void LocalSearch::clearSVC()
{
    svcNodes.clear();
}

void LocalSearch::initializeSVCForLoop()
{
    if (!useSVC) return;
    
    // 第一轮循环：SVC包含所有节点（全面搜索）
    if (loopID == 0) {
        for (int i = 1; i <= params.nbClients; i++) {
            svcNodes.insert(i);
        }
    }
    // 后续循环：SVC仅包含上一轮被修改的节点
    // （已在insertNode/swapNode中添加）
}
```

#### 步骤3：修改insertNode和swapNode函数

在`LocalSearch.cpp`的`insertNode`函数后添加SVC更新（约第790行）：

```cpp
void LocalSearch::insertNode(Node * U, Node * V)
{
    // 原有代码保持不变
    Node * myPred = U->prev;
    Node * mySucc = U->next;
    myPred->next = mySucc;
    mySucc->prev = myPred;
    U->prev = V;
    U->next = V->next;
    V->next->prev = U;
    V->next = U;
    
    // ===== 新增：SVC更新 =====
    // 将受影响的节点加入SVC（用于下一轮局部搜索）
    if (useSVC) {
        addToSVCWithNeighbors(U->cour, svcExpansionLevel);
        addToSVCWithNeighbors(V->cour, svcExpansionLevel);
        addToSVCWithNeighbors(myPred->cour, svcExpansionLevel);
        if (!mySucc->isDepot)
            addToSVCWithNeighbors(mySucc->cour, svcExpansionLevel);
    }
}
```

类似地修改`swapNode`函数（约第820行后）：

```cpp
void LocalSearch::swapNode(Node * U, Node * V)
{
    // 原有代码保持不变
    Node * myVPred = V->prev;
    Node * myVSuiv = V->next;
    Node * myUPred = U->prev;
    Node * myUSuiv = U->next;
    Route * myRouteU = U->route;
    Route * myRouteV = V->route;

    myUPred->next = V;
    myUSuiv->prev = V;
    myVPred->next = U;
    myVSuiv->prev = U;

    U->prev = myVPred;
    U->next = myVSuiv;
    V->prev = myUPred;
    V->next = myUSuiv;

    U->route = myRouteV;
    V->route = myRouteU;
    
    // ===== 新增：SVC更新 =====
    if (useSVC) {
        addToSVCWithNeighbors(U->cour, svcExpansionLevel);
        addToSVCWithNeighbors(V->cour, svcExpansionLevel);
        addToSVCWithNeighbors(myUPred->cour, svcExpansionLevel);
        addToSVCWithNeighbors(myUSuiv->cour, svcExpansionLevel);
        addToSVCWithNeighbors(myVPred->cour, svcExpansionLevel);
        addToSVCWithNeighbors(myVSuiv->cour, svcExpansionLevel);
    }
}
```

#### 步骤4：修改主局部搜索循环

修改`LocalSearch::run`函数中的主循环（约第33-103行）：

```cpp
void LocalSearch::run(Individual & indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
    // ... 原有初始化代码 ...
    
    // ===== 新增：初始化SVC配置 =====
    useSVC = true;              // 启用SVC
    svcExpansionLevel = 1;      // 扩展级别：包含直接邻居
    
    loadIndividual(indiv);
    penaltyCapacityLS = penaltyCapacityLS;
    penaltyDurationLS = penaltyDurationLS;

    std::shuffle(orderNodes.begin(), orderNodes.end(), params.ran);
    std::shuffle(orderRoutes.begin(), orderRoutes.end(), params.ran);
    for (int i = 1; i <= params.nbClients; i++)
        if (params.ran() % params.ap.nbGranular == 0)
            std::shuffle(params.correlatedVertices[i].begin(), params.correlatedVertices[i].end(), params.ran);

    searchCompleted = false;
    for (loopID = 0; !searchCompleted; loopID++)
    {
        // ===== 新增：每轮循环开始时初始化SVC =====
        initializeSVCForLoop();
        
        if (loopID > 1)
            searchCompleted = true;

        /* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES */
        for (int posU = 0; posU < params.nbClients; posU++)
        {
            nodeU = &clients[orderNodes[posU]];
            
            // ===== 新增：SVC过滤 =====
            // 第一轮（loopID==0）检查所有节点，后续轮次仅检查SVC中的节点
            if (useSVC && loopID > 0 && svcNodes.find(nodeU->cour) == svcNodes.end())
                continue;  // 跳过不在SVC中的节点
            
            int lastTestRINodeU = nodeU->whenLastTestedRI;
            nodeU->whenLastTestedRI = nbMoves;
            
            for (int posV = 0; posV < (int)params.correlatedVertices[nodeU->cour].size(); posV++)
            {
                nodeV = &clients[params.correlatedVertices[nodeU->cour][posV]];
                
                // 原有的增量更新检查保持不变
                if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)
                {
                    // ... 原有的move1-move9逻辑保持不变 ...
                }
            }
        }
        
        // ===== 新增：清空SVC，准备下一轮 =====
        if (loopID > 0) clearSVC();  // 第一轮后清空，由insertNode/swapNode重新填充
        
        // ... SWAP*部分保持不变（将在构想3中优化）...
    }
    
    exportIndividual(indiv);
}
```

#### 步骤5：在构造函数中初始化SVC参数

修改`LocalSearch::LocalSearch`构造函数（约第1015行）：

```cpp
LocalSearch::LocalSearch(Params & params) : params (params)
{
    // ... 原有初始化代码 ...

    // ===== 新增：SVC初始化 =====
    useSVC = true;
    svcExpansionLevel = 1;  // 默认包含直接邻居
    svcNodes.reserve(params.nbClients);  // 预分配内存
}
```

---

## 构想3：SWAP*顺序搜索与剪枝

### 优化1：SWAP* + SVC集成（最高优先级）

修改`LocalSearch::run`中的SWAP*部分（约第105-124行）：

```cpp
if (params.ap.useSwapStar == 1 && params.areCoordinatesProvided)
{
    /* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
    for (int rU = 0; rU < params.nbVehicles; rU++)
    {
        routeU = &routes[orderRoutes[rU]];

        // ===== 新增：检查路线U是否包含SVC节点 =====
        bool routeUHasSVC = false;
        if (useSVC && loopID > 0) {
            for (Node* node = routeU->depot->next; !node->isDepot; node = node->next) {
                if (svcNodes.find(node->cour) != svcNodes.end()) {
                    routeUHasSVC = true;
                    break;
                }
            }
            if (!routeUHasSVC) continue;  // 路线U不包含SVC节点，跳过
        }

        int lastTestSWAPStarRouteU = routeU->whenLastTestedSWAPStar;
        routeU->whenLastTestedSWAPStar = nbMoves;

        for (int rV = 0; rV < params.nbVehicles; rV++)
        {
            routeV = &routes[orderRoutes[rV]];

            if (routeU->nbCustomers > 0 && routeV->nbCustomers > 0 && routeU->cour < routeV->cour
                && (loopID == 0 || std::max<int>(routeU->whenLastModified, routeV->whenLastModified)
                    > lastTestSWAPStarRouteU))
                if (CircleSector::overlap(routeU->sector, routeV->sector))
                    swapStar();
        }
    }
}
```

### 优化2：SWAP*内部的顺序剪枝

修改`LocalSearch::swapStar`函数（约第599-698行）：

```cpp
bool LocalSearch::swapStar()
{
    SwapStarElement myBestSwapStar;
    preprocessInsertions(routeU, routeV);
    preprocessInsertions(routeV, routeU);

    double deltaPenRouteU = penaltyExcessLoad(routeV->load) - routeU->penalty;
    double deltaPenRouteV = penaltyExcessLoad(routeU->load) - routeV->penalty;

    // ===== 新增：顺序搜索与多层剪枝 =====

    // 情况1：交换nodeU和nodeV（带顺序剪枝）
    for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
    {
        // ===== 层次1剪枝：SVC过滤 =====
        if (useSVC && loopID > 0 && svcNodes.find(nodeU->cour) == svcNodes.end())
            continue;

        // ===== 层次2剪枝：nodeU移除下界 =====
        double lowerBoundU = deltaPenRouteU + nodeU->deltaRemoval;
        if (lowerBoundU >= myBestSwapStar.moveCost)
            continue;  // 即使最优的nodeV也无法改进

        for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
        {
            // ===== 层次3剪枝：nodeU+nodeV移除下界 =====
            double lowerBoundUV = lowerBoundU + deltaPenRouteV + nodeV->deltaRemoval;
            if (lowerBoundUV >= myBestSwapStar.moveCost)
                continue;

            SwapStarElement mySwapStar;
            mySwapStar.U = nodeU;
            mySwapStar.V = nodeV;

            // ===== 层次4剪枝：加上nodeU插入成本 =====
            double extraV = getCheapestInsertSimultRemoval(nodeU, nodeV, mySwapStar.bestPositionU);
            if (lowerBoundUV + extraV >= myBestSwapStar.moveCost)
                continue;

            // ===== 层次5：完整成本计算（仅对通过前4层的候选） =====
            double extraU = getCheapestInsertSimultRemoval(nodeV, nodeU, mySwapStar.bestPositionV);

            mySwapStar.moveCost = lowerBoundUV + extraV + extraU
                + penaltyExcessDuration(routeU->duration + nodeU->deltaRemoval + extraU
                    + params.cli[nodeV->cour].serviceDuration - params.cli[nodeU->cour].serviceDuration)
                + penaltyExcessDuration(routeV->duration + nodeV->deltaRemoval + extraV
                    - params.cli[nodeV->cour].serviceDuration + params.cli[nodeU->cour].serviceDuration);

            if (mySwapStar.moveCost < myBestSwapStar.moveCost)
                myBestSwapStar = mySwapStar;
        }
    }

    // 情况2和3：RELOCATE（保持原有逻辑，但添加SVC过滤）
    for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
    {
        // ===== 新增：SVC过滤 =====
        if (useSVC && loopID > 0 && svcNodes.find(nodeU->cour) == svcNodes.end())
            continue;

        // ... 原有RELOCATE逻辑 ...
    }

    // ... 其余代码保持不变 ...
}
```

### 优化3：启发式排序（可选，进一步优化）

在`LocalSearch.h`中添加辅助函数声明：

```cpp
// 在LocalSearch类的private部分添加
std::vector<Node*> getSortedNodesByRemovalCost(Route* route, bool ascending = true);
```

在`LocalSearch.cpp`中实现：

```cpp
std::vector<Node*> LocalSearch::getSortedNodesByRemovalCost(Route* route, bool ascending)
{
    std::vector<Node*> nodes;
    for (Node* node = route->depot->next; !node->isDepot; node = node->next) {
        // 仅包含SVC中的节点（如果启用SVC）
        if (!useSVC || loopID == 0 || svcNodes.find(node->cour) != svcNodes.end()) {
            nodes.push_back(node);
        }
    }

    // 按deltaRemoval排序（移除成本节省大的优先）
    std::sort(nodes.begin(), nodes.end(),
        [ascending](Node* a, Node* b) {
            return ascending ? (a->deltaRemoval < b->deltaRemoval)
                             : (a->deltaRemoval > b->deltaRemoval);
        });

    return nodes;
}
```

修改`swapStar`使用排序后的节点：

```cpp
bool LocalSearch::swapStar()
{
    SwapStarElement myBestSwapStar;
    preprocessInsertions(routeU, routeV);
    preprocessInsertions(routeV, routeU);

    double deltaPenRouteU = penaltyExcessLoad(routeV->load) - routeU->penalty;
    double deltaPenRouteV = penaltyExcessLoad(routeU->load) - routeV->penalty;

    // ===== 新增：获取排序后的节点列表 =====
    std::vector<Node*> sortedNodesU = getSortedNodesByRemovalCost(routeU, true);  // 升序
    std::vector<Node*> sortedNodesV = getSortedNodesByRemovalCost(routeV, true);

    // 情况1：使用排序后的列表进行搜索
    for (Node* nodeU : sortedNodesU)
    {
        double lowerBoundU = deltaPenRouteU + nodeU->deltaRemoval;
        if (lowerBoundU >= myBestSwapStar.moveCost)
            break;  // 由于已排序，后续节点更不可能改进

        for (Node* nodeV : sortedNodesV)
        {
            double lowerBoundUV = lowerBoundU + deltaPenRouteV + nodeV->deltaRemoval;
            if (lowerBoundUV >= myBestSwapStar.moveCost)
                break;  // 由于已排序，后续nodeV更不可能改进

            // ... 其余逻辑与优化2相同 ...
        }
    }

    // ... 其余代码保持不变 ...
}
```

---

## 性能对比基准

### 添加性能统计

在`LocalSearch.h`中添加统计变量：

```cpp
// 在LocalSearch类的private部分添加
struct PerformanceStats {
    long long totalMoveEvaluations;      // 总移动评估次数
    long long svcFilteredMoves;          // 被SVC过滤的移动数
    long long swapStarEvaluations;       // SWAP*评估次数
    long long swapStarPruned;            // SWAP*剪枝次数
    double timeInLocalSearch;            // 局部搜索总时间
    double timeInSwapStar;               // SWAP*总时间

    void reset() {
        totalMoveEvaluations = 0;
        svcFilteredMoves = 0;
        swapStarEvaluations = 0;
        swapStarPruned = 0;
        timeInLocalSearch = 0.0;
        timeInSwapStar = 0.0;
    }

    void print() const {
        std::cout << "=== Local Search Performance Stats ===" << std::endl;
        std::cout << "Total move evaluations: " << totalMoveEvaluations << std::endl;
        std::cout << "SVC filtered moves: " << svcFilteredMoves
                  << " (" << (100.0 * svcFilteredMoves / totalMoveEvaluations) << "%)" << std::endl;
        std::cout << "SWAP* evaluations: " << swapStarEvaluations << std::endl;
        std::cout << "SWAP* pruned: " << swapStarPruned
                  << " (" << (100.0 * swapStarPruned / swapStarEvaluations) << "%)" << std::endl;
        std::cout << "Time in LS: " << timeInLocalSearch << "s" << std::endl;
        std::cout << "Time in SWAP*: " << timeInSwapStar << "s"
                  << " (" << (100.0 * timeInSwapStar / timeInLocalSearch) << "%)" << std::endl;
    }
};

PerformanceStats perfStats;
```

在关键位置添加统计代码：

```cpp
// 在LocalSearch::run开始处
void LocalSearch::run(Individual & indiv, double penaltyCapacityLS, double penaltyDurationLS)
{
    auto startTime = std::chrono::high_resolution_clock::now();
    perfStats.reset();

    // ... 原有代码 ...

    // 在主循环中
    for (int posU = 0; posU < params.nbClients; posU++)
    {
        nodeU = &clients[orderNodes[posU]];

        if (useSVC && loopID > 0 && svcNodes.find(nodeU->cour) == svcNodes.end()) {
            perfStats.svcFilteredMoves += params.correlatedVertices[nodeU->cour].size();
            continue;
        }

        for (int posV = 0; posV < (int)params.correlatedVertices[nodeU->cour].size(); posV++)
        {
            perfStats.totalMoveEvaluations++;
            // ... 原有代码 ...
        }
    }

    // 在函数结束处
    auto endTime = std::chrono::high_resolution_clock::now();
    perfStats.timeInLocalSearch = std::chrono::duration<double>(endTime - startTime).count();

    if (params.verbose) {
        perfStats.print();
    }
}
```

---

## 使用示例

### 编译选项

在编译时可以通过宏定义控制优化：

```bash
# 启用所有FILO-2优化
g++ -DUSE_SVC -DUSE_SWAP_STAR_PRUNING -O3 ...

# 仅启用SVC
g++ -DUSE_SVC -O3 ...

# 禁用所有优化（对比基准）
g++ -O3 ...
```

### 运行时配置

可以通过`AlgorithmParameters`添加配置选项：

```cpp
// 在AlgorithmParameters.h中添加
struct AlgorithmParameters {
    // ... 现有字段 ...

    int useSVC;              // 是否启用SVC：0=禁用，1=启用
    int svcExpansionLevel;   // SVC扩展级别：0=仅修改节点，1=+邻居，2=+二级邻居
    int useSwapStarPruning;  // 是否启用SWAP*顺序剪枝
};
```

### 性能测试脚本

```python
# test_filo2_performance.py
import subprocess
import time

instances = ["X-n101-k25.vrp", "X-n200-k36.vrp", "X-n500-k60.vrp"]
configs = [
    {"name": "Baseline", "svc": 0, "pruning": 0},
    {"name": "SVC-Only", "svc": 1, "pruning": 0},
    {"name": "SVC+Pruning", "svc": 1, "pruning": 1},
]

for instance in instances:
    print(f"\n=== Testing {instance} ===")
    for config in configs:
        cmd = f"./hgs --instance {instance} --useSVC {config['svc']} --useSwapStarPruning {config['pruning']}"
        start = time.time()
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        elapsed = time.time() - start

        print(f"{config['name']}: {elapsed:.2f}s")
        # 解析输出中的解质量和统计信息
```

---

## 预期性能提升总结

| 优化组合 | 实施难度 | 代码量 | 预期加速 | 推荐优先级 |
|---------|---------|--------|---------|-----------|
| 仅SVC（构想1） | ⭐☆☆☆☆ | 200行 | 1.8-2.5x | 🔥🔥🔥🔥🔥 |
| SVC + SWAP*过滤（构想3-优化1） | ⭐☆☆☆☆ | 50行 | 额外1.2-1.5x | 🔥🔥🔥🔥🔥 |
| + 顺序剪枝（构想3-优化2） | ⭐⭐☆☆☆ | 150行 | 额外1.3-1.8x | 🔥🔥🔥🔥☆ |
| + 启发式排序（构想3-优化3） | ⭐⭐⭐☆☆ | 100行 | 额外1.2-1.4x | 🔥🔥🔥☆☆ |
| **总计（全部实施）** | ⭐⭐☆☆☆ | **500行** | **2.5-4x** | - |

**建议实施路径**：
1. 第1天：实施SVC（构想1）
2. 第2天：实施SWAP*+SVC（构想3-优化1）
3. 第3-5天：实施顺序剪枝（构想3-优化2）
4. 可选：实施启发式排序（构想3-优化3）

这样可以在2-3天内获得**2-3倍**的整体加速，投入产出比极高！

