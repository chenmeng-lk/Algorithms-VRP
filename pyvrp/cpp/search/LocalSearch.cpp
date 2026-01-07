// 局部搜索算法实现，用于改进车辆路径问题（VRP）的解。通过节点和路径操作符进行邻域搜索，结合扰动和强化策略寻找更优解。

#include "LocalSearch.h"
#include "DynamicBitset.h"
#include "Measure.h"
#include "Trip.h"
#include "primitives.h"

#include <algorithm>
#include <cassert>
#include <numeric>

using pyvrp::Solution;
using pyvrp::search::LocalSearch;
using pyvrp::search::NodeOperator;
using pyvrp::search::RouteOperator;
using pyvrp::search::SearchSpace;

// 主搜索算子：对给定解进行完整的局部搜索，包括扰动、节点搜索和路径强化
pyvrp::Solution LocalSearch::operator()(pyvrp::Solution const &solution,
                                        CostEvaluator const &costEvaluator)
{
    loadSolution(solution);
    perturbationManager_.perturb(solution_, searchSpace_, costEvaluator);  // 先对解进行扰动(扰动的客户及直接邻居都标记为有希望的)

    while (true)
    {
        search(costEvaluator);  // 执行节点级邻域搜索
        auto const numUpdates = numUpdates_;  // 记录节点搜索后的更新次数

        intensify(costEvaluator);  // 执行路径级强化搜索
        if (numUpdates_ == numUpdates)
            // 如果强化搜索没有带来新的更新，说明解已达到局部最优，停止搜索
            break;
    }

    return solution_.unload();  // 返回优化后的解
}

// 仅执行节点搜索的接口
pyvrp::Solution LocalSearch::search(pyvrp::Solution const &solution,
                                    CostEvaluator const &costEvaluator)
{
    loadSolution(solution);
    search(costEvaluator);
    return solution_.unload();
}

// 仅执行路径强化搜索的接口
pyvrp::Solution LocalSearch::intensify(pyvrp::Solution const &solution,
                                       CostEvaluator const &costEvaluator)
{
    loadSolution(solution);
    intensify(costEvaluator);
    return solution_.unload();
}

// 节点搜索核心函数：遍历所有客户端节点，应用节点操作符进行邻域搜索
// TODO: 搜索分两大类，一类是节点操作，一类是路径操作，节点操作包括交换、
//     重定位、移除、插入等，路径操作包括合并、分裂、移除、插入等。
// 结点搜索：外部迭代直到没有改进为止，内部遍历所有有希望的客户端节点，遍历其邻居得到
//     一对对搜索位置（都要在解中，所在路径自上次测试后有更新），依次尝试所有节点操作符，
//     选择第一个改进移动并应用(仅执行一个改进移动)，执行就看下一个邻居
// 路径强化搜索:外部迭代直到没有改进为止，内部遍历每一对路径（都非空，且至少一个自上次测
//     试后有更新），遍历所有路径操作符，选择第一个改进移动并应用
void LocalSearch::search(CostEvaluator const &costEvaluator)
{
    if (nodeOps.empty())
        return;

    markRequiredMissingAsPromising();  // 标记必须但缺失的客户为“有希望的”

    searchCompleted_ = false;
    for (int step = 0; !searchCompleted_; ++step)
    {
        searchCompleted_ = true;

        // 按顺序遍历所有客户端节点
        for (auto const uClient : searchSpace_.clientOrder())
        {
            if (!searchSpace_.isPromising(uClient))
                continue;  // 跳过非“有希望的”节点

            auto *U = &solution_.nodes[uClient];

            auto const lastTested = lastTestedNodes[U->client()];//记录U上次测试时的更新次数
            lastTestedNodes[U->client()] = numUpdates_;//记录U当前的更新次数

            // 首先尝试插入或移除可选客户端（如奖收集问题中的非必须节点）
            applyOptionalClientMoves(U, costEvaluator);

            // 如果该客户端属于某个组，执行组内移动
            applyGroupMoves(U, costEvaluator);

            if (!U->route())  // 如果U不在任何路径中，则跳过后续操作
                continue;

            // 如果U的相邻节点是重载仓库，尝试移除该仓库
            applyDepotRemovalMove(p(U), costEvaluator);
            applyDepotRemovalMove(n(U), costEvaluator);

            // 对U的每个邻居V，执行节点操作符（如交换、重定位等）
            for (auto const vClient : searchSpace_.neighboursOf(U->client()))
            {
                auto *V = &solution_.nodes[vClient];

                if (!V->route())//V不在解中
                    continue;

                // 检查U或V所在路径是否自上次测试后有更新，若有则尝试移动
                if (lastUpdated[U->route()->idx()] > lastTested
                    || lastUpdated[V->route()->idx()] > lastTested)
                {
                    if (applyNodeOps(U, V, costEvaluator))//执行改进操作后进入下一次循环
                        continue;

                    // 如果V是路径起点，尝试U与路径起点前驱（虚拟仓库）的移动
                    if (p(V)->isStartDepot()
                        && applyNodeOps(U, p(V), costEvaluator))
                        continue;
                }
            }

            // 在第一步之后，尝试将U移入空路径（避免过早使用过多车辆——step的作用）
            if (step > 0)
                applyEmptyRouteMoves(U, costEvaluator);
        }
    }
}

// 路径强化搜索：遍历所有路径对，应用路径操作符（如路径交换、交叉等）
void LocalSearch::intensify(CostEvaluator const &costEvaluator)
{
    if (routeOps.empty())
        return;

    searchCompleted_ = false;
    while (!searchCompleted_)
    {
        searchCompleted_ = true;

        // 按顺序遍历所有路径
        for (auto const rU : searchSpace_.routeOrder())
        {
            auto *U = &solution_.routes[rU];
            assert(U->idx() == rU);

            if (U->empty())//路线不能为空
                continue;

            auto const lastTested = lastTestedRoutes[U->idx()];//记录U上次测试时的更新次数
            lastTestedRoutes[U->idx()] = numUpdates_;//记录U当前的更新次数

            // 对每一对路径（U, V）应用路径操作符
            for (size_t rV = U->idx() + 1; rV != solution_.routes.size(); ++rV)
            {
                auto *V = &solution_.routes[rV];
                assert(V->idx() == rV);

                if (V->empty())
                    continue;

                // 如果U或V路径自上次测试后有更新，则尝试路径操作，确保搜索只在必要时进行
                //路径未变化时，邻域和成本评估结果通常不变，无需重新评估操作符，节省计算资源
                if (lastUpdated[U->idx()] > lastTested
                    || lastUpdated[V->idx()] > lastTested)
                    applyRouteOps(U, V, costEvaluator);
            }
        }
    }
}

// 随机打乱内部组件顺序，增加搜索多样性
void LocalSearch::shuffle(RandomNumberGenerator &rng)
{
    perturbationManager_.shuffle(rng);
    searchSpace_.shuffle(rng);

    rng.shuffle(nodeOps.begin(), nodeOps.end());
    rng.shuffle(routeOps.begin(), routeOps.end());
}

// 应用节点操作符：依次尝试所有节点操作符，选择第一个改进移动并应用(仅执行一个移动)，应用成功返回true
bool LocalSearch::applyNodeOps(Route::Node *U,
                               Route::Node *V,
                               CostEvaluator const &costEvaluator)
{
    for (auto *nodeOp : nodeOps)
    {
        auto const deltaCost = nodeOp->evaluate(U, V, costEvaluator);//虚函数，具体操作取决于采用的算子
        if (deltaCost < 0)  // 如果移动能降低成本
        {
            auto *rU = U->route();  // 保存移动前的路径指针
            auto *rV = V->route();

            [[maybe_unused]] auto const costBefore
                = costEvaluator.penalisedCost(*rU)
                  + Cost(rU != rV) * costEvaluator.penalisedCost(*rV);

            searchSpace_.markPromising(U);  // 标记相关节点为“有希望的”
            searchSpace_.markPromising(V);

            nodeOp->apply(U, V);  // 执行移动，虚函数，具体操作取决于采用的算子
            update(rU, rV);  // 更新路径状态和记录

            [[maybe_unused]] auto const costAfter
                = costEvaluator.penalisedCost(*rU)
                  + Cost(rU != rV) * costEvaluator.penalisedCost(*rV);

            // 断言：移动后的代价应等于移动前代价加上评估的增量代价
            assert(costAfter == costBefore + deltaCost);

            return true;  // 移动已应用
        }
    }

    return false;  // 没有改进移动
}

// 应用路径操作符：依次尝试所有路径操作符，选择第一个改进移动并应用
bool LocalSearch::applyRouteOps(Route *U,
                                Route *V,
                                CostEvaluator const &costEvaluator)
{
    for (auto *routeOp : routeOps)
    {
        auto const deltaCost = routeOp->evaluate(U, V, costEvaluator);//评估移动的模版函数，具体操作取决于采用的算子
        if (deltaCost < 0)  // 如果移动能降低代价
        {
            [[maybe_unused]] auto const costBefore
                = costEvaluator.penalisedCost(*U)
                  + Cost(U != V) * costEvaluator.penalisedCost(*V);

            routeOp->apply(U, V);  // 执行移动，每个操作符执行各自的移动逻辑
            update(U, V);  // 更新路径状态和记录

            [[maybe_unused]] auto const costAfter
                = costEvaluator.penalisedCost(*U)
                  + Cost(U != V) * costEvaluator.penalisedCost(*V);

            // 断言：移动后的代价应等于移动前代价加上评估的增量代价
            assert(costAfter == costBefore + deltaCost);

            return true;  // 移动已应用
        }
    }

    return false;  // 没有改进移动
}
//VRP with Intermediate Replenishment Facilities（带中间补给设施的车辆路径问题）
// 尝试移除重载仓库节点：如果移除不会增加代价（甚至可能减少），则移除(不必要的重载仓库访问)
void LocalSearch::applyDepotRemovalMove(Route::Node *U,
                                        CostEvaluator const &costEvaluator)
{
    if (!U->isReloadDepot())  // 如果不是重载仓库，直接返回
        return;

    // 如果移除代价小于等于0（即不劣化），则移除该仓库节点
    if (removeCost(U, data, costEvaluator) <= 0)
    {
        searchSpace_.markPromising(U);  // 标记U及其相邻节点为“有希望的”
        auto *route = U->route();
        route->remove(U->idx());
        update(route, route);
    }
}

// 尝试将节点U移入空路径以利用不同的车辆类型：按车辆类型顺序查找空路径并尝试插入
// Multi-Vehicle Type VRP（多车型VRP） 或 Heterogeneous Fleet VRP（异构车队VRP）
// 的一个特定方面，即考虑不同车辆类型的空路径利用
void LocalSearch::applyEmptyRouteMoves(Route::Node *U,
                                       CostEvaluator const &costEvaluator)
{
    assert(U->route());

    // 按随机顺序遍历车辆类型，避免总是优先选择固定成本低的车辆
    for (auto const &[vehType, offset] : searchSpace_.vehTypeOrder())
    {
        auto const begin = solution_.routes.begin() + offset;
        auto const end = begin + data.vehicleType(vehType).numAvailable;
        auto const pred = [](auto const &route) { return route.empty(); };
        auto empty = std::find_if(begin, end, pred);  // 查找该类型下的空路径

        if (empty != end && applyNodeOps(U, (*empty)[0], costEvaluator))
            break;  // 如果成功移动，则停止尝试
    }
}
//VRP with Optional Customers（带可选客户的车辆路径问题） 的一个变体
//通常也称为 Prize-Collecting VRP（带奖励收集的VRP） 
// 处理可选客户端节点：尝试移除或重新插入非必须节点
void LocalSearch::applyOptionalClientMoves(Route::Node *U,
                                           CostEvaluator const &costEvaluator)
{
    ProblemData::Client const &uData = data.location(U->client());

    if (uData.required && !U->route())  // 如果是必须但未在解中的节点，强制插入
    {
        solution_.insert(U, searchSpace_, costEvaluator, true);
        update(U->route(), U->route());
        searchSpace_.markPromising(U);
    }

    // 必须节点(已在解中)或组节点不在此处理（组有专门的操作）
    if (uData.required || uData.group)
        return;
    //在解中且非必须节点
    if (removeCost(U, data, costEvaluator) < 0)  // 如果移除此可选客户能改进，则移除
    {
        searchSpace_.markPromising(U);
        auto *route = U->route();
        route->remove(U->idx());
        update(route, route);
    }
    //在解中且非必须节点，移除又不能改进
    if (U->route())
        return;  // 如果节点仍在路径中，则不需要重新插入

    // 移除后，尝试将U重新插入到邻居节点之后
    for (auto const vClient : searchSpace_.neighboursOf(U->client()))
    {
        auto *V = &solution_.nodes[vClient];
        auto *route = V->route();

        if (!route)//邻居节点不在路径中
            continue;

        if (insertCost(U, V, data, costEvaluator) < 0)  // 如果插入能改进
        {
            route->insert(V->idx() + 1, U);
            update(route, route);
            searchSpace_.markPromising(U);
            return;
        }

        // 如果V也是可选客户，尝试用U替换V
        ProblemData::Client const &vData = data.location(V->client());
        if (!vData.required && inplaceCost(U, V, data, costEvaluator) < 0)
        {
            searchSpace_.markPromising(V);
            auto const idx = V->idx();
            route->remove(idx);
            route->insert(idx, U);
            update(route, route);
            searchSpace_.markPromising(U);
            return;
        }
    }
}
//处理 VRP with Mutual Exclusion Constraints（带互斥约束的车辆路径问题）
// 处理组节点U：对于互斥组，确保组内只有一个节点在解中，并尝试替换
void LocalSearch::applyGroupMoves(Route::Node *U,
                                  CostEvaluator const &costEvaluator)
{
    ProblemData::Client const &uData = data.location(U->client());

    if (!uData.group)//U不属于任何组
        return;

    auto const &group = data.group(*uData.group);
    assert(group.mutuallyExclusive);  // 确保组是互斥的

    // 筛选出已经在解中的同组节点
    std::vector<size_t> inSol;
    auto const pred
        = [&](auto client) { return solution_.nodes[client].route(); };
    std::copy_if(group.begin(), group.end(), std::back_inserter(inSol), pred);

    if (inSol.empty())  // 如果组内没有节点在解中，直接插入U
    {
        auto const required = group.required;
        if (solution_.insert(U, searchSpace_, costEvaluator, required))//插入U到解中
        {
            update(U->route(), U->route());
            searchSpace_.markPromising(U);
        }

        return;
    }

    // 组中还有其他节点在解中，计算移除组内每个节点的代价
    std::vector<Cost> costs;
    for (auto const client : inSol)
    {
        auto cost = removeCost(&solution_.nodes[client], data, costEvaluator);
        costs.push_back(cost);
    }

    // 按移除代价升序排序（代价越小，移除越有利）
    std::vector<size_t> range(inSol.size());
    std::iota(range.begin(), range.end(), 0);
    std::sort(range.begin(),
              range.end(),
              [&costs](auto idx1, auto idx2)
              { return costs[idx1] < costs[idx2]; });

    // 移除代价最小的前n-1个节点，保留代价最大的一个节点V
    for (auto idx = range.begin(); idx != range.end() - 1; ++idx)
    {
        auto const client = inSol[*idx];
        auto const &node = solution_.nodes[client];
        auto *route = node.route();

        searchSpace_.markPromising(&node);
        route->remove(node.idx());
        update(route, route);
    }

    // 尝试用U替换剩下的节点V。如果替换能降低成本，则执行替换
    auto *V = &solution_.nodes[inSol[range.back()]];
    if (U != V && inplaceCost(U, V, data, costEvaluator) < 0)
    {
        auto *route = V->route();
        auto const idx = V->idx();
        route->remove(idx);
        route->insert(idx, U);
        update(route, route);
        searchSpace_.markPromising(U);
    }
}

// 标记必须但缺失的客户为“有希望的”，以便后续插入操作
void LocalSearch::markRequiredMissingAsPromising()
{
    for (auto client = data.numDepots(); client != data.numLocations();
         ++client)//遍历客户点
    {
        if (solution_.nodes[client].route())  // 如果已在解中，跳过
            continue;

        ProblemData::Client const &clientData = data.location(client);
        if (clientData.required)  // 必须节点且不在解中
        {
            searchSpace_.markPromising(client);
            continue;
        }

        if (clientData.group)  // 如果是必须组的第一个节点，也标记为“有希望的”
        {
            auto const &group = data.group(clientData.group.value());
            if (group.required && group.clients().front() == client)
            {
                searchSpace_.markPromising(client);
                continue;
            }
        }
    }
}

// 更新路径状态：记录更新次数，标记搜索未完成，更新路径信息并同步操作符缓存
void LocalSearch::update(Route *U, Route *V)
{
    numUpdates_++;
    searchCompleted_ = false;

    U->update();  // 更新路径U的内部状态（如负载、距离等）
    lastUpdated[U->idx()] = numUpdates_;  // 记录路径U的最后更新次数

    for (auto *op : routeOps)  // 通知路径操作符更新缓存
        op->update(U);

    if (U != V)  // 如果涉及两条路径，同样更新路径V
    {
        V->update();
        lastUpdated[V->idx()] = numUpdates_;

        for (auto *op : routeOps)
            op->update(V);
    }
}

// 加载解到内部数据结构，初始化状态和操作符
void LocalSearch::loadSolution(pyvrp::Solution const &solution)
{
    std::fill(lastTestedNodes.begin(), lastTestedNodes.end(), -1);
    std::fill(lastTestedRoutes.begin(), lastTestedRoutes.end(), -1);
    std::fill(lastUpdated.begin(), lastUpdated.end(), 0);
    searchSpace_.markAllPromising();  // 初始时所有节点都视为“有希望的”
    numUpdates_ = 0;

    solution_.load(solution);  // 将外部解加载到内部表示

    for (auto *nodeOp : nodeOps)  // 初始化节点操作符
        nodeOp->init(solution);

    for (auto *routeOp : routeOps)  // 初始化路径操作符
        routeOp->init(solution);
}

// 添加节点操作符，在solve.py中被调用
void LocalSearch::addNodeOperator(NodeOperator &op)
{
    nodeOps.emplace_back(&op);
}

// 添加路径操作符，在solve.py中被调用
void LocalSearch::addRouteOperator(RouteOperator &op)
{
    routeOps.emplace_back(&op);
}

// 返回节点操作符列表
std::vector<NodeOperator *> const &LocalSearch::nodeOperators() const
{
    return nodeOps;
}

// 返回路径操作符列表
std::vector<RouteOperator *> const &LocalSearch::routeOperators() const
{
    return routeOps;
}

// 设置邻居结构
void LocalSearch::setNeighbours(SearchSpace::Neighbours neighbours)
{
    searchSpace_.setNeighbours(neighbours);
}

// 返回当前邻居结构
SearchSpace::Neighbours const &LocalSearch::neighbours() const
{
    return searchSpace_.neighbours();
}

// 收集搜索统计信息：总移动评估次数、改进移动次数、总更新次数
LocalSearch::Statistics LocalSearch::statistics() const
{
    size_t numMoves = 0;
    size_t numImproving = 0;

    auto const count = [&](auto const *op)
    {
        auto const &stats = op->statistics();
        numMoves += stats.numEvaluations;  // 累计评估次数
        numImproving += stats.numApplications;  // 累计应用次数
    };

    std::for_each(nodeOps.begin(), nodeOps.end(), count);
    std::for_each(routeOps.begin(), routeOps.end(), count);

    assert(numImproving <= numUpdates_);  // 改进移动数不应超过总更新次数
    return {numMoves, numImproving, numUpdates_};
}

// 构造函数：初始化数据结构、搜索空间、扰动管理器及各种记录数组
LocalSearch::LocalSearch(ProblemData const &data,
                         SearchSpace::Neighbours neighbours,
                         PerturbationManager &perturbationManager)
    : data(data),
      solution_(data),
      searchSpace_(data, neighbours),
      perturbationManager_(perturbationManager),
      lastTestedNodes(data.numLocations()),
      lastTestedRoutes(data.numVehicles()),
      lastUpdated(data.numVehicles())
{
}
