#include "SwapStar.h"

#include <cassert>
#include <limits>

using pyvrp::Cost;
using pyvrp::Load;
using pyvrp::search::Route;
using pyvrp::search::SwapStar;

void SwapStar::updateRemovalCosts(Route *R, CostEvaluator const &costEvaluator)
{
    // 遍历路线R中的所有节点
    for (auto const *U : *R)
    {
        auto const idx = U->idx();  // 获取节点在路线中的索引
        // 创建一个移除节点U的提案：连接U的前驱节点和后继节点
        auto const proposal
            = Route::Proposal(R->before(idx - 1), R->after(idx + 1));

        Cost deltaCost = 0;
        // 计算移除节点U的成本变化，<true, true>表示考虑距离和负载
        costEvaluator.deltaCost<true, true>(deltaCost, proposal);
        // 将计算出的移除成本存入矩阵中，行索引为路线ID，列索引为客户ID
        removalCosts(R->idx(), U->client()) = deltaCost;
    }

    isCached(R->idx(), 0) = true;  // removal costs are now updated // 移除成本已更新，标记路线级别的缓存为有效
    for (size_t idx = data.numDepots(); idx != data.numLocations(); ++idx)
        isCached(R->idx(), idx) = false;  // but insert costs not yet // 但插入成本尚未更新，将每个客户的插入缓存标记为无效
}

void SwapStar::updateInsertPoints(Route *R,
                                  Route::Node *U,
                                  CostEvaluator const &costEvaluator)
{
    // 获取路线R中客户U的三个最佳插入点缓存
    auto &insertPoints = insertCache(R->idx(), U->client());
    // 用最大值和空指针初始化三个插入点，表示尚未找到有效插入点
    insertPoints.fill({std::numeric_limits<Cost>::max(), nullptr});

    // 遍历路线R中除最后一个位置外的所有可能插入位置
    for (size_t idx = 0; idx != R->size() - 1; ++idx)
    {
        // 创建在位置idx处插入节点U的提案
        auto const proposal = Route::Proposal(
            R->before(idx), U->route()->at(U->idx()), R->after(idx + 1));

        Cost deltaCost = 0;
        // 计算插入成本变化
        costEvaluator.deltaCost<true, true>(deltaCost, proposal);

        auto *V = (*R)[idx];  // 获取当前插入位置的前驱节点V
        // 维护三个最佳插入点的排序：插入点按成本升序排列
        if (deltaCost < insertPoints[0].first)  // 如果当前插入成本小于最小值
        {
            insertPoints[2] = insertPoints[1];
            insertPoints[1] = insertPoints[0];
            insertPoints[0] = {deltaCost, V};  // 更新最佳插入点
        }
        else if (deltaCost < insertPoints[1].first)  // 如果小于次小值
        {
            insertPoints[2] = insertPoints[1];
            insertPoints[1] = {deltaCost, V};
        }
        else if (deltaCost < insertPoints[2].first)  // 如果小于第三小值
            insertPoints[2] = {deltaCost, V};
    }

    // 标记该路线和客户的插入点缓存为已更新
    isCached(R->idx(), U->client()) = true;
}

Cost SwapStar::deltaLoadCost(Route::Node *U,
                             Route::Node *V,
                             CostEvaluator const &costEvaluator) const
{
    auto const *uRoute = U->route();  // 获取U所在路线
    auto const *vRoute = V->route();  // 获取V所在路线

    ProblemData::Client const &uClient = data.location(U->client());  // 获取U的客户数据
    ProblemData::Client const &vClient = data.location(V->client());  // 获取V的客户数据

    auto const &uLoad = uRoute->load();  // U路线的当前负载
    auto const &uCap = uRoute->capacity();  // U路线的容量

    auto const &vLoad = vRoute->load();  // V路线的当前负载
    auto const &vCap = vRoute->capacity();  // V路线的容量

    // Separating removal and insertion means that the effects on load are not
    // counted correctly: during insert, U is still in the route, and now V is
    // added as well. The following addresses this issue with an approximation,
    // which is inexact when there are both pickups and deliveries in the data.
    // So it's pretty rough but fast and seems to mostly work well enough.
    // 分离移除和插入意味着负载的影响没有被正确计算：在插入时，U仍在路线中，并且V也被添加进来。
    // 以下代码通过近似方法解决这个问题，当数据中同时存在取货和送货时，这种方法不精确。
    // 因此它相当粗略但快速，并且似乎大多情况下工作得足够好。
    Cost cost = 0;
    // 遍历所有负载维度（如重量、体积等）
    for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
    {
        // 计算U和V在最大交付量和取货量上的差异，即"净需求差异"，近似计算
        auto const delta
            = std::max(uClient.delivery[dim], uClient.pickup[dim])
              - std::max(vClient.delivery[dim], vClient.pickup[dim]);

        // 计算U路线负载变化后的惩罚成本变化
        cost += costEvaluator.loadPenalty(uLoad[dim] - delta, uCap[dim], dim);//计算交换后的负载惩罚，-delta即-U+V
        cost -= costEvaluator.loadPenalty(uLoad[dim], uCap[dim], dim);//减去交换前的负载惩罚，得到变化值

        // 计算V路线负载变化后的惩罚成本变化
        cost += costEvaluator.loadPenalty(vLoad[dim] + delta, vCap[dim], dim);
        cost -= costEvaluator.loadPenalty(vLoad[dim], vCap[dim], dim);
    }

    return cost; // 返回负载变化导致的成本变化
}

SwapStar::InsertPoint SwapStar::bestInsertPoint(
    Route::Node *U, Route::Node *V, CostEvaluator const &costEvaluator)
{
    auto *route = V->route();  // 获取V所在路线

    // 如果该路线和U客户的插入点缓存未更新，则先更新
    if (!isCached(route->idx(), U->client()))
        updateInsertPoints(route, U, costEvaluator); // 如果缓存未命中，则更新插入点信息

    // 遍历缓存中的三个最佳插入点
    for (auto [cost, where] : insertCache(route->idx(), U->client()))
        // 检查插入点是否有效：插入点存在、不是V本身、不是V的前驱节点，并且V与插入点在同一个行程中
        if (where && where != V && n(where) != V && V->trip() == where->trip())
            // Only if V is not adjacent. We also require that V is in the same
            // trip as the node we plan to remove, because we cannot currently
            // evaluate segments with intermediate reloads in them.
            // 只有当V不相邻时才考虑。我们还要求V与计划移除的节点在同一行程中，
            // 因为目前我们无法评估包含中间重新装载的路径段。
            return std::make_pair(cost, where);  // 返回找到的插入点

    // As a fallback option, we consider inserting in the place of V.
    // 作为备选方案，我们考虑在V的位置插入。
    Cost deltaCost = 0;
    // 计算将U插入到V位置的精确成本变化
    costEvaluator.deltaCost<true, true>(
        deltaCost,
        Route::Proposal(route->before(V->idx() - 1),
                        U->route()->at(U->idx()),
                        route->after(V->idx() + 1)));

    return std::make_pair(deltaCost, p(V)); // 返回在V的位置插入的成本和插入点（V的前驱节点）
}

Cost SwapStar::evaluateMove(Route::Node const *U,
                            Route::Node const *V,
                            Route::Node const *remove,
                            CostEvaluator const &costEvaluator) const
{
    assert(V->route() == remove->route()); // 确保V和remove在同一路线
    assert(V != remove); // 确保V和remove不是同一个节点
    assert(!remove->isDepot()); // 确保remove不是仓库节点

    auto const *route = V->route();  // 获取路线

    Cost deltaCost = 0;

    // 根据V和remove的相对位置，计算插入U并移除remove的成本变化
    if (V->idx() + 1 == remove->idx())  // then we insert U in place of remove
        // 如果V紧接着remove，则我们将U插入到remove的位置替换
        costEvaluator.deltaCost<true>(
            deltaCost,
            Route::Proposal(route->before(V->idx()),
                            U->route()->at(U->idx()),
                            route->after(V->idx() + 2)));
    else if (V->idx() < remove->idx())  // V在remove之前
        costEvaluator.deltaCost<true>(
            deltaCost,
            Route::Proposal(route->before(V->idx()),
                            U->route()->at(U->idx()),
                            route->between(V->idx() + 1, remove->idx() - 1),
                            route->after(remove->idx() + 1)));
    else if (V->idx() > remove->idx())  // V在remove之后
        costEvaluator.deltaCost<true>(
            deltaCost,
            Route::Proposal(route->before(remove->idx() - 1),
                            route->between(remove->idx() + 1, V->idx()),
                            U->route()->at(U->idx()),
                            route->after(V->idx() + 1)));

    return deltaCost; // 返回移动操作的成本变化
}

void SwapStar::init(Solution const &solution)
{
    RouteOperator::init(solution); // 调用基类的初始化函数（重置统计信息）
    // 将所有路线的移除成本缓存标记为未更新，因为解已变化
    for (size_t row = 0; row != isCached.numRows(); ++row)
        isCached(row, 0) = false; // 将所有路线的移除成本缓存标记为未更新
}

Cost SwapStar::evaluate(Route *routeU,
                        Route *routeV,
                        CostEvaluator const &costEvaluator)
{
    stats_.numEvaluations++; // 评估次数加一

    // 如果两条路线没有足够的重叠，则不进行评估
    if (!routeU->overlapsWith(*routeV, overlapTolerance))
        return 0; // 如果两条路线没有重叠，则返回0

    best = {}; // 重置最佳移动记录

    // 如果路线U的移除成本缓存未更新，则更新
    if (!isCached(routeU->idx(), 0))
        updateRemovalCosts(routeU, costEvaluator); // 更新路线U的移除成本缓存

    // 如果路线V的移除成本缓存未更新，则更新
    if (!isCached(routeV->idx(), 0))
        updateRemovalCosts(routeV, costEvaluator); // 更新路线V的移除成本缓存

    // 遍历两条路线中的所有客户节点对
    for (auto *U : *routeU)
        for (auto *V : *routeV)
        {
            assert(!U->isDepot() && !V->isDepot()); // 确保U和V都不是仓库节点

            // The following lines compute a delta cost of removing U and V from
            // their own routes and inserting them into the other's route in the
            // best place. This is approximate since removal and insertion are
            // evaluated separately, not taking into account that while U leaves
            // its route, V will be inserted (and vice versa).
            // 以下代码计算将U和V从各自路线中移除，并插入到另一条路线的最佳位置的成本变化。
            // 这是近似的，因为移除和插入是分开评估的，没有考虑到当U离开其路线时，V将被插入（反之亦然）。
            Cost deltaCost = 0;

            // Load is a bit tricky, so we compute that separately.
            // 负载有点棘手，所以我们单独计算。
            deltaCost += deltaLoadCost(U, V, costEvaluator);

            // 加上移除U和V的成本
            deltaCost += removalCosts(routeU->idx(), U->client());
            deltaCost += removalCosts(routeV->idx(), V->client());

            // 获取将U插入到V路线的最佳插入点和成本
            auto [extraV, UAfter] = bestInsertPoint(U, V, costEvaluator);
            deltaCost += extraV;

            // 如果当前deltaCost已经非负，则跳过后续计算，因为移动不会改进
            if (deltaCost >= 0)  // continuing here avoids evaluating another
                continue;        // costly insertion point below
                // 如果当前成本变化已经非负，则跳过，避免评估另一个代价高的插入点

            // 获取将V插入到U路线的最佳插入点和成本
            auto [extraU, VAfter] = bestInsertPoint(V, U, costEvaluator);
            deltaCost += extraU;

            // 如果找到更优的移动，则更新最佳移动记录
            if (deltaCost < best.cost)
            {
                best.cost = deltaCost;

                best.U = U;
                best.UAfter = UAfter;

                best.V = V;
                best.VAfter = VAfter;
            }
        }

    // It is possible for positive delta costs to turn negative when we do an
    // exact evaluation. But in practice that almost never happens, and is not
    // worth spending time on.
    // 当我们进行精确评估时，正的成本变化有可能变为负值。但在实践中，这几乎不会发生，
    // 因此不值得花费时间。
    if (best.cost >= 0)  // 如果没有找到改进移动，则直接返回
        return best.cost;

    // 对最佳移动进行精确评估，返回两个移动的精确成本变化之和
    return evaluateMove(best.V, best.VAfter, best.U, costEvaluator)
           + evaluateMove(best.U, best.UAfter, best.V, costEvaluator);
}

void SwapStar::apply(Route *U, Route *V) const
{
    stats_.numApplications++; // 应用次数加一
    // 确保最佳移动中的节点都存在
    assert(best.U); // 确保最佳移动中的U节点存在
    assert(best.UAfter); // 确保最佳移动中的UAfter节点存在
    assert(best.V); // 确保最佳移动中的V节点存在
    assert(best.VAfter); // 确保最佳移动中的VAfter节点存在

    // 执行交换：先从各自路线中移除节点
    U->remove(best.U->idx()); // 从路线U中移除最佳移动的U节点
    V->remove(best.V->idx()); // 从路线V中移除最佳移动的V节点

    // 然后将节点插入到对方路线中的指定位置
    V->insert(best.UAfter->idx() + 1, best.U); // 将U节点插入到路线V的指定位置
    U->insert(best.VAfter->idx() + 1, best.V); // 将V节点插入到路线U的指定位置
}

void SwapStar::update(Route *U) 
{ 
    // 当路线U发生变化时，标记其移除成本缓存为无效，下次评估时需要重新计算
    isCached(U->idx(), 0) = false; // 标记路线U的移除成本缓存为未更新
}

SwapStar::SwapStar(ProblemData const &data, double overlapTolerance)
    : RouteOperator(data),  // 调用基类构造函数
      overlapTolerance(overlapTolerance),  // 初始化重叠容忍度参数
      insertCache(data.numVehicles(), data.numLocations()), // 初始化插入点缓存矩阵（行数=车辆数，列数=位置数）
      isCached(data.numVehicles(), data.numLocations()), // 初始化缓存状态矩阵
      removalCosts(data.numVehicles(), data.numLocations()) // 初始化移除成本矩阵
{
    // 检查重叠容忍度参数是否在有效范围内[0, 1]
    if (overlapTolerance < 0 || overlapTolerance > 1)
        throw std::invalid_argument("overlap_tolerance must be in [0, 1]."); // 检查重叠容忍度参数的有效性
}
