#include "SwapTails.h"

#include "Route.h"

#include <cassert>

using pyvrp::search::SwapTails;

namespace
{
// 判断节点是否在最后一个行程段上
bool onLastTrip(pyvrp::search::Route::Node *node)
{
    auto const *route = node->route();
    return node->trip() + 1 == route->numTrips();
}
}  // namespace

pyvrp::Cost SwapTails::evaluate(Route::Node *U,
                                Route::Node *V,
                                CostEvaluator const &costEvaluator)
{
    stats_.numEvaluations++;  // 增加评估次数统计
    assert(!U->isEndDepot() && !U->isReloadDepot());  // 断言U不是终点仓库或重载仓库
    assert(!V->isEndDepot() && !V->isReloadDepot());  // 断言V不是终点仓库或重载仓库

    auto const *uRoute = U->route();  // 获取节点U所在路径
    auto const *vRoute = V->route();  // 获取节点V所在路径

    if (uRoute == vRoute)  // 如果两个节点在同一路径上
        return 0;  // same route  // 相同路径，返回成本0

    if (uRoute->idx() > vRoute->idx() && !uRoute->empty() && !vRoute->empty())//只处理U在V前面的情况，避免重复计算
        return 0;  // move will be tackled in a later iteration  // 移动将在后续迭代中处理，避免重复计算

    if (!onLastTrip(U) || !onLastTrip(V))  // 如果U或V不在最后一个行程段上
        // We cannot move reload depots, so we only evaluate a move if it does
        // not include a reload depot.
        // 我们不能移动重载仓库，所以只有当移动不包含重载仓库时才进行评估
        return 0;

    Cost deltaCost = 0;  // 初始化成本变化

    // We're going to incur fixed cost if a route is currently empty but
    // becomes non-empty due to the proposed move.
    // 如果一条路径当前为空，但由于提议的移动变为非空，我们将产生固定成本
    if (uRoute->empty() && !n(V)->isEndDepot())  // 如果U所在路径为空且V的下一个节点不是终点仓库
        deltaCost += uRoute->fixedVehicleCost();  // 增加U路径的固定车辆成本

    if (vRoute->empty() && !n(U)->isEndDepot())  // 如果V所在路径为空且U的下一个节点不是终点仓库
        deltaCost += vRoute->fixedVehicleCost();  // 增加V路径的固定车辆成本

    // We lose fixed cost if a route becomes empty due to the proposed move.
    // 如果一条路径由于提议的移动变为空，我们将失去固定成本
    if (!uRoute->empty() && U->isStartDepot() && n(V)->isEndDepot())  // 如果U路径非空，U是起点仓库，且V的下一个节点是终点仓库
        deltaCost -= uRoute->fixedVehicleCost();  // 减少U路径的固定车辆成本

    if (!vRoute->empty() && V->isStartDepot() && n(U)->isEndDepot())  // 如果V路径非空，V是起点仓库，且U的下一个节点是终点仓库
        deltaCost -= vRoute->fixedVehicleCost();  // 减少V路径的固定车辆成本

    if (!n(U)->isEndDepot() && !n(V)->isEndDepot())  // 如果U和V的下一个节点都不是终点仓库
    {
        auto const uProposal
            = Route::Proposal(uRoute->before(U->idx()),  // U之前的部分
                              vRoute->between(V->idx() + 1, vRoute->size() - 2),  // V之后的尾部（从V+1到倒数第二个节点）
                              uRoute->at(uRoute->size() - 1));  // U路径的终点仓库

        auto const vProposal
            = Route::Proposal(vRoute->before(V->idx()),  // V之前的部分
                              uRoute->between(U->idx() + 1, uRoute->size() - 2),  // U之后的尾部（从U+1到倒数第二个节点）
                              vRoute->at(vRoute->size() - 1));  // V路径的终点仓库

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);  // 计算路径改变后的成本变化
    }
    else if (!n(U)->isEndDepot() && n(V)->isEndDepot())  // 如果U的下一个节点不是终点仓库，但V的下一个节点是终点仓库
    {
        auto const uProposal = Route::Proposal(uRoute->before(U->idx()),  // U之前的部分
                                               uRoute->at(uRoute->size() - 1));  // U路径的终点仓库

        auto const vProposal
            = Route::Proposal(vRoute->before(V->idx()),  // V之前的部分
                              uRoute->between(U->idx() + 1, uRoute->size() - 2),  // U之后的尾部（从U+1到倒数第二个节点）
                              vRoute->at(vRoute->size() - 1));  // V路径的终点仓库

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);  // 计算路径改变后的成本变化
    }
    else if (n(U)->isEndDepot() && !n(V)->isEndDepot())  // 如果U的下一个节点是终点仓库，但V的下一个节点不是终点仓库
    {
        auto const uProposal
            = Route::Proposal(uRoute->before(U->idx()),  // U之前的部分
                              vRoute->between(V->idx() + 1, vRoute->size() - 2),  // V之后的尾部（从V+1到倒数第二个节点）
                              uRoute->at(uRoute->size() - 1));  // U路径的终点仓库

        auto const vProposal = Route::Proposal(vRoute->before(V->idx()),  // V之前的部分
                                               vRoute->at(vRoute->size() - 1));  // V路径的终点仓库

        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);  // 计算路径改变后的成本变化
    }

    return deltaCost;  // 返回计算出的成本变化
}

void SwapTails::apply(Route::Node *U, Route::Node *V) const
{
    stats_.numApplications++;  // 增加应用次数统计
    auto *nU = n(U);  // 获取U的下一个节点
    auto *nV = n(V);  // 获取V的下一个节点

    auto insertIdx = U->idx() + 1;  // 在U路径上的插入位置（U之后）
    while (!nV->isEndDepot())  // 循环直到V的尾部节点全部移动完
    {
        auto *node = nV;  // 当前要移动的节点
        nV = n(nV);  // 更新nV为下一个节点
        V->route()->remove(node->idx());  // 从V路径移除该节点
        U->route()->insert(insertIdx++, node);  // 插入到U路径的指定位置
    }

    insertIdx = V->idx() + 1;  // 在V路径上的插入位置（V之后）
    while (!nU->isEndDepot())  // 循环直到U的尾部节点全部移动完
    {
        auto *node = nU;  // 当前要移动的节点
        nU = n(nU);  // 更新nU为下一个节点
        U->route()->remove(node->idx());  // 从U路径移除该节点
        V->route()->insert(insertIdx++, node);  // 插入到V路径的指定位置
    }//更新路线信息是在算子调用处完成的
}

template <> bool pyvrp::search::supports<SwapTails>(ProblemData const &data)
{
    // Does not work for TSP, since the operator needs at least two routes.
    // 不适用于TSP，因为该操作需要至少两条路径
    return data.numVehicles() > 1;  // 车辆数量大于1时才支持该操作
}
