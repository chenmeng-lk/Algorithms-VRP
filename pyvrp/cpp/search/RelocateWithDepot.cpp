#include "RelocateWithDepot.h"

#include "Route.h"

#include <cassert>

using pyvrp::search::RelocateWithDepot;

namespace
{
/**
 * Simple wrapper class that implements the required evaluation interface for
 * a single reload depot.
 *
 * 简单的包装类，为单个重载仓库实现所需的评估接口。
 */
class ReloadDepotSegment
{
    pyvrp::ProblemData const &data_;  // 问题数据引用
    size_t depot_;                    // 仓库索引

public:
    ReloadDepotSegment(pyvrp::ProblemData const &data, size_t depot)
        : data_(data), depot_(depot)
    {
        assert(depot < data.numDepots());  // must be an actual depot, 必须是一个有效的仓库索引
    }

    pyvrp::search::Route const *route() const { return nullptr; }  // 重载仓库不属于任何路径，返回空指针

    size_t first() const { return depot_; }   // 返回仓库索引作为第一个元素
    size_t last() const { return depot_; }    // 返回仓库索引作为最后一个元素
    size_t size() const { return 1; }         // 大小为1（只有一个仓库）

    bool startsAtReloadDepot() const { return true; }   // 起始于重载仓库
    bool endsAtReloadDepot() const { return true; }     // 结束于重载仓库

    pyvrp::Distance distance([[maybe_unused]] size_t profile) const
    {
        return 0;   // 重载仓库本身没有距离，返回0
    }

    pyvrp::DurationSegment duration([[maybe_unused]] size_t profile) const
    {
        pyvrp::ProblemData::Depot const &depot = data_.location(depot_);  // 获取仓库数据
        return {depot};   // 返回仓库的持续时间段（包含仓库的服务时间等）
    }

    pyvrp::LoadSegment load([[maybe_unused]] size_t dimension) const
    {
        return {};   // 重载仓库的负载段为空（不携带任何负载）
    }
};
}  // namespace
//评估在U之前插入重载仓库的移动，即V -> depot -> U
void RelocateWithDepot::evalDepotBefore(Cost fixedCost,
                                        Route::Node *U,
                                        Route::Node *V,
                                        CostEvaluator const &costEvaluator)
{
    auto const *uRoute = U->route();   // 获取U节点所在的路径
    auto const *vRoute = V->route();   // 获取V节点所在的路径
    auto const &vehType = data.vehicleType(vRoute->vehicleType());  // 获取V路径的车辆类型

    if (uRoute != vRoute)   // 如果U和V在不同路径中（跨路径移动）
    {
        auto const uProposal = Route::Proposal(uRoute->before(U->idx() - 1),
                                               uRoute->after(U->idx() + 1));
        // 构建U路径的提案：移除U节点，即连接U前一个节点和U后一个节点

        for (auto const depot : vehType.reloadDepots)   // 遍历车辆类型允许的所有重载仓库
        {
            auto deltaCost = fixedCost;   // 初始化成本变化为固定成本
            costEvaluator.deltaCost(
                deltaCost,
                uProposal,
                // 构建V路径的提案：在V之后插入重载仓库和U节点
                Route::Proposal(vRoute->before(V->idx()),
                                ReloadDepotSegment(data, depot),   // 插入重载仓库
                                uRoute->at(U->idx()),              // 插入U节点
                                vRoute->after(V->idx() + 1)));     // V之后的路径部分

            if (deltaCost < move_.cost)   // 如果找到成本更低的移动
                move_ = {deltaCost, MoveType::DEPOT_U, depot};   // 更新最佳移动记录
        }
    }
    else  // within same route, 在同一路径内移动
    {
        auto const *route = vRoute;   // 获取路径（U和V在同一路径）
        for (auto const depot : vehType.reloadDepots)   // 遍历车辆类型允许的所有重载仓库
        {
            auto deltaCost = fixedCost;   // 初始化成本变化为固定成本
            if (U->idx() < V->idx())   // 如果U在V之前
                costEvaluator.deltaCost(
                    deltaCost,
                    // 构建提案：将U节点移动到V之后，并在它们之间插入重载仓库
                    Route::Proposal(route->before(U->idx() - 1),
                                    route->between(U->idx() + 1, V->idx()),
                                    ReloadDepotSegment(data, depot),   // 插入重载仓库
                                    route->at(U->idx()),              // 插入U节点
                                    route->after(V->idx() + 1)));
            else   // 如果U在V之后
                costEvaluator.deltaCost(
                    deltaCost,
                    // 构建提案：将U节点移动到V之后，并在它们之间插入重载仓库
                    Route::Proposal(route->before(V->idx()),
                                    ReloadDepotSegment(data, depot),   // 插入重载仓库
                                    route->at(U->idx()),              // 插入U节点
                                    route->between(V->idx() + 1, U->idx() - 1),
                                    route->after(U->idx() + 1)));

            if (deltaCost < move_.cost)   // 如果找到成本更低的移动
                move_ = {deltaCost, MoveType::DEPOT_U, depot};   // 更新最佳移动记录
        }
    }
}
//评估在U之后插入重载仓库的移动，即V -> U -> depot
void RelocateWithDepot::evalDepotAfter(Cost fixedCost,
                                       Route::Node *U,
                                       Route::Node *V,
                                       CostEvaluator const &costEvaluator)
{
    auto const *uRoute = U->route();   // 获取U节点所在的路径
    auto const *vRoute = V->route();   // 获取V节点所在的路径
    auto const &vehType = data.vehicleType(vRoute->vehicleType());  // 获取V路径的车辆类型

    if (uRoute != vRoute)   // 如果U和V在不同路径中（跨路径移动）
    {
        auto const uProposal = Route::Proposal(uRoute->before(U->idx() - 1),
                                               uRoute->after(U->idx() + 1));
        // 构建U路径的提案：移除U节点，即连接U前一个节点和U后一个节点

        for (auto const depot : vehType.reloadDepots)   // 遍历车辆类型允许的所有重载仓库
        {
            Cost deltaCost = fixedCost;   // 初始化成本变化为固定成本
            costEvaluator.deltaCost(
                deltaCost,
                uProposal,
                // 构建V路径的提案：在V之后插入U节点和重载仓库
                Route::Proposal(vRoute->before(V->idx()),
                                uRoute->at(U->idx()),              // 先插入U节点
                                ReloadDepotSegment(data, depot),   // 再插入重载仓库
                                vRoute->after(V->idx() + 1)));     // V之后的路径部分

            if (deltaCost < move_.cost)   // 如果找到成本更低的移动
                move_ = {deltaCost, MoveType::U_DEPOT, depot};   // 更新最佳移动记录
        }
    }
    else  // within same route, 在同一路径内移动
    {
        auto const *route = vRoute;   // 获取路径（U和V在同一路径）
        for (auto const depot : vehType.reloadDepots)   // 遍历车辆类型允许的所有重载仓库
        {
            Cost deltaCost = fixedCost;   // 初始化成本变化为固定成本
            if (U->idx() < V->idx())   // 如果U在V之前
                costEvaluator.deltaCost(
                    deltaCost,
                    // 构建提案：将U节点移动到V之后，并在U之后插入重载仓库
                    Route::Proposal(route->before(U->idx() - 1),
                                    route->between(U->idx() + 1, V->idx()),
                                    route->at(U->idx()),              // 插入U节点
                                    ReloadDepotSegment(data, depot),   // 插入重载仓库
                                    route->after(V->idx() + 1)));
            else   // 如果U在V之后
                costEvaluator.deltaCost(
                    deltaCost,
                    // 构建提案：将U节点移动到V之后，并在U之后插入重载仓库
                    Route::Proposal(route->before(V->idx()),
                                    route->at(U->idx()),              // 插入U节点
                                    ReloadDepotSegment(data, depot),   // 插入重载仓库
                                    route->between(V->idx() + 1, U->idx() - 1),
                                    route->after(U->idx() + 1)));

            if (deltaCost < move_.cost)   // 如果找到成本更低的移动
                move_ = {deltaCost, MoveType::U_DEPOT, depot};   // 更新最佳移动记录
        }
    }
}
//评估移动的成本，把U节点移动到V之后，并在它们之间插入重载仓库
pyvrp::Cost RelocateWithDepot::evaluate(Route::Node *U,
                                        Route::Node *V,
                                        CostEvaluator const &costEvaluator)
{
    assert(!U->isDepot() && !V->isEndDepot());  // 断言U不是仓库，V不是结束仓库
    stats_.numEvaluations++;   // 增加评估次数统计

    auto const *uRoute = U->route();   // 获取U节点所在的路径
    auto const *vRoute = V->route();   // 获取V节点所在的路径

    if (U == n(V) || vRoute->empty())  // if V's empty, Exchange<1, 0> suffices
        // 如果U是V的后继节点或V的路径为空，则使用Exchange<1, 0>就够了，返回0表示无需评估
        return 0;

    if (vRoute->numTrips() == vRoute->maxTrips())   // 如果V的路径已达到最大行程数
        return 0;   // 返回0表示不可行

    // Cannot evaluate this move because it requires a load segment to contain
    // a reload depot in the middle, which makes concatenation far more complex.
    // 无法评估此移动，因为它需要在负载段中间包含一个重载仓库，这使得连接操作变得非常复杂。
    if (uRoute == vRoute && U->trip() != V->trip())   // 如果在同一路径但不同行程
        return 0;   // 返回0表示无法评估

    move_ = {};   // 重置移动记录

    Cost fixedCost = 0;   // 初始化固定成本变化为0
    if (uRoute != vRoute && uRoute->numClients() == 1)  // empty after move
        // 如果跨路径移动且U的路径只有一个客户（移动后U的路径将为空）
        fixedCost -= uRoute->fixedVehicleCost();   // 减去U路径的固定车辆成本（因为路径将变空）

    if (!V->isReloadDepot())   // 如果V本身不是重载仓库
        // 如果V已经是重载仓库，那么在它后面直接插入另一个重载仓库没有意义。
        // 但如果V不是重载仓库，那么插入重载仓库可能有助于处理初始车辆负载。
        evalDepotBefore(fixedCost, U, V, costEvaluator);   // 评估在U之前插入仓库的移动

    if (!n(V)->isReloadDepot())   // 如果V的后继节点不是重载仓库
        // 如果n(V)是重载仓库，那么在它前面直接插入另一个重载仓库没有意义。
        // 但如果n(V)不是重载仓库，那么插入重载仓库可能有助于确保车辆空载返回。
        evalDepotAfter(fixedCost, U, V, costEvaluator);   // 评估在U之后插入仓库的移动

    return move_.cost;   // 返回找到的最佳移动成本
}
//应用移动操作
void RelocateWithDepot::apply(Route::Node *U, Route::Node *V) const
{
    stats_.numApplications++;   // 增加应用次数统计

    auto *uRoute = U->route();   // 获取U节点所在的路径
    uRoute->remove(U->idx());    // 从U的路径中移除U节点

    auto *vRoute = V->route();   // 获取V节点所在的路径
    Route::Node depot = {move_.depot};   // 创建重载仓库节点

    if (move_.type == MoveType::DEPOT_U)   // 如果是DEPOT_U类型的移动（仓库在U之前）
    {
        vRoute->insert(V->idx() + 1, U);     // 在V之后插入U节点
        vRoute->insert(V->idx() + 1, &depot); // 在V之后插入仓库节点（在U之前）
    }

    // We need to be careful to insert the depot last, because doing so could
    // invalidate V (it might trigger an update to the route's internal data
    // layout, which could invalidate V if V is a depot).
    // 我们需要小心最后插入仓库节点，因为这样做可能使V无效（如果V是仓库，插入操作可能触发路径内部数据布局的更新，从而使V无效）。
    if (move_.type == MoveType::U_DEPOT)   // 如果是U_DEPOT类型的移动（U在仓库之前）
    {
        vRoute->insert(V->idx() + 1, U);     // 在V之后插入U节点
        vRoute->insert(V->idx() + 2, &depot); // 在U之后插入仓库节点
    }
}

template <>
bool pyvrp::search::supports<RelocateWithDepot>(ProblemData const &data)//返回是否支持RelocateWithDepot操作
{
    // We need at least one vehicle type for which reloading is enabled.
    // 我们需要至少一种启用了重载功能的车辆类型。
    for (auto const &vehType : data.vehicleTypes())   // 遍历所有车辆类型
        if (!vehType.reloadDepots.empty() && vehType.maxReloads != 0)   // 如果车辆类型有重载仓库且允许重载
            return true;   // 支持RelocateWithDepot操作

    return false;   // 不支持RelocateWithDepot操作
}