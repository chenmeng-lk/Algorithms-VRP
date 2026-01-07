#ifndef PYVRP_SEARCH_EXCHANGE_H
#define PYVRP_SEARCH_EXCHANGE_H

#include "LocalSearchOperator.h"
#include "Route.h"
#include "primitives.h"

#include <cassert>

namespace pyvrp::search
{
/**
 * Exchange(data: ProblemData)
 *
 * The :math:`(N, M)`-exchange operators exchange :math:`N` consecutive clients
 * from :math:`U`'s route (starting at :math:`U`) with :math:`M` consecutive
 * clients from :math:`V`'s route (starting at :math:`V`). This includes
 * the RELOCATE and SWAP operators as special cases.
 *
 * The :math:`(N, M)`-exchange class uses C++ templates for different :math:`N`
 * and :math:`M` to efficiently evaluate these moves.
 *
 * Exchange(data: ProblemData)
 * (N, M)-exchange算子从U所在的路径中交换N个连续的客户（从U开始）与V所在路径中的M个连续的客户（从V开始）。
 * 这包括RELOCATE和SWAP算子作为特例。
 * (N, M)-exchange类使用C++模板针对不同的N和M来高效评估这些移动。
 */
template <size_t N, size_t M> class Exchange : public NodeOperator
{
    using NodeOperator::NodeOperator;

    static_assert(N >= M && N > 0, "N < M or N == 0 does not make sense");  // 静态断言：N必须大于等于M且大于0，否则没有意义

    // Tests if the segment starting at node of given length contains the depot
    // 测试从给定节点开始、长度为segLength的段是否包含仓库
    bool containsDepot(Route::Node *node, size_t segLength) const;

    // Tests if the segments of U and V overlap in the same route
    // 测试U和V的段在同一路径中是否重叠
    bool overlap(Route::Node *U, Route::Node *V) const;

    // Tests if the segments of U and V are adjacent in the same route
    // 测试U和V的段在同一路径中是否相邻
    bool adjacent(Route::Node *U, Route::Node *V) const;

    // Special case that's applied when M == 0
    // 当M == 0时应用的特殊情况（即搬迁操作）
    Cost evalRelocateMove(Route::Node *U,
                          Route::Node *V,
                          CostEvaluator const &costEvaluator) const;

    // Applied when M != 0
    // 当M != 0时应用的情况（即交换操作）
    Cost evalSwapMove(Route::Node *U,
                      Route::Node *V,
                      CostEvaluator const &costEvaluator) const;

public:
    Cost evaluate(Route::Node *U,
                  Route::Node *V,
                  CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U, Route::Node *V) const override;
};

template <size_t N, size_t M>
bool Exchange<N, M>::containsDepot(Route::Node *node, size_t segLength) const
{
    auto const first = node->idx();                    // 获取段的起始索引
    auto const last = first + segLength - 1;           // 计算段的结束索引
    auto const &route = *node->route();                // 获取节点所在路径的引用

    return first == 0                               // contains start depot
           || last >= route.size() - 1              // contains end depot
           || node->trip() != route[last]->trip();  // contains reload depot
           // 如果起始索引为0则包含起始仓库，如果结束索引大于等于路径大小减1则包含结束仓库，如果起始节点与结束节点的行程不同则包含重载仓库
}

template <size_t N, size_t M>
bool Exchange<N, M>::overlap(Route::Node *U, Route::Node *V) const
{
    return U->route() == V->route()           // 首先检查是否在同一路径中
           // We need max(M, 1) here because when V is the depot and M == 0,
           // this would turn negative and wrap around to a large number.
           // 这里需要max(M, 1)因为当V是仓库且M==0时，V->idx() + M - 1可能为负数并回绕成一个很大的数
           && U->idx() <= V->idx() + std::max<size_t>(M, 1) - 1
           && V->idx() <= U->idx() + N - 1;
           // 检查U的段是否与V的段有重叠：U的起始索引小于等于V的结束索引，且V的起始索引小于等于U的结束索引
}

template <size_t N, size_t M>
bool Exchange<N, M>::adjacent(Route::Node *U, Route::Node *V) const
{
    return U->route() == V->route()           // 首先检查是否在同一路径中
           && (U->idx() + N == V->idx() || V->idx() + M == U->idx());
           // 检查两个段是否相邻：U的结束索引的下一个位置是V的起始索引，或者V的结束索引的下一个位置是U的起始索引
}

template <size_t N, size_t M>
Cost Exchange<N, M>::evalRelocateMove(Route::Node *U,
                                      Route::Node *V,
                                      CostEvaluator const &costEvaluator) const
{
    assert(U->idx() > 0);  // 断言U不是起始仓库（因为起始仓库不能被移动）

    Cost deltaCost = 0;    // 初始化成本变化为0

    if (U->route() != V->route())   // 如果U和V不在同一路径中（即跨路径搬迁）
    {
        auto const *uRoute = U->route();   // 获取U所在路径
        auto const *vRoute = V->route();   // 获取V所在路径

        // We're going to incur V's fixed cost if V is currently empty.
        // 如果V当前是空路径（V是起始仓库且路径为空），我们将承担V的固定车辆成本
        if (V->isStartDepot() && vRoute->empty())
            deltaCost += vRoute->fixedVehicleCost();

        // We lose U's fixed cost if we're moving all U's clients.
        // 如果我们移动了U的所有客户，我们将失去U的固定车辆成本
        if (uRoute->numClients() == N)
            deltaCost -= uRoute->fixedVehicleCost();

        // 构建U路径的提案：移除从U开始的N个节点，即连接U前一个节点和U后第N个节点
        auto const uProposal = Route::Proposal(uRoute->before(U->idx() - 1),
                                               uRoute->after(U->idx() + N));
        // 构建V路径的提案：在V之后插入从U开始的N个节点
        auto const vProposal
            = Route::Proposal(vRoute->before(V->idx()),
                              uRoute->between(U->idx(), U->idx() + N - 1),
                              vRoute->after(V->idx() + 1));

        // 通过成本评估器计算两个提案的成本变化
        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }
    else  // within same route, 在同一路径内移动（即路径内重排）
    {
        auto *route = U->route();   // 获取路径（U和V在同一路径）

        if (U->idx() < V->idx())   // 如果U在V之前
            costEvaluator.deltaCost(
                deltaCost,
                // 构建提案：将U开始的N个节点移动到V之后
                Route::Proposal(route->before(U->idx() - 1),
                                route->between(U->idx() + N, V->idx()),
                                route->between(U->idx(), U->idx() + N - 1),
                                route->after(V->idx() + 1)));
        else   // 如果U在V之后
            costEvaluator.deltaCost(
                deltaCost,
                // 构建提案：将U开始的N个节点移动到V之后（注意顺序）
                Route::Proposal(route->before(V->idx()),
                                route->between(U->idx(), U->idx() + N - 1),
                                route->between(V->idx() + 1, U->idx() - 1),
                                route->after(U->idx() + N)));
    }

    return deltaCost;   // 返回计算出的成本变化
}

template <size_t N, size_t M>
Cost Exchange<N, M>::evalSwapMove(Route::Node *U,
                                  Route::Node *V,
                                  CostEvaluator const &costEvaluator) const
{
    assert(U->idx() > 0 && V->idx() > 0);   // 断言U和V都不是起始仓库
    assert(U->route() && V->route());       // 断言U和V都有所属路径

    Cost deltaCost = 0;   // 初始化成本变化为0

    if (U->route() != V->route())   // 如果U和V不在同一路径中（即跨路径交换）
    {
        auto const *uRoute = U->route();   // 获取U所在路径
        auto const *vRoute = V->route();   // 获取V所在路径

        // 构建U路径的提案：将U开始的N个节点替换为V开始的M个节点
        auto const uProposal
            = Route::Proposal(uRoute->before(U->idx() - 1),
                              vRoute->between(V->idx(), V->idx() + M - 1),
                              uRoute->after(U->idx() + N));
        // 构建V路径的提案：将V开始的M个节点替换为U开始的N个节点
        auto const vProposal
            = Route::Proposal(vRoute->before(V->idx() - 1),
                              uRoute->between(U->idx(), U->idx() + N - 1),
                              vRoute->after(V->idx() + M));

        // 通过成本评估器计算两个提案的成本变化
        costEvaluator.deltaCost(deltaCost, uProposal, vProposal);
    }
    else  // within same route, 在同一路径内交换
    {
        auto const *route = U->route();   // 获取路径（U和V在同一路径）

        if (U->idx() < V->idx())   // 如果U在V之前
            costEvaluator.deltaCost(
                deltaCost,
                // 构建提案：交换U开始的N个节点和V开始的M个节点
                Route::Proposal(route->before(U->idx() - 1),
                                route->between(V->idx(), V->idx() + M - 1),
                                route->between(U->idx() + N, V->idx() - 1),
                                route->between(U->idx(), U->idx() + N - 1),
                                route->after(V->idx() + M)));
        else   // 如果U在V之后
            costEvaluator.deltaCost(
                deltaCost,
                // 构建提案：交换V开始的M个节点和U开始的N个节点（注意顺序）
                Route::Proposal(route->before(V->idx() - 1),
                                route->between(U->idx(), U->idx() + N - 1),
                                route->between(V->idx() + M, U->idx() - 1),
                                route->between(V->idx(), V->idx() + M - 1),
                                route->after(U->idx() + N)));
    }

    return deltaCost;   // 返回计算出的成本变化
}

template <size_t N, size_t M>
Cost Exchange<N, M>::evaluate(Route::Node *U,
                              Route::Node *V,
                              CostEvaluator const &costEvaluator)
{
    stats_.numEvaluations++;   // 增加评估次数统计

    if (containsDepot(U, N) || overlap(U, V))   // 如果U的段包含仓库或两个段重叠
        return 0;   // 返回0表示不可行或成本无变化

    if constexpr (M > 0)   // 编译时检查：如果M>0
        if (containsDepot(V, M))   // 如果V的段包含仓库
            return 0;   // 返回0表示不可行

    // We cannot easily evaluate across trips, so we cannot determine this move.
    // 我们不容易跨行程评估，因此无法确定这个移动是否可行
    if (U->route() == V->route() && U->trip() != V->trip())
        return 0;   // 如果在同一路径但不同行程，返回0

    if constexpr (M == 0)  // special case where nothing in V is moved, M==0的特殊情况：V处没有节点被移动（即搬迁操作）
    {
        if (U == n(V))   // 如果U是V的后继节点（避免无效移动）
            return 0;   // 返回0

        return evalRelocateMove(U, V, costEvaluator);   // 评估搬迁移动的成本
    }
    else   // M != 0的情况（交换操作）
    {
        if constexpr (N == M)  // symmetric, so only have to evaluate this once
            // 如果N==M，交换是对称的，只需评估一次（避免重复评估）
            if (U->client() >= V->client())   // 通过客户编号避免重复评估相同的交换
                return 0;   // 返回0

        if (adjacent(U, V))   // 如果两个段相邻
            return 0;   // 返回0（相邻交换可能无效或已由其他算子处理）

        return evalSwapMove(U, V, costEvaluator);   // 评估交换移动的成本
    }
}

template <size_t N, size_t M>
void Exchange<N, M>::apply(Route::Node *U, Route::Node *V) const
{
    stats_.numApplications++;   // 增加应用次数统计

    auto &uRoute = *U->route();   // 获取U所在路径的引用
    auto &vRoute = *V->route();   // 获取V所在路径的引用
    auto *uToInsert = N == 1 ? U : uRoute[U->idx() + N - 1];   // 确定要插入的U段最后一个节点（如果N==1就是U本身）
    auto *insertUAfter = M == 0 ? V : vRoute[V->idx() + M - 1];   // 确定U段要插入的位置（在哪个节点之后插入）

    // Insert these 'extra' nodes of U after the end of V...
    // 将U段中“额外”的节点（即N-M个节点）插入到V段之后...
    for (size_t count = 0; count != N - M; ++count)
    {
        auto *prev = p(uToInsert);   // 获取当前要插入节点的前驱节点（因为要从后往前插入）
        uRoute.remove(uToInsert->idx());   // 从U路径中移除当前节点
        vRoute.insert(insertUAfter->idx() + 1, uToInsert);   // 将当前节点插入到V路径中指定位置之后
        uToInsert = prev;   // 移动到前一个节点（继续插入前一个节点）
    }

    // ...and swap the overlapping nodes!
    // ...然后交换重叠的节点！（即剩下的M个节点与V段中的M个节点交换）
    for (size_t count = 0; count != M; ++count)
    {
        Route::swap(U, V);   // 交换U和V两个节点
        U = n(U);   // U移动到原U段中的下一个节点
        V = n(V);   // V移动到原V段中的下一个节点
    }
}
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_EXCHANGE_H
