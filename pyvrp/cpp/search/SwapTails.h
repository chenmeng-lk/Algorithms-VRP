#ifndef PYVRP_SEARCH_SWAPTAILS_H
#define PYVRP_SEARCH_SWAPTAILS_H

#include "LocalSearchOperator.h"

namespace pyvrp::search
{
/**
 * SwapTails(data: ProblemData)
 *
 * Given two nodes :math:`U` and :math:`V`, tests whether replacing the arc of
 * :math:`U` to its successor :math:`n(U)` and :math:`V` to :math:`n(V)` by
 * :math:`U \rightarrow n(V)` and :math:`V \rightarrow n(U)` is an improving
 * move.
 *
 * .. note::
 *
 *    This operator is also known as 2-OPT* in the VRP literature.
 * 给定两个节点U和V，测试将U到其后继n(U)的弧以及V到其后继n(V)的弧替换为U→n(V)和V→n(U)是否是一个改进的移动。
 * 注意：该操作在VRP文献中也被称为2-OPT*。
 */
class SwapTails : public NodeOperator
{
    using NodeOperator::NodeOperator;

public:
// 评估移动的成本变化，重写基类虚函数
    Cost evaluate(Route::Node *U,
                  Route::Node *V,
                  CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U, Route::Node *V) const override;// 应用移动操作
};

template <> bool supports<SwapTails>(ProblemData const &data);// 模板特化，判断SwapTails操作是否支持给定的问题数据
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_SWAPTAILS_H
