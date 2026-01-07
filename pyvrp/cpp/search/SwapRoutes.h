#ifndef PYVRP_SEARCH_SWAPROUTES_H
#define PYVRP_SEARCH_SWAPROUTES_H

#include "LocalSearchOperator.h"
#include "SwapTails.h"

namespace pyvrp::search
{
/**
 * SwapRoutes(data: ProblemData)
 *
 * This operator evaluates exchanging the visits of two routes :math:`U` and
 * :math:`V`.
 * 在两条路径的起点仓库之后交换整个路径
 * 利用车辆类型的差异性来优化整体解决方案
 */
class SwapRoutes : public RouteOperator
{
    SwapTails op;  // 使用SwapTails算子来实现路径交换

public:
    Cost
    evaluate(Route *U, Route *V, CostEvaluator const &costEvaluator) override;

    void apply(Route *U, Route *V) const override;

    explicit SwapRoutes(ProblemData const &data);
};

template <> bool supports<SwapRoutes>(ProblemData const &data);
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_SWAPROUTES_H
