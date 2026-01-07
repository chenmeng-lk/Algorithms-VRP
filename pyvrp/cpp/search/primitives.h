#ifndef PYVRP_SEARCH_PRIMITIVES_H
#define PYVRP_SEARCH_PRIMITIVES_H

#include "CostEvaluator.h"
#include "Measure.h"
#include "Route.h"

// This file stores a few basic functions for (precisely) evaluating really
// common moves. Those primitives may be useful implementing higher order
// operators.
namespace pyvrp::search
{
/**
 * Evaluates the delta cost of inserting U after V in V's route. The evaluation
 * is exact.
 *
 * Parameters
 * ----------
 * U
 *     Node to insert.
 * V
 *     Node to insert U after. Must be in a route.
 * data
 *     Problem data instance.
 * cost_evaluator
 *     Cost evaluator to use.
 *
 * Returns
 * -------
 * int
 *     Exact delta cost of inserting U after V.
 * 评估将U插入V之后V的路线中的增量成本。评估是精确的。
 * U：要插入的节点。必须不在任何路线中。V：之后要插入U的节点。必须当前在路线中。
 * data：问题数据实例。
 * cost_evaluator：要使用的成本评估器。
 * 返回值：将U插入V之后V的路线的精确增量成本。
 */
//评估将U插入V之后V的路线中的增量成本。
Cost insertCost(Route::Node *U,
                Route::Node *V,
                ProblemData const &data,
                CostEvaluator const &costEvaluator);

/**
 * Evaluates the delta cost of inserting U in the place of V. The evaluation is
 * exact.
 *
 * Parameters
 * ----------
 * U
 *     Node to insert. Must not be in a route.
 * V
 *     Node to insert U in place of. Must be in a route.
 * data
 *     Problem data instance.
 * cost_evaluator
 *     Cost evaluator to use.
 *
 * Returns
 * -------
 * int
 *     Exact delta cost of inserting U in place of V.
 * 评估将U插入V的位置的增量成本。评估是精确的。
 * U：要插入的节点。必须不在任何路线中。V：要插入U的位置的节点。必须当前在路线中。
 * data：问题数据实例。
 * cost_evaluator：要使用的成本评估器。
 * 返回值：将U插入V的位置的精确增量成本。
 */
//评估将U插入V的位置的增量成本
Cost inplaceCost(Route::Node *U,
                 Route::Node *V,
                 ProblemData const &data,
                 CostEvaluator const &costEvaluator);

/**
 * Evaluates removing U from its current route. The evaluation is exact.
 *
 * Parameters
 * ----------
 * U
 *     Node to remove (client or reload depot). Must currently be in a route.
 * data
 *     Problem data instance.
 * cost_evaluator
 *     Cost evaluator to use.
 *
 * Returns
 * -------
 * int
 *     Exact delta cost of removing U.
 * 评估从其当前路线中移除U的增量成本。评估是精确的。
 * U：要移除的节点（客户或重载配送中心）。必须当前在路线中。
 * data：问题数据实例。
 * cost_evaluator：要使用的成本评估器。
 * 返回值：移除U的精确增量成本。
 */
//评估从其当前路线中移除U的增量成本
Cost removeCost(Route::Node *U,
                ProblemData const &data,
                CostEvaluator const &costEvaluator);
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_PRIMITIVES_H
