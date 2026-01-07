#ifndef PYVRP_SEARCH_RELOCATEWITHDEPOT_H
#define PYVRP_SEARCH_RELOCATEWITHDEPOT_H

#include "LocalSearchOperator.h"

namespace pyvrp::search
{
/**
 * RelocateWithDepot(data: ProblemData)
 *
 * Tests if inserting a reload depot while relocating :math:`U` after :math:`V`
 * results in an improving move. Concretely, this operator implements the second
 * and third insertion scheme of Francois et al. [1]_.
 *
 * RelocateWithDepot(data: ProblemData)
 *
 * 测试在将U重新定位到V之后时插入重载仓库是否能产生改进的移动。
 * 具体来说，该算子实现了Francois等人[1]中的第二和第三种插入方案。
 *
 * 参考文献
 * ----------
 * .. [1] Francois, V., Y. Arda, and Y. Crama (2019). Adaptive Large
 *        Neighborhood Search for Multitrip Vehicle Routing with Time Windows.
 *        *Transportation Science*, 53(6): 1706 - 1730.
 *        https://doi.org/10.1287/trsc.2019.0909.
 */
class RelocateWithDepot : public NodeOperator
{
    using NodeOperator::NodeOperator;  // 继承基类的构造函数

    enum class MoveType   // 移动类型枚举
    {
        DEPOT_U,  // V -> depot -> U，即V之后先插入仓库再插入U
        U_DEPOT,  // V -> U -> depot，即V之后先插入U再插入仓库
    };

    struct Move   // 移动结构体，用于记录最佳移动信息
    {
        Cost cost = std::numeric_limits<Cost>::max();  // 移动成本，初始化为最大值
        MoveType type = MoveType::DEPOT_U;             // 移动类型，默认为DEPOT_U
        size_t depot = 0;                              // 仓库索引
    };

    Move move_;   // 存储当前找到的最佳移动

    // Evaluates moves where a reload depot is inserted before U, as
    // V -> depot -> U.
    // 评估在U之前插入重载仓库的移动，即V -> depot -> U
    void evalDepotBefore(Cost fixedCost,
                         Route::Node *U,
                         Route::Node *V,
                         CostEvaluator const &costEvaluator);

    // Evaluates moves where a reload depot is inserted after U, as
    // V -> U -> depot.
    // 评估在U之后插入重载仓库的移动，即V -> U -> depot
    void evalDepotAfter(Cost fixedCost,
                        Route::Node *U,
                        Route::Node *V,
                        CostEvaluator const &costEvaluator);

public:
    // 评估移动的成本
    Cost evaluate(Route::Node *U,
                  Route::Node *V,
                  CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U, Route::Node *V) const override;   // 应用移动操作
};

template <> bool supports<RelocateWithDepot>(ProblemData const &data);  // 模板特化：检查是否支持RelocateWithDepot操作
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_RELOCATEWITHDEPOT_H
