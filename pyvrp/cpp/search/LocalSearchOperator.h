#ifndef PYVRP_SEARCH_LOCALSEARCHOPERATOR_H
#define PYVRP_SEARCH_LOCALSEARCHOPERATOR_H

#include "../Solution.h"  // pyvrp::Solution
#include "CostEvaluator.h"
#include "Measure.h"
#include "ProblemData.h"
#include "Route.h"

namespace pyvrp::search
{
/**
 * Simple data structure that tracks statistics about the number of times
 * an operator was evaluated and applied.
 * 简单数据结构：跟踪算子被评估和应用次数的统计信息
 *
 * Attributes
 * ----------
 * num_evaluations
 *     Number of evaluated moves.
 *     评估的移动数量
 * num_applications
 *     Number of applied, improving moves.
 *     应用的改进移动数量
 */
struct OperatorStatistics  // 算子统计信息结构
{
    size_t numEvaluations = 0;  // 评估次数
    size_t numApplications = 0;  // 应用次数
};

template <typename Arg> class LocalSearchOperator  // 局部搜索算子基类模板，Arg可以是Route::Node或Route类型
{
    // Can only be specialised into either a Node or Route operator; there
    // are no other types that are expected to work.
    // 只能特化为节点或路线算子；没有其他预期工作的类型
    static_assert(std::is_same<Arg, Route::Node>::value
                  || std::is_same<Arg, Route>::value);  // 编译时断言，确保模板参数只能是Route::Node或Route

protected:
    ProblemData const &data;  // 问题数据引用，存储问题实例的常量引用，提供客户、车辆等数据
    mutable OperatorStatistics stats_;  // 算子统计信息，mutable允许在const成员函数中修改

public:
    /**
     * Determines the cost delta of applying this operator to the arguments.
     * If the cost delta is negative, this is an improving move.
     * <br />
     * The contract is as follows: if the cost delta is negative, that is the
     * true cost delta of this move. As such, improving moves are fully
     * evaluated. The operator, however, is free to return early if it knows
     * the move will never be good: that is, when it determines the cost delta
     * cannot become negative at all. In that case, the returned (non-negative)
     * cost delta does not constitute a full evaluation.
     * 计算将算子应用于参数U和V时的成本变化值（delta）。如果delta为负，则表示这是一个改进移动。
     * 契约如下：如果成本delta为负，则表示此移动的真实成本变化。因此，改进移动会被完整评估。
     * 但是，如果算子确定移动永远不会变好（即成本delta不可能为负），则可以提前返回非负值，此时返回的delta不代表完整评估结果。
     */
    virtual Cost evaluate(Arg *U, Arg *V, CostEvaluator const &costEvaluator) = 0;  // 纯虚函数：评估移动的成本变化，需子类实现

    /**
     * Applies this operator to the given arguments. For improvements, should
     * only be called if <code>evaluate()</code> returns a negative delta cost.
     * 将算子应用于给定的参数U和V。对于改进移动，仅当evaluate()返回负成本delta时才应调用此函数。
     */
    // TODO remove arguments - always applies to most recently evaluated pair.
    virtual void apply(Arg *U, Arg *V) const = 0;  // 纯虚函数：执行移动，需子类实现

    /**
     * Called once after loading the solution to improve. This can be used to
     * e.g. update local operator state.
     * 在加载待改进解后调用一次。可用于更新算子内部状态（如缓存）。
     */
    virtual void init([[maybe_unused]] pyvrp::Solution const &solution)  // 虚函数：初始化算子状态，默认重置统计信息
    {
        stats_ = {};  // reset call statistics  // 重置统计信息（评估和应用次数归零）
    };

    /**
     * Returns evaluation and application statistics collected since the last
     * solution initialisation.
     * 返回自上次解初始化以来收集的评估和应用统计信息。
     */
    OperatorStatistics const &statistics() const { return stats_; }  // 返回统计信息的常量引用

    LocalSearchOperator(ProblemData const &data) : data(data){};  // 构造函数，初始化问题数据引用
    virtual ~LocalSearchOperator() = default;  // 虚析构函数，确保正确清理派生类资源
};

/**
 * Node operator base class.
 * 节点算子基类，特化LocalSearchOperator用于操作Route中的节点（客户点）。
 */
using NodeOperator = LocalSearchOperator<Route::Node>;  // 类型别名：节点算子，模板参数为Route::Node

/**
 * Route operator base class.
 * 路线算子基类，特化LocalSearchOperator用于操作整条路线。
 */
class RouteOperator : public LocalSearchOperator<Route>  // 路由算子基类，继承自LocalSearchOperator<Route>
{
    using LocalSearchOperator::LocalSearchOperator;  // 继承基类的构造函数

public:
    /**
     * Called when a route has been changed. Can be used to update caches, but
     * the implementation should be fast: this is called every time something
     * changes!
     * 当路线被更改时调用。可用于更新缓存，但实现应该快速：每次有变化时都会调用！
     * 更新路线缓存，虚函数各个算子具体实现
     */
    virtual void update([[maybe_unused]] Route *U) {};  // 虚函数：更新缓存，默认空实现，派生类可按需重写
};

/**
 * Helper template function that may be specialised to determine if an operator
 * can find improving moves for the given data instance.
 * 辅助模板函数，可特化以判断给定算子是否能在特定问题实例中找到改进移动。
 */
template <typename Op> bool supports([[maybe_unused]] ProblemData const &data)  // 检查算子是否支持特定问题实例
{
    return true;  // 默认返回true，表示支持所有实例；特化版本可提供具体判断逻辑
}
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_LOCALSEARCHOPERATOR_H
