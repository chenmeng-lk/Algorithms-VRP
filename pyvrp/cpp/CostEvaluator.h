#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

#include "Measure.h"
#include "Solution.h"

#include <cassert>
#include <concepts>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace pyvrp
{
// The following methods must be implemented for a type to be evaluatable by
// the CostEvaluator.
// 以下方法必须被实现，以便一个类型能被CostEvaluator评估
template <typename T>
concept CostEvaluatable = requires(T arg) {
    { arg.distanceCost() } -> std::same_as<Cost>;  // 距离成本
    { arg.durationCost() } -> std::same_as<Cost>;  // 持续时间成本
    { arg.fixedVehicleCost() } -> std::same_as<Cost>;  // 固定车辆成本
    { arg.excessLoad() } -> std::convertible_to<std::vector<Load>>;  // 超载量
    { arg.excessDistance() } -> std::same_as<Distance>;  // 超距距离
    { arg.timeWarp() } -> std::same_as<Duration>;  // 时间扭曲
    { arg.empty() } -> std::same_as<bool>;  // 是否为空
    { arg.isFeasible() } -> std::same_as<bool>;  // 是否可行
};

// If, additionally, methods related to optional clients and prize collecting
// are implemented we can also take that aspect into account. See the
// CostEvaluator implementation for details.
// 如果额外实现了与可选客户和奖励收集相关的方法，我们也可以考虑这个方面。详情参见CostEvaluator实现。
template <typename T>
concept PrizeCostEvaluatable = CostEvaluatable<T> && requires(T arg) {
    { arg.uncollectedPrizes() } -> std::same_as<Cost>;  // 未收集的奖励
};

// The following methods must be available before a type's delta cost can be
// evaluated by the CostEvaluator.
// 以下方法必须可用，以便一个类型的增量成本可以被CostEvaluator评估
template <typename T>
concept DeltaCostEvaluatable = requires(T arg, size_t dimension) {
    { arg.route() };  // 所属路线
    { arg.distance() } -> std::convertible_to<std::pair<Cost, Distance>>;  // 距离信息（成本，超距）
    { arg.duration() } -> std::convertible_to<std::pair<Cost, Duration>>;  // 持续时间信息（成本，时间扭曲）
    { arg.excessLoad(dimension) } -> std::same_as<Load>;  // 指定维度的超载量
};

/**
 * CostEvaluator(
 *     load_penalties: list[float],
 *     tw_penalty: float,
 *     dist_penalty: float,
 * )
 *
 * Creates a CostEvaluator instance.
 *
 * This class stores various penalty terms, and can be used to determine the
 * costs of certain constraint violations.
 *
 * Parameters
 * ----------
 * load_penalties
 *    The penalty terms (one for each load dimension) for each unit of load in
 *    excess of the vehicle capacity.
 * tw_penalty
 *    The penalty for each unit of time warp.
 * dist_penalty
 *    The penalty for each unit of distance in excess of the vehicle's maximum
 *    distance constraint.
 *
 * Raises
 * ------
 * ValueError
 *     When any of the given penalty terms are negative.
 */
class CostEvaluator  // 成本评估器类：存储各种惩罚项并用于确定约束违反的成本
{
    std::vector<double> loadPenalties_;  // per load dimension 每个负载维度的惩罚项
    double twPenalty_;  // 时间扭曲惩罚项
    double distPenalty_;  // 距离惩罚项

    /**
     * Computes the cost penalty incurred from the given excess loads. This is
     * a convenient shorthand for calling ``loadPenalty`` for each dimension.
     */
    // 计算超载惩罚
    [[nodiscard]] inline Cost
    excessLoadPenalties(std::vector<Load> const &excessLoads) const;

public:
    // 构造函数：初始化成本评估器
    CostEvaluator(std::vector<double> loadPenalties,
                  double twPenalty,
                  double distPenalty);

    /**
     * Computes the total excess load penalty for the given load and vehicle
     * capacity, and dimension.
     */
    // 计算负载惩罚
    [[nodiscard]] inline Cost
    loadPenalty(Load load, Load capacity, size_t dimension) const;

    /**
     * Computes the time warp penalty for the given time warp.
     */
    // 计算时间扭曲惩罚
    [[nodiscard]] inline Cost twPenalty(Duration timeWarp) const;

    /**
     * Computes the total excess distance penalty for the given distance.
     */
    // 计算距离惩罚
    [[nodiscard]] inline Cost distPenalty(Distance distance,
                                          Distance maxDistance) const;

    /**
     * Computes the excess distance penalty for the given excess distance.
     */
    // 计算超距惩罚
    [[nodiscard]] inline Cost excessDistPenalty(Distance excessDistance) const;

    /**
     * Computes a smoothed objective (penalised cost) for a given solution.
     */
    // The docstring above is written for Python, where we only expose this
    // method for Solution.
    // 计算惩罚成本
    template <CostEvaluatable T>
    [[nodiscard]] Cost penalisedCost(T const &arg) const;

    /**
     * Hand-waving some details, each solution consists of a set of non-empty
     * routes :math:`\mathcal{R}`. Each route :math:`R \in \mathcal{R}` can be
     * represented as a sequence of edges, starting and ending at a depot. A
     * route :math:`R` has an assigned vehicle type that equips the route with
     * fixed vehicle cost :math:`f_R`, and unit distance, duration and overtime
     * costs :math:`c^\text{distance}_R`, :math:`c^\text{duration}_R`,
     * :math:`c^\text{overtime}_R`, respectively. Let
     * :math:`V_R = \{i : (i, j) \in R \}` be the set of locations visited by
     * route :math:`R`, and :math:`d_R`, :math:`t_R`, and :math:`o_R` the total
     * route distance, duration, and overtime, respectively. The objective value
     * is then given by
     *
     * .. math::
     *
     *    \sum_{R \in \mathcal{R}}
     *      \left[
     *          f_R + c^\text{distance}_R d_R
     *              + c^\text{duration}_R t_R
     *              + c^\text{overtime}_R o_R
     *      \right]
     *    + \sum_{i \in V} p_i - \sum_{R \in \mathcal{R}} \sum_{i \in V_R} p_i,
     *
     * where the first part lists each route's fixed, distance, duration and
     * overtime costs, respectively, and the second part the uncollected prizes
     * of unvisited clients.
     *
     * .. note::
     *
     *    The above cost computation only holds for feasible solutions. If the
     *    solution argument is *infeasible*, we return a very large number.
     *    If that is not what you want, consider calling :meth:`penalised_cost`
     *    instead.
     */
    // The docstring above is written for Python, where we only expose this
    // method for the Solution class.
    // 计算成本
    template <CostEvaluatable T> [[nodiscard]] Cost cost(T const &arg) const;

    /**
     * Evaluates the cost delta of the given route proposal, and writes the
     * resulting cost delta to the ``out`` parameter. The evaluation can be
     * exact, if the relevant template argument is set. Else it may shortcut
     * once it determines that the proposal does not constitute an improving
     * move. Optionally, several aspects of the evaluation may be skipped.
     *
     * The return value indicates whether the evaluation was exact or not.
     */
    // 计算成本增量
    template <bool exact = false,
              bool skipLoad = false,
              typename... Args,
              template <typename...>
              class T>
        requires(DeltaCostEvaluatable<T<Args...>>)
    bool deltaCost(Cost &out, T<Args...> const &proposal) const;

    /**
     * Evaluates the cost delta of the given route proposals, and writes the
     * resulting cost delta to the ``out`` parameter. The evaluation can be
     * exact, if the relevant template argument is set. Else it may shortcut
     * once it determines that the proposals do not constitute an improving
     * move. Optionally, several aspects of the evaluation may be skipped.
     *
     * The return value indicates whether the evaluation was exact or not.
     */
    // 计算两个提议的成本增量
    template <bool exact = false,
              bool skipLoad = false,
              typename... uArgs,
              typename... vArgs,
              template <typename...>
              class T>
        requires(DeltaCostEvaluatable<T<uArgs...>>
                 && DeltaCostEvaluatable<T<vArgs...>>)
    bool deltaCost(Cost &out,
                   T<uArgs...> const &uProposal,
                   T<vArgs...> const &vProposal) const;
};

 // 计算超载惩罚
Cost CostEvaluator::excessLoadPenalties(
    std::vector<Load> const &excessLoads) const
{
    assert(excessLoads.size() == loadPenalties_.size());  // 确保超载向量与负载惩罚项数量一致

    Cost cost = 0;
    for (size_t dim = 0; dim != loadPenalties_.size(); ++dim)  // 遍历每个负载维度
        cost += loadPenalties_[dim] * excessLoads[dim].get();  // 累加每个维度的超载惩罚

    return cost;  // 返回总超载惩罚
}

// 计算负载惩罚
Cost CostEvaluator::loadPenalty(Load load,
                                Load capacity,
                                size_t dimension) const
{
    assert(dimension < loadPenalties_.size());  // 确保维度索引有效
    auto const excessLoad = std::max<Load>(load - capacity, 0);  // 计算超载量（非负）
    return static_cast<Cost>(excessLoad.get() * loadPenalties_[dimension]);  // 返回该维度的超载惩罚
}

// 计算时间扭曲惩罚
Cost CostEvaluator::twPenalty([[maybe_unused]] Duration timeWarp) const
{
    return static_cast<Cost>(timeWarp.get() * twPenalty_);  // 时间扭曲惩罚 = 时间扭曲值 × 时间扭曲惩罚系数
}

// 计算距离惩罚
Cost CostEvaluator::distPenalty(Distance distance, Distance maxDistance) const
{
    auto const excessDistance = std::max<Distance>(distance - maxDistance, 0);  // 计算超距距离（非负）
    return excessDistPenalty(excessDistance);  // 调用超距惩罚函数
}

// 计算超距惩罚
Cost CostEvaluator::excessDistPenalty(Distance excessDistance) const
{
    return static_cast<Cost>(excessDistance.get() * distPenalty_);  // 超距惩罚 = 超距距离 × 距离惩罚系数
}

// 计算惩罚成本（包含约束违反的惩罚）
template <CostEvaluatable T>
Cost CostEvaluator::penalisedCost(T const &arg) const
{
    if (arg.empty())  // 如果参数为空（例如空路线或空解）
    {
        if constexpr (PrizeCostEvaluatable<T>)  // 如果参数支持奖励收集
            return arg.uncollectedPrizes();  // 返回未收集的奖励（即所有客户的奖励总和）
        return 0;  // 否则返回0
    }

    // Standard objective plus infeasibility-related penalty terms.
    // 标准目标函数加上与不可行性相关的惩罚项
    auto const cost  // 计算总成本 = 距离成本 + 时间成本 + 固定车辆成本 + 超载惩罚 + 时间扭曲惩罚 + 超距惩罚
        = arg.distanceCost() + arg.durationCost() + arg.fixedVehicleCost()
          + excessLoadPenalties(arg.excessLoad()) + twPenalty(arg.timeWarp())
          + distPenalty(arg.excessDistance(), 0);

    if constexpr (PrizeCostEvaluatable<T>)  // 如果参数支持奖励收集
        return cost + arg.uncollectedPrizes();  // 总成本加上未收集的奖励

    return cost;  // 返回总成本
}

// 计算成本（仅对可行解返回惩罚成本，否则返回最大值）
template <CostEvaluatable T> Cost CostEvaluator::cost(T const &arg) const
{
    // Penalties are zero when the solution is feasible, so we can fall back to
    // penalised cost in that case.
    // 当解可行时，惩罚项为零，因此我们可以回退到惩罚成本
    return arg.isFeasible() ? penalisedCost(arg)  // 如果可行，返回惩罚成本（实际上就是标准成本）
                            : std::numeric_limits<Cost>::max();  // 如果不可行，返回最大成本值（表示不可接受）
}

// 计算单个提议的成本增量（修改操作前后的成本变化）
template <bool exact,
          bool skipLoad,
          typename... Args,
          template <typename...>
          class T>
    requires(DeltaCostEvaluatable<T<Args...>>)
bool CostEvaluator::deltaCost(Cost &out, T<Args...> const &proposal) const
{
    auto const *route = proposal.route();  // 获取提议对应的路线
    if (!route->empty())  // 如果路线非空
    {
        // 减去原始路线的各项成本（因为提议会修改路线，所以需要先减去原成本）
        out -= route->distanceCost();  // 减去距离成本
        out -= excessDistPenalty(route->excessDistance());  // 减去超距惩罚

        if constexpr (!skipLoad)  // 如果不跳过负载检查
            out -= excessLoadPenalties(route->excessLoad());  // 减去超载惩罚

        out -= route->durationCost();  // 减去持续时间成本
        out -= twPenalty(route->timeWarp());  // 减去时间扭曲惩罚
    }

    // 加上提议的新成本
    if (route->hasDistanceCost())  // 如果路线有距离成本（即非空路线）
    {
        auto const [cost, excess] = proposal.distance();  // 获取提议的距离信息（成本，超距）
        out += cost;  // 加上新距离成本
        out += excessDistPenalty(excess);  // 加上新超距惩罚
    }

    if constexpr (!skipLoad)  // 如果不跳过负载检查
    {
        auto const &capacity = route->capacity();  // 获取路线的容量（每个维度）
        for (size_t dim = 0; dim != capacity.size(); ++dim)  // 遍历每个负载维度
        {
            if constexpr (!exact)  // 如果不需要精确计算
                if (out >= 0)  // 如果当前增量已经非负（即不是改进），则提前返回
                    return false;

            // 加上新负载惩罚（注意：这里第二个参数为0，表示不考虑容量限制？实际上loadPenalty函数会计算超载）
            // 但proposal.excessLoad(dim)已经是超载量，所以这里计算的是超载惩罚
            out += loadPenalty(proposal.excessLoad(dim), 0, dim);
        }
    }

    if (route->hasDurationCost())  // 如果路线有持续时间成本（即非空路线）
    {
        auto const [cost, timeWarp] = proposal.duration();  // 获取提议的持续时间信息（成本，时间扭曲）
        out += cost;  // 加上新持续时间成本
        out += twPenalty(timeWarp);  // 加上新时间扭曲惩罚
    }

    return true;  // 返回true表示计算完成（如果是exact模式，总是返回true；如果是非exact模式，可能提前返回false）
}

// 计算两个提议的成本增量（同时修改两条路线的情况）
template <bool exact,
          bool skipLoad,
          typename... uArgs,
          typename... vArgs,
          template <typename...>
          class T>
    requires(DeltaCostEvaluatable<T<uArgs...>>
             && DeltaCostEvaluatable<T<vArgs...>>)

//计算两个提议的成本增量（同时修改两条路线的情况）
bool CostEvaluator::deltaCost(Cost &out,
                              T<uArgs...> const &uProposal,
                              T<vArgs...> const &vProposal) const
{
    // 处理第一条路线（u路线）
    auto const *uRoute = uProposal.route();  // 获取u提议对应的路线
    if (!uRoute->empty())  // 如果u路线非空
    {
        // 减去原始u路线的各项成本
        out -= uRoute->distanceCost();
        out -= excessDistPenalty(uRoute->excessDistance());

        if constexpr (!skipLoad)
            out -= excessLoadPenalties(uRoute->excessLoad());

        out -= uRoute->durationCost();
        out -= twPenalty(uRoute->timeWarp());
    }

    // 处理第二条路线（v路线）
    auto const *vRoute = vProposal.route();  // 获取v提议对应的路线
    if (!vRoute->empty())  // 如果v路线非空
    {
        // 减去原始v路线的各项成本
        out -= vRoute->distanceCost();
        out -= excessDistPenalty(vRoute->excessDistance());

        if constexpr (!skipLoad)
            out -= excessLoadPenalties(vRoute->excessLoad());

        out -= vRoute->durationCost();
        out -= twPenalty(vRoute->timeWarp());
    }

    // 加上u提议的新成本
    if (uRoute->hasDistanceCost())
    {
        auto const [cost, excess] = uProposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    // 加上v提议的新成本
    if (vRoute->hasDistanceCost())
    {
        auto const [cost, excess] = vProposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    if constexpr (!skipLoad)
    {
        // 处理u路线的负载惩罚
        auto const &uCapacity = uRoute->capacity();
        for (size_t dim = 0; dim != uCapacity.size(); ++dim)
        {
            if constexpr (!exact)
                if (out >= 0)  // 如果当前增量已经非负，提前返回
                    return false;

            out += loadPenalty(uProposal.excessLoad(dim), 0, dim);
        }

        // 处理v路线的负载惩罚
        auto const &vCapacity = vRoute->capacity();
        for (size_t dim = 0; dim != vCapacity.size(); ++dim)
        {
            if constexpr (!exact)
                if (out >= 0)  // 如果当前增量已经非负，提前返回
                    return false;

            out += loadPenalty(vProposal.excessLoad(dim), 0, dim);
        }
    }

    // 在添加时间成本之前，再次检查（非精确模式）
    if constexpr (!exact)
        if (out >= 0)  // 如果当前增量已经非负，提前返回
            return false;

    // 加上u提议的时间成本
    if (uRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = uProposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
    }

    // 加上v提议的时间成本
    if (vRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = vProposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
    }

    return true;  // 返回true表示计算完成
}
}  // namespace pyvrp

#endif  // PYVRP_COSTEVALUATOR_H
