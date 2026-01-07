#ifndef PYVRP_SEARCH_SWAPSTAR_H
#define PYVRP_SEARCH_SWAPSTAR_H

#include "LocalSearchOperator.h"
#include "Matrix.h"

#include <array>
#include <utility>

namespace pyvrp::search
{
/**
 * SwapStar(data: ProblemData, overlap_tolerance: float = 0.05)
 *
 * Explores the SWAP* neighbourhood of [1]_. The SWAP* neighbourhood consists
 * of free form re-insertions of clients :math:`U` and :math:`V` in the given
 * routes (so the clients are swapped, but they are not necessarily inserted
 * in the place of the other swapped client).
 *
 * 探索[1]_中提出的SWAP*邻域。SWAP*邻域包括在给定路线中自由重新插入客户:math:`U`和:math:`V`
 * （因此客户被交换，但它们不一定插入到另一个被交换客户的位置）。
 *
 * References
 * ----------
 * .. [1] Thibaut Vidal. 2022. Hybrid genetic search for the CVRP: Open-source
 *        implementation and SWAP* neighborhood. *Comput. Oper. Res*. 140.
 *        https://doi.org/10.1016/j.cor.2021.105643
 */
class SwapStar : public RouteOperator  // SWAP*算子类，继承自RouteOperator
{
    using InsertPoint = std::pair<Cost, Route::Node *>;  // 插入点类型：成本和对节点的指针
    using ThreeBest = std::array<InsertPoint, 3>;  // 三个最佳插入点的数组

    struct BestMove  // 记录最佳SWAP*移动的结构体
    {
        Cost cost = 0;  // 移动的成本变化值（delta）

        Route::Node *U = nullptr;  // 第一个要交换的客户节点
        Route::Node *UAfter = nullptr;  // 在V的路线中，将U插入到这个节点之后

        Route::Node *V = nullptr;  // 第二个要交换的客户节点
        Route::Node *VAfter = nullptr;  // 在U的路线中，将V插入到这个节点之后
    };

    // To limit computational efforts, by default not all route pairs are
    // considered: only those route pairs that share some overlap when
    // considering their center's angle to the center of all clients. This value
    // controls the amount of overlap needed before two routes are evaluated.
    // 为了限制计算量，默认不考虑所有路线对：只考虑那些在考虑其中心与所有客户中心的角度时共享一定重叠的路线对。
    // 这个值控制了两条路线被评估所需的重叠量。
    double const overlapTolerance;  // 重叠容忍度参数

    // Tracks the three best insert locations, for each route and client.
    // 为每条路线和每个客户跟踪三个最佳插入位置。
    Matrix<ThreeBest> insertCache;  // 插入位置缓存矩阵，存储每条路线每个客户的三个最佳插入点

    // Tracks whether the insert locations and removal costs are still up to
    // date. In particular, isCached(R, 0) tracks route-wise removal cost
    // validity, while isCached(R, U) with U > 0 tracks (route, client) insert
    // location validity.
    // 跟踪插入位置和移除成本是否仍然是最新的。
    // 具体来说，isCached(R, 0)跟踪路线级别的移除成本有效性，而isCached(R, U)（U>0）跟踪(路线, 客户)插入位置有效性。
    Matrix<bool> isCached;  // 缓存有效性矩阵

    // Tracks the removal costs of removing a client from its route.
    // 跟踪将客户从其路线中移除的成本。
    Matrix<Cost> removalCosts;  // 移除成本矩阵

    BestMove best;  // 当前找到的最佳移动

    // Updates the removal costs of clients in the given route
    // 更新给定路线中客户的移除成本
    void updateRemovalCosts(Route *R, CostEvaluator const &costEvaluator);

    // Updates the cache storing the three best positions in the given route for
    // the passed-in node (client).
    // 更新缓存，存储传入节点（客户）在给定路线中的三个最佳位置。
    void updateInsertPoints(Route *R,
                            Route::Node *U,
                            CostEvaluator const &costEvaluator);

    // 计算交换两个客户U和V时的负载成本变化
    Cost deltaLoadCost(Route::Node *U,
                       Route::Node *V,
                       CostEvaluator const &costEvaluator) const;

    // 查找在V的路线中插入U的最佳插入点
    InsertPoint bestInsertPoint(Route::Node *U,
                                Route::Node *V,
                                CostEvaluator const &costEvaluator);

    // Evaluates the delta cost for ``V``'s route of inserting ``U`` after
    // ``V``, while removing ``remove`` from ``V``'s route.
    // 评估在V的路线中，将U插入到V之后，同时从V的路线中移除remove的成本变化。
    Cost evaluateMove(Route::Node const *U,
                      Route::Node const *V,
                      Route::Node const *remove,
                      CostEvaluator const &costEvaluator) const;

public:
    // 初始化算子，设置缓存数据结构并重置统计信息
    void init(pyvrp::Solution const &solution) override;

    // 评估两条路线之间执行SWAP*移动的成本变化
    Cost
    evaluate(Route *U, Route *V, CostEvaluator const &costEvaluator) override;

    // 应用最佳找到的SWAP*移动
    void apply(Route *U, Route *V) const override;

    // 更新路线U的缓存信息，标记路线U的移除成本缓存为未更新
    void update(Route *U) override;

    // 构造函数，需要问题数据和重叠容忍度参数
    explicit SwapStar(ProblemData const &data, double overlapTolerance = 0.05);
};
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_SWAPSTAR_H
