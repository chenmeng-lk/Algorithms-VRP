#ifndef PYVRP_SEARCH_LOCALSEARCH_H
#define PYVRP_SEARCH_LOCALSEARCH_H

#include "CostEvaluator.h"
#include "LocalSearchOperator.h"
#include "PerturbationManager.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"
#include "SearchSpace.h"
#include "Solution.h"  // pyvrp::search::Solution

#include <functional>
#include <stdexcept>
#include <vector>

namespace pyvrp::search
{
class LocalSearch  // 局部搜索类：执行局部搜索以改进解
{
    ProblemData const &data;  // 问题数据引用

    // Stores the node-based solution representation used during LS.
    // 存储局部搜索期间使用的基于节点的解表示
    Solution solution_;

    // Manages the granular neighbourhood, promising clients, and the order in
    // which nodes and routes are searched.
    // 管理细粒度邻域、有希望的客户以及节点和路线的搜索顺序
    SearchSpace searchSpace_;

    // Perturbation manager that determines the size of the perturbation during
    // each LS invocation.
    // 扰动管理器，确定每次局部搜索调用期间的扰动大小
    PerturbationManager &perturbationManager_;

    std::vector<NodeOperator *> nodeOps;   // 节点算子列表
    std::vector<RouteOperator *> routeOps; // 路线算子列表

    std::vector<int> lastTestedNodes;   // tracks node operator evaluation 跟踪节点算子评估
    std::vector<int> lastTestedRoutes;  // tracks route operator evaluation 跟踪路线算子评估
    std::vector<int> lastUpdated;       // tracks when routes were last modified 跟踪路线最后修改时间

    size_t numUpdates_ = 0;         // modification counter 修改计数器
    bool searchCompleted_ = false;  // No further improving move found? 是否找到进一步的改进移动

    // Load an initial solution that we will attempt to improve.
    // 加载一个初始解，我们将尝试改进它
    void loadSolution(pyvrp::Solution const &solution);

    // Tests the node pair (U, V).
    // 测试节点对(U, V)
    bool applyNodeOps(Route::Node *U,
                      Route::Node *V,
                      CostEvaluator const &costEvaluator);

    // Tests the route pair (U, V).
    // 测试路线对(U, V)
    bool applyRouteOps(Route *U, Route *V, CostEvaluator const &costEvaluator);

    // Tests a move removing the given reload depot.
    // 测试移除给定重新装载仓库的移动
    void applyDepotRemovalMove(Route::Node *U,
                               CostEvaluator const &costEvaluator);

    // Tests moves involving empty routes.
    // 测试涉及空路线的移动
    void applyEmptyRouteMoves(Route::Node *U,
                              CostEvaluator const &costEvaluator);

    // Tests moves involving missing or optional clients.
    // 测试涉及缺失或可选客户的移动
    void applyOptionalClientMoves(Route::Node *U,
                                  CostEvaluator const &costEvaluator);

    // Tests moves involving clients in client groups.
    // 测试涉及客户组中客户的移动
    void applyGroupMoves(Route::Node *U, CostEvaluator const &costEvaluator);

    // Updates solution state after an improving local search move.
    // 在改进的局部搜索移动后更新解状态
    void update(Route *U, Route *V);

    // Performs search on the currently loaded solution.
    // 对当前加载的解执行搜索
    void search(CostEvaluator const &costEvaluator);

    // Performs intensify on the currently loaded solution.
    // 对当前加载的解执行强化搜索
    void intensify(CostEvaluator const &costEvaluator);

    // Marks missing but required clients and groups as promising, to ensure
    // they get inserted.
    // 将有希望但缺失的必需客户和组标记为有希望的，以确保它们被插入
    void markRequiredMissingAsPromising();

public:
    /**
     * Simple data structure that tracks statistics about the number of local
     * search moves applied to the most recently improved solution.
     *
     * Attributes
     * ----------
     * num_moves
     *     Number of evaluated node and route operator moves.
     * num_improving
     *     Number of evaluated moves that led to an objective improvement.
     * num_updates
     *     Total number of changes to the solution. This always includes the
     *     number of evaluated improving moves, but also e.g. insertion of
     *     required but missing clients.
     */
    struct Statistics  // 统计信息结构：跟踪应用于最近改进解的局部搜索移动数量
    {
        // Number of evaluated moves, that is, number of evaluations of a node
        // or route operator.
        // 评估的移动数量，即节点或路线算子的评估次数
        size_t const numMoves;

        // Number of evaluated moves that led to an objective improvement.
        // 导致目标改进的评估移动数量
        size_t const numImproving;

        // Number of times the solution has been modified in some way.
        // 解被修改的次数（包括改进移动和必需客户的插入等）
        size_t const numUpdates;
    };

    /**
     * Adds a local search operator that works on node/client pairs U and V.
     */
    void addNodeOperator(NodeOperator &op);// 添加一个作用于节点/客户对U和V的局部搜索算子

    /**
     * Adds a local search operator that works on route pairs U and V.
     */
    void addRouteOperator(RouteOperator &op);// 添加一个作用于路线对U和V的局部搜索算子

    /**
     * Returns the node operators in use. Note that there is no defined
     * ordering.
     */
    std::vector<NodeOperator *> const &nodeOperators() const;// 返回正在使用的节点算子。请注意，没有定义的顺序

    /**
     * Returns the route operators in use. Note that there is no defined
     * ordering.
     */
    std::vector<RouteOperator *> const &routeOperators() const;// 返回正在使用的路线算子。请注意，没有定义的顺序

    /**
     * Set neighbourhood structure to use by the local search. For each client,
     * the neighbourhood structure is a vector of nearby clients. Depots have
     * no nearby client.
     *  设置局部搜索使用的邻域结构。对于每个客户，邻域结构是一个邻近客户的向量。仓库没有邻近客户
     */
    void setNeighbours(SearchSpace::Neighbours neighbours);

    /**
     * Returns the current neighbourhood structure.
     */
    SearchSpace::Neighbours const &neighbours() const;// 返回当前的邻域结构

    /**
     * Returns search statistics for the currently loaded solution.
     */
    Statistics statistics() const;// 返回当前加载解的搜索统计信息

    /**
     * Iteratively calls ``search()`` and ``intensify()`` until no further
     * improvements are made.
     *  迭代调用search()和intensify()，直到不再有进一步的改进
     */
    pyvrp::Solution operator()(pyvrp::Solution const &solution,
                               CostEvaluator const &costEvaluator);

    /**
     * Performs regular (node-based) local search around the given solution,
     * and returns a new, hopefully improved solution.
     * 对给定解周围的解执行常规（基于节点的）局部搜索，并返回一个新的、有希望的改进解
     */
    pyvrp::Solution search(pyvrp::Solution const &solution,
                           CostEvaluator const &costEvaluator);

    /**
     * Performs a more intensive route-based local search around the given
     * solution, and returns a new, hopefully improved solution.
     * 对给定解周围的解执行更密集的基于路线的局部搜索，并返回一个新的、有希望的改进解
     */
    pyvrp::Solution intensify(pyvrp::Solution const &solution,
                              CostEvaluator const &costEvaluator);

    /**
     * Shuffles the order in which the node and route pairs are evaluated, and
     * the order in which operators are applied.
     * 随机打乱节点和路线对的评估顺序，以及算子的应用顺序
     */
    void shuffle(RandomNumberGenerator &rng);

    LocalSearch(ProblemData const &data,
                SearchSpace::Neighbours neighbours,
                PerturbationManager &perturbationManager);
};
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_LOCALSEARCH_H
