#ifndef PYVRP_SEARCH_SOLUTION_H
#define PYVRP_SEARCH_SOLUTION_H

#include "../Solution.h"  // pyvrp::Solution
#include "CostEvaluator.h"
#include "ProblemData.h"
#include "Route.h"  // pyvrp::search::Route
#include "SearchSpace.h"

#include <vector>

namespace pyvrp::search
{
/**
 * Solution(data: ProblemData)
 *
 * An alternative representation of a routing solution that is more amenable
 * to efficient modification. This is intended for use in the local search.
 *
 * This solution struct owns a vector of nodes, for the depots and clients. It
 * additionally owns a vector of (search) routes, which store non-owning
 * pointers into the nodes to model route visits. Modifying the solution via
 * search operators involves copying pointers, not whole nodes. That is very
 * efficient in practice.
 *
 * The solution does not protect its internal state---it is just a simple
 * wrapper around nodes and routes. Ensuring the solution remains valid is
 * up to the interacting code.
 */
// 解决方案类：用于本地搜索的高效修改的路径解决方案表示
class Solution
{
    ProblemData const &data_;  // 问题数据引用，包含所有位置和车辆信息

public:
    // 所有位置（包括仓库和客户）的节点向量，大小为numLocations()
    std::vector<Route::Node> nodes;  // size numLocations()
    
    // 所有车辆路线的向量，按车辆类型排序，大小为numVehicles()
    std::vector<Route> routes;       // size numVehicles(), ordered by type

    // 构造函数：基于问题数据创建解决方案
    Solution(ProblemData const &data);

    // 将给定的解决方案转换为基于节点的内部表示
    // Converts the given solution into our node-based representation.
    void load(pyvrp::Solution const &solution);

    // 将内部表示转换为标准解决方案格式
    // Converts from our representation to a proper solution.
    pyvrp::Solution unload() const;

    // 将给定节点插入到解决方案中 - 要么插入到其邻域位置，要么插入到空路线中（如果改进或需要）。
    // 如果节点成功插入返回true，否则返回false。更新搜索空间和插入路线的操作留给调用代码。
    // Inserts the given node into the solution - either in its neighbourhood,
    // or in an empty route, if improving or required. Returns true if the node
    // was successfully inserted, false otherwise. Updating the search space and
    // inserted route is left to the calling code.
    bool insert(Route::Node *node,
                SearchSpace const &searchSpace,
                CostEvaluator const &costEvaluator,
                bool required);
};
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_SOLUTION_H
