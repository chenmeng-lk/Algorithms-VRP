#ifndef PYVRP_SEARCH_SEARCHSPACE_H
#define PYVRP_SEARCH_SEARCHSPACE_H

#include "DynamicBitset.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"

#include <vector>

namespace pyvrp::search
{
/**
 * SearchSpace(data: ProblemData, neighbours: list[list[int]])
 *
 * Manages a search space for the local search. The search space is granular,
 * around the given neighbourhood, and uses the concept of promising clients
 * to determine which client's neighbourhoods to search. It can also be used
 * to define a (randomised) search ordering for clients, routes, and vehicle
 * types.
 */
class SearchSpace  // 搜索空间类：管理局部搜索的搜索空间
{
public:
    using Neighbours = std::vector<std::vector<size_t>>;  // 邻域类型：每个客户的邻近客户列表

private:
    // Neighborhood restrictions: list of nearby clients for each client (size
    // numLocations, but nothing is stored for the depots!).
    // 邻域限制：每个客户的邻近客户列表（大小为numLocations，但仓库不存储任何内容！）
    Neighbours neighbours_;

    // Tracks clients that can likely be improved by local search operators.
    // 跟踪可能被局部搜索算子改进的客户
    DynamicBitset promising_;

    // Client order used for node-based search.
    // 用于基于节点搜索的客户顺序
    std::vector<size_t> clientOrder_;

    // Route order used for route-based search.
    // 用于基于路线搜索的路线顺序
    std::vector<size_t> routeOrder_;

    // Vehicle type order - pairs of [veh type, offset] - used for empty route
    // search.
    // 车辆类型顺序 - [车辆类型, 偏移量]对 - 用于空路线搜索
    std::vector<std::pair<size_t, size_t>> vehTypeOrder_;

public:
    SearchSpace(ProblemData const &data, Neighbours neighbours);

    /**
     * Set the neighbourhood structure of this search space. For each client,
     * the neighbourhood structure is a vector of nearby clients. Depots have
     * no nearby clients.
     * 设置此搜索空间的邻域结构。对于每个客户，邻域结构是一个邻近客户的向量。仓库没有邻近客户
     */
    void setNeighbours(Neighbours neighbours);

    /**
     * Returns the current neighbourhood structure.
     * 返回当前的邻域结构
     */
    Neighbours const &neighbours() const;

    /**
     * Returns the vector of neighbours for a given client.
     * 返回给定客户的邻居向量
     */
    std::vector<size_t> const &neighboursOf(size_t client) const;

    /**
     * Returns whether the given client is a promising evaluation candidate.
     * 返回给定客户是否为有希望的评估候选者
     */
    bool isPromising(size_t client) const;

    /**
     * Marks the given client as promising.
     * 将给定客户及其直接客户邻居标记为有希望的
     */
    void markPromising(size_t client);

    /**
     * Convenient overload for route nodes. Since this is typically used during
     * insert and removals, this method marks the given node and its direct
     * client neighbours as promising. The node must currently be in a route.
     * Does not mark depots.
     * 方便的路线节点重载。由于这通常在插入和删除期间使用，此方法将给定节点及其直接客户
     * 邻居标记为有希望的。该节点必须当前在路线中。不标记仓库。
     */
    void markPromising(Route::Node const *node);

    /**
     * Marks all clients as promising.
     * 将所有客户标记为有希望的
     */
    void markAllPromising();

    /**
     * Unmarks all clients as promising.
     * 将所有客户取消标记为有希望的
     */
    void unmarkAllPromising();

    /**
     * Returns a randomised order in which the client search space may be
     * traversed. This order remains unchanged until :meth:`~shuffle` is called.
     * 返回一个随机化的顺序，可以遍历客户搜索空间。此顺序保持不变，直到调用：meth：`~shuffle`。
     */
    std::vector<size_t> const &clientOrder() const;

    /**
     * Returns a randomised order in which the route search space may be
     * traversed. This order remains unchanged until :meth:`~shuffle` is called.
     * 返回一个随机化的顺序，可以遍历路线搜索空间。此顺序保持不变，直到调用：meth：`~shuffle`。
     */
    std::vector<size_t> const &routeOrder() const;

    /**
     * Returns a randomised order in which the vehicle type space may be
     * traversed. This order remains unchanged until :meth:`~shuffle` is called.
     * 返回一个随机化的顺序，可以遍历车辆类型空间。此顺序保持不变，直到调用：meth：`~shuffle`。
     */
    std::vector<std::pair<size_t, size_t>> const &vehTypeOrder() const;

    /**
     * Randomises the client, route, and vehicle type orders using the given
     * random number generator.
     * 随机打乱客户、路线和车辆类型订单，使用给定的随机数生成器。
     */
    void shuffle(RandomNumberGenerator &rng);
};
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_SEARCHSPACE_H
