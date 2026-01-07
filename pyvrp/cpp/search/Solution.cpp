#include "Solution.h"

#include "primitives.h"

#include <algorithm>
#include <cassert>
#include <iterator>

using pyvrp::search::Solution;

// 构造函数：基于问题数据初始化解决方案
Solution::Solution(ProblemData const &data) : data_(data)
{
    // 为所有位置（包括仓库和客户）预留空间并创建节点
    nodes.reserve(data.numLocations());
    for (size_t loc = 0; loc != data.numLocations(); ++loc)
        nodes.emplace_back(loc);

    // 为所有车辆路线预留空间并按车辆类型创建路线
    routes.reserve(data.numVehicles());
    size_t rIdx = 0;
    for (size_t vehType = 0; vehType != data.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data.vehicleType(vehType).numAvailable;
        for (size_t vehicle = 0; vehicle != numAvailable; ++vehicle)
            routes.emplace_back(data, rIdx++, vehType);
    }
}

// 将标准解决方案加载到内部表示中
void Solution::load(pyvrp::Solution const &solution)
{
    // Determine offsets for vehicle types.
    // 计算每种车辆类型的起始偏移量，用于跟踪每种类型车辆的使用位置
    std::vector<size_t> vehicleOffset(data_.numVehicleTypes(), 0);//此数组记录每种车辆类型已使用的车辆数量
    for (size_t vehType = 1; vehType < data_.numVehicleTypes(); vehType++)
    {
        auto const prevAvail = data_.vehicleType(vehType - 1).numAvailable;
        vehicleOffset[vehType] = vehicleOffset[vehType - 1] + prevAvail;
    }

    // 遍历解决方案中的所有路线
    for (auto const &solRoute : solution.routes())
    {
        // Determine index of next route of this type to load, where we rely
        // on solution to be valid to not exceed the number of vehicles per
        // vehicle type.
        // 确定要加载的该类型路线的下一个索引，假设解决方案是有效的（不超过每种车辆类型的可用数量）
        auto const idx = vehicleOffset[solRoute.vehicleType()]++;//对应车辆类型已使用的车辆数量数加一
        auto &route = routes[idx];

        // 如果当前路线与解决方案中的路线相同，则可以跳过更新
        if (route == solRoute)  // then the current route is still OK and we
            continue;           // can skip inserting and updating

        // Else we need to clear the route and insert the updated route from
        // the solution.
        // 否则需要清除当前路线并从解决方案中插入更新后的路线
        route.clear();

        // Routes use a representation with nodes for each client, reload depot
        // (one per trip), and start/end depots. The start depot doubles as the
        // reload depot for the first trip.
        // 路线使用节点表示，包括每个客户、每个行程的补给仓库以及起始/结束仓库。
        // 起始仓库也作为第一个行程的补给仓库。
        route.reserve(solRoute.size() + solRoute.numTrips() + 1);

        // 遍历解决方案路线中的所有行程
        for (size_t tripIdx = 0; tripIdx != solRoute.numTrips(); ++tripIdx)
        {
            auto const &trip = solRoute.trip(tripIdx);

            // 如果不是第一个行程，需要先插入行程分隔符（补给仓库）
            if (tripIdx != 0)  // then we first insert a trip delimiter.
            {
                Route::Node depot = {trip.startDepot()};
                route.push_back(&depot);
            }

            // 将行程中的客户节点插入路线
            for (auto const client : trip)
                route.push_back(&nodes[client]);
        }

        // 更新路线统计信息
        route.update();
    }

    // Finally, we clear any routes that we have not re-used or inserted from
    // the solution.
    // 最后，清除任何未从解决方案中重新使用或插入的路线，保持内部状态与给定解决方案一致。
    size_t firstOfType = 0;
    for (size_t vehType = 0; vehType != data_.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data_.vehicleType(vehType).numAvailable;
        auto const firstOfNextType = firstOfType + numAvailable;//下一个车辆类型在路线数组中的起始索引
        for (size_t idx = vehicleOffset[vehType]; idx != firstOfNextType; ++idx)
            routes[idx].clear();

        firstOfType = firstOfNextType;
    }
}

// 将内部表示转换为标准解决方案格式
pyvrp::Solution Solution::unload() const
{
    std::vector<pyvrp::Route> solRoutes;
    solRoutes.reserve(data_.numVehicles());

    std::vector<size_t> visits;  // 临时存储行程中的客户访问序列

    // 遍历所有内部路线
    for (auto const &route : routes)
    {
        if (route.empty())
            continue;

        std::vector<Trip> trips;//行程列表
        trips.reserve(route.numTrips());

        visits.clear();
        visits.reserve(route.numClients());

        // 从起始仓库开始处理
        auto const *prevDepot = route[0];
        for (size_t idx = 1; idx != route.size(); ++idx)
        {
            auto const *node = route[idx];

            // 如果节点不是仓库，则将其添加到当前行程的访问序列中
            if (!node->isDepot())
            {
                visits.push_back(node->client());
                continue;
            }

            // 遇到仓库表示一个行程结束，创建行程对象
            trips.emplace_back(data_,
                               visits,
                               route.vehicleType(),
                               prevDepot->client(),
                               node->client());

            visits.clear();
            prevDepot = node;
        }

        // 验证行程数量是否匹配
        assert(trips.size() == route.numTrips());
        // 创建标准路线对象并添加到解决方案中
        solRoutes.emplace_back(data_, std::move(trips), route.vehicleType());
    }

    // 返回完整的解决方案
    return {data_, std::move(solRoutes)};
}

// 插入节点到解决方案中
bool Solution::insert(Route::Node *U,
                      SearchSpace const &searchSpace,
                      CostEvaluator const &costEvaluator,
                      bool required)
{
    // 验证节点U的指针有效性
    assert(size_t(std::distance(nodes.data(), U)) < nodes.size());

    // 初始插入位置设为第一个路线的第一个节点（备用选项）
    Route::Node *UAfter = routes[0][0];  // fallback option
    auto bestCost = insertCost(U, UAfter, data_, costEvaluator);

    // First attempt a neighbourhood search to place U into routes that are
    // already in use.
    // 首先尝试邻域搜索，将U插入已使用的路线中
    for (auto const vClient : searchSpace.neighboursOf(U->client()))
    {
        auto *V = &nodes[vClient];

        if (!V->route())//V需要在路线中
            continue;

        auto const cost = insertCost(U, V, data_, costEvaluator);
        if (cost < bestCost)
        {
            bestCost = cost;
            UAfter = V;
        }
    }

    // Next consider empty routes, of each vehicle type. We insert into the
    // first improving route.
    // 接下来考虑每种车辆类型的空路线，插入到第一个能改进的路线中
    for (auto const &[vehType, offset] : searchSpace.vehTypeOrder())
    {
        auto const begin = routes.begin() + offset;
        auto const end = begin + data_.vehicleType(vehType).numAvailable;
        auto const pred = [](auto const &route) { return route.empty(); };
        auto empty = std::find_if(begin, end, pred);

        if (empty == end)//此车辆类型没有空路线
            continue;

        auto const cost = insertCost(U, (*empty)[0], data_, costEvaluator);
        if (cost < bestCost)//找到第一个能改进的空路线就退出
        {
            bestCost = cost;
            UAfter = (*empty)[0];
            break;
        }
    }

    // 如果required为true（强制插入）或插入能改进成本，则执行插入操作
    if (required || bestCost < 0)
    {
        auto *route = UAfter->route();
        route->insert(UAfter->idx() + 1, U);
        return true;
    }

    return false;
}
