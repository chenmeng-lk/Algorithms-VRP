#include "primitives.h"

#include <cassert>

namespace
{
/**
 * Simple wrapper class that implements the required evaluation interface for
 * a single client that might not currently be in the solution.
 * 简单包装类，为当前可能不在解中的单个客户实现所需的评估接口。
 */
class ClientSegment
{
    pyvrp::ProblemData const &data;
    size_t client;

public:
    ClientSegment(pyvrp::ProblemData const &data, size_t client)
        : data(data), client(client)
    {
        assert(client >= data.numDepots());  // must be an actual client // 必须是实际客户，不能是站点
    }

    pyvrp::search::Route const *route() const { return nullptr; }  // 该客户段不属于任何路线

    size_t first() const { return client; }   // 客户段的第一个节点就是该客户
    size_t last() const { return client; }    // 客户段的最后一个节点也是该客户
    size_t size() const { return 1; }         // 客户段大小为1（只有一个客户）

    bool startsAtReloadDepot() const { return false; }  // 不以重载站点开始
    bool endsAtReloadDepot() const { return false; }    // 不以重载站点结束

    pyvrp::Distance distance([[maybe_unused]] size_t profile) const
    {
        return 0;  // 单个客户没有内部距离
    }

    pyvrp::DurationSegment duration([[maybe_unused]] size_t profile) const
    {
        pyvrp::ProblemData::Client const &clientData = data.location(client);
        return {clientData};  // 返回客户的时间段信息
    }

    pyvrp::LoadSegment load(size_t dimension) const
    {
        return {data.location(client), dimension};  // 返回客户在指定维度上的负载段
    }
};
}  // namespace

// 计算将节点U插入到节点V之后的成本变化
pyvrp::Cost pyvrp::search::insertCost(Route::Node *U,
                                      Route::Node *V,
                                      ProblemData const &data,
                                      CostEvaluator const &costEvaluator)
{
    if (!V->route() || U->isDepot())  // 如果V不在路线中或U是站点，则成本变化为0
        return 0;

    auto *route = V->route();  // 获取V所在的路线
    ProblemData::Client const &client = data.location(U->client());  // 获取U的客户数据

    // 计算固定车辆成本和客户奖励的差值（如果路线为空则包含固定车辆成本）
    Cost deltaCost
        = Cost(route->empty()) * route->fixedVehicleCost() - client.prize;

    // 使用成本评估器计算插入操作的成本变化
    costEvaluator.deltaCost<true>(
        deltaCost,
        Route::Proposal(route->before(V->idx()),  // V之前的路线段
                        ClientSegment(data, U->client()),  // 要插入的客户段
                        route->after(V->idx() + 1)));  // V之后的路线段（从V的下一个节点开始）

    return deltaCost;  // 返回总成本变化
}

// 计算从路线中移除节点U的成本变化
pyvrp::Cost pyvrp::search::removeCost(Route::Node *U,
                                      ProblemData const &data,
                                      CostEvaluator const &costEvaluator)
{
    // 如果U不在路线中，或是起始/结束站点，则成本变化为0
    if (!U->route() || U->isStartDepot() || U->isEndDepot())
        return 0;

    auto *route = U->route();  // 获取U所在的路线
    Cost deltaCost = 0;  // 初始化成本变化

    if (!U->isDepot())  // 如果U不是站点（即客户节点）
    {
        ProblemData::Client const &client = data.location(U->client());  // 获取U的客户数据
        // 计算客户奖励和固定车辆成本的差值（如果移除后路线为空则减去固定车辆成本）
        deltaCost
            = client.prize
              - Cost(route->numClients() == 1) * route->fixedVehicleCost();
    }

    // 使用成本评估器计算移除操作的成本变化
    costEvaluator.deltaCost<true>(deltaCost,
                                  Route::Proposal(route->before(U->idx() - 1),  // U之前的路线段（到U的前一个节点）
                                                  route->after(U->idx() + 1)));  // U之后的路线段（从U的下一个节点开始）

    return deltaCost;  // 返回总成本变化
}

// 计算用节点U替换节点V（原地替换）的成本变化
pyvrp::Cost pyvrp::search::inplaceCost(Route::Node *U,
                                       Route::Node *V,
                                       ProblemData const &data,
                                       CostEvaluator const &costEvaluator)
{
    // 如果U已在路线中或V不在路线中，则成本变化为0
    if (U->route() || !V->route())
        return 0;

    auto const *route = V->route();  // 获取V所在的路线
    ProblemData::Client const &uClient = data.location(U->client());  // 获取U的客户数据
    ProblemData::Client const &vClient = data.location(V->client());  // 获取V的客户数据
    // 计算客户奖励的差值（V的奖励减去U的奖励）
    Cost deltaCost = vClient.prize - uClient.prize;

    // 使用成本评估器计算替换操作的成本变化
    costEvaluator.deltaCost<true>(
        deltaCost,
        Route::Proposal(route->before(V->idx() - 1),  // V之前的路线段（到V的前一个节点）
                        ClientSegment(data, U->client()),  // 替换为U的客户段
                        route->after(V->idx() + 1)));  // V之后的路线段（从V的下一个节点开始）

    return deltaCost;  // 返回总成本变化
}
