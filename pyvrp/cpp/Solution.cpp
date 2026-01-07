#include "Solution.h"
#include "DurationSegment.h"
#include "DynamicBitset.h"

#include <algorithm>
#include <fstream>
#include <numeric>
#include <unordered_map>

using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;
using pyvrp::Route;
using pyvrp::Solution;

using Client = size_t;
using Routes = std::vector<Route>;
using Neighbours = std::vector<std::optional<std::pair<Client, Client>>>;

// 评估解的特征：计算所有统计信息
void Solution::evaluate(ProblemData const &data)
{
    // 计算所有客户的总奖励值
    Cost allPrizes = 0;
    for (auto const &client : data.clients())
        allPrizes += client.prize;

    // 初始化超额负载向量
    excessLoad_ = std::vector<Load>(data.numLoadDimensions(), 0);
    for (auto const &route : routes_)
    {
        // Whole solution statistics. 整个解的统计信息
        numClients_ += route.size();  // 累加客户数量
        prizes_ += route.prizes();    // 累加奖励值
        distance_ += route.distance();  // 累加距离
        distanceCost_ += route.distanceCost();  // 累加距离成本
        duration_ += route.duration();  // 累加持续时间
        overtime_ += route.overtime();  // 累加加班时间
        durationCost_ += route.durationCost();  // 累加持续时间成本
        excessDistance_ += route.excessDistance();  // 累加超额距离
        timeWarp_ += route.timeWarp();  // 累加时间扭曲
        fixedVehicleCost_ += data.vehicleType(route.vehicleType()).fixedCost;  // 累加固定车辆成本

        // 累加每个维度的超额负载
        auto const &excessLoad = route.excessLoad();
        for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
            excessLoad_[dim] += excessLoad[dim];
    }

    // 计算未收集的奖励值
    uncollectedPrizes_ = allPrizes - prizes_;
}

// 判断解是否为空（没有客户且没有路线）
bool Solution::empty() const { return numClients() == 0 && numRoutes() == 0; }

// 返回解中的路线数量
size_t Solution::numRoutes() const { return routes_.size(); }

// 返回解中的行程总数（所有路线的行程数之和）
size_t Solution::numTrips() const
{
    return std::accumulate(routes_.begin(),
                           routes_.end(),
                           0,
                           [](size_t count, auto const &route)
                           { return count + route.numTrips(); });
}

// 返回解中的客户数量
size_t Solution::numClients() const { return numClients_; }

// 返回缺失的必需客户数量
size_t Solution::numMissingClients() const { return numMissingClients_; }

// 返回路线列表
Routes const &Solution::routes() const { return routes_; }

// 返回邻居列表（每个客户的[前驱, 后继]对）
Neighbours const &Solution::neighbours() const { return neighbours_; }

bool Solution::isFeasible() const
{
    // clang-format off
    return !hasExcessLoad()//所有路线都没有超额负载
        && !hasTimeWarp()//没有时间扭曲
        && !hasExcessDistance()//所有路线都没有超额距离
        && isComplete()//没有缺失客户
        && isGroupFeasible();//客户组可行
    // clang-format on
}

bool Solution::isGroupFeasible() const { return isGroupFeas_; }//关于客户组是否可行

bool Solution::isComplete() const { return numMissingClients_ == 0; }//有无缺失的必需客户

bool Solution::hasExcessLoad() const//所有路线是否有超额负载
{
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

bool Solution::hasExcessDistance() const { return excessDistance_ > 0; }//所有路线的超额距离总和

bool Solution::hasTimeWarp() const { return timeWarp_ > 0; }//所有路线的总时间扭曲

Distance Solution::distance() const { return distance_; }//所有路线的总行驶距离

Cost Solution::distanceCost() const { return distanceCost_; }

Duration Solution::duration() const { return duration_; }

Duration Solution::overtime() const { return overtime_; }

Cost Solution::durationCost() const { return durationCost_; }

std::vector<Load> const &Solution::excessLoad() const { return excessLoad_; }

Distance Solution::excessDistance() const { return excessDistance_; }

Cost Solution::fixedVehicleCost() const { return fixedVehicleCost_; }

Cost Solution::prizes() const { return prizes_; }

Cost Solution::uncollectedPrizes() const { return uncollectedPrizes_; }

Duration Solution::timeWarp() const { return timeWarp_; }

// 构建邻居关系：确定每个已分配客户的[前驱, 后继]对
void Solution::makeNeighbours()
{
    for (auto const &route : routes_)
        for (auto const &trip : route.trips())
        {
            auto const startDepot = trip.startDepot();  // 行程的起始仓库
            auto const endDepot = trip.endDepot();      // 行程的结束仓库

            // 为行程中的每个客户设置前驱和后继
            for (size_t idx = 0; idx != trip.size(); ++idx)
                neighbours_[trip[idx]] = {
                    idx == 0 ? startDepot : trip[idx - 1],               // pred 前驱：第一个客户的前驱是起始仓库，否则是前一个客户
                    idx == trip.size() - 1 ? endDepot : trip[idx + 1]};  // succ 后继：最后一个客户的后继是结束仓库，否则是下一个客户
        }
}

// 解的相等性比较运算符
bool Solution::operator==(Solution const &other) const
{
    // clang-format off
    bool const attributeChecks = distance_ == other.distance_
                              && duration_ == other.duration_
                              && distanceCost_ == other.distanceCost_
                              && durationCost_ == other.durationCost_
                              && timeWarp_ == other.timeWarp_
                              && isGroupFeas_ == other.isGroupFeas_
                              && routes_.size() == other.routes_.size()
                              && neighbours_ == other.neighbours_;
    // clang-format on

    if (!attributeChecks)
        return false;

    // The visits are the same for both solutions, but the vehicle
    // assignments need not be. We check this via a mapping from the first
    // client in each route to the vehicle type of that route. We need to
    // base this on the visits since the route order can differ between
    // solutions.
    std::unordered_map<Client, VehicleType> client2vehType;
    for (auto const &route : routes_)
        client2vehType[route[0]] = route.vehicleType();

    for (auto const &route : other.routes_)
        if (client2vehType[route[0]] != route.vehicleType())//路线的车辆类型不同
            return false;

    return true;
}

// Solution随机构造函数：创建随机生成的解
Solution::Solution(ProblemData const &data, RandomNumberGenerator &rng)
    : neighbours_(data.numLocations(), std::nullopt)  // 初始化邻居列表
{
    // Add all required and randomly selected optional clients.
    // 添加所有必需的客户和随机选择的可选客户
    std::vector<size_t> clients;
    clients.reserve(data.numClients());
    for (size_t idx = data.numDepots(); idx != data.numLocations(); ++idx)//遍历客户
    {
        ProblemData::Client const &clientData = data.location(idx);
        // 如果客户是必需的，或者随机数小于0.5，则添加到客户列表
        if (clientData.required || rng.rand() < 0.5)
            clients.push_back(idx);
    }

    // Shuffle clients to create random routes.
    // 打乱客户顺序以创建随机路线
    rng.shuffle(clients.begin(), clients.end());

    // Distribute clients evenly over the routes: the total number of
    // clients per vehicle, with an adjustment in case the division is not
    // perfect and there are not enough vehicles for single-client routes.
    // 将客户均匀分配到路线：每辆车的客户总数，如果除法不完美且没有足够的车辆用于单客户路线，则进行调整
    auto const numVehicles = data.numVehicles();
    auto const numClients = clients.size();
    auto const perVehicle = std::max<size_t>(numClients / numVehicles, 1);
    auto const adjustment
        = numClients > numVehicles && numClients % numVehicles != 0;
    auto const perRoute = perVehicle + adjustment;
    auto const numRoutes = (numClients + perRoute - 1) / perRoute;

    // 将客户分配到各个路线
    std::vector<std::vector<Client>> routes(numRoutes);
    for (size_t idx = 0; idx != numClients; ++idx)
        routes[idx / perRoute].push_back(clients[idx]);

    // 创建车辆类型列表
    std::vector<size_t> vehTypes;
    vehTypes.reserve(data.numVehicles());
    for (size_t vehType = 0; vehType != data.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data.vehicleType(vehType).numAvailable;
        std::fill_n(std::back_inserter(vehTypes), numAvailable, vehType);
    }

    if (data.numVehicleTypes() > 1)
        // Shuffle vehicle types when there is more than one. This ensures
        // some additional diversity in the initial solutions, which
        // sometimes (e.g. with heterogeneous fleet VRP) matters for
        // consistent convergence.
        // 当有多个车辆类型时打乱车辆类型顺序。这确保了初始解中的一些额外多样性，
        // 有时（例如异构车队VRP）这对一致收敛很重要。
        rng.shuffle(vehTypes.begin(), vehTypes.end());

    // 创建路线对象
    routes_.reserve(numRoutes);
    for (size_t idx = 0; idx != routes.size(); idx++)
        routes_.emplace_back(data, routes[idx], vehTypes[idx]);

    // 使用创建的路线构建最终解
    *this = Solution(data, routes_);
}

// Solution构造函数：从客户访问列表创建解（假设所有路线使用第一种车辆类型）
Solution::Solution(ProblemData const &data,
                   std::vector<std::vector<Client>> const &routes)
{
    Routes transformedRoutes;
    transformedRoutes.reserve(routes.size());
    // 将客户访问列表转换为路线对象
    for (auto const &visits : routes)
        transformedRoutes.emplace_back(data, visits, 0);  // 使用第一种车辆类型（索引0）

    *this = Solution(data, transformedRoutes);
}

// Solution构造函数：从路线列表创建解
Solution::Solution(ProblemData const &data, std::vector<Route> routes)
    : routes_(std::move(routes)), neighbours_(data.numLocations(), std::nullopt)
{
    // 验证路线数量不超过车辆数量
    if (routes_.size() > data.numVehicles())
    {
        auto const msg = "Number of routes must not exceed number of vehicles.";
        throw std::runtime_error(msg);
    }

    // 跟踪已访问的客户和使用的车辆
    DynamicBitset isVisited(data.numLocations());
    std::vector<size_t> usedVehicles(data.numVehicleTypes(), 0);
    for (auto const &route : routes_)
    {
        // 验证路线不为空
        if (route.empty())
            throw std::runtime_error("Solution should not have empty routes.");

        usedVehicles[route.vehicleType()]++;  // 累加使用的车辆数
        // 检查每个客户是否被重复访问
        for (auto const client : route)
        {
            if (isVisited[client])  // client is also visited by an earlier
            {                       // route if this is true
                // 如果客户已被访问，说明被重复访问
                std::ostringstream msg;
                msg << "Client " << client << " is visited more than once.";
                throw std::runtime_error(msg.str());
            }

            isVisited[client] = true;
        }
    }

    // 统计缺失的必需客户
    for (size_t client = data.numDepots(); client != data.numLocations();
         ++client)
        if (!isVisited[client])  // we need to check if the client visit
        {                        // is required if this is true
            // 如果客户未被访问，检查是否为必需客户
            ProblemData::Client const &clientData = data.location(client);
            numMissingClients_ += clientData.required;
        }

    // 检查客户组约束的可行性
    for (auto const &group : data.groups())
    {
        // The solution is feasible w.r.t. this client group if exactly one
        // of the clients in the group is in the solution. When the group is
        // not required, we relax this to at most one client.
        // 解关于此客户组可行，如果组中恰好有一个客户在解中。当组不是必需的时，我们放宽为最多一个客户。
        assert(group.mutuallyExclusive);
        auto const inSol = [&](auto client) { return isVisited[client]; };
        auto const numInSol = std::count_if(group.begin(), group.end(), inSol);//客户组中在解中的客户数量
        isGroupFeas_ &= group.required ? numInSol == 1 : numInSol <= 1;
    }

    // 验证使用的车辆数不超过可用车辆数
    for (size_t vehType = 0; vehType != data.numVehicleTypes(); vehType++)
        if (usedVehicles[vehType] > data.vehicleType(vehType).numAvailable)
        {
            std::ostringstream msg;
            auto const numAvailable = data.vehicleType(vehType).numAvailable;
            msg << "Used more than " << numAvailable << " vehicles of type "
                << vehType << '.';
            throw std::runtime_error(msg.str());
        }

    // 构建邻居关系并评估解
    makeNeighbours();
    evaluate(data);
}

Solution::Solution(size_t numClients,
                   size_t numMissingClients,
                   Distance distance,
                   Cost distanceCost,
                   Duration duration,
                   Duration overtime,
                   Cost durationCost,
                   Distance excessDistance,
                   std::vector<Load> excessLoad,
                   Cost fixedVehicleCost,
                   Cost prizes,
                   Cost uncollectedPrizes,
                   Duration timeWarp,
                   bool isGroupFeasible,
                   Routes routes,
                   Neighbours neighbours)
    : numClients_(numClients),
      numMissingClients_(numMissingClients),
      distance_(distance),
      distanceCost_(distanceCost),
      duration_(duration),
      overtime_(overtime),
      durationCost_(durationCost),
      excessDistance_(excessDistance),
      excessLoad_(std::move(excessLoad)),
      fixedVehicleCost_(fixedVehicleCost),
      prizes_(prizes),
      uncollectedPrizes_(uncollectedPrizes),
      timeWarp_(timeWarp),
      isGroupFeas_(isGroupFeasible),
      routes_(std::move(routes)),
      neighbours_(std::move(neighbours))
{
}

std::ostream &operator<<(std::ostream &out, Solution const &sol)
{
    auto const &routes = sol.routes();

    for (size_t idx = 0; idx != routes.size(); ++idx)
        out << "Route #" << idx + 1 << ": " << routes[idx] << '\n';

    return out;
}
