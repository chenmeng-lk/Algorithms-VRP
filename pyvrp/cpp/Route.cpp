#include "Route.h"
#include "DurationSegment.h"
#include "LoadSegment.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <numeric>

using pyvrp::Coordinate;
using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;
using pyvrp::Route;
using pyvrp::Trip;

using Client = size_t;

// Route类迭代器的构造函数，用于在路线中定位特定索引位置
Route::Iterator::Iterator(Route const &route, size_t idx)
    : route_(&route), trip_(route.numTrips()), idx_(0)
{
    assert(idx <= route.size());

    auto const &trips = route.trips();
    for (size_t trip = 0; trip != trips.size(); ++trip)
    {
        if (idx < trips[trip].size())
        {
            trip_ = trip;
            idx_ = idx;
            break;
        }

        idx -= trips[trip].size();
    }
}

// 迭代器相等性比较运算符
bool Route::Iterator::operator==(Iterator const &other) const
{
    return route_ == other.route_ && trip_ == other.trip_ && idx_ == other.idx_;
}

// 解引用运算符，返回当前迭代器指向的客户索引
Client Route::Iterator::operator*() const
{
    auto const &trips = route_->trips();
    assert(trip_ < trips.size());
    assert(idx_ < trips[trip_].size());

    return trips[trip_][idx_];
}

// 后置递增运算符
Route::Iterator Route::Iterator::operator++(int)
{
    auto tmp = *this;
    ++*this;
    return tmp;
}

// 前置递增运算符
Route::Iterator &Route::Iterator::operator++()
{
    auto const &trips = route_->trips();
    if (idx_ + 1 < trips[trip_].size())
    {
        ++idx_;
        return *this;
    }

    // 然后我们移动到下一个行程。这个行程可能为空 - 在这种情况下
    // 我们继续前进直到耗尽所有行程，或者找到一个非空行程
    ++trip_;
    while (trip_ < trips.size() && trips[trip_].empty())
        ++trip_;

    idx_ = 0;
    return *this;
}

// ScheduledVisit结构体的构造函数
Route::ScheduledVisit::ScheduledVisit(size_t location,
                                      size_t trip,
                                      Duration startService,
                                      Duration endService,
                                      Duration waitDuration,
                                      Duration timeWarp)
    : location(location),
      trip(trip),
      startService(startService),
      endService(endService),
      waitDuration(waitDuration),
      timeWarp(timeWarp)
{
    assert(startService <= endService);
}

// 计算服务持续时间
Duration Route::ScheduledVisit::serviceDuration() const
{
    return endService - startService;
}

// 验证路线的一致性
void Route::validate(ProblemData const &data) const
{
    auto const &vehData = data.vehicleType(vehicleType_);

    // 验证行程数量不超过车辆类型的最大行程数
    if (trips_.size() > vehData.maxTrips())
        throw std::invalid_argument("Vehicle cannot perform this many trips.");

    // 验证第一个行程的起始仓库必须匹配路线的起始仓库
    if (trips_[0].startDepot() != startDepot_)
    {
        auto const *msg = "Route must start at vehicle's start_depot.";
        throw std::invalid_argument(msg);
    }

    // 验证最后一个行程的结束仓库必须匹配路线的结束仓库
    if (trips_.back().endDepot() != endDepot_)
        throw std::invalid_argument("Route must end at vehicle's end_depot.");

    // 验证每个行程必须使用路线的车辆类型
    for (auto const &trip : trips_)
        if (trip.vehicleType() != vehicleType_)
        {
            auto const *msg = "Each trip must use the route's vehicle type.";
            throw std::invalid_argument(msg);
        }

    // 验证连续行程：前一个行程的结束仓库必须等于下一个行程的起始仓库
    for (size_t idx = 0; idx + 1 != trips_.size(); ++idx)
        if (trips_[idx].endDepot() != trips_[idx + 1].startDepot())
        {
            auto *msg = "Consecutive trips must start at previous' end_depot.";
            throw std::invalid_argument(msg);
        }
}

// 创建路线的计划数据：计算每个访问的时间安排
void Route::makeSchedule(ProblemData const &data)
{
    schedule_.clear();
    schedule_.reserve(size() + numTrips() + 1);  // clients and depots 客户和仓库

    auto const &vehData = data.vehicleType(vehicleType_);
    auto const &durations = data.durationMatrix(vehData.profile);  // 获取持续时间矩阵

    auto now = startTime_;  // 当前时间从路线开始时间开始
    // 处理访问的lambda函数：计算等待时间、时间扭曲和服务时间
    auto const handle
        = [&](auto const &where, size_t location, size_t trip, Duration service)
    {
        // 计算等待时间（如果早到）和时间扭曲（如果晚到）
        auto const wait = std::max<Duration>(where.twEarly - now, 0);  // 等待时间
        auto const tw = std::max<Duration>(now - where.twLate, 0);    // 时间扭曲

        now += wait;  // 加上等待时间
        now -= tw;    // 减去时间扭曲（时间倒流）

        // 添加计划访问
        schedule_.emplace_back(location, trip, now, now + service, wait, tw);

        now += service;  // 加上服务时间
    };

    for (size_t tripIdx = 0; tripIdx != trips_.size(); ++tripIdx)
    {
        auto const &trip = trips_[tripIdx];
        ProblemData::Depot const &start = data.location(trip.startDepot());

        auto const earliestStart = std::max(
            start.twEarly, std::min(trip.releaseTime(), start.twLate));
        auto const latestStart = tripIdx == 0  // first trip also accounts for
                                               // the latest start constraint
                                     ? std::min(start.twLate, vehData.startLate)
                                     : start.twLate;

        auto const wait = std::max<Duration>(earliestStart - now, 0);
        auto const tw = std::max<Duration>(now - latestStart, 0);

        now += wait;
        now -= tw;

        schedule_.emplace_back(trip.startDepot(), tripIdx, now, now, wait, tw);

        size_t prevClient = trip.startDepot();
        for (auto const client : trip)
        {
            now += durations(prevClient, client);

            ProblemData::Client const &clientData = data.location(client);
            handle(clientData, client, tripIdx, clientData.serviceDuration);

            prevClient = client;
        }

        now += durations(prevClient, trip.endDepot());
    }

    ProblemData::Depot const &end = data.location(endDepot_);
    handle(end, endDepot_, numTrips(), 0);
}

// 构造函数：使用单个行程的访问列表创建路线
Route::Route(ProblemData const &data, Visits visits, size_t vehicleType)
    : Route(data, {{data, std::move(visits), vehicleType}}, vehicleType)
{
}

// 主要构造函数：使用多个行程创建路线
Route::Route(ProblemData const &data, Trips trips, size_t vehType)
    : trips_(std::move(trips)),
      delivery_(data.numLoadDimensions(), 0),
      pickup_(data.numLoadDimensions(), 0),
      excessLoad_(data.numLoadDimensions(), 0),
      vehicleType_(vehType)
{
    if (trips_.empty())  // then we insert a dummy trip for ease. 如果行程为空，则插入一个虚拟行程以便处理
        trips_.emplace_back(data, Visits{}, vehType);

    auto const &vehData = data.vehicleType(vehType);
    startDepot_ = vehData.startDepot;  // 设置起始仓库
    endDepot_ = vehData.endDepot;      // 设置结束仓库

    validate(data);  // 验证路线的一致性

    for (auto const &trip : trips_)  // general statistics 收集一般统计信息
    {
        distance_ += trip.distance();        // 累计距离
        service_ += trip.serviceDuration();  // 累计服务时间
        travel_ += trip.travelDuration();    // 累计行驶时间
        prizes_ += trip.prizes();            // 累计奖励

        auto const [x, y] = trip.centroid();
        auto const numClients = empty() ? 1 : size();  // avoid division by zero 避免除以零
        centroid_.first += (x.get() * trip.size()) / numClients;    // 计算质心横坐标
        centroid_.second += (y.get() * trip.size()) / numClients;   // 计算质心纵坐标
    }

    distanceCost_ = vehData.unitDistanceCost * static_cast<Cost>(distance_);  // 计算距离成本
    excessDistance_ = std::max<Distance>(distance_ - vehData.maxDistance, 0);  // 计算超额距离

    for (size_t idx = 0; idx != trips_.size(); ++idx)  // load statistics 负载统计
    {
        auto const &trip = trips_[idx];
        auto const &tripDeliv = trip.delivery();  // 当前行程的配送量
        auto const &tripPick = trip.pickup();     // 当前行程的提取量
        auto const &tripLoad = trip.load();       // 当前行程的负载

        for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)  // 遍历所有负载维度
        {
            LoadSegment ls = {tripDeliv[dim], tripPick[dim], tripLoad[dim], 0};  // 创建负载段

            if (idx == 0 && vehData.initialLoad[dim] > 0)  // 如果是第一个行程且存在初始负载
                // This is initial load that the first trip does not know about
                // that we need to account for first.
                // 这是第一个行程不知道的初始负载，我们需要先考虑
                ls = LoadSegment::merge({vehData, dim}, ls);  // 合并初始负载

            delivery_[dim] += ls.delivery();      // 累计配送量
            pickup_[dim] += ls.pickup();          // 累计提取量
            excessLoad_[dim] += ls.excessLoad(vehData.capacity[dim]);  // 计算超额负载
        }
    }

    // Duration statistics. We iterate in reverse, that is, from the last to
    // the first visit.
    // 持续时间统计。我们反向迭代，即从最后一个访问到第一个访问
    auto const &durations = data.durationMatrix(vehData.profile);  // 获取持续时间矩阵
    DurationSegment ds = {vehData, vehData.twLate};  // 创建持续时间段
    for (auto trip = trips_.rbegin(); trip != trips_.rend(); ++trip)  // 反向遍历行程
    {
        if (trip != trips_.rbegin())  // need to finalise before next trip, 需要在下一个行程之前完成
            ds = ds.finaliseFront();  // unless this is the first one 除非这是第一个

        ProblemData::Depot const &end = data.location(trip->endDepot());  // 获取行程结束仓库
        ds = DurationSegment::merge(0, {end}, ds);  // 合并结束仓库的持续时间段

        size_t nextClient = trip->endDepot();  // 下一个客户是结束仓库
        for (auto it = trip->rbegin(); it != trip->rend(); ++it)  // 反向遍历行程中的客户
        {
            auto const client = *it;
            auto const edgeDuration = durations(client, nextClient);  // 获取边持续时间
            ProblemData::Client const &clientData = data.location(client);  // 获取客户数据

            ds = DurationSegment::merge(edgeDuration, {clientData}, ds);  // 合并客户持续时间段
            nextClient = client;  // 更新下一个客户
        }

        auto const edgeDuration = durations(trip->startDepot(), nextClient);  // 获取起始仓库到第一个客户的边持续时间
        ProblemData::Depot const &start = data.location(trip->startDepot());  // 获取起始仓库
        DurationSegment const depotDS = {start};  // 创建起始仓库持续时间段

        ds = DurationSegment::merge(edgeDuration, depotDS, ds);  // 合并起始仓库持续时间段
    }

    ds = DurationSegment::merge(0, {vehData, vehData.startLate}, ds);  // 合并车辆类型的最晚开始时间约束

    duration_ = ds.duration();  // 获取总持续时间
    overtime_ = std::max<Duration>(duration_ - vehData.shiftDuration, 0);  // 计算加班时间
    durationCost_ = vehData.unitDurationCost * static_cast<Cost>(duration_)  // 计算持续时间成本
                    + vehData.unitOvertimeCost * static_cast<Cost>(overtime_);  // 加上加班成本
    startTime_ = ds.startEarly();  // 获取开始时间
    slack_ = ds.slack();  // 获取松弛时间
    timeWarp_ = ds.timeWarp(vehData.maxDuration);  // 计算时间扭曲

    makeSchedule(data);  // 创建计划时间表
}

// 构造函数：使用已有数据创建路线（用于复制或重构）
Route::Route(Trips trips,
             Distance distance,
             Cost distanceCost,
             Distance excessDistance,
             std::vector<Load> delivery,
             std::vector<Load> pickup,
             std::vector<Load> excessLoad,
             Duration duration,
             Duration overtime,
             Cost durationCost,
             Duration timeWarp,
             Duration travel,
             Duration service,
             Duration startTime,
             Duration slack,
             Cost prizes,
             std::pair<Coordinate, Coordinate> centroid,
             size_t vehicleType,
             size_t startDepot,
             size_t endDepot,
             std::vector<ScheduledVisit> schedule)
    : trips_(std::move(trips)),
      schedule_(std::move(schedule)),
      distance_(distance),
      distanceCost_(distanceCost),
      excessDistance_(excessDistance),
      delivery_(std::move(delivery)),
      pickup_(std::move(pickup)),
      excessLoad_(std::move(excessLoad)),
      duration_(duration),
      overtime_(overtime),
      durationCost_(durationCost),
      timeWarp_(timeWarp),
      travel_(travel),
      service_(service),
      startTime_(startTime),
      slack_(slack),
      prizes_(prizes),
      centroid_(centroid),
      vehicleType_(vehicleType),
      startDepot_(startDepot),
      endDepot_(endDepot)
{
}

// 检查路线是否为空
bool Route::empty() const { return size() == 0; }

// 返回路线中的客户总数
size_t Route::size() const
{
    return std::accumulate(trips_.begin(),
                           trips_.end(),
                           0,
                           [](size_t count, auto const &trip)
                           { return count + trip.size(); });
}

// 返回路线中的行程数量
size_t Route::numTrips() const { return trips_.size(); }

// 下标运算符，通过索引访问客户
Client Route::operator[](size_t idx) const
{
    for (auto const &trip : trips_)
        if (idx < trip.size())
            return trip[idx];
        else
            idx -= trip.size();

    throw std::out_of_range("Index out of range.");
}

// 返回指向路线开始的迭代器
Route::Iterator Route::begin() const { return Iterator(*this, 0); }

// 返回指向路线结束的迭代器
Route::Iterator Route::end() const { return Iterator(*this, size()); }

// 返回路线的所有行程
Route::Trips const &Route::trips() const { return trips_; }

// 返回指定索引的行程
Trip const &Route::trip(size_t idx) const
{
    assert(idx < trips_.size());
    return trips_[idx];
}

// 返回路线的所有访问（客户索引）
Route::Visits Route::visits() const { return {begin(), end()}; }

// 返回路线的计划时间表
std::vector<Route::ScheduledVisit> const &Route::schedule() const
{
    return schedule_;
}

// 返回路线的总距离
Distance Route::distance() const { return distance_; }

// 返回路线的距离成本
Cost Route::distanceCost() const { return distanceCost_; }

// 返回路线的超额距离
Distance Route::excessDistance() const { return excessDistance_; }

// 返回路线的配送量
std::vector<Load> const &Route::delivery() const { return delivery_; }

// 返回路线的提取量
std::vector<Load> const &Route::pickup() const { return pickup_; }

// 返回路线的超额负载
std::vector<Load> const &Route::excessLoad() const { return excessLoad_; }

// 返回路线的总持续时间
Duration Route::duration() const { return duration_; }

// 返回路线的加班时间
Duration Route::overtime() const { return overtime_; }

// 返回路线的持续时间成本
Cost Route::durationCost() const { return durationCost_; }

// 返回路线的总服务时间
Duration Route::serviceDuration() const { return service_; }

// 返回路线的时间扭曲
Duration Route::timeWarp() const { return timeWarp_; }

// 返回路线的等待时间
Duration Route::waitDuration() const { return duration_ - travel_ - service_; }

// 返回路线的行驶时间
Duration Route::travelDuration() const { return travel_; }

// 返回路线的开始时间
Duration Route::startTime() const { return startTime_; }

// 返回路线的结束时间
Duration Route::endTime() const { return startTime_ + duration_ - timeWarp_; }

// 返回路线的松弛时间
Duration Route::slack() const { return slack_; }

// 返回路线的释放时间（第一个行程的释放时间）
Duration Route::releaseTime() const { return trips_[0].releaseTime(); }

// 返回路线的总奖励
Cost Route::prizes() const { return prizes_; }

// 返回路线的质心坐标
std::pair<Coordinate, Coordinate> const &Route::centroid() const
{
    return centroid_;
}

// 返回路线的车辆类型
size_t Route::vehicleType() const { return vehicleType_; }

// 返回路线的起始仓库
size_t Route::startDepot() const { return startDepot_; }

// 返回路线的结束仓库
size_t Route::endDepot() const { return endDepot_; }

// 检查路线是否可行（无超额负载、无时间扭曲、无超额距离）
bool Route::isFeasible() const
{
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance();
}

// 检查路线是否有超额负载
bool Route::hasExcessLoad() const
{
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

// 检查路线是否有超额距离
bool Route::hasExcessDistance() const { return excessDistance_ > 0; }

// 检查路线是否有时间扭曲
bool Route::hasTimeWarp() const { return timeWarp_ > 0; }

// 路线相等性比较运算符
bool Route::operator==(Route const &other) const
{
    // First compare simple attributes, since that's a quick and cheap check.
    // Only when these are the same we test if the visits are all equal.
    // 首先比较简单属性，因为这是快速且廉价的检查。
    // 只有当这些相同时，我们才测试访问是否全部相等。
    // clang-format off
    return distance_ == other.distance_
        && duration_ == other.duration_
        && timeWarp_ == other.timeWarp_
        && vehicleType_ == other.vehicleType_
        && trips_ == other.trips_;
    // clang-format on
}

// 输出流运算符，用于打印路线
std::ostream &operator<<(std::ostream &out, Route const &route)
{
    auto const &trips = route.trips();
    for (size_t idx = 0; idx != trips.size(); ++idx)
    {
        if (idx != 0)
            out << " | ";
        out << trips[idx];
    }

    return out;
}
