#include "Route.h"

#include <cmath>
#include <numbers>
#include <ostream>
#include <utility>

using pyvrp::search::Route;

// Node 构造函数，初始化节点位置，其他成员设为默认值
Route::Node::Node(size_t loc) : loc_(loc), idx_(0), trip_(0), route_(nullptr) {}

// 将节点分配到指定路线的特定位置和行程
void Route::Node::assign(Route *route, size_t idx, size_t trip)
{
    idx_ = idx;
    trip_ = trip;
    route_ = route;
}

// 取消节点的分配，重置为未分配状态
void Route::Node::unassign()
{
    idx_ = 0;
    trip_ = 0;
    route_ = nullptr;
}

// Iterator 构造函数，传入节点向量和起始索引
Route::Iterator::Iterator(std::vector<Node *> const &nodes, size_t idx)
    : nodes_(&nodes), idx_(idx)
{
    ensureValidIndex();
}

// 确保迭代器索引有效，跳过中间的重载站点（reload depot）
void Route::Iterator::ensureValidIndex()
{
    // size() - 1 is the index of the end depot, and what's returned by
    // Route::end() - we must not exceed it.
    while (idx_ < nodes_->size() - 1 && operator*() -> isReloadDepot())
        idx_++;  // skip any intermediate reload depots // 跳过中间的重载站点

    assert(0 < idx_ && idx_ < nodes_->size());
}

// 迭代器相等性比较
bool Route::Iterator::operator==(Iterator const &other) const
{
    return nodes_ == other.nodes_ && idx_ == other.idx_;
}

// 解引用迭代器，返回当前节点指针
Route::Node *Route::Iterator::operator*() const { return (*nodes_)[idx_]; }

// 后置递增运算符
Route::Iterator Route::Iterator::operator++(int)
{
    auto tmp = *this;
    ++*this;
    return tmp;
}

// 前置递增运算符，递增后确保索引有效
Route::Iterator &Route::Iterator::operator++()
{
    idx_++;
    ensureValidIndex();
    return *this;
}

// Route 构造函数，初始化路线数据
Route::Route(ProblemData const &data, size_t idx, size_t vehicleType)
    : data(data),
      vehicleType_(data.vehicleType(vehicleType)),  // 获取车辆类型
      idx_(idx),
      loadAt(data.numLoadDimensions()),      // 每个维度的在节点处的负载
      loadAfter(data.numLoadDimensions()),   // 每个维度的节点后负载
      loadBefore(data.numLoadDimensions()),  // 每个维度的节点前负载
      load_(data.numLoadDimensions()),       // 总负载
      excessLoad_(data.numLoadDimensions())  // 超载量
{
    clear();  // 初始化时清空路线
}

Route::~Route() { clear(); }  // 析构时清空路线

// 返回路线中第一个客户节点的迭代器（跳过起始站点）
Route::Iterator Route::begin() const { return Iterator(nodes, 1); }

// 返回路线中结束站点的迭代器（最后一个位置）
Route::Iterator Route::end() const { return Iterator(nodes, nodes.size() - 1); }

// 返回路线的质心坐标（用于几何计算）
std::pair<pyvrp::Coordinate, pyvrp::Coordinate> const &Route::centroid() const
{
    assert(!dirty);  // 确保数据是最新的
    return centroid_;
}

// 返回车辆类型索引
size_t Route::vehicleType() const
{
    auto const &vehicleTypes = data.vehicleTypes();
    return std::distance(&vehicleTypes[0], &vehicleType_);
}

// 判断两条路线是否在角度上重叠（用于聚类分析）
bool Route::overlapsWith(Route const &other, double tolerance) const
{
    assert(!dirty && !other.dirty);  // 确保两条路线数据都是最新的

    auto const [dX, dY] = data.centroid();  // 数据集的质心
    auto const [tX, tY] = this->centroid_;  // 当前路线质心
    auto const [oX, oY] = other.centroid_;  // 另一条路线质心

    // 计算从数据集质心到两个路线质心的角度
    auto const thisAngle = std::atan2((tY - dY).get(), (tX - dX).get());
    auto const otherAngle = std::atan2((oY - dY).get(), (oX - dX).get());
    auto const absDiff = std::abs(thisAngle - otherAngle);  // 角度差的绝对值

    // 判断角度差是否在容忍范围内（考虑周期边界）
    auto constexpr tau = 2 * std::numbers::pi;
    return absDiff <= tolerance * tau || absDiff >= (1 - tolerance) * tau;
}

// 清空路线，重置为只有起始和结束站点
void Route::clear()
{
    if (nodes.size() == 2)  // then the route is already empty and we have
        return;             // nothing to do. // 如果已经是空路线则直接返回

    // 遍历所有节点，如果节点属于当前路线则取消分配
    for (auto *node : nodes)        // only unassign if in route; node may not
        if (node->route() == this)  // be if it's been assigned to another route
            node->unassign();       // while loading a new solution into the LS

    nodes.clear();    // 清空节点向量
    depots_.clear();  // 清空站点向量

    // 添加起始和结束站点
    depots_.emplace_back(vehicleType_.startDepot);
    depots_.emplace_back(vehicleType_.endDepot);

    // 将起始和结束站点分配到路线中
    for (size_t idx : {0, 1})
    {
        nodes.push_back(&depots_[idx]);
        depots_[idx].assign(this, idx, idx);
    }

    update();   // 更新路线数据
    assert(empty());  // 确保路线为空（只有两个站点）
}

// 预留节点向量空间
void Route::reserve(size_t size) { nodes.reserve(size); }

// 在指定索引处插入节点（update在调用这个函数的位置后，而不是在这个函数中）
void Route::insert(size_t idx, Node *node)
{
    assert(0 < idx && idx < nodes.size());  // 确保索引有效（不在边界）
    auto const isDepot = node->client() < data.numDepots();  // 检查是否是站点

    if (isDepot)  // is depot, so we need to insert a copy into our own memory
    {
        // 如果是站点，需要在本地内存中插入副本,创建站点副本的核心原因是确保每个Route对象对自己路线中的
        // 站点节点有完全的所有权和控制权，避免：跨路线状态干扰、悬垂指针问题、复杂的内存管理。
        // 如果depots_容量不足，需要重新分配
        if (depots_.size() == depots_.capacity())  // then we reallocate and
        {                                          // must update references
            depots_.reserve(depots_.size() + 1);
            // 重新分配内存后，原来nodes中的指针会失效,更新节点向量中的指针
            for (auto &depot : depots_)
                nodes[depot.idx()] = &depot;
        }

        node = &depots_.emplace_back(node->client());  // 创建站点副本
    }

    // 检查行程数是否超过限制
    if (numTrips() > maxTrips())
        throw std::invalid_argument("Vehicle cannot perform this many trips.");

    // 插入节点
    nodes.insert(nodes.begin() + idx, node);
    node->assign(this, idx, nodes[idx - 1]->trip());  // 分配节点到当前路线

    // 更新插入位置之后所有节点的索引（和行程索引，如果是站点）
    for (size_t after = idx; after != nodes.size(); ++after)
    {
        nodes[after]->idx_ = after;
        if (isDepot)  // then we need to bump each following trip index
            nodes[after]->trip_++;
    }
}

// 在路线末尾（结束站点之前）插入节点
void Route::push_back(Node *node) { insert(nodes.size() - 1, node); }

// 移除指定索引处的节点
void Route::remove(size_t idx)
{
    // 确保索引有效（不是起始或结束站点）且节点属于当前路线
    assert(0 < idx && idx < nodes.size() - 1);  // is not start or end depot
    assert(nodes[idx]->route() == this);        // must be in this route
    auto const isDepot = nodes[idx]->isReloadDepot();  // 检查是否是重载站点

    if (isDepot)
    {
        // 如果是重载站点，从 depots_ 向量中删除并更新引用
        auto const depotIdx = std::distance(depots_.data(), nodes[idx]);
        auto it = depots_.erase(depots_.begin() + depotIdx);
        // 更新被删除站点之后所有站点的节点向量指针
        for (; it != depots_.end(); ++it)
            nodes[it->idx()] = &*it;
    }
    else
        // 如果不是站点，只需取消节点的分配
        nodes[idx]->unassign();

    // 从节点向量中删除节点
    nodes.erase(nodes.begin() + idx);  // remove dangling pointer
    // 更新删除位置之后所有节点的索引（和行程索引，如果是站点）
    for (auto after = idx; after != nodes.size(); ++after)
    {
        nodes[after]->idx_ = after;
        if (isDepot)  // then we need to decrease each following trip index
            nodes[after]->trip_--;
    }

#ifndef NDEBUG
    dirty = true;  // 在调试模式下标记数据需要更新
#endif
}

// 交换两个节点在各自路线中的位置（仅限客户节点）
void Route::swap(Node *first, Node *second)
{
    assert(!first->isDepot() && !second->isDepot());  // 确保不是站点

    // TODO specialise std::swap for Node
    // 交换两个节点在各自路线节点向量中的位置
    if (first->route_)
        first->route_->nodes[first->idx_] = second;

    if (second->route_)
        second->route_->nodes[second->idx_] = first;

    // 交换节点的路线、索引和行程信息
    std::swap(first->route_, second->route_);
    std::swap(first->idx_, second->idx_);
    std::swap(first->trip_, second->trip_);

#ifndef NDEBUG
    // 在调试模式下标记两条路线的数据需要更新
    if (first->route_)
        first->route_->dirty = true;

    if (second->route_)
        second->route_->dirty = true;
#endif
}

// 更新路线的所有计算数据（距离、时间、负载等）在交换节点/更改解决方案后调用 O(N*D)
//TODO:每次执行移动操作后都调用update，时间复杂度较高，但评估远多于应用，评估时间O(k)，k为涉及的段数
//有没有可能批量更新，先执行多个移动操作，然后一次性更新？
void Route::update()
{
    // 1. 更新访问序列
    visits.clear();
    for (auto const *node : nodes)
        visits.emplace_back(node->client());

    // 2. 计算质心（仅客户节点）
    centroid_ = {0, 0};
    for (auto const *node : nodes)
    {
        if (node->isDepot())
            continue;

        ProblemData::Client const &clientData = data.location(node->client());
        centroid_.first += static_cast<double>(clientData.x) / numClients();
        centroid_.second += static_cast<double>(clientData.y) / numClients();
    }

    // 3. 距离计算
    auto const &distMat = data.distanceMatrix(profile());

    cumDist.resize(nodes.size());
    cumDist[0] = 0;
    for (size_t idx = 1; idx != nodes.size(); ++idx)
        cumDist[idx] = cumDist[idx - 1] + distMat(visits[idx - 1], visits[idx]);

    // 4. 时间计算
    durAt.resize(nodes.size());

    // 起始站点的时间段
    ProblemData::Depot const &start = data.location(startDepot());
    DurationSegment const vehStart(vehicleType_, vehicleType_.startLate); //  创建车辆开始时间的时间段，使用车辆类型的最晚开始时间
    DurationSegment const depotStart(start); //  创建仓库开始时间的时间段
    durAt[0] = DurationSegment::merge(0, vehStart, depotStart);

    // 结束站点的时间段
    ProblemData::Depot const &end = data.location(endDepot());
    DurationSegment const depotEnd(end);
    DurationSegment const vehEnd(vehicleType_, vehicleType_.twLate);
    durAt[nodes.size() - 1] = DurationSegment::merge(0, depotEnd, vehEnd);

    // 中间节点（客户或重载站点）的时间段
    for (size_t idx = 1; idx != nodes.size() - 1; ++idx)
    {
        auto const *node = nodes[idx];

        if (!node->isReloadDepot())//是客户结点
        {
            ProblemData::Client const &client = data.location(node->client());
            durAt[idx] = {client};
        }
        else
        {
            ProblemData::Depot const &depot = data.location(node->client());
            durAt[idx] = {depot};
        }
    }

    // 计算每个节点的前向和后向累积时间
    auto const &durations = data.durationMatrix(profile());

    durBefore.resize(nodes.size());
    durBefore[0] = durAt[0];
    for (size_t idx = 1; idx != nodes.size(); ++idx)
    {
        auto const prev = idx - 1;
        auto const before = nodes[prev]->isReloadDepot()
                                ? durBefore[prev].finaliseBack()//重载站点，新行程
                                : durBefore[prev];

        auto const edgeDur = durations(visits[prev], visits[idx]);
        durBefore[idx] = DurationSegment::merge(edgeDur, before, durAt[idx]);
    }

    durAfter.resize(nodes.size());
    durAfter[nodes.size() - 1] = durAt[nodes.size() - 1];
    for (size_t next = nodes.size() - 1; next != 0; --next)
    {
        auto const idx = next - 1;
        auto const after = nodes[next]->isReloadDepot()
                               ? durAfter[next].finaliseFront()
                               : durAfter[next];

        auto const edgeDur = durations(visits[idx], visits[next]);
        durAfter[idx] = DurationSegment::merge(edgeDur, durAt[idx], after);
    }

    // 5. 负载计算（多个维度）支持多维度车辆路径问题 Multi-Dimensional VRP
    for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
    {
        auto const capacity = vehicleType_.capacity[dim];  // 当前维度的容量

        loadAt[dim].resize(nodes.size());
        loadAt[dim][0] = {vehicleType_, dim};  // 初始负载
        loadAt[dim][nodes.size() - 1] = {};

        // 设置每个节点的负载段（重载站点为空）
        for (size_t idx = 1; idx != nodes.size() - 1; ++idx)
            loadAt[dim][idx]
                = nodes[idx]->isReloadDepot()
                      ? LoadSegment{}
                      : LoadSegment{data.location(visits[idx]), dim};

        // 前向累积负载
        loadBefore[dim].resize(nodes.size());
        loadBefore[dim][0] = loadAt[dim][0];
        for (size_t idx = 1; idx != nodes.size(); ++idx)
        {
            auto const prev = idx - 1;
            if (nodes[prev]->isReloadDepot())  // 前一个节点是重载站点则重置
                loadBefore[dim][idx] = LoadSegment::merge(
                    loadBefore[dim][prev].finalise(capacity), loadAt[dim][idx]);
            else
                loadBefore[dim][idx] = LoadSegment::merge(loadBefore[dim][prev],
                                                          loadAt[dim][idx]);
        }

        // 计算总负载和超载量
        load_[dim] = 0;
        excessLoad_[dim]
            = loadBefore[dim][nodes.size() - 1].excessLoad(capacity);
        for (auto it = depots_.begin() + 1; it != depots_.end(); ++it)
            load_[dim] += loadBefore[dim][it->idx()].load();//每个重载站点的前向累积负载

        // 后向累积负载
        loadAfter[dim].resize(nodes.size());
        loadAfter[dim][nodes.size() - 1] = loadAt[dim][nodes.size() - 1];
        for (size_t idx = nodes.size() - 1; idx != 0; --idx)
        {
            auto const prev = idx - 1;
            if (nodes[idx]->isReloadDepot())  // 当前节点是重载站点则重置
                loadAfter[dim][prev] = LoadSegment::merge(
                    loadAt[dim][prev], loadAfter[dim][idx].finalise(capacity));
            else
                loadAfter[dim][prev] = LoadSegment::merge(loadAt[dim][prev],
                                                          loadAfter[dim][idx]);
        }
    }

    // 6. 缓存常用的成本分量
    distance_ = cumDist.back();
    excessDistance_ = std::max<Distance>(distance_ - maxDistance(), 0);
    distanceCost_ = unitDistanceCost() * static_cast<Cost>(distance_);

    duration_ = durAfter[0].duration();
    timeWarp_ = durAfter[0].timeWarp(maxDuration());

    auto const overtime = std::max<Duration>(duration_ - shiftDuration(), 0);
    durationCost_ = unitDurationCost() * static_cast<Cost>(duration_)
                    + unitOvertimeCost() * static_cast<Cost>(overtime);

#ifndef NDEBUG
    dirty = false;  // 在调试模式下标记数据已更新
#endif
}

// 比较两条路线是否相等（比较关键属性和访问序列）
bool Route::operator==(Route const &other) const
{
    assert(!dirty && !other.dirty);  // 确保数据是最新的

    // First compare simple attributes, since that's a quick and cheap check.
    // Only when these are the same we test if the visits are all equal.
    // 首先比较简单属性（快速检查），如果相同再比较访问序列
    // clang-format off
    return distance_ == other.distance_
        && duration_ == other.duration_
        && timeWarp_ == other.timeWarp_
        && vehicleType_ == other.vehicleType_
        && visits == other.visits;
    // clang-format on
}

// 与 pyvrp::Route 类型的路线比较
bool Route::operator==(pyvrp::Route const &other) const
{
    assert(!dirty);  // 确保数据是最新的

    // 简单属性比较
    // clang-format off
    bool const simpleChecks = distance_ == other.distance()
                              && duration_ == other.duration()
                              && timeWarp_ == other.timeWarp()
                              && vehicleType() == other.vehicleType()
                              && numTrips() == other.numTrips()
                              && numClients() == other.size();
    // clang-format on

    if (!simpleChecks)
        return false;

    // 详细比较每个行程和访问序列
    size_t idx = 0;
    for (auto const &trip : other.trips())
    {
        if (trip.startDepot() != visits[idx++])
            return false;  // not the same reload depot // 重载站点不同

        for (auto const visit : trip)
            if (visit != visits[idx++])
                return false;
    }

    return true;
}

// 输出路线的字符串表示（用于调试和日志）
std::ostream &operator<<(std::ostream &out, Route const &route)
{
    for (size_t idx = 1; idx != route.size() - 1; ++idx)
    {
        if (idx != 1)
            out << ' ';

        if (route[idx]->isReloadDepot())
            out << '|';  // 用 | 表示重载站点
        else
            out << *route[idx];
    }

    return out;
}

// 输出节点的字符串表示（节点编号）
std::ostream &operator<<(std::ostream &out, Route::Node const &node)
{
    return out << node.client();
}
