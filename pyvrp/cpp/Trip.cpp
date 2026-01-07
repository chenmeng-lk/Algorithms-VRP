#include "Trip.h"
#include "LoadSegment.h"

#include <algorithm>
#include <fstream>

using pyvrp::Coordinate;
using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;
using pyvrp::ProblemData;
using pyvrp::Trip;

namespace
{
// Returns whether given vehicle type can start a trip from the given depot.
// 返回给定车辆类型是否可以从给定仓库开始行程
bool canStartAt(ProblemData::VehicleType const &vehType, size_t depot)
{
    auto const &reloads = vehType.reloadDepots;  // 重新装载仓库列表
    // 检查仓库是否为起始仓库或重新装载仓库
    return depot == vehType.startDepot
           || std::find(reloads.begin(), reloads.end(), depot) != reloads.end();
}

// Returns whether given vehicle type can end a trip at the given depot.
// 返回给定车辆类型是否可以在给定仓库结束行程
bool canEndAt(ProblemData::VehicleType const &vehType, size_t depot)
{
    auto const &reloads = vehType.reloadDepots;  // 重新装载仓库列表
    // 检查仓库是否为结束仓库或重新装载仓库
    return depot == vehType.endDepot
           || std::find(reloads.begin(), reloads.end(), depot) != reloads.end();
}
}  // namespace

// Trip构造函数：初始化行程的所有属性并计算统计信息
Trip::Trip(ProblemData const &data,
           Visits visits,
           size_t vehicleType,
           std::optional<size_t> startDepot,
           std::optional<size_t> endDepot)
    : visits_(std::move(visits)),  // 移动客户访问列表
      delivery_(data.numLoadDimensions(), 0),  // 初始化配送量向量
      pickup_(data.numLoadDimensions(), 0),     // 初始化拾取量向量
      load_(data.numLoadDimensions(), 0),       // 初始化负载向量
      excessLoad_(data.numLoadDimensions(), 0), // 初始化超额负载向量
      vehicleType_(vehicleType)
{
    auto const &vehData = data.vehicleType(vehicleType_);
    // 如果未提供起始/结束仓库，则使用车辆类型的默认值
    startDepot_ = startDepot.value_or(vehData.startDepot);
    endDepot_ = endDepot.value_or(vehData.endDepot);

    // 验证车辆可以从起始仓库开始
    if (!canStartAt(vehData, startDepot_))
        throw std::invalid_argument("Vehicle cannot start from start_depot.");

    // 验证车辆可以在结束仓库结束
    if (!canEndAt(vehData, endDepot_))
        throw std::invalid_argument("Vehicle cannot end at end_depot.");

    // 验证所有客户索引有效
    for (auto const client : visits_)
        if (client < data.numDepots() || client >= data.numLocations())
        {
            std::ostringstream msg;
            msg << "Client " << client << " is not understood.";
            throw std::invalid_argument(msg.str());
        }

    // 获取距离和持续时间矩阵
    auto const &distances = data.distanceMatrix(vehData.profile);
    auto const &durations = data.durationMatrix(vehData.profile);

    // 计算距离、行驶时间、服务时间、释放时间和奖励值
    for (size_t prevClient = startDepot_; auto const client : visits_)
    {
        distance_ += distances(prevClient, client);  // 累加距离
        travel_ += durations(prevClient, client);    // 累加行驶时间

        ProblemData::Client const &clientData = data.location(client);

        service_ += clientData.serviceDuration;  // 累加服务时间
        release_ = std::max(release_, clientData.releaseTime);  // 更新释放时间（取最大值）
        prizes_ += clientData.prize;  // 累加奖励值

        // 计算中心点（累加坐标）
        centroid_.first += static_cast<double>(clientData.x) / size();
        centroid_.second += static_cast<double>(clientData.y) / size();

        prevClient = client;
    }

    // 加上从最后一个客户到结束仓库的距离和行驶时间
    auto const last = empty() ? startDepot_ : visits_.back();
    distance_ += distances(last, endDepot_);
    travel_ += durations(last, endDepot_);

    // 计算每个负载维度的配送量、拾取量、负载和超额负载
    for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
    {
        LoadSegment segment;
        // 合并所有客户的负载段
        for (auto const client : visits_)
        {
            ProblemData::Client const &clientData = data.location(client);
            segment = LoadSegment::merge(segment, {clientData, dim});
        }

        delivery_[dim] = segment.delivery();  // 配送量
        pickup_[dim] = segment.pickup();     // 拾取量
        load_[dim] = segment.load();         // 负载
        excessLoad_[dim] = segment.excessLoad(vehData.capacity[dim]);  // 超额负载
    }
}

Trip::Trip(Visits visits,
           Distance distance,
           std::vector<Load> delivery,
           std::vector<Load> pickup,
           std::vector<Load> load,
           std::vector<Load> excessLoad,
           Duration travel,
           Duration service,
           Duration release,
           Cost prizes,
           std::pair<Coordinate, Coordinate> centroid,
           size_t vehicleType,
           size_t startDepot,
           size_t endDepot)
    : visits_(std::move(visits)),
      distance_(distance),
      delivery_(std::move(delivery)),
      pickup_(std::move(pickup)),
      load_(std::move(load)),
      excessLoad_(std::move(excessLoad)),
      travel_(travel),
      service_(service),
      release_(release),
      prizes_(prizes),
      centroid_(centroid),
      vehicleType_(vehicleType),
      startDepot_(startDepot),
      endDepot_(endDepot)
{
}

bool Trip::empty() const { return visits_.empty(); }

size_t Trip::size() const { return visits_.size(); } // 返回行程中客户访问的数量

Trip::Client Trip::operator[](size_t idx) const { return visits_[idx]; } // 通过索引访问行程中的客户

Trip::Visits::const_iterator Trip::begin() const { return visits_.begin(); } // 返回行程访问列表的开始迭代器

Trip::Visits::const_iterator Trip::end() const { return visits_.end(); } // 返回行程访问列表的结束迭代器

Trip::Visits::const_reverse_iterator Trip::rbegin() const
{
    return visits_.rbegin();
} // 返回行程访问列表的反向开始迭代器

Trip::Visits::const_reverse_iterator Trip::rend() const
{
    return visits_.rend();
} // 返回行程访问列表的反向结束迭代器

Trip::Visits const &Trip::visits() const { return visits_; } // 返回行程的客户访问列表引用

Distance Trip::distance() const { return distance_; } // 返回行程的总距离

std::vector<Load> const &Trip::delivery() const { return delivery_; } // 返回行程的配送量向量引用

std::vector<Load> const &Trip::pickup() const { return pickup_; } // 返回行程的拾取量向量引用

std::vector<Load> const &Trip::load() const { return load_; } // 返回行程的负载向量引用

std::vector<Load> const &Trip::excessLoad() const { return excessLoad_; } // 返回行程的超额负载向量引用

Duration Trip::serviceDuration() const { return service_; } // 返回行程的总服务时间

Duration Trip::travelDuration() const { return travel_; } // 返回行程的总行驶时间

Duration Trip::releaseTime() const { return release_; } // 返回行程的释放时间

Cost Trip::prizes() const { return prizes_; } // 返回行程的总奖励值

std::pair<Coordinate, Coordinate> const &Trip::centroid() const
{
    return centroid_;
} // 返回行程的中心点坐标对

size_t Trip::vehicleType() const { return vehicleType_; } // 返回行程使用的车辆类型索引

size_t Trip::startDepot() const { return startDepot_; } // 返回行程的起始仓库索引

size_t Trip::endDepot() const { return endDepot_; } // 返回行程的结束仓库索引

bool Trip::hasExcessLoad() const
{
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
} // 检查行程是否有任何维度的超额负载

bool Trip::operator==(Trip const &other) const// 比较两个行程是否相等
{
    // First compare simple attributes, since that's a quick and cheap check.
    // Only when these are the same we test if the visits are all equal.
    // clang-format off
    return distance_ == other.distance_
        && travel_ == other.travel_
        && service_ == other.service_
        && startDepot_ == other.startDepot_  // might vary between trips
        && endDepot_ == other.endDepot_      // might vary between trips
        && vehicleType_ == other.vehicleType_
        && visits_ == other.visits_;
    // clang-format on
}

std::ostream &operator<<(std::ostream &out, Trip const &trip) // 将行程的客户访问列表输出到流中
{
    for (size_t idx = 0; idx != trip.size(); ++idx)
    {
        if (idx != 0)
            out << ' ';
        out << trip[idx];
    }

    return out;
}
