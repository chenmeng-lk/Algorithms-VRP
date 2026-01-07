#include "ProblemData.h"

#include <algorithm>
#include <cstring>
#include <numeric>
#include <stdexcept>

using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;
using pyvrp::Matrix;
using pyvrp::ProblemData;
//存储 VRP 问题的所有数据
namespace
{
// Small local helper for what is essentially strdup() from the C23 standard,
// which my compiler does not (yet) have. See here for the actual recipe:
// https://stackoverflow.com/a/252802/4316405 (modified to use new instead of
// malloc). We do all this so we can use C-style strings, rather than C++'s
// std::string, which are much larger objects.
// 字符串复制辅助函数，用于复制C风格字符串（类似C23标准的strdup）
static char *duplicate(char const *src)
{
    char *dst = new char[std::strlen(src) + 1];  // space for src + null 为源字符串和空字符分配空间
    std::strcpy(dst, src);
    return dst;
}

// Pad vec1 with zeroes to the size of vec1 and vec2, whichever is larger.
// 将vec1用零填充到vec1和vec2中较大的那个大小
auto &pad(auto &vec1, auto const &vec2)
{
    vec1.resize(std::max(vec1.size(), vec2.size()));
    return vec1;
}

bool isNegative(auto value) { return value < 0; }  // 判断值是否为负数

// Small helper that determines if the time windows of a and b overlap.
// 判断a和b的时间窗是否重叠的辅助函数
bool hasTimeOverlap(auto const &a, auto const &b)
{
    // See https://stackoverflow.com/a/325964/4316405.
    return a.twEarly <= b.twLate && a.twLate >= b.twEarly;
}

// 判断参数是否有时间窗约束
bool hasTimeWindow(auto const &arg)
{
    auto const hasTw = arg.twEarly != 0
                       || arg.twLate != std::numeric_limits<Duration>::max();

    if constexpr (requires { arg.startLate; })
        return hasTw || arg.startLate != std::numeric_limits<Duration>::max();

    return hasTw;
}
}  // namespace

// Client构造函数：初始化客户的所有属性并进行验证
ProblemData::Client::Client(Coordinate x,
                            Coordinate y,
                            std::vector<Load> delivery,
                            std::vector<Load> pickup,
                            Duration serviceDuration,
                            Duration twEarly,
                            Duration twLate,
                            Duration releaseTime,
                            Cost prize,
                            bool required,
                            std::optional<size_t> group,
                            std::string name)
    : x(x),
      y(y),
      serviceDuration(serviceDuration),
      twEarly(twEarly),
      twLate(twLate),
      delivery(pad(delivery, pickup)),  // 将delivery和pickup填充到相同大小
      pickup(pad(pickup, delivery)),    // 将pickup和delivery填充到相同大小
      releaseTime(releaseTime),
      prize(prize),
      required(required),
      group(group),
      name(duplicate(name.data()))  // 复制名称字符串
{
    assert(delivery.size() == pickup.size());

    // 验证配送量必须非负
    if (std::any_of(delivery.begin(), delivery.end(), isNegative<Load>))
        throw std::invalid_argument("delivery amounts must be >= 0.");

    // 验证拾取量必须非负
    if (std::any_of(pickup.begin(), pickup.end(), isNegative<Load>))
        throw std::invalid_argument("pickup amounts must be >= 0.");

    // 验证服务时间必须非负
    if (serviceDuration < 0)
        throw std::invalid_argument("service_duration must be >= 0.");

    // 验证时间窗：最早时间必须小于等于最晚时间
    if (twEarly > twLate)
        throw std::invalid_argument("tw_early must be <= tw_late.");

    // 验证最早时间必须非负
    if (twEarly < 0)
        throw std::invalid_argument("tw_early must be >= 0.");

    // 验证释放时间必须小于等于最晚时间
    if (releaseTime > twLate)
        throw std::invalid_argument("release_time must be <= tw_late");

    // 验证释放时间必须非负
    if (releaseTime < 0)
        throw std::invalid_argument("release_time must be >= 0.");

    // 验证奖励必须非负
    if (prize < 0)
        throw std::invalid_argument("prize must be >= 0.");
}

// Client拷贝构造函数：复制客户的所有属性
ProblemData::Client::Client(Client const &client)
    : x(client.x),
      y(client.y),
      serviceDuration(client.serviceDuration),
      twEarly(client.twEarly),
      twLate(client.twLate),
      delivery(client.delivery),
      pickup(client.pickup),
      releaseTime(client.releaseTime),
      prize(client.prize),
      required(client.required),
      group(client.group),
      name(duplicate(client.name))  // 复制名称字符串
{
}

// Client移动构造函数：移动客户的所有属性
ProblemData::Client::Client(Client &&client)
    : x(client.x),
      y(client.y),
      serviceDuration(client.serviceDuration),
      twEarly(client.twEarly),
      twLate(client.twLate),
      delivery(std::move(client.delivery)),  // 移动delivery向量
      pickup(std::move(client.pickup)),      // 移动pickup向量
      releaseTime(client.releaseTime),
      prize(client.prize),
      required(client.required),
      group(client.group),
      name(client.name)  // we can steal 可以窃取名称指针
{
    client.name = nullptr;  // stolen 将原对象的名称指针置空
}

// Client析构函数：释放名称字符串内存
ProblemData::Client::~Client() { delete[] name; }

// Client相等运算符：比较两个客户是否相等
bool ProblemData::Client::operator==(Client const &other) const
{
    // clang-format off
    return x == other.x
        && y == other.y
        && delivery == other.delivery
        && pickup == other.pickup
        && serviceDuration == other.serviceDuration
        && twEarly == other.twEarly
        && twLate == other.twLate
        && releaseTime == other.releaseTime
        && prize == other.prize
        && required == other.required
        && group == other.group
        && std::strcmp(name, other.name) == 0;  // 比较名称字符串
    // clang-format on
}

// ClientGroup构造函数：初始化客户组并添加客户
ProblemData::ClientGroup::ClientGroup(std::vector<size_t> clients,
                                      bool required,
                                      std::string name)
    : required(required), name(duplicate(name.data()))  // 复制名称字符串
{
    for (auto const client : clients)
        addClient(client);  // 添加每个客户到组中
}

// ClientGroup拷贝构造函数：复制客户组的所有属性
ProblemData::ClientGroup::ClientGroup(ClientGroup const &group)
    : clients_(group.clients_),
      required(group.required),
      name(duplicate(group.name))  // 复制名称字符串
{
}

// ClientGroup移动构造函数：移动客户组的所有属性
ProblemData::ClientGroup::ClientGroup(ClientGroup &&group)
    : clients_(std::move(group.clients_)),  // 移动客户列表
      required(group.required),
      name(group.name)  // we can steal 可以窃取名称指针
{
    group.name = nullptr;  // stolen 将原对象的名称指针置空
}

// ClientGroup相等运算符：比较两个客户组是否相等
bool ProblemData::ClientGroup::operator==(ClientGroup const &other) const
{
    // clang-format off
    return clients_ == other.clients_
        && required == other.required
        && mutuallyExclusive == other.mutuallyExclusive
        && std::strcmp(name, other.name) == 0;  // 比较名称字符串
    // clang-format on
}

// ClientGroup析构函数：释放名称字符串内存
ProblemData::ClientGroup::~ClientGroup() { delete[] name; }

// 判断客户组是否为空
bool ProblemData::ClientGroup::empty() const { return clients_.empty(); }

// 返回客户组中的客户数量
size_t ProblemData::ClientGroup::size() const { return clients_.size(); }

// 返回客户组客户列表的起始迭代器
std::vector<size_t>::const_iterator ProblemData::ClientGroup::begin() const
{
    return clients_.begin();
}

// 返回客户组客户列表的结束迭代器
std::vector<size_t>::const_iterator ProblemData::ClientGroup::end() const
{
    return clients_.end();
}

// 返回客户组中的客户列表
std::vector<size_t> const &ProblemData::ClientGroup::clients() const
{
    return clients_;
}

// 向客户组中添加客户
void ProblemData::ClientGroup::addClient(size_t client)
{
    // 检查客户是否已在组中
    if (std::find(clients_.begin(), clients_.end(), client) != clients_.end())
        throw std::invalid_argument("Client already in group.");

    clients_.push_back(client);
}

// 清空客户组中的所有客户
void ProblemData::ClientGroup::clear() { clients_.clear(); }

// Depot构造函数：初始化仓库的所有属性并进行验证
ProblemData::Depot::Depot(Coordinate x,
                          Coordinate y,
                          Duration twEarly,
                          Duration twLate,
                          std::string name)
    : x(x), y(y), twEarly(twEarly), twLate(twLate), name(duplicate(name.data()))  // 复制名称字符串
{
    // 验证时间窗：最早时间必须小于等于最晚时间
    if (twEarly > twLate)
        throw std::invalid_argument("tw_early must be <= tw_late.");

    // 验证最早时间必须非负
    if (twEarly < 0)
        throw std::invalid_argument("tw_early must be >= 0.");
}

// Depot拷贝构造函数：复制仓库的所有属性
ProblemData::Depot::Depot(Depot const &depot)
    : x(depot.x),
      y(depot.y),
      twEarly(depot.twEarly),
      twLate(depot.twLate),
      name(duplicate(depot.name))  // 复制名称字符串
{
}

// Depot移动构造函数：移动仓库的所有属性
ProblemData::Depot::Depot(Depot &&depot)
    : x(depot.x),
      y(depot.y),
      twEarly(depot.twEarly),
      twLate(depot.twLate),
      name(depot.name)  // we can steal 可以窃取名称指针
{
    depot.name = nullptr;  // stolen 将原对象的名称指针置空
}

// Depot析构函数：释放名称字符串内存
ProblemData::Depot::~Depot() { delete[] name; }

// Depot相等运算符：比较两个仓库是否相等
bool ProblemData::Depot::operator==(Depot const &other) const
{
    // clang-format off
    return x == other.x 
        && y == other.y
        && twEarly == other.twEarly
        && twLate == other.twLate
        && std::strcmp(name, other.name) == 0;  // 比较名称字符串
    // clang-format on
}

// VehicleType构造函数：初始化车辆类型的所有属性并进行验证
ProblemData::VehicleType::VehicleType(size_t numAvailable,
                                      std::vector<Load> capacity,
                                      size_t startDepot,
                                      size_t endDepot,
                                      Cost fixedCost,
                                      Duration twEarly,
                                      Duration twLate,
                                      Duration shiftDuration,
                                      Distance maxDistance,
                                      Cost unitDistanceCost,
                                      Cost unitDurationCost,
                                      size_t profile,
                                      std::optional<Duration> startLate,
                                      std::vector<Load> initialLoad,
                                      std::vector<size_t> reloadDepots,
                                      size_t maxReloads,
                                      Duration maxOvertime,
                                      Cost unitOvertimeCost,
                                      std::string name)
    : numAvailable(numAvailable),
      startDepot(startDepot),
      endDepot(endDepot),
      capacity(pad(capacity, initialLoad)),      // 将capacity和initialLoad填充到相同大小
      twEarly(twEarly),
      twLate(twLate),
      shiftDuration(shiftDuration),
      maxDistance(maxDistance),
      fixedCost(fixedCost),
      unitDistanceCost(unitDistanceCost),
      unitDurationCost(unitDurationCost),
      profile(profile),
      startLate(startLate.value_or(twLate)),      // 如果未提供startLate，则使用twLate
      initialLoad(pad(initialLoad, capacity)),    // 将initialLoad和capacity填充到相同大小
      reloadDepots(reloadDepots),
      maxReloads(maxReloads),
      maxOvertime(maxOvertime),
      unitOvertimeCost(unitOvertimeCost),
      // We need to check >= 0 here to avoid overflow. If the arguments are
      // negative the validation checks further below will raise, so it doesn't
      // matter what we set as long as we get to those checks.
      // 计算最大持续时间（包括加班时间），需要检查非负以避免溢出
      maxDuration(shiftDuration >= 0 && maxOvertime >= 0
                          && maxOvertime < std::numeric_limits<Duration>::max()
                                               - shiftDuration
                      ? shiftDuration + maxOvertime
                      : std::numeric_limits<Duration>::max()),
      name(duplicate(name.data()))  // 复制名称字符串
{
    // 验证可用车辆数量必须大于0
    if (numAvailable == 0)
        throw std::invalid_argument("num_available must be > 0.");

    // 验证容量必须非负
    if (std::any_of(capacity.begin(), capacity.end(), isNegative<Load>))
        throw std::invalid_argument("capacity amounts must be >= 0.");

    // 验证时间窗：最早时间必须小于等于最晚开始时间
    if (twEarly > this->startLate)
        throw std::invalid_argument("tw_early must be <= start_late.");

    // 验证最晚开始时间必须小于等于最晚时间
    if (this->startLate > twLate)
        throw std::invalid_argument("start_late must be <= tw_late.");

    // 验证最早时间必须非负
    if (twEarly < 0)
        throw std::invalid_argument("tw_early must be >= 0.");

    // 验证班次持续时间必须非负
    if (shiftDuration < 0)
        throw std::invalid_argument("shift_duration must be >= 0.");

    // 验证最大距离必须非负
    if (maxDistance < 0)
        throw std::invalid_argument("max_distance must be >= 0.");

    // 验证固定成本必须非负
    if (fixedCost < 0)
        throw std::invalid_argument("fixed_cost must be >= 0.");

    // 验证单位距离成本必须非负
    if (unitDistanceCost < 0)
        throw std::invalid_argument("unit_distance_cost must be >= 0.");

    // 验证单位持续时间成本必须非负
    if (unitDurationCost < 0)
        throw std::invalid_argument("unit_duration_cost must be >= 0.");

    // 验证初始负载必须非负
    if (std::any_of(initialLoad.begin(), initialLoad.end(), isNegative<Load>))
        throw std::invalid_argument("initial load amounts must be >= 0.");

    // 验证初始负载不能超过容量
    for (size_t dim = 0; dim != initialLoad.size(); ++dim)
        if (initialLoad[dim] > capacity[dim])
            throw std::invalid_argument("initial load exceeds capacity.");

    // 验证最大加班时间必须非负
    if (maxOvertime < 0)
        throw std::invalid_argument("max_overtime must be >= 0.");

    // 验证单位加班成本必须非负
    if (unitOvertimeCost < 0)
        throw std::invalid_argument("unit_overtime_cost must be >= 0.");
}

// VehicleType拷贝构造函数：复制车辆类型的所有属性
ProblemData::VehicleType::VehicleType(VehicleType const &vehicleType)
    : numAvailable(vehicleType.numAvailable),
      startDepot(vehicleType.startDepot),
      endDepot(vehicleType.endDepot),
      capacity(vehicleType.capacity),
      twEarly(vehicleType.twEarly),
      twLate(vehicleType.twLate),
      shiftDuration(vehicleType.shiftDuration),
      maxDistance(vehicleType.maxDistance),
      fixedCost(vehicleType.fixedCost),
      unitDistanceCost(vehicleType.unitDistanceCost),
      unitDurationCost(vehicleType.unitDurationCost),
      profile(vehicleType.profile),
      startLate(vehicleType.startLate),
      initialLoad(vehicleType.initialLoad),
      reloadDepots(vehicleType.reloadDepots),
      maxReloads(vehicleType.maxReloads),
      maxOvertime(vehicleType.maxOvertime),
      unitOvertimeCost(vehicleType.unitOvertimeCost),
      maxDuration(vehicleType.maxDuration),
      name(duplicate(vehicleType.name))  // 复制名称字符串
{
}

// VehicleType移动构造函数：移动车辆类型的所有属性
ProblemData::VehicleType::VehicleType(VehicleType &&vehicleType)
    : numAvailable(vehicleType.numAvailable),
      startDepot(vehicleType.startDepot),
      endDepot(vehicleType.endDepot),
      capacity(std::move(vehicleType.capacity)),      // 移动capacity向量
      twEarly(vehicleType.twEarly),
      twLate(vehicleType.twLate),
      shiftDuration(vehicleType.shiftDuration),
      maxDistance(vehicleType.maxDistance),
      fixedCost(vehicleType.fixedCost),
      unitDistanceCost(vehicleType.unitDistanceCost),
      unitDurationCost(vehicleType.unitDurationCost),
      profile(vehicleType.profile),
      startLate(vehicleType.startLate),
      initialLoad(std::move(vehicleType.initialLoad)),  // 移动initialLoad向量
      reloadDepots(std::move(vehicleType.reloadDepots)), // 移动reloadDepots向量
      maxReloads(vehicleType.maxReloads),
      maxOvertime(vehicleType.maxOvertime),
      unitOvertimeCost(vehicleType.unitOvertimeCost),
      maxDuration(vehicleType.maxDuration),
      name(vehicleType.name)  // we can steal 可以窃取名称指针
{
    vehicleType.name = nullptr;  // stolen 将原对象的名称指针置空
}

// VehicleType析构函数：释放名称字符串内存
ProblemData::VehicleType::~VehicleType() { delete[] name; }

// VehicleType替换方法：返回一个新的VehicleType，用给定参数替换相应属性
ProblemData::VehicleType ProblemData::VehicleType::replace(
    std::optional<size_t> numAvailable,
    std::optional<std::vector<Load>> capacity,
    std::optional<size_t> startDepot,
    std::optional<size_t> endDepot,
    std::optional<Cost> fixedCost,
    std::optional<Duration> twEarly,
    std::optional<Duration> twLate,
    std::optional<Duration> shiftDuration,
    std::optional<Distance> maxDistance,
    std::optional<Cost> unitDistanceCost,
    std::optional<Cost> unitDurationCost,
    std::optional<size_t> profile,
    std::optional<Duration> startLate,
    std::optional<std::vector<Load>> initialLoad,
    std::optional<std::vector<size_t>> reloadDepots,
    std::optional<size_t> maxReloads,
    std::optional<Duration> maxOvertime,
    std::optional<Cost> unitOvertimeCost,
    std::optional<std::string> name) const
{
    // 使用提供的值，如果未提供则使用当前值
    return {numAvailable.value_or(this->numAvailable),
            capacity.value_or(this->capacity),
            startDepot.value_or(this->startDepot),
            endDepot.value_or(this->endDepot),
            fixedCost.value_or(this->fixedCost),
            twEarly.value_or(this->twEarly),
            twLate.value_or(this->twLate),
            shiftDuration.value_or(this->shiftDuration),
            maxDistance.value_or(this->maxDistance),
            unitDistanceCost.value_or(this->unitDistanceCost),
            unitDurationCost.value_or(this->unitDurationCost),
            profile.value_or(this->profile),
            startLate.value_or(this->startLate),
            initialLoad.value_or(this->initialLoad),
            reloadDepots.value_or(this->reloadDepots),
            maxReloads.value_or(this->maxReloads),
            maxOvertime.value_or(this->maxOvertime),
            unitOvertimeCost.value_or(this->unitOvertimeCost),
            name.value_or(this->name)};
}

// 返回该车辆类型可以执行的最大行程数
size_t ProblemData::VehicleType::maxTrips() const
{
    // When maxReloads is at its maximum size, maxReloads + 1 wraps around to 0,
    // and then std::max() ensures we still return a reasonable value.
    // 当maxReloads达到最大值时，maxReloads + 1会回绕到0，std::max()确保返回合理值
    return reloadDepots.empty() ? 1 : std::max(maxReloads, maxReloads + 1);
}

// VehicleType相等运算符：比较两个车辆类型是否相等
bool ProblemData::VehicleType::operator==(VehicleType const &other) const
{
    // clang-format off
    return numAvailable == other.numAvailable
        && capacity == other.capacity
        && startDepot == other.startDepot
        && endDepot == other.endDepot
        && fixedCost == other.fixedCost
        && twEarly == other.twEarly
        && twLate == other.twLate
        && shiftDuration == other.shiftDuration
        && maxDistance == other.maxDistance
        && unitDistanceCost == other.unitDistanceCost
        && unitDurationCost == other.unitDurationCost
        && profile == other.profile
        && startLate == other.startLate
        && initialLoad == other.initialLoad
        && reloadDepots == other.reloadDepots
        && maxReloads == other.maxReloads
        && maxOvertime == other.maxOvertime
        && unitOvertimeCost == other.unitOvertimeCost
        && std::strcmp(name, other.name) == 0;  // 比较名称字符串
    // clang-format on
}

// 返回所有客户的列表
std::vector<ProblemData::Client> const &ProblemData::clients() const
{
    return clients_;
}

// 返回所有仓库的列表
std::vector<ProblemData::Depot> const &ProblemData::depots() const
{
    return depots_;
}

// 返回所有客户组的列表
std::vector<ProblemData::ClientGroup> const &ProblemData::groups() const
{
    return groups_;
}

// 返回所有车辆类型的列表
std::vector<ProblemData::VehicleType> const &ProblemData::vehicleTypes() const
{
    return vehicleTypes_;
}

// 返回所有距离矩阵的列表
std::vector<Matrix<Distance>> const &ProblemData::distanceMatrices() const
{
    return dists_;
}

// 返回所有持续时间矩阵的列表
std::vector<Matrix<Duration>> const &ProblemData::durationMatrices() const
{
    return durs_;
}

// 返回指定索引的客户组
ProblemData::ClientGroup const &ProblemData::group(size_t group) const
{
    assert(group < groups_.size());
    return groups_[group];
}

// 返回指定索引的车辆类型
ProblemData::VehicleType const &
ProblemData::vehicleType(size_t vehicleType) const
{
    assert(vehicleType < vehicleTypes_.size());
    return vehicleTypes_[vehicleType];
}

// 返回客户位置的中心点
std::pair<pyvrp::Coordinate, pyvrp::Coordinate> const &
ProblemData::centroid() const
{
    return centroid_;
}

// 返回客户数量
size_t ProblemData::numClients() const { return clients_.size(); }

// 返回仓库数量
size_t ProblemData::numDepots() const { return depots_.size(); }

// 返回客户组数量
size_t ProblemData::numGroups() const { return groups_.size(); }

// 返回位置总数（仓库数 + 客户数）
size_t ProblemData::numLocations() const { return numDepots() + numClients(); }

// 返回车辆类型数量
size_t ProblemData::numVehicleTypes() const { return vehicleTypes_.size(); }

// 返回车辆总数
size_t ProblemData::numVehicles() const { return numVehicles_; }

// 返回路径规划配置数量
size_t ProblemData::numProfiles() const
{
    assert(dists_.size() == durs_.size());  // 距离矩阵和持续时间矩阵数量应该相等
    return dists_.size();
}

// 返回负载维度数量
size_t ProblemData::numLoadDimensions() const { return numLoadDimensions_; }

// 验证问题数据的一致性和有效性
void ProblemData::validate() const
{
    // Client checks. 客户检查
    for (size_t idx = numDepots(); idx != numLocations(); ++idx)
    {
        ProblemData::Client const &client = location(idx);

        // 检查配送量维度是否一致
        if (client.delivery.size() != numLoadDimensions_)
        {
            auto const *msg = "Client has inconsistent delivery size.";
            throw std::invalid_argument(msg);
        }

        // 检查拾取量维度是否一致
        if (client.pickup.size() != numLoadDimensions_)
        {
            auto const *msg = "Client has inconsistent pickup size.";
            throw std::invalid_argument(msg);
        }

        // 如果客户不在任何组中，跳过组相关检查
        if (!client.group)
            continue;

        // 检查客户引用的组索引是否有效
        if (*client.group >= numGroups())
            throw std::out_of_range("Client references invalid group.");

        auto const &group = groups_[*client.group];
        // 检查客户是否在其引用的组中
        if (std::find(group.begin(), group.end(), idx) == group.end())
        {
            auto const *msg = "Client not in the group it references.";
            throw std::invalid_argument(msg);
        }

        // 检查必需客户不能在互斥组中
        if (client.required && group.mutuallyExclusive)
        {
            auto const *msg = "Required client in mutually exclusive group.";
            throw std::invalid_argument(msg);
        }
    }

    // Depot checks. 仓库检查
    if (depots_.empty())
        throw std::invalid_argument("Expected at least one depot.");

    // Group checks. 客户组检查
    for (size_t idx = 0; idx != numGroups(); ++idx)
    {
        auto const &group = groups_[idx];

        // 检查组不能为空
        if (group.empty())
            throw std::invalid_argument("Empty client group not understood.");

        // 检查组中每个客户的索引是否有效
        for (auto const client : group)
        {
            if (client < numDepots() || client >= numLocations())
                throw std::out_of_range("Group references invalid client.");

            ProblemData::Client const &clientData = location(client);
            // 检查客户是否引用了正确的组
            if (!clientData.group || *clientData.group != idx)
            {
                auto const *msg = "Group references client not in group.";
                throw std::invalid_argument(msg);
            }
        }
    }

    // Vehicle type checks. 车辆类型检查
    if (vehicleTypes_.empty())
        throw std::invalid_argument("Expected at least one vehicle type.");

    for (auto const &vehicleType : vehicleTypes_)
    {
        // 检查容量维度是否一致
        if (vehicleType.capacity.size() != numLoadDimensions_)
        {
            auto const *msg = "Vehicle type has inconsistent capacity size.";
            throw std::invalid_argument(msg);
        }

        // 检查出发仓库索引是否有效
        if (vehicleType.startDepot >= numDepots())
            throw std::out_of_range("Vehicle type has invalid start depot.");

        // 检查返回仓库索引是否有效
        if (vehicleType.endDepot >= numDepots())
            throw std::out_of_range("Vehicle type has invalid end depot.");

        // 检查配置索引是否有效
        if (vehicleType.profile >= dists_.size())
            throw std::out_of_range("Vehicle type has invalid profile.");

        // 检查车辆和出发仓库的时间窗是否重叠
        if (!hasTimeOverlap(depots_[vehicleType.startDepot], vehicleType))
            throw std::invalid_argument("Vehicle and its start depot have no "
                                        "overlapping time windows.");

        // 检查车辆和返回仓库的时间窗是否重叠
        if (!hasTimeOverlap(depots_[vehicleType.endDepot], vehicleType))
            throw std::invalid_argument("Vehicle and its end depot have no "
                                        "overlapping time windows.");

        // 检查重新装载仓库索引是否有效
        for (auto const depot : vehicleType.reloadDepots)
            if (depot >= numDepots())
                throw std::out_of_range("Vehicle has invalid reload depot.");
    }

    // Matrix checks. 矩阵检查
    if (dists_.empty() || durs_.empty())
        throw std::invalid_argument("Need at least one distance and duration "
                                    "matrix.");

    // 检查距离矩阵和持续时间矩阵数量是否一致
    if (dists_.size() != durs_.size())
        throw std::invalid_argument("Inconsistent number of distance and "
                                    "duration matrices.");

    // 检查每个矩阵的形状和对角线
    for (size_t idx = 0; idx != dists_.size(); ++idx)
    {
        auto const numLocs = numLocations();
        auto const &dist = dists_[idx];
        auto const &dur = durs_[idx];

        // 检查距离矩阵形状是否匹配问题规模
        if (dist.numRows() != numLocs || dist.numCols() != numLocs)
            throw std::invalid_argument("Distance matrix shape does not match "
                                        "the problem size.");

        // 检查持续时间矩阵形状是否匹配问题规模
        if (dur.numRows() != numLocs || dur.numCols() != numLocs)
            throw std::invalid_argument("Duration matrix shape does not match "
                                        "the problem size.");

        // 检查矩阵对角线必须全为0（自己到自己的距离和时间为0）
        for (size_t loc = 0; loc != numLocs; ++loc)
        {
            if (dist(loc, loc) != 0)
                throw std::invalid_argument("Distance matrix diagonals must be "
                                            "all zero.");

            if (dur(loc, loc) != 0)
                throw std::invalid_argument("Duration matrix diagonals must be "
                                            "all zero.");
        }
    }
}

// 替换方法：返回一个新的ProblemData实例，用给定参数替换相应数据
ProblemData
ProblemData::replace(std::optional<std::vector<Client>> &clients,
                     std::optional<std::vector<Depot>> &depots,
                     std::optional<std::vector<VehicleType>> &vehicleTypes,
                     std::optional<std::vector<Matrix<Distance>>> &distMats,
                     std::optional<std::vector<Matrix<Duration>>> &durMats,
                     std::optional<std::vector<ClientGroup>> &groups) const
{
    // 使用提供的值，如果未提供则使用当前值
    return {clients.value_or(clients_),
            depots.value_or(depots_),
            vehicleTypes.value_or(vehicleTypes_),
            distMats.value_or(dists_),
            durMats.value_or(durs_),
            groups.value_or(groups_)};
}

// ProblemData构造函数：初始化问题数据的所有属性
ProblemData::ProblemData(std::vector<Client> clients,
                         std::vector<Depot> depots,
                         std::vector<VehicleType> vehicleTypes,
                         std::vector<Matrix<Distance>> distMats,
                         std::vector<Matrix<Duration>> durMats,
                         std::vector<ClientGroup> groups)
    : dists_(std::move(distMats)),        // 移动距离矩阵列表
      durs_(std::move(durMats)),          // 移动持续时间矩阵列表
      clients_(std::move(clients)),       // 移动客户列表
      depots_(std::move(depots)),         // 移动仓库列表
      vehicleTypes_(std::move(vehicleTypes)), // 移动车辆类型列表
      groups_(std::move(groups)),         // 移动客户组列表
      // 计算车辆总数：累加所有车辆类型的可用车辆数
      numVehicles_(std::accumulate(vehicleTypes_.begin(),
                                   vehicleTypes_.end(),
                                   0,
                                   [](auto sum, VehicleType const &type)
                                   { return sum + type.numAvailable; })),
      // 计算负载维度数量
      numLoadDimensions_(
          clients_.empty()
              // If there are no clients we look at the vehicle types. If both
              // are empty we default to 0. Clients have pickups and deliveries
              // but the client constructor already ensures those are of equal
              // size (within a single client).
              // 如果没有客户，则查看车辆类型。如果两者都为空，默认为0。
              // 客户有拾取和配送，但客户构造函数已确保它们大小相等（在单个客户内）。
              ? (vehicleTypes_.empty() ? 0 : vehicleTypes_[0].capacity.size())
              : clients_[0].delivery.size()),
      // 检查是否存在时间窗约束
      hasTimeWindows_(
          std::any_of(clients_.begin(), clients_.end(), hasTimeWindow<Client>)
          || std::any_of(depots_.begin(), depots_.end(), hasTimeWindow<Depot>)
          || std::any_of(vehicleTypes_.begin(),
                         vehicleTypes_.end(),
                         hasTimeWindow<VehicleType>))
{
    // 计算客户位置的中心点
    for (auto const &client : clients_)
    {
        centroid_.first += static_cast<double>(client.x) / numClients();
        centroid_.second += static_cast<double>(client.y) / numClients();
    }

    // 验证数据一致性
    validate();
}
