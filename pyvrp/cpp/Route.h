#ifndef PYVRP_ROUTE_H
#define PYVRP_ROUTE_H

#include "Measure.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Trip.h"

#include <iosfwd>
#include <optional>
#include <vector>

namespace pyvrp
{
/**
 * Route(data: ProblemData, visits: list[int] | list[Trip], vehicle_type: int)
 *
 * A simple class that stores the route plan and some statistics. Internally,
 * a route consists of one or more :class:`~pyvrp._pyvrp.Trip` objects.
 * 由一个或多个Trip组成，表示完整的车辆规划路径，包括多个行程的组合。
 */
class Route
{
    using Client = size_t;
    using Depot = size_t;
    using VehicleType = size_t;
    using Trips = std::vector<Trip>;
    using Visits = std::vector<Client>;

    // Validates the consistency of the constructed instance.
    // 验证构造实例的一致性
    void validate(ProblemData const &data) const;

    // Creates the data returned by ``schedule()``.
    // 创建由``schedule()``返回的数据
    void makeSchedule(ProblemData const &data);

public:
    /**
     * Forward iterator through the clients visited by this route.
     */
    class Iterator
    {
        Route const *route_ = nullptr;
        size_t trip_ = 0;
        size_t idx_ = 0;

    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Client;

        Iterator(Route const &route, size_t idx);

        Iterator() = default;
        Iterator(Iterator const &other) = default;
        Iterator(Iterator &&other) = default;

        Iterator &operator=(Iterator const &other) = default;
        Iterator &operator=(Iterator &&other) = default;

        bool operator==(Iterator const &other) const;

        Client operator*() const;

        Iterator operator++(int);
        Iterator &operator++();
    };

    /**
     * Simple object that stores some data about a client or depot visit.
     *
     * Attributes
     * ----------
     * location : int
     *     Index of the visited location (client or depot).
     * trip : int
     *     Index of the trip visiting this location.
     * start_service : int
     *     Time at which service begins.
     * end_service : int
     *     Time at which service completes.
     * service_duration : int
     *     Duration of the service.
     * wait_duration : int
     *     If the vehicle arrives early, this is the duration it has to wait
     *     until it can begin service.
     * time_warp : int
     *     If the vehicle arrives late, this is the duration it has to 'travel
     *     back in time' to begin service. Non-zero time warp indicates an
     *     infeasible route.
     */
    struct ScheduledVisit  // 计划访问结构：存储客户或仓库访问的数据
    {
        size_t const location = 0;  // 访问的位置索引（客户或仓库）
        size_t const trip = 0;      // 访问此位置的行程索引
        Duration const startService = 0;  // 服务开始时间
        Duration const endService = 0;   // 服务结束时间
        Duration const waitDuration = 0;  // 等待持续时间（如果车辆早到）
        Duration const timeWarp = 0;     // 时间扭曲（如果车辆晚到）

        ScheduledVisit(size_t location,
                       size_t trip,
                       Duration startService,
                       Duration endService,
                       Duration waitDuration,
                       Duration timeWarp);

        [[nodiscard]] Duration serviceDuration() const;  // 返回服务持续时间
    };

private:
    Trips trips_ = {};  // 组成此路线的行程列表
    std::vector<ScheduledVisit> schedule_ = {};  // Client visit schedule data 客户访问计划数据
    Distance distance_ = 0;        // Total travel distance on this route 此路线的总行驶距离
    Cost distanceCost_ = 0;        // Total cost of travel distance 行驶距离的总成本
    Distance excessDistance_ = 0;  // Excess travel distance 超额行驶距离
    std::vector<Load> delivery_;   // Total delivery amount served on this route 此路线服务的总配送量
    std::vector<Load> pickup_;     // Total pickup amount gathered on this route 此路线收集的总拾取量
    std::vector<Load> excessLoad_;  // Excess pickup or delivery demand 超额拾取或配送需求
    Duration duration_ = 0;         // Total duration of this route 此路线的总持续时间
    Duration overtime_ = 0;         // Total overtime of this route 此路线的总加班时间
    Cost durationCost_ = 0;         // Total cost of route duration 路线持续时间的总成本
    Duration timeWarp_ = 0;         // Total time warp on this route 此路线的总时间扭曲
    Duration travel_ = 0;           // Total *travel* duration on this route 此路线的总*行驶*持续时间
    Duration service_ = 0;          // Total *service* duration on this route 此路线的总*服务*持续时间
    Duration startTime_ = 0;        // (earliest) start time of this route （最早）此路线的开始时间
    Duration slack_ = 0;            // Total time slack on this route 此路线的总时间松弛
    Cost prizes_ = 0;               // Total value of prizes on this route 此路线的总奖励值

    std::pair<Coordinate, Coordinate> centroid_;  // Route center 路线中心点
    VehicleType vehicleType_;                     // Type of vehicle 车辆类型
    Depot startDepot_;                            // Assigned start depot 分配的起始仓库
    Depot endDepot_;                              // Assigned end depot 分配的结束仓库

public:
    [[nodiscard]] bool empty() const;

    /**
     * Returns the number of clients visited by this route.
     * 返回路线中的客户总数
     */
    [[nodiscard]] size_t size() const;

    /**
     * Returns the number of trips in this route.
     * 返回路线中的行程总数
     */
    [[nodiscard]] size_t numTrips() const;

    [[nodiscard]] Client operator[](size_t idx) const;

    [[nodiscard]] Iterator begin() const;
    [[nodiscard]] Iterator end() const;

    /**
     * Returns the trips that make up this route.
     * 返回组成此路线的行程
     */
    [[nodiscard]] Trips const &trips() const;

    /**
     * Returns the trip at the given index.
     * 返回给定索引处的行程
     */
    [[nodiscard]] Trip const &trip(size_t idx) const;

    /**
     * Route visits, as a list of clients.
     * 路线访问，作为客户列表
     */
    [[nodiscard]] Visits visits() const;

    /**
     * Statistics about each visit and the overall route schedule. This includes
     * all client visits, but also starting and leaving depots.
     *
     * .. note::
     *
     *    The schedule assumes the route starts at :meth:`~start_time`. Starting
     *    later may be feasible, but shifts the schedule.
     * 路线的计划访问，作为客户列表
     */
    [[nodiscard]] std::vector<ScheduledVisit> const &schedule() const;

    /**
     * Total distance travelled on this route.
     * 此路线的总行驶距离
     */
    [[nodiscard]] Distance distance() const;

    /**
     * Total cost of the distance travelled on this route.
     * 此路线行驶距离的总成本
     */
    [[nodiscard]] Cost distanceCost() const;

    /**
     * Distance in excess of the vehicle's maximum distance constraint.
     * 车辆最大距离约束之外的距离
     */
    [[nodiscard]] Distance excessDistance() const;

    /**
     * Total client delivery load on this route.
     * 此路线的总配送量
     */
    [[nodiscard]] std::vector<Load> const &delivery() const;

    /**
     * Total client pickup load on this route.
     * 此路线的总提取量
     */
    [[nodiscard]] std::vector<Load> const &pickup() const;

    /**
     * Pickup or delivery loads in excess of the vehicle's capacity.
     * 超额拾取或配送需求
     */
    [[nodiscard]] std::vector<Load> const &excessLoad() const;

    /**
     * Total route duration, including travel, service, waiting and overtime.
     * 此路线的总持续时间，包括行驶、服务、等待和加班时间
     */
    [[nodiscard]] Duration duration() const;

    /**
     * Overtime incurred on this route.
     * 此路线的总加班时间
     */
    [[nodiscard]] Duration overtime() const;

    /**
     * Total cost of the duration of this route, including overtime.
     * 此路线持续时间的总成本，包括加班
     */
    [[nodiscard]] Cost durationCost() const;

    /**
     * Total duration of client service on this route.
     * 此路线的总服务时间
     */
    [[nodiscard]] Duration serviceDuration() const;

    /**
     * Amount of time warp incurred on this route.
     * 此路线的总时间扭曲
     */
    [[nodiscard]] Duration timeWarp() const;

    /**
     * Total duration of travel on this route.
     * 此路线的总行驶时间
     */
    [[nodiscard]] Duration travelDuration() const;

    /**
     * Total waiting duration on this route.
     * 此路线的总等待时间
     */
    [[nodiscard]] Duration waitDuration() const;

    /**
     * Start time of this route. This is the earliest possible time at which
     * the route can leave the depot and have a minimal duration and time warp.
     * If there is positive :meth:`~slack`, the start time can be delayed by at
     * most :meth:`~slack` time units without increasing the total (minimal)
     * route duration, or time warp.
     *
     * .. note::
     *
     *    It may be possible to leave before the start time (if the vehicle's
     *    time window allows for it). That will introduce additional waiting
     *    time, such that the route duration will then no longer be minimal.
     *    Delaying departure by more than :meth:`~slack` time units always
     *    increases time warp, which could turn the route infeasible.
     * 路线的开始时间
     */
    [[nodiscard]] Duration startTime() const;

    /**
     * End time of the route. This is equivalent to
     * ``start_time + duration - time_warp``.
     * 路线的结束时间
     */
    [[nodiscard]] Duration endTime() const;

    /**
     * Time by which departure from the depot can be delayed without resulting
     * in (additional) time warp or increased route duration.
     * 路线的松弛时间
     */
    [[nodiscard]] Duration slack() const;

    /**
     * Earliest time at which this route can leave the depot. Follows from the
     * release times of clients visited on the first trip of this route.
     *
     * .. note::
     *
     *    The route's release time should not be later than its start time,
     *    unless the route has time warp.
     * 路线的释放时间
     */
    [[nodiscard]] Duration releaseTime() const;

    /**
     * Total prize value collected on this route.
     * 此路线收集的总奖励值
     */
    [[nodiscard]] Cost prizes() const;

    /**
     * Center point of the client locations on this route.
     * 此路线的客户位置中心点
     */
    [[nodiscard]] std::pair<Coordinate, Coordinate> const &centroid() const;

    /**
     * Index of the type of vehicle used on this route.
     * 此路线使用的车辆类型的索引
     */
    [[nodiscard]] VehicleType vehicleType() const;

    /**
     * Location index of the route's starting depot.
     * 此路线的起始仓库的位置索引
     */
    [[nodiscard]] Depot startDepot() const;

    /**
     * Location index of the route's ending depot.
     * 此路线的结束仓库的位置索引
     */
    [[nodiscard]] Depot endDepot() const;

    /**
     * Returns whether this route is feasible.
     * 返回此路线是否可行
     */
    [[nodiscard]] bool isFeasible() const;

    /**
     * Returns whether this route violates capacity constraints.
     * 返回此路线是否违反容量约束
     */
    [[nodiscard]] bool hasExcessLoad() const;

    /**
     * Returns whether this route violates maximum distance constraints.
     * 返回此路线是否违反最大距离约束
     */
    [[nodiscard]] bool hasExcessDistance() const;

    /**
     * Returns whether this route violates time window or maximum duration
     * constraints.
     * 返回此路线是否违反时间窗口或最大持续时间约束
     */
    [[nodiscard]] bool hasTimeWarp() const;

    bool operator==(Route const &other) const;

    Route &operator=(Route const &other) = default;
    Route &operator=(Route &&other) = default;

    Route() = delete;

    Route(Route const &other) = default;
    Route(Route &&other) = default;

    Route(ProblemData const &data, Trips trips, VehicleType vehicleType);

    Route(ProblemData const &data, Visits visits, VehicleType vehicleType);

    // This constructor does *no* validation. Useful when unserialising objects.
    Route(Trips trips,
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
          VehicleType vehicleType,
          Depot startDepot,
          Depot endDepot,
          std::vector<ScheduledVisit> schedule);
};
}  // namespace pyvrp

std::ostream &operator<<(std::ostream &out, pyvrp::Route const &route);

#endif  // PYVRP_ROUTE_H
