#ifndef PYVRP_TRIP_H
#define PYVRP_TRIP_H

#include "ProblemData.h"

#include <optional>
#include <vector>

namespace pyvrp
{
/**
 * Trip(
 *     data: ProblemData,
 *     visits: list[int],
 *     vehicle_type: int,
 *     start_depot: int | None = None,
 *     end_depot: int | None = None,
 * )
 *
 * A simple class that stores the trip plan and some related statistics. The
 * start and end depots default to the vehicle type's start and end depots if
 * not explicitly given.
 *
 * .. note::
 *
 *    A trip does not stand on its own - it is intended to be part of a
 *    :class:`~pyvrp._pyvrp.Route`, which tracks overall route statistics
 *    involving all trips, and determines route feasibility.
 */
class Trip
{
public:
    using Client = size_t;
    using Visits = std::vector<Client>;

private:
    Visits visits_;  // 此行程访问的客户列表

    Distance distance_ = 0;         // Total travel distance on this trip 此行程的总行驶距离
    std::vector<Load> delivery_;    // Total delivery amount served on this trip 此行程服务的总配送量
    std::vector<Load> pickup_;      // Total pickup amount gathered on this trip 此行程收集的总拾取量
    std::vector<Load> load_;        // Load on this trip 此行程的负载
    std::vector<Load> excessLoad_;  // Excess pickup or delivery demand 超额拾取或配送需求
    Duration travel_ = 0;           // Total *travel* duration on this trip 此行程的总*行驶*持续时间
    Duration service_ = 0;          // Total *service* duration on this trip 此行程的总*服务*持续时间
    Duration release_ = 0;          // Release time of this trip 此行程的释放时间
    Cost prizes_ = 0;               // Total value of prizes on this trip 此行程的总奖励值

    std::pair<Coordinate, Coordinate> centroid_;  // Trip center 行程中心点
    size_t vehicleType_;                          // Type of vehicle 车辆类型
    size_t startDepot_;                           // assigned start location 分配的起始位置
    size_t endDepot_;                             // assigned end location 分配的结束位置

public:
    [[nodiscard]] bool empty() const;

    /**
     * Returns the number of clients visited by this trip.
     */
    [[nodiscard]] size_t size() const;

    [[nodiscard]] Client operator[](size_t idx) const;

    [[nodiscard]] Visits::const_iterator begin() const;
    [[nodiscard]] Visits::const_iterator end() const;

    [[nodiscard]] Visits::const_reverse_iterator rbegin() const;
    [[nodiscard]] Visits::const_reverse_iterator rend() const;

    /**
     * Trip visits, as a list of clients.
     */
    [[nodiscard]] Visits const &visits() const;

    /**
     * Total distance travelled on this trip.
     */
    [[nodiscard]] Distance distance() const;

    /**
     * Total client delivery load on this trip.
     */
    [[nodiscard]] std::vector<Load> const &delivery() const;

    /**
     * Total client pickup load on this trip.
     */
    [[nodiscard]] std::vector<Load> const &pickup() const;

    /**
     * Maximum load at any point of this trip.
     */
    [[nodiscard]] std::vector<Load> const &load() const;

    /**
     * Pickup or delivery loads in excess of the vehicle's capacity.
     */
    [[nodiscard]] std::vector<Load> const &excessLoad() const;

    /**
     * Total duration of service on this trip.
     */
    [[nodiscard]] Duration serviceDuration() const;

    /**
     * Total duration of travel on this trip.
     * 此行程的总行驶持续时间
     */
    [[nodiscard]] Duration travelDuration() const;

    /**
     * Earliest time at which this trip can leave the depot. Follows from the
     * release times of clients visited on this trip.
     * 此行程可以离开仓库的最早时间。遵循此行程访问的客户的释放时间。
     */
    [[nodiscard]] Duration releaseTime() const;

    /**
     * Total prize value collected on this trip.
     * 此行程收集的总奖励值
     */
    [[nodiscard]] Cost prizes() const;

    /**
     * Center point of the client locations on this trip.
     * 此行程的客户位置中心点
     */
    [[nodiscard]] std::pair<Coordinate, Coordinate> const &centroid() const;

    /**
     * Index of the type of vehicle used on this trip.
     * 此行程使用的车辆类型的索引
     */
    [[nodiscard]] size_t vehicleType() const;

    /**
     * Location index of the trip's starting depot.
     * 行程的起始仓库位置索引
     */
    [[nodiscard]] size_t startDepot() const;

    /**
     * Location index of the trip's ending depot.
     * 行程的结束仓库位置索引
     */
    [[nodiscard]] size_t endDepot() const;

    /**
     * Returns whether this trip violates capacity constraints.
     * 返回此行程是否违反容量约束
     */
    [[nodiscard]] bool hasExcessLoad() const;

    bool operator==(Trip const &other) const;

    Trip &operator=(Trip const &other) = default;
    Trip &operator=(Trip &&other) = default;

    Trip() = delete;

    Trip(Trip const &other) = default;
    Trip(Trip &&other) = default;

    Trip(ProblemData const &data,
         Visits visits,
         size_t vehicleType,
         std::optional<size_t> startDepot = std::nullopt,
         std::optional<size_t> endDepot = std::nullopt);

    // This constructor does *no* validation. Useful when unserialising objects.
    Trip(Visits visits,
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
         size_t endDepot);
};
}  // namespace pyvrp

std::ostream &operator<<(std::ostream &out, pyvrp::Trip const &trip);

#endif  // PYVRP_TRIP_H
