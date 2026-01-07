#ifndef PYVRP_SEARCH_ROUTE_H
#define PYVRP_SEARCH_ROUTE_H

#include "../Route.h"  // pyvrp::Route
#include "DurationSegment.h"
#include "LoadSegment.h"
#include "ProblemData.h"

#include <algorithm>
#include <cassert>
#include <concepts>
#include <iosfwd>
#include <utility>
//Search Route用于搜索优化中的局部评估。依赖主Route，但扩展了概念以支持更复杂的搜索操作。
//支持快速增量成本计算和就地修改。用于评估邻域操作（如交换、插入客户），而不直接修改原始解。
namespace pyvrp::search
{
// 定义访问段的最小接口要求
template <typename T>
concept Segment = requires(T arg, size_t profile, size_t dimension) {
    { arg.route() };
    { arg.first() } -> std::same_as<size_t>;
    { arg.last() } -> std::same_as<size_t>;
    { arg.size() } -> std::same_as<size_t>;
    { arg.startsAtReloadDepot() } -> std::same_as<bool>;
    { arg.endsAtReloadDepot() } -> std::same_as<bool>;
    { arg.distance(profile) } -> std::convertible_to<Distance>;
    { arg.duration(profile) } -> std::convertible_to<DurationSegment>;
    { arg.load(dimension) } -> std::convertible_to<LoadSegment>;
};

namespace detail
{
// 元组反转的实现细节
template <class Tuple, std::size_t... Indices>
auto constexpr reverse_impl(Tuple &&tuple, std::index_sequence<Indices...>)
{
    return std::make_tuple(std::get<sizeof...(Indices) - 1 - Indices>(
        std::forward<Tuple>(tuple))...);
}

// 反转元组的函数
template <class Tuple> auto constexpr reverse(Tuple &&tuple)
{
    auto constexpr size = std::tuple_size_v<std::remove_reference_t<Tuple>>;
    auto constexpr indices = std::make_index_sequence<size>{};
    return reverse_impl(tuple, indices);
}
}  // namespace detail

/**
 * 这个Route类支持快速的增量成本计算和原地修改。可用于实现移动操作的评估。
 *
 * Route对象跟踪完整路线，包括仓库。路线上的客户和仓库可以通过Route::operator[]访问。
 *
 * .. note::
 *
 *    对Route对象的修改不会立即传播到其统计数据（如时间窗、负载和距离数据）。
 *    需要调用Route::update()才能使统计数据同步！
 */
class Route
{
public:
    /**
     * 用于跟踪提议路线结构的简单类。通过调用适当的成员函数可以高效评估这个新结构，
     * 详细说明提议路线的新统计数据。
     *
     * .. note::
     *
     *    当成员函数检测到某个统计数据对新提议路线的成本没有影响时，可能会走快捷路径。
     */
    template <Segment... Segments> class Proposal
    {
        std::tuple<Segments...> segments_;

        /**
         * 返回提议路线中的仓库和客户数量。
         */
        size_t size() const;

        /**
         * 返回提议路线是否为空。
         */
        bool empty() const;

    public:
        Proposal(Segments &&...segments);

        /**
         * 提议的路线。这是与第一个和最后一个段相关联的路线，决定了评估提议时使用的车辆类型和路线配置文件。
         */
        Route const *route() const;

        /**
         * 返回提议路线的（距离成本，超额距离）属性。
         */
        std::pair<Cost, Distance> distance() const;

        /**
         * 返回提议路线的（持续时间成本，时间扭曲）属性。
         */
        std::pair<Cost, Duration> duration() const;

        /**
         * 返回提议路线的超载量。
         */
        Load excessLoad(size_t dimension) const;
    };

    /**
     * 围绕客户或仓库位置的轻量级包装类。此类跟踪它所在的路线，以及它在该路线中的当前位置和角色。
     */
    class Node
    {
        friend class Route;

        size_t loc_;    // 该节点代表的位置
        size_t idx_;    // 在路线中的位置
        size_t trip_;   // 行程索引
        Route *route_;  // 指示节点所属的路线（如果有）

    public:
        Node(size_t loc);

        /**
         * 返回该节点代表的位置。
         */
        [[nodiscard]] inline size_t client() const;  

        /**
         * 返回该节点在路线中的位置。当节点不在路线中时，此值为0。
         */
        [[nodiscard]] inline size_t idx() const;

        /**
         * 返回该节点分配的行程号。当节点*不*在路线中时，此值为0。
         */
        [[nodiscard]] inline size_t trip() const;

        /**
         * 返回该节点当前所在的路线。如果节点不在路线中，返回None（C++：nullptr）。
         */
        [[nodiscard]] inline Route *route() const;

        /**
         * 返回该节点是否为仓库。
         */
        [[nodiscard]] inline bool isDepot() const;

        /**
         * 返回该节点是否为起始仓库。
         */
        [[nodiscard]] inline bool isStartDepot() const;

        /**
         * 返回该节点是否为结束仓库。
         */
        [[nodiscard]] inline bool isEndDepot() const;

        /**
         * 返回该节点是否为补给仓库。
         */
        [[nodiscard]] inline bool isReloadDepot() const;

        /**
         * 将节点分配到给定路线的给定索引处，在给定的行程中。
         */
        void assign(Route *route, size_t idx, size_t trip);

        /**
         * 将节点从其分配的路线中移除（如果有）。
         */
        void unassign();
    };

    /**
     * 遍历该路线访问的客户节点的前向迭代器。
     */
    class Iterator
    {
        std::vector<Node *> const *nodes_;
        size_t idx_ = 0;

        // 确保我们跳过补给仓库
        void ensureValidIndex();

    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Node *;

        Iterator(std::vector<Node *> const &nodes, size_t idx);

        Iterator() = default;
        Iterator(Iterator const &other) = default;
        Iterator(Iterator &&other) = default;

        Iterator &operator=(Iterator const &other) = default;
        Iterator &operator=(Iterator &&other) = default;

        bool operator==(Iterator const &other) const;

        Node *operator*() const;

        Iterator operator++(int);
        Iterator &operator++();
    };

private:
    using LoadSegments = std::vector<LoadSegment>;

    /**
     * 存储从start开始到结束仓库（包含）的路线段相关数据的类。
     */
    class SegmentAfter
    {
        Route const &route_;
        size_t const start;

    public:
        inline Route const *route() const;

        inline size_t first() const;  // start处的客户
        inline size_t last() const;   // 结束仓库
        inline size_t size() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentAfter(Route const &route, size_t start);
        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline LoadSegment const &load(size_t dimension) const;
    };

    /**
     * 存储从起始仓库开始到end（包含）的路线段相关数据的类。
     */
    class SegmentBefore
    {
        Route const &route_;
        size_t const end;

    public:
        inline Route const *route() const;

        inline size_t first() const;  // 起始仓库
        inline size_t last() const;   // end处的客户
        inline size_t size() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBefore(Route const &route, size_t end);
        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline LoadSegment const &load(size_t dimension) const;
    };

    /**
     * 存储从start开始到end（包含）的路线段相关数据的类。
     * 该段必须仅包含单个行程，可能包括其结束仓库。
     */
    class SegmentBetween
    {
        Route const &route_;
        size_t const start;
        size_t const end;

    public:
        inline Route const *route() const;

        inline size_t first() const;  // start处的客户
        inline size_t last() const;   // end处的客户
        inline size_t size() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBetween(Route const &route, size_t start, size_t end);
        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline LoadSegment load(size_t dimension) const;
    };

    ProblemData const &data;

    ProblemData::VehicleType const &vehicleType_;
    size_t const idx_;

    Distance distance_;  // 单独缓存的成本组件
    Cost distanceCost_;
    Distance excessDistance_;
    Duration duration_;
    Cost durationCost_;
    Duration timeWarp_;

    std::vector<Node> depots_;  // 起始、结束和补给仓库（按此顺序）

    std::vector<Node *> nodes;   // 此路线中的节点，包括仓库
    std::vector<size_t> visits;  // 此路线中的位置，包括仓库
    std::pair<Coordinate, Coordinate> centroid_;  // 客户中心点

    std::vector<Distance> cumDist;  // 从起始到节点的累计距离（包含）

    // 负载数据，针对每个负载维度。这些向量形成矩阵，其中行索引负载维度，列索引节点。
    std::vector<LoadSegments> loadAt;      // 每个节点的负载数据
    std::vector<LoadSegments> loadAfter;   // 从节点到结束（包含）的负载
    std::vector<LoadSegments> loadBefore;  // 从起始到节点（包含）的负载

    std::vector<Load> load_;        // 路线负载（针对每个维度）
    std::vector<Load> excessLoad_;  // 路线超载（针对每个维度）

    std::vector<DurationSegment> durAt;      // 每个节点的持续时间数据
    std::vector<DurationSegment> durAfter;   // 从节点到结束（包含）的持续时间
    std::vector<DurationSegment> durBefore;  // 从起始到节点（包含）的持续时间

#ifndef NDEBUG
    // 当启用调试断言时，我们使用此标志来检查统计数据是否仍与路线的节点列表同步。
    // 统计数据仅在调用update()后更新。如果在插入或移除节点后尚未调用该函数，则此标志处于活动状态，
    // 并且对统计数据的断言将失败。
    bool dirty = false;
#endif

public:
    /**
     * 路线索引。
     */
    [[nodiscard]] inline size_t idx() const;

    /**
     * @return 给定idx处的客户或仓库节点。
     */
    [[nodiscard]] inline Node *operator[](size_t idx);
    [[nodiscard]] inline Node const *operator[](size_t idx) const;

    [[nodiscard]] Iterator begin() const;
    [[nodiscard]] Iterator end() const;

    /**
     * 测试此路线是否可行。
     *
     * @return 如果路线可行则返回true，否则返回false。
     */
    [[nodiscard]] inline bool isFeasible() const;

    /**
     * 确定此路线是否负载可行。
     *
     * @return 如果路线超过容量则返回true，否则返回false。
     */
    [[nodiscard]] inline bool hasExcessLoad() const;

    /**
     * 确定此路线是否距离可行。
     *
     * @return 如果路线超过最大距离约束则返回true，否则返回false。
     */
    [[nodiscard]] inline bool hasExcessDistance() const;

    /**
     * 确定此路线是否时间可行。
     *
     * @return 如果路线存在时间扭曲则返回true，否则返回false。
     */
    [[nodiscard]] inline bool hasTimeWarp() const;

    /**
     * 此路线的总负载。
     */
    [[nodiscard]] inline std::vector<Load> const &load() const;

    /**
     * 超过车辆容量的取货或送货负载。
     */
    [[nodiscard]] inline std::vector<Load> const &excessLoad() const;

    /**
     * 超出分配的车辆类型最大距离约束的行驶距离。
     */
    [[nodiscard]] inline Distance excessDistance() const;

    /**
     * 服务此路线的车辆的容量。
     */
    [[nodiscard]] inline std::vector<Load> const &capacity() const;

    /**
     * @return 此路线起始仓库的位置索引。
     */
    [[nodiscard]] inline size_t startDepot() const;

    /**
     * @return 此路线结束仓库的位置索引。
     */
    [[nodiscard]] inline size_t endDepot() const;

    /**
     * @return 服务此路线的车辆的固定成本。
     */
    [[nodiscard]] inline Cost fixedVehicleCost() const;

    /**
     * @return 此路线的总行驶距离。
     */
    [[nodiscard]] inline Distance distance() const;

    /**
     * @return 此路线行驶距离的成本。
     */
    [[nodiscard]] inline Cost distanceCost() const;

    /**
     * @return 此路线每单位行驶距离的成本。
     */
    [[nodiscard]] inline Cost unitDistanceCost() const;

    /**
     * 如果此路线具有距离相关成本组件（通过目标函数或惩罚约束），则返回true。否则返回false。
     */
    [[nodiscard]] inline bool hasDistanceCost() const;

    /**
     * @return 此路线的持续时间。
     */
    [[nodiscard]] inline Duration duration() const;

    /**
     * @return 此路线的加班时间。
     */
    [[nodiscard]] inline Duration overtime() const;

    /**
     * @return 此路线持续时间的成本，包括加班。
     */
    [[nodiscard]] inline Cost durationCost() const;

    /**
     * @return 此路线每单位持续时间的成本。
     */
    [[nodiscard]] inline Cost unitDurationCost() const;

    /**
     * @return 此路线每单位加班时间的成本。
     */
    [[nodiscard]] inline Cost unitOvertimeCost() const;

    /**
     * 如果此路线具有持续时间相关成本组件（通过目标函数或惩罚约束），则返回true。否则返回false。
     */
    [[nodiscard]] inline bool hasDurationCost() const;

    /**
     * @return 服务此路线的车辆支持的（软）最大班次持续时间。这可以选择通过加班延长。
     */
    [[nodiscard]] inline Duration shiftDuration() const;

    /**
     * @return 服务此路线的车辆支持的（硬）最大路线持续时间。
     */
    [[nodiscard]] inline Duration maxDuration() const;

    /**
     * @return 服务此路线的车辆支持的最大加班时间。
     */
    [[nodiscard]] inline Duration maxOvertime() const;

    /**
     * @return 服务此路线的车辆支持的最大路线距离。
     */
    [[nodiscard]] inline Distance maxDistance() const;

    /**
     * @return 此路线上的总时间扭曲。
     */
    [[nodiscard]] inline Duration timeWarp() const;

    /**
     * @return 服务此路线的车辆的路由配置文件。
     * 用于区分不同距离和持续时间矩阵的标识符
     * 在车辆路径问题中，这允许不同的车辆类型使用不同的距离/时间计算方式。
     */
    [[nodiscard]] inline size_t profile() const;

    /**
     * 如果此路线没有客户访问，则返回true，否则返回false。
     */
    [[nodiscard]] inline bool empty() const;

    /**
     * 此路线上的客户和仓库数量。
     */
    [[nodiscard]] inline size_t size() const;

    /**
     * 此路线上的客户数量。
     */
    [[nodiscard]] inline size_t numClients() const;

    /**
     * 返回此路线中起始、结束和补给仓库的数量。
     */
    [[nodiscard]] inline size_t numDepots() const;

    /**
     * 返回此路线中的行程数。
     */
    [[nodiscard]] inline size_t numTrips() const;

    /**
     * 返回此路线允许的最大行程数。
     */
    [[nodiscard]] inline size_t maxTrips() const;

    /**
     * 返回一个可以查询idx处节点相关数据的对象。
     */
    [[nodiscard]] inline SegmentBetween at(size_t idx) const;

    /**
     * 返回一个可以查询从start开始的段相关数据的对象。
     */
    [[nodiscard]] inline SegmentAfter after(size_t start) const;

    /**
     * 返回一个可以查询到end结束的段相关数据的对象。
     */
    [[nodiscard]] inline SegmentBefore before(size_t end) const;

    /**
     * 返回一个可以查询[start, end]之间段相关数据的对象。
     */
    [[nodiscard]] inline SegmentBetween between(size_t start, size_t end) const;

    /**
     * 此路线上客户位置的中心点。
     */
    [[nodiscard]] std::pair<Coordinate, Coordinate> const &centroid() const;

    /**
     * @return 此路线的车辆类型。
     */
    [[nodiscard]] size_t vehicleType() const;

    /**
     * 测试此路线是否与另一条路线在给定的容差[0,1]范围内可能重叠。
     */
    [[nodiscard]] bool overlapsWith(Route const &other, double tolerance) const;

    /**
     * 清除此路线上的所有客户。调用此方法后，empty()返回true。
     */
    void clear();

    /**
     * 为至少给定size数量的节点（仓库和客户）预留容量。
     */
    void reserve(size_t size);

    /**
     * 在索引idx之前插入给定节点。假定给定索引有效。仓库节点被复制到内部内存中，
     * 但客户节点不获取所有权。
     */
    void insert(size_t idx, Node *node);

    /**
     * 在路线末尾追加给定节点指针。仓库节点被复制到内部内存中，但客户节点不获取所有权。
     */
    void push_back(Node *node);

    /**
     * 从路线中移除idx处的节点。起始和结束仓库不能被移除。
     */
    void remove(size_t idx);

    /**
     * 交换给定的两个节点。
     */
    static void swap(Node *first, Node *second);

    /**
     * 更新此路线。在交换节点/更改解决方案后调用。
     */
    void update();

    bool operator==(Route const &other) const;
    bool operator==(pyvrp::Route const &other) const;

    Route(ProblemData const &data, size_t idx, size_t vehicleType);
    ~Route();
};

/**
 * 访问参数节点直接前驱节点的便捷方法。
 */
inline Route::Node *p(Route::Node *node)
{
    auto &route = *node->route();
    return route[node->idx() - 1];
}

inline Route::Node const *p(Route::Node const *node)
{
    auto const &route = *node->route();
    return route[node->idx() - 1];
}

/**
 * 访问参数节点直接后继节点的便捷方法。
 */
inline Route::Node *n(Route::Node *node)
{
    auto &route = *node->route();
    return route[node->idx() + 1];
}

inline Route::Node const *n(Route::Node const *node)
{
    auto const &route = *node->route();
    return route[node->idx() + 1];
}

size_t Route::Node::client() const { return loc_; }//返回该节点代表的位置

size_t Route::Node::idx() const { return idx_; }//返回该节点在路线中的索引

size_t Route::Node::trip() const { return trip_; }//返回该节点所在的行程

Route *Route::Node::route() const { return route_; }//返回该节点当前所在的路线

bool Route::Node::isDepot() const//返回该节点是否为仓库
{
    return isStartDepot() || isEndDepot() || isReloadDepot();
}

bool Route::Node::isStartDepot() const
{
    return route_ && this == &route_->depots_[0];
}

bool Route::Node::isEndDepot() const
{
    return route_ && this == &route_->depots_[1];
}

bool Route::Node::isReloadDepot() const
{
    // clang-format off
    return route_
        && loc_ < route_->data.numDepots()
        && !isStartDepot()
        && !isEndDepot();
    // clang-format on
}

Route::SegmentAfter::SegmentAfter(Route const &route, size_t start)//构造函数
    : route_(route), start(start)
{
    assert(start < route.size());
}

Route::SegmentBefore::SegmentBefore(Route const &route, size_t end)
    : route_(route), end(end)
{
    assert(end < route.size());
}

Route::SegmentBetween::SegmentBetween(Route const &route,
                                      size_t start,
                                      size_t end)
    : route_(route), start(start), end(end)
{
    assert(start <= end && end < route.size());

    // 该段必须仅由单个行程组成，可能包括开始下一个行程（并结束当前行程）的仓库。因此行程差最多为1。
    assert(route[end]->trip() - route[start]->trip() <= route[end]->isDepot());
}

Distance Route::SegmentAfter::distance([[maybe_unused]] size_t profile) const //计算路线段结束位置到开始位置的距离
{
    assert(profile == route_.profile());
    return {route_.cumDist.back() - route_.cumDist[start]}; //  返回从起点到终点的累积距离差
}

DurationSegment
Route::SegmentAfter::duration([[maybe_unused]] size_t profile) const//计算路线段结束位置到开始位置的持续时间
{
    assert(profile == route_.profile());
    return route_.durAfter[start];
}

LoadSegment const &Route::SegmentAfter::load(size_t dimension) const //该方法用于获取指定维度下，从特定起始位置开始的负载段。
{
    return route_.loadAfter[dimension][start];
}

Distance Route::SegmentBefore::distance([[maybe_unused]] size_t profile) const//返回路线结束位置的累积距离
{
    assert(profile == route_.profile());
    return route_.cumDist[end]; 
}

DurationSegment
Route::SegmentBefore::duration([[maybe_unused]] size_t profile) const //返回路线开始位置的累积持续时间
{
    assert(profile == route_.profile());
    return route_.durBefore[end];
}

LoadSegment const &Route::SegmentBefore::load(size_t dimension) const//返回路线开始位置的累积负载
{
    return route_.loadBefore[dimension][end];
}

Route const *Route::SegmentBefore::route() const { return &route_; }

size_t Route::SegmentBefore::first() const { return route_.visits.front(); }
size_t Route::SegmentBefore::last() const { return route_.visits[end]; }
size_t Route::SegmentBefore::size() const { return end + 1; }

bool Route::SegmentBefore::startsAtReloadDepot() const { return false; }
bool Route::SegmentBefore::endsAtReloadDepot() const
{
    return route_.nodes[end]->isReloadDepot();
}

Route const *Route::SegmentAfter::route() const { return &route_; }

size_t Route::SegmentAfter::first() const { return route_.visits[start]; }
size_t Route::SegmentAfter::last() const { return route_.visits.back(); }
size_t Route::SegmentAfter::size() const { return route_.size() - start; }

bool Route::SegmentAfter::startsAtReloadDepot() const
{
    return route_.nodes[start]->isReloadDepot();
}
bool Route::SegmentAfter::endsAtReloadDepot() const { return false; }

Route const *Route::SegmentBetween::route() const { return &route_; }

size_t Route::SegmentBetween::first() const { return route_.visits[start]; }
size_t Route::SegmentBetween::last() const { return route_.visits[end]; }
size_t Route::SegmentBetween::size() const { return end - start + 1; }

bool Route::SegmentBetween::startsAtReloadDepot() const
{
    return route_.nodes[start]->isReloadDepot();
}
bool Route::SegmentBetween::endsAtReloadDepot() const
{
    return route_.nodes[end]->isReloadDepot();
}

Distance Route::SegmentBetween::distance(size_t profile) const ////该函数用于计算路线中两个指定点之间的距离
{
    if (profile != route_.profile()) //  检查当前配置文件是否与路线的配置文件匹配,如果不匹配，则需要从头计算距离段
    {
        auto const &mat = route_.data.distanceMatrix(profile); //  获取指定配置文件的距离矩阵
        Distance distance = 0;

        for (size_t step = start; step != end; ++step)
        {
            auto const from = route_.visits[step];
            auto const to = route_.visits[step + 1];
            distance += mat(from, to);
        }

        return distance;
    }

    auto const startDist = route_.cumDist[start]; //  如果配置文件匹配，则使用预先计算的距离
    auto const endDist = route_.cumDist[end];

    assert(startDist <= endDist);
    return endDist - startDist;
}

DurationSegment
Route::SegmentBetween::duration([[maybe_unused]] size_t profile) const //该函数用于计算路线中两个指定点之间的持续时间段
{
    auto const &mat = route_.data.durationMatrix(profile);
    auto durSegment = route_.durAt[start];

    for (size_t step = start; step != end; ++step) //  遍历路线段中的每一步
    {
        auto const from = route_.visits[step]; //  获取当前访问点和下一个访问点
        auto const to = route_.visits[step + 1];
        auto const &durAt = route_.durAt[step + 1]; //  获取下一个点的持续时间
        durSegment = DurationSegment::merge(mat(from, to), durSegment, durAt); //  合并当前持续时间段、两点间的持续时间和下一个点的持续时间
    }

    return durSegment; //  返回路段持续时间
}

LoadSegment Route::SegmentBetween::load(size_t dimension) const //获取路径中指定维度上从起始点到结束点的负载段
{
    auto const &loads = route_.loadAt[dimension];

    auto loadSegment = loads[start];
    for (size_t step = start; step != end; ++step)
        loadSegment = LoadSegment::merge(loadSegment, loads[step + 1]);

    return loadSegment;
}

bool Route::isFeasible() const
{
    assert(!dirty);
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance();
}

bool Route::hasExcessLoad() const //检查路由是否存在超额负载。
{
    assert(!dirty);
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

bool Route::hasExcessDistance() const
{
    assert(!dirty);
    return excessDistance() > 0;
}

bool Route::hasTimeWarp() const
{
    assert(!dirty);
    return timeWarp() > 0;
}

size_t Route::idx() const { return idx_; } //获取路由对象的索引值

Route::Node *Route::operator[](size_t idx) //获取路由节点集合中指定索引的节点元素
{
    assert(idx < nodes.size());
    return nodes[idx];
}

Route::Node const *Route::operator[](size_t idx) const //获取路由中指定索引位置的节点
{
    assert(idx < nodes.size());
    return nodes[idx];
}

std::vector<Load> const &Route::load() const
{
    assert(!dirty);
    return load_;
}

std::vector<Load> const &Route::excessLoad() const
{
    assert(!dirty);
    return excessLoad_;
}

Distance Route::excessDistance() const
{
    assert(!dirty);
    return excessDistance_;
}

std::vector<Load> const &Route::capacity() const
{
    return vehicleType_.capacity;
}

size_t Route::startDepot() const { return vehicleType_.startDepot; }

size_t Route::endDepot() const { return vehicleType_.endDepot; }

Cost Route::fixedVehicleCost() const { return vehicleType_.fixedCost; }

Distance Route::distance() const
{
    assert(!dirty);
    return distance_;
}

Cost Route::distanceCost() const
{
    assert(!dirty);
    return distanceCost_;
}

Cost Route::unitDistanceCost() const { return vehicleType_.unitDistanceCost; }

bool Route::hasDistanceCost() const
{
    return unitDistanceCost() != 0
           || maxDistance() != std::numeric_limits<Distance>::max();
}

Duration Route::duration() const
{
    assert(!dirty);
    return duration_;
}

Duration Route::overtime() const
{
    assert(!dirty);
    return std::max<Duration>(duration() - shiftDuration(), 0);
}

Cost Route::durationCost() const
{
    assert(!dirty);
    return durationCost_;
}

Cost Route::unitDurationCost() const { return vehicleType_.unitDurationCost; }

Cost Route::unitOvertimeCost() const { return vehicleType_.unitOvertimeCost; }

bool Route::hasDurationCost() const
{
    // clang-format off
    return data.hasTimeWindows()
        || unitDurationCost() != 0
        || (unitOvertimeCost() != 0 && maxOvertime() != 0)
        || maxDuration() != std::numeric_limits<Duration>::max();
    // clang-format on
}

Duration Route::shiftDuration() const { return vehicleType_.shiftDuration; }

Duration Route::maxDuration() const { return vehicleType_.maxDuration; }

Duration Route::maxOvertime() const { return vehicleType_.maxOvertime; }

Distance Route::maxDistance() const { return vehicleType_.maxDistance; }

Duration Route::timeWarp() const
{
    assert(!dirty);
    return timeWarp_;
}

size_t Route::profile() const { return vehicleType_.profile; }

bool Route::empty() const { return numClients() == 0; }

size_t Route::size() const { return nodes.size(); }

size_t Route::numClients() const { return size() - numDepots(); }

size_t Route::numDepots() const { return depots_.size(); }

size_t Route::numTrips() const { return depots_.size() - 1; }

size_t Route::maxTrips() const { return vehicleType_.maxTrips(); }

Route::SegmentBetween Route::at(size_t idx) const
{
    assert(!dirty);
    return {*this, idx, idx};
}

Route::SegmentAfter Route::after(size_t start) const
{
    assert(!dirty);
    return {*this, start};
}

Route::SegmentBefore Route::before(size_t end) const
{
    assert(!dirty);
    return {*this, end};
}

Route::SegmentBetween Route::between(size_t start, size_t end) const
{
    assert(!dirty);
    return {*this, start, end};
}

template <Segment... Segments>
Route::Proposal<Segments...>::Proposal(Segments &&...segments)
    : segments_(std::forward<Segments>(segments)...)
{
    static_assert(sizeof...(Segments) > 0, "Proposal cannot be empty.");

    [[maybe_unused]] auto &&first = std::get<0>(segments_);
    [[maybe_unused]] auto &&last = std::get<sizeof...(Segments) - 1>(segments_);
    assert(first.route() == last.route());  // 必须开始和结束于同一路线

    [[maybe_unused]] auto const *route = this->route();
    assert(first.first() == route->startDepot());  // 必须开始于路线起点
    assert(last.last() == route->endDepot());      // 必须结束于路线终点
}

template <Segment... Segments> size_t Route::Proposal<Segments...>::size() const
{
    return std::apply([](auto &&...args) { return (args.size() + ...); },
                      segments_);
}

template <Segment... Segments> bool Route::Proposal<Segments...>::empty() const
{
    return size() == 2;  // 如果提议只包含起始和结束仓库，则为空
}

template <Segment... Segments>
Route const *Route::Proposal<Segments...>::route() const
{
    return std::get<0>(segments_).route();
}

template <Segment... Segments>
/**
 * 计算路线的提案（Proposal）的距离和成本
 * @return 返回一个pair，包含总成本和超出最大距离的部分
 */
std::pair<Cost, Distance> Route::Proposal<Segments...>::distance() const
{
    // 如果路线为空，直接返回0成本和0距离
    if (empty())
        return std::make_pair(0, 0);

    // 获取路线相关的各种参数
    auto const &data = route()->data;  // 路线数据
    auto const unitDistanceCost = route()->unitDistanceCost();  // 单位距离成本
    auto const maxDistance = route()->maxDistance();  // 最大允许距离
    auto const profile = route()->profile();  // 路线配置文件
    auto const &matrix = data.distanceMatrix(profile);  // 距离矩阵

    // 定义一个lambda函数，用于计算单个路段的距离和成本
    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto distance = segment.distance(profile);  // 获取当前路段的距离
        auto last = segment.last();  // 获取当前路段的最后一个点

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {//这是一个递归lambda表达式，用于合并路径或计算距离
            distance += matrix(last, other.first()) + other.distance(profile);//计算从当前段的最后一个点到下一个段的第一个点的距离，并加上下一个段的距离
            last = other.last();//更新最后一个点为下一个段的最后一个点

            if constexpr (sizeof...(args) != 0)
                self(self, std::forward<decltype(args)>(args)...);
        };

        merge(merge, std::forward<decltype(args)>(args)...);

        auto const excess = std::max<Distance>(distance - maxDistance, 0);
        auto const cost = unitDistanceCost * static_cast<Cost>(distance);
        return std::make_pair(cost, excess); //  返回一个包含成本和超出距离的pair
    };

    return std::apply(fn, segments_);
}

template <Segment... Segments>
std::pair<Cost, Duration> Route::Proposal<Segments...>::duration() const
{
    if (empty())
        return std::make_pair(0, 0);

    auto const &data = route()->data; //  路线数据
    auto const unitDurationCost = route()->unitDurationCost(); //  单位时间成本
    auto const unitOvertimeCost = route()->unitOvertimeCost(); //  单位加班成本
    auto const shiftDuration = route()->shiftDuration(); //  班次持续时间
    auto const maxDuration = route()->maxDuration(); //  最大持续时间
    auto const profile = route()->profile(); //  路线配置文件
    auto const &matrix = data.durationMatrix(profile); //  持续时间矩阵

    // 使用持续时间段完成计算代价高。但是finaliseFront比finaliseBack成本低得多。
    // 为了使用它，我们反向迭代段（从右到左，而不是默认的从左到右）。
    auto const fn = [&](auto &&segment, auto &&...args) 
    {//定义lambda函数用于处理每个段
        auto ds = segment.duration(profile); //  获取段的持续时间
        auto first = segment.first(); //  获取段的第一个元素

        if (segment.startsAtReloadDepot()) //  如果当前段从补给仓库开始，则完成当前段的前部分
            ds = ds.finaliseFront();

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {//  定义一个合并函数，用于处理两个段的合并操作
            auto edgeDur = matrix(other.last(), first); //  计算从另一个段的最后一个位置到当前段第一个位置的行程时间

            if (other.endsAtReloadDepot()) //  如果另一个段结束于补给仓库
            {
                // 另一个段结束于补给仓库，所以我们前往那里并完成当前段。
                // 我们首先旅行到那里。我们需要在仓库的时间窗内结束该段，以正确考虑我们段上的任何释放时间。
                ProblemData::Depot const &depot = data.location(other.last());
                ds = DurationSegment::merge(edgeDur, {depot}, ds); //  合并行程时间和仓库信息到当前段
                ds = ds.finaliseFront(); //  完成当前段的前部分

                edgeDur = 0;  //  重置行程时间为0
            }

            ds = DurationSegment::merge(edgeDur, other.duration(profile), ds); //  合并行程时间和另一个段的持续时间信息
            first = other.first(); //  更新当前段的第一个位置为另一个段的第一个位置

            if constexpr (sizeof...(args) != 0) //  使用constexpr if在编译时检查参数包的大小是否不为0
            {
                if (other.startsAtReloadDepot() && other.size() > 1)
                    // 仅当该段包含的内容不只是仓库时。检查大小可以加快补给仓库插入的常见情况。
                    ds = ds.finaliseFront(); //  调用finaliseFront方法完成前端处理

                self(self, std::forward<decltype(args)>(args)...); //  递归调用自身，转发参数
            }
        };

        merge(merge, std::forward<decltype(args)>(args)...); //  使用forward完美转发参数，递归调用merge函数

        auto const duration = ds.duration(); //  计算持续时间
        auto const overtime = std::max<Duration>(duration - shiftDuration, 0); //  计算超时时间，如果持续时长大于轮班时长则为差值，否则为0
        auto const cost = unitDurationCost * static_cast<Cost>(duration) //  计算成本，包括常规成本和超时成本
                          + unitOvertimeCost * static_cast<Cost>(overtime);
        auto const timeWarp = ds.timeWarp(maxDuration); //  计算时间扭曲，即超过最大允许时间的部分
        return std::make_pair(cost, timeWarp); //  返回成本和时间扭曲的键值对
    };

    return std::apply(fn, detail::reverse(segments_)); //  使用apply调用函数fn，参数为反转后的segments_
}

template <Segment... Segments>
Load Route::Proposal<Segments...>::excessLoad(size_t dimension) const
{
    if (empty())
        return 0;

    auto const &capacities = route()->capacity();
    auto const capacity = capacities[dimension];

    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto ls = segment.load(dimension);
        if (segment.endsAtReloadDepot())
            ls = ls.finalise(capacity);

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            if (other.startsAtReloadDepot())
                ls = ls.finalise(capacity);

            ls = LoadSegment::merge(ls, other.load(dimension));

            if constexpr (sizeof...(args) != 0)
            {
                if (other.endsAtReloadDepot() && other.size() > 1)
                    // 仅当该段包含的内容不只是仓库时。检查大小可以加快补给仓库插入的常见情况。
                    ls = ls.finalise(capacity);

                self(self, std::forward<decltype(args)>(args)...);
            }
        };

        merge(merge, std::forward<decltype(args)>(args)...);
        return ls.excessLoad(capacity);
    };

    return std::apply(fn, segments_);
}
}  // namespace pyvrp::search

// 以可读格式将路线输出到给定的ostream中
std::ostream &operator<<(std::ostream &out, pyvrp::search::Route const &route);

std::ostream &operator<<(std::ostream &out,  // 用于调试
                         pyvrp::search::Route::Node const &node);

#endif  // PYVRP_SEARCH_ROUTE_H
