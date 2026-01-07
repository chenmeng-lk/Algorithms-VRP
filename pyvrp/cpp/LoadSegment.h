// 此文件定义了LoadSegment类，用于表示车辆路径中的负载段，支持高效合并和容量违规跟踪。 
#ifndef PYVRP_LOADSEGMENT_H
#define PYVRP_LOADSEGMENT_H

#include "Measure.h"
#include "ProblemData.h"

#include <iosfwd>

namespace pyvrp
{
/**
 * LoadSegment(delivery: int, pickup: int, load: int, excess_load: int = 0)
 *
 * Creates a new load segment for delivery and pickup loads in a single
 * dimension. These load segments can be efficiently concatenated, and track
 * statistics about capacity violations resulting from visiting clients in the
 * concatenated order.
 *
 * Parameters
 * ----------
 * delivery
 *     Total delivery amount on this segment.
 * pickup
 *     Total pickup amount on this segment.
 * load
 *     Maximum load on this segment.
 * excess_load
 *     Cumulative excess load on this segment, possibly from earlier trips.
 */
class LoadSegment // LoadSegment类用于管理车辆路径中的负载信息，包括交付、拾取、当前负载和超载情况，支持合并和最终化操作。 
{
    Load delivery_ = 0;    // of client demand on current trip
    Load pickup_ = 0;      // of client demand on current trip
    Load load_ = 0;        // on current trip
    Load excessLoad_ = 0;  // cumulative excess load over other trips in segment

public:
    [[nodiscard]] static inline LoadSegment merge(LoadSegment const &first,
                                                  LoadSegment const &second); // merge函数用于合并两个LoadSegment，计算合并后的交付、拾取、最大负载和超载。

    /**
     * Finalises the load on this segment, and returns a new segment where any
     * excess load has been moved to the cumulative excess load field. This is
     * useful with reloading, because the finalised segment can be concatenated
     * with load segments of subsequent trips.
     * 最终化此段的负载，并返回一个新段，其中任何超载都被移动到累积超载字段。这对于重新加载很有用，因为最终化的段可以与后续行程的负载段连接。
     */
    [[nodiscard]] inline LoadSegment finalise(Load capacity) const;

    /**
     * Returns the delivery amount, that is, the total amount of load delivered
     * to clients on this segment.
     */
    [[nodiscard]] Load delivery() const; // delivery函数返回此段的总交付量。 

    /**
     * Returns the amount picked up from clients on this segment.
     */
    [[nodiscard]] Load pickup() const; // pickup函数返回此段的总拾取量。 

    /**
     * Returns the maximum load encountered on this segment.
     */
    [[nodiscard]] Load load() const; // load函数返回此段的负载。 

    /**
     * Returns the load violation on this segment.
     *
     * Parameters
     * ----------
     * capacity
     *     Segment capacity, if any.
     */
    [[nodiscard]] inline Load excessLoad(Load capacity) const; // excessLoad函数计算并返回此段的负载违规量。 

    LoadSegment() = default;  // default is all zero // 默认构造函数，初始化所有值为零。 

    // Construct from load attributes of the given client and dimension.
    LoadSegment(ProblemData::Client const &client, size_t dimension); // 从客户端和维度构造LoadSegment。 

    // Construct from initial load attributes of the given vehicle type and
    // dimension.
    LoadSegment(ProblemData::VehicleType const &vehicleType, size_t dimension); // 从车辆类型和维度构造LoadSegment。 

    // Construct from raw data.
    inline LoadSegment(Load delivery,
                       Load pickup,
                       Load load,
                       Load excessLoad = 0); // 从原始数据构造LoadSegment。 

    // Move or copy construct from the other load segment.
    inline LoadSegment(LoadSegment const &) = default; // 默认拷贝构造函数。 
    inline LoadSegment(LoadSegment &&) = default; // 默认移动构造函数。 

    // Move or copy assign form the other load segment.
    inline LoadSegment &operator=(LoadSegment const &) = default; // 默认拷贝赋值运算符。 
    inline LoadSegment &operator=(LoadSegment &&) = default; // 默认移动赋值运算符。 
};

LoadSegment LoadSegment::merge(LoadSegment const &first,
                               LoadSegment const &second) // merge函数实现：根据Vidal等人的公式合并两个段。 
{
    // See Vidal et al. (2014) for details. This function implements equations
    // (9) -- (11) of https://doi.org/10.1016/j.ejor.2013.09.045.
    return { // 返回合并后的LoadSegment，计算交付、拾取、最大负载和超载。 
        first.delivery_ + second.delivery_,
        first.pickup_ + second.pickup_,
        std::max(first.load_ + second.delivery_, second.load_ + first.pickup_), // 计算最大负载：考虑两种可能的负载变化。 
        first.excessLoad_ + second.excessLoad_}; // 累积超载相加。 
}

Load LoadSegment::excessLoad(Load capacity) const // excessLoad函数实现：计算总超载，包括当前段和累积超载。 
{
    return excessLoad_ + std::max<Load>(load_ - capacity, 0); // 返回累积超载加上当前负载超过容量的部分。 
}

LoadSegment LoadSegment::finalise(Load capacity) const // finalise函数实现：最终化负载，将超载转移到累积字段。 
{
    return {0, 0, 0, excessLoad(capacity)}; // 返回新段，交付、拾取、负载置零，超载设置为计算值。 
}

LoadSegment::LoadSegment(Load delivery, Load pickup, Load load, Load excessLoad) // 构造函数实现：初始化成员变量。 
    : delivery_(delivery), pickup_(pickup), load_(load), excessLoad_(excessLoad) // 初始化交付、拾取、负载和超载。 
{
}
}  // namespace pyvrp

std::ostream &operator<<(std::ostream &out,  // helpful for debugging，输出运算符，用于调试输出LoadSegment。 
                         pyvrp::LoadSegment const &segment);

#endif  // PYVRP_LOADSEGMENT_H
