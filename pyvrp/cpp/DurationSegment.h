#ifndef PYVRP_DURATIONSEGMENT_H
#define PYVRP_DURATIONSEGMENT_H

#include "Matrix.h"        // 包含矩阵数据结构，用于存储距离/时间等信息
#include "Measure.h"       // 包含测量相关定义，可能包含时间单位等
#include "ProblemData.h"   // 包含问题数据定义，如客户端、仓库、车辆类型等

#include <cassert>         // 断言宏，用于调试时检查条件
#include <iosfwd>          // 前向声明输入输出流，用于声明但不定义流操作

namespace pyvrp
{
/**
 * DurationSegment(
 *     duration: int,
 *     time_warp: int,
 *     start_early: int,
 *     start_late: int,
 *     release_time: int,
 *     cum_duration: int = 0,
 *     cum_time_warp: int = 0,
 *     prev_end_late: int = np.iinfo(np.int64).max,
 * )
 *
 * Creates a duration segment.
 * 创建一个时间段对象。
 *
 * Duration segments can be efficiently concatenated, and track statistics
 * about route and trip duration and time warp resulting from visiting clients
 * in the concatenated order.
 * 时间段可以高效地连接，并跟踪按照连接顺序访问客户点所产生的路线和行程持续时间以及时间扭曲的统计信息。
 *
 * Parameters
 * 参数
 * ----------
 * duration
 *     Total duration, including waiting time, of the current trip.
 *     当前行程的总持续时间，包括等待时间。
 * time_warp
 *     Total time warp on the current trip.
 *     当前行程的总时间扭曲。
 * start_early
 *     Earliest start time of the current trip.
 *     当前行程的最早开始时间。
 * start_late
 *     Latest start time of the current trip.
 *     当前行程的最晚开始时间。
 * release_time
 *     Earliest moment to start this trip segment at the depot.
 *     在仓库开始此行程段的最早时刻。
 * cum_duration
 *     Cumulative duration of other trips in segment.
 *     段中其他行程的累计持续时间。
 * cum_time_warp
 *     Cumulative time warp of other trips in segment.
 *     段中其他行程的累计时间扭曲。
 * prev_end_late
 *     Latest end time of the previous trip, if any. Default unconstrained.
 *     前一次行程的最晚结束时间（如果有的话）。默认为无约束。
 */
class DurationSegment
{
    Duration duration_ = 0;    // of current trip 当前行程的持续时间
    Duration timeWarp_ = 0;    // of current trip 当前行程的时间扭曲
    Duration startEarly_ = 0;  // of current trip 当前行程的最早开始时间
    Duration startLate_
        = std::numeric_limits<Duration>::max();  // of current trip 当前行程的最晚开始时间
    Duration releaseTime_ = 0;                   // of current trip 当前行程的释放时间
    Duration cumDuration_ = 0;  // cumulative, excl. current trip 累计持续时间，不包括当前行程
    Duration cumTimeWarp_ = 0;  // cumulative, excl. current trip 累计时间扭曲，不包括当前行程
    Duration prevEndLate_
        = std::numeric_limits<Duration>::max();  // of prev trip 前一次行程的最晚结束时间

public:
    [[nodiscard]] static inline DurationSegment
    merge(Duration const edgeDuration,
          DurationSegment const &first,
          DurationSegment const &second);  // 合并两个时间段，edgeDuration为连接边的持续时间

    /**
     * Finalises this segment towards the back (at the end of the segment),
     * and returns a new segment where release times have been reset, and all
     * other statistics have been suitably adjusted. This is useful with
     * multiple trips because the finalised segment can be concatenated with
     * segments of later trips.
     * 在段的后端（段的结束处）完成此段，并返回一个新的段，其中释放时间已被重置，
     * 所有其他统计信息都已适当调整。这对于多行程很有用，因为完成的段可以与后续行程的段连接。
     */
    [[nodiscard]] inline DurationSegment finaliseBack() const;

    /**
     * Finalises this segment towards the front (at the start of the segment),
     * and returns a new segment where release times have been reset, and all
     * other statistics have been suitably adjusted. This is useful with
     * multiple trips because the finalised segment can be concatenated with
     * segments of earlier trips.
     * 在段的前端（段的开始处）完成此段，并返回一个新的段，其中释放时间已被重置，
     * 所有其他统计信息都已适当调整。这对于多行程很有用，因为完成的段可以与先前行程的段连接。
     */
    [[nodiscard]] inline DurationSegment finaliseFront() const;

    /**
     * The total duration of the whole segment.
     * 整个段的总持续时间。
     */
    [[nodiscard]] inline Duration duration() const;  // 获取整个段的总持续时间

    /**
     * Returns the time warp on this whole segment. Additionally, any time warp
     * incurred by violating the maximum duration argument is also counted.
     * 返回整个段的时间扭曲。此外，违反最大持续时间参数所产生的任何时间扭曲也会被计算在内。
     *
     * Parameters
     * 参数
     * ----------
     * max_duration
     *     Maximum allowed duration, if provided. If the segment's duration
     *     exceeds this value, any excess duration is counted as time warp.
     *     Default unconstrained.
     *     允许的最大持续时间（如果提供）。如果段的持续时间超过此值，则任何超出部分都被计为时间扭曲。默认无约束。
     *
     * Returns
     * 返回值
     * -------
     * int
     *     Total time warp on this route segment.
     *     此路线段的总时间扭曲。
     */
    [[nodiscard]] inline Duration
    timeWarp(Duration maxDuration = std::numeric_limits<Duration>::max()) const;  // 获取整个段的时间扭曲，可传入最大持续时间约束

    /**
     * Earliest start time for the current trip.
     * 当前行程的最早开始时间。
     */
    [[nodiscard]] inline Duration startEarly() const;  // 获取当前行程的最早开始时间

    /**
     * Latest start time for the current trip.
     * 当前行程的最晚开始时间。
     */
    [[nodiscard]] inline Duration startLate() const;  // 获取当前行程的最晚开始时间

    /**
     * Earliest end time of the current trip.
     * 当前行程的最早结束时间。
     */
    [[nodiscard]] inline Duration endEarly() const;  // 获取当前行程的最早结束时间

    /**
     * Latest end time of the current trip.
     * 当前行程的最晚结束时间。
     */
    [[nodiscard]] inline Duration endLate() const;  // 获取当前行程的最晚结束时间

    /**
     * Latest end time of the previous trip.
     * 前一次行程的最晚结束时间。
     */
    [[nodiscard]] Duration prevEndLate() const;  // 获取前一次行程的最晚结束时间

    /**
     * Release time of the clients on the current trip of this segment.
     * 此段当前行程中客户点的释放时间。
     */
    [[nodiscard]] Duration releaseTime() const;  // 获取当前行程的释放时间

    /**
     * Slack in the route schedule. This is the amount of time by which the
     * start of the current trip can be delayed without increasing the overall
     * route duration.
     * 路线计划中的松弛时间。这是当前行程的开始可以延迟而不增加整个路线持续时间的量。
     */
    [[nodiscard]] Duration slack() const;  // 获取路线计划中的松弛时间

    DurationSegment() = default;  // default is all zero 默认构造函数，所有成员初始化为零

    // Construct from attributes of the given client.
    // 根据给定客户端的属性构造。
    DurationSegment(ProblemData::Client const &client);  // 从客户端数据构造DurationSegment

    /**
     * Construct from attributes of the given depot.
     * 根据给定仓库的属性构造。
     */
    DurationSegment(ProblemData::Depot const &depot);  // 从仓库数据构造DurationSegment

    /**
     * Construct from attributes of the given vehicle type and latest finish.
     * 根据给定车辆类型和最晚结束时间构造。
     */
    DurationSegment(ProblemData::VehicleType const &vehicleType,
                    Duration const twLate);  // 从车辆类型和最晚结束时间构造DurationSegment

    // Construct from raw data.
    // 从原始数据构造。
    inline DurationSegment(Duration duration,
                           Duration timeWarp,
                           Duration startEarly,
                           Duration startLate,
                           Duration releaseTime,
                           Duration cumDuration = 0,
                           Duration cumTimeWarp = 0,
                           Duration prevEndLate
                           = std::numeric_limits<Duration>::max());  // 使用原始参数构造DurationSegment

    // Move or copy construct from the other duration segment.
    // 从另一个时间段移动或复制构造。
    inline DurationSegment(DurationSegment const &) = default;  // 默认复制构造函数
    inline DurationSegment(DurationSegment &&) = default;       // 默认移动构造函数

    // Move or copy assign form the other duration segment.
    // 从另一个时间段移动或复制赋值。
    inline DurationSegment &operator=(DurationSegment const &) = default;  // 默认复制赋值运算符
    inline DurationSegment &operator=(DurationSegment &&) = default;       // 默认移动赋值运算符
};

DurationSegment
DurationSegment::merge([[maybe_unused]] Duration const edgeDuration,
                       [[maybe_unused]] DurationSegment const &first,
                       [[maybe_unused]] DurationSegment const &second)
{
    // Because clients' default time windows are [0, INT_MAX], the ternaries in
    // this method are carefully designed to avoid integer over- and underflow
    // issues. Be very careful when changing things here!
    // 因为客户端的默认时间窗口是[0, INT_MAX]，所以此方法中的三元运算符经过精心设计以避免整数上溢和下溢问题。更改此处时请务必小心！

    // atSecond is the time (relative to our starting time) at which we arrive
    // at the second's initial location.
    // atSecond是我们到达第二个段初始位置的时刻（相对于我们的开始时间）。
    //Δ = D(σ) - TW(σ) + δ表示在不使用时间扭曲的理想情况下，完成σ并行驶到σ'起点所花费的净时间。
    auto const atSecond = first.duration_ - first.timeWarp_ + edgeDuration;

    // Time warp increases when we arrive after the time window closes.
    // 当我们到达时间窗口关闭之后时，时间扭曲增加。
    //Δ_TW：如果σ最早结束时间（σ的最早开始时间 E(σ) + 净时间Δ）晚于σ'的最晚开始时间 L(σ')，则会产生新的时间扭曲
    auto const diffTw = first.startEarly_ + atSecond > second.startLate_
                            ? first.startEarly_ + atSecond - second.startLate_
                            : 0;

    // Wait duration increases if we arrive before the time window opens.
    // 如果我们在时间窗口打开之前到达，等待时间增加。
    //Δ_WT：如果σ'的最早开始时间 E(σ') 晚于σ最晚结束时间（σ的最晚开始时间 L(σ) + 净时间Δ），则必须插入等待时间。
    auto const diffWait = second.startEarly_ - atSecond > first.startLate_
                              ? second.startEarly_ - atSecond - first.startLate_
                              : 0;

    auto const secondLate  // new startLate for the second segment，第二个段的新最晚开始时间
        = atSecond > second.startLate_ - std::numeric_limits<Duration>::max()
              ? second.startLate_ - atSecond
              : second.startLate_;  // 第二个段的新startLate

    return {first.duration_ + second.duration_ + edgeDuration + diffWait,  // 总持续时间 = 第一段持续+第二段持续+旅行时间+可能的额外等待时间
            first.timeWarp_ + second.timeWarp_ + diffTw,                   // 总时间扭曲 = 第一段时间扭曲+第二段时间扭曲+时间扭曲增加
            std::max(first.startEarly_, second.startEarly_ - atSecond)    // 最早开始时间 = max(第一段最早开始, 第二段最早开始 - atSecond)
                - diffWait,                                                // 减去等待增加部分
            std::min(first.startLate_, secondLate) + diffTw,              // 最晚开始时间 = min(第一段最晚开始, 第二段新最晚开始) + 时间扭曲增加
            std::max(first.releaseTime_, second.releaseTime_),            // 释放时间 = max(第一段释放时间, 第二段释放时间)
            first.cumDuration_ + second.cumDuration_,                     // 累计持续时间 = 第一段累计持续+第二段累计持续
            first.cumTimeWarp_ + second.cumTimeWarp_,                     // 累计时间扭曲 = 第一段累计时间扭曲+第二段累计时间扭曲
            first.prevEndLate_                                            // 前一次行程的最晚结束时间
        };  // field is evaluated left-to-right 字段从左到右求值
}

DurationSegment DurationSegment::finaliseBack() const
{
    // We finalise this segment by taking into account the end time of the
    // previous trip, and then merging with this segment, finalised at the
    // start, because that accounts for release times and our earliest and
    // latest start (and, as a consequence, end).
    // 我们通过考虑前一次行程的结束时间来完成此段，然后与在开始处完成的此段合并，
    // 因为这会考虑释放时间以及我们的最早和最晚开始（以及因此的结束）。
    //这个虚拟段prev代表了一个约束："当前段必须在时间 prevEndLate_ 之前开始"
    DurationSegment const prev = {0, 0, 0, prevEndLate_, 0};  // 前一段，持续和时间扭曲为0，时间窗口为[0, prevEndLate_]
    DurationSegment const finalised = merge(0, prev, finaliseFront());  // 合并前一段和已完成前端处理的当前段
    //合并后的 finalised 段：已经综合考虑了前一段的结束时间约束和当前段的释放时间约束
    return {0,
            0,
            finalised.endEarly(),  // 新段的最早开始时间为已完成段的结束时间
            // The next trip is free to start at any time after this trip can
            // end, so the latest start is not constrained. However, starting
            // after our latest end will incur wait duration at the depot.
            // 下一次行程可以在此行程结束后随时开始，因此最晚开始时间不受约束。但是，在我们最晚结束之后开始将会在仓库产生等待时间。
            std::numeric_limits<Duration>::max(),  // 最晚开始时间设为最大，表示无约束
            // The next trip cannot leave the depot before we return, so we
            // impose our earliest end as a release time.
            // 下一次行程不能在我们返回之前离开仓库，因此我们将最早结束时间作为释放时间。
            finalised.endEarly(),                  // 释放时间设为已完成段的最早结束时间
            cumDuration_ + finalised.duration(),   // 累计持续时间 = 原累计持续 + 已完成段的持续
            cumTimeWarp_ + finalised.timeWarp(),   // 累计时间扭曲 = 原累计时间扭曲 + 已完成段的时间扭曲
            finalised.endLate()};                  // 前一段的最晚结束时间设为已完成段的最晚结束时间
}

DurationSegment DurationSegment::finaliseFront() const
{
    // We finalise at the start of this segment. This is pretty easy, via a
    // merge with our release times, if they are binding.
    // 我们在段的开始处完成。这相当容易，通过与释放时间合并（如果它们具有约束力）。
    DurationSegment const curr
        = {duration_, timeWarp_, startEarly_, startLate_, 0};  // 当前段，但释放时间设为0
    DurationSegment const release = {0, 0, startEarly(), startLate(), 0};  // 释放时间相关的虚段，持续和时间扭曲为0，时间窗口与当前段调整后的时间窗口相同

    return merge(0, release, curr);  // 合并释放虚段和当前段，返回完成前端处理的段
}

Duration DurationSegment::duration() const
{
    auto const duration = cumDuration_ + duration_;  // 计算累计持续加上当前段持续
    return duration + std::max<Duration>(startEarly() - prevEndLate_, 0);  // 加上开始时间与前一段最晚结束时间之间的等待时间（如果开始时间晚于前一段结束时间）
}

Duration DurationSegment::timeWarp(Duration maxDuration) const
{
    auto const timeWarp = cumTimeWarp_ + timeWarp_;  // 累计时间扭曲加上当前段时间扭曲
    auto const netDuration = duration() - timeWarp;  // 净持续时间 = 总持续 - 时间扭曲

    return timeWarp
           // Additional time warp from having to wait until release time.
           // 由于必须等待直到释放时间而产生的额外时间扭曲。
           + std::max<Duration>(releaseTime_ - startLate_, 0)
           // Max duration constraint applies only to net route duration,
           // subtracting existing time warp. Use ternary to avoid underflow.
           // 最大持续时间约束仅适用于净路线持续时间，减去现有时间扭曲。使用三元运算符避免下溢。
           + (netDuration > maxDuration ? netDuration - maxDuration : 0);
}

Duration DurationSegment::startEarly() const
{
    // When startEarly_ < releaseTime_, we need to wait until at least
    // releaseTime_ before we can start.
    // 当 startEarly_ < releaseTime_ 时，我们需要等到至少 releaseTime_ 才能开始。
    return std::max(startEarly_, releaseTime_);  // 最早开始时间为 startEarly_ 和 releaseTime_ 中的较大值
}

Duration DurationSegment::startLate() const
{
    // When startLate_ < releaseTime_, we need to wait until releaseTime_ before
    // we can start. That wait always incurs time warp.
    // 当 startLate_ < releaseTime_ 时，我们需要等到 releaseTime_ 才能开始。这种等待总是会产生时间扭曲。
    return std::max(startLate_, releaseTime_);  // 最晚开始时间为 startLate_ 和 releaseTime_ 中的较大值
}

Duration DurationSegment::endEarly() const
{
    auto const tripDuration = duration() - cumDuration_;  // 当前行程的持续时间 = 总持续 - 累计持续
    auto const tripTimeWarp = timeWarp() - cumTimeWarp_;  // 当前行程的时间扭曲 = 总时间扭曲 - 累计时间扭曲
    return startEarly() + tripDuration - tripTimeWarp;    // 最早结束时间 = 最早开始 + 行程持续 - 行程时间扭曲
}

Duration DurationSegment::endLate() const
{
    auto const tripDuration = duration() - cumDuration_;  // 当前行程的持续时间
    auto const tripTimeWarp = timeWarp() - cumTimeWarp_;  // 当前行程的时间扭曲
    auto const netDuration = tripDuration - tripTimeWarp; // 当前行程的净持续时间
    return netDuration > std::numeric_limits<Duration>::max() - startLate()
               ? std::numeric_limits<Duration>::max()  // 如果净持续时间加上 startLate 会溢出，则返回最大值
               : startLate() + netDuration;            // 否则返回 startLate + 净持续时间
}

DurationSegment::DurationSegment(Duration duration,
                                 Duration timeWarp,
                                 Duration startEarly,
                                 Duration startLate,
                                 Duration releaseTime,
                                 Duration cumDuration,
                                 Duration cumTimeWarp,
                                 Duration prevEndLate)
    : duration_(duration),        // 初始化当前行程持续时间
      timeWarp_(timeWarp),        // 初始化当前行程时间扭曲
      startEarly_(startEarly),    // 初始化当前行程最早开始时间
      startLate_(startLate),      // 初始化当前行程最晚开始时间
      releaseTime_(releaseTime),  // 初始化释放时间
      cumDuration_(cumDuration),  // 初始化累计持续时间
      cumTimeWarp_(cumTimeWarp),  // 初始化累计时间扭曲
      prevEndLate_(prevEndLate)   // 初始化前一段最晚结束时间
{
}
}  // namespace pyvrp

std::ostream &operator<<(std::ostream &out,  // helpful for debugging 用于调试的输出运算符重载
                         pyvrp::DurationSegment const &segment);

#endif  // PYVRP_DURATIONSEGMENT_H
