#include "DurationSegment.h"

#include <fstream>

using pyvrp::Duration;  // 使用pyvrp命名空间中的Duration类型，表示时间间隔
using pyvrp::DurationSegment;  // 使用pyvrp命名空间中的DurationSegment类，表示时间段信息

Duration DurationSegment::prevEndLate() const { return prevEndLate_; }  // 返回前一段行程的最晚结束时间，用于计算时间窗和松弛

Duration DurationSegment::releaseTime() const { return releaseTime_; }  // 返回释放时间，即车辆可以离开仓库/前一位置开始此行程的最早时刻

Duration DurationSegment::slack() const
{
    // We have wait duration if release time is after the end of the previous
    // trip. Starting any later only increases that wait duration, so there
    // is then definitely no slack.
    // 如果释放时间在前一趟行程结束之后，则存在等待时间；开始时间再晚只会增加等待时间，因此肯定没有松弛时间。
    auto const prevSlack = std::max<Duration>(prevEndLate_ - releaseTime_, 0);  // 计算前一段的松弛时间：前段最晚结束时间减释放时间，结果非负
    return std::min(startLate() - startEarly(), prevSlack);  // 返回当前段开始时间窗的松弛时间（最晚开始减最早开始）与前一段松弛时间的较小值，作为整体松弛时间
}

DurationSegment::DurationSegment(ProblemData::Client const &client)
    : duration_(client.serviceDuration),  // 初始化服务时长，从客户端数据获取
      startEarly_(client.twEarly),        // 初始化时间窗最早开始时间，从客户端数据获取
      startLate_(client.twLate),          // 初始化时间窗最晚开始时间，从客户端数据获取
      releaseTime_(client.releaseTime)    // 初始化释放时间，从客户端数据获取
{
    // 构造函数，根据客户端数据初始化DurationSegment对象，用于表示客户点的服务时间段
}

DurationSegment::DurationSegment(ProblemData::Depot const &depot)
    : startEarly_(depot.twEarly),  // 初始化仓库的最早开始时间，从仓库数据获取
      startLate_(depot.twLate)     // 初始化仓库的最晚开始时间，从仓库数据获取
{
    // 构造函数，根据仓库数据初始化DurationSegment对象，用于表示仓库的时间窗信息
}

DurationSegment::DurationSegment(ProblemData::VehicleType const &vehicleType,
                                 Duration const twLate)
    : startEarly_(vehicleType.twEarly),  // 初始化车辆类型的最早开始时间，从车辆类型数据获取
      startLate_(twLate)                 // 初始化传入的最晚开始时间，作为时间窗上限
{
    // 构造函数，根据车辆类型和最晚开始时间初始化DurationSegment对象，用于表示车辆相关的时间段
}

std::ostream &operator<<(std::ostream &out, DurationSegment const &segment)
{
    // clang-format off
    return out << "duration=" << segment.duration() 
               << ", time_warp=" << segment.timeWarp()
               << ", start_early=" << segment.startEarly()
               << ", start_late=" << segment.startLate()
               << ", release_time=" << segment.releaseTime()
               << ", prev_end_late=" << segment.prevEndLate();
    // clang-format on
    // 重载输出运算符，格式化输出DurationSegment对象的所有成员信息，便于调试和日志记录
}
