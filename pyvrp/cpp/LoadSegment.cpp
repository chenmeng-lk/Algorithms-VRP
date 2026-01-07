#include "LoadSegment.h"

#include <fstream>

using pyvrp::Load;
using pyvrp::LoadSegment;

Load LoadSegment::delivery() const { return delivery_; }

Load LoadSegment::pickup() const { return pickup_; }

Load LoadSegment::load() const { return load_; }

LoadSegment::LoadSegment(ProblemData::Client const &client, size_t dimension)
    : delivery_(client.delivery[dimension]),
      pickup_(client.pickup[dimension]),
      load_(std::max<Load>(delivery_, pickup_))
{
}

LoadSegment::LoadSegment(ProblemData::VehicleType const &vehicleType,
                         size_t dimension)
    :  // Initial load is always a pickup quantity: it's already on the vehicle,
       // and needs to be dropped off at a (reload) depot.
       //初始化负载量总是取货量：它已经在车上，需要在（重新）仓库中卸下。
      pickup_(vehicleType.initialLoad[dimension]),
      load_(vehicleType.initialLoad[dimension])
{
}

std::ostream &operator<<(std::ostream &out, LoadSegment const &segment)
{
    // Define 'capacity' as current load, so we can see only the cumulative
    // excess load when printing.
    //定义'capacity'为当前负载，这样我们就可以在打印时只看到累积的过剩负载。
    auto const capacity = segment.load();

    // clang-format off
    return out << "delivery=" << segment.delivery() 
               << ", pickup=" << segment.pickup()
               << ", load=" << segment.load()
               << ", excess_load=" << segment.excessLoad(capacity);
    // clang-format on
}
