#include "SwapRoutes.h"

using pyvrp::Cost;
using pyvrp::search::SwapRoutes;

Cost SwapRoutes::evaluate(Route *U,
                          Route *V,
                          CostEvaluator const &costEvaluator)
{
    stats_.numEvaluations++;

    if (U == V || U->vehicleType() == V->vehicleType())//如果路径相同或车辆类型相同，则无法交换
        return 0;

    // Evaluate swapping the routes after the two depots.
    // 评估在两条路径的起点仓库之后交换整个路径
    return op.evaluate((*U)[0], (*V)[0], costEvaluator);
}

void SwapRoutes::apply(Route *U, Route *V) const
{
    stats_.numApplications++;
    // 应用交换路径操作
    op.apply((*U)[0], (*V)[0]);
}

SwapRoutes::SwapRoutes(ProblemData const &data) : RouteOperator(data), op(data)
{
}

template <> bool pyvrp::search::supports<SwapRoutes>(ProblemData const &data)
{
    // Swapping routes has no benefit if all vehicles are the same.
    // 如果所有车辆类型相同，交换路径没有收益
    //如果车辆类型完全相同（容量、成本、速度等），交换路径只是标签互换,总成本不变
    return data.numVehicleTypes() > 1;
}
