#include "CostEvaluator.h"

#include <stdexcept>

using pyvrp::CostEvaluator;

// CostEvaluator构造函数：初始化惩罚项并验证它们非负
CostEvaluator::CostEvaluator(std::vector<double> loadPenalties,
                             double twPenalty,
                             double distPenalty)
    : loadPenalties_(std::move(loadPenalties)),  // 移动负载惩罚项列表
      twPenalty_(twPenalty),                     // 时间扭曲惩罚项
      distPenalty_(distPenalty)                  // 距离惩罚项
{
    // 验证所有负载惩罚项非负
    for (auto const penalty : loadPenalties_)
        if (penalty < 0)
            throw std::invalid_argument("load_penalties must be >= 0.");

    // 验证时间扭曲惩罚项非负
    if (twPenalty_ < 0)
        throw std::invalid_argument("tw_penalty must be >= 0.");

    // 验证距离惩罚项非负
    if (distPenalty_ < 0)
        throw std::invalid_argument("dist_penalty must be >= 0.");
}
