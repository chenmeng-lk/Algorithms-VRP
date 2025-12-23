// 装箱问题(Bin Packing Problem)求解器
// 用于估计CVRP问题所需的最少路径数（车辆数）
#ifndef _FILO2_BPP_HPP_
#define _FILO2_BPP_HPP_

#include <algorithm>
#include <vector>

#include "../instance/Instance.hpp"


namespace bpp {

    // Simple greedy solution of the bin packing problem associated with the CVRP instance.
    // 使用首次适应递减(First Fit Decreasing, FFD)算法求解装箱问题
    // 该算法提供CVRP所需最少路径数的下界估计
    // 算法步骤：
    // 1. 按需求量降序排列所有客户
    // 2. 依次将每个客户放入第一个能容纳它的箱子（路径）
    // 3. 如果所有现有箱子都无法容纳，则开启新箱子
    inline int greedy_first_fit_decreasing(const cobra::Instance& instance) {

        // 创建客户列表
        std::vector<int> customers(instance.get_customers_num());
        for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {
            customers[i - 1] = i;
        }

        // 按需求量降序排序客户
        std::sort(customers.begin(), customers.end(),
                  [&instance](auto i, auto j) { return instance.get_demand(i) > instance.get_demand(j); });

        // bins[i]表示第i个箱子（路径）的当前负载
        std::vector<int> bins(instance.get_customers_num(), 0);

        int used_bins = 0;
        // 依次处理每个客户
        for (auto i : customers) {
            const int demand = instance.get_demand(i);
            // 尝试将客户放入第一个能容纳它的箱子
            for (int p = 0; p < static_cast<int>(bins.size()); p++) {
                if (bins[p] + demand <= instance.get_vehicle_capacity()) {
                    bins[p] += demand;
                    if (p + 1 > used_bins) {
                        used_bins = p + 1;
                    }
                    break;
                }
            }
        }

        return used_bins;
    }

}  // namespace bpp


#endif