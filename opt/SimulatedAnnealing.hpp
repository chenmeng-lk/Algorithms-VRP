// 模拟退火算法辅助类
// 实现接受准则，用于决定是否接受较差的解以逃离局部最优
#ifndef _FILO2_NEIGHBORACCEPTANCE_HPP_
#define _FILO2_NEIGHBORACCEPTANCE_HPP_

#include <cmath>
#include <random>

#include "../solution/Solution.hpp"

namespace cobra {

    // Simulated annealing helper class.
    // 模拟退火辅助类
    // 使用几何冷却策略：T(k) = T(0) * factor^k
    class SimulatedAnnealing {
    public:
        // 构造函数
        // @param initial_temperature_ 初始温度
        // @param final_temperature_ 最终温度
        // @param rand_engine_ 随机数生成器
        // @param max_iter 最大迭代次数
        SimulatedAnnealing(double initial_temperature_, double final_temperature_, std::mt19937 &rand_engine_, int max_iter)
            : rand_engine(rand_engine_), uniform_dist(0.0, 1.0) {

            initial_temperature = initial_temperature_;
            final_temperature = final_temperature_;
            period = max_iter;

            temperature = initial_temperature;
            // 计算冷却因子，使得T(max_iter) = final_temperature
            factor = std::pow(final_temperature / initial_temperature, 1.0 / static_cast<double>(period));
        }

        // 降低温度（几何冷却）
        void decrease_temperature() {
            temperature *= factor;
        }

        // 接受准则：决定是否接受新解(neighbor)
        // 使用修改的Metropolis准则：
        // 如果 cost(neighbor) < cost(reference) - T * log(U(0,1))，则接受
        // 其中U(0,1)是[0,1]上的均匀分布随机数
        // 这等价于：接受概率 = exp(-(delta/T))，其中delta = cost(neighbor) - cost(reference)
        bool accept(const double reference_solution_cost, const Solution &neighbor) {
            return neighbor.get_cost() < reference_solution_cost - temperature * std::log(uniform_dist(rand_engine));
        }

        // 获取当前温度
        double get_temperature() const {
            return temperature;
        }

    private:
        double initial_temperature;  // 初始温度
        double final_temperature;    // 最终温度
        double temperature;          // 当前温度
        int period;                  // 冷却周期（迭代次数）

        std::mt19937 &rand_engine;                          // 随机数生成器
        std::uniform_real_distribution<double> uniform_dist; // 均匀分布

        double factor;  // 冷却因子
    };

}  // namespace cobra

#endif