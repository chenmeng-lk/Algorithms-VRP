// Clarke-Wright节约算法 (Clarke-Wright Savings Algorithm)
// 用于构造CVRP问题的初始解
#ifndef _FILO2_SOLUTIONALGORITHMS_HPP_
#define _FILO2_SOLUTIONALGORITHMS_HPP_

#include "../base/Timer.hpp"
#include "Solution.hpp"


namespace cobra {

    // Limited savings algorithm.
    // 限制版本的Clarke-Wright节约算法
    // 只考虑每个客户的neighbors_num个最近邻居，而不是所有客户对
    //
    // 算法原理：
    // 1. 初始解：每个客户单独成一条路径
    // 2. 计算节约值：s(i,j) = c(0,i) + c(0,j) - λ*c(i,j)
    //    其中0是仓库，λ是调节参数
    // 3. 按节约值降序排序
    // 4. 贪心合并路径：如果i是某路径的尾部，j是另一路径的头部，
    //    且合并后不违反容量约束，则合并这两条路径
    //
    // @param instance CVRP实例
    // @param solution 解对象（会被修改）
    // @param lambda 节约值计算中的权重参数
    // @param neighbors_num 每个客户考虑的邻居数量
    inline void clarke_and_wright(const Instance &instance, Solution &solution, const double lambda, int neighbors_num) {

        solution.reset();

        // 步骤1：为每个客户创建单客户路径
        for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {
            solution.build_one_customer_route</*record_acion=*/false>(i);
        }
        assert(solution.is_feasible());

        // 限制邻居数量，不大于N-1
        neighbors_num = std::min(instance.get_customers_num() - 1, neighbors_num);
        // 节约值的数量
        const auto savings_num = instance.get_customers_num() * neighbors_num;

        // 节约值结构
        struct Saving {
            int i;          // 客户i
            int j;          // 客户j
            double value;   // 节约值
        };

        auto savings = std::vector<Saving>();
        savings.reserve(static_cast<unsigned long>(savings_num));//预分配内存

        // 步骤2：计算节约值
        // 只考虑每个客户的neighbors_num个最近邻居
        for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {

            for (auto n = 1u, added = 0u; added < static_cast<unsigned int>(neighbors_num) && n < instance.get_neighbors_of(i).size();
                 n++) {

                const auto j = instance.get_neighbors_of(i)[n];//顶点i的邻居列表,第一个元素是i自己

                // 避免重复计算(i,j)和(j,i)
                if (i < j) {

                    // 节约值公式：s(i,j) = c(0,i) + c(0,j) - λ*c(i,j)
                    // 表示合并路径(0-i-0)和(0-j-0)为(0-i-j-0)节省的成本
                    const double value = +instance.get_cost(i, instance.get_depot()) + instance.get_cost(instance.get_depot(), j) -
                                         lambda * instance.get_cost(i, j);

                    savings.push_back({i, j, value});

                    added++;//额外控制邻居数量<=neighbors_num
                }
            }
        }

        // 步骤3：按节约值降序排序
        std::sort(savings.begin(), savings.end(), [](const Saving &a, const Saving &b) { return a.value > b.value; });

#ifdef VERBOSE
        Timer timer;
#endif

        // 步骤4：贪心合并路径
        for (auto n = 0; n < static_cast<int>(savings.size()); ++n) {

            const auto &saving = savings[n];

            const auto i = saving.i;
            const auto j = saving.j;

            const auto iRoute = solution.get_route_index(i);
            const auto jRoute = solution.get_route_index(j);

            // 如果i和j已经在同一条路径中，跳过
            if (iRoute == jRoute) {
                continue;
            }

            // 情况1：i是iRoute的尾部，j是jRoute的头部
            // 可以将jRoute追加到iRoute后面
            if (solution.get_last_customer(iRoute) == i && solution.get_first_customer(jRoute) == j &&
                solution.get_route_load(iRoute) + solution.get_route_load(jRoute) <= instance.get_vehicle_capacity()) {

                solution.append_route(iRoute, jRoute);


            // 情况2：j是jRoute的尾部，i是iRoute的头部
            // 可以将iRoute追加到jRoute后面
            } else if (solution.get_last_customer(jRoute) == j && solution.get_first_customer(iRoute) == i &&
                       solution.get_route_load(iRoute) + solution.get_route_load(jRoute) <= instance.get_vehicle_capacity()) {

                solution.append_route(jRoute, iRoute);
            }
            //其他情况无法合并
#ifdef VERBOSE
            if (timer.elapsed_time<std::chrono::seconds>() > 2) {
                std::cout << "Progress: " << 100.0 * (n + 1) / savings.size() << "%, Solution cost: " << solution.get_cost() << " \n";
                timer.reset();
            }
#endif
        }
        assert(solution.is_feasible());
    }

}  // namespace cobra

#endif