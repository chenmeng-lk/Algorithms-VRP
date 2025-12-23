// 破坏-重构算子 (Ruin and Recreate Operator)
// 这是一种大邻域搜索算子，通过破坏当前解并重新构造来探索解空间
#ifndef _FILO2_RUINANDRECREATE_HPP_
#define _FILO2_RUINANDRECREATE_HPP_

#include <algorithm>
#include <random>

#include "../base/SparseIntSet.hpp"
#include "../instance/Instance.hpp"
#include "../solution/Solution.hpp"

// 破坏-重构类
// 实现了一种自适应的大邻域搜索策略
class RuinAndRecreate {

public:
    // 构造函数
    // @param instance_ CVRP实例
    // @param rand_engine_ 随机数生成器
    RuinAndRecreate(const cobra::Instance& instance_, std::mt19937& rand_engine_)
        : instance(instance_)
        , rand_engine(rand_engine_)
        , boolean_dist(std::uniform_int_distribution(0, 1))  // 0或1的均匀分布
        , customers_distribution(instance.get_customers_begin(), instance.get_customers_end() - 1)  // 客户索引分布
        , rand_uniform(0, 3)  // 0-3的均匀分布，用于选择排序策略
        , routes(instance.get_vertices_num()) { }

    // 应用破坏-重构算子
    // @param solution 当前解（会被修改）
    // @param omega 每个顶点的破坏强度参数
    // @return 种子客户的索引
    int apply(cobra::Solution& solution, std::vector<int>& omega) {

        assert(solution.is_feasible());

        removed.clear();
        routes.clear();

        // 随机选择一个种子客户
        auto seed = customers_distribution(rand_engine);

        // 要移除的客户数量由omega[seed]决定
        const auto N = omega[seed];

        auto curr = seed;

        // 破坏阶段：移除N个客户
        for (auto n = 0; n < N; n++) {

            assert(curr != instance.get_depot());

            auto next = cobra::Solution::dummy_vertex;

            auto route = solution.get_route_index(curr);

            removed.push_back(curr);
            routes.insert(route);

            // 决定下一个要移除的客户
            if (solution.get_route_size(route) > 1 && boolean_dist(rand_engine)) {
                // Move within the current route.
                // 策略1：在当前路径内移动（选择前驱或后继）

                if (boolean_dist(rand_engine)) {
                    next = solution.get_next_vertex(curr);
                    if (next == instance.get_depot()) {//是车场，再往后找一个
                        next = solution.get_next_vertex(route, next);
                    }
                } else {
                    next = solution.get_prev_vertex(curr);
                    if (next == instance.get_depot()) {
                        next = solution.get_prev_vertex(route, next);
                    }
                }

            } else {
                // Jump to a neighbor route.
                // 策略2：跳转到邻居路径

                if (boolean_dist(rand_engine)) {
                    // It could be a route that we have not yet visited.
                    // 选择一个尚未访问过的邻居路径

                    for (auto m = 1u; m < instance.get_neighbors_of(curr).size(); m++) {
                        const auto neighbor = instance.get_neighbors_of(curr)[m];//curr的邻居
                        if (neighbor == instance.get_depot() || !solution.is_customer_in_solution(neighbor) ||
                            routes.contains(solution.get_route_index(neighbor))) {//车场、不在解中、路径已经访问过
                            continue;
                        }
                        next = neighbor;
                        break;
                    }

                } else {
                    // It could be any route, even one where we have already removed some customers.
                    // 选择任意邻居路径（即使已经访问过）

                    for (auto m = 1u; m < instance.get_neighbors_of(curr).size(); m++) {
                        const auto neighbor = instance.get_neighbors_of(curr)[m];
                        if (neighbor == instance.get_depot() || !solution.is_customer_in_solution(neighbor)) {
                            continue;
                        }//允许访问已经访问过的路径
                        next = neighbor;
                        break;
                    }
                }
            }

            assert(next != instance.get_depot());

            // 从解中移除当前客户
            solution.remove_vertex(route, curr);//其中有SVC缓存相关受影响顶点
            if (solution.is_route_empty(route)) {
                solution.remove_route(route);
            }

            // 如果找不到下一个客户，提前结束
            if (next == cobra::Solution::dummy_vertex) {
                break;
            }

            curr = next;
        }

        // Shuffle customers around a little.
        // 对移除的客户进行排序，使用4种不同的策略之一
        switch (rand_uniform(rand_engine)) {
        case 0:
            // 策略0：随机顺序
            std::shuffle(removed.begin(), removed.end(), rand_engine);
            break;
        case 1:
            // 策略1：按需求量降序排序（大需求优先）
            std::sort(removed.begin(), removed.end(), [this](int a, int b) { return instance.get_demand(a) > instance.get_demand(b); });
            break;
        case 2:
            // 策略2：按到仓库距离降序排序（远的优先）
            std::sort(removed.begin(), removed.end(), [this](int a, int b) {
                return instance.get_cost(a, instance.get_depot()) > instance.get_cost(b, instance.get_depot());
            });
            break;
        case 3:
            // 策略3：按到仓库距离升序排序（近的优先）
            std::sort(removed.begin(), removed.end(), [this](int a, int b) {
                return instance.get_cost(a, instance.get_depot()) < instance.get_cost(b, instance.get_depot());
            });
            break;
        }

        assert(solution.is_feasible());

        // Insert removed customers back into the solution.
        // 重构阶段：将移除的客户重新插入到解中
        for (auto customer : removed) {

            assert(customer != instance.get_depot());

            auto best_route = cobra::Solution::dummy_route;
            auto best_where = cobra::Solution::dummy_vertex;
            auto best_cost = std::numeric_limits<double>::max();

            const auto& neighbors = instance.get_neighbors_of(customer);
            routes.clear();

            // We only try to insert customers back into routes serving neighboring customers. That's not necessarily the smartest choice,
            // especially for very long routes, but seems to be working well enough.
            // 只考虑服务邻居客户的路径
            // 这是一种启发式策略，减少了搜索空间（仅当实例>1500顶点时生效）
            for (int n = 1; n < static_cast<int>(neighbors.size()); n++) {
                int where = neighbors[n];
                if (where == instance.get_depot() || !solution.is_customer_in_solution(where)) continue;//邻居不能是车场或被移除节点
                routes.insert(solution.get_route_index(where));
            }

            // 缓存到仓库的距离，避免重复计算
            const auto c_customer_depot = instance.get_cost(customer, instance.get_depot());

            // 在候选路径中寻找最佳插入位置
            for (auto route : routes.get_elements()) {

                // 检查容量约束
                if (solution.get_route_load(route) + instance.get_demand(customer) > instance.get_vehicle_capacity()) {
                    continue;
                }

                // 尝试在路径中的每个位置插入
                for (auto where = solution.get_first_customer(route); where != instance.get_depot();
                     where = solution.get_next_vertex(where)) {

                    const auto prev = solution.get_prev_vertex(where);

                    // 计算插入成本：移除(prev, where)，添加(prev, customer)和(customer, where)
                    const auto cost = -solution.get_cost_prev_customer(where) + instance.get_cost(prev, customer) +
                                      instance.get_cost(customer, where);

                    if (cost < best_cost) {

                        best_cost = cost;
                        best_route = route;
                        best_where = where;
                    }
                }

                // 尝试在路径末尾插入（在最后一个客户和仓库之间）
                const auto cost = -solution.get_cost_prev_depot(route) + instance.get_cost(solution.get_last_customer(route), customer) +
                                  c_customer_depot;

                if (cost < best_cost) {

                    best_cost = cost;
                    best_route = route;
                    best_where = instance.get_depot();
                }
            }

            // 决定是创建新路径还是插入到现有路径
            // 如果没有找到合适的路径，或者创建新路径的成本更低，则创建新路径
            if (best_route == solution.get_end_route() || (2.0 * c_customer_depot < best_cost)) {
                solution.build_one_customer_route(customer);
            } else {
                solution.insert_vertex_before(best_route, best_where, customer);// 其中将相关受影响的顶点添加到了SVC
            }

            assert(solution.is_feasible());
        }

        return seed;
    }

private:
    const cobra::Instance& instance;                        // CVRP实例
    std::mt19937& rand_engine;                              // 随机数生成器
    std::uniform_int_distribution<int> boolean_dist;        // 布尔分布（0或1）
    std::uniform_int_distribution<int> customers_distribution;  // 客户索引分布
    std::uniform_int_distribution<int> rand_uniform;        // 0-3均匀分布（用于选择排序策略）
    std::vector<int> removed;                               // 被移除的客户列表
    cobra::SparseIntSet routes;                             // 候选路径集合（稀疏集合）
};

#endif