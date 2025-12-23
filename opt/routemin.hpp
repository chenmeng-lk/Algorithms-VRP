// 路径最小化过程 (Route Minimization Procedure)
// 用于减少解中的路径数量，同时保持或改善解的质量
#ifndef _FILO2_ROUTEMIN_HPP_
#define _FILO2_ROUTEMIN_HPP_

#include <chrono>

#include "../base/PrettyPrinter.hpp"
#include "../base/SparseIntSet.hpp"
#include "../instance/Instance.hpp"
#include "../localsearch/LocalSearch.hpp"
#include "../movegen/MoveGenerators.hpp"
#include "../solution/Solution.hpp"

// Route minimization procedure.
// 路径最小化过程
//
// 算法思想：
// 1. 迭代地选择一些路径
// 2. 移除这些路径中的所有顾客
// 3. 尝试将顾客重新插入到其他路径中
// 4. 如果无法插入，概率性地创建新路径或保持未服务状态
// 5. 使用局部搜索优化部分解
// 6. 如果得到更好的完整解，更新最优解
//
// 目标：减少路径数量，同时保持解的质量
//
// @param instance CVRP实例
// @param source 初始解
// @param rand_engine 随机数生成器
// @param move_generators 移动生成器
// @param kmin 目标路径数量（估计的最少路径数）
// @param max_iter 最大迭代次数
// @param tolerance 成本比较的容差
// @return 优化后的解
inline cobra::Solution routemin(const cobra::Instance &instance, const cobra::Solution &source, std::mt19937 &rand_engine,
                                cobra::MoveGenerators &move_generators, int kmin, int max_iter, double tolerance) {

#ifdef VERBOSE
    auto partial_time_begin = std::chrono::high_resolution_clock::now();
    auto partial_time_end = std::chrono::high_resolution_clock::now();
#endif

    // Setup the local search engine.
    // 设置局部搜索引擎
    // 使用RVND（随机化变邻域下降）和所有可用的邻域算子
    // handle_partial_solutions=true 表示可以处理部分解（有未服务顾客的解）
    auto rvnd0 = cobra::RandomizedVariableNeighborhoodDescent</*handle_partial_solutions=*/true>(
        instance, move_generators,
        {cobra::E11,   cobra::E10,   cobra::TAILS, cobra::SPLIT, cobra::RE22B, cobra::E22,  cobra::RE20,  cobra::RE21,
         cobra::RE22S, cobra::E21,   cobra::E20,   cobra::TWOPT, cobra::RE30,  cobra::E30,  cobra::RE33B, cobra::E33,
         cobra::RE31,  cobra::RE32B, cobra::RE33S, cobra::E31,   cobra::E32,   cobra::RE32S},
        rand_engine, tolerance);
    auto local_search = cobra::VariableNeighborhoodDescentComposer(tolerance);
    local_search.append(&rvnd0);

    // We are going to use all the available move generators for during this procedure.
    // 在此过程中使用所有可用的移动生成器
    // gamma=1.0表示不使用粒度规则，考虑所有移动
    auto gamma_vertices = std::vector<int>();//需要更新的顶点列表
    auto gamma = std::vector<double>(instance.get_vertices_num(), 1.0f);
    for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
        gamma_vertices.emplace_back(i);
    }
    move_generators.set_active_percentage(gamma, gamma_vertices);

    auto best_solution = source;  // 当前最优解

    // 随机数分布
    // 均匀分布，生成0到1之间的随机数
    auto uniform_01_dist = std::uniform_real_distribution<double>(0.0f, 1.0f);
    //随机在客户中选择一个
    auto customers_distribution = std::uniform_int_distribution(instance.get_customers_begin(), instance.get_customers_end() - 1);

    // The value of `t` identifies the probability for a customer to remain unserved if it cannot be inserted into the existing routes.
    // t值控制顾客保持未服务状态的概率
    // t从1.0逐渐降低到0.01，意味着：
    // - 初期：更容易接受未服务的顾客（探索更多可能性）
    // - 后期：更倾向于将所有顾客都服务（收敛到完整解）
    const auto t_base = 1.00f;
    const auto t_end = 0.01f;
    auto t = t_base;
    auto c = std::pow(t_end / t_base, 1.0 / max_iter);  // 几何冷却系数

    // 存储被移除的顾客
    auto removed = std::vector<int>();
    removed.reserve(instance.get_customers_num());

    // 存储仍然未服务的顾客
    auto still_removed = std::vector<int>();
    still_removed.reserve(instance.get_customers_num());

    // 存储邻居路径的集合
    cobra::SparseIntSet neighbor_routes(instance.get_vertices_num());

    auto solution = best_solution;  // 当前工作解

#ifdef VERBOSE
    const auto main_opt_loop_begin_time = std::chrono::high_resolution_clock::now();

    auto printer = cobra::PrettyPrinter({{"%", cobra::PrettyPrinter::Field::Type::INTEGER, 3, " "},
                                         {"Objective", cobra::PrettyPrinter::Field::Type::INTEGER, 10, " "},
                                         {"Routes", cobra::PrettyPrinter::Field::Type::INTEGER, 6, " "},
                                         {"Iter/s", cobra::PrettyPrinter::Field::Type::REAL, 7, " "},
                                         {"Eta (s)", cobra::PrettyPrinter::Field::Type::REAL, 6, " "},
                                         {"% Inf", cobra::PrettyPrinter::Field::Type::REAL, 6, " "}});

    auto number_infeasible_solutions = 0;

#endif

    // 主优化循环，迭代1000次，当生成的新解路径数小于等于kmin时提前终止
    for (auto iter = 0; iter < max_iter; iter++) {

#ifdef VERBOSE
        // 打印进度信息
        partial_time_end = std::chrono::high_resolution_clock::now();
        const auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(partial_time_end - partial_time_begin).count();
        if (elapsed_time > 1) {

            const auto progress = 100.0f * (iter + 1.0f) / static_cast<double>(max_iter);
            const auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() -
                                                                                          main_opt_loop_begin_time)
                                             .count();
            const auto iter_per_second = static_cast<double>(iter + 1.0f) / (static_cast<double>(elapsed_seconds) + 0.01f);
            const auto remaining_iter = max_iter - iter;
            const auto estimated_rem_time = static_cast<double>(remaining_iter) / iter_per_second;
            const auto fraction_infeasible_solutions = static_cast<double>(number_infeasible_solutions) / (iter + 1.0f);

            printer.print(progress, best_solution.get_cost(), best_solution.get_routes_num(), iter_per_second, estimated_rem_time,
                          fraction_infeasible_solutions);


            partial_time_begin = std::chrono::high_resolution_clock::now();
        }
#endif

        solution.clear_svc();  // 清空SVC

        // Pick a random seed customer and use it used to identify a random route.
        // 随机选择一个种子顾客，用于确定要破坏的路径
        auto seed = cobra::Solution::dummy_vertex;
        do {
            seed = customers_distribution(rand_engine);
        } while (!solution.is_customer_in_solution(seed));//找到一个被服务的顾客
        auto selected_routes = std::vector<int>();
        selected_routes.push_back(solution.get_route_index(seed));  // 添加种子顾客所在的路径
        const auto &neighbors = instance.get_neighbors_of(seed);

        // Scan the neighbors of this seed customer to select a neighbor route.
        // 扫描种子顾客的邻居，选择一条邻居路径
        // 这样可以选择地理位置接近的两条路径进行合并
        for (auto n = 1u; n < neighbors.size(); n++) {
            const auto vertex = neighbors[n];
            if (vertex == instance.get_depot()) {
                continue;
            }
            if (!solution.is_customer_in_solution(vertex)) {//邻居客户已经被移除了，跳过
                continue;
            }
            const auto route = solution.get_route_index(vertex);
            if (route != selected_routes[0]) {//邻居客户在另一条路径上
                selected_routes.push_back(route);  // 添加邻居路径
                break;
            }
        }

        // 准备被移除顾客的列表
        removed.clear();
        removed.insert(removed.end(), still_removed.begin(), still_removed.end());  // 包含上次迭代未服务的顾客
        still_removed.clear();

        // Remove all customers from the selected routes.
        // 从选中的路径中移除所有顾客
        for (auto selected_route : selected_routes) {
            auto curr = solution.get_first_customer(selected_route);
            do {
                const auto next = solution.get_next_vertex(curr);
                solution.remove_vertex(selected_route, curr);
                removed.emplace_back(curr);
                curr = next;
            } while (curr != instance.get_depot());//遍历路径所有客户并移除
            solution.remove_route(selected_route);  // 移除空路径
        }

        // Pick an order for the removed customers.
        // 为被移除的顾客选择一个插入顺序
        // 50%概率按需求降序排序（大需求优先）
        // 50%概率随机打乱
        if (rand_engine() % 2 == 0) {
            std::sort(removed.begin(), removed.end(),
                      [&instance](auto i, auto j) { return instance.get_demand(i) > instance.get_demand(j); });
        } else {
            std::shuffle(removed.begin(), removed.end(), rand_engine);
        }

        // Tentatively find an insertion position for the removed customers.
        // 尝试为被移除的顾客找到插入位置
        for (auto i : removed) {

            auto best_route = -1;
            auto best_where = -1;
            auto best_delta = std::numeric_limits<double>::max();

            // Only consider insertion in routes serving the neighbors of the removed customer.
            // (That's not necessarily the smartest choice, especially for long routes it may not be worth considering insertion far from
            // the removed customer.)
            // 只考虑插入到服务该顾客邻居的路径中
            // 这是一种启发式策略，减少搜索空间
            const auto &neighbors = instance.get_neighbors_of(i);
            neighbor_routes.clear();
            for (int n = 1; n < static_cast<int>(neighbors.size()); n++) {
                int where = neighbors[n];
                if (where == instance.get_depot() || !solution.is_customer_in_solution(where)) continue;//邻居是车场或未服务，跳过
                neighbor_routes.insert(solution.get_route_index(where));
            }

            // Accessing the cost matrix is super expensive, cache whenever possible!
            // 访问成本矩阵很昂贵，尽可能缓存
            const auto c_i_depot = instance.get_cost(i, instance.get_depot());

            // 在邻居路径中寻找最佳插入位置
            for (auto route : neighbor_routes.get_elements()) {

                // 检查容量约束
                if (solution.get_route_load(route) + instance.get_demand(i) > instance.get_vehicle_capacity()) {
                    continue;
                }

                // Consider insertion before customer `j`.
                // 考虑在顾客j之前插入
                for (auto j = solution.get_first_customer(route); j != instance.get_depot(); j = solution.get_next_vertex(j)) {
                    const auto prev = solution.get_prev_vertex(route, j);
                    const auto delta = -solution.get_cost_prev_customer(j) + instance.get_cost(prev, i) + instance.get_cost(i, j);
                    if (delta < best_delta) {
                        best_route = route;
                        best_where = j;
                        best_delta = delta;
                    }
                }

                // Consider insertion before the depot.
                // 考虑在仓库之前插入（即路径末尾）
                const auto delta = -solution.get_cost_prev_depot(route) + instance.get_cost(solution.get_last_customer(route), i) +
                                   c_i_depot;
                if (delta < best_delta) {
                    best_route = route;
                    best_where = instance.get_depot();
                    best_delta = delta;
                }
            }

            if (best_route == -1) {
                // If we can't find an insertion position, probabilistically leave the customer unserved.
                // 如果找不到插入位置，概率性地保持未服务状态
                // r > t 时创建新路径
                // r <= t 时保持未服务（允许部分解）
                // 但如果路径数少于kmin，总是创建新路径
                const auto r = uniform_01_dist(rand_engine);
                if (r > t || solution.get_routes_num() < kmin) {
                    solution.build_one_customer_route(i);
                } else {
                    still_removed.push_back(i);  // 保持未服务
                }
            } else {
                // 找到了插入位置，执行插入
                solution.insert_vertex_before(best_route, best_where, i);
            }
        }

        // Reoptimize the (partial) solution.
        // 使用局部搜索重新优化（部分）解
        // 即使有未服务的顾客，局部搜索也可以处理（handle_partial_solutions=true）
        local_search.sequential_apply(solution);

        if (still_removed.empty()) {
            // If there are no unserved customers, let's check whether this is a good solution!
            // 如果没有未服务的顾客，检查是否得到了更好的解
            if (solution.get_cost() < best_solution.get_cost() ||
                (solution.get_cost() == best_solution.get_cost() && solution.get_routes_num() < best_solution.get_routes_num())) {

                // 更新最优解
                solution.apply_do_list1(best_solution);
                solution.clear_do_list1();
                solution.clear_undo_list1();
                assert(best_solution == solution);

                // We are satisfied when the current solution has the estimated number of routes.
                // 如果达到了目标路径数，提前终止
                if (best_solution.get_routes_num() <= kmin) {
                    goto end;
                }
            }

        } else {
            // 有未服务的顾客，这是一个不可行解

#ifdef VERBOSE
            number_infeasible_solutions++;
#endif
        }

        if (solution.get_cost() > best_solution.get_cost()) {
            // Reset to the best solution, since we don't want to spend time exploring worsening solutions.
            // 如果解变差了，回退到最优解
            // 避免在变差的解上浪费时间
            solution.apply_undo_list1(solution);
            solution.clear_do_list1();
            solution.clear_undo_list1();
            assert(solution == best_solution);

            still_removed.clear();  // 清空未服务顾客列表
        }

        t *= c;  // 降低接受未服务顾客的概率

        assert(solution.is_feasible());
    }

end:

    assert(best_solution.is_feasible());

    return best_solution;
}

#endif