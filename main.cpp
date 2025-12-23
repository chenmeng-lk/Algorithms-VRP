// FILO2算法主程序 - 用于求解带容量约束的车辆路径问题(CVRP)
// 该算法结合了多种优化技术：Clarke-Wright初始解构造、路径最小化、局部搜索和模拟退火

#include <fstream>

#include "Parameters.hpp"
#include "base/PrettyPrinter.hpp"
#include "base/Timer.hpp"
#include "base/Welford.hpp"
#include "instance/Instance.hpp"
#include "localsearch/LocalSearch.hpp"
#include "movegen/MoveGenerators.hpp"
#include "opt/RuinAndRecreate.hpp"
#include "opt/SimulatedAnnealing.hpp"
#include "opt/bpp.hpp"
#include "opt/routemin.hpp"
#include "solution/Solution.hpp"
#include "solution/savings.hpp"

#ifdef GUI
    #include "Renderer.hpp"
#endif

// 从完整路径中提取文件名
auto get_basename(const std::string& pathname) -> std::string {
    return {std::find_if(pathname.rbegin(), pathname.rend(), [](char c) { return c == '/'; }).base(), pathname.end()};
}


// Few notes:
// - Inputs are never checked when the solver is compiled in release mode. There are just a lot of assertions checked in debug mode.
int main(int argc, char* argv[]) {

#ifndef NDEBUG
    std::cout << "******************************\n";
    std::cout << "Probably running in DEBUG mode\n";
    std::cout << "******************************\n\n";
#endif

    // 全局计时器，用于记录整个算法的运行时间
    cobra::Timer global_timer;
#ifdef VERBOSE
    cobra::Timer timer;
#endif

    // 解析命令行参数
    const auto params = Parameters(argc, argv);

#ifdef VERBOSE
    std::cout << "Pre-processing the instance.\n";
    timer.reset();
#endif
    // 从文件加载CVRP实例，并预计算每个顶点的邻居列表
    std::optional<cobra::Instance> maybe_instance = cobra::Instance::make(params.get_instance_path(), params.get_neighbors_num());
#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n\n";
#endif
    // 如果实例加载失败，退出程序
    if (!maybe_instance.has_value()) {
        return EXIT_FAILURE;
    }

    const cobra::Instance instance = std::move(maybe_instance.value());//复制实例对象

    // 创建解对象，用于存储最优解
    // 解对象包含一个LRU缓存，用于跟踪最近修改的顶点
    auto best_solution = cobra::Solution(instance, std::min(instance.get_vertices_num(), params.get_solution_cache_size()));

#ifdef VERBOSE
    std::cout << "Running CLARKE&WRIGHT to generate an initial solution.\n";
    timer.reset();
#endif
    // 使用Clarke-Wright节约算法构造初始解
    // 这是一个经典的启发式算法，通过合并路径来减少总成本
    cobra::clarke_and_wright(instance, best_solution, params.get_cw_lambda(), params.get_cw_neighbors());
#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n";
    std::cout << "Initial solution: obj = " << best_solution.get_cost() << ", n. of routes = " << best_solution.get_routes_num() << ".\n\n";
#endif

    // k是粒度规则中使用的邻居数量，用于稀疏化搜索空间，默认25个
    auto k = params.get_sparsification_rule_neighbors();

#ifdef VERBOSE
    std::cout << "Setting up MOVEGENERATORS data structures.\n";
    timer.reset();
#endif

    // 移动生成器：基于k近邻的稀疏化策略
    // 只考虑每个顶点的k个最近邻居，而不是所有可能的边
    // 这大大减少了搜索空间，同时保持解的质量
    auto move_generators = cobra::MoveGenerators(instance, k);

#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n";
    const auto tot_arcs = static_cast<unsigned long>(instance.get_vertices_num()) * static_cast<unsigned long>(instance.get_vertices_num());
    const auto move_gen_num = move_generators.size();
    const auto move_gen_perc = 100.0 * static_cast<double>(move_gen_num) / static_cast<double>(tot_arcs);
    std::cout << "Using at most " << move_generators.size() << " move-generators out of " << tot_arcs << " total arcs ";
    std::cout << std::fixed;
    std::cout << std::setprecision(5);
    std::cout << "(approx. " << move_gen_perc << "%)\n\n";
    std::cout << std::defaultfloat;
#endif


#ifdef VERBOSE
    std::cout << "Computing a greedy upper bound on the n. of routes.\n";
    timer.reset();
#endif

    // 使用装箱问题(BPP)的首次适应递减算法估计最少需要的路径数
    // 这为路径最小化过程提供了一个目标
    auto kmin = bpp::greedy_first_fit_decreasing(instance);

#ifdef VERBOSE
    std::cout << "Done in " << timer.elapsed_time<std::chrono::milliseconds>() << " milliseconds.\n";
    std::cout << "Around " << kmin << " routes should do the job.\n\n";
#endif

    // 随机数生成器，使用指定的种子以确保可重复性
    auto rand_engine = std::mt19937(params.get_seed());
    // 容差值，用于判断解的改进是否显著
    const auto tolerance = params.get_tolerance();

    // 如果估计的最小路径数小于当前解的路径数，则运行路径最小化过程
    if (kmin < best_solution.get_routes_num()) {

        const auto routemin_iterations = params.get_routemin_iterations();// // 路径最小化迭代次数

#ifdef VERBOSE
        std::cout << "Running ROUTEMIN heuristic for at most " << routemin_iterations << " iterations.\n";
        std::cout << "Starting solution: obj = " << best_solution.get_cost() << ", n. of routes = " << best_solution.get_routes_num()
                  << ".\n";
        timer.reset();
#endif

        // 路径最小化：尝试减少路径数量，同时保持或改进解的质量
        // 该过程允许部分不可行解（某些客户未被服务），逐步构建完整解
        best_solution = routemin(instance, best_solution, rand_engine, move_generators, kmin, routemin_iterations, tolerance);

#ifdef VERBOSE
        std::cout << "Final solution: obj = " << best_solution.get_cost() << ", n. routes = " << best_solution.get_routes_num() << "\n";
        std::cout << "Done in " << timer.elapsed_time<std::chrono::seconds>() << " seconds.\n\n";
#endif
    }

    // 设置局部搜索算法
    // RVND0: 随机化变邻域下降算法，包含多种邻域算子
    // 邻域算子包括：
    // - E11, E10: 1-1和1-0交换
    // - TAILS: 尾部交换
    // - SPLIT: 分割操作
    // - RE22B, RE22S: 反向2-2交换（大邻域和小邻域）
    // - E22, E21, E20: 2-2, 2-1, 2-0交换
    // - TWOPT: 2-opt操作
    // - RE30, E30: 反向3-0和3-0交换
    // - RE33B, E33: 反向3-3和3-3交换
    // - RE31, E31: 反向3-1和3-1交换
    // - RE32B, RE32S, E32: 反向3-2和3-2交换
    auto rvnd0 = cobra::RandomizedVariableNeighborhoodDescent(//只能处理完整解
        instance, move_generators,
        {cobra::E11,   cobra::E10,   cobra::TAILS, cobra::SPLIT, cobra::RE22B, cobra::E22,  cobra::RE20,  cobra::RE21,
         cobra::RE22S, cobra::E21,   cobra::E20,   cobra::TWOPT, cobra::RE30,  cobra::E30,  cobra::RE33B, cobra::E33,
         cobra::RE31,  cobra::RE32B, cobra::RE33S, cobra::E31,   cobra::E32,   cobra::RE32S},
        rand_engine, tolerance);

    // RVND1: 使用弹射链(Ejection Chain)邻域算子
    // 弹射链是一种更复杂的邻域结构，可以进行更深入的搜索
    auto rvnd1 = cobra::RandomizedVariableNeighborhoodDescent(instance, move_generators, {cobra::EJCH}, rand_engine, tolerance);

    // 组合多个VND层次，形成分层局部搜索
    auto local_search = cobra::VariableNeighborhoodDescentComposer(tolerance);
    local_search.append(&rvnd0);
    local_search.append(&rvnd1);


    // 核心优化阶段的迭代次数
    const auto coreopt_iterations = params.get_coreopt_iterations();

    // 工作解，用于在优化过程中进行修改
    auto neighbor = best_solution;

    // Gamma参数：控制粒度规则的动态调整
    // gamma_base是初始值，gamma存储每个顶点的当前gamma值
    // gamma值越大，考虑的邻居越多（搜索空间越大）
    const auto gamma_base = params.get_gamma_base();
    auto gamma = std::vector<double>(instance.get_vertices_num(), gamma_base);
    auto gamma_counter = std::vector<int>(instance.get_vertices_num(), 0);//记录顶点未改进的迭代次数

    // Delta参数：用于计算gamma更新的阈值
    const auto delta = params.get_delta();
    // Welford算法：在线计算访问顶点数量的均值和方差
    auto average_number_of_vertices_accessed = cobra::Welford();

    // 初始化所有顶点的gamma值
    auto gamma_vertices = std::vector<int>();//需要更新移动生成器百分比的顶点列表
    for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
        gamma_vertices.emplace_back(i);
    }
    // 根据gamma值设置移动生成器的活跃百分比
    move_generators.set_active_percentage(gamma, gamma_vertices);

    // 存储被破坏（移除）的客户
    auto ruined_customers = std::vector<int>();

    // 破坏-重构算子：从解中移除一些客户，然后重新插入
    auto rr = RuinAndRecreate(instance, rand_engine);

    // 扰动强度的下界和上界因子
    // 用于控制破坏-重构过程的强度
    const auto intensification_lb = params.get_shaking_lb_factor();
    const auto intensification_ub = params.get_shaking_ub_factor();

    // 计算解中平均每条弧的成本，cost(S)/(|V|+2*|S|)
    // 用于动态调整扰动强度
    const auto mean_solution_arc_cost = neighbor.get_cost() / (static_cast<double>(instance.get_customers_num()) +
                                                               2.0 * static_cast<double>(neighbor.get_routes_num()));

    auto shaking_lb_factor = mean_solution_arc_cost * intensification_lb;
    auto shaking_ub_factor = mean_solution_arc_cost * intensification_ub;

#ifdef VERBOSE
    std::cout << "Shaking LB = " << shaking_lb_factor << "\n";
    std::cout << "Shaking UB = " << shaking_ub_factor << "\n";
#endif

    // Omega参数：控制破坏-重构过程中移除的客户数量
    // omega_base是基础值，omega存储每个顶点的当前omega值，最小是1，大规模问题是log|V|
    // omega值越大，移除的客户越多（扰动强度越大）
    const auto omega_base = std::max(1, static_cast<int>(std::ceil(std::log(instance.get_vertices_num()))));
    auto omega = std::vector<int>(instance.get_vertices_num(), omega_base);
    auto random_choice = std::uniform_int_distribution(0, 1);

    // 计算模拟退火的初始温度
    // 通过随机采样一些边的成本来估计合适的温度范围
    auto vertices_dist = std::uniform_int_distribution(instance.get_vertices_begin(), instance.get_vertices_end() - 1);//设置顶点索引范围
    cobra::Welford welf;
    for (int i = 0; i < instance.get_vertices_num(); ++i) {//采样|V|个弧成本
        welf.update(instance.get_cost(vertices_dist(rand_engine), vertices_dist(rand_engine)));
    }

    // 模拟退火参数
    // 初始温度 = 平均边成本 × 初始因子0.1
    // 最终温度 = 初始温度 × 最终因子0.01
    const auto sa_initial_temperature = welf.get_mean() * params.get_sa_initial_factor();
    const auto sa_final_temperature = sa_initial_temperature * params.get_sa_final_factor();

    // 模拟退火接受准则：用于决定是否接受较差的解
    //初始化模拟退火参数
    auto sa = cobra::SimulatedAnnealing(sa_initial_temperature, sa_final_temperature, rand_engine, coreopt_iterations);

#ifdef VERBOSE
    std::cout << "Simulated annealing temperature goes from " << sa_initial_temperature << " to " << sa_final_temperature << ".\n\n";
#endif


#ifdef VERBOSE
    std::cout << "Running COREOPT for " << coreopt_iterations << " iterations.\n";

    // 统计信息收集器
    auto welford_rac_before_shaking = cobra::Welford();  // 扰动前访问的客户数
    auto welford_rac_after_shaking = cobra::Welford();   // 扰动后访问的客户数
    auto welford_local_optima = cobra::Welford();        // 局部最优解的成本
    auto welford_shaken_solutions = cobra::Welford();    // 扰动后解的成本

    // 美化输出表格
    auto printer = cobra::PrettyPrinter({{"%", cobra::PrettyPrinter::Field::Type::REAL, 5, " "},
                                         {"Iterations", cobra::PrettyPrinter::Field::Type::INTEGER, 10, " "},
                                         {"Objective", cobra::PrettyPrinter::Field::Type::INTEGER, 10, " "},
                                         {"Routes", cobra::PrettyPrinter::Field::Type::INTEGER, 6, " "},
                                         {"Iter/s", cobra::PrettyPrinter::Field::Type::REAL, 10, " "},
                                         {"Eta (s)", cobra::PrettyPrinter::Field::Type::REAL, 10, " "},
                                         {"RR (micro)", cobra::PrettyPrinter::Field::Type::REAL, 10, " "},
                                         {"LS (micro)", cobra::PrettyPrinter::Field::Type::REAL, 10, " "},
                                         {"Gamma", cobra::PrettyPrinter::Field::Type::REAL, 5, " "},
                                         {"Omega", cobra::PrettyPrinter::Field::Type::REAL, 6, " "},
                                         {"Temp", cobra::PrettyPrinter::Field::Type::REAL, 6, " "}});

    auto elapsed_minutes = 0UL;
    timer.reset();
    cobra::Timer coreopt_timer;

    cobra::Welford welford_rr;  // 破坏-重构的运行时间统计
    cobra::Welford welford_ls;  // 局部搜索的运行时间统计

#endif


#ifdef GUI
    auto renderer = Renderer(instance, neighbor.get_cost());
#endif

    // Cost of the working solution, from which neighbor is obtained after shaking and local search.
    // 参考解的成本，用于模拟退火的接受准则
    // neighbor是从reference_solution经过扰动和局部搜索得到的
    double reference_solution_cost = neighbor.get_cost();

    // 核心优化循环：迭代地应用破坏-重构和局部搜索
    for (auto iter = 0; iter < coreopt_iterations; iter++) {

        // 撤销上一次迭代的修改，恢复到参考解
        neighbor.apply_undo_list1(neighbor);
        neighbor.clear_do_list1();
        neighbor.clear_undo_list1();
        // 清空最近修改的顶点集合(SVC - Set of Visited Customers)
        neighbor.clear_svc();

#ifdef VERBOSE
        if (global_timer.elapsed_time<std::chrono::minutes>() >= elapsed_minutes + 5) {
            printer.notify("Optimizing for " + std::to_string(global_timer.elapsed_time<std::chrono::minutes>()) + " minutes.");
            elapsed_minutes += 5;
        }

        cobra::Timer rr_timer;
#endif

        // 应用破坏-重构算子
        // walk_seed是随机选择的种子客户，用于开始破坏过程
        // omega控制移除的客户数量
        //相关受影响顶点被添加到了SVC
        const auto walk_seed = rr.apply(neighbor, omega);


#ifdef VERBOSE
        const auto rr_time = rr_timer.elapsed_time<std::chrono::microseconds>();
        welford_rr.update(rr_time);
#endif

#ifdef GUI
        const auto shaken_solution_cost = neighbor.get_cost();
#endif

        // 收集被破坏（移除）的客户列表
        // SVC包含所有在破坏-重构过程中被影响的顶点
        ruined_customers.clear();
        for (auto i = neighbor.get_svc_begin(); i != neighbor.get_svc_end(); i = neighbor.get_svc_next(i)) {
            ruined_customers.emplace_back(i);
        }

#ifdef VERBOSE
        welford_rac_after_shaking.update(static_cast<double>(neighbor.get_svc_size()));
        welford_shaken_solutions.update(neighbor.get_cost());

        cobra::Timer ls_timer;
#endif

        // 应用局部搜索改进解
        local_search.sequential_apply(neighbor);

#ifdef VERBOSE
        const auto ls_time = ls_timer.elapsed_time<std::chrono::microseconds>();
        welford_ls.update(ls_time);
#endif

#ifdef GUI

        const auto local_optimum_cost = neighbor.get_cost();
#endif

        // 更新访问顶点数量的统计信息
        average_number_of_vertices_accessed.update(static_cast<double>(neighbor.get_svc_size()));

        // 计算gamma更新的阈值
        // 如果一个顶点在max_non_improving_iterations次迭代中没有改进，则增加其gamma值
        auto max_non_improving_iterations = static_cast<int>(std::ceil(delta * static_cast<double>(coreopt_iterations) *
                                                                       static_cast<double>(average_number_of_vertices_accessed.get_mean()) /    
                                                                       static_cast<double>(instance.get_vertices_num())));
                                                                        //先前局部搜索执行后缓存顶点的平均数量 、|V|
#ifdef GUI
        if (iter % 1000 == 0) {
            renderer.draw(best_solution, neighbor.get_svc(), move_generators);
        }
#endif

#ifdef VERBOSE
        welford_rac_before_shaking.update(static_cast<double>(neighbor.get_svc_size()));
        welford_local_optima.update(neighbor.get_cost());
#endif

        bool improved_best_solution;

        // 检查是否找到了更好的解
        if (neighbor.get_cost() < best_solution.get_cost()) {

            // best_solution = solution;

            improved_best_solution = true;

            // 将neighbor的修改应用到best_solution
            // do_list2包含之前接受的修改，do_list1包含最新的修改
            neighbor.apply_do_list2(best_solution);
            neighbor.apply_do_list1(best_solution);  // latest changes
            neighbor.clear_do_list2();

            assert(best_solution == neighbor);

            // 重置修改过的顶点的gamma值
            // 这是一种强化策略：当找到更好的解时，减小搜索范围
            gamma_vertices.clear();
            for (auto i = neighbor.get_svc_begin(); i != neighbor.get_svc_end(); i = neighbor.get_svc_next(i)) {
                gamma[i] = gamma_base;
                gamma_counter[i] = 0;
                gamma_vertices.emplace_back(i);
            }
            move_generators.set_active_percentage(gamma, gamma_vertices);

#ifdef VERBOSE
            welford_local_optima.reset();
            welford_local_optima.update(neighbor.get_cost());
            welford_shaken_solutions.reset();
            welford_shaken_solutions.update(neighbor.get_cost());
#endif

        } else {// 如果没有改进

            improved_best_solution = false;

            // 如果没有改进，则增加未改进顶点的gamma计数器
            // 当顶点i计数器达到阈值时，增加i的gamma值（扩大搜索空间）
            for (auto i = neighbor.get_svc_begin(); i != neighbor.get_svc_end(); i = neighbor.get_svc_next(i)) {
                gamma_counter[i]++;
                if (gamma_counter[i] >= max_non_improving_iterations) {
                    gamma[i] = std::min(gamma[i] * 2.0, 1.0);//gamma最大是1
                    gamma_counter[i] = 0;
                    gamma_vertices.clear();
                    gamma_vertices.emplace_back(i);
                    move_generators.set_active_percentage(gamma, gamma_vertices);
                }
            }
        }

        // 自适应调整omega参数
        // omega控制破坏-重构过程中移除的客户数量
        const auto seed_shake_value = omega[walk_seed];//与当前种子客户关联的扰动参数值

        // 如果扰动后的解质量太差（超过上界），减少omega（减少扰动强度）
        if (neighbor.get_cost() > shaking_ub_factor + reference_solution_cost) {//delta>UB
            for (auto i : ruined_customers) {
                if (omega[i] > seed_shake_value - 1) {//向种子的新值移动，但不超出该值，防止：顶点j被一组顶点包围，后者omega很小，导致j的omega间接增大到很大（或减小到很小）
                    omega[i]--;
                }
            }
        // 如果扰动后的解质量太好（接近参考解），增加omega（增加扰动强度）
        } else if (neighbor.get_cost() >= reference_solution_cost && neighbor.get_cost() < reference_solution_cost + shaking_lb_factor) {//delta<LB
            for (auto i : ruined_customers) {
                if (omega[i] < seed_shake_value + 1) {
                    omega[i]++;
                }
            }
        // 如果扰动强度适中，随机调整omega
        } else {
            for (auto i : ruined_customers) {
                if (random_choice(rand_engine)) {
                    if (omega[i] > seed_shake_value - 1) {
                        omega[i]--;
                    }
                } else {
                    if (omega[i] < seed_shake_value + 1) {
                        omega[i]++;
                    }
                }
            }
        }

        // 模拟退火接受准则：决定是否接受当前解作为新的参考解
        if (sa.accept(reference_solution_cost, neighbor)) {

            // 如果接受了解但没有改进最优解，将do_list1追加到do_list2
            //相当于暂存中间步骤，后续步骤中改进了最优解的话就会应用这些中间步骤
            if (!improved_best_solution) {
                neighbor.append_do_list1_to_do_list2();
            }

            neighbor.clear_do_list1();
            neighbor.clear_undo_list1();//清空，这样解neighbor无法回溯到之前的参考解

            // 更新参考解成本
            reference_solution_cost = neighbor.get_cost();

            // 重新计算扰动强度的界限
            const auto updated_mean_solution_arc_cost = neighbor.get_cost() / (static_cast<double>(instance.get_customers_num()) +
                                                                               2.0 * static_cast<double>(neighbor.get_routes_num()));
            shaking_lb_factor = updated_mean_solution_arc_cost * intensification_lb;
            shaking_ub_factor = updated_mean_solution_arc_cost * intensification_ub;
        }

        // 降低模拟退火温度
        sa.decrease_temperature();

#ifdef GUI
        renderer.add_trajectory_point(shaken_solution_cost, local_optimum_cost, reference_solution_cost, best_solution.get_cost());
#endif

#ifdef VERBOSE
        // 每秒更新一次进度信息
        if (timer.elapsed_time<std::chrono::seconds>() > 1) {
            timer.reset();

            const auto progress = 100.0 * (iter + 1.0) / coreopt_iterations;
            const auto elapsed_seconds = coreopt_timer.elapsed_time<std::chrono::seconds>();
            const auto iter_per_second = static_cast<double>(iter + 1) / (static_cast<double>(elapsed_seconds) + 0.01);
            const auto remaining_iter = coreopt_iterations - iter;
            const auto estimated_rem_time = static_cast<double>(remaining_iter) / iter_per_second;

            // 计算gamma的平均值
            auto gamma_mean = 0.0;
            for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
                gamma_mean += gamma[i];
            }
            gamma_mean = (gamma_mean / static_cast<double>(instance.get_vertices_num()));

            // 计算omega的平均值
            auto omega_mean = 0.0;
            for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {
                omega_mean += omega[i];
            }
            omega_mean /= static_cast<double>(instance.get_customers_num());

            // 打印进度信息
            printer.print(progress, iter + 1, best_solution.get_cost(), best_solution.get_routes_num(), iter_per_second, estimated_rem_time,
                          welford_rr.get_mean(), welford_ls.get_mean(), gamma_mean, omega_mean, sa.get_temperature());
        }
#endif
    }//core-opt迭代结束

    // 记录总运行时间
    int global_time_elapsed = global_timer.elapsed_time<std::chrono::seconds>();

#ifdef VERBOSE
    std::cout << "\n";
    std::cout << "Best solution found:\n";
    std::cout << "obj = " << best_solution.get_cost() << ", n. routes = " << best_solution.get_routes_num() << "\n";

    std::cout << "\n";
    std::cout << "Run completed in " << global_time_elapsed << " seconds ";
#endif

    // 构造输出文件名
    const auto outfile = params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" + std::to_string(params.get_seed()) +
                         ".out";

    // 创建输出目录（如果不存在）
    std::filesystem::create_directories(params.get_outpath());

    // 保存结果到文件
    // .out文件包含目标值和运行时间
    auto out_stream = std::ofstream(outfile);
    out_stream << std::setprecision(10);
    out_stream << best_solution.get_cost() << "\t" << global_time_elapsed << "\n";
    // .vrp.sol文件包含详细的路径信息
    cobra::Solution::store_to_file(
        instance, best_solution,
        params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" + std::to_string(params.get_seed()) + ".vrp.sol");

#ifdef VERBOSE
    std::cout << "\n";
    std::cout << "Results stored in\n";
    std::cout << " - " << outfile << "\n";
    std::cout << " - "
              << params.get_outpath() + get_basename(params.get_instance_path()) + "_seed-" + std::to_string(params.get_seed()) + ".vrp.sol"
              << "\n";
#endif

    return EXIT_SUCCESS;
}