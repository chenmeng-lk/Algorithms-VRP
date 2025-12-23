#ifndef _F4D_EJECTIONCHAIN_HPP_
#define _F4D_EJECTIONCHAIN_HPP_

// 此文件实现弹射链（Ejection Chain）局部搜索算子，用于车辆路径问题（VRP）的邻域搜索。
// 通过构建一系列客户点的重定位操作来改进当前解，同时处理容量约束以保持解的可行性。

#include "../base/BinaryHeap.hpp"
#include "../base/BitMatrix.hpp"
#include "../base/SmallFlatMap.hpp"
#include "AbstractOperator.hpp"

namespace cobra {

    // 弹射链操作器类，继承自抽象操作器。通过重定位链在多个路径间移动客户点以优化解。
    // 模板参数 max_relocation_nodes 指定重定位链的最大长度（节点数）。
    template <int max_relocation_nodes = 25>
    class EjectionChain : public AbstractOperator {

    public:
        static constexpr bool is_symmetric = false;//不对称

        // 构造函数，初始化基类及成员变量。
        EjectionChain(const Instance &instance_, MoveGenerators &moves_, double tolerance_)
            : AbstractOperator(instance_, moves_, tolerance_), forbidden_i(max_relocation_nodes), forbidden_j(max_relocation_nodes) {
            relocation_nodes.resize(max_relocation_nodes);
        }//forbidden_i：禁止作为重定位起点的客户点
        // forbidden_j：禁止作为重定位目标的客户点

    protected:
        // 预处理函数，当前为空实现。
        inline void pre_processing(__attribute__((unused)) Solution &solution) override { }

        // 计算给定移动（move）的成本变化（delta值）。
        inline double compute_cost(const Solution &solution, const MoveGenerator &move) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            auto delta = 0.0;

            if (j != iNext) {//i插到j之前
                delta = -solution.get_cost_prev_vertex(iRoute, i) - solution.get_cost_prev_vertex(iRoute, iNext) +
                        this->instance.get_cost(iPrev, iNext) - solution.get_cost_prev_vertex(jRoute, j) +
                        this->instance.get_cost(jPrev, i) + moves.get_edge_cost(move);
            }

            return delta;
        }

        // 可行性检查：判断是否可以通过弹射链使移动可行。构建重定位树，搜索可行链。
        bool is_feasible(Solution &solution, const MoveGenerator &generating_move) override {

            // 重定位节点索引，表示已生成的树节点数量。
            short rni = 0;

            // 重置可行链的索引。
            feasible_rni = -1;

            // 首先检查当前生成移动是否本身就是一个可行的重定位移动。
            // 如果是，则直接应用，无需进一步搜索弹射链。
            {

                auto i = generating_move.get_first_vertex();
                auto j = generating_move.get_second_vertex();

                auto iRoute = solution.get_route_index(i, j);
                auto jRoute = solution.get_route_index(j, i);

                auto iPrev = solution.get_prev_vertex(iRoute, i);
                auto iNext = solution.get_next_vertex(iRoute, i);
                auto jPrev = solution.get_prev_vertex(jRoute, j);

                assert(j != iNext);

                relocation_nodes[rni].move = &generating_move;

                // 如果移动在同一条路径上，或者目标路径容量允许，则移动可行。
                if (iRoute == jRoute ||
                    (solution.get_route_load(jRoute) + this->instance.get_demand(i) <= this->instance.get_vehicle_capacity())) {
                    feasible_rni = 0;
                    relocation_nodes[0].predecessor = -1;
                    forbidden_i.reset(0);
                    forbidden_j.reset(0);
                    forbidden_i.set(0, iPrev);
                    forbidden_i.set(0, i);
                    forbidden_i.set(0, iNext);
                    forbidden_i.set(0, jPrev);
                    forbidden_i.set(0, j);
                    return true;
                }

                // 如果生成移动本身不可行，则开始构建重定位链。
                relocation_nodes[rni].delta_sum = generating_move.get_delta();//记录此移动的delta值

                forbidden_i.reset(rni);
                forbidden_i.set(rni, iPrev);
                forbidden_i.set(rni, jPrev);

                forbidden_j.reset(rni);
                forbidden_j.set(rni, i);
                forbidden_j.set(rni, iNext);
                forbidden_j.set(rni, j);
                //修改后的路径负载
                relocation_nodes[rni].modified_routes_loads.clear();
                relocation_nodes[rni].modified_routes_loads[iRoute] = solution.get_route_load(iRoute) - this->instance.get_demand(i);
                relocation_nodes[rni].modified_routes_loads[jRoute] = solution.get_route_load(jRoute) + this->instance.get_demand(i);
                relocation_nodes[rni].predecessor = -1;

                relo_heap.reset();
                relo_heap.insert(&relocation_nodes[rni]);
                rni++;
            }

            // 主循环：使用堆（优先队列）管理重定位节点，搜索可行链。
            while (!relo_heap.empty()) {//遍历堆中所有重定位顶点

                auto &curr = *relo_heap.get();
                const auto curr_index = index_of(relocation_nodes, curr);//数组中对应的索引

                // 获取需要移除顶点的路径（即当前移动的目标路径）。
                const auto iRoute = solution.get_route_index(curr.move->get_second_vertex());

                // 获取更新后的路径负载（该路径应在映射表中）。
                assert(curr.modified_routes_loads.count(iRoute));
                const auto iRoute_load = curr.modified_routes_loads[iRoute];

                // 扫描路径 iRoute，寻找可以移除的客户点以恢复路径可行性。
                for (auto i = solution.get_first_customer(iRoute); i != this->instance.get_depot(); i = solution.get_next_vertex(i)) {

                    // 检查移除客户点 i 是否能使路径负载满足容量约束。
                    const auto iDemand = this->instance.get_demand(i);
                    if (iRoute_load - iDemand > this->instance.get_vehicle_capacity()) {
                        continue;
                    }

                    // 检查 i 是否可以作为重定位的起点或终点（避免重复处理已涉及变更的路径段）。
                    if (forbidden_i.is_set(curr_index, i) || forbidden_j.is_set(curr_index, i)) {//i被禁止作为重定位的起点或终点
                        continue;
                    }

                    const auto iPrev = solution.get_prev_vertex(iRoute, i);
                    const auto iNext = solution.get_next_vertex(iRoute, i);

                    // 由于成本矩阵访问开销大，此处使用延迟计算与缓存。
                    bool iCost_computed = false;
                    double iCost = 0.0;

                    // 遍历涉及 i 作为第一个顶点的所有活动移动生成器。
                    for (const auto move_index : this->moves.get_move_generator_indices_involving_1st(i)) {

                        auto &move = this->moves.get(move_index);
                        assert(move.get_first_vertex() == i);

                        const auto j = move.get_second_vertex();

                        // 确保 j 可以作为重定位的目标。
                        if (j == this->instance.get_depot() || forbidden_j.is_set(curr_index, j)) {
                            continue;
                        }

                        // 重定位到不同路径，以释放当前路径空间。（当前路径负载已满，无法再添加客户点）
                        const auto jRoute = solution.get_route_index(j);
                        if (jRoute == iRoute) {
                            continue;
                        }

                        // 获取路径 jRoute 更新后的负载（可能已被修改）。
                        auto jRoute_load = [&] {
                            if (const auto &pair = curr.modified_routes_loads.find(jRoute); pair.first != 0) {//键存在
                                return pair.second;//值是负载
                            } else {
                                return solution.get_route_load(jRoute);
                            }
                        }();

                        const auto jPrev = solution.get_prev_vertex(jRoute, j);

                        // 检查移动生成器是否已计算过 delta 值（通过堆状态或缓存标志）。
                        if (move.get_heap_index() == MoveGeneratorsHeap::unheaped && !move.is_computed_for_ejch()) {

                            if (!iCost_computed) {
                                iCost = -solution.get_cost_prev_customer(i) - solution.get_cost_prev_vertex(iRoute, iNext) +
                                        this->instance.get_cost(iPrev, iNext);//移除i的成本
                                iCost_computed = true;
                            }

                            const auto correct_delta = iCost - solution.get_cost_prev_customer(j) + this->instance.get_cost(jPrev, i) +
                                                       this->moves.get_edge_cost(move);
                            move.set_delta(correct_delta);//设置正确的delta
                            move.set_computed_for_ejch(true);
                            computed_for_ejch.emplace_back(move_index);//记录move索引，后续设置为false
                        }
                        assert(std::abs(move.get_delta() - this->compute_cost(solution, move)) < 0.01);

                        // 剪枝策略：仅考虑使链持续改进的移动（delta 和为负）。
                        if (move.get_delta() + curr.delta_sum > -this->tolerance) {//这一步的delta+之前所有步的累计delta
                            continue;
                        }

                        // 至此，移动 (i, j) 可以恢复 iRoute 的可行性（可能违反 jRoute 容量）。
                        // 创建新的重定位节点。
                        relocation_nodes[rni].move = &move;//记录选择的移动
                        relocation_nodes[rni].delta_sum = curr.delta_sum + move.get_delta();//从根节点累计的delta

                        forbidden_i.overwrite(curr_index, rni);//复制之前的禁止访问的顶点
                        forbidden_i.set(rni, iPrev);//设置此次新增的禁止访问的顶点
                        forbidden_i.set(rni, jPrev);

                        forbidden_j.overwrite(curr_index, rni);
                        forbidden_j.set(rni, i);
                        forbidden_j.set(rni, iNext);
                        forbidden_j.set(rni, j);

                        relocation_nodes[rni].modified_routes_loads = curr.modified_routes_loads;
                        relocation_nodes[rni].modified_routes_loads[iRoute] = iRoute_load - iDemand;//更新表示负载值的键值对
                        relocation_nodes[rni].modified_routes_loads[jRoute] = jRoute_load + iDemand;

                        relocation_nodes[rni].predecessor = curr_index;//记录前驱节点，后面会从末尾向前遍历执行每一步移动
                        relo_heap.insert(&relocation_nodes[rni]);//这步重定位插入堆中

                        // 如果 jRoute 也满足容量约束，则找到可行链！（到这说明成本是更优的）
                        if (jRoute_load + iDemand <= this->instance.get_vehicle_capacity()) {
                            feasible_rni = rni;
                            goto end;
                        }

                        rni++;

                        if (rni == max_relocation_nodes) {//重定位链达到最大长度
                            goto end;
                        }
                    }
                }
            }

        end:
            return feasible_rni != -1;
        }

        // 执行弹射链移动：应用所有重定位操作，更新解，并标记受影响顶点。
        inline void execute(Solution &solution, __attribute__((unused)) const MoveGenerator &generating_move,
                            SparseIntSet &affected_vertices) override {

            // 收集所有受影响顶点（即链中涉及的所有顶点）。
            for (auto i : forbidden_i.get_set_entries_possibly_with_duplicates(feasible_rni)) {
                affected_vertices.insert(i);
            }
            for (auto j : forbidden_j.get_set_entries_possibly_with_duplicates(feasible_rni)) {
                affected_vertices.insert(j);
            }

            // 重置受影响顶点相关的缓存 delta 值。
            for (const int i : affected_vertices.get_elements()) {
                for (const int move_index : moves.get_move_generator_indices_involving(i)) {
                    moves.get(move_index).set_computed_for_ejch(false);
                    moves.get(move_index + 1).set_computed_for_ejch(false);
                }
            }

            // 从可行链的末尾向前遍历，依次执行每个重定位移动。
            for (auto ptr = feasible_rni; ptr != -1; ptr = relocation_nodes[ptr].predecessor) {
                const auto &move = relocation_nodes[ptr].move;

                const auto i = move->get_first_vertex();
                const auto j = move->get_second_vertex();

                const auto iRoute = solution.get_route_index(i, j);
                const auto jRoute = solution.get_route_index(j, i);

                // 更新相关顶点的状态标记（用于后续局部搜索更新）。
                this->update_bits.at(solution.get_prev_vertex(iRoute, i), UPDATE_BITS_FIRST, true);
                this->update_bits.at(i, UPDATE_BITS_FIRST, true);
                this->update_bits.at(i, UPDATE_BITS_SECOND, true);
                const auto iNext = solution.get_next_vertex(iRoute, i);
                this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
                this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
                this->update_bits.at(j, UPDATE_BITS_FIRST, true);
                this->update_bits.at(j, UPDATE_BITS_SECOND, true);
                this->update_bits.at(solution.get_prev_vertex(jRoute, j), UPDATE_BITS_FIRST, true);

                // 执行移除和插入操作。
                solution.remove_vertex(iRoute, i);
                solution.insert_vertex_before(jRoute, j, i);

                // 如果原路径变空，则移除该路径。
                if (solution.is_route_empty(iRoute)) {
                    solution.remove_route(iRoute);
                }
            }

            assert(solution.is_feasible());
        }

        // 后处理：清理弹射链计算中使用的临时缓存数据。
        void post_processing(__attribute__((unused)) Solution &solution) override {
            for (const int move_index : computed_for_ejch) {
                const int base_index = move_index & (~1);
                const int twin_index = base_index + 1;
                moves.get(base_index).set_computed_for_ejch(false);
                moves.get(twin_index).set_computed_for_ejch(false);
            }
            computed_for_ejch.clear();
        }

        // 缓存结构，用于存储顶点及其前后邻居信息，以及部分预计算成本。
        struct Cache12 {
            int v, prev, next;
            double vrem, prevrem;
        };

        // 为给定顶点准备缓存信息（正常路径情况）。
        inline Cache12 prepare_cache12(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache12();
            c.v = vertex;
            c.prev = solution.get_prev_vertex(c.v);
            c.next = solution.get_next_vertex(c.v);
            const auto route = solution.get_route_index(c.v);

            // 预计算移除顶点 v 作为重定位起点时的成本变化部分。
            c.vrem = -solution.get_cost_prev_customer(c.v) - solution.get_cost_prev_vertex(route, c.next) +
                     this->instance.get_cost(c.prev, c.next);
            // 预计算将顶点 v 作为重定位目标时的成本变化部分。
            c.prevrem = -solution.get_cost_prev_customer(c.v);

            return c;
        }

        // 为重定位到空路径或特殊情况准备缓存信息。
        inline Cache12 prepare_cache12(const Solution &solution, int vertex, int backup) {
            assert(backup != instance.get_depot());

            auto c = Cache12();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route);
            c.next = solution.get_first_customer(route);

            // 针对空路径或特殊路径的预计算。
            c.vrem = -solution.get_cost_prev_depot(route) - solution.get_cost_prev_customer(c.next) +
                     this->instance.get_cost(c.prev, c.next);
            c.prevrem = -solution.get_cost_prev_depot(route);

            return c;
        }

        // 计算一对移动（双向）的成本变化。
        inline std::pair<double, double> compute_cost_pair(const MoveGenerator &move, const struct Cache12 i, const struct Cache12 j) {

            const auto c_iv_jv = this->moves.get_edge_cost(move);
            const auto delta1 = j.v != i.next ? i.vrem + j.prevrem + this->instance.get_cost(j.prev, i.v) + c_iv_jv : 0.0;
            const auto delta2 = i.v != j.next ? j.vrem + i.prevrem + this->instance.get_cost(i.prev, j.v) + c_iv_jv : 0.0;

            return {delta1, delta2};
        }

        // 简化缓存结构，仅存储重定位起点的信息。
        struct Cache1 {
            int v, prev, next;
            double vrem;
        };

        inline Cache1 prepare_cache1(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache1();
            c.v = vertex;
            c.prev = solution.get_prev_vertex(c.v);
            c.next = solution.get_next_vertex(c.v);
            const auto route = solution.get_route_index(c.v);

            c.vrem = -solution.get_cost_prev_customer(c.v) - solution.get_cost_prev_vertex(route, c.next) +
                     this->instance.get_cost(c.prev, c.next);

            return c;
        }

        inline Cache1 prepare_cache1(const Solution &solution, int vertex, int backup) {
            assert(backup != instance.get_depot());
            auto c = Cache1();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route);
            c.next = solution.get_first_customer(route);
            c.vrem = -solution.get_cost_prev_depot(route) - solution.get_cost_prev_customer(c.next) +
                     this->instance.get_cost(c.prev, c.next);
            return c;
        }

        // 简化缓存结构，仅存储重定位目标的信息。
        struct Cache2 {
            int v, prev;
            double prevrem;
        };

        inline Cache2 prepare_cache2(const Solution &solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache2();
            c.v = vertex;
            c.prev = solution.get_prev_vertex(c.v);
            c.prevrem = -solution.get_cost_prev_customer(c.v);
            return c;
        }

        inline Cache2 prepare_cache2(const Solution &solution, int vertex, int backup) {
            assert(backup != instance.get_depot());
            auto c = Cache2();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route);
            c.prevrem = -solution.get_cost_prev_depot(route);
            return c;
        }

        // 使用缓存计算移动成本（适用于不同类型缓存的泛型版本）。
        template <typename C1, typename C2>
        inline double compute_cost(const MoveGenerator &move, const C1 i, const C2 j) {
            return j.v != i.next ? i.vrem + j.prevrem + this->instance.get_cost(j.prev, i.v) + this->moves.get_edge_cost(move) : 0.0;
        }

    private:
        static constexpr int heap_unheaped = -1;
        static constexpr auto max_chain_length = max_relocation_nodes;

        // 重定位节点结构，表示弹射链中的一个步骤。
        struct Relocation {
            short heap_index = heap_unheaped;          // 在堆中的索引
            short predecessor = 0;                     // 前驱节点索引
            double delta_sum = 0.0;                    // 累积成本变化
            const MoveGenerator *move = nullptr;       // 对应的移动
            SmallFlatMap<int, int, 0, 25> modified_routes_loads; // 修改后的路径负载映射
        };

        // 位矩阵，用于记录禁止访问的顶点（避免环路或无效移动）。
        BitMatrix<2 * max_chain_length + 3> forbidden_i; // 禁止作为重定位起点的顶点
        BitMatrix<3 * max_chain_length> forbidden_j;     // 禁止作为重定位目标的顶点

        std::vector<Relocation> relocation_nodes;        // 重定位节点数组
        short feasible_rni;                              // 可行链的终止节点索引

        std::vector<int> computed_for_ejch;              // 记录为弹射链计算过的移动索引

        // 二叉堆，用于按 delta_sum 排序重定位节点（优先队列）。
        BinaryHeapPtr<Relocation, &Relocation::heap_index, &Relocation::delta_sum> relo_heap;
        inline int index_of(std::vector<Relocation> &nodes, Relocation &node) {
            return &node - nodes.data();
        }
    };

}  // namespace cobra

#endif
