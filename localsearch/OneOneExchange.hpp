#ifndef _FILO2_ONEONEEXCHANGE_HPP_
#define _FILO2_ONEONEEXCHANGE_HPP_

// OneOneExchange.hpp
// 1-1交换操作符实现文件
// 实现车辆路径问题(VRP)中的1-1交换邻域操作：交换i和j

#include "AbstractOperator.hpp"

namespace cobra {

    // 1-1交换操作符类
    // 继承自AbstractOperator，实现两个客户在不同路线间的交换操作
    class OneOneExchange : public AbstractOperator {

    public:
        // 构造函数：初始化1-1交换操作符
        OneOneExchange(const Instance& instance_, MoveGenerators& moves_, double tolerance_)
            : AbstractOperator(instance_, moves_, tolerance_) { }

        static constexpr bool is_symmetric = false; // 非对称操作符：需要同时考虑(i,j)和(j,i)

    protected:
        // 预处理函数：在局部搜索循环开始前调用（此处为空实现）
        inline void pre_processing(__attribute__((unused)) Solution& solution) override { }

        // 计算移动成本：计算应用移动后解的成本变化（仅调试用）
        inline double compute_cost(const Solution& solution, const MoveGenerator& move) override {

            const auto i = move.get_first_vertex(); // 移动的第一个顶点i
            const auto j = move.get_second_vertex(); // 移动的第二个顶点j

            const auto iRoute = solution.get_route_index(i, j); // 顶点i所在的路线
            const auto jRoute = solution.get_route_index(j, i); // 顶点j所在的路线

            const auto iPrev = solution.get_prev_vertex(iRoute, i); // i的前驱节点
            const auto iNext = solution.get_next_vertex(iRoute, i); // i的后继节点

            const auto jPrev = solution.get_prev_vertex(jRoute, j); // j的前驱节点
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev); // j的前驱的前驱节点

            // 计算移除的边的成本，移除i、j的前驱
            const auto iRem = -solution.get_cost_prev_vertex(iRoute, i) - solution.get_cost_prev_vertex(iRoute, iNext);
            const auto jPrevRem = -solution.get_cost_prev_vertex(jRoute, jPrev) - solution.get_cost_prev_vertex(jRoute, j);
            
            // 计算新增的边的成本
            const auto iAdd = +this->instance.get_cost(jPrevPrev, i) + this->instance.get_cost(i, j);
            const auto jPrevAdd = +this->instance.get_cost(iPrev, jPrev) + this->instance.get_cost(jPrev, iNext);

            return iAdd + jPrevAdd + iRem + jPrevRem; // 返回总成本变化
        }

        // 检查移动是否可行：验证容量约束和路线关系
        inline bool is_feasible(Solution& solution, const MoveGenerator& move) override {

            const auto i = move.get_first_vertex(); // 顶点i
            const auto j = move.get_second_vertex(); // 顶点j

            const auto i_route = solution.get_route_index(i, j); // i的路线索引
            const auto j_route = solution.get_route_index(j, i); // j的路线索引

            const auto j_prev = solution.get_prev_vertex(j_route, j); // j的前驱节点

            // 可行条件：
            // 1. 如果i和j在不同路线：交换i和j的前驱(j_prev)，检查容量约束
            // 2. 如果i和j在同一路线：确保i不是j的前驱，且j的前驱不是i的后继
            return (i_route != j_route && j_prev != this->instance.get_depot() &&
                    solution.get_route_load(i_route) - this->instance.get_demand(i) + this->instance.get_demand(j_prev) <=
                        this->instance.get_vehicle_capacity() &&
                    solution.get_route_load(j_route) - this->instance.get_demand(j_prev) + this->instance.get_demand(i) <=
                        this->instance.get_vehicle_capacity()) ||
                   (i_route == j_route && i != j_prev && j_prev != solution.get_next_vertex(i_route, i));
        }

        // 执行移动：实际交换两个客户，并记录受影响顶点和更新位
        inline void execute(Solution& solution, const MoveGenerator& move, SparseIntSet& storage) override {

            const auto i = move.get_first_vertex(); // 顶点i
            const auto j = move.get_second_vertex(); // 顶点j

            const auto iRoute = solution.get_route_index(i, j); // i的路线
            const auto jRoute = solution.get_route_index(j, i); // j的路线

            // 获取i相关顶点：前驱、后继、后继的后继
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            // 获取j相关顶点：前驱、前驱的前驱、后继
            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jNext = solution.get_next_vertex(jRoute, j);

            // 将受影响的顶点插入存储集合
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);

            // 设置更新位：标记哪些移动生成器需要更新
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);

            // 执行交换操作：将i插入到j之前，将j的前驱插入到i的后继之前
            solution.remove_vertex(iRoute, i);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrev);
        }

        // 后处理函数：局部搜索循环结束后的清理工作（此处为空实现）
        void post_processing(__attribute__((unused)) Solution& solution) override { }

        // 缓存结构体12：存储顶点及其前驱、后继、前驱的前驱信息，以及移除成本
        struct Cache12 {
            int v, prev, next, prevprev; // 顶点、前驱、后继、前驱的前驱
            double vrem, prevrem;        // 顶点移除成本、前驱移除成本
        };

        // 准备缓存12：为普通顶点准备缓存信息
        inline Cache12 prepare_cache12(const Solution& solution, int vertex) {

            assert(vertex != this->instance.get_depot()); // 顶点不能是depot
            auto c = Cache12();
            c.v = vertex;
            c.prev = solution.get_prev_vertex(c.v); // 前驱
            c.next = solution.get_next_vertex(c.v); // 后继
            const auto route = solution.get_route_index(c.v); // 所在路线
            c.prevprev = solution.get_prev_vertex(route, c.prev); // 前驱的前驱

            // 计算顶点作为(i,j)中i时的移除成本
            c.vrem = -solution.get_cost_prev_customer(c.v) - solution.get_cost_prev_vertex(route, c.next);
            // 计算顶点作为(i,j)中j时的前驱移除成本
            c.prevrem = -solution.get_cost_prev_vertex(route, c.prev) - solution.get_cost_prev_customer(c.v);

            return c;
        }

        // 准备缓存12（针对depot）：当顶点是depot时使用备份顶点确定路线
        inline Cache12 prepare_cache12(const Solution& solution, int vertex, int backup) {
            assert(backup != instance.get_depot()); // 备份顶点不能是depot

            auto c = Cache12();
            c.v = vertex; // 顶点（depot）
            const auto route = solution.get_route_index(backup); // 通过备份顶点确定路线
            c.prev = solution.get_last_customer(route); // 路线的最后一个客户作为前驱
            c.next = solution.get_first_customer(route); // 路线的第一个客户作为后继
            c.prevprev = solution.get_prev_vertex(c.prev); // 前驱的前驱

            // 计算depot作为i时的移除成本
            c.vrem = -solution.get_cost_prev_depot(route) - solution.get_cost_prev_customer(c.next);
            // 计算depot作为j时的前驱移除成本
            c.prevrem = -solution.get_cost_prev_customer(c.prev) - solution.get_cost_prev_depot(route);

            return c;
        }

        // 计算成本对：同时计算(i,j)和(j,i)两个移动的成本变化
        inline std::pair<double, double> compute_cost_pair(const MoveGenerator& move, const struct Cache12 i, const struct Cache12 j) {

            const auto c_iv_jv = this->moves.get_edge_cost(move); // 移动生成器中i和j之间的边成本(i,j)
            const auto c_iprev_jprev = this->instance.get_cost(i.prev, j.prev); // i前驱和j前驱之间的成本

            // 计算(i,j)移动的成本变化
            const auto delta1 = this->instance.get_cost(j.prevprev, i.v) + c_iv_jv + c_iprev_jprev +//jprevprev->i + i->j + i.prev->j.prev
                                this->instance.get_cost(j.prev, i.next) + i.vrem + j.prevrem;//+j.prev->i.next-移除成本
            // 计算(j,i)移动的成本变化
            const auto delta2 = this->instance.get_cost(i.prevprev, j.v) + c_iv_jv + c_iprev_jprev +
                                this->instance.get_cost(i.prev, j.next) + j.vrem + i.prevrem;

            return {delta1, delta2}; // 返回两个成本变化
        }

        // 缓存结构体1：存储顶点作为第一个顶点时的信息
        struct Cache1 {
            int v, prev, next; // 顶点、前驱、后继
            double vrem;       // 顶点移除成本
        };

        // 准备缓存1：为顶点作为第一个顶点时准备缓存
        inline Cache1 prepare_cache1(const Solution& solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache1();
            c.v = vertex;
            const auto route = solution.get_route_index(vertex);
            c.prev = solution.get_prev_vertex(c.v); // 前驱
            c.next = solution.get_next_vertex(c.v); // 后继
            c.vrem = -solution.get_cost_prev_customer(c.v) - solution.get_cost_prev_vertex(route, c.next); // 移除成本
            return c;
        }

        // 准备缓存1（针对depot）：当顶点是depot且作为第一个顶点时
        inline Cache1 prepare_cache1(const Solution& solution, int vertex, int backup) {
            assert(backup != instance.get_depot());
            auto c = Cache1();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route); // 最后一个客户作为前驱
            c.next = solution.get_first_customer(route); // 第一个客户作为后继
            c.vrem = -solution.get_cost_prev_depot(route) - solution.get_cost_prev_customer(c.next); // 移除成本
            return c;
        }

        // 缓存结构体2：存储顶点作为第二个顶点时的信息
        struct Cache2 {
            int v, prev, prevprev; // 顶点、前驱、前驱的前驱
            double prevrem;        // 前驱移除成本
        };

        // 准备缓存2：为顶点作为第二个顶点时准备缓存
        inline Cache2 prepare_cache2(const Solution& solution, int vertex) {
            assert(vertex != this->instance.get_depot());
            auto c = Cache2();
            c.v = vertex;
            const auto route = solution.get_route_index(c.v);
            c.prev = solution.get_prev_vertex(c.v); // 前驱
            c.prevprev = solution.get_prev_vertex(route, c.prev); // 前驱的前驱
            c.prevrem = -solution.get_cost_prev_vertex(route, c.prev) - solution.get_cost_prev_customer(c.v); // 前驱移除成本
            return c;
        }

        // 准备缓存2（针对depot）：当顶点是depot且作为第二个顶点时
        inline Cache2 prepare_cache2(const Solution& solution, int vertex, int backup) {
            assert(backup != instance.get_depot());
            auto c = Cache2();
            c.v = vertex;
            const auto route = solution.get_route_index(backup);
            c.prev = solution.get_last_customer(route); // 最后一个客户作为前驱
            c.prevprev = solution.get_prev_vertex(c.prev); // 前驱的前驱
            c.prevrem = -solution.get_cost_prev_customer(c.prev) - solution.get_cost_prev_depot(route); // 前驱移除成本
            return c;
        }

        // 计算成本：使用缓存的i和j信息计算移动成本
        template <typename C1, typename C2>
        inline double compute_cost(const MoveGenerator& move, const C1 i, const C2 j) {

            const auto delta = this->instance.get_cost(j.prevprev, i.v) + this->moves.get_edge_cost(move) +
                               this->instance.get_cost(i.prev, j.prev) + this->instance.get_cost(j.prev, i.next) + i.vrem + j.prevrem;
                        //其中删除成本（负值）被打包在 i.vrem 和 j.prevrem 中
            return delta; // 返回成本变化
        }
    };

}  // namespace cobra

#endif