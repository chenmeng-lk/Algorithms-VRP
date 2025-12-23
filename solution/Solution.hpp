// CVRP解的表示类
// 使用双向链表存储路径，支持高效的插入、删除和修改操作
#ifndef _FILO2_SOLUTION_HPP_
#define _FILO2_SOLUTION_HPP_

#include <fstream>
#include <iostream>
#include <vector>

#include "../base/FixedSizeValueStack.hpp"
#include "../base/LRUCache.hpp"
#include "../base/macro.hpp"
#include "../instance/Instance.hpp"

namespace cobra {

    // Class representing a CVRP solution.
    // CVRP解的表示类
    //
    // A few highlevel notes:
    // 高层设计说明：
    // - Route is not a first class concept (e.g., there is no Route class, but all operations on routes go through this Solution class).
    //   路径不是一等公民（没有独立的Route类，所有路径操作都通过Solution类）
    // - Routes are stored as doubly linked lists (implemented with a prev and next vectors).
    //   路径使用双向链表存储（通过prev和next向量实现）
    // - There is a single depot, and this makes it quite a special node since it belongs to all routes, but it cannot be used to identify
    //   any specific route. For this reason, there are methods such as `get_route_index` that either take a single customer in input or two
    //   vertices: the vertex for which we are interested in getting the route index, and a fallback that is a vertex (different from the
    //   previous one), that should be used in case the former is the depot.
    //   只有一个仓库节点，它属于所有路径但不能用于标识特定路径。因此有些方法（如get_route_index）需要额外的fallback参数
    //
    // 数据结构设计：
    // - 使用双向链表存储每条路径的顾客序列
    // - 使用LRU缓存（SVC）跟踪最近修改的顶点
    // - 支持do/undo操作列表，用于高效的移动评估和回退
    // - 维护累积载重信息，支持容量约束检查
    class Solution {

    public:
        // Dummy value used to identify an invalid customer.
        // 无效顾客的哨兵值
        static inline const int dummy_vertex = -1;

        // Dummy value used to identify an invalid route.
        // 无效路径的哨兵值
        static inline const int dummy_route = 0;

        // 构造函数 - 使用默认历史长度
        explicit Solution(const Instance &instance_) : Solution(instance_, instance_.get_vertices_num()) { }

        // 构造函数 - 指定历史长度
        // @param instance_ CVRP实例
        // @param history_len SVC（最近修改顶点集合）的容量
        Solution(const Instance &instance_, int history_len)
            : instance(instance_)
            , solution_cost(INFINITY)  // 初始成本为无穷大，需要调用reset()
            , max_number_routes(instance_.get_vertices_num() + 1)  // 最多路径数
            , routes_pool(max_number_routes - 1, [](int index) { return index + 1; })  // 路径池
            , depot_node({Solution::dummy_route, 0})  // 仓库节点
            , routes_list(max_number_routes)  // 路径列表
            , customers_list(instance_.get_vertices_num())  // 顾客列表
            , cache(history_len, instance_.get_vertices_num()) { }  // SVC缓存

        // 拷贝构造函数
        // 注意：这是一个昂贵的操作，应尽量使用do/undo列表代替
        Solution(const Solution &source)
            : instance(source.instance)
            , solution_cost(INFINITY)
            , max_number_routes(instance.get_vertices_num() + 1)
            , routes_pool(max_number_routes - 1, [](int index) { return index + 1; })
            , depot_node({Solution::dummy_route, 0})
            , routes_list(max_number_routes)
            , customers_list(instance.get_vertices_num())
            , cache(source.cache) {
            copy(source);  // 深拷贝
        }

        ~Solution() = default;

        // 赋值运算符
        Solution &operator=(const Solution &source) {
            if (this == &source) {
                return *this;
            }
            copy(source);  // 深拷贝
            return *this;
        }

        // 相等比较运算符
        // 比较成本和所有顾客的前驱后继关系
        bool operator==(const Solution &other) const {
            if (std::fabs(solution_cost - other.solution_cost) >= 0.01) {
                return false;
            }
            for (int i = instance.get_customers_begin(); i < instance.get_customers_end(); ++i) {
                if (get_prev_vertex(i) != other.get_prev_vertex(i) || get_next_vertex(i) != other.get_next_vertex(i)) {
                    return false;
                }
            }
            return true;
        }

        // 不等比较运算符
        bool operator!=(const Solution &other) const {
            return !(*this == other);
        }

        // Reset a solution. Currently a newly constructed solution object needs to be reset before usage. That's ugly and must be fixed.
        // Why is the solution cost set to infinity in the constructor? Weird.
        // 重置解
        // 注意：新构造的Solution对象需要先调用reset()才能使用
        // TODO: 这个设计不太好，应该改进
        void reset() {

            solution_cost = 0.0;  // 重置成本

            routes_pool.reset();  // 重置路径池

            depot_node.first_route = Solution::dummy_route;  // 清空路径链表
            depot_node.num_routes = 0;

            // 重置所有路径
            for (int r = 0; r < max_number_routes; r++) {
                reset_route(r);
            }

            // 重置所有顶点
            for (int i = 0; i < instance.get_vertices_num(); i++) {
                reset_vertex(i);
            }

            cache.clear();  // 清空SVC缓存

            // 清空所有操作列表
            undo_list1.clear();
            do_list1.clear();
            do_list2.clear();
        }

        // Returns the solution cost.
        // 获取解的总成本
        inline double get_cost() const {
            return solution_cost;
        }

        // Returns the number of routes in the solution.
        // 获取解中的路径数量
        inline int get_routes_num() const {
            return depot_node.num_routes;
        }

        // Builds a one-customer route and returns the route index.
        // 构建一条只包含一个顾客的路径
        //
        // 路径结构：depot -> customer -> depot
        //
        // @tparam record_action 是否记录操作到do/undo列表
        // @param customer 要添加的顾客
        // @return 新路径的索引
        template <bool record_action = true>
        int build_one_customer_route(const int customer) {

            assert(!is_customer_in_solution(customer));//客户不在解中
            assert(customer != instance.get_depot());//客户不是仓库

            // 从路径池中请求一条新路径
            const auto route = request_route();

            // 记录操作（用于do/undo）
            if constexpr (record_action) {
                do_list1.emplace_back(Action::create_one_customer_route(route, customer));
                undo_list1.emplace_back(Action::remove_one_customer_route(route, customer));
            }

            // 设置顾客的前驱和后继都是仓库
            customers_list[customer].prev = instance.get_depot();
            customers_list[customer].next = instance.get_depot();
            customers_list[customer].route_ptr = route;
            customers_list[customer].c_prev_curr = instance.get_cost(instance.get_depot(), customer);

            // Head insert the route in the list.
            // 将新路径插入到路径链表的头部
            const auto next_route = depot_node.first_route;
            routes_list[route].next = next_route;
            depot_node.first_route = route;
            routes_list[route].prev = Solution::dummy_route;
            routes_list[next_route].prev = route;

            // 设置路径信息
            routes_list[route].first_customer = customer;
            routes_list[route].last_customer = customer;
            routes_list[route].load = instance.get_demand(customer);
            routes_list[route].size = 1;
            routes_list[route].c_prev_curr = customers_list[customer].c_prev_curr;

            // 更新总成本：depot->customer + customer->depot
            solution_cost += 2.0 * customers_list[customer].c_prev_curr;

            // 将顾客添加到SVC缓存
            cache.insert(customer);

            // 标记需要更新累积载重
            routes_list[route].needs_cumulative_load_update = true;

            return route;
        }

        // Returns the index of the route serving the given customer. Returns Solution::dummy_route if the customer is not served.
        // 获取服务指定顾客的路径索引
        // @param customer 顾客索引（不能是仓库）
        // @return 路径索引，如果顾客未被服务则返回dummy_route
        inline int get_route_index(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].route_ptr;
        }

        // Returns the index of the route serving the given vertex. If the vertex is the depot, the fallback is used to identify the route
        // index.
        // 获取服务指定顶点的路径索引
        // 如果顶点是仓库，使用fallback来确定路径索引
        // @param vertex 顶点索引（可以是仓库）
        // @param fallback 备用顶点（当vertex是仓库时使用）
        // @return 路径索引
        inline int get_route_index(const int vertex, const int fallback) const {
            if (unlikely(vertex == instance.get_depot())) {//很小概率vertex是车场，减少预测失败数
                return customers_list[fallback].route_ptr;
            } else {
                return customers_list[vertex].route_ptr;
            }
        }

        // Returns the load of the given route.
        // 获取指定路径的载重
        inline int get_route_load(const int route) const {
            return routes_list[route].load;
        }

        // Returns the index of the first route in the solution.
        // 获取解中第一条路径的索引
        inline int get_first_route() const {
            return depot_node.first_route;
        }

        // Returns the index of the next route in the list of routes.
        // 获取路径链表中下一条路径的索引
        inline int get_next_route(const int route) const {
            return routes_list[route].next;
        }

        // Returns the index after the last route. Useful to check for termination when looping through routes.
        // 获取路径链表的结束标记
        // 用于遍历路径时检查是否结束
        inline int get_end_route() const {
            return dummy_route;
        }

        // Returns whether the route does not serve any customer. Note that empty routes must always be removed from the solution.
        // 检查路径是否为空（不服务任何顾客）
        // 注意：空路径必须从解中移除
        inline bool is_route_empty(const int route) const {
            return routes_list[route].load == 0;
        }

        // Removes the vertex from the given route.
        // 从指定路径中移除顶点
        //
        // 操作：
        // - 如果移除的是仓库：将路径变为不一致状态（缺少仓库）
        // - 如果移除的是顾客：更新链表指针和路径信息
        //
        // @tparam record_action 是否记录操作到do/undo列表
        // @param route 路径索引
        // @param vertex 要移除的顶点
        // @return 成本变化量（delta）
        template <bool record_action = true>
        double remove_vertex(const int route, const int vertex) {

            assert(contains_vertex(route, vertex));

            // 记录操作
            if constexpr (record_action) {
                do_list1.emplace_back(Action::remove_vertex(route, vertex));
                undo_list1.emplace_back(Action::insert_vertex(route, get_next_vertex(route, vertex), vertex));
            }

            // 情况1：移除仓库（将路径变为不一致状态）
            if (unlikely(vertex == instance.get_depot())) {

                assert(!is_route_empty(route));

                const auto next = routes_list[route].first_customer;
                const auto prev = routes_list[route].last_customer;

                // 将相关顶点添加到SVC
                cache.insert(vertex);
                cache.insert(prev);
                cache.insert(next);

                // 连接首尾顾客
                set_prev_vertex_ptr(route, next, prev);
                set_next_vertex_ptr(route, prev, next);

                // 标记路径缺少仓库
                routes_list[route].first_customer = Solution::dummy_vertex;
                routes_list[route].last_customer = Solution::dummy_vertex;

                // 更新成本
                assert(next != instance.get_depot());
                customers_list[next].c_prev_curr = instance.get_cost(prev, next);

                const auto delta = +customers_list[next].c_prev_curr - instance.get_cost(prev, vertex) - instance.get_cost(vertex, next);

                solution_cost += delta;

                routes_list[route].needs_cumulative_load_update = true;

                return delta;

            } else {
                // 情况2：移除顾客

                assert(contains_vertex(route, vertex));
                assert(instance.get_depot() != vertex);

                const auto next = customers_list[vertex].next;
                const auto prev = customers_list[vertex].prev;

                // 将相关顶点添加到SVC
                cache.insert(vertex);
                cache.insert(prev);
                cache.insert(next);

                // 更新链表指针
                if (vertex == routes_list[route].first_customer) {
                    // 移除的是第一个顾客
                    routes_list[route].first_customer = next;
                    set_prev_vertex_ptr(route, next, instance.get_depot());  // Next might be the root of the route.
                } else if (vertex == routes_list[route].last_customer) {
                    // 移除的是最后一个顾客
                    routes_list[route].last_customer = prev;
                    set_next_vertex_ptr(route, prev, instance.get_depot());  // Prev might be the root of the route.
                } else {
                    // 移除的是中间顾客
                    customers_list[prev].next = next;  // If vertex != route.first_customer, then prev is not the root.
                    customers_list[next].prev = prev;  // If vertex != route.last_customer, then next is not the root.
                }

                // 更新路径载重和大小
                routes_list[route].load -= instance.get_demand(vertex);
                routes_list[route].size -= 1;

                // 更新成本
                const auto c_prev_next = instance.get_cost(prev, next);

                if (next == instance.get_depot()) {
                    routes_list[route].c_prev_curr = c_prev_next;
                } else {
                    customers_list[next].c_prev_curr = c_prev_next;
                }

                const auto delta = +c_prev_next - instance.get_cost(prev, vertex) - instance.get_cost(vertex, next);

                solution_cost += delta;

                // TODO: is this really necessary?
                // 重置顶点状态
                reset_vertex(vertex);

                routes_list[route].needs_cumulative_load_update = true;

                return delta;
            }
        }

        // Removes the route from the solution. The route must be empty.
        // 从解中移除一条路径。路径必须为空。
        template <bool record_action = true>
        void remove_route(const int route) {
            assert(is_route_empty(route));

            if constexpr (record_action) {
                do_list1.emplace_back(Action::remove_route(route));
                undo_list1.emplace_back(Action::create_route(route));
            }

            release_route(route);
        }

        // Returns the first customer of the route.
        // 获取路径的第一个顾客
        inline int get_first_customer(const int route) const {
            return routes_list[route].first_customer;
        }

        // Returns the last customer of the route.
        // 获取路径的最后一个顾客
        inline int get_last_customer(const int route) const {
            return routes_list[route].last_customer;
        }

        // Returns the customer after the given one in its route.
        // 获取路径中指定顾客的后继顾客
        inline int get_next_vertex(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].next;
        }

        // Returns the vertex after the given one in its route. Vertex must belong to route. This is the right method if vertex might be the
        // depot.
        // 获取路径中指定顶点的后继顶点
        // 如果顶点是仓库，返回路径的第一个顾客
        inline int get_next_vertex(const int route, const int vertex) const {
            assert(contains_vertex(route, vertex));
            if (unlikely(vertex == instance.get_depot())) {
                return routes_list[route].first_customer;
            } else {
                return customers_list[vertex].next;
            }
        }

        // Returns the customer before the given one in its route.
        // 获取路径中指定顾客的前驱顾客
        inline int get_prev_vertex(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].prev;
        }

        // Returns the vertex before the given one in its route. Vertex must belong to route. This is the right method if vertex might be
        // the depot.
        // 获取路径中指定顶点的前驱顶点
        // 如果顶点是仓库，返回路径的最后一个顾客
        inline int get_prev_vertex(const int route, const int vertex) const {
            assert(contains_vertex(route, vertex));
            if (unlikely(vertex == instance.get_depot())) {
                return get_last_customer(route);
            } else {
                return get_prev_vertex(vertex);
            }
        }

        // Inserts `vertex` before `where` in `route`. `where` must belong to `route` and `vertex` must be unserved.
        // 在路径中的指定位置之前插入顶点
        //
        // 操作：
        // - 如果插入的是仓库：将不一致的路径恢复为正常状态
        // - 如果插入的是顾客：更新链表指针和路径信息
        //
        // @tparam record_action 是否记录操作到do/undo列表
        // @param route 路径索引
        // @param where 插入位置（在此顶点之前插入）
        // @param vertex 要插入的顶点
        template <bool record_action = true>
        void insert_vertex_before(const int route, const int where, const int vertex) {

            // 记录操作
            if constexpr (record_action) {
                do_list1.emplace_back(Action::insert_vertex(route, where, vertex));
                undo_list1.emplace_back(Action::remove_vertex(route, vertex));
            }

            assert(where != vertex);

            // 情况1：插入仓库（恢复不一致的路径）
            if (unlikely(vertex == instance.get_depot())) {

                assert(routes_list[route].first_customer == Solution::dummy_vertex);
                assert(routes_list[route].last_customer == Solution::dummy_vertex);
                assert(where != instance.get_depot());

                assert(!is_route_empty(route));

                const auto prev = customers_list[where].prev;

                // 将相关顶点添加到SVC
                cache.insert(prev);
                cache.insert(where);

                assert(prev != instance.get_depot());

                // 恢复路径的首尾指针
                routes_list[route].first_customer = where;
                routes_list[route].last_customer = prev;

                // 断开环形链表，插入仓库
                customers_list[prev].next = instance.get_depot();
                customers_list[where].prev = instance.get_depot();

                // 更新成本
                routes_list[route].c_prev_curr = instance.get_cost(prev, instance.get_depot());

                double old_cost_prev_where = customers_list[where].c_prev_curr;
                customers_list[where].c_prev_curr = instance.get_cost(instance.get_depot(), where);

                const auto delta = +routes_list[route].c_prev_curr + customers_list[where].c_prev_curr - old_cost_prev_where;

                solution_cost += delta;

            } else {
                // 情况2：插入顾客

                assert(!is_customer_in_solution(vertex));
                assert(vertex != instance.get_depot());

                const auto prev = get_prev_vertex(route, where);

                // 将相关顶点添加到SVC
                cache.insert(prev);
                cache.insert(where);

                // 设置新顾客的链表指针
                customers_list[vertex].next = where;
                customers_list[vertex].prev = prev;
                customers_list[vertex].route_ptr = route;

                // 更新前驱和后继的指针
                set_next_vertex_ptr(route, prev, vertex);
                set_prev_vertex_ptr(route, where, vertex);

                // 更新成本
                double old_cost_prev_where;
                const auto c_vertex_where = instance.get_cost(vertex, where);
                if (where == instance.get_depot()) {
                    old_cost_prev_where = routes_list[route].c_prev_curr;
                    routes_list[route].c_prev_curr = c_vertex_where;
                } else {
                    old_cost_prev_where = customers_list[where].c_prev_curr;
                    customers_list[where].c_prev_curr = c_vertex_where;
                }
                customers_list[vertex].c_prev_curr = instance.get_cost(prev, vertex);

                // 计算成本变化：新增两条边，删除一条边
                const auto delta = +customers_list[vertex].c_prev_curr + c_vertex_where - old_cost_prev_where;

                solution_cost += delta;
                routes_list[route].load += instance.get_demand(vertex);
                routes_list[route].size += 1;
            }

            // 标记需要更新累积载重
            routes_list[route].needs_cumulative_load_update = true;
        }

        // Reverses the sub-path identified by vertex_begin and vertex_end.
        // 反转路径中的一段子路径
        //
        // 操作：将vertex_begin到vertex_end之间的顶点序列反转，并更新两端连接
        // @tparam record_action 是否记录操作到do/undo列表
        // @param route 路径索引
        // @param vertex_begin 子路径起始顶点
        // @param vertex_end 子路径结束顶点
        template <bool record_action = true>
        void reverse_route_path(const int route, const int vertex_begin, const int vertex_end) {

            // 记录操作
            if constexpr (record_action) {
                do_list1.emplace_back(Action::reverse_route_path(route, vertex_begin, vertex_end));
                undo_list1.emplace_back(Action::reverse_route_path(route, vertex_end, vertex_begin));
            }

            assert(vertex_begin != vertex_end);

            // 获取子路径的前驱和后继
            const auto pre = get_prev_vertex(route, vertex_begin);
            const auto stop = get_next_vertex(route, vertex_end);
            //前驱边成本
            const auto c_pre_begin = get_cost_prev_vertex(route, vertex_begin);
            //新增边成本
            const double c_pre_vertex_end = instance.get_cost(pre, vertex_end);
            const double c_vertex_begin_stop = instance.get_cost(stop, vertex_begin);

            // 将相关顶点添加到SVC
            cache.insert(pre);
            cache.insert(stop);

            // 反转子路径中的所有顶点
            auto curr = vertex_begin;
            do {

                cache.insert(curr);

                const auto prev = get_prev_vertex(route, curr);
                const auto next = get_next_vertex(route, curr);

                // 交换前驱和后继指针
                if (curr == instance.get_depot()) {
                    routes_list[route].last_customer = next;
                    routes_list[route].first_customer = prev;
                    assert(next != instance.get_depot());
                    routes_list[route].c_prev_curr = customers_list[next].c_prev_curr;
                } else {
                    customers_list[curr].prev = next;
                    customers_list[curr].next = prev;
                    customers_list[curr].c_prev_curr = get_cost_prev_vertex(route, next);
                }

                curr = next;

            } while (curr != stop);

            // 处理边界情况和更新连接
            if (vertex_end == pre && vertex_begin == stop) {
                // `vertex_begin` and `vertex_end` are contiguous.
                // vertex_begin和vertex_end相邻的特殊情况，整个环被反转
                if (vertex_end == instance.get_depot()) {
                    routes_list[route].c_prev_curr = c_pre_begin;
                } else {
                    customers_list[vertex_end].c_prev_curr = c_pre_begin;
                }
            } else {
                // 一般情况：更新子路径两端的连接

                set_next_vertex_ptr(route, vertex_begin, stop);
                set_next_vertex_ptr(route, pre, vertex_end);

                if (vertex_end == instance.get_depot()) {
                    routes_list[route].last_customer = pre;
                    routes_list[route].c_prev_curr = c_pre_vertex_end;
                } else {
                    customers_list[vertex_end].prev = pre;
                    customers_list[vertex_end].c_prev_curr = c_pre_vertex_end;
                }

                if (stop == instance.get_depot()) {
                    routes_list[route].last_customer = vertex_begin;
                    routes_list[route].c_prev_curr = c_vertex_begin_stop;
                } else {
                    customers_list[stop].prev = vertex_begin;
                    customers_list[stop].c_prev_curr = c_vertex_begin_stop;
                }
            }

            // 计算成本变化
            const auto delta = -instance.get_cost(pre, vertex_begin) - instance.get_cost(vertex_end, stop) + c_pre_vertex_end +
                               c_vertex_begin_stop;

            solution_cost += delta;

            routes_list[route].needs_cumulative_load_update = true;
        }

        // Appends the customer of `route_to_append` to `route` and removes `route_to_append` from the solution.
        // 将route_to_append的所有顾客追加到route的末尾
        // 然后从解中移除route_to_append
        //
        // @param route 目标路径
        // @param route_to_append 要追加的路径
        // @return 成本变化量
        int append_route(const int route, const int route_to_append) {

            const int route_end = routes_list[route].last_customer;
            const int route_to_append_start = routes_list[route_to_append].first_customer;

            assert(route_end != instance.get_depot());
            assert(route_to_append_start != instance.get_depot());

            // 连接两条路径
            customers_list[route_end].next = route_to_append_start;
            customers_list[route_to_append_start].prev = route_end;
            customers_list[route_to_append_start].c_prev_curr = instance.get_cost(route_end, route_to_append_start);

            // 更新路径信息
            routes_list[route].last_customer = routes_list[route_to_append].last_customer;
            routes_list[route].load += routes_list[route_to_append].load;
            routes_list[route].size += routes_list[route_to_append].size;
            routes_list[route].c_prev_curr = routes_list[route_to_append].c_prev_curr;

            // 计算成本变化：新增连接边，删除两条到仓库的边
            const double delta = +customers_list[route_to_append_start].c_prev_curr - instance.get_cost(route_end, instance.get_depot()) -
                                 instance.get_cost(instance.get_depot(), route_to_append_start);

            solution_cost += delta;

            cache.insert(route_end);

            // 更新所有追加顾客的路径指针并加入到SVC
            for (int curr = route_to_append_start; curr != instance.get_depot(); curr = customers_list[curr].next) {
                customers_list[curr].route_ptr = route;

                cache.insert(curr);
            }

            // 释放被追加的路径
            release_route(route_to_append);

            routes_list[route].needs_cumulative_load_update = true;

            return route;
        }

        // Generate a string representation of the given route.
        // 生成路径的字符串表示
        // 格式：[route_id] depot customer1 customer2 ... depot
        std::string to_string(const int route) const {
            std::string str;
            str += "[" + std::to_string(route) + "] ";
            str += std::to_string(instance.get_depot()) + " ";
            for (int curr = routes_list[route].first_customer; curr != instance.get_depot(); curr = customers_list[curr].next) {
                str += std::to_string(curr) + " ";
            }
            str += std::to_string(instance.get_depot());
            return str;
        }

        // Prints a string representation of the given route.
        // 打印路径的字符串表示
        void print(const int route) const {
            if (is_missing_depot(route)) {
                std::cout << "Route " << route << " is in an INCONSISTENT state: missing the depot. It cannot be accessed without it.\n";
            } else {
                std::cout << to_string(route) << " (" << get_route_load(route) << ") " << get_route_cost(route) << "\n";
            }
        }

        // Prints the solution.
        // 打印整个解
        void print() const {
            for (auto route = depot_node.first_route; route != Solution::dummy_route; route = routes_list[route].next) {
                print(route);
            }
            std::cout << "Solution cost = " << solution_cost << "\n";
        }

        // Returns the route's cumulative load before the given customer included.
        // 获取从仓库到指定顾客（包含）的累积载重
        // 如果需要，会先更新累积载重信息
        inline int get_route_load_before_included(const int customer) {
            assert(customer != instance.get_depot());
            const int route = customers_list[customer].route_ptr;
            if (routes_list[route].needs_cumulative_load_update) {
                update_cumulative_route_loads(route);
                routes_list[route].needs_cumulative_load_update = false;
            }
            return customers_list[customer].load_before;
        }

        // Returns the route's cumulative load after the given customer included.
        // 获取从指定顾客（包含）到仓库的累积载重
        // 如果需要，会先更新累积载重信息
        inline int get_route_load_after_included(const int customer) {
            assert(customer != instance.get_depot());
            const int route = customers_list[customer].route_ptr;
            if (routes_list[route].needs_cumulative_load_update) {
                update_cumulative_route_loads(route);
                routes_list[route].needs_cumulative_load_update = false;
            }
            return customers_list[customer].load_after;
        }

        // Returns whether the given route index is currently served in the solution.
        // 检查指定路径是否在解中
        inline bool is_route_in_solution(const int route) const {
            return routes_list[route].in_solution;
        }

        // Returns whether the given customer is currently served in the solution.
        // 检查指定顾客是否在解中被服务
        inline bool is_customer_in_solution(const int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].route_ptr != Solution::dummy_route;
        }

        // Returns whether the given vertex is currently served in the solution. Use this method, if vertex could be the depot.
        // 检查指定顶点是否在解中
        // 如果顶点可能是仓库，使用此方法
        inline bool is_vertex_in_solution(const int vertex) const {
            return vertex == instance.get_depot() || is_customer_in_solution(vertex);
        }

        // Returns whether the given vertex is served by route. Alwats returns true when vertex is the depot.
        // 检查指定顶点是否被指定路径服务
        // 如果顶点是仓库，总是返回true
        inline bool contains_vertex(const int route, const int vertex) const {
            assert(vertex >= instance.get_vertices_begin() && vertex < instance.get_vertices_end() && route >= 0 &&
                   route < max_number_routes);
            return customers_list[vertex].route_ptr == route || vertex == instance.get_depot();
        }

        // Returns the number of customers served by the given route.
        // 获取路径服务的顾客数量
        inline int get_route_size(const int route) const {
            return routes_list[route].size;
        }

        // Swaps the customers [iNext, ..., lastCustomer(iRoute)] from `iRoute`  and [j, lastCustomer(jRoute)] from `jRoute`.
        //
        // [iRoute] = depot, o, o, o, i, iNext, o, o, o, depot
        //                              \ /
        //                               X___
        //                              /    |
        // [jRoute] = depot, o, o, o, jPrev, j, o, o, o, depot
        //
        // Definitely not the best picture, but this replaces (i, iNext) with (i, j) and (jPrev, j) with (jPrev, jNext).
        void swap_tails(const int i, const int iRoute, const int j, const int jRoute) {

            assert(i != instance.get_depot());
            assert(j != instance.get_depot());
            assert(iRoute != jRoute);
            assert(contains_vertex(iRoute, i));
            assert(contains_vertex(jRoute, j));

            const auto iNext = customers_list[i].next;
            //j及之后的顾客加入到i之后
            auto curr = j;
            while (curr != instance.get_depot()) {
                const auto next = customers_list[curr].next;
                remove_vertex(jRoute, curr);
                insert_vertex_before(iRoute, iNext, curr);
                curr = next;
            }
            //iNext及之后的顾客依次加入到jRoute末尾
            curr = iNext;
            while (curr != instance.get_depot()) {
                const auto next = customers_list[curr].next;
                remove_vertex(iRoute, curr);
                insert_vertex_before(jRoute, instance.get_depot(), curr);
                curr = next;
            }

            routes_list[iRoute].needs_cumulative_load_update = true;
            routes_list[jRoute].needs_cumulative_load_update = true;
        }

        // Replaces (i, iNext) with (i, j) and reverts (depot, j). Replaces (j, jNext) with (iNext, jNext) and reverts (iNext, depot).
        //
        // [iRoute] = depot, o, o, o, i  iNext<-o<-o<-o<-depot
        //                            |     |
        //                            |     |
        // [jRoute] = depot<-o<-o<-o<-j  jNext, o, o, o, depot
        void split(const int i, const int iRoute, const int j, const int jRoute) {

            assert(i != instance.get_depot());
            assert(j != instance.get_depot());

            const auto iNext = customers_list[i].next;
            const auto jNext = customers_list[j].next;
            //j及之前的所有顾客加入到i之后
            auto curr = j;
            while (curr != instance.get_depot()) {
                const auto prev = customers_list[curr].prev;
                remove_vertex(jRoute, curr);
                insert_vertex_before(iRoute, iNext, curr);
                curr = prev;
            }
            //iNext及之后的所有顾客加入到jNext之前
            auto before = jNext;
            curr = iNext;
            while (curr != instance.get_depot()) {
                const auto next = customers_list[curr].next;
                remove_vertex(iRoute, curr);
                insert_vertex_before(jRoute, before, curr);
                before = curr;
                curr = next;
            }

            routes_list[iRoute].needs_cumulative_load_update = true;
            routes_list[jRoute].needs_cumulative_load_update = true;
        }

        // Returns the cost of the arc (prev, vertex) where prev is the predecessor of vertex in route. Vertex must be served by route. Use
        // this method if vertex could be the depot.
        //路线中结点与前驱结点的边成本
        inline double get_cost_prev_vertex(int route, int vertex) const {
            if (vertex == instance.get_depot()) {
                return routes_list[route].c_prev_curr;
            } else {
                return customers_list[vertex].c_prev_curr;
            }
        }

        // Returns the cost of the arc (prev, customer) where prev is the predecessor of customer in route. Customer must be served by
        // route.
        //顾客与前驱结点的边成本
        inline double get_cost_prev_customer(int customer) const {
            assert(customer != instance.get_depot());
            return customers_list[customer].c_prev_curr;
        }

        // Returns the cost of the arc (prev, depot) where prev is the predecessor of depot in route.
        //仓库与前驱顾客的边成本
        inline double get_cost_prev_depot(int route) const {
            return routes_list[route].c_prev_curr;
        }

        // Returns the route cost. This is currently a linear procedure in the instance size. Use with caution!
        //获取路径成本
        double get_route_cost(const int route) const {
            int curr = routes_list[route].first_customer;
            double sum = instance.get_cost(instance.get_depot(), curr);
            while (curr != instance.get_depot()) {
                const int next = customers_list[curr].next;
                sum += instance.get_cost(curr, next);
                curr = next;
            }

            return sum;
        }

        // Clears the set of recently modified vertices.
        //清空svc缓存
        inline void clear_svc() {
            cache.clear();
        }

        // Returns the cache containing recently modified vertices. Note that it is not safe to perform any operations on the solution while
        // iterating the returned set, so store the elements somewhere else if necessary.
        // 获取SVC（最近修改的顶点集合）
        // 注意：遍历SVC时不要修改解，必要时先将元素存储到其他地方
        inline const LRUCache &get_svc() const {
            return cache;
        }

        // Returns the first element in the set of recently modified vertices.
        // 获取SVC中的第一个元素
        inline int get_svc_begin() const {
            return cache.begin();
        }

        // Returns the element after the given one in the set of recently modified vertices.
        // 获取SVC中指定元素的下一个元素
        inline int get_svc_next(const int i) const {
            return cache.get_next(i);
        }

        // Returns an invalid vertex, useful to terminate iteration over the cache.
        // 获取SVC的结束标记（用于终止遍历）
        inline int get_svc_end() const {
            return cache.end();
        }

        // Returns the number of elements in the set of recently modified vertices.
        // 获取SVC中的元素数量
        inline int get_svc_size() const {
            return cache.size();
        }

        // Returns whether the route does not violate load constraints.
        // 检查路径是否满足载重约束
        inline bool is_load_feasible(const int route) const {
            return routes_list[route].load <= instance.get_vehicle_capacity();
        }

        // Returns whether the solution does not violate load constraints.
        // 检查整个解是否满足载重约束
        inline bool is_load_feasible() const {
            for (auto r = get_first_route(); r != dummy_route; r = get_next_route(r)) {//遍历每条路线
                if (!is_load_feasible(r)) {
                    return false;
                }
            }
            return true;
        }

        // Stores the solution to file.
        // 将解保存到文件
        // 格式：每条路径一行，最后一行是总成本
        static void store_to_file(const cobra::Instance &instance, const cobra::Solution &solution, const std::string &path) {
            auto out_stream = std::ofstream(path);
            for (auto route = solution.get_first_route(), idx = 1; route != cobra::Solution::dummy_route;
                 route = solution.get_next_route(route), idx++) {
                out_stream << "Route #" << idx << ":";
                for (auto customer = solution.get_first_customer(route); customer != instance.get_depot();
                     customer = solution.get_next_vertex(customer)) {
                    out_stream << " " << std::to_string(customer);
                }
                out_stream << "\n";
            }
            out_stream << "Cost " << std::to_string(solution.get_cost());
        }

        // Applies the do-list 1 to solution.
        // 将do_list1中的所有操作应用到解上
        // 用于评估移动后的解状态
        void apply_do_list1(Solution &solution) const {
            assert(solution.is_feasible());
            for (int i = 0; i < static_cast<int>(do_list1.size()); ++i) {
                apply_action(solution, do_list1[i]);
            }
            assert(solution.is_feasible());
        }

        // Applied the do-list 2 to solution.
        // 将do_list2中的所有操作应用到解上
        void apply_do_list2(Solution &solution) const {
            assert(solution.is_feasible());
            for (int i = 0; i < static_cast<int>(do_list2.size()); ++i) {
                apply_action(solution, do_list2[i]);
            }
            assert(solution.is_feasible());
        }

        // Appends the do-list 1 to the do-list 2.
        // 将do_list1追加到do_list2
        // 用于组合多个移动操作
        void append_do_list1_to_do_list2() {
            for (const auto &entry : do_list1) {
                do_list2.emplace_back(entry);
            }
        }

        // Applies the undo-list 1 to solution.
        // 将undo_list1中的所有操作应用到解上
        // 用于回退到之前的状态
        // 注意：undo操作需要反向执行
        void apply_undo_list1(Solution &solution) const {
            assert(solution.is_feasible());
            for (int i = static_cast<int>(undo_list1.size()) - 1; i >= 0; --i) {
                apply_action(solution, undo_list1[i]);
            }
            assert(solution.is_feasible());
        }

        // Clears the do-list 1.
        // 清空do_list1
        inline void clear_do_list1() {
            do_list1.clear();
        }

        // Clears the do-list 2.
        // 清空do_list2
        inline void clear_do_list2() {
            do_list2.clear();
        }

        // Clears the undo-list 1.
        // 清空undo_list1
        inline void clear_undo_list1() {
            undo_list1.clear();
        }


        // Returns whether the solution is CVRP feasible. This is a very expensive procedure. Must only be used for debugging purposes.
        // 检查解的可行性，这是一个非常昂贵的过程，仅用于调试目的
        bool is_feasible(const bool error_on_load_infeasible = true, const bool verbose = false) const;

    private:
        // Performs a deep copy a the given source solution. It should not really be used too often if the instance is big.
        // 实例较大时，应尽量避免使用深拷贝
        void copy(const Solution &source) {

#ifndef NDEBUG
            // 警告：深拷贝解可能非常昂贵，应尽量使用do/undo列表代替
            std::cout << "WARNING: performing a full solution copy could be very expensive. Consider using do/undo lists.\n";
#endif

            // 复制路径池、仓库节点、顾客列表、路径列表、成本、SVC
            routes_pool = source.routes_pool;
            depot_node = source.depot_node;
            customers_list = source.customers_list;
            routes_list = source.routes_list;
            solution_cost = source.solution_cost;
            cache = source.cache;
        }
        // 重置路径
        void reset_route(const int route) {
            routes_list[route].load = 0;
            routes_list[route].size = 0;
            routes_list[route].first_customer = Solution::dummy_vertex;
            routes_list[route].last_customer = Solution::dummy_vertex;
            routes_list[route].prev = Solution::dummy_route;
            routes_list[route].next = Solution::dummy_route;
            routes_list[route].needs_cumulative_load_update = true;
            routes_list[route].in_solution = false;
        }
        // 重置顾客
        void reset_vertex(const int customer) {
            customers_list[customer].next = Solution::dummy_vertex;
            customers_list[customer].prev = Solution::dummy_vertex;
            customers_list[customer].route_ptr = Solution::dummy_route;
        }
        //设置当前顶点的后继顶点
        inline void set_next_vertex_ptr(const int route, const int vertex, const int next) {
            //输入参数是路线索引，顶点索引，下一个顶点索引
            if (unlikely(vertex == instance.get_depot())) {
                routes_list[route].first_customer = next;
            } else {
                customers_list[vertex].next = next;
            }
        }
        //设置当前顶点的前驱顶点
        inline void set_prev_vertex_ptr(const int route, const int vertex, const int prev) {
            if (unlikely(vertex == instance.get_depot())) {
                routes_list[route].last_customer = prev;
            } else {
                customers_list[vertex].prev = prev;
            }
        }
        // 从路径池中请求一条新路径
        inline int request_route() {
            assert(!routes_pool.is_empty());

            const auto route = routes_pool.get();
            routes_list[route].in_solution = true;

            depot_node.num_routes++;

            return route;
        }
        // 释放一条路径，将其归还到路径池中
        inline void release_route(const int route) {

            const auto prevRoute = routes_list[route].prev;
            const auto nextRoute = routes_list[route].next;

            routes_list[prevRoute].next = nextRoute;
            routes_list[nextRoute].prev = prevRoute;
            depot_node.num_routes--;

            // Head remove. 首条路径特殊处理
            if (depot_node.first_route == route) {
                depot_node.first_route = nextRoute;
            }

            reset_route(route);

            routes_pool.push(route);
        }
        // 检查路径是否缺少仓库
        inline bool is_missing_depot(const int route) const {
            return get_first_customer(route) == Solution::dummy_vertex;
        }
        // 更新路径的累积载重
        void update_cumulative_route_loads(const int route) {

            assert(!is_route_empty(route));
            //特殊处理第一个顾客的累计载重
            auto prev = routes_list[route].first_customer;

            customers_list[prev].load_before = instance.get_demand(prev);
            customers_list[prev].load_after = routes_list[route].load;
            //继续计算后续顾客
            auto curr = customers_list[prev].next;

            while (curr != instance.get_depot()) {

                customers_list[curr].load_before = customers_list[prev].load_before + instance.get_demand(curr);
                customers_list[curr].load_after = customers_list[prev].load_after - instance.get_demand(prev);

                prev = curr;
                curr = customers_list[curr].next;
            }
        }

        // Structure representing the depot.
        // 仓库节点结构
        // 仓库是所有路径的起点和终点
        struct DepotNode {
            // Index of the first route in the route linked list.
            int first_route;  // 路径链表的第一条路径索引
            // Current number of routes in the route linked list.
            int num_routes;   // 当前路径数量
        };

        // Structure representing a customer.
        // 顾客节点结构
        // 使用双向链表存储路径中的顾客序列
        struct CustomerNode {
            // Index of the next customer in the route.
            int next;  // 路径中下一个顾客的索引
            // Index of the previous customer in the route.
            int prev;  // 路径中前一个顾客的索引
            // Index of the route serving this customer.
            int route_ptr;  // 服务该顾客的路径索引
            // Cumulative load sum from this customer up to the depot in this route.
            int load_after;  // 从该顾客到仓库的累积载重
            // Cumulative load sum from the depot up to this customer (included) in this route.
            int load_before;  // 从仓库到该顾客（包含）的累积载重
            // Cost of the arc (prev, this customer) where prev is the predecessor of this customer.
            double c_prev_curr;  // 从前驱到该顾客的边成本
        };

        // Structure representing a route.
        // 路径节点结构
        // 路径也使用双向链表组织
        struct RouteNode {
            // Index of the first customer in the route.
            int first_customer;  // 路径中第一个顾客的索引
            // Index of the last customer in the route.
            int last_customer;   // 路径中最后一个顾客的索引
            // Overall load of the route.
            int load;  // 路径的总载重
            // Index of the next route in solution.
            int next;  // 解中下一条路径的索引
            // Index of the previous route in solution.
            int prev;  // 解中前一条路径的索引
            // Number of customers in the route.
            int size;  // 路径中的顾客数量
            // Whether customers of this route require an update to their `load_after` and `load_before`.
            bool needs_cumulative_load_update;  // 是否需要更新累积载重
            // Whether this route is in solution.
            bool in_solution;  // 路径是否在解中
            // Cost of the arc (depot, last customer in this route).
            double c_prev_curr;  // 从最后一个顾客到仓库的边成本
        };

        const Instance &instance;  // CVRP实例引用
        double solution_cost;      // 解的总成本

        // TODO: Is this really necessary?
        const int max_number_routes;  // 最大路径数

        FixedSizeValueStack<int> routes_pool;  // 路径池（管理可用路径索引）
        struct DepotNode depot_node;           // 仓库节点
        std::vector<RouteNode> routes_list;    // 所有路径的列表
        std::vector<CustomerNode> customers_list;  // 所有顾客的列表
        cobra::LRUCache cache;  // SVC（最近修改的顶点集合）

        // 操作类型枚举
        // 用于do/undo列表，记录对解的修改操作
        enum class ActionType {
            INSERT_VERTEX,              // 插入顶点
            REMOVE_VERTEX,              // 移除顶点
            CREATE_ROUTE,               // 创建路径
            REMOVE_ROUTE,               // 移除路径
            REVERSE_ROUTE_PATH,         // 反转路径片段
            CREATE_ONE_CUSTOMER_ROUTE,  // 创建单顾客路径
            REMOVE_ONE_CUSTOMER_ROUTE   // 移除单顾客路径
        };

        // 操作类
        // 封装对解的修改操作，支持do/undo
        //
        // 设计思想：
        // - 每个修改操作都记录到do_list和undo_list
        // - 可以高效地评估移动而不实际修改解
        // - 可以快速回退到之前的状态
        class Action {
        public:
            // 创建插入顶点操作
            static Action insert_vertex(int route, int where, int vertex) {
                return Action(ActionType::INSERT_VERTEX, route, vertex, where);
            }
            // 创建移除顶点操作
            static Action remove_vertex(int route, int vertex) {
                return Action(ActionType::REMOVE_VERTEX, route, vertex, Solution::dummy_vertex);
            }
            // 创建创建路径操作
            static Action create_route(int route) {
                return Action(ActionType::CREATE_ROUTE, route, Solution::dummy_vertex, Solution::dummy_vertex);
            }
            // 创建移除路径操作
            static Action remove_route(int route) {
                return Action(ActionType::REMOVE_ROUTE, route, Solution::dummy_vertex, Solution::dummy_vertex);
            }
            // 创建反转路径片段操作
            static Action reverse_route_path(int route, int begin, int end) {
                return Action(ActionType::REVERSE_ROUTE_PATH, route, begin, end);
            }
            // 创建单顾客路径操作
            static Action create_one_customer_route(int route, int customer) {
                return Action(ActionType::CREATE_ONE_CUSTOMER_ROUTE, route, customer, Solution::dummy_vertex);
            }
            // 创建移除单顾客路径操作
            static Action remove_one_customer_route(int route, int customer) {
                return Action(ActionType::REMOVE_ONE_CUSTOMER_ROUTE, route, customer, Solution::dummy_vertex);
            }
            ActionType type;  // 操作类型
            int route;        // 路径索引
            int i, j;         // 操作参数（顶点索引、位置等）

        private:
            Action(ActionType type, int route, int i, int j) : type(type), route(route), i(i), j(j) { }
        };

        // This is not very nice.
        // 操作列表
        std::vector<Action> undo_list1;  // 撤销操作列表
        std::vector<Action> do_list1;    // 执行操作列表1
        std::vector<Action> do_list2;    // 执行操作列表2
        //执行操作列表中的操作
        static void apply_action(Solution &solution, const Action &action) {

            switch (action.type) {
            case ActionType::INSERT_VERTEX:
                if (solution.is_route_in_solution(action.route)) {
                    solution.insert_vertex_before<false>(action.route, action.j, action.i);
                } else {
                    assert(action.j == 0);  // depot
                    solution.build_one_customer_route<false>(action.i);
                }
                break;
            case ActionType::REMOVE_VERTEX:
                solution.remove_vertex<false>(action.route, action.i);
                break;
            case ActionType::CREATE_ROUTE:
                assert(!solution.is_route_in_solution(action.route));
#ifndef NDEBUG
                {
                    int route = solution.request_route();
                    assert(route == action.route);
                    solution.release_route(route);
                }
#endif
                break;
            case ActionType::REMOVE_ROUTE:
                assert(solution.is_route_empty(action.route));
                solution.remove_route<false>(action.route);
                break;
            case ActionType::REVERSE_ROUTE_PATH:
                solution.reverse_route_path<false>(action.route, action.i, action.j);
                break;
            case ActionType::CREATE_ONE_CUSTOMER_ROUTE:
                solution.build_one_customer_route<false>(action.i);
                break;
            case ActionType::REMOVE_ONE_CUSTOMER_ROUTE:
                assert(action.i != solution.instance.get_depot());
                assert(solution.is_customer_in_solution(action.i));
                assert(solution.is_route_in_solution(action.route));
                assert(solution.get_route_index(action.i) == action.route);
                solution.remove_vertex<false>(action.route, action.i);
                assert(solution.is_route_empty(action.route));
                solution.remove_route<false>(action.route);
                break;
            }
        }
    };

}  // namespace cobra


#endif
