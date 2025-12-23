// 移动生成器 (Move Generators)
// 实现粒度规则（Granularity Rule）的核心数据结构
// 用于稀疏化搜索空间，只考虑有希望的移动
#ifndef _FILO2_MOVEGENERATORS_HPP_
#define _FILO2_MOVEGENERATORS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "../base/BinaryHeap.hpp"
#include "../base/Flat2DVector.hpp"
#include "../base/SparseIntSet.hpp"
#include "../base/VectorView.hpp"
#include "../instance/Instance.hpp"

namespace cobra {

    // Simple generator of incremental numbers.
    // 时间戳生成器：用于标记移动生成器的更新时间
    class TimestampGenerator : private NonCopyable<TimestampGenerator> {
    public:
        TimestampGenerator() = default;
        inline unsigned long get() {
            return value;
        }
        inline void increment() {
            ++value;
        }

    private:
        unsigned long value = 0;
    };

    // Class representing a move generator or static move descriptor.
    // 移动生成器类
    // 表示一个潜在的移动：将顶点i移动到顶点j附近
    // 每个移动生成器存储移动的增量成本(delta)和其他元数据
    class MoveGenerator : private NonCopyable<MoveGenerator> {
    public:
        MoveGenerator(int i, int j) : i(i), j(j) { }

        // 获取第一个顶点（要移动的顶点）
        inline auto get_first_vertex() const {
            return i;
        }
        // 获取第二个顶点（目标位置附近的顶点）
        inline auto get_second_vertex() const {
            return j;
        }
        // 获取移动的增量成本
        inline auto get_delta() const {
            return delta;
        }
        // 设置移动的增量成本
        inline auto set_delta(double value) {
            delta = value;
        }
        // 获取在堆中的索引（用于优先队列）
        inline auto get_heap_index() const {
            return heap_index;
        }
        // 设置在堆中的索引
        inline auto set_heap_index(int index) {
            heap_index = index;
        }
        // 检查是否已为弹射链算子计算过
        inline bool is_computed_for_ejch() const {
            return computed_for_ejch;
        }
        // 设置弹射链计算标记
        inline void set_computed_for_ejch(bool value) {
            computed_for_ejch = value;
        }

    private:
        int i;                          // 第一个顶点
        int j;                          // 第二个顶点
        double delta = 0.0;             // 移动的增量成本
        int heap_index = -1;            // 在堆中的索引
        bool computed_for_ejch = false; // 是否已为弹射链计算过
    };

    // 移动生成器比较器（用于堆）
    struct MGCompare {
        auto operator()(MoveGenerator* mg1, MoveGenerator* mg2) {
            assert(mg1 && mg2);
            return mg1->get_delta() - mg2->get_delta();
        }
    };

    // 获取移动生成器在堆中的索引
    struct MGGetIdx {
        auto operator()(MoveGenerator* mg1) {
            assert(mg1);
            return mg1->get_heap_index();
        }
    };

    // 设置移动生成器在堆中的索引
    struct MGSetIdx {
        void operator()(MoveGenerator* mg1, int idx) {
            assert(mg1);
            mg1->set_heap_index(idx);
        }
    };

    // 更新移动生成器的delta值
    struct MGUpdate {
        auto operator()(MoveGenerator* mg1, double delta) {
            assert(mg1);
            const auto res = mg1->get_delta() - delta;
            mg1->set_delta(delta);
            return res;
        }
    };

    // Heap data structure specialized to contain move generators.
    // 移动生成器堆
    // 使用二叉堆维护移动生成器，按delta值排序
    // 支持高效的插入、删除和更新操作
    class MoveGeneratorsHeap
        : private NonCopyable<MoveGeneratorsHeap>//私有继承两个类
        , private BinaryHeap<MoveGenerator*, MGCompare, MGGetIdx, MGSetIdx, MGUpdate, -1> {

        typedef BinaryHeap<MoveGenerator*, MGCompare, MGGetIdx, MGSetIdx, MGUpdate> BHeap;

    public:
        MoveGeneratorsHeap() = default;
        MoveGeneratorsHeap(const MoveGeneratorsHeap& other) = delete;//删除拷贝构造函数


        void reset() {//重置堆，清空所有元素。
            BHeap::reset();
        }
        bool is_empty() const {//检查堆是否为空
            return BHeap::empty();
        }
        void insert(MoveGenerator* mg) {//插入移动生成器
            BHeap::insert(mg);
        }
        MoveGenerator* get() {//获取并移除堆顶的移动生成器
            return BHeap::get();
        }
        void remove(int heap_index) {//移除指定索引的移动生成器
            BHeap::remove(heap_index);
        }
        void change_value(int heap_index, double value) {//更新指定索引的移动生成器的delta值
            BHeap::update(heap_index, value);
        };
        int size() const {//获取堆中元素数量
            return BHeap::size();
        };
        MoveGenerator* spy(int heap_index) {//查看指定索引的移动生成器（不修改堆）
            return BHeap::spy(heap_index);
        }

        static const int unheaped = -1;//表示移动生成器不在堆中的特殊索引值

    private:
        void dump() override {//打印堆中所有移动生成器的信息
            for (auto n = 0; n < size(); n++) {
                const auto& move = spy(n);
                std::cout << "[" << n << "] (" << move->get_first_vertex() << ", " << move->get_second_vertex()
                          << ") delta = " << move->get_delta() << " heap index = " << move->get_heap_index() << "\n";
            }
        }
    };

    // K-nearest neighbors move generators.
    // 移动生成器管理类
    // 实现粒度规则：只为每个顶点的k个最近邻居创建移动生成器
    // 支持动态调整活跃邻居数（通过gamma参数）
    class MoveGenerators : private NonCopyable<MoveGenerators> {
    public:
        // 构造函数，为每个顶点的k个最近邻居创建移动生成器并按边成本升序排序
        // @param instance CVRP实例
        // @param k 每个顶点考虑的最大邻居数
        MoveGenerators(const Instance& instance, int k)
            :  // -1 as neighbors[0] == i and we skip (i, i) move gens.
            // neighbors[0]总是i自己，所以实际邻居数是k-1
            max_num_neighbors(std::min(k, instance.get_vertices_num() - 1))
            , heap(MoveGeneratorsHeap())
            , vertex_timestamp(instance.get_vertices_num(), 0)
            , vertices_in_updated_moves(instance.get_vertices_num())
            , unique_endpoints(instance.get_vertices_num()) {

            update_bits.resize(instance.get_vertices_num(), 2);

            base_move_indices_involving.resize(instance.get_vertices_num());
            active_move_indices_involving_1st.resize(instance.get_vertices_num());
            current_num_neighbors.resize(instance.get_vertices_num(), 0);

            const int neighbors_begin = 1;  // 跳过neighbors[0]（即i自己）
            const int neighbors_end = neighbors_begin + max_num_neighbors;

            // 为每个顶点的k个最近邻居创建移动生成器
            for (int i = instance.get_vertices_begin(); i < instance.get_vertices_end(); ++i) {
                const auto& ineighbors = instance.get_neighbors_of(i);

                for (int p = neighbors_begin; p < neighbors_end; ++p) {//遍历k个最近邻居
                    assert(p < static_cast<int>(ineighbors.size()));
                    const int j = ineighbors[p];
                    const double cost = instance.get_cost(i, j);

                    // 插入客户对(a,b)到移动生成器的函数
                    const auto insert = [this, cost](int a, int b) {
                        const int base_idx = moves.size();//插入位置的偶数索引
                        assert(base_idx == get_base_move_generator_index(base_idx));
                        moves.emplace_back(a, b);
                        moves.emplace_back(b, a);
                        edge_costs.emplace_back(cost);
                        assert(std::fabs(get_edge_cost(moves[base_idx]) - cost) < 0.01);
                        assert(std::fabs(get_edge_cost(moves[base_idx + 1]) - cost) < 0.01);
                        base_move_indices_involving[a].emplace_back(base_idx);//记录对应偶数索引
                        base_move_indices_involving[b].emplace_back(base_idx);
                        assert(moves[base_idx].get_first_vertex() == a);
                        assert(moves[base_idx + 1].get_first_vertex() == b);
                    };

                    assert(i != j);
                    if (i < j) {//避免重复插入
                        insert(i, j);
                        continue;
                    }

                    const auto& jneighbors = instance.get_neighbors_of(j);//j的邻居列表

                    const double cij = instance.get_cost(i, j);
                    const double cjn = instance.get_cost(j, jneighbors[neighbors_end - 1]);//j与其第k个邻居的距离
                    if (cij > cjn) {//ij的距离比j与其第k个邻居的距离还远
                        insert(j, i);//插入j i，因为后续遍历j的邻居时是无法找到i的
                        continue;
                    }

                    if (std::fabs(cij - cjn) < 0.00001) {//i是j的第k个邻居
                        bool add = true;
                        for (int idx : base_move_indices_involving[j]) {//遍历j的所有基础索引
                            if (moves[idx].get_second_vertex() == i) {//j,i已经存在
                                add = false;
                                break;
                            }
                        }

                        if (add) {
                            insert(j, i);
                        }
                        continue;
                    }
                }
            }

            // 对每个顶点的移动生成器按边成本排序
            // 这样可以优先考虑成本较低的移动
            for (int i = instance.get_vertices_begin(); i < instance.get_vertices_end(); ++i) {
                std::sort(base_move_indices_involving[i].begin(), base_move_indices_involving[i].end(), [this](int a, int b) {
                    const auto& a_move = get(a);
                    const double a_cost = get_edge_cost(a_move);

                    const auto& b_move = get(b);
                    const double b_cost = get_edge_cost(b_move);

                    return a_cost < b_cost;
                });
            }

            // We only consider base indices (thus size / 2).
            // 只考虑基础索引（因此大小为size/2）
            // 因为每条边(i,j)和(j,i)共享一个基础索引
            move_active_in_1st.resize(moves.size() / 2, false);
            move_active_in_2nd.resize(moves.size() / 2, false);
        }

        // 获取指定索引的移动生成器（非const版本）
        inline MoveGenerator& get(int idx) {
            assert(idx >= 0 && idx < static_cast<int>(moves.size()));
            return moves[idx];
        }

        // 获取指定索引的移动生成器（const版本）
        inline const MoveGenerator& get(int idx) const {
            assert(idx >= 0 && idx < static_cast<int>(moves.size()));
            return moves[idx];
        }

        // 获取涉及指定顶点作为第一个顶点的活跃移动生成器索引
        // 返回所有形如(vertex, j)的移动生成器索引
        inline const auto& get_move_generator_indices_involving_1st(int vertex) const {
            return active_move_indices_involving_1st[vertex];
        }

        // 获取涉及指定顶点作为第二个顶点的活跃移动生成器索引
        // 返回所有形如(j, vertex)的移动生成器索引
        // 通过twin_functor将(vertex, j)转换为(j, vertex)
        inline auto get_move_generator_indices_involving_2nd(int vertex) const {
            const auto& v = active_move_indices_involving_1st[vertex];
            return VectorView<decltype(v.begin()), twin_functor>(v.begin(), v.end());
        }

        // 获取涉及指定顶点的所有活跃移动生成器的基础索引
        // 返回所有涉及vertex的移动生成器的基础索引（偶数索引）
        inline auto get_move_generator_indices_involving(int vertex) const {
            const auto& v = active_move_indices_involving_1st[vertex];
            return VectorView<decltype(v.begin()), base_functor>(v.begin(), v.end());
        }

        // 设置活跃移动生成器的百分比（粒度规则）
        // @param percentage 每个顶点的活跃邻居百分比（0.0-1.0）
        // @param vertices 需要更新的顶点列表
        //
        // 粒度规则：只考虑每个顶点的前k个最近邻居
        // percentage[v] * max_num_neighbors = 实际考虑的邻居数
        void set_active_percentage(std::vector<double>& percentage, std::vector<int>& vertices) {
            vertices_getting_updated.clear();
            vertices_in_updated_moves.clear();

            for (const int vertex : vertices) {
                // Convert the percentage to the number of neighbors to consider.
                // 将百分比转换为实际考虑的邻居数
                const int num_neigbors = std::round(percentage[vertex] * max_num_neighbors);
                assert(num_neigbors <= static_cast<int>(base_move_indices_involving[vertex].size()));

                // Check if we are already using the requested number of neighbors.
                // 检查是否已经在使用请求的邻居数
                if (num_neigbors == current_num_neighbors[vertex]) {
                    continue;
                }
                //顶点的移动生成器需要更新
                vertices_getting_updated.push_back(vertex);

                // 如果需要减少邻居数，停用一些移动生成器
                if (num_neigbors < current_num_neighbors[vertex]) {
                    for (int n = num_neigbors; n < current_num_neighbors[vertex]; ++n) {
                        const int idx = base_move_indices_involving[vertex][n];
                        const MoveGenerator& move = moves[idx];

                        assert(is_active_in(move, vertex));
                        set_not_active_in(move, vertex);

                        // 记录受影响的顶点
                        vertices_in_updated_moves.insert(move.get_first_vertex());
                        vertices_in_updated_moves.insert(move.get_second_vertex());
                    }
                } else {
                    // 如果需要增加邻居数，激活一些移动生成器
                    for (int n = current_num_neighbors[vertex]; n < num_neigbors; ++n) {
                        const int idx = base_move_indices_involving[vertex][n];
                        const MoveGenerator& move = moves[idx];

                        assert(!is_active_in(move, vertex));
                        set_active_in(move, vertex);

                        // 记录受影响的顶点
                        vertices_in_updated_moves.insert(move.get_first_vertex());
                        vertices_in_updated_moves.insert(move.get_second_vertex());
                    }
                }

                current_num_neighbors[vertex] = num_neigbors;
            }

            unique_move_generators.clear();
            unique_endpoints.clear();

            // 更新受影响顶点的活跃移动生成器列表
            for (const int vertex : vertices_in_updated_moves.get_elements()) {
                unique_move_generators.clear();
                unique_endpoints.clear();

                // We need to scan all base move indices as some movegen may be active due to the other vertex.
                // 需要扫描所有基础移动索引，因为某些移动生成器可能因为另一个顶点而活跃
                // 例如：(i,j)可能因为i或j（或两者）而活跃
                for (int base_idx : base_move_indices_involving[vertex]) {
                    assert(base_idx == get_base_move_generator_index(base_idx));
                    const auto& move = moves[base_idx];

                    // 跳过完全不活跃的移动生成器
                    if (!is_active_in_any(move)) {
                        continue;
                    }

                    // 确定正确的索引：如果vertex是第一个顶点，使用base_idx；否则使用twin索引
                    const int idx = vertex == move.get_first_vertex() ? base_idx : get_twin_move_generator_index(base_idx);

                    // 获取第二个顶点（移动的目标顶点）
                    const int j = moves[idx].get_second_vertex();
                    // 确保每个端点只出现一次（去重）
                    if (!unique_endpoints.contains(j)) {
                        unique_endpoints.insert_without_checking_existance(j);
                        unique_move_generators.push_back(idx);
                    }
                }
                // 更新该顶点的活跃移动生成器列表
                active_move_indices_involving_1st[vertex] = unique_move_generators;
            }
        }

        // 获取移动生成器堆（用于局部搜索中的优先队列）
        inline MoveGeneratorsHeap& get_heap() {
            return heap;
        }

        // 获取孪生移动生成器的索引
        // (i,j)和(j,i)是孪生关系，索引相差1
        // 使用XOR 1来切换：偶数变奇数，奇数变偶数
        static inline int get_twin_move_generator_index(int index) {
            return index ^ 1;
        }

        // 获取基础移动生成器的索引（偶数索引）
        // 将任意索引转换为对应的基础索引
        // 使用AND ~1将最低位清零：奇数变偶数，偶数不变
        static inline int get_base_move_generator_index(int index) {
            return index & ~1;
        }

        // 获取时间戳生成器（用于增量更新）
        inline TimestampGenerator& get_timestamp_generator() {
            return timegen;
        }

        // 获取顶点时间戳向量（用于跟踪顶点的更新状态）
        inline std::vector<unsigned long>& get_vertex_timestamp() {
            return vertex_timestamp;
        }

        // 获取更新位向量（用于标记哪些移动需要更新）
        inline Flat2DVector<bool>& get_update_bits() {
            return update_bits;
        }

        // 获取移动生成器对应边的成本
        // 通过指针算术计算索引，然后除以2得到边成本索引
        inline double get_edge_cost(const MoveGenerator& move) {
            return edge_costs[(&move - moves.data()) / 2];
        }

        // 获取移动生成器的总数
        inline size_t size() {
            return moves.size();
        }

    private:
        // Sets that move is active because of vertex.
        // 设置移动生成器因为指定顶点而活跃
        //
        // 一个移动(i,j)可以因为i或j（或两者）而活跃
        // 这取决于粒度规则中i和j各自的邻居设置
        inline void set_active_in(const MoveGenerator& move, int vertex) {
            const int idx = (&move - moves.data()) / 2;
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            if (vertex == move.get_first_vertex()) {
                move_active_in_1st[idx] = true;
            } else {
                move_active_in_2nd[idx] = true;
            }
        }

        // Sets that move is no longer active because of vertex.
        // 设置移动生成器不再因为指定顶点而活跃
        inline void set_not_active_in(const MoveGenerator& move, int vertex) {
            const int idx = (&move - moves.data()) / 2;
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            if (vertex == move.get_first_vertex()) {
                move_active_in_1st[idx] = false;
            } else {
                move_active_in_2nd[idx] = false;
            }
        }

        // 检查移动生成器是否因为指定顶点而活跃
        inline bool is_active_in(const MoveGenerator& move, int vertex) const {
            const int idx = (&move - moves.data()) / 2;
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            return vertex == move.get_first_vertex() ? move_active_in_1st[idx] : move_active_in_2nd[idx];
        }

        // 检查移动生成器是否因为任一顶点而活跃
        // 只要第一个或第二个顶点使其活跃，就返回true
        inline bool is_active_in_any(const MoveGenerator& move) const {
            const int idx = (&move - moves.data()) / 2;
            return move_active_in_1st[idx] || move_active_in_2nd[idx];
        }

        // 每个顶点考虑的最大邻居数（粒度参数k）
        const int max_num_neighbors;

        // Vector of move generators. In even position it contains move gen (i, j), and in odd positions move gen (j, i).
        // 移动生成器向量
        // 偶数位置存储(i,j)，奇数位置存储(j,i)
        // 例如：moves[0]=(i,j), moves[1]=(j,i), moves[2]=(k,l), moves[3]=(l,k)
        std::vector<MoveGenerator> moves;

        // Containes all the even indexed move generator indices associated with every vertex.
        // Use get_twin_move_generator_index to get the twin move generator index.
        // 包含与每个顶点相关的所有偶数索引移动生成器索引
        // 对于顶点i，存储所有涉及i的移动的基础索引（偶数索引）
        // 使用get_twin_move_generator_index获取孪生移动生成器索引
        std::vector<std::vector<int>> base_move_indices_involving;

        // Indices of *active* move generators (i, j) for every vertex i.
        // 每个顶点i的活跃移动生成器(i,j)的索引
        // 只包含当前根据粒度规则激活的移动生成器
        std::vector<std::vector<int>> active_move_indices_involving_1st;

        // Edge cost of the associated move generator. Since costs are symmetric and we are storing both move generators (ij) and (ji)
        // consecutively, the vector edge_cost contains c(ij)=c(ji) once only.
        // 关联移动生成器的边成本
        // 由于成本是对称的，且我们连续存储(ij)和(ji)
        // edge_costs向量只存储c(ij)=c(ji)一次
        std::vector<double> edge_costs;

        // For every vertex, the number of neighbors currently active
        // 每个顶点当前活跃的邻居数
        // 用于跟踪粒度规则的当前状态
        std::vector<int> current_num_neighbors;

        // For every move generators, these vectors track whether they are active because of node i (first), node j (second), or both.
        // 对于每个移动生成器，这些向量跟踪它们是否因为节点i（第一个）、节点j（第二个）或两者而活跃
        // move_active_in_1st[idx]: 移动是否因为第一个顶点而活跃
        // move_active_in_2nd[idx]: 移动是否因为第二个顶点而活跃
        std::vector<bool> move_active_in_1st;
        std::vector<bool> move_active_in_2nd;

        // Stores ordered move generators during local search applications.
        // 在局部搜索应用期间存储有序的移动生成器
        // 使用二叉堆维护按增益排序的移动生成器
        MoveGeneratorsHeap heap;

        // The `updated_bits` are used by local search operators to identify whether `(i, j)`, `(j, i)` or both require an update upon a
        // move execution.
        // update_bits用于局部搜索算子识别在执行移动时
        // (i,j)、(j,i)或两者是否需要更新
        // 这是增量更新策略的一部分
        Flat2DVector<bool> update_bits;

        // Used by local search operators to identify whether move generators of a given vertex have already been updated.
        // 局部搜索算子用于识别给定顶点的移动生成器是否已经更新
        // 通过时间戳避免重复更新
        std::vector<unsigned long> vertex_timestamp;
        TimestampGenerator timegen;

        // The variables below are stored here to avoid re-allocations.
        // 以下变量存储在这里以避免重新分配
        // 这些是临时工作变量，重用以提高性能

        // Vertices that are updated given the current number of neighbors and the required one (computed from the percentage vector).
        // 根据当前邻居数和所需邻居数（从百分比向量计算）需要更新的顶点
        std::vector<int> vertices_getting_updated;

        // 涉及已更新移动的顶点集合
        SparseIntSet vertices_in_updated_moves;

        // 唯一的移动生成器列表（去重后）
        std::vector<int> unique_move_generators;

        // 唯一的端点集合（用于去重）
        SparseIntSet unique_endpoints;
    };


}  // namespace cobra

#endif