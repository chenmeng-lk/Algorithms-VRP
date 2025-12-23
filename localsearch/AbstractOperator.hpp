#ifndef _FILO2_ABSTRACTOPERATOR_HPP_
#define _FILO2_ABSTRACTOPERATOR_HPP_
// 该文件定义了局部搜索操作符的抽象基类和通用实现类。
// 局部搜索操作符基于SMDs（Submove Descriptors）和GNs（Generalized Neighborhoods），
// 用于在车辆路径问题（VRP）中执行局部搜索改进。

#include "../instance/Instance.hpp"
#include "../movegen/MoveGenerators.hpp"
#include "../solution/Solution.hpp"

// Update bits are used to restrict the update of SMDs for asymmetric neighborhoods. After a local search move is executed, if
// `updated_bits.at(i, UPDATE_BITS_FIRST)` is true for an affected vertex `i` identifies that move generators `(i, j)` where `i` is in first
// position need to be updated. Similarly, `updated_bits.at(i, UPDATE_BITS_SECOND)` identifies that move generators `(j, i)` where `i` is
// the second vertex require an update.
//
// For more info refer to Appendix B.2.0.1 of the FILO paper.
// 更新位用于限制非对称邻域中SMDs的更新。在执行局部搜索移动后，如果受影响顶点i的updated_bits.at(i, UPDATE_BITS_FIRST)为真，
// 则表示需要更新以i为第一个顶点的移动生成器(i, j)。类似地，updated_bits.at(i, UPDATE_BITS_SECOND)表示需要更新以i为第二个顶点的移动生成器(j, i)。
#define UPDATE_BITS_FIRST (0)
#define UPDATE_BITS_SECOND (1)

namespace cobra {

    // An abstract local search operator based on SMDs and GNs.
    // 基于SMDs和GNs的抽象局部搜索操作符。
    class AbstractOperator : public NonCopyable<AbstractOperator> {
    public:
        // 构造函数：初始化操作符，传入实例、移动生成器和容差。
        AbstractOperator(const Instance& instance_, MoveGenerators& moves_, double tolerance_)
            : instance(instance_), moves(moves_), heap(moves.get_heap()), tolerance(tolerance_), update_bits(moves_.get_update_bits()) { }
        virtual ~AbstractOperator() = default;

        // Applies the operator to `solution`.
        // 将操作符应用于解决方案solution，返回是否进行了改进。
        virtual bool apply_rough_best_improvement(Solution& solution) = 0;

    protected:
        // Performs some setup useful during a local search cycle, if necessary.
        virtual void pre_processing(Solution& solution) = 0; // 局部搜索循环前的预处理

        // Computes the effect of `move` to `solution` without caring about whether its application would make `solution` infeasible. This
        // function is currently used only for debug purposes. For the actual move cost computation see for example `symmetric_init`.
        virtual double compute_cost(const Solution& solution, const MoveGenerator& move) = 0; // 计算移动的成本影响（仅调试用）

        // Returns whether applying `move` to `solution` would create a feasible solution.
        virtual bool is_feasible(Solution& solution, const MoveGenerator& move) = 0; // 检查移动是否可行

        // Applies `move` to `solution` and populates `affected_vertices` with the vertices `i` for which at least one move generator `(i,
        // j)` or `(j, i)` require an update.
        virtual void execute(Solution& solution, const MoveGenerator& move, SparseIntSet& affected_vertices) = 0; // 执行移动并记录受影响顶点

        // Performs some final cleanup at the end of the local search cycle, if necessary.
        virtual void post_processing(Solution& solution) = 0; // 局部搜索循环后的清理

        const Instance& instance; // VRP实例引用
        MoveGenerators& moves; // 移动生成器引用
        MoveGeneratorsHeap& heap; // 移动生成器堆引用
        const double tolerance; // 改进容忍度

        // It allows to identify whether `(i, j)`, `(j, i)` or both require an update after a move execution. Values are set during
        // `execute` for vertices affected by the move application. This data structure is shared across operator to minimize memory
        // occupation.
        Flat2DVector<bool>& update_bits; // 更新位矩阵，用于标识需要更新的移动生成器
    };

    // A base local search operator containing code which is operator independent such as SMDs initialization, update and search logic.
    // Templating is used to perform compile time inheritance.
    // 通用局部搜索操作符基类，包含操作符无关的SMDs初始化、更新和搜索逻辑。使用模板实现编译时继承。
    //
    // 模板参数T的虚函数说明：
    // - T::prepare_cache12(solution, vertex): 准备顶点的缓存数据，用于快速计算移动成本（顶点可作为第一或第二个顶点）
    // - T::prepare_cache12(solution, vertex, backup): depot专用版本，需要backup参数确定路线
    // - T::prepare_cache1(solution, vertex): 准备顶点作为第一个顶点的缓存（仅非对称操作符）
    // - T::prepare_cache2(solution, vertex): 准备顶点作为第二个顶点的缓存（仅非对称操作符）
    // - T::compute_cost(move, cache1, cache2): 基于缓存计算移动的成本变化
    // - T::compute_cost_pair(move, cache1, cache2): 同时计算(i,j)和(j,i)两个方向的成本（仅非对称操作符）
    // - T::is_feasible(solution, move): 检查移动是否可行
    // - T::execute(solution, move, affected_vertices): 执行移动并记录受影响顶点
    // - T::pre_processing(solution): 局部搜索前的预处理
    // - T::post_processing(solution): 局部搜索后的清理
    template <class T, bool handle_partial_solutions = false>
    class CommonOperator : public T {

    public:
        CommonOperator(const Instance& instance_, MoveGenerators& moves_, double tolerance_)
            : T(instance_, moves_, tolerance_)
            , timegen(moves_.get_timestamp_generator())
            , affected_vertices(instance_.get_vertices_num()) { }

        // Applies a local search cycle to `solution` and returns whether there were some improvements.
        // 对解决方案执行局部搜索循环，返回是否有改进
        bool apply_rough_best_improvement(Solution& solution) {
            T::heap.reset(); // 重置堆

            // T::pre_processing - 虚函数：在局部搜索循环开始前执行预处理操作
            T::pre_processing(solution); // 调用预处理

            initialize_descriptors(solution); // 初始化SMDs
            assert(__check_deltas(solution)); // 调试检查：验证堆中移动的delta值


            auto improved = false; // 改进标志

            auto index = 0; // 堆索引

            // 主搜索循环：遍历堆中的移动
            while (index < T::heap.size()) {
                auto& move = *T::heap.spy(index++); // 查看堆中下一个移动（不弹出）

                if constexpr (handle_partial_solutions) {
                    // 如果处理部分解，检查移动涉及的两个顶点是否都在当前解中
                    if (!solution.is_vertex_in_solution(move.get_first_vertex()) ||
                        !solution.is_vertex_in_solution(move.get_second_vertex())) {
                        continue;
                    }
                }

                // T::is_feasible - 虚函数：检查应用移动后解是否仍然可行（满足容量等约束）
                if (!T::is_feasible(solution, move)) { // 检查移动可行性
                    continue;
                }

#ifndef NDEBUG
                auto old_cost = solution.get_cost(); // 记录旧成本（仅调试用）
#endif

                // T::execute - 虚函数：执行移动操作，修改解并记录受影响的顶点
                T::execute(solution, move, affected_vertices); // 执行移动

                assert(old_cost > solution.get_cost()); // 验证成本确实降低，因为堆中只会存储改进移动所以成本必降
                assert(solution.is_feasible()); // 验证解仍可行

                improved = true; // 标记有改进
                index = 0; // 重置索引，重新从头搜索（因为解已改变）

                descriptors_update(solution); // 更新受影响的SMDs
                assert(__check_deltas(solution)); // 再次验证delta值

                affected_vertices.clear(); // 清空受影响顶点集合
            }

            // T::post_processing - 虚函数：在局部搜索循环结束后执行清理操作
            T::post_processing(solution); // 调用后处理

            return improved; // 返回是否改进
        }


    private:
        // Check the delta of heaped moves.
        // 检查堆中移动的delta值是否正确（调试用）
        bool __check_deltas(Solution& solution) {
            for (int i = 0; i < T::heap.size(); ++i) {
                auto& move = *T::heap.spy(i);

                if (!T::is_feasible(solution, move)) {
                    // Skip non feasible moves since the delta is wrong for inter-route only moves such as split and tails when i and j
                    // belong to the same route.
                    // 跳过不可行移动，因为对于某些移动（如split和tails），当i和j属于同一路线时delta值不正确
                    continue;
                }
                // T::compute_cost - 虚函数：计算移动对解的成本影响（不考虑可行性，主要用于调试验证）
                if (std::fabs(move.get_delta() - T::compute_cost(solution, move)) > 0.01) { // 检查存储的delta值与重新计算的值是否一致
                    std::cout << "Operator: " << typeid(this).name() << "\n";
                    std::cout << "Error: delta cost mismatch!!\n";
                    std::cout << "Stored = " << move.get_delta() << "\n";
                    std::cout << "Computed = " << T::compute_cost(solution, move) << "\n";
                    int i = move.get_first_vertex();
                    int j = move.get_second_vertex();
                    std::cout << "\ti=" << i << "\n";
                    std::cout << "\tj=" << j << "\n";
                    int i_route = solution.get_route_index(i, j);
                    int j_route = solution.get_route_index(j, i);
                    solution.print(i_route);
                    solution.print(j_route);
                    return false;
                }
            }
            return true;
        }

        // Initializes SMDs at the beginning of a new local search cycle.
        // 在新的局部搜索循环开始时初始化SMDs
        inline void initialize_descriptors(const Solution& solution) {
            if constexpr (T::is_symmetric) { // 根据操作符对称性选择初始化方式
                symmetric_init(solution); // 对称操作符初始化
            } else {
                asymmetric_init(solution); // 非对称操作符初始化
            }
        }

        // Performs an initialization for a symmetric operator. Thus only one between (i, j) and (j, i) will be initialized and used during
        // the search.
        // 对称操作符初始化：只初始化(i, j)和(j, i)中的一个
        inline void symmetric_init(const Solution& solution) {
            const auto currenttime = timegen.get() + 1; // 获取当前时间戳
            auto& vtimestamp = T::moves.get_vertex_timestamp(); // 顶点时间戳引用

            auto depot = false; // 标记是否遇到depot

            // Consider only moves involving vertices in the SVC.
            // 只考虑涉及SVC（Selected Vertex Candidates）中顶点的移动
            for (auto i = solution.get_svc_begin(); i != solution.get_svc_end(); i = solution.get_svc_next(i)) {

                if constexpr (handle_partial_solutions) {
                    if (!solution.is_vertex_in_solution(i)) { // 如果处理部分解且顶点不在解中，跳过
                        continue;
                    }
                }

                // Postpone depot handling. This way we have better chances to compact move generators and reuse cached data.
                // 推迟处理depot，以便更好地压缩移动生成器和重用缓存数据
                if (i == T::instance.get_depot()) {
                    depot = true;
                    continue;
                }

                // Compute a partial move application.
                // T::prepare_cache12 - 虚函数：为顶点准备缓存数据，用于快速计算移动成本
                // 该函数预计算并缓存顶点的邻居信息和相关成本，避免重复计算
                // 参数：solution - 当前解，vertex - 要缓存的顶点
                // 返回：包含顶点邻居和成本信息的缓存结构（具体结构由子类定义）
                const auto icache = T::prepare_cache12(solution, i);

                // Iterate over move generators `(i, j)` so as to exploit `icache`.
                // 遍历以i为第一个顶点的移动生成器(i, j)，利用icache
                for (auto move_i1st_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                    const auto& move_i1st = T::moves.get(move_i1st_index);
                    const auto j = move_i1st.get_second_vertex();

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(j)) { // 如果顶点j不在解中，跳过
                            continue;
                        }
                    }

                    // Skip the move if we have already processed move `(j, i)`. This could happen since we might have both `i` and `j` in
                    // the SVC.
                    // 如果已经处理过移动(j, i)，则跳过（因为i和j可能都在SVC中）
                    if (vtimestamp[j] == currenttime) {
                        continue;
                    }

                    // Always consider a fixed move generator between `(i, j)` and `(j, i)` identified by the base index.
                    // 使用基索引确定的固定移动生成器（代表(i, j)和(j, i)对）
                    const auto move_index = MoveGenerators::get_base_move_generator_index(move_i1st_index);
                    auto& move = T::moves.get(move_index);

                    // Compute the remaining partial move application.
                    // T::prepare_cache12 - 当顶点是depot时，需要额外的backup参数来确定路线
                    const auto jcache = j == T::instance.get_depot() ? T::prepare_cache12(solution, j, i) : T::prepare_cache12(solution, j);

                    // T::compute_cost - 虚函数：基于两个顶点的缓存数据计算移动的成本变化
                    // 参数：move - 移动生成器，icache/jcache - 两个顶点的缓存数据
                    // 返回：应用该移动后的成本变化值（负值表示改进）
                    const auto delta = T::compute_cost(move, icache, jcache); // 计算移动的delta值

                    move.set_delta(delta); // 设置delta值
                    move.set_heap_index(MoveGeneratorsHeap::unheaped); // 标记为未在堆中
                    if (move.get_delta() < -T::tolerance) { // 如果改进超过容忍度，插入堆中
                        T::heap.insert(&move);
                    }
                }

                vtimestamp[i] = currenttime; // 标记顶点i已处理
            }

            // Do the same as above for the depot. Not super dry true.
            // 对depot进行类似处理，区别在之前i是由SVC得到，现在只能为0，并且icache被确定需要备份结点j
            if (depot) {

                const auto i = T::instance.get_depot();
                //遍历以i为第一个顶点的移动生成器(i, j)
                for (auto move_i1st_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                    const auto& move_i1st = T::moves.get(move_i1st_index);
                    const auto j = move_i1st.get_second_vertex();

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(j)) {// 如果顶点j不在解中，跳过
                            continue;
                        }
                    }

                    if (vtimestamp[j] == currenttime) {// 如果已经处理过移动(j, i)，则跳过
                        continue;
                    }

                    const auto move_index = MoveGenerators::get_base_move_generator_index(move_i1st_index);
                    auto& move = T::moves.get(move_index);

                    // T::prepare_cache12 - depot作为第一个顶点时，使用j作为backup参数
                    const auto icache = T::prepare_cache12(solution, i, j);
                    // Note that since `i` is the depot, `j` cannot be the depot.
                    // 注意：由于i是depot，j不可能是depot
                    const auto jcache = T::prepare_cache12(solution, j);

                    const auto delta = T::compute_cost(move, icache, jcache);// 计算移动的delta值
                    move.set_delta(delta);
                    move.set_heap_index(MoveGeneratorsHeap::unheaped);// 标记为未在堆中
                    if (move.get_delta() < -T::tolerance) {// 如果改进超过容忍度（成本下降超过0.01)，插入堆中（是改进操作）
                        T::heap.insert(&move);
                    }
                }

                vtimestamp[i] = currenttime;
            }

            timegen.increment(); // 递增时间戳
        }

        // Performs an initialization for an asymmetric operator. Both (i, j) and (j, i) are initialized and used throghout the search.
        // 非对称操作符初始化：同时初始化(i, j)和(j, i)
        //与对称的区别在计算delta时会同时计算(i, j)和(j, i)两个方向的成本
        inline void asymmetric_init(const Solution& solution) {

            const auto currenttime = timegen.get() + 1;

            auto& vtimestamp = T::moves.get_vertex_timestamp();

            auto depot = false;

            // To make the local search localized, we only consider move generators for which at least one between `i` and `j` are cached.
            // 为了使局部搜索局部化，只考虑至少有一个顶点（i或j）被缓存的移动生成器
            for (auto i = solution.get_svc_begin(); i != solution.get_svc_end(); i = solution.get_svc_next(i)) {

                if constexpr (handle_partial_solutions) {
                    if (!solution.is_vertex_in_solution(i)) {
                        continue;
                    }
                }

                // Postpone depot handling. This way we have better chances to compact move generators and reuse cached data.
                if (i == T::instance.get_depot()) {
                    depot = true;
                    continue;
                }

                // Compute a partial move application where `i` is either the first vertex or the second vertex.
                // T::prepare_cache12 - 为非对称操作符准备缓存，顶点可以作为第一或第二个顶点
                const auto icache = T::prepare_cache12(solution, i);

                for (const auto move_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                    auto& move = T::moves.get(move_index);

                    const auto j = move.get_second_vertex();

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(j)) {
                            continue;
                        }
                    }

                    // Skip the move if we have already processed move `(j, i)`. This could happen since we might have both `i` and `j` in
                    // the SVC.
                    if (vtimestamp[j] == currenttime) {
                        continue;
                    }

                    // Compute a partial move application where `j` is either the first or the second vertex.
                    const auto jcache = j == T::instance.get_depot() ? T::prepare_cache12(solution, j, i) : T::prepare_cache12(solution, j);

                    // Compute the actual move generator delta for (i, j) and (j, i).
                    // T::compute_cost_pair - 虚函数：同时计算(i, j)和(j, i)两个方向的移动成本
                    // 用于非对称操作符，返回一对delta值分别对应两个方向的移动
                    const auto [delta1, delta2] = T::compute_cost_pair(move, icache, jcache);

                    move.set_delta(delta1); // 设置(i, j)的delta值
                    move.set_heap_index(MoveGeneratorsHeap::unheaped);
                    if (move.get_delta() < -T::tolerance) {
                        T::heap.insert(&move);
                    }

                    const auto twin_move_index = MoveGenerators::get_twin_move_generator_index(move_index); // 获取孪生移动索引
                    auto& twin_move = T::moves.get(twin_move_index); // 获取孪生移动(j, i)
                    twin_move.set_delta(delta2); // 设置(j, i)的delta值
                    twin_move.set_heap_index(MoveGeneratorsHeap::unheaped);
                    if (twin_move.get_delta() < -T::tolerance) {
                        T::heap.insert(&twin_move);
                    }
                }

                vtimestamp[i] = currenttime;
            }

            if (depot) {//区别在之前i是由SVC得到，现在只能为0，并且icache被确定需要备份结点j

                const auto i = T::instance.get_depot();

                for (const auto move_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                    auto& move = T::moves.get(move_index);

                    const auto j = move.get_second_vertex();

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(j)) {
                            continue;
                        }
                    }

                    if (vtimestamp[j] == currenttime) {
                        continue;
                    }

                    const auto icache = T::prepare_cache12(solution, i, j);
                    // Vertex `j` cannot be the depot since `i` is the depot.
                    // 顶点j不可能是depot，因为i已经是depot
                    const auto jcache = T::prepare_cache12(solution, j);

                    const auto [delta1, delta2] = T::compute_cost_pair(move, icache, jcache);

                    move.set_delta(delta1);
                    move.set_heap_index(MoveGeneratorsHeap::unheaped);
                    if (move.get_delta() < -T::tolerance) {
                        T::heap.insert(&move);
                    }

                    const auto twin_move_index = MoveGenerators::get_twin_move_generator_index(move_index);
                    auto& twin_move = T::moves.get(twin_move_index);
                    twin_move.set_delta(delta2);
                    twin_move.set_heap_index(MoveGeneratorsHeap::unheaped);
                    if (twin_move.get_delta() < -T::tolerance) {
                        T::heap.insert(&twin_move);
                    }
                }

                vtimestamp[i] = currenttime;
            }

            timegen.increment();
        }

        // Updates affected SMDs once a move is applied.
        // 使用移动后，更新受影响的SMDs
        inline void descriptors_update(const Solution& solution) {

            if constexpr (T::is_symmetric) { // 根据操作符对称性选择更新方式
                symmetric_update(solution); // 对称更新
            } else {
                asymmetric_update(solution); // 非对称更新
            }
        }

        // Performs the update for a symmetric operator.
        // 对称操作符的更新。与初始化的区别在遍历受影响节点（初始化是遍历SVC）更新；根据delta值插入或更新堆中的移动
        inline void symmetric_update(const Solution& solution) {

            const auto currenttime = timegen.get() + 1;

            auto& vtimestamp = T::moves.get_vertex_timestamp();

            // Lambda函数：根据delta值插入或更新堆中的移动
            const auto heap_insert = [this](MoveGenerator& move, double delta) {
                if (delta > -T::tolerance) { // 如果不是改进移动（delta >= -tolerance）

                    if (move.get_heap_index() != MoveGeneratorsHeap::unheaped) { // 如果移动在堆中，移除它
                        T::heap.remove(move.get_heap_index());
                    }

                    move.set_delta(delta); // 更新delta值
                } else { // 如果是改进移动（delta < -tolerance）

                    if (move.get_heap_index() == MoveGeneratorsHeap::unheaped) { // 如果移动不在堆中，插入它
                        move.set_delta(delta);
                        T::heap.insert(&move);
                    } else { // 如果移动已在堆中，更新其值
                        T::heap.change_value(move.get_heap_index(), delta);
                    }
                }
            };

            auto depot = false;
            for (auto i : affected_vertices.get_elements()) { // 遍历受影响的顶点

                if constexpr (handle_partial_solutions) {
                    if (!solution.is_vertex_in_solution(i)) { // 如果顶点不在解中，跳过
                        continue;
                    }
                }

                // Postpone depot handling. This way we have better chances to compact move generators and reuse cached data.
                if (i == T::instance.get_depot()) {
                    depot = true;
                    continue;
                }

                // Compute a partial move application.
                // T::prepare_cache12 - 准备可用于计算两个方向移动的缓存
                auto icache = T::prepare_cache12(solution, i);

                // Iterate over move generators `(i, j)` so as to exploit `icache`.
                for (auto move_i1st_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                    const auto& move_i1st = T::moves.get(move_i1st_index);
                    const auto j = move_i1st.get_second_vertex();

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(j)) {
                            continue;
                        }
                    }

                    // Skip the move if we have already processed move `(j, i)`. This could happen since we might have both `i` and `j` in
                    // the SVC.
                    if (vtimestamp[j] == currenttime) {
                        continue;
                    }

                    // Always consider a fixed move generator between `(i, j)` and `(j, i)` identified by the base index.
                    const auto move_index = MoveGenerators::get_base_move_generator_index(move_i1st_index);
                    auto& move = T::moves.get(move_index);

                    const auto jcache = j == T::instance.get_depot() ? T::prepare_cache12(solution, j, i) : T::prepare_cache12(solution, j);

                    const auto delta = T::compute_cost(move, icache, jcache); // 重新计算移动的delta值
                    heap_insert(move, delta); // 插入或更新堆
                }
            }

            if (depot) {//区别在之前i是由SVC得到，现在只能为0，并且icache被确定需要备份结点j

                const auto i = T::instance.get_depot();

                for (auto move_i1st_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                    const auto& move_i1st = T::moves.get(move_i1st_index);
                    const auto j = move_i1st.get_second_vertex();

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(j)) {
                            continue;
                        }
                    }

                    if (vtimestamp[j] == currenttime) {
                        continue;
                    }

                    const auto move_index = MoveGenerators::get_base_move_generator_index(move_i1st_index);
                    auto& move = T::moves.get(move_index);

                    const auto icache = T::prepare_cache12(solution, i, j);
                    // Note that `j` cannot be the depot.
                    const auto jcache = T::prepare_cache12(solution, j);

                    const auto delta = T::compute_cost(move, icache, jcache);
                    heap_insert(move, delta);
                }

                vtimestamp[i] = currenttime;
            }

            timegen.increment();
        }

        // 非对称操作符的更新。
        inline void asymmetric_update(const Solution& solution) {

            const auto currenttime = timegen.get() + 1;

            auto& vtimestamp = T::moves.get_vertex_timestamp();

            const auto heap_insert = [this](MoveGenerator& move, double delta) {
                if (delta > -T::tolerance) {//不接受

                    if (move.get_heap_index() != MoveGeneratorsHeap::unheaped) {//在堆中则移除
                        T::heap.remove(move.get_heap_index());
                    }

                    move.set_delta(delta);
                } else {//接收但不在堆中

                    if (move.get_heap_index() == MoveGeneratorsHeap::unheaped) {
                        move.set_delta(delta);
                        T::heap.insert(&move);
                    } else {
                        T::heap.change_value(move.get_heap_index(), delta);//更新delta
                    }
                }
            };

            auto depot = false;
            for (auto i : affected_vertices.get_elements()) {

                if constexpr (handle_partial_solutions) {
                    if (!solution.is_vertex_in_solution(i)) {
                        continue;
                    }
                }

                // Postpone depot handling. This way we have better chances to compact move generators and reuse cached data.
                if (i == T::instance.get_depot()) {
                    depot = true;
                    continue;
                }

                const auto iupij = T::update_bits.at(i, UPDATE_BITS_FIRST); // 检查是否需要更新(i, j)
                const auto iupji = T::update_bits.at(i, UPDATE_BITS_SECOND); // 检查是否需要更新(j, i)

                // Perform a restricted update guided by the update bits state.
                // 根据更新位的状态进行受限更新
                if (iupij && iupji) {  // Update both `(i, j)` and `(j, i)`.
                    // 需要同时更新(i, j)和(j, i)

                    // T::prepare_cache12 - 准备可用于计算两个方向移动的缓存
                    const auto icache = T::prepare_cache12(solution, i); // 准备i的缓存（作为第一或第二个顶点）

                    for (const auto move_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                        auto& move = T::moves.get(move_index);
                        const auto j = move.get_second_vertex();

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(j)) {
                                continue;
                            }
                        }

                        // Move generators `(j, i)` and `(i, j)` may have been already updated. We need to check since update bits are not
                        // symmetric.
                        // 移动生成器(j, i)和(i, j)可能已经被更新。由于更新位不对称，需要检查
                        if (vtimestamp[j] == currenttime) {

                            const auto jupji = T::update_bits.at(j, UPDATE_BITS_FIRST);
                            const auto jupij = T::update_bits.at(j, UPDATE_BITS_SECOND);

                            if (jupji && jupij) {
                                // Both `(i, j)` and `(j, i)` were already updated.
                                // 两个移动都已被更新，无需处理
                            } else if (jupji) {  // Move generator `(j, i)` was already updated, thus update `(i, j)` only.
                                // 只有(j, i)被更新过，因此只更新(i, j)

                                // T::prepare_cache2 - 虚函数：准备顶点作为第二个顶点时的缓存数据
                                const auto jcache = j == T::instance.get_depot() ? T::prepare_cache2(solution, j, i)
                                                                                 : T::prepare_cache2(solution, j);
                                const auto delta = T::compute_cost(move, icache, jcache);

                                heap_insert(move, delta);

                            } else if (jupij) {  // Move generator `(i, j)` was already updated, thus update `(j, i)` only.
                                // 只有(i, j)被更新过，因此只更新(j, i)

                                // T::prepare_cache1 - 虚函数：准备顶点作为第一个顶点时的缓存数据
                                const auto jcache = j == T::instance.get_depot() ? T::prepare_cache1(solution, j, i)
                                                                                 : T::prepare_cache1(solution, j);

                                const auto twin_move_index = MoveGenerators::get_twin_move_generator_index(move_index);
                                auto& twin_move = T::moves.get(twin_move_index);

                                const auto twin_delta = T::compute_cost(twin_move, jcache, icache);

                                heap_insert(twin_move, twin_delta);
                            }

                        } else {  // Move generators involving `j` were not updated before. Update both `(i, j)` and `(j, i)`.
                            // 涉及顶点j的移动生成器之前没有被更新过。同时更新(i, j)和(j, i)

                            const auto jcache = j == T::instance.get_depot() ? T::prepare_cache12(solution, j, i)
                                                                             : T::prepare_cache12(solution, j);

                            const auto twin_move_index = MoveGenerators::get_twin_move_generator_index(move_index);
                            auto& twin_move = T::moves.get(twin_move_index);

                            const auto [delta1, delta2] = T::compute_cost_pair(twin_move, icache, jcache);

                            heap_insert(move, delta1);
                            heap_insert(twin_move, delta2);
                        }
                    }

                } else if (iupij) {  // Update only `(i, j)`.
                    // 只需要更新(i, j)

                    // T::prepare_cache1 - 准备顶点i作为第一个顶点的缓存
                    const auto icache = T::prepare_cache1(solution, i); // 准备i的缓存（作为第一个顶点）

                    for (const auto move_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                        auto& move = T::moves.get(move_index);
                        const auto j = move.get_second_vertex();

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(j)) {
                                continue;
                            }
                        }

                        if (vtimestamp[j] != currenttime ||               // Move gen of vertex `j` not process before ...
                            (vtimestamp[j] == currenttime &&              // ... or ...
                             !T::update_bits.at(j, UPDATE_BITS_SECOND)))  // move gen of vertex `j` were processed but `(i, j)` was not
                                                                          // updated because not required.
                        // 如果顶点j的移动生成器之前没有被处理过，或者被处理过但(i, j)没有被更新（因为不需要）
                        {

                            // T::prepare_cache2 - 准备顶点j作为第二个顶点的缓存
                            const auto jcache = j == T::instance.get_depot() ? T::prepare_cache2(solution, j, i)
                                                                             : T::prepare_cache2(solution, j);
                            const auto delta = T::compute_cost(move, icache, jcache);

                            heap_insert(move, delta);
                        }
                    }

                } else if (iupji) {  // Update only `(j, i)`.
                    // 只需要更新(j, i)

                    // T::prepare_cache2 - 准备顶点i作为第二个顶点的缓存
                    const auto icache = T::prepare_cache2(solution, i); // 准备i的缓存（作为第二个顶点）

                    for (const auto move_index : T::moves.get_move_generator_indices_involving_2nd(i)) {

                        auto& move = T::moves.get(move_index);
                        const auto j = move.get_first_vertex();

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(j)) {
                                continue;
                            }
                        }

                        if (vtimestamp[j] != currenttime ||              // Move gen of vertex `j` not process before ...
                            (vtimestamp[j] == currenttime &&             // ... or ...
                             !T::update_bits.at(j, UPDATE_BITS_FIRST)))  // move gen of vertex `j` were processed but `(j, i)` was not
                                                                         // updated because not required.
                        // 如果顶点j的移动生成器之前没有被处理过，或者被处理过但(j, i)没有被更新（因为不需要）
                        {

                            // T::prepare_cache1 - 准备顶点j作为第一个顶点的缓存
                            const auto jcache = j == T::instance.get_depot() ? T::prepare_cache1(solution, j, i)
                                                                             : T::prepare_cache1(solution, j);
                            const auto delta = T::compute_cost(move, jcache, icache);

                            heap_insert(move, delta);
                        }
                    }
                }

                vtimestamp[i] = currenttime;
            }

            // Same as above but for the depot.
            // 对depot进行类似处理,区别在之前i是由SVC得到，现在只能为0，并且icache被确定需要备份结点j
            if (depot) {

                const auto i = T::instance.get_depot();

                const auto iupij = T::update_bits.at(i, UPDATE_BITS_FIRST);
                const auto iupji = T::update_bits.at(i, UPDATE_BITS_SECOND);

                if (iupij && iupji) {

                    for (const auto move_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                        auto& move = T::moves.get(move_index);

                        const auto j = move.get_second_vertex();

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(j)) {
                                continue;
                            }
                        }

                        if (vtimestamp[j] == currenttime) {

                            const auto jupji = T::update_bits.at(j, UPDATE_BITS_FIRST);
                            const auto jupij = T::update_bits.at(j, UPDATE_BITS_SECOND);

                            if (jupji && jupij) {
                            } else if (jupji) {

                                // T::prepare_cache1 - depot作为第一个顶点，准备缓存
                                const auto icache = T::prepare_cache1(solution, i, j);
                                // T::prepare_cache2 - 准备顶点j作为第二个顶点的缓存
                                const auto jcache = T::prepare_cache2(solution, j);
                                const auto delta = T::compute_cost(move, icache, jcache);

                                heap_insert(move, delta);

                            } else if (jupij) {

                                const auto twin_move_index = MoveGenerators::get_twin_move_generator_index(move_index);
                                auto& twin_move = T::moves.get(twin_move_index);

                                // T::prepare_cache2 - depot作为第二个顶点，准备缓存
                                const auto icache = T::prepare_cache2(solution, i, j);
                                // T::prepare_cache1 - 准备顶点j作为第一个顶点的缓存
                                const auto jcache = T::prepare_cache1(solution, j);
                                const auto twin_delta = T::compute_cost(twin_move, jcache, icache);

                                heap_insert(twin_move, twin_delta);
                            }

                        } else {

                            // T::prepare_cache12 - depot的双向缓存准备
                            const auto icache = T::prepare_cache12(solution, i, j);
                            const auto jcache = T::prepare_cache12(solution, j);
                            // T::compute_cost_pair - 同时计算两个方向的移动成本
                            const auto [delta1, delta2] = T::compute_cost_pair(move, icache, jcache);

                            heap_insert(move, delta1);

                            const auto twin_move_index = MoveGenerators::get_twin_move_generator_index(move_index);
                            auto& twin_move = T::moves.get(twin_move_index);
                            heap_insert(twin_move, delta2);
                        }
                    }

                } else if (iupij) {


                    for (const auto move_index : T::moves.get_move_generator_indices_involving_1st(i)) {

                        auto& move = T::moves.get(move_index);
                        const auto j = move.get_second_vertex();

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(j)) {
                                continue;
                            }
                        }

                        if (vtimestamp[j] != currenttime || (vtimestamp[j] == currenttime && !T::update_bits.at(j, UPDATE_BITS_SECOND))) {

                            // T::prepare_cache1 - depot作为第一个顶点的缓存
                            const auto icache = T::prepare_cache1(solution, i, j);
                            // T::prepare_cache2 - 准备顶点j作为第二个顶点的缓存
                            const auto jcache = T::prepare_cache2(solution, j);
                            const auto delta = T::compute_cost(move, icache, jcache);
                            heap_insert(move, delta);
                        }
                    }

                } else if (iupji) {

                    for (const auto move_index : T::moves.get_move_generator_indices_involving_2nd(i)) {

                        auto& move = T::moves.get(move_index);
                        const auto j = move.get_first_vertex();

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(j)) {
                                continue;
                            }
                        }

                        if (vtimestamp[j] != currenttime || (vtimestamp[j] == currenttime && !T::update_bits.at(j, UPDATE_BITS_FIRST))) {

                            // T::prepare_cache2 - depot作为第二个顶点的缓存
                            const auto icache = T::prepare_cache2(solution, i, j);
                            // T::prepare_cache1 - 准备顶点j作为第一个顶点的缓存
                            const auto jcache = T::prepare_cache1(solution, j);
                            const auto delta = T::compute_cost(move, jcache, icache);
                            heap_insert(move, delta);
                        }
                    }
                }

                vtimestamp[i] = currenttime;
            }

            // Reset the update bits associated with affected vertices.
            // 重置受影响顶点的更新位为0，表示不需要更新了
            for (auto i : affected_vertices.get_elements()) {
                T::update_bits.at(i, UPDATE_BITS_FIRST, false);
                T::update_bits.at(i, UPDATE_BITS_SECOND, false);
            }

            timegen.increment();
        }

        // Generate unique increasing numbers to tag updates.
        TimestampGenerator& timegen; // 时间戳生成器，用于标记更新

        // Set of vertices affected by a move application. This set together with `update_bits` define the move generators requiring an
        // update.
        SparseIntSet affected_vertices; // 受移动影响的顶点集合
    };


}  // namespace cobra
#endif
