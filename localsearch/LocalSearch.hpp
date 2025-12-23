#ifndef _FILO2_LOCALSEARCH_HPP_
#define _FILO2_LOCALSEARCH_HPP_
//该文件实现了车辆路径问题（VRP）的局部搜索框架，基于变量邻域下降（Variable Neighborhood Descent, VND）
//算法，提供了一套完整的局部搜索操作符管理和执行机制
#include <algorithm>
#include <cfloat>
#include <random>

#include "../instance/Instance.hpp"
#include "../movegen/MoveGenerators.hpp"
#include "../solution/Solution.hpp"
#include "AbstractOperator.hpp"
#include "EjectionChain.hpp"
#include "OneOneExchange.hpp"
#include "OneZeroExchange.hpp"
#include "RevThreeOneExchange.hpp"
#include "RevThreeThreeExchange.hpp"
#include "RevThreeTwoExchange.hpp"
#include "RevThreeZeroExchange.hpp"
#include "RevTwoOneExchange.hpp"
#include "RevTwoTwoExchange.hpp"
#include "RevTwoZeroExchange.hpp"
#include "SplitExchange.hpp"
#include "TailsExchange.hpp"
#include "ThreeOneExchange.hpp"
#include "ThreeThreeExchange.hpp"
#include "ThreeTwoExchange.hpp"
#include "ThreeZeroExchange.hpp"
#include "TwoOneExchange.hpp"
#include "TwoOptExchange.hpp"
#include "TwoTwoExchange.hpp"
#include "TwoZeroExchange.hpp"

namespace cobra {

    // Supported local search operators.
    // 支持的局部搜索操作符枚举
    enum Operator {
        E10,    // 1-0交换（将一个客户从一条路线移到另一条路线）
        E11,    // 1-1交换（两个客户在不同路线间交换）
        E20,    // 2-0交换（将两个连续客户从一条路线移到另一条路线）
        E21,    // 2-1交换（两个连续客户与一个客户在不同路线间交换）
        E22,    // 2-2交换（两个连续客户与另外两个连续客户在不同路线间交换）
        E30,    // 3-0交换（将三个连续客户从一条路线移到另一条路线）
        E31,    // 3-1交换（三个连续客户与一个客户在不同路线间交换）
        E32,    // 3-2交换（三个连续客户与两个连续客户在不同路线间交换）
        E33,    // 3-3交换（三个连续客户与另外三个连续客户在不同路线间交换）
        SPLIT,  // 分割交换（将两个客户序列分别分割成两部分并重新分配）
        TAILS,  // 尾部交换（交换两条路线的尾部）
        TWOPT,  // 2-opt交换（在一条路线内反转一段客户序列）
        EJCH,   // 弹出链（复杂的多路线客户重分配）
        RE20,   // 反向2-0交换（与E20方向相反）
        RE21,   // 反向2-1交换（与E21方向相反）
        RE22B,  // 反向2-2交换（块版本）
        RE22S,  // 反向2-2交换（单步版本）
        RE30,   // 反向3-0交换（与E30方向相反）
        RE31,   // 反向3-1交换（与E31方向相反）
        RE32B,  // 反向3-2交换（块版本）
        RE32S,  // 反向3-2交换（单步版本）
        RE33B,  // 反向3-3交换（块版本）
        RE33S,  // 反向3-3交换（单步版本）
    };

    // General VND interface.
    // 变量邻域下降（VND）通用接口
    class VariableNeighborhoodDescentInterface : private NonCopyable<VariableNeighborhoodDescentInterface> {
    public:
        // Applies the VND to the given solution.
        // 将VND应用于给定的解决方案
        virtual void apply(Solution& solution) = 0;
    };

    // VND interface implementation as RVND.
    // 随机变量邻域下降（RVND）实现
    template <bool handle_partial_solutions = false>//是否处理部分解，默认不能处理
    class RandomizedVariableNeighborhoodDescent : public VariableNeighborhoodDescentInterface {
    public:
        RandomizedVariableNeighborhoodDescent(const Instance& instance_, MoveGenerators& moves_, const std::vector<Operator>& operator_list,
                                              std::mt19937& rand_engine_, double tolerance = 0.01)
            : instance(instance_), moves(moves_), rand_engine(rand_engine_) {

            // 初始化操作符创建函数表：将操作符枚举映射到具体的操作符对象创建函数
            // 使用lambda表达式创建具体的操作符对象
            OperatorInitTable[E10] = [this, tolerance]() {
                return new CommonOperator<OneZeroExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E11] = [this, tolerance]() {
                return new CommonOperator<OneOneExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E20] = [this, tolerance]() {
                return new CommonOperator<TwoZeroExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E21] = [this, tolerance]() {
                return new CommonOperator<TwoOneExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E22] = [this, tolerance]() {
                return new CommonOperator<TwoTwoExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E30] = [this, tolerance]() {
                return new CommonOperator<ThreeZeroExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E31] = [this, tolerance]() {
                return new CommonOperator<ThreeOneExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E32] = [this, tolerance]() {
                return new CommonOperator<ThreeTwoExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[E33] = [this, tolerance]() {
                return new CommonOperator<ThreeThreeExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[SPLIT] = [this, tolerance]() {
                return new CommonOperator<SplitExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[TAILS] = [this, tolerance]() {
                return new CommonOperator<TailsExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[TWOPT] = [this, tolerance]() {
                return new CommonOperator<TwoOptExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[EJCH] = [this, tolerance]() {
                if constexpr (handle_partial_solutions) {
                    std::cout << "EjectionChain is not supported for partial solutions\n";
                    exit(1);
                }
                return new CommonOperator<EjectionChain<25>, false>(instance, moves, tolerance);
            };
            OperatorInitTable[RE20] = [this, tolerance]() {
                return new CommonOperator<RevTwoZeroExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE21] = [this, tolerance]() {
                return new CommonOperator<RevTwoOneExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE22B] = [this, tolerance]() {
                return new CommonOperator<RevTwoTwoExchange<true>, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE22S] = [this, tolerance]() {
                return new CommonOperator<RevTwoTwoExchange<false>, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE30] = [this, tolerance]() {
                return new CommonOperator<RevThreeZeroExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE31] = [this, tolerance]() {
                return new CommonOperator<RevThreeOneExchange, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE32B] = [this, tolerance]() {
                return new CommonOperator<RevThreeTwoExchange<true>, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE32S] = [this, tolerance]() {
                return new CommonOperator<RevThreeTwoExchange<false>, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE33B] = [this, tolerance]() {
                return new CommonOperator<RevThreeThreeExchange<true>, handle_partial_solutions>(instance, moves, tolerance);
            };
            OperatorInitTable[RE33S] = [this, tolerance]() {
                return new CommonOperator<RevThreeThreeExchange<false>, handle_partial_solutions>(instance, moves, tolerance);
            };

            // 根据传入的操作符列表创建对应的操作符对象
            for (auto op : operator_list) {
                auto ptr = OperatorInitTable[op]();
                operators.push_back(ptr);
            }
        }

        virtual ~RandomizedVariableNeighborhoodDescent() {
            // 析构函数：释放所有操作符对象
            for (auto op : operators) {
                delete op;
            }
        }

        void apply(Solution& solution) override {
            // RVND核心：随机打乱操作符顺序，然后依次应用
            std::shuffle(operators.begin(), operators.end(), rand_engine);

            // Instead of re-applying all the operators if some improvement is found, we just perform a single. This does not harm quality
            // too much and saves some computing time.
            // 如果找到改进，不重新应用所有操作符，只执行单次循环（不同于FILO）。这对质量影响不大，节省计算时间。
            for (auto& op : operators) {
                op->apply_rough_best_improvement(solution);
            }

            assert(solution.is_feasible()); // 确保解决方案仍可行
        }

    private:
        const Instance& instance; // VRP实例引用
        MoveGenerators& moves;    // 移动生成器引用
        std::mt19937& rand_engine; // 随机数引擎
        std::unordered_map<Operator, std::function<AbstractOperator*()>> OperatorInitTable; // 操作符初始化表
        std::vector<AbstractOperator*> operators; // 操作符对象列表
    };

    // Links together VNDs.
    // VND组合器：将多个VND链接在一起，支持分层搜索
    class VariableNeighborhoodDescentComposer {

    public:
        VariableNeighborhoodDescentComposer(double tolerance_) : tolerance(tolerance_){};

        // 添加VND到组合器中
        void append(VariableNeighborhoodDescentInterface* vnd) {
            tiers.push_back(vnd);
        }

        // 顺序应用所有VND，如果某个VND改进了解决方案，则重新从第一个VND开始
        void sequential_apply(Solution& solution) {

        __again__: // 标签：用于跳转重新开始
            for (auto n = 0u; n < tiers.size(); n++) {
                const auto curr_cost = solution.get_cost(); // 记录当前成本
                tiers[n]->apply(solution); // 应用第n个VND，其中的算子都只使用一次
                if (n > 0 && solution.get_cost() + tolerance < curr_cost) {
                    // 如果在第n>0个VND后找到了改进，重新从头开始
                    goto __again__;
                }
            }
        }//在搜索深度和计算效率之间取得平衡。基础算子已经是最常用的，如果在基础算子中找到改进，通常可以继续前进；
        //而复杂算子的改进可能意味着解的结构发生了较大变化，值得重新从基础算子开始评估。

    private:
        const double tolerance; // 改进容忍度
        std::vector<VariableNeighborhoodDescentInterface*> tiers; // VND层级列表
    };


}  // namespace cobra


#endif