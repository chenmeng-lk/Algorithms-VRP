// 参数管理类 - 处理命令行参数和默认配置
#ifndef _FILO2_PARAMETERS_HPP_
#define _FILO2_PARAMETERS_HPP_

#include <filesystem>
#include <iostream>
#include <string>

// Default parameters.
// 默认参数定义
#define DEFAULT_OUTPATH ("./")                              // 输出路径
#define DEFAULT_SOLUTION_CACHE_HISTORY (50)                 // 解缓存历史大小
#define DEFAULT_CW_LAMBDA (1.0)                             // Clarke-Wright算法的lambda参数
#define DEFAULT_NEIGHBORS_NUM (1500)                        // 预计算的邻居数量
#define DEFAULT_CW_NEIGHBORS (100)                          // Clarke-Wright算法中考虑的邻居数
#define DEFAULT_ROUTEMIN_ITERATIONS (1000)                  // 路径最小化的迭代次数
#define DEFAULT_COREOPT_ITERATIONS (100000)                 // 核心优化的迭代次数
#define DEFAULT_SPARSIFICATION_RULE1_NEIGHBORS (25)         // 粒度规则中的邻居数（k值）
#define DEFAULT_SPARSIFICATION_FACTOR (0.25)                // 稀疏化因子（gamma基础值）
#define DEFAULT_SPARSIFICATION_MULTIPLIER (0.50)            // 稀疏化乘数（delta值）
#define DEFAULT_SHAKING_LB_FACTOR (0.375)                   // 扰动强度下界因子
#define DEFAULT_SHAKING_UB_FACTOR (0.85)                    // 扰动强度上界因子
#define DEFAULT_TOLERANCE (0.01)                            // 容差值
#define DEFAULT_SEED (0)                                    // 随机种子
#define DEFAULT_SA_INIT_FACTOR (0.1)                        // 模拟退火初始温度因子
#define DEFAULT_SA_FINAL_FACTOR (0.01)                      // 模拟退火最终温度因子

// Tokens.
// 命令行参数标记
#define TOKEN_OUTPATH ("--outpath")
#define TOKEN_TOLERANCE ("--tolerance")
#define TOKEN_NEIGHBORS_NUM ("--neighbors-num")
#define TOKEN_SPARSIFICATION_RULE1_NEIGHBORS ("--granular-neighbors")
#define TOKEN_SOLUTION_CACHE_HISTORY ("--cache")
#define TOKEN_ROUTEMIN_ITERATIONS ("--routemin-iterations")
#define TOKEN_COREOPT_ITERATIONS ("--coreopt-iterations")
#define TOKEN_SPARSIFICATION_FACTOR ("--granular-gamma-base")
#define TOKEN_SPARSIFICATION_MULTIPLIER ("--granular-delta")
#define TOKEN_SHAKING_LB_FACTOR ("--shaking-lower-bound")
#define TOKEN_SHAKING_UB_FACTOR ("--shaking-upper-bound")
#define TOKEN_SEED ("--seed")
#define TOKEN_HELP ("--help")
#define TOKEN_SA_INIT_FACTOR ("--sa-initial-factor")
#define TOKEN_SA_FINAL_FACTOR ("--sa-final-factor")


// 参数类：管理所有算法参数
class Parameters {

public:
    // 构造函数：解析命令行参数
    explicit Parameters(int argc, char* argv[]) {

        if (argc == 1) {
            std::cout << "Missing input instance.\n\n";
            exit(EXIT_FAILURE);
        }

        // 第一个参数是实例文件路径
        instance_path = std::string(argv[1]);

        // 解析其余的键值对参数
        for (auto n = 2; n < argc; n += 2) {

            auto token = std::string(argv[n]);

            if (n + 1 >= argc) {
                std::cout << "Missing value for '" << token << "'.\n\n";
                exit(EXIT_FAILURE);
            }
            auto value = std::string(argv[n + 1]);

            set(token, value);
        }
    }

    // Getter方法：获取各种参数值
    inline int get_solution_cache_size() const {
        return solution_cache_history;
    }
    inline double get_cw_lambda() const {
        return cw_lambda;
    }
    inline int get_cw_neighbors() const {
        return cw_neighbors;
    }
    inline int get_routemin_iterations() const {
        return routemin_iterations;
    }
    inline int get_coreopt_iterations() const {
        return coreopt_iterations;
    }
    inline int get_sparsification_rule_neighbors() const {//获取粒度规则中的邻居数
        return sparsification_rule_neighbors;
    }
    inline double get_gamma_base() const {
        return gamma_base;
    }
    inline double get_delta() const {
        return delta;
    }
    inline double get_shaking_lb_factor() const {
        return shaking_lb_factor;
    }
    inline double get_shaking_ub_factor() const {
        return shaking_ub_factor;
    }
    inline double get_tolerance() const {
        return tolerance;
    }
    inline std::string get_instance_path() const {
        return instance_path;
    }
    inline std::string get_outpath() const {
        return outpath;
    }
    inline int get_seed() const {
        return seed;
    }
    inline auto get_sa_initial_factor() const {
        return sa_initial_factor;
    }
    inline auto get_sa_final_factor() const {
        return sa_final_factor;
    }

    inline int get_neighbors_num() const {
        return neighbors_num;
    }

    // 设置参数值
    void set(const std::string& key, const std::string& value) {

        if (key == TOKEN_OUTPATH) {
            outpath = value;
            if (outpath.back() != std::filesystem::path::preferred_separator) {
                outpath += std::filesystem::path::preferred_separator;
            }
        } else if (key == TOKEN_TOLERANCE) {
            tolerance = std::stof(value);
        } else if (key == TOKEN_SPARSIFICATION_RULE1_NEIGHBORS) {
            sparsification_rule_neighbors = std::stoi(value);
        } else if (key == TOKEN_SOLUTION_CACHE_HISTORY) {
            solution_cache_history = std::stoi(value);
        } else if (key == TOKEN_ROUTEMIN_ITERATIONS) {
            routemin_iterations = std::stoi(value);
        } else if (key == TOKEN_COREOPT_ITERATIONS) {
            coreopt_iterations = std::stoi(value);
        } else if (key == TOKEN_SPARSIFICATION_FACTOR) {
            gamma_base = std::stof(value);
        } else if (key == TOKEN_SPARSIFICATION_MULTIPLIER) {
            delta = std::stof(value);
        } else if (key == TOKEN_SHAKING_LB_FACTOR) {
            shaking_lb_factor = std::stof(value);
        } else if (key == TOKEN_SHAKING_UB_FACTOR) {
            shaking_ub_factor = std::stof(value);
        } else if (key == TOKEN_SEED) {
            seed = std::stoi(value);
        } else if (key == TOKEN_SA_INIT_FACTOR) {
            sa_initial_factor = std::stof(value);
        } else if (key == TOKEN_SA_FINAL_FACTOR) {
            sa_final_factor = std::stof(value);
        } else if (key == TOKEN_NEIGHBORS_NUM) {
            neighbors_num = std::stoi(value);
        } else {
            std::cout << "Error: unknown argument '" << key << "'. Try --help for more information.\n";
            exit(EXIT_SUCCESS);
        }
    }

private:
    // 私有成员变量：存储所有参数值
    std::string instance_path;                                      // 实例文件路径
    std::string outpath = DEFAULT_OUTPATH;                          // 输出路径
    double tolerance = DEFAULT_TOLERANCE;                           // 容差值
    int solution_cache_history = DEFAULT_SOLUTION_CACHE_HISTORY;    // 解缓存大小
    double cw_lambda = DEFAULT_CW_LAMBDA;                           // CW算法lambda参数
    int cw_neighbors = DEFAULT_CW_NEIGHBORS;                        // CW算法邻居数
    int routemin_iterations = DEFAULT_ROUTEMIN_ITERATIONS;          // 路径最小化迭代次数
    int coreopt_iterations = DEFAULT_COREOPT_ITERATIONS;            // 核心优化迭代次数
    int sparsification_rule_neighbors = DEFAULT_SPARSIFICATION_RULE1_NEIGHBORS;  // 粒度规则邻居数
    double gamma_base = DEFAULT_SPARSIFICATION_FACTOR;              // gamma基础值
    double delta = DEFAULT_SPARSIFICATION_MULTIPLIER;               // delta值
    double shaking_lb_factor = DEFAULT_SHAKING_LB_FACTOR;           // 扰动下界因子
    double shaking_ub_factor = DEFAULT_SHAKING_UB_FACTOR;           // 扰动上界因子
    int seed = DEFAULT_SEED;                                        // 随机种子
    double sa_initial_factor = DEFAULT_SA_INIT_FACTOR;              // SA初始温度因子
    double sa_final_factor = DEFAULT_SA_FINAL_FACTOR;               // SA最终温度因子
    int neighbors_num = DEFAULT_NEIGHBORS_NUM;                      // 预计算邻居数
};


#endif