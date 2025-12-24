//
// Created by chkwon on 3/23/22.
//

// This header file must be readable in C.
//算法的默认参数
#ifndef ALGORITHMPARAMETERS_H
#define ALGORITHMPARAMETERS_H

struct AlgorithmParameters {
	int nbGranular;			// Granular search parameter, limits the number of moves in the RI local search 粒度搜索参数，限制 RI 局部搜索中考虑的邻域数量
	int mu;					// Minimum population size 种群的最小大小
	int lambda;				// Number of solutions created before reaching the maximum population size (i.e., generation size) 种群达到最大大小前生成的解数量
	int nbElite;			// Number of elite individuals 精英个体数
	int nbClose;			// Number of closest solutions/individuals considered when calculating diversity contribution 计算多样性贡献时考虑的最近解数量

	int nbIterPenaltyManagement;  // Number of iterations between penalty updates 惩罚参数更新间隔代数
	double targetFeasible;	      // Reference proportion for the number of feasible individuals, used for the adaptation of the penalty parameters 可行个体的期望占比
	double penaltyDecrease;	      // Multiplier used to decrease penalty parameters if there are sufficient feasible individuals
	double penaltyIncrease;	      // Multiplier used to increase penalty parameters if there are insufficient feasible individuals

	int seed;				// Random seed. Default value: 0
	int nbIter;				// Nb iterations without improvement until termination (or restart if a time limit is specified). Default value: 20,000 iterations 没有提升的迭代次数
	int nbIterTraces;       // Number of iterations between traces display during HGS execution 打印的间隔代数
	double timeLimit;		// CPU time limit until termination in seconds. Default value: 0 (i.e., inactive)
	int useSwapStar;		// Use SWAP* local search or not. Default value: 1. Only available when coordinates are provided.
};

#ifdef __cplusplus
extern "C"
#endif
struct AlgorithmParameters default_algorithm_parameters();

#ifdef __cplusplus
void print_algorithm_parameters(const AlgorithmParameters & ap);
#endif

#endif //ALGORITHMPARAMETERS_H
