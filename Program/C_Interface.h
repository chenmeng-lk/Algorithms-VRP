//
// Created by chkwon on 3/23/22.
//

#ifndef C_INTERFACE_H
#define C_INTERFACE_H
#include "AlgorithmParameters.h"

struct SolutionRoute
{
	int length;
	int * path;
};

// 中文注释（SolutionRoute）:
// - 表示一条路线的结果结构：
//   * length: 该路线上客户数量
//   * path: 指向包含客户编号的整数数组（由调用方/返回方负责内存管理，C_Interface.cpp 中使用 new 分配，delete_solution 负责释放）


struct Solution
{
	double cost;
	double time;
	int n_routes;
	struct SolutionRoute * routes;
};

// 中文注释（Solution）:
// - 汇总解的信息结构：
//   * cost: 最终的惩罚化代价（包含距离与基于 penalty 的超量成本），双精度
//   * time: 从启动到求解完成所消耗的时间（秒）
//   * n_routes: 路线数量
//   * routes: 指向长度为 n_routes 的 SolutionRoute 数组（动态分配），每个元素是一个具体路线


#ifdef __cplusplus
extern "C"
#endif
struct Solution * solve_cvrp(
	int n, double* x, double* y, double* serv_time, double* dem,
	double vehicleCapacity, double durationLimit, char isRoundingInteger, char isDurationConstraint,
	int max_nbVeh, const struct AlgorithmParameters* ap, char verbose);

// 中文注释（solve_cvrp）:
// - C 接口函数：基于二维坐标与服务时长等输入求解 CVRP。
// - 参数说明（主要）：
//   * n: 节点数（包括 depot）。
//   * x,y: 指向长度为 n 的坐标数组（可为 nullptr 但通常需提供，用于 swap*）。
//   * serv_time: 指向长度为 n 的服务时间数组。
//   * dem: 指向长度为 n 的需求数组。
//   * vehicleCapacity, durationLimit: 容量与时长限制。
//   * isRoundingInteger: 若为真则距离四舍五入为整数。
//   * isDurationConstraint: 是否启用时长约束。
//   * max_nbVeh: 用户指定的车辆上限（可为 INT_MAX 表示未指定）。
//   * ap: 指向 AlgorithmParameters 的指针（不可为 null），函数内部会拷贝用于配置算法。
//   * verbose: 是否打印详细日志。
// - 返回值：指向动态分配的 Solution 结构（调用者须在不需要时调用 delete_solution 释放内存）。

#ifdef __cplusplus
extern "C"
#endif
struct Solution *solve_cvrp_dist_mtx(
	int n, double* x, double* y, double *dist_mtx, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isDurationConstraint,
	int max_nbVeh, const struct AlgorithmParameters *ap, char verbose);

// 中文注释（solve_cvrp_dist_mtx）:
// - C 接口函数：当调用方直接提供距离矩阵（dist_mtx，按行主序排列 n*n 元素）时使用。
// - 与 solve_cvrp 的区别：避免在接口中重复计算欧几里得距离，而是直接使用提供的距离矩阵。
// - 返回值与内存管理规则同上。

#ifdef __cplusplus
extern "C"
#endif
void delete_solution(struct Solution * sol);

// 中文注释（delete_solution）:
// - 释放由 solve_cvrp 或 solve_cvrp_dist_mtx 返回的 Solution 结构及其内部动态分配的数组。
// - 使用后务必调用以避免内存泄漏。


#endif //C_INTERFACE_H
