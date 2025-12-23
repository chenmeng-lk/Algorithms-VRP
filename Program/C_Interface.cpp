//
// Created by chkwon on 3/23/22.
//

#include "C_Interface.h"
#include "Population.h"
#include "Params.h"
#include "Genetic.h"
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

Solution *prepare_solution(Population &population, Params &params)
{
	// Preparing the best solution
	Solution *sol = new Solution;
	sol->time = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;

	if (population.getBestFound() != nullptr) {
		// Best individual
		auto best = population.getBestFound();

		// setting the cost
		sol->cost = best->eval.penalizedCost;

		// finding out the number of routes in the best individual
		int n_routes = 0;
		for (int k = 0; k < params.nbVehicles; k++)
			if (!best->chromR[k].empty()) ++n_routes;

		// filling out the route information
		sol->n_routes = n_routes;
		sol->routes = new SolutionRoute[n_routes];
		for (int k = 0; k < n_routes; k++) {
			sol->routes[k].length = (int)best->chromR[k].size();
			sol->routes[k].path = new int[sol->routes[k].length];
			std::copy(best->chromR[k].begin(), best->chromR[k].end(), sol->routes[k].path);
		}
	}
	else {
		sol->cost = 0.0;
		sol->n_routes = 0;
		sol->routes = nullptr;
	}
	return sol;
}

// 中文注释（prepare_solution）:
// - 将内部的 Population 中找到的最佳个体转换为 C API 返回的 Solution 结构，分配堆内存用于
//   Solution 及其内部的 routes/path 数组。
// - 注意：本函数只负责在堆上分配内存（使用 new/new[]），调用方（C 层使用者）需在不再需要
//   该结果时调用 `delete_solution` 进行相应释放，以避免内存泄漏。
// - 计算的 time 是从 Params 的 startTime 到当前时刻的秒数差。


extern "C" Solution *solve_cvrp(
	int n, double *x, double *y, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isRoundingInteger, char isDurationConstraint,
	int max_nbVeh, const AlgorithmParameters *ap, char verbose)
{
	Solution *result;

	try {
		std::vector<double> x_coords(x, x + n);
		std::vector<double> y_coords(y, y + n);
		std::vector<double> service_time(serv_time, serv_time + n);
		std::vector<double> demands(dem, dem + n);

		std::vector<std::vector<double> > distance_matrix(n, std::vector<double>(n));
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				distance_matrix[i][j] = std::sqrt(
					(x_coords[i] - x_coords[j])*(x_coords[i] - x_coords[j])
					+ (y_coords[i] - y_coords[j])*(y_coords[i] - y_coords[j])
				);
				if (isRoundingInteger)
					distance_matrix[i][j] = std::round(distance_matrix[i][j]);
			}
		}

		Params params(x_coords,y_coords,distance_matrix,service_time,demands,vehicleCapacity,durationLimit,max_nbVeh,isDurationConstraint,verbose,*ap);

		// 中文注释（solve_cvrp 内部流程）:
		// 1. 根据传入的坐标与参数构造 Params（包含距离矩阵拷贝），注意 Params 的构造会做必要的
		//    检查与默认值设置（例如 nbVehicles 的默认上界处理）。
		// 2. 使用 Params 构造 Genetic 求解器并执行 `run()` 启动求解过程。
		// 3. 使用 prepare_solution 将求解得到的最佳个体转换为 C 层的 Solution，并返回。
		// 异常情况下会捕获并打印异常信息，但仍然返回 result（若异常在构造之前发生，result 未定义）。
		Genetic solver(params);
		solver.run();
		result = prepare_solution(solver.population, params);
	}
	catch (const std::string &e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception &e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

	return result;
}

extern "C" Solution *solve_cvrp_dist_mtx(
	int n, double *x, double *y, double *dist_mtx, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isDurationConstraint,
	int max_nbVeh, const AlgorithmParameters *ap, char verbose)
{
	Solution *result;
	std::vector<double> x_coords;
	std::vector<double> y_coords;

	try {
		if (x != nullptr && y != nullptr) {
			x_coords = {x, x + n};
			y_coords = {y, y + n};
		}

		std::vector<double> service_time(serv_time, serv_time + n);
		std::vector<double> demands(dem, dem + n);

		std::vector<std::vector<double> > distance_matrix(n, std::vector<double>(n));
		for (int i = 0; i < n; i++) { // row
			for (int j = 0; j < n; j++) { // column
				distance_matrix[i][j] = dist_mtx[n * i + j];
			}
		}

		Params params(x_coords,y_coords,distance_matrix,service_time,demands,vehicleCapacity,durationLimit,max_nbVeh,isDurationConstraint,verbose,*ap);
		
		// Running HGS and returning the result
		Genetic solver(params);
		solver.run();
		result = prepare_solution(solver.population, params);
	}
	catch (const std::string &e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception &e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

	return result;
}

extern "C" void delete_solution(Solution *sol)
{
	for (int i = 0; i < sol->n_routes; ++i)
		delete[] sol->routes[i].path;

	delete[] sol->routes;
	delete sol;
}

// 中文注释（delete_solution）:
// - 释放 Solution 及其内部各路线上分配的 `path` 数组。
// - 假定传入的 Solution 是通过 prepare_solution 或者 solve_cvrp 系列函数分配的。
// - 在使用该 API 的宿主程序中，应对返回的 Solution 进行异常安全管理（例如在异常路径中
//   也要确保调用 delete_solution 以释放已分配的部分资源）。