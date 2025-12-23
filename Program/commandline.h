/*MIT License

Copyright(c) 2020 Thibaut Vidal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <iostream>
#include <string>
#include <climits>
#include "AlgorithmParameters.h"

class CommandLine
{
public:
	AlgorithmParameters ap = default_algorithm_parameters();

	int nbVeh		 = INT_MAX;		// Number of vehicles. Default value: infinity
	// 中文注释：车辆数。默认值为 INT_MAX，表示未指定车队规模（可视作“无穷大/未定”），后续 Params 构造会处理该情况并给出默认上界。
	std::string pathInstance;		// Instance path
	// 中文注释：输入实例文件路径（.vrp 文件）
	std::string pathSolution;		// Solution path
	// 中文注释：输出解文件路径
	bool verbose     = true;
	// 中文注释：是否输出详细日志（命令行 -log 控制）
	bool isRoundingInteger = true;
	// 中文注释：是否将距离四舍五入为整数（命令行 -round 控制）

	// Reads the line of command and extracts possible options
	CommandLine(int argc, char* argv[])
	{
		if (argc % 2 != 1 || argc > 35 || argc < 3)
		{
			std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc << std::endl;
			display_help(); throw std::string("Incorrect line of command");
		}
		else
		{
			pathInstance = std::string(argv[1]);
			pathSolution = std::string(argv[2]);
			for (int i = 3; i < argc; i += 2)
			{
				if (std::string(argv[i]) == "-t")
					ap.timeLimit = atof(argv[i+1]);
					// 中文注释：设置时间上限（秒），覆盖默认
				else if (std::string(argv[i]) == "-it")
					ap.nbIter  = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-seed")
					ap.seed    = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-veh")
					nbVeh = atoi(argv[i+1]);
					// 中文注释：强制指定车辆数，上层 Params 将使用该值而不是自动估算
				else if (std::string(argv[i]) == "-round")
					isRoundingInteger = atoi(argv[i+1]);
					// 中文注释：控制距离是否四舍五入为整数（0/1）
				else if (std::string(argv[i]) == "-log")
					verbose = atoi(argv[i+1]);
					// 中文注释：设置日志输出开关（0/1）
				else if (std::string(argv[i]) == "-nbGranular")
					ap.nbGranular = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-mu")
					ap.mu = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-lambda")
					ap.lambda = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbElite")
					ap.nbElite = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbClose")
					ap.nbClose = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterPenaltyManagement")
					ap.nbIterPenaltyManagement = atoi(argv[i+1]);
				else if (std::string(argv[i]) == "-nbIterTraces")
					ap.nbIterTraces = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-targetFeasible")
					ap.targetFeasible = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyIncrease")
					ap.penaltyIncrease = atof(argv[i+1]);
				else if (std::string(argv[i]) == "-penaltyDecrease")
					ap.penaltyDecrease = atof(argv[i+1]);
				else
				{
					std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i]) << std::endl;
					// 中文注释：未识别参数时打印帮助并抛出异常
					display_help(); throw std::string("Incorrect line of command");
				}
			}
		}
	}

	// Printing information about how to use the code
	void display_help()
	{
		// 中文注释：打印程序使用帮助与参数说明（英文原文保持不变，下面也可阅读中文注释）
		std::cout << std::endl;
		std::cout << "-------------------------------------------------- HGS-CVRP algorithm (2020) ---------------------------------------------------" << std::endl;
		std::cout << "Call with: ./hgs instancePath solPath [-it nbIter] [-t myCPUtime] [-seed mySeed] [-veh nbVehicles] [-log verbose]               " << std::endl;
		// 中文注释：调用格式示例
		std::cout << "[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                         " << std::endl;
		std::cout << "    中文：设置最多允许的无改进迭代次数，默认 20000" << std::endl;
		std::cout << "[-t <double>] sets a time limit in seconds. If this parameter is set the code will be run iteratively until the time limit      " << std::endl;
		std::cout << "    中文：设置运行时间上限（秒），若设置则按时间限制迭代运行" << std::endl;
		std::cout << "[-seed <int>] sets a fixed seed. Defaults to 0                                                                                  " << std::endl;
		std::cout << "    中文：设置随机数种子，默认 0" << std::endl;
		std::cout << "[-veh <int>] sets a prescribed fleet size. Otherwise a reasonable UB on the the fleet size is calculated                        " << std::endl;
		std::cout << "    中文：指定车队规模；若不指定，程序会根据实例估算一个合理的上界" << std::endl;
		std::cout << "[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. " << std::endl;
		std::cout << "    中文：是否将距离四舍五入为整数（0/1），默认 1" << std::endl;
		std::cout << "[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                     " << std::endl;
		std::cout << "    中文：设置日志输出（0/1），默认 1" << std::endl;
		std::cout << std::endl;
		std::cout << "Additional Arguments:                                                                                                           " << std::endl;
		std::cout << "    中文：下列为可选的高级参数及其默认值" << std::endl;
		std::cout << "[-nbIterTraces <int>] Number of iterations between traces display during HGS execution. Defaults to 500                         " << std::endl;
		std::cout << "    中文：HGS 运行时 trace 显示的间隔迭代数，默认 500" << std::endl;
		std::cout << "[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20                " << std::endl;
		std::cout << "    中文：粒度搜索参数，限制 RI 局部搜索中考虑的邻域数量，默认 20" << std::endl;
		std::cout << "[-mu <int>] Minimum population size. Defaults to 25                                                                             " << std::endl;
		std::cout << "    中文：初始种群最小规模，默认 25" << std::endl;
		std::cout << "[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40 " << std::endl;
		std::cout << "    中文：每代生成的解数量（用于达到最大种群规模的参数），默认 40" << std::endl;
		std::cout << "[-nbElite <int>] Number of elite individuals. Defaults to 5                                                                     " << std::endl;
		std::cout << "    中文：精英个体数量，默认 5" << std::endl;
		std::cout << "[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 4      " << std::endl;
		std::cout << "    中文：用于计算多样性贡献的最近解数量，默认 4" << std::endl;
		std::cout << "[-nbIterPenaltyManagement <int>] Number of iterations between penalty updates. Defaults to 100                                  " << std::endl;
		std::cout << "    中文：惩罚系数调整的迭代间隔，默认 100" << std::endl;
		std::cout << "[-targetFeasible <double>] target ratio of feasible individuals between penalty updates. Defaults to 0.2                        " << std::endl;
		std::cout << "    中文：期望在惩罚更新周期内可行个体比例目标，默认 0.2" << std::endl;
		std::cout << "[-penaltyIncrease <double>] penalty increase if insufficient feasible individuals between penalty updates. Defaults to 1.2      " << std::endl;
		std::cout << "    中文：在可行个体过少时惩罚系数放大因子，默认 1.2" << std::endl;
		std::cout << "[-penaltyDecrease <double>] penalty decrease if sufficient feasible individuals between penalty updates. Defaults to 0.85       " << std::endl;
		std::cout << "    中文：在可行个体充足时惩罚系数缩小因子，默认 0.85" << std::endl;
		std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
		std::cout << std::endl;
	};
};
#endif
