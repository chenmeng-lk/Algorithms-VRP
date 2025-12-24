#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
#include "InstanceCVRPLIB.h"
using namespace std;

int main(int argc, char *argv[])
{
		/*
		（程序入口总体说明）:
		- 该文件包含程序主入口，执行以下主要步骤：
			1. 使用 `CommandLine` 解析命令行参数和算法参数（包括 seed、车辆数、输出路径等）。
			2. 根据命令行和实例文件构造 `InstanceCVRPLIB`，读取 VRP 实例（坐标、需求、服务时间、容量、时间窗/时长限制等）。
			3. 使用实例数据构造 `Params`（包含距离矩阵、客户信息、随机数生成器、惩罚系数初始值等）。
			4. 构造 `Genetic` 求解器并调用 `run()` 启动遗传算法 + 局部搜索流程（HGS 主循环）。
			5. 若找到最优解，导出为 CVRPLib 格式并保存搜索进度（CSV）。

		说明：主函数只负责任务级的组织与异常捕获，具体算法细节在 `Genetic`, `Population`, `LocalSearch`, `Split` 等模块中实现。
		*/
	try
	{
		// Reading the arguments of the program
		CommandLine commandline(argc, argv);

		// Print all algorithm parameter values
		if (commandline.verbose) print_algorithm_parameters(commandline.ap);

		// Reading the data file and initializing some data structures
		// 根据命令行提供的实例路径构造 InstanceCVRPLIB，
		// 该构造函数负责解析 .vrp 文件并填充坐标、需求、服务时间、距离矩阵等信息。
		if (commandline.verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		InstanceCVRPLIB cvrp(commandline.pathInstance, commandline.isRoundingInteger);

		Params params(cvrp.x_coords,cvrp.y_coords,cvrp.dist_mtx,cvrp.service_time,cvrp.demands,
			          cvrp.vehicleCapacity,cvrp.durationLimit,commandline.nbVeh,cvrp.isDurationConstraint,commandline.verbose,commandline.ap);

		// Running HGS
		Genetic solver(params);
		solver.run();
		
		// Exporting the best solution
		if (solver.population.getBestFound() != NULL)
		{
			if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
			solver.population.exportCVRPLibFormat(*solver.population.getBestFound(),commandline.pathSolution);
			solver.population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance);
		}
	}
	catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }
	return 0;
}
