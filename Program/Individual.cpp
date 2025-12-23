#include "Individual.h" 

void Individual::evaluateCompleteCost(const Params & params)
{
	/*
	中文注释（evaluateCompleteCost）:
	- 计算个体的完整评估指标（distance、nbRoutes、capacityExcess、durationExcess），并据此计算惩罚化代价 penalizedCost 及可行性标志 isFeasible。
	- 输入假设：chromR 已包含按路线划分的客户序列，params 提供距离矩阵、容量与时长限制及惩罚系数。
	- 过程说明：
	  * 对每条车辆路线，累加从 depot -> 第一个客户，再到下一客户的边距，最后回到 depot 的距离。
	  * 累加该路线的负载与总服务时间（用于时长超限判定）。
	  * 更新 predecessors 与 successors 以便其它模块（例如 brokenPairsDistance）能直接读取邻接信息。
	  * 根据 vehicleCapacity 与 durationLimit 计算超量并累积到 eval.capacityExcess 与 eval.durationExcess。
	  * 最后合成 penalizedCost = distance + penaltyCapacity*capacityExcess + penaltyDuration*durationExcess。
	*/
	eval = EvalIndiv();
	for (int r = 0; r < params.nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			double distance = params.timeCost[0][chromR[r][0]];
			double load = params.cli[chromR[r][0]].demand;
			double service = params.cli[chromR[r][0]].serviceDuration;
			predecessors[chromR[r][0]] = 0;
			for (int i = 1; i < (int)chromR[r].size(); i++)
			{
				distance += params.timeCost[chromR[r][i-1]][chromR[r][i]];
				load += params.cli[chromR[r][i]].demand;
				service += params.cli[chromR[r][i]].serviceDuration;
				predecessors[chromR[r][i]] = chromR[r][i-1];
				successors[chromR[r][i-1]] = chromR[r][i];
			}
			successors[chromR[r][chromR[r].size()-1]] = 0;
			distance += params.timeCost[chromR[r][chromR[r].size()-1]][0];
			eval.distance += distance;
			eval.nbRoutes++;
			if (load > params.vehicleCapacity) eval.capacityExcess += load - params.vehicleCapacity;
			if (distance + service > params.durationLimit) eval.durationExcess += distance + service - params.durationLimit;
		}
	}

	eval.penalizedCost = eval.distance + eval.capacityExcess*params.penaltyCapacity + eval.durationExcess*params.penaltyDuration;
	eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);
}

Individual::Individual(Params & params)
{
	/*
	中文注释（默认构造 Individual）:
	- 创建一个随机的 giant-tour 表示（chromT），并为 chromR 分配 params.nbVehicles 个空路线。
	- successors/predecessors 向量根据 params.nbClients 大小分配，用于后续 evaluateCompleteCost 与 brokenPairsDistance。
	- 使用 params.ran 对 chromT 进行随机打乱以初始化个体的随机排列。
	*/
	successors = std::vector <int>(params.nbClients + 1);
	predecessors = std::vector <int>(params.nbClients + 1);
	chromR = std::vector < std::vector <int> >(params.nbVehicles);
	chromT = std::vector <int>(params.nbClients);
	for (int i = 0; i < params.nbClients; i++) chromT[i] = i + 1;
	std::shuffle(chromT.begin(), chromT.end(), params.ran);
	eval.penalizedCost = 1.e30;	
}

Individual::Individual(Params & params, std::string fileName) : Individual(params)
{
	/*
	中文注释（从文件构造 Individual）:
	- 从给定的解文件读取按行格式的 Route 信息（CVRPLib 风格），构造 chromR 和 chromT。
	- 读取完成后执行若干安全检查：节点数量、可行性校验、文件中声明的成本与计算成本一致性。
	- 若任何检查失败则抛出异常；若成功并且 params.verbose，则打印成功信息。
	*/
	double readCost;
	chromT.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops in the input file as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route"; r++)
		{
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			while (ss >> inputCustomer) // Loops as long as there is an integer to read in this route
			{
				chromT.push_back(inputCustomer);
				chromR[r].push_back(inputCustomer);
			}
			inputFile >> inputString;
		}
		if (inputString == "Cost") inputFile >> readCost;
		else throw std::string("Unexpected token in input solution");

		// Some safety checks and printouts
		evaluateCompleteCost(params);
		if ((int)chromT.size() != params.nbClients) throw std::string("Input solution does not contain the correct number of clients");
		if (!eval.isFeasible) throw std::string("Input solution is infeasible");
		if (eval.penalizedCost != readCost)throw std::string("Input solution has a different cost than announced in the file");
		if (params.verbose) std::cout << "----- INPUT SOLUTION HAS BEEN SUCCESSFULLY READ WITH COST " << eval.penalizedCost << std::endl;
	}
	else 
		throw std::string("Impossible to open solution file provided in input in : " + fileName);
}
