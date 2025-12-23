#include "Split.h" 

void Split::generalSplit(Individual & indiv, int nbMaxVehicles)
{
	/*
	中文注释（Split::generalSplit 总体说明）:
	- Split 模块负责将 giant-tour（indiv.chromT）解码为具体的车辆路线集合 indiv.chromR。
	- generalSplit 会先用一些预处理把每个位置的需求、服务时间、到下一个节点的距离等信息填入 cliSplit 与前缀和数组，
	  然后尝试两种 Split 算法：首先是简单的 Bellman 风格实现（splitSimple），若不成功则使用受限车队的 Split（splitLF）。
	- nbMaxVehicles 提供了 Split 可使用的车辆上界，实际最少车辆数由总需求/容量下界决定。
	*/
	// Do not apply Split with fewer vehicles than the trivial (LP) bin packing bound
	maxVehicles = std::max<int>(nbMaxVehicles, std::ceil(params.totalDemand/params.vehicleCapacity));//使用车辆上界

	// Initialization of the data structures for the linear split algorithms
	// Direct application of the code located at https://github.com/vidalt/Split-Library
	for (int i = 1; i <= params.nbClients; i++)//遍历所有客户
	{
		cliSplit[i].demand = params.cli[indiv.chromT[i - 1]].demand;
		cliSplit[i].serviceTime = params.cli[indiv.chromT[i - 1]].serviceDuration;
		// 存储距离信息
		cliSplit[i].d0_x = params.timeCost[0][indiv.chromT[i - 1]];
		cliSplit[i].dx_0 = params.timeCost[indiv.chromT[i - 1]][0];
		// 存储到下一个客户的距离（最后一个客户为-∞）
		if (i < params.nbClients) cliSplit[i].dnext = params.timeCost[indiv.chromT[i - 1]][indiv.chromT[i]];
		else cliSplit[i].dnext = -1.e30;
		//计算前缀和
		sumLoad[i] = sumLoad[i - 1] + cliSplit[i].demand;
		sumService[i] = sumService[i - 1] + cliSplit[i].serviceTime;
		sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
	}

	// We first try the simple split, and then the Split with limited fleet if this is not successful
	// We first try the simple split, and then the Split with limited fleet if this is not successful
	// 中文注释：优先尝试更快的 splitSimple（通常近似 O(n)），若因车辆上界或其他约束不满足再回退到更一般的 splitLF
	if (splitSimple(indiv) == 0)
		splitLF(indiv);

	// Build up the rest of the Individual structure 评估最终成本
	indiv.evaluateCompleteCost(params);
}

int Split::splitSimple(Individual & indiv)//无限制车队分割
{	//cliSplit[i]已经对应了chromT[i-1]
	/*
	中文注释（splitSimple）:
	- 使用 Bellman 最短路径思想按拓扑顺序构建最优分割，适用于无限车队（或不显式限制车辆数）的情形。
	- 若存在时长约束，复杂度为 O(n^2)（双重循环），否则采用 O(n) 的双端队列技巧以加速。
	- 算法计算从位置 i+1 到 j 的成本（包括回到仓库的成本与对超载/超时的惩罚），并用最小化累计成本更新 potential 表与 pred 表来回溯解。
	*/
	// Reinitialize the potential structures
	//0辆车服务0个客户的成本为0,其他初始化为无穷大
	potential[0][0] = 0;
	for (int i = 1; i <= params.nbClients; i++)
		potential[0][i] = 1.e30;//0是因为不考虑车辆数

	// MAIN ALGORITHM -- Simple Split using Bellman's algorithm in topological order
	// This code has been maintained as it is very simple and can be easily adapted to a variety of constraints, whereas the O(n) Split has a more restricted application scope
	if (params.isDurationConstraint)//有时间约束，O(n^2)
	{
		for (int i = 0; i < params.nbClients; i++)
		{
			double load = 0.;
			double distance = 0.;
			double serviceDuration = 0.;
			for (int j = i + 1; j <= params.nbClients && load <= 1.5 * params.vehicleCapacity ; j++)//加速技巧：当负载超过1.5倍容量时停止（因为惩罚已很大）
			{
				load += cliSplit[j].demand;
				serviceDuration += cliSplit[j].serviceTime;
				if (j == i + 1) distance += cliSplit[j].d0_x;//车场到客户0x
				else distance += cliSplit[j - 1].dnext;//j-1 to j
				//计算从j直接终止返回车场的总惩罚成本
				double cost = distance + cliSplit[j].dx_0 
					+ params.penaltyCapacity * std::max<double>(load - params.vehicleCapacity, 0.)
					+ params.penaltyDuration * std::max<double>(distance + cliSplit[j].dx_0 + serviceDuration - params.durationLimit, 0.);
				if (potential[0][i] + cost < potential[0][j])//新开一辆车服务从i+1到j，总成本比不新开车服务到j低
				{
					potential[0][j] = potential[0][i] + cost;
					pred[0][j] = i;//新开设了一辆车，上一辆车服务到了i
				}
			}
		}
	}
	else//无时间约束，接近O(n)
	{
		Trivial_Deque queue = Trivial_Deque(params.nbClients + 1, 0);
		for (int i = 1; i <= params.nbClients; i++)
		{
			// The front is the best predecessor for i 
			potential[0][i] = propagate(queue.get_front(), i, 0);//记录已经从队头元素服务到i
			pred[0][i] = queue.get_front();

			if (i < params.nbClients)
			{
				// If i is not dominated by the last of the pile 如果i不被队尾支配，则加入队列
				if (!dominates(queue.get_back(), i, 0))//也就是说可以从i单独开个路线
				{//从队尾开始往前找，被i支配说明不可能单独开路线
					// then i will be inserted, need to remove whoever is dominated by i. 移除被i支配的队尾元素
					while (queue.size() > 0 && dominatesRight(queue.get_back(), i, 0))
						queue.pop_back();
					queue.push_back(i);
				}//现在队尾是i，下一轮又会找到新的新开路线的元素
				// Check iteratively if front is dominated by the next front 检查队首是否被第二个元素支配
				while (queue.size() > 1 && propagate(queue.get_front(), i + 1, 0) > propagate(queue.get_next_front(), i + 1, 0) - MY_EPSILON)
				//从队首元素开始转移比第二个开始转移差，移除队首元素
					queue.pop_front();
			}//队首存的是一个新的路线的开头
		}
	}

	if (potential[0][params.nbClients] > 1.e29)
		throw std::string("ERROR : no Split solution has been propagated until the last node");

	// Filling the chromR structure
	for (int k = params.nbVehicles - 1; k >= maxVehicles; k--)
		indiv.chromR[k].clear();

	int end = params.nbClients;
	for (int k = maxVehicles - 1; k >= 0; k--)
	{
		indiv.chromR[k].clear();
		int begin = pred[0][end];//上一辆车的结束位置，注意pred是[0,n-1]，而客户编号应该加一，所以下面直接是begin到end-1
		for (int ii = begin; ii < end; ii++)//将[begin, end)区间的客户加入路线k
			indiv.chromR[k].push_back(indiv.chromT[ii]);
		end = begin;// 移动到前一段路线
	}

	// Return OK in case the Split algorithm reached the beginning of the routes
	return (end == 0);
}

// Split for problems with limited fleet
int Split::splitLF(Individual & indiv)
{
	// Initialize the potential structures
	potential[0][0] = 0;
	for (int k = 0; k <= maxVehicles; k++)
		for (int i = 1; i <= params.nbClients; i++)
			potential[k][i] = 1.e30;

	// MAIN ALGORITHM -- Simple Split using Bellman's algorithm in topological order
	// This code has been maintained as it is very simple and can be easily adapted to a variety of constraints, whereas the O(n) Split has a more restricted application scope
	if (params.isDurationConstraint) 
	{
		for (int k = 0; k < maxVehicles; k++)//O(maxVehicles * n^2)
		{
			for (int i = k; i < params.nbClients && potential[k][i] < 1.e29 ; i++)//显然k辆车最少服务了前k个客户
			{
				double load = 0.;
				double serviceDuration = 0.;
				double distance = 0.;
				for (int j = i + 1; j <= params.nbClients && load <= 1.5 * params.vehicleCapacity ; j++) // Setting a maximum limit on load infeasibility to accelerate the algorithm
				{
					load += cliSplit[j].demand;
					serviceDuration += cliSplit[j].serviceTime;
					if (j == i + 1) distance += cliSplit[j].d0_x;//从i+1开始新设路径
					else distance += cliSplit[j - 1].dnext;
					double cost = distance + cliSplit[j].dx_0//从j返回车场总惩罚成本
								+ params.penaltyCapacity * std::max<double>(load - params.vehicleCapacity, 0.)
								+ params.penaltyDuration * std::max<double>(distance + cliSplit[j].dx_0 + serviceDuration - params.durationLimit, 0.);
					if (potential[k][i] + cost < potential[k + 1][j])//新设i+1到j的路线比之前的的更好
					{
						potential[k + 1][j] = potential[k][i] + cost;
						pred[k + 1][j] = i;
					}
				}
			}
		}
	}
	else // MAIN ALGORITHM -- Without duration constraints in O(n), from "Vidal, T. (2016). Split algorithm in O(n) for the capacitated vehicle routing problem. C&OR"
	{//O(maxVehicles * n)
		Trivial_Deque queue = Trivial_Deque(params.nbClients + 1, 0);
		for (int k = 0; k < maxVehicles; k++)
		{
			// in the Split problem there is always one feasible solution with k routes that reaches the index k in the tour.
			queue.reset(k);//// 重置队列，起始点为k（用k辆车至少能服务k个客户）

			// The range of potentials < 1.29 is always an interval.
			// The size of the queue will stay >= 1 until we reach the end of this interval.
			for (int i = k + 1; i <= params.nbClients && queue.size() > 0; i++)
			{
				// The front is the best predecessor for i
				potential[k + 1][i] = propagate(queue.get_front(), i, k);
				pred[k + 1][i] = queue.get_front();

				if (i < params.nbClients)
				{
					// If i is not dominated by the last of the pile 
					if (!dominates(queue.get_back(), i, k))
					{
						// then i will be inserted, need to remove whoever he dominates
						while (queue.size() > 0 && dominatesRight(queue.get_back(), i, k))
							queue.pop_back();
						queue.push_back(i);
					}

					// Check iteratively if front is dominated by the next front
					while (queue.size() > 1 && propagate(queue.get_front(), i + 1, k) > propagate(queue.get_next_front(), i + 1, k) - MY_EPSILON)
						queue.pop_front();
				}
			}
		}
	}

	if (potential[maxVehicles][params.nbClients] > 1.e29)
		throw std::string("ERROR : no Split solution has been propagated until the last node");

	// It could be cheaper to use a smaller number of vehicles
	//可能使用更少的车辆获得更低的成本（因为惩罚减少）
	double minCost = potential[maxVehicles][params.nbClients];
	int nbRoutes = maxVehicles;
	for (int k = 1; k < maxVehicles; k++)
		if (potential[k][params.nbClients] < minCost)
			{minCost = potential[k][params.nbClients]; nbRoutes = k;}//获取最小成本对应的方案

	// Filling the chromR structure
	for (int k = params.nbVehicles-1; k >= nbRoutes ; k--)
		indiv.chromR[k].clear();

	int end = params.nbClients;
	for (int k = nbRoutes - 1; k >= 0; k--)
	{
		indiv.chromR[k].clear();
		int begin = pred[k+1][end];
		for (int ii = begin; ii < end; ii++)
			indiv.chromR[k].push_back(indiv.chromT[ii]);
		end = begin;
	}

	// Return OK in case the Split algorithm reached the beginning of the routes
	return (end == 0);
}

Split::Split(const Params & params): params(params)
{
	// Structures of the linear Split
	cliSplit = std::vector <ClientSplit>(params.nbClients + 1);
	sumDistance = std::vector <double>(params.nbClients + 1,0.);
	sumLoad = std::vector <double>(params.nbClients + 1,0.);
	sumService = std::vector <double>(params.nbClients + 1, 0.);
	potential = std::vector < std::vector <double> >(params.nbVehicles + 1, std::vector <double>(params.nbClients + 1,1.e30));
	pred = std::vector < std::vector <int> >(params.nbVehicles + 1, std::vector <int>(params.nbClients + 1,0));
}
