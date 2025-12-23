#include "Genetic.h"

void Genetic::run()
{	
	/*
	中文注释（Genetic::run 总体流程）:
	- 该函数是主遗传算法循环的实现：
		1. 生成初始种群（population.generatePopulation）。
		2. 在每次迭代中进行选择和交叉（crossoverOX），得到子代 offspring。
		3. 对子代执行局部搜索（localSearch.run），并尝试将其加入种群（population.addIndividual）。
		4. 对不可行解按概率或规则进行修复（提高惩罚系数再运行 LS）。
		5. 管理惩罚、打印追踪信息、必要时重启种群（restart）。
	- 循环终止条件为：达到无改进迭代上限或超过时间上限（如果设置）。
	*/
	/* INITIAL POPULATION */
	population.generatePopulation();

	int nbIter;
	int nbIterNonProd = 1;
	if (params.verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
	for (nbIter = 0 ; nbIterNonProd <= params.ap.nbIter && (params.ap.timeLimit == 0 || (double)(clock()-params.startTime)/(double)CLOCKS_PER_SEC < params.ap.timeLimit) ; nbIter++)
	{	
		/* SELECTION AND CROSSOVER */
		crossoverOX(offspring, population.getBinaryTournament(),population.getBinaryTournament());

		/* LOCAL SEARCH */
		localSearch.run(offspring, params.penaltyCapacity, params.penaltyDuration);
		bool isNewBest = population.addIndividual(offspring,true);//updateFeasible=true控制更新可行性窗口
		if (!offspring.eval.isFeasible && params.ran()%2 == 0) // Repair half of the solutions in case of infeasibility
		{
			localSearch.run(offspring, params.penaltyCapacity*10., params.penaltyDuration*10.);//提高惩罚系数
			if (offspring.eval.isFeasible) isNewBest = (population.addIndividual(offspring,false) || isNewBest);
		}

		/* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
		if (isNewBest) nbIterNonProd = 1;
		else nbIterNonProd ++ ;//未提升的迭代次数

		/* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
		if (nbIter % params.ap.nbIterPenaltyManagement == 0) population.managePenalties();//更新惩罚系数
		if (nbIter % params.ap.nbIterTraces == 0) population.printState(nbIter, nbIterNonProd);

		/* FOR TESTS INVOLVING SUCCESSIVE RUNS UNTIL A TIME LIMIT: WE RESET THE ALGORITHM/POPULATION EACH TIME maxIterNonProd IS ATTAINED*/
		if (params.ap.timeLimit != 0 && nbIterNonProd == params.ap.nbIter)
		{
			population.restart();
			nbIterNonProd = 1;
		}
	}
	if (params.verbose) std::cout << "----- GENETIC ALGORITHM FINISHED AFTER " << nbIter << " ITERATIONS. TIME SPENT: " << (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC << std::endl;
}

void Genetic::crossoverOX(Individual & result, const Individual & parent1, const Individual & parent2)
{
		/*
		中文注释（OX 交叉说明 crossoverOX）:
		- 使用经典的 Order Crossover (OX) 变体对 parent1 和 parent2 进行顺序交叉，生成 result 的 giant-tour 表示（chromT）。
		- 具体步骤：
			1. 随机选择一个交叉区间 [start,end]（确保 start != end）。
			2. 从 parent1 的该区间直接拷贝到 result 的相同位置，并在 freqClient 中标记已插入的客户。
			3. 以 parent2 的顺序把剩余未插入的客户按循环方式填充到 result 的空位置。
			4. 最后对 giant-tour 调用 Split 算法（split.generalSplit）将其解码为路由方案（chromR），并用 parent1 的路由数量作为分割参数的初始参考。
		- 注意：result 的最终结构会经过 Split 解码并随后被局部搜索/评估。
		*/
	// Frequency table to track the customers which have been already inserted
	std::vector <bool> freqClient = std::vector <bool> (params.nbClients + 1, false);

	// Picking the beginning and end of the crossover zone
	std::uniform_int_distribution<> distr(0, params.nbClients-1);
	int start = distr(params.ran);
	int end = distr(params.ran);

	// Avoid that start and end coincide by accident
	while (end == start) end = distr(params.ran);

	// Copy from start to end
	int j = start;
	while (j % params.nbClients != (end + 1) % params.nbClients)
	{
		result.chromT[j % params.nbClients] = parent1.chromT[j % params.nbClients];
		freqClient[result.chromT[j % params.nbClients]] = true;
		j++;
	}

	// Fill the remaining elements in the order given by the second parent
	for (int i = 1; i <= params.nbClients; i++)
	{
		int temp = parent2.chromT[(end + i) % params.nbClients];
		if (freqClient[temp] == false)
		{
			result.chromT[j % params.nbClients] = temp;
			j++;
		}
	}

	// Complete the individual with the Split algorithm
	split.generalSplit(result, parent1.eval.nbRoutes);
}

Genetic::Genetic(Params & params) : 
	params(params), 
	split(params),
	localSearch(params),
	population(params,this->split,this->localSearch),
	offspring(params){}

