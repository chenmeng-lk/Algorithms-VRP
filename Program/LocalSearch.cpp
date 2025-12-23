#include "LocalSearch.h" 

void LocalSearch::run(Individual & indiv, double penaltyCapacityLS, double penaltyDurationLS)
{	
	/*
	（函数总体说明）:
	LocalSearch::run 是局部搜索（Local Search, LS）的主入口函数。
	- 将从外部传入的惩罚系数记录到当前 LS 对象，作为后续各种移动（move）成本中超量容量/时长惩罚的依据。
	- 将给定的个体（Individual）加载到 LS 内部的链式路由/节点结构中（调用 loadIndividual），以便对节点/路线做原地的修改（插入、交换、2-opt 等）。
	- 随机打乱节点与路线的遍历顺序以引入非确定性，从而提升搜索多样性与避免系统性偏差。
	- 在主循环中反复尝试若干类局部移动（move1..move9, swapStar），直到没有改进（searchCompleted=true）。
	- 最终把 LS 中的结果写回到传入的 indiv（调用 exportIndividual）。

	设计要点：
	- 为了性能，许多移动在执行前会先做快速的下界/剪枝判断（early pruning），以避免昂贵的约束检查。
	- 若某一移动导致路线被修改，会更新对应路线数据（updateRouteData），并将 searchCompleted 置为 false，触发进一步搜索。
	- 使用 params.ap.nbGranular 等参数调节搜索粒度和速度。
	*/
	// 设置惩罚系数
	this->penaltyCapacityLS = penaltyCapacityLS;
	this->penaltyDurationLS = penaltyDurationLS;
	loadIndividual(indiv);// 加载个体到内部数据结构

	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	//随机化搜索顺序，增加搜索多样性
	std::shuffle(orderNodes.begin(), orderNodes.end(), params.ran);
	std::shuffle(orderRoutes.begin(), orderRoutes.end(), params.ran);
	for (int i = 1; i <= params.nbClients; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVertices[i].begin(), params.correlatedVertices[i].end(), params.ran);

	searchCompleted = false;
	for (loopID = 0; !searchCompleted; loopID++)
	{
		/*
		循环控制说明：
		- loopID 为循环次数计数器；第一次循环(loopID==0)会跳过针对空路线的某些检查以避免过早膨胀车队规模。
		- searchCompleted 初始为 false，当经过一轮没有任何改进（由各移动返回）且 loopID>1 时，searchCompleted 会被置为 true，结束循环。
		*/
		if (loopID > 1) // Allows at least two loops since some moves involving empty routes are not checked at the first loop
			searchCompleted = true;

		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		for (int posU = 0; posU < params.nbClients; posU++)//遍历所有客户顶点
		{
			nodeU = &clients[orderNodes[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVertices[nodeU->cour].size(); posV++)//遍历邻居列表中的邻居
			{
				nodeV = &clients[params.correlatedVertices[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) 
				// only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				//增量更新检查：只评估自上次评估以来被修改过的路线
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are 
					//already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					/*
					（节点对循环逻辑）:
					- 外层遍历被打乱的节点顺序 orderNodes，内层遍历为该节点的相关邻域（params.correlatedVertices），该邻域基于几何相近或预计算的邻居列表。
					- 为了性能，仅当涉及的任一路线自上次检查后被修改（whenLastModified）时，才重新评估与 nodeU 相关的移动。
					- 对同一节点对，会按一组优先级检查不同类型的移动（move1..move9），一旦找到改善立刻应用（使用 continue 跳出当前内层循环并重新开始外层循环的下一个节点），这保证了贪心地快速应用改进。
					*/
					setLocalVariablesRouteV();
					if (move1()) continue; // RELOCATE
					if (move2()) continue; // RELOCATE
					if (move3()) continue; // RELOCATE
					if (nodeUIndex <= nodeVIndex && move4()) continue; // SWAP
					if (move5()) continue; // SWAP
					if (nodeUIndex <= nodeVIndex && move6()) continue; // SWAP
					if (intraRouteMove && move7()) continue; // 2-OPT
					if (!intraRouteMove && move8()) continue; // 2-OPT*
					if (!intraRouteMove && move9()) continue; // 2-OPT*

					// Trying moves that insert nodeU directly after the depot
					// 检查将节点直接插入到仓库后的移动
					if (nodeV->prev->isDepot)
					{
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						if (move1()) continue; // RELOCATE
						if (move2()) continue; // RELOCATE
						if (move3()) continue; // RELOCATE
						if (!intraRouteMove && move8()) continue; // 2-OPT*
						if (!intraRouteMove && move9()) continue; // 2-OPT*
					}
				}
			}

			/* MOVES INVOLVING AN EMPTY ROUTE -- NOT TESTED IN THE FIRST LOOP TO AVOID INCREASING TOO MUCH THE FLEET SIZE */
			// 检查涉及空路线的移动（第二轮循环才开始检查）
			if (loopID > 0 && !emptyRoutes.empty())
			{
				nodeV = routes[*emptyRoutes.begin()].depot;
				setLocalVariablesRouteU();
				setLocalVariablesRouteV();
				if (move1()) continue; // RELOCATE
				if (move2()) continue; // RELOCATE
				if (move3()) continue; // RELOCATE
				if (move9()) continue; // 2-OPT*
			}
		}

		if (params.ap.useSwapStar == 1 && params.areCoordinatesProvided)
		{
			/* (SWAP*) MOVES LIMITED TO ROUTE PAIRS WHOSE CIRCLE SECTORS OVERLAP */
			for (int rU = 0; rU < params.nbVehicles; rU++)
			{
				routeU = &routes[orderRoutes[rU]];
				int lastTestSWAPStarRouteU = routeU->whenLastTestedSWAPStar;
				routeU->whenLastTestedSWAPStar = nbMoves;
				for (int rV = 0; rV < params.nbVehicles; rV++)
				{
					routeV = &routes[orderRoutes[rV]];
					// 条件过滤：路线不为空、U<V避免重复、增量更新检查、扇形区域重叠
					if (routeU->nbCustomers > 0 && routeV->nbCustomers > 0 && routeU->cour < routeV->cour
						&& (loopID == 0 || std::max<int>(routeU->whenLastModified, routeV->whenLastModified)
							> lastTestSWAPStarRouteU))
						if (CircleSector::overlap(routeU->sector, routeV->sector))
							swapStar();
				}
			}
		}
	}

	// Register the solution produced by the LS in the individual
	exportIndividual(indiv);
}

void LocalSearch::setLocalVariablesRouteU()
{	/*
	（setLocalVariablesRouteU）:
	- 从当前的 nodeU 指向的位置读取并设置一组局部变量，这些变量会被许多 moveX() 函数反复使用。
	- 这些局部变量包括：
		nodeX: nodeU 的下一个节点（插入/交换的目标相关节点）
		nodeXNextIndex: nodeX->next 的客户编号（用于距离矩算式）
		nodeUIndex/nodeUPrevIndex/nodeXIndex: 各节点的索引（用于访问 params.timeCost 和 params.cli）
		loadU/serviceU/loadX/serviceX: 节点的需求和服务时间（用于负载/时长惩罚计算）
	- 通过提前解包这些变量，移动评估可以避免重复内存访问并提高可读性。
	*/
	routeU = nodeU->route;
	nodeX = nodeU->next;
	nodeXNextIndex = nodeX->next->cour;
	nodeUIndex = nodeU->cour;
	nodeUPrevIndex = nodeU->prev->cour;
	nodeXIndex = nodeX->cour;
	loadU    = params.cli[nodeUIndex].demand;
	serviceU = params.cli[nodeUIndex].serviceDuration;
	loadX	 = params.cli[nodeXIndex].demand;
	serviceX = params.cli[nodeXIndex].serviceDuration;
}

void LocalSearch::setLocalVariablesRouteV()
{
		/*
		（setLocalVariablesRouteV）:
		- 与 setLocalVariablesRouteU 类似，为 nodeV 及其邻居提取并缓存常用变量。
		- 额外设置 intraRouteMove 标志，表示当前考虑的两个节点是否属于同一路线（routeU == routeV），
			该标志用于区分在同一路线（intra）与不同路线（inter）时约束与成本计算的差异。
		*/
	routeV = nodeV->route;
	nodeY = nodeV->next;
	nodeYNextIndex = nodeY->next->cour;
	nodeVIndex = nodeV->cour;
	nodeVPrevIndex = nodeV->prev->cour;
	nodeYIndex = nodeY->cour;
	loadV    = params.cli[nodeVIndex].demand;
	serviceV = params.cli[nodeVIndex].serviceDuration;
	loadY	 = params.cli[nodeYIndex].demand;
	serviceY = params.cli[nodeYIndex].serviceDuration;
	intraRouteMove = (routeU == routeV);
}

bool LocalSearch::move1()
{
	/*
	（move1 - 单点 RELOCATE）:
	- 这是经典的单点搬迁（Relocate）操作：将 nodeU 从其当前位置移除并插入到 nodeV 之后。
	- 首先计算两个受影响路线的距离变化 costSuppU, costSuppV（仅基于距离的增量下界）。
	- 若为跨路（!intraRouteMove），先做快速剪枝：若 costSuppU + costSuppV >= routeU->penalty + routeV->penalty，则无改进可能，直接返回 false。
	- 否则将考虑时长/载重的惩罚增量（使用 penaltyExcessDuration/penaltyExcessLoad），并再做一次阈值判断。
	- 若满足改进（总改变量 < 0），执行插入（insertNode）、更新计数并调用 updateRouteData 更新路线信息。
	- 同一路线上的搬迁会跳过一些跨路的惩罚计算，但仍遵循成本判断与相交条件（避免 self-move）。
	*/
	
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex];//移除U结点后成本变化值
	double costSuppV = params.timeCost[nodeVIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];//V后添加U结点后成本变化值

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;
		//考虑时长/载重的惩罚增量（进行move1后总的惩罚变化值，若为正说明移动不可取）
		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU)//原路线总耗时+更改路线变化-U服务时间
			+ penaltyExcessLoad(routeU->load - loadU)//U所在路线移除U，容量减小loadU
			- routeU->penalty;//原路线惩罚

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU)
			+ penaltyExcessLoad(routeV->load + loadU)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;//（进行move1后总的惩罚变化值，若为正说明移动不可取）
	if (nodeUIndex == nodeYIndex) return false;//将 nodeU 从其当前位置移除并插入到 nodeV 之后，但U本来就在V之后，无效移动

	insertNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);//U被移除，更新路线参数
	if (!intraRouteMove) updateRouteData(routeV);//路线间移动，V所在路线也要更新
	return true;
}

bool LocalSearch::move2()
{
	/*
	（move2 - 双节点连续插入 RELOCATE）:
	- 该移动把 nodeU 和其后继 nodeX 两个紧邻节点移动到 nodeV 之后，保持它们的相对顺序。
	- 成本计算更复杂，涉及 nodeX 的后继（nodeXNextIndex）以评估移除后链路代价。
	- 同样先做跨路快速剪枝，再考虑惩罚项，最后在满足改进时执行两次插入操作（insertNode）。
	- 此移动需要额外检查避免与 depot 或相邻关系冲突。
	*/
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];////移除U X结点后路线成本变化值
	double costSuppV = params.timeCost[nodeVIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params.timeCost[nodeUIndex][nodeXIndex] - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params.timeCost[nodeUIndex][nodeXIndex] + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeV == nodeX || nodeX->isDepot) return false;//U本来就在V后；U在V前；X是车场

	insertNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move3()
{
	/*
	（move3 - 另一种二元 RELOCATE 组合）:
	- 这个移动首先把 nodeX 插入到 nodeV 后，然后把 nodeU 插到 nodeX 的位置，从而实现特定的相对重排。
	- 成本与剪枝逻辑与 move2 类似，但移动顺序不同，所以对相邻性和 depot 的检查条件也不同。
	*/
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVIndex][nodeXIndex] + params.timeCost[nodeXIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeY || nodeX == nodeV || nodeX->isDepot) return false;

	insertNode(nodeX, nodeV);
	insertNode(nodeU, nodeX);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

bool LocalSearch::move4()
{
	/*
	（move4 - 双点 SWAP）:
	- 将 nodeU 与 nodeV 直接交换位置（两点互换），即经典的 1-1 交换操作。
	- 先计算交换引起的距离差分 costSuppU, costSuppV，并在跨路时使用 penalty 进行快速剪枝。
	- 如果成本改进成立且不触发简单的相邻冲突条件，则执行 swapNode 操作来调整链表连接，然后更新路线。
	*/
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeUIndex][nodeXIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeUIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU + serviceV - serviceU)
			+ penaltyExcessLoad(routeU->load + loadV - loadU)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV - serviceV + serviceU)
			+ penaltyExcessLoad(routeV->load + loadU - loadV)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeUIndex == nodeVPrevIndex || nodeUIndex == nodeYIndex) return false;//U V相邻
	//判定相邻并直接返回 false 是为了防止 swapNode 在相邻场景下引发链表损坏，并利用已有的 relocate/2-opt 实现覆盖相邻情形的改进。
	swapNode(nodeU, nodeV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

//UX作为整体和V交换
bool LocalSearch::move5()
{
	/*
	（move5 - 1-1 交换的扩展）:
	- 这是 swap 类移动的一种变体，涉及 nodeU、nodeV、nodeX 的组合：先 swap，然后对 nodeX 做相应插入，
		从而实现更复杂的局部再排列。相当于把UX作为整体和V交换
	- 同样按照“快速剪枝 -> 约束惩罚修正 -> 应用变动 -> 更新路线”的模式执行。
	*/
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeVIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeVIndex][nodeYIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params.timeCost[nodeUIndex][nodeXIndex] + serviceV - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load + loadV - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params.timeCost[nodeUIndex][nodeXIndex] - serviceV + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX - loadV)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeU == nodeV->prev || nodeX == nodeV->prev || nodeU == nodeY || nodeX->isDepot) return false;
	//以上分别是UV(X)、UXV、VUX、U0情况
	swapNode(nodeU, nodeV);
	insertNode(nodeX, nodeU);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

//交换UX（作为整体）和VY
bool LocalSearch::move6()
{
	/*
	（move6 - 双点与其后继的交换）:
	- 该移动在两个路线（或同一路线）上同时交换了一对节点 (U<->V) 以及它们的后继 (X<->Y)，相当于交换两个二元块。
	- 条件较多，必须确保不会涉及 depot 或相邻冲突，并在跨路情况下计算惩罚修正。
	*/
	double costSuppU = params.timeCost[nodeUPrevIndex][nodeVIndex] + params.timeCost[nodeYIndex][nodeXNextIndex] - params.timeCost[nodeUPrevIndex][nodeUIndex] - params.timeCost[nodeXIndex][nodeXNextIndex];
	double costSuppV = params.timeCost[nodeVPrevIndex][nodeUIndex] + params.timeCost[nodeXIndex][nodeYNextIndex] - params.timeCost[nodeVPrevIndex][nodeVIndex] - params.timeCost[nodeYIndex][nodeYNextIndex];

	if (!intraRouteMove)
	{
		// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
		if (costSuppU + costSuppV >= routeU->penalty + routeV->penalty) return false;

		costSuppU += penaltyExcessDuration(routeU->duration + costSuppU - params.timeCost[nodeUIndex][nodeXIndex] + params.timeCost[nodeVIndex][nodeYIndex] + serviceV + serviceY - serviceU - serviceX)
			+ penaltyExcessLoad(routeU->load + loadV + loadY - loadU - loadX)
			- routeU->penalty;

		costSuppV += penaltyExcessDuration(routeV->duration + costSuppV + params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex] - serviceV - serviceY + serviceU + serviceX)
			+ penaltyExcessLoad(routeV->load + loadU + loadX - loadV - loadY)
			- routeV->penalty;
	}

	if (costSuppU + costSuppV > -MY_EPSILON) return false;
	if (nodeX->isDepot || nodeY->isDepot || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next) return false;

	swapNode(nodeU, nodeV);
	swapNode(nodeX, nodeY);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	if (!intraRouteMove) updateRouteData(routeV);
	return true;
}

//经典2-opt
bool LocalSearch::move7()
{
	/*
	（move7 - intra-route 2-OPT）:
	- 当 nodeU 与 nodeV 在同一路线上且 nodeU 在 nodeV 之前时，执行经典的 2-OPT 段反转（局部反转），用于消除路径交叉。
	- 计算基于 cumulatedReversalDistance 的成本改变量，若改进则反转这段链表中的指针并更新路线数据。
	- 注意：该操作在同一路线内有效（intraRouteMove），并且避免对相邻节点执行无效反转。
	*/
	if (nodeU->position > nodeV->position) return false;
	//距离改变量 + X到V的反转成本
	double cost = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex] + nodeV->cumulatedReversalDistance - nodeX->cumulatedReversalDistance;

	if (cost > -MY_EPSILON) return false;//反转成本为负值才采用
	if (nodeU->next == nodeV) return false;//避免对相邻节点执行无效反转

	Node * nodeNum = nodeX->next;//从X到V反转
	nodeX->prev = nodeNum;
	nodeX->next = nodeY;

	while (nodeNum != nodeV)
	{
		Node * temp = nodeNum->next;
		nodeNum->next = nodeNum->prev;
		nodeNum->prev = temp;
		nodeNum = temp;
	}

	nodeV->next = nodeV->prev;
	nodeV->prev = nodeU;
	nodeU->next = nodeV;
	nodeY->prev = nodeX;

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);//调用前已经确定了是路线内移动
	return true;
}

//0->U连V再反转到0；0反转到X连Y->0
bool LocalSearch::move8()
{
	/*
	（move8 - inter-route 2-OPT*）:
	- 这是 inter-route 的 2-OPT* 变体，既可以交换两个子路径并将它们移到对方的路线中，
		以此来同时改变两条路线的结构，常用于搜索跨路的大范围改进。
	- 先用距离下界过滤（cost >= 0 则直接返回），再准确计算涉及两条路线上多段时间和负载的惩罚增量。
	- 若改进成立，则通过一系列指针翻转（交换 next/prev），并把节点的 route 指针切换到新的路线，最后更新两条路线。
	- 此处需要谨慎处理 depot 连接（特殊分支）以保证链表连通性正确。
	*/
	double cost = params.timeCost[nodeUIndex][nodeVIndex] + params.timeCost[nodeXIndex][nodeYIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex]
		+ nodeV->cumulatedReversalDistance + routeU->reversalDistance - nodeX->cumulatedReversalDistance//X到depot反转，depot到V反转
		- routeU->penalty - routeV->penalty;

	// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
	if (cost >= 0) return false;
			//原路线U的前半段（直到nodeU）和原路线V的前半段反转（直到nodeV）合并
	cost += penaltyExcessDuration(nodeU->cumulatedTime + nodeV->cumulatedTime + nodeV->cumulatedReversalDistance + params.timeCost[nodeUIndex][nodeVIndex])
		//U的后段时间成本 - UX + X->0的反转成本 + Y后段时间成本 - VY + XY
		+ penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - params.timeCost[nodeUIndex][nodeXIndex] + routeU->reversalDistance - nodeX->cumulatedReversalDistance + routeV->duration - nodeV->cumulatedTime - params.timeCost[nodeVIndex][nodeYIndex] + params.timeCost[nodeXIndex][nodeYIndex])
		//0到U和0到V的负载惩罚
		+ penaltyExcessLoad(nodeU->cumulatedLoad + nodeV->cumulatedLoad)
		//X到0和Y到0的负载惩罚
		+ penaltyExcessLoad(routeU->load + routeV->load - nodeU->cumulatedLoad - nodeV->cumulatedLoad);
		
	if (cost > -MY_EPSILON) return false;

	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = routeU->depot->prev;
	Node * depotVFin = routeV->depot->prev;
	Node * depotVSuiv = depotV->next;

	Node * temp;
	Node * xx = nodeX;
	Node * vv = nodeV;

	while (!xx->isDepot)//反转X到depot(U)，并归到V路径
	{
		temp = xx->next;
		xx->next = xx->prev;
		xx->prev = temp;
		xx->route = routeV;
		xx = temp;
	}

	while (!vv->isDepot)//反转depot(V)到V，并归到路线U
	{
		temp = vv->prev;
		vv->prev = vv->next;
		vv->next = temp;
		vv->route = routeU;
		vv = temp;
	}
	//连接UV、XY
	nodeU->next = nodeV;
	nodeV->prev = nodeU;
	nodeX->next = nodeY;
	nodeY->prev = nodeX;

	if (nodeX->isDepot)//确保链表闭合不丢失节点
	{
		depotUFin->next = depotU;
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;//// 路线U变成空路线
		depotV->next = nodeY;
		nodeY->prev = depotV;// 路线V：仓库V直接连接到nodeY
	}
	else if (nodeV->isDepot)
	{
		depotV->next = depotUFin->prev;// 路线V变成空路线
		depotV->next->prev = depotV;
		depotV->prev = depotVFin;
		depotUFin->prev = nodeU;// 路线U：nodeU后面连接到路线U的末尾
		nodeU->next = depotUFin;
	}
	else
	{
		depotV->next = depotUFin->prev;
		depotV->next->prev = depotV;// 路线V的仓库连接到路线U末尾的前一个节点
		depotUFin->prev = depotVSuiv;
		depotUFin->prev->next = depotUFin;// 路线U的末尾连接到路线V的开始（除仓库外的第一个节点）
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

//连接UY、VX，断开UX、VY，不需要反转
bool LocalSearch::move9()
{
	/*
	（move9 - inter-route 2-OPT* 的另一种实现）:
	- move9 实现另一类 inter-route 子路径交换逻辑（与 move8 目标类似但实现细节与临界条件不同）。
	- 同样包含快速剪枝、惩罚增量计算、节点 route 更新与 depot 连接修复。
	*/
	double cost = params.timeCost[nodeUIndex][nodeYIndex] + params.timeCost[nodeVIndex][nodeXIndex] - params.timeCost[nodeUIndex][nodeXIndex] - params.timeCost[nodeVIndex][nodeYIndex]
		        - routeU->penalty - routeV->penalty;

	// Early move pruning to save CPU time. Guarantees that this move cannot improve without checking additional (load, duration...) constraints
	if (cost >= 0) return false;
		
	cost += penaltyExcessDuration(nodeU->cumulatedTime + routeV->duration - nodeV->cumulatedTime - params.timeCost[nodeVIndex][nodeYIndex] + params.timeCost[nodeUIndex][nodeYIndex])
		+ penaltyExcessDuration(routeU->duration - nodeU->cumulatedTime - params.timeCost[nodeUIndex][nodeXIndex] + nodeV->cumulatedTime + params.timeCost[nodeVIndex][nodeXIndex])
		+ penaltyExcessLoad(nodeU->cumulatedLoad + routeV->load - nodeV->cumulatedLoad)
		+ penaltyExcessLoad(nodeV->cumulatedLoad + routeU->load - nodeU->cumulatedLoad);

	if (cost > -MY_EPSILON) return false;

	Node * depotU = routeU->depot;
	Node * depotV = routeV->depot;
	Node * depotUFin = depotU->prev;
	Node * depotVFin = depotV->prev;
	Node * depotUpred = depotUFin->prev;

	Node * count = nodeY;
	while (!count->isDepot)//调整Y到depot成U的路线
	{
		count->route = routeU;
		count = count->next;
	}

	count = nodeX;
	while (!count->isDepot)
	{
		count->route = routeV;
		count = count->next;
	}

	nodeU->next = nodeY;
	nodeY->prev = nodeU;
	nodeV->next = nodeX;
	nodeX->prev = nodeV;

	if (nodeX->isDepot)
	{
		depotUFin->prev = depotVFin->prev;// 路线U的末尾连接到路线V末尾的前一个节点
		depotUFin->prev->next = depotUFin;
		nodeV->next = depotVFin;// nodeV连接到路线V的末尾
		depotVFin->prev = nodeV;
	}
	else
	{
		depotUFin->prev = depotVFin->prev;// 路线U的末尾连接到路线V末尾的前一个节点
		depotUFin->prev->next = depotUFin;
		depotVFin->prev = depotUpred;// 路线V的末尾连接到路线U末尾的前一个节点
		depotVFin->prev->next = depotVFin;
	}

	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

bool LocalSearch::swapStar()
{
	/*
	（swapStar - SWAP* 算子）:
	- Swap* 是一种高效的多位置交换策略：对于两条路线，尝试移除某个节点并将其插入到另一条路线的若干最佳位置（并结合同时移除对方节点的再插入成本），
		通过预处理插入成本（preprocessInsertions）与贪心选择最优组合，找到能带来全局改进的复合移动。
	- 算法流程：
		1. 对两条路线分别调用 preprocessInsertions 预计算每个候选节点的 top-3 插入成本。
		2. 枚举 routeU 中的每个 nodeU 与 routeV 中的每个 nodeV，快速计算容量惩罚变化与 removal 成本下界，进行过滤。
		3. 对符合条件的候选，调用 getCheapestInsertSimultRemoval 来计算在移除对方节点的情形下的最优再插入成本。
		4. 选出最小的 moveCost，并在最终判定为改进时执行对应的插入（或 relocate）操作。
	- 该算子能发现复杂的跨路改进，通常比单纯的单点或 2-OPT 更强，但计算更重，因此受限于圈扇（CircleSector）重叠用于减少比较对数。
	*/
	SwapStarElement myBestSwapStar;

	// Preprocessing insertion costs
	preprocessInsertions(routeU, routeV);// 计算routeU中节点插入routeV的cost
	preprocessInsertions(routeV, routeU);// 计算routeV中节点插入routeU的cost
	//情况1：U、V各自插入到对方线路最佳位置
	// Evaluating the moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{//遍历U路线所有客户结点
		for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
		{
			//仅考虑负载惩罚变化
			double deltaPenRouteU = penaltyExcessLoad(routeU->load + params.cli[nodeV->cour].demand - params.cli[nodeU->cour].demand) - routeU->penalty;
			double deltaPenRouteV = penaltyExcessLoad(routeV->load + params.cli[nodeU->cour].demand - params.cli[nodeV->cour].demand) - routeV->penalty;

			// Quick filter: possibly early elimination of many SWAP* due to the capacity constraints/penalties and bounds on insertion costs
			if (deltaPenRouteU + nodeU->deltaRemoval + deltaPenRouteV + nodeV->deltaRemoval <= 0)//负载惩罚变化+移除距离变化得到改进时
			{//如果U、V此方案有效
				SwapStarElement mySwapStar;
				mySwapStar.U = nodeU;
				mySwapStar.V = nodeV;

				// Evaluate best reinsertion cost of U in the route of V where V has been removed
				double extraV = getCheapestInsertSimultRemoval(nodeU, nodeV, mySwapStar.bestPositionU);

				// Evaluate best reinsertion cost of V in the route of U where U has been removed
				double extraU = getCheapestInsertSimultRemoval(nodeV, nodeU, mySwapStar.bestPositionV);

				// Evaluating final cost     move总成本：负载惩罚变化+两边插入的距离变化+超时惩罚
				mySwapStar.moveCost = deltaPenRouteU + nodeU->deltaRemoval + extraU + deltaPenRouteV + nodeV->deltaRemoval + extraV
					+ penaltyExcessDuration(routeU->duration + nodeU->deltaRemoval + extraU + params.cli[nodeV->cour].serviceDuration - params.cli[nodeU->cour].serviceDuration)
					+ penaltyExcessDuration(routeV->duration + nodeV->deltaRemoval + extraV - params.cli[nodeV->cour].serviceDuration + params.cli[nodeU->cour].serviceDuration);

				if (mySwapStar.moveCost < myBestSwapStar.moveCost)//记录当前最优交换组合
					myBestSwapStar = mySwapStar;
			}
		}
	}
	//情况2：仅把U插入到V线路的最佳方案
	// Including RELOCATE from nodeU towards routeV (costs nothing to include in the evaluation at this step since we already have the best insertion location)
	// Moreover, since the granularity criterion is different, this can lead to different improving moves
	for (nodeU = routeU->depot->next; !nodeU->isDepot; nodeU = nodeU->next)
	{
		SwapStarElement mySwapStar;
		mySwapStar.U = nodeU;
		mySwapStar.bestPositionU = bestInsertClient[routeV->cour][nodeU->cour].bestLocation[0];
		double deltaDistRouteU = params.timeCost[nodeU->prev->cour][nodeU->next->cour] - params.timeCost[nodeU->prev->cour][nodeU->cour] - params.timeCost[nodeU->cour][nodeU->next->cour];
		double deltaDistRouteV = bestInsertClient[routeV->cour][nodeU->cour].bestCost[0];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load - params.cli[nodeU->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load + params.cli[nodeU->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU - params.cli[nodeU->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV + params.cli[nodeU->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}
	//情况3：仅把V插入到U线路的最佳方案
	// Including RELOCATE from nodeV towards routeU
	for (nodeV = routeV->depot->next; !nodeV->isDepot; nodeV = nodeV->next)
	{
		SwapStarElement mySwapStar;
		mySwapStar.V = nodeV;
		mySwapStar.bestPositionV = bestInsertClient[routeU->cour][nodeV->cour].bestLocation[0];
		double deltaDistRouteU = bestInsertClient[routeU->cour][nodeV->cour].bestCost[0];
		double deltaDistRouteV = params.timeCost[nodeV->prev->cour][nodeV->next->cour] - params.timeCost[nodeV->prev->cour][nodeV->cour] - params.timeCost[nodeV->cour][nodeV->next->cour];
		mySwapStar.moveCost = deltaDistRouteU + deltaDistRouteV
			+ penaltyExcessLoad(routeU->load + params.cli[nodeV->cour].demand) - routeU->penalty
			+ penaltyExcessLoad(routeV->load - params.cli[nodeV->cour].demand) - routeV->penalty
			+ penaltyExcessDuration(routeU->duration + deltaDistRouteU + params.cli[nodeV->cour].serviceDuration)
			+ penaltyExcessDuration(routeV->duration + deltaDistRouteV - params.cli[nodeV->cour].serviceDuration);

		if (mySwapStar.moveCost < myBestSwapStar.moveCost)
			myBestSwapStar = mySwapStar;
	}

	if (myBestSwapStar.moveCost > -MY_EPSILON) return false;//最优的移动不行

	// Applying the best move in case of improvement	应用最优解
	if (myBestSwapStar.bestPositionU != NULL) insertNode(myBestSwapStar.U, myBestSwapStar.bestPositionU);
	if (myBestSwapStar.bestPositionV != NULL) insertNode(myBestSwapStar.V, myBestSwapStar.bestPositionV);
	nbMoves++; // Increment move counter before updating route data
	searchCompleted = false;
	updateRouteData(routeU);
	updateRouteData(routeV);
	return true;
}

//U、V同时移除情况下的最佳插入，返回最低插入成本
double LocalSearch::getCheapestInsertSimultRemoval(Node * U, Node * V, Node *& bestPosition)
{
	/*
	（getCheapestInsertSimultRemoval）:
	- 目的：给定要插入的节点 U 与已移除节点 V 的上下文，返回在 V 被移除后 U 在 V 的路线中最便宜的插入位置及对应成本。
	- 实现细节：
		* 使用预计算的 top-3 插入位置（ThreeBestInsert），先从中选一个不与 V 相邻的位置（避免造成自相邻冲突）。
		* 如果预计算位置都与 V 相邻或不可用，函数会显式地计算把 U 插入到 V 的位置（即 V->prev）的成本，并与先前最优比较。
	- 返回值：最小插入代价（仅距离增量），并通过 bestPosition 输出对应的插入节点（表示插入到 bestPosition 之后）。
	*/
	ThreeBestInsert * myBestInsert = &bestInsertClient[V->route->cour][U->cour];
	bool found = false;

	// Find best insertion in the route such that V is not next or pred (can only belong to the top three locations)
	bestPosition = myBestInsert->bestLocation[0];
	double bestCost = myBestInsert->bestCost[0];
	found = (bestPosition != V && bestPosition->next != V);
	if (!found && myBestInsert->bestLocation[1] != NULL)//最佳插入位置与V相邻且存在第二好位置
	{
		bestPosition = myBestInsert->bestLocation[1];
		bestCost = myBestInsert->bestCost[1];
		found = (bestPosition != V && bestPosition->next != V);
		if (!found && myBestInsert->bestLocation[2] != NULL)//第二个位置也不可行
		{
			bestPosition = myBestInsert->bestLocation[2];//第三个位置，显然不会和V再相邻了
			bestCost = myBestInsert->bestCost[2];
			found = true;
		}
	}

	// Compute insertion in the place of V 把U插到V的位置
	double deltaCost = params.timeCost[V->prev->cour][U->cour] + params.timeCost[U->cour][V->next->cour] - params.timeCost[V->prev->cour][V->next->cour];
	if (!found || deltaCost < bestCost)//如果预计算位置都与 V 相邻或不可用，函数会显式地计算把 U 插入到 V 的位置（即 V->prev）的成本，并与先前最优比较
	{//把U插到V的位置成本更低就更新
		bestPosition = V->prev;
		bestCost = deltaCost;
	}

	return bestCost;
}

//为 R1 中的每个节点 U 预计算其在 R2 中的 top-3 最优插入位置及对应成本
void LocalSearch::preprocessInsertions(Route * R1, Route * R2)
{
	/*
	（preprocessInsertions）:
	- 目的：为 R1 中的每个节点 U 预计算其在 R2 中的 top-3 最优插入位置及对应成本，并缓存到 bestInsertClient。
	- 作用：swapStar 算子会频繁查询某个节点在另一条路线的最佳插入位置，预处理可以显著减少重复计算。
	- 细节：
		* 对 R1 中每个 U，先计算其移除带来的距离节省（deltaRemoval）。
		* 若 R2 自上次计算后发生修改（whenLastModified），则重置并重新计算 U 在 R2 的候选插入成本（包括 depot 前插入作为初始候选）。
	*/
	for (Node * U = R1->depot->next; !U->isDepot; U = U->next)//遍历R1中的所有客户节点
	{
		// Performs the preprocessing
		U->deltaRemoval = params.timeCost[U->prev->cour][U->next->cour] - params.timeCost[U->prev->cour][U->cour] - params.timeCost[U->cour][U->next->cour];
		if (R2->whenLastModified > bestInsertClient[R2->cour][U->cour].whenLastCalculated)//只有R2在插入成本计算后被修改过，才需要重新计算
		{
			bestInsertClient[R2->cour][U->cour].reset();//初始化第一个最佳位置:R2中depot到第一个客户之间
			bestInsertClient[R2->cour][U->cour].whenLastCalculated = nbMoves;
			bestInsertClient[R2->cour][U->cour].bestCost[0] = params.timeCost[0][U->cour] + params.timeCost[U->cour][R2->depot->next->cour] - params.timeCost[0][R2->depot->next->cour];
			bestInsertClient[R2->cour][U->cour].bestLocation[0] = R2->depot;
			for (Node * V = R2->depot->next; !V->isDepot; V = V->next)//遍历所有可能的插入位置
			{//U插到V和V->next之间
				double deltaCost = params.timeCost[V->cour][U->cour] + params.timeCost[U->cour][V->next->cour] - params.timeCost[V->cour][V->next->cour];
				bestInsertClient[R2->cour][U->cour].compareAndAdd(deltaCost, V);
			}
		}
	}
}

//将 U 从当前链表位置摘除并插入到 V 之后
void LocalSearch::insertNode(Node * U, Node * V)
{
	/*
	（insertNode）:
	- 在链表上执行原子插入操作：将 U 从当前链表位置摘除并插入到 V 之后（即 U 将成为 V->next）。
	- 具体操作：
		* 先将 U 的前驱连到 U 的后继，从原来位置移除 U。
		* 然后把 U 插入到 V 与 V->next 之间，修正 prev/next 指针并更新 U->route 指向。
	- 该实现假定调用者已确保插入是合法的（不会导致 self-loop 或 depot 冲突），并在插入后调用 updateRouteData 来修正聚合信息。
	*/
	U->prev->next = U->next;
	U->next->prev = U->prev;
	V->next->prev = U;
	U->prev = V;
	U->next = V->next;
	V->next = U;
	U->route = V->route;
}

//在链表上交换两个独立节点U与V的位置
void LocalSearch::swapNode(Node * U, Node * V)
{
		/*
		（swapNode）:
		- 在链表上交换两个独立节点 U 与 V 的位置（不改变两节点各自后继/前驱的内部相对顺序），
			即把 U 插到 V 的位置，把 V 插到 U 的位置，并保持链表其它部分连通性。
		- 实现思路：先保存 U/V 的邻居指针与所属路线指针，然后调整四个邻居的 next/prev 指向，最后修改 U/V 的 prev/next 与 route 字段。
		- 该操作不会直接更新路线的聚合信息（需调用 updateRouteData）。
		*/
	Node * myVPred = V->prev;
	Node * myVSuiv = V->next;
	Node * myUPred = U->prev;
	Node * myUSuiv = U->next;
	Route * myRouteU = U->route;
	Route * myRouteV = V->route;

	myUPred->next = V;
	myUSuiv->prev = V;
	myVPred->next = U;
	myVSuiv->prev = U;

	U->prev = myVPred;
	U->next = myVSuiv;
	V->prev = myUPred;
	V->next = myUSuiv;

	U->route = myRouteV;
	V->route = myRouteU;
}

//重新计算并更新给定路线 myRoute 的一系列聚合信息
void LocalSearch::updateRouteData(Route * myRoute)
{
		/*
		（updateRouteData）:
		- 功能：重新计算并更新给定路线 myRoute 的一系列聚合信息，包括：
			* 每个节点的 position, cumulatedLoad, cumulatedTime, cumulatedReversalDistance
			* 路线总的 duration, load, penalty（基于 penaltyExcessDuration/Load）、nbCustomers, reversalDistance
			* 路线的极角重心（polarAngleBarycenter）和 CircleSector，用于 swap* 的几何过滤
		- 关键要点：
			* 使用链式结构从 depot 遍历路线，累积时间与负载，更新节点的前缀量（前缀和）便于后续 2-OPT 等计算使用。
			* 对空路线设置 polarAngleBarycenter 为 1.e30，并把该路线 id 插入 emptyRoutes 集合，
				非空路线则移出 emptyRoutes 并计算实际的极角重心（用于按角度对路线排序/导出）。
			* 最后记录 myRoute->whenLastModified = nbMoves，用于后续移动评估的过滤。
		*/
	int myplace = 0;
	double myload = 0.;
	double mytime = 0.;
	double myReversalDistance = 0.;
	double cumulatedX = 0.;
	double cumulatedY = 0.;

	Node * mynode = myRoute->depot;
	mynode->position = 0;
	mynode->cumulatedLoad = 0.;
	mynode->cumulatedTime = 0.;
	mynode->cumulatedReversalDistance = 0.;

	bool firstIt = true;
	while (!mynode->isDepot || firstIt)
	{
		mynode = mynode->next;//从车场开始遍历路径
		myplace++;
		mynode->position = myplace;
		//各个结点累积负载、时间与反转距离
		myload += params.cli[mynode->cour].demand;
		mytime += params.timeCost[mynode->prev->cour][mynode->cour] + params.cli[mynode->cour].serviceDuration;
		myReversalDistance += params.timeCost[mynode->cour][mynode->prev->cour] - params.timeCost[mynode->prev->cour][mynode->cour] ;
		mynode->cumulatedLoad = myload;
		mynode->cumulatedTime = mytime;
		mynode->cumulatedReversalDistance = myReversalDistance;
		//客户结点，计算路线的极角重心
		if (!mynode->isDepot)
		{
			cumulatedX += params.cli[mynode->cour].coordX;
			cumulatedY += params.cli[mynode->cour].coordY;
			if (firstIt) myRoute->sector.initialize(params.cli[mynode->cour].polarAngle);
			else myRoute->sector.extend(params.cli[mynode->cour].polarAngle);
		}
		firstIt = false;
	}
	//路线的总时间、负载、惩罚、客户数、反转距离、记录路线最后一次修改时间（移动操作计数器）
	myRoute->duration = mytime;
	myRoute->load = myload;
	myRoute->penalty = penaltyExcessDuration(mytime) + penaltyExcessLoad(myload);
	myRoute->nbCustomers = myplace-1;
	myRoute->reversalDistance = myReversalDistance;
	// Remember "when" this route has been last modified (will be used to filter unnecessary move evaluations)
	myRoute->whenLastModified = nbMoves ;

	if (myRoute->nbCustomers == 0) //空路线
	{
		myRoute->polarAngleBarycenter = 1.e30;
		emptyRoutes.insert(myRoute->cour);
	}
	else //非空路线则移出 emptyRoutes 并计算实际的极角重心
	{
		myRoute->polarAngleBarycenter = atan2(cumulatedY/(double)myRoute->nbCustomers - params.cli[0].coordY, cumulatedX/(double)myRoute->nbCustomers - params.cli[0].coordX);
		emptyRoutes.erase(myRoute->cour);
	}
}

//将一个 Individual 的染色体(chromR/chromT) 转换成 LS 使用的链式路线与节点结构
void LocalSearch::loadIndividual(const Individual & indiv)
{
		/*
		（loadIndividual）:
		- 作用：将一个 Individual 的染色体(chromR/chromT) 转换成 LS 使用的链式路线与节点结构（clients, routes, depots, depotsEnd）。
		- 过程：
			* 清空 emptyRoutes，重置 nbMoves。
			* 对于每辆车 r：把 depot 与 depotEnd 连接起来；若 individ.chromR[r] 非空，则用 clients 向链表中插入对应客户节点，并设置它们的 route 指针、prev/next。
			* 在每条路线装填完成后调用 updateRouteData 计算聚合信息。
			* 初始化与该路线相关的 bestInsertClient 缓存（whenLastCalculated=-1），以及全局 clients 的 whenLastTestedRI。
		- 该函数把离散的解表示转为便于原地修改的指针结构，是 LocalSearch 正确工作的前提。
		*/
	emptyRoutes.clear();
	nbMoves = 0; 
	for (int r = 0; r < params.nbVehicles; r++)//遍历每条路线中的结点
	{
		Node * myDepot = &depots[r];
		Node * myDepotFin = &depotsEnd[r];
		Route * myRoute = &routes[r];
		myDepot->prev = myDepotFin;
		myDepotFin->next = myDepot;
		if (!indiv.chromR[r].empty())//路线r的序列非空
		{
			Node * myClient = &clients[indiv.chromR[r][0]];
			myClient->route = myRoute;
			myClient->prev = myDepot;
			myDepot->next = myClient;
			for (int i = 1; i < (int)indiv.chromR[r].size(); i++)//链表建立
			{
				Node * myClientPred = myClient;
				myClient = &clients[indiv.chromR[r][i]]; 
				myClient->prev = myClientPred;
				myClientPred->next = myClient;
				myClient->route = myRoute;
			}
			myClient->next = myDepotFin;
			myDepotFin->prev = myClient;
		}
		else//r是空路线，直接连接depot和哨兵depotfin
		{
			myDepot->next = myDepotFin;
			myDepotFin->prev = myDepot;
		}
		//初始化路线数据
		updateRouteData(&routes[r]);
		routes[r].whenLastTestedSWAPStar = -1;
		for (int i = 1; i <= params.nbClients; i++) // Initializing memory structures
			bestInsertClient[r][i].whenLastCalculated = -1;
	}

	for (int i = 1; i <= params.nbClients; i++) // Initializing memory structures
		clients[i].whenLastTestedRI = -1;
}

//将 LS 内部链式结构的结果写回传入的 Individual 对象
void LocalSearch::exportIndividual(Individual & indiv)
{
		/*
		（exportIndividual）:
		- 作用：将 LS 内部链式结构的结果写回传入的 Individual 对象（覆盖其 chromR 和 chromT）。
		- 细节：
			* 先按路线的极角重心对路线进行排序，以便输出时把相近路线放在一起（empty routes 通过 1.e30 保证排在末尾）。
			* 遍历每条路线，将节点按顺序写入 indiv.chromR，同时按顺序填充 indiv.chromT（giant tour 表示）。
			* 最后调用 indiv.evaluateCompleteCost(params) 校验并更新个体的评估值（distance, penalizedCost 等）。
		*/
	std::vector < std::pair <double, int> > routePolarAngles ;//路径r和其极角重心
	for (int r = 0; r < params.nbVehicles; r++)
		routePolarAngles.push_back(std::pair <double, int>(routes[r].polarAngleBarycenter, r));
	std::sort(routePolarAngles.begin(), routePolarAngles.end()); // empty routes have a polar angle of 1.e30, and therefore will always appear at the end

	int pos = 0;//遍历每条路线，将节点按顺序写入 indiv.chromR，同时按顺序填充 indiv.chromT
	for (int r = 0; r < params.nbVehicles; r++)
	{
		indiv.chromR[r].clear();
		Node * node = depots[routePolarAngles[r].second].next;
		while (!node->isDepot)
		{
			indiv.chromT[pos] = node->cour;
			indiv.chromR[r].push_back(node->cour);
			node = node->next;
			pos++;
		}
	}

	indiv.evaluateCompleteCost(params);
}

LocalSearch::LocalSearch(Params & params) : params (params)
{
	/*
	（构造函数 LocalSearch）:
	- 在构造时根据 params 的规模分配内部容器：clients, routes, depots, depotsEnd, bestInsertClient 等。
	- 初始化 clients 的索引与 depot 节点的元信息（isDepot/route 指向），并填充 orderNodes/orderRoutes 供 run 时打乱遍历使用。
	- 该构造函数不会填充具体路线，实际使用前需通过 loadIndividual 将解导入。
	*/
	clients = std::vector < Node >(params.nbClients + 1);
	routes = std::vector < Route >(params.nbVehicles);
	depots = std::vector < Node >(params.nbVehicles);
	depotsEnd = std::vector < Node >(params.nbVehicles);
	bestInsertClient = std::vector < std::vector <ThreeBestInsert> >(params.nbVehicles, std::vector <ThreeBestInsert>(params.nbClients + 1));

	for (int i = 0; i <= params.nbClients; i++) 
	{ 
		clients[i].cour = i; 
		clients[i].isDepot = false; 
	}
	for (int i = 0; i < params.nbVehicles; i++)
	{
		routes[i].cour = i;
		routes[i].depot = &depots[i];
		depots[i].cour = 0;
		depots[i].isDepot = true;
		depots[i].route = &routes[i];
		depotsEnd[i].cour = 0;
		depotsEnd[i].isDepot = true;
		depotsEnd[i].route = &routes[i];
	}
	//用来随机化遍历顺序
	for (int i = 1 ; i <= params.nbClients ; i++) orderNodes.push_back(i);
	for (int r = 0 ; r < params.nbVehicles ; r++) orderRoutes.push_back(r);
}

