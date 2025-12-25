package Improvement;

// 局部搜索类：负责执行整个解决方案的局部搜索优化，包括路径间的节点移动、交换和交叉操作
// TODO:遍历所有路径，对被修改过的路径进行搜索得到移动操作列表，遍历列表执行，执行移动后同时更新被改变路径的移动操作
// FILO是顶点级移动生成器，外层是选择某种算子，内层是遍历SVC中的顶点来查找要执行的移动生成器放入堆中再执行
// HGS像AILS，外层是遍历所有顶点和邻居，得到一对点，内层调用多种算子进行尝试，有改进则再看另一对点。区别是后者外层遍历更改了
// 的路径的所有顶点及对应邻居，应用多个算子（都应用一遍），得到移动列表，不断执行优化。

import java.util.Arrays;

import Data.Instance;
import Evaluators.CostEvaluation;
import Evaluators.FeasibilityEvaluation;
import Evaluators.ExecuteMovement;
import Evaluators.MovementType;
import SearchMethod.Config;
import Solution.Node;
import Solution.Route;
import Solution.Solution;


public class LocalSearch 
{
	private Route routes[]; // 当前解决方案中的所有路径数组
	private  CandidateNode improvedMoves[]; // 存储候选改进操作的数组
	private  CandidateNode improvedMatrix[][]; // 改进操作矩阵，用于快速查找路径对之间的最佳改进
	private CandidateNode improvedNode; // 当前评估的改进节点
	private int topBest=0; // 当前improvedMoves数组中有效候选操作的数量
	private int numRoutes; // 当前解决方案中的路径数量
	
	Node bestPrevNoRouteI,bestPrevNoRouteJ; // 在SwapStar操作中，用于记录两个路径中最佳插入位置的前驱节点

	double f=0; // 当前解决方案的总成本
	
	double cost; // 临时存储操作成本
	Node auxRouteI,auxRouteJ; // 辅助节点：auxRouteI表示从路径I操作的节点，auxRouteJ表示从路径J操作的节点
	Route routeA,routeB; // 临时存储涉及改进的两条路径
	CostEvaluation evaluateCost; // 成本评估器
	FeasibilityEvaluation feasibilityEvaluation; // 可行性评估器
	ExecuteMovement executeMovement; // 移动执行器
	Node solution[]; // 解决方案中所有节点的数组
	int limitAdj; // 邻域搜索限制参数，限制每个节点考虑的最近邻数量
	IntraLocalSearch intraLocalSearch; // 内部局部搜索对象，用于单路径内优化
	double epsilon; // 数值精度参数，用于判断成本改进是否显著
	int activeNodesCounter; // 计数器，用于统计涉及特定路径的活跃候选操作数量
	
	// 构造函数：初始化局部搜索所需组件
	public LocalSearch(Instance instance,Config config, IntraLocalSearch intraLocalSearch)
	{
		this.evaluateCost=new CostEvaluation(instance);
		this.feasibilityEvaluation=new FeasibilityEvaluation();
		this.executeMovement=new ExecuteMovement(instance);
		// 初始化改进操作数组，最大大小为所有路径对的数量
		this.improvedMoves=new CandidateNode[instance.getMaxNumberRoutes()*(instance.getMaxNumberRoutes()-1)/2];
		// 初始化改进矩阵，用于存储任意两条路径之间的最佳改进操作
		this.improvedMatrix=new CandidateNode[instance.getMaxNumberRoutes()][instance.getMaxNumberRoutes()];
		
		// 初始化改进矩阵，只初始化上三角部分，下三角引用相同的对象以节省空间
		for (int i = 0; i < improvedMatrix.length; i++)
		{
			for (int j = i+1; j < improvedMatrix.length; j++) 
			{
				improvedMatrix[i][j]=new CandidateNode(evaluateCost);
				improvedMatrix[j][i]=improvedMatrix[i][j]; // 下三角引用上三角的对象
			}
		}
		
		// 设置邻域搜索限制，取配置值和实例大小-1的较小值
		this.limitAdj=Math.min(config.getVarphi(), instance.getSize()-1);
		this.intraLocalSearch=intraLocalSearch;
		this.epsilon=config.getEpsilon(); // 设置数值精度参数
	}
	
	// 设置当前解决方案信息
	private void setSolution(Solution solution) 
	{
		this.numRoutes=solution.numRoutes; // 记录路径数量
		this.solution=solution.getSolution(); // 获取所有节点的数组
		this.f=solution.f; // 记录当前解决方案总成本
		this.routes=solution.routes; // 获取所有路径数组
	}

	// 将局部搜索后的结果赋值回解决方案对象
	private void assignResult(Solution solution) 
	{
		solution.numRoutes=this.numRoutes; // 更新路径数量
		solution.f=this.f; // 更新总成本
	}

	// 局部搜索主方法：执行整个解决方案的优化
	public void localSearch(Solution solution,boolean isRouteEmptyflag)
	{
		setSolution(solution); // 初始化解决方案信息
		topBest=0; // 重置候选改进操作计数器
			
		// 为每条路径计算累积需求，用于可行性检查
		for (int i = 0; i < numRoutes; i++) 
			routes[i].setAccumulatedDemand();
		
		// 遍历所有路径，对每条修改过的路径进行搜索
		for (int i = 0; i < numRoutes; i++) 
		{
			if(routes[i].numElements>1&&routes[i].modified) // 只处理有多个节点且被修改过的路径
				browseRoutes(routes[i]); // 搜索该路径的改进机会
		}
		
		// 如果找到候选改进操作，则执行它们
		if(topBest>0)
			execute();
		
		assignResult(solution); // 更新解决方案
		// 如果标志为真，移除空的路径
		if(isRouteEmptyflag)
			solution.removeEmptyRoutes();
	}
	
	//-----------------------------------使用最佳改进策略执行操作-------------------------------------------
	
	// 执行最佳改进操作循环：不断执行最佳改进操作直到没有改进
	private void execute() 
	{
		while(topBest>0) // 当还有候选改进操作时
		{
			// 对候选改进操作按成本进行排序（升序，成本越低越好）
			Arrays.sort(improvedMoves,0,topBest);
			routeA=improvedMoves[0].routeA; // 获取最佳改进涉及的第一条路径
			routeB=improvedMoves[0].routeB; // 获取最佳改进涉及的第二条路径
			
			// 执行最佳改进操作，并更新总成本
			f+=executeMovement.apply(improvedMoves[0]);

			// 对涉及的两条路径分别进行内部局部搜索优化
			intraLocalSearch(routeA);
			intraLocalSearch(routeB);
			
			// 重新计算两条路径的累积需求
			routeA.setAccumulatedDemand();
			routeB.setAccumulatedDemand();
			
			// 清理涉及这两条路径的所有候选改进操作，因为它们可能已不再有效
			activeNodesCounter=0;
			for (int k = 0; k < topBest; k++) 
			{
				if(improvedMoves[k].routeA==routeA||improvedMoves[k].routeB==routeA||improvedMoves[k].routeA==routeB||improvedMoves[k].routeB==routeB)
				{
					improvedMoves[k].clean(); // 标记为无效
					activeNodesCounter++; // 统计被清理的操作数量
				}
			}
	
			// 重新排序，将无效操作移到数组末尾
			Arrays.sort(improvedMoves,0,topBest);
			topBest-=activeNodesCounter; // 更新有效操作数量
			
			// 重新搜索两条路径的改进机会
			browseRoutes(routeA);
			browseRoutes(routeB);
		}
	}
	
	// 浏览一条路径的所有可能改进操作
	public void browseRoutes(Route route)
	{
		if(route.numElements>1) // 只有路径中有多个节点时才进行搜索
		{
			searchBestSHIFT(route); // 搜索最佳移位操作
			searchBestSwapStarKnn(route); // 搜索最佳SwapStar操作（基于K最近邻）
			searchBestCross(route); // 搜索最佳交叉操作
		}
	}
	
	// 搜索最佳SwapStar操作：交换两个路径中的节点并在各自路径中找到最佳插入位置
	private void searchBestSwapStarKnn(Route route)
	{
		auxRouteI=route.first.next; // 从路径的第一个客户节点开始（跳过仓库节点）
		do
		{
			if(auxRouteI.modified) // 只考虑修改过的节点
			{
				// 遍历当前节点的最近邻
				for (int j = 0; j < limitAdj; j++) 
				{
					// 确保最近邻节点存在且不在同一路径中
					if(auxRouteI.getKnn()[j]!=0&&solution[auxRouteI.getKnn()[j]-1].route!=route)
					{
						auxRouteJ=solution[auxRouteI.getKnn()[j]-1]; // 获取最近邻节点
						// 检查SwapStar操作的可行性（容量约束）
						if(feasibilityEvaluation.gainSWAP(auxRouteI, auxRouteJ)>=0)
						{
							// 在两个路径中分别寻找最佳插入位置（排除某些节点后）
							bestPrevNoRouteI=auxRouteJ.route.findBestPositionExceptAfterNodeKNN(auxRouteI,auxRouteJ,solution);
							bestPrevNoRouteJ=auxRouteI.route.findBestPositionExceptAfterNodeKNN(auxRouteJ,auxRouteI,solution);
							
							// 计算SwapStar操作的成本
							cost=evaluateCost.costSwapStar(auxRouteI,auxRouteJ,bestPrevNoRouteI,bestPrevNoRouteJ);
							// 获取这两条路径对应的改进矩阵条目
							improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
							
							// 如果新成本比之前记录的成本低且为负改进（成本降低）
							if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
							{
								// 如果该改进操作之前未激活，则添加到候选数组
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								// 更新改进节点信息
								improvedNode.setImprovedNode(cost, MovementType.SWAPStar, auxRouteI, auxRouteJ,bestPrevNoRouteI, bestPrevNoRouteJ, cost);
							}
						}
					}
				}
			}
			
			auxRouteI=auxRouteI.next; // 移动到下一个节点
		}
		while(auxRouteI!=route.first); // 遍历整个路径
	}
	
	// 搜索最佳移位操作：将一个节点从一个路径移动到另一个路径
	private void searchBestSHIFT(Route route)
	{
		auxRouteI=route.first.next; // 从路径的第一个客户节点开始
		do
		{
			if(auxRouteI.modified) // 只考虑修改过的节点
			{
				for (int j = 0; j < limitAdj; j++) 
				{
					// 处理最近邻索引为0的情况（表示仓库节点）
					if(auxRouteI.getKnn()[j]==0)
					{
						// 遍历所有其他路径，考虑将节点移到其他路径的起始位置
						for (int i = 0; i < numRoutes; i++) 
						{
							if(routes[i]!=route) // 排除当前路径
							{
								auxRouteJ=routes[i].first; // 目标路径的起始节点（仓库）
								// 检查移位操作的可行性
								if(feasibilityEvaluation.gainSHIFT(auxRouteI, auxRouteJ)>=0)
								{
									cost=evaluateCost.costSHIFT(auxRouteI,auxRouteJ); // 计算成本
									improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
									
									// 如果找到更好的改进
									if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
									{
										if(!improvedNode.active)
											improvedMoves[topBest++]=improvedNode;
										
										improvedNode.setImprovedNode(cost, MovementType.SHIFT, auxRouteI, auxRouteJ,cost);
									}
								}
							}
						}
					}
					// 处理最近邻节点不在同一路径的情况
					else if(solution[auxRouteI.getKnn()[j]-1].route!=route)
					{
						auxRouteJ=solution[auxRouteI.getKnn()[j]-1]; // 获取最近邻节点
						// 检查将auxRouteI移到auxRouteJ之后的可行性
						if(feasibilityEvaluation.gainSHIFT(auxRouteI, auxRouteJ)>=0)
						{
							cost=evaluateCost.costSHIFT(auxRouteI,auxRouteJ);
							improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
							
							if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.SHIFT, auxRouteI, auxRouteJ,cost);
							}
						}
						
						// 检查反向移位：将auxRouteJ移到auxRouteI之后
						if(feasibilityEvaluation.gainSHIFT(auxRouteJ, auxRouteI)>=0)
						{
							cost=evaluateCost.costSHIFT(auxRouteJ, auxRouteI);
							improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
							
							if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.SHIFT, auxRouteJ,auxRouteI, cost);
							}
						}
					}
				}
			}
			
			auxRouteI=auxRouteI.next; // 移动到下一个节点
		}
		while(auxRouteI!=route.first); // 遍历整个路径
	}
	
	// 搜索最佳交叉操作：交换两条路径中的边
	private void searchBestCross(Route route)
	{
		auxRouteI=route.first; // 从路径的起始节点（仓库）开始
		do
		{
			if(auxRouteI.modified) // 只考虑修改过的节点
			{
				for (int j = 0; j < limitAdj; j++) 
				{
					// 处理最近邻索引为0的情况（表示仓库节点）
					if(auxRouteI.getKnn()[j]==0)
					{
						// 遍历所有其他路径
						for (int i = 0; i < numRoutes; i++) 
						{
							if(routes[i]!=route) // 排除当前路径
							{
								auxRouteJ=routes[i].first; // 目标路径的起始节点（仓库）
								// 检查标准交叉操作的可行性
								if(feasibilityEvaluation.gainCross(auxRouteI, auxRouteJ)>=0)
								{
									cost=evaluateCost.costCross(auxRouteI,auxRouteJ);
									improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
									
									if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
									{
										if(!improvedNode.active)
											improvedMoves[topBest++]=improvedNode;
										
										improvedNode.setImprovedNode(cost, MovementType.Cross, auxRouteI, auxRouteJ,cost);
									}
								}
								
								// 检查反向交叉操作的可行性
								if(feasibilityEvaluation.gainCrossInverted(auxRouteI, auxRouteJ)>=0)
								{
									cost=evaluateCost.inversedCostCross(auxRouteI,auxRouteJ);
									improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
									
									if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
									{
										if(!improvedNode.active)
											improvedMoves[topBest++]=improvedNode;
										
										improvedNode.setImprovedNode(cost, MovementType.CrossInverted, auxRouteI, auxRouteJ,cost);
									}
								}
							}
						}
					}
					// 处理最近邻节点不在同一路径的情况
					else if(solution[auxRouteI.getKnn()[j]-1].route!=route)
					{
						auxRouteJ=solution[auxRouteI.getKnn()[j]-1]; // 获取最近邻节点
						// 检查标准交叉操作
						if(feasibilityEvaluation.gainCross(auxRouteI, auxRouteJ)>=0)
						{
							cost=evaluateCost.costCross(auxRouteI,auxRouteJ);
							improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
							
							if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.Cross, auxRouteI, auxRouteJ,cost);
							}
						}
						// 检查反向交叉操作
						if(feasibilityEvaluation.gainCrossInverted(auxRouteI, auxRouteJ)>=0)
						{
							cost=evaluateCost.inversedCostCross(auxRouteI,auxRouteJ);
							improvedNode=improvedMatrix[auxRouteI.route.nameRoute][auxRouteJ.route.nameRoute];
							
							if(((cost-improvedNode.cost)<-epsilon)&&((cost-0)<-epsilon))
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.CrossInverted, auxRouteI, auxRouteJ,cost);
							}
						}
					}
				}
			}
			
			auxRouteI=auxRouteI.next; // 移动到下一个节点
		}
		while(auxRouteI!=route.first); // 遍历整个路径
	}
	
	// 对单条路径执行内部局部搜索
	private void intraLocalSearch(Route route)
	{
		// 调用内部局部搜索方法，并更新总成本
		f+=intraLocalSearch.intraLocalSearch(route, solution);
	}
	
}
