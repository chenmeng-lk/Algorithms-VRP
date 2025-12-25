// 可行性修复阶段：通过节点移动操作（如SHIFT、SWAP、CROSS等）将不可行解转换为可行解
package Improvement;

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

public class FeasibilityPhase 
{
	private Route routes[]; // 路线数组
	private  CandidateNode improvedMoves[]; // 改进的移动操作候选数组
	private  CandidateNode improvedMatrix[][]; // 改进移动操作的矩阵，用于快速查找

	private CandidateNode improvedNode; // 当前改进的节点

	private int topBest=0; // 最佳改进移动的数量
	private int numRoutes; // 路线数量
	
	Node bestPrevNoRouteI,bestPrevNoRouteJ; // 用于SWAP操作的最佳前驱节点

	double f=0; // 解的总成本
	
	Node auxOut,auxIn; // 用于交换的两个节点
	int gain; // 移动操作的收益
	double cost; // 移动操作的成本
	double evaluationCost; // 评估成本
	Route routeA,routeB; // 两条相关路线
	
	CostEvaluation evaluateCost; // 成本评估器
	FeasibilityEvaluation feasibilityEvaluation; // 可行性评估器
	ExecuteMovement executeMovement; // 移动执行器
	Node solution[]; // 解中的节点数组
	int limitAdj; // 相邻节点的限制数量
	IntraLocalSearch intraLocalSearch; // 内部本地搜索
	double epsilon; // 容差值
	int activeNodesCounter; // 活动节点计数器
	
	public FeasibilityPhase(Instance instance,Config config, IntraLocalSearch intraLocalSearch)
	{
		this.evaluateCost=new CostEvaluation(instance);
		this.feasibilityEvaluation=new FeasibilityEvaluation();
		this.executeMovement=new ExecuteMovement(instance);
		this.improvedMoves=new CandidateNode[instance.getMaxNumberRoutes()*(instance.getMaxNumberRoutes()-1)/2];
		this.improvedMatrix=new CandidateNode[instance.getMaxNumberRoutes()][instance.getMaxNumberRoutes()];
		
		// 初始化改进矩阵
		for (int i = 0; i < improvedMatrix.length; i++)
		{
			for (int j = 0; j < improvedMatrix.length; j++) 
			{
				improvedMatrix[i][j]=new CandidateNode(evaluateCost);
				improvedMatrix[j][i]=improvedMatrix[i][j];
			}
		}
		
		this.limitAdj=Math.min(config.getVarphi(), instance.getSize()-1);
		
		this.intraLocalSearch=intraLocalSearch;
		this.epsilon=config.getEpsilon();
	}
	
	private boolean feasible() //检查解的可行性（有路线容量不足）
	{
		for (int i = 0; i < numRoutes; i++)
		{
			if(routes[i].availableCapacity()<0)
				return false;
		}
		return true;
	}
	
	private void setSolution(Solution solution) //设置解等于给定解
	{
		this.numRoutes=solution.numRoutes;
		this.solution=solution.getSolution();
		this.f=solution.f;
		this.routes=solution.routes;
	}

	private void assignResult(Solution solution) //将结果赋值回解
	{
		solution.numRoutes=this.numRoutes;
		solution.f=this.f;
	}

	public void makeFeasible(Solution solution)//可行化解,遍历所有修改过的路线，寻找改进的移动并执行
	{
		setSolution(solution);//设置解等于给定解
		boolean feasible=false;
		
		do
		{
			topBest=0;
			// 计算每条路线的累计需求
			for (int i = 0; i < numRoutes; i++) 
				routes[i].setAccumulatedDemand();
			
			// 遍历所有修改过的路线，寻找改进的移动
			for (int i = 0; i < numRoutes; i++) 
			{
				if(routes[i].modified)
					browseRoutes(routes[i]); 
			}
			
			// 如果有改进的移动，则执行
			if(topBest>0)
				execute();

			// 检查是否可行
			if(feasible())
			{
				// 执行内部本地搜索
				intraLocalSearch();//在所有路径内部搜索改进机会，每条路找到最佳移动并执行，返回成本改进值
				
				assignResult(solution);
				solution.removeEmptyRoutes();//移除空路线
				feasible=true;
			}
			else
			{
				// 如果不可行，增加一条新路线
				numRoutes++;
				routes[numRoutes-1].clean();
			}
		}
		while(!feasible);
		
	}
	
	//-----------------------------------Factibilizando com o Best Improviment-------------------------------------------
	
	private void execute() // 执行最佳改进移动
	{
		while(topBest>0)//可移动操作数量
		{
			// 按评估成本排序改进移动
			Arrays.sort(improvedMoves,0,topBest);
			
			routeA=improvedMoves[0].routeA;
			routeB=improvedMoves[0].routeB;
			// 应用最佳移动并更新总成本
			f+=executeMovement.apply(improvedMoves[0]);
			
			// 对两条相关路线进行内部本地搜索
			intraLocalSearch(routeA);
			intraLocalSearch(routeB);
			
			// 重新计算累计需求
			routeA.setAccumulatedDemand();
			routeB.setAccumulatedDemand();
			
			// 清除与已移动路线相关的改进移动
			activeNodesCounter=0;
			for (int k = 0; k < topBest; k++) 
			{
				if(improvedMoves[k].routeA==routeA||improvedMoves[k].routeA==routeB||improvedMoves[k].routeB==routeA||improvedMoves[k].routeB==routeB)
				{
					improvedMoves[k].clean();
					activeNodesCounter++;
				}
			}
	
			Arrays.sort(improvedMoves,0,topBest);
			topBest-=activeNodesCounter;
			
			// 重新浏览两条相关路线，寻找新的改进移动
			browseRoutes(routeA);
			browseRoutes(routeB);
		}
	}
	
	public void browseRoutes(Route route) // 浏览路线，寻找各种改进移动
	{
		if(route.getNumElements()>1)
		{
			searchBestSHIFT(route);
			searchBestSwapStarKnn(route);
			searchBestCross(route);
		}
	}
	
	public void calculateCost() // 计算评估成本，每单位容量节省的成本变化=成本变化值除以容量节省量
	{
		if(cost>=0)//当成本增加时额外加1以避免零除
			evaluationCost=((double)cost+1)/gain;
		else
			evaluationCost=(double)cost/gain;
	}
	
	private void searchBestSwapStarKnn(Route route) // 搜索最佳的SWAP*移动（基于K近邻）
	{
		auxOut=route.first.next;
		do
		{
			if(auxOut.modified)
			{
				// 遍历K近邻
				for (int j = 0; j < limitAdj; j++) 
				{
					// 只考虑一条路线可行而另一条不可行的情况
					if(auxOut.getKnn()[j]!=0&&auxOut.route.isFeasible()^solution[auxOut.getKnn()[j]-1].route.isFeasible())
					{
						auxIn=solution[auxOut.getKnn()[j]-1];
						gain=feasibilityEvaluation.gainSWAP(auxOut, auxIn);//计算swap移动的可行性改善程度，大于0可行
						if(gain>0)//不违反容量限制
						{
							// 寻找最佳插入位置
							bestPrevNoRouteI=auxIn.route.findBestPositionExceptAfterNodeKNN(auxOut,auxIn,solution);
							bestPrevNoRouteJ=auxOut.route.findBestPositionExceptAfterNodeKNN(auxIn,auxOut,solution);
							
							cost=evaluateCost.costSwapStar(auxOut,auxIn,bestPrevNoRouteI,bestPrevNoRouteJ);//移动成本变化，没考虑容量变化
							calculateCost();// 计算评估成本，每单位容量节省的成本变化
							
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							// 如果评估成本更好，则更新改进节点
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.SWAPStar, auxOut, auxIn,bestPrevNoRouteI, bestPrevNoRouteJ, evaluationCost, gain);
							}
						}
					}
				}
			}
			auxOut=auxOut.next;
		}
		while(auxOut!=route.first);//遍历路线一遍
	}
	
	private void searchBestSHIFT(Route route) // 搜索最佳的SHIFT移动
	{
		auxOut=route.first.next;
		do
		{
			if(auxOut.modified)
			{
				// 遍历所有路线
				for (int i = 0; i < numRoutes; i++) 
				{
					// 顶点auxOut从不可行路线移动到可行路线
					if(!auxOut.route.isFeasible()&&routes[i].isFeasible())
					{
						auxIn=routes[i].first;
						gain=feasibilityEvaluation.gainSHIFT(auxOut, auxIn);
						if(gain>0)
						{
							cost=evaluateCost.costSHIFT(auxOut, auxIn);
							calculateCost();
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.SHIFT, auxOut, auxIn,evaluationCost,gain);
							}
						}
					}
				}
				
				// 遍历K近邻
				for (int j = 0; j < limitAdj; j++) 
				{
					// 从不可行路线移动到可行路线（通过K近邻）
					if(auxOut.getKnn()[j]!=0&&!auxOut.route.isFeasible()&&solution[auxOut.getKnn()[j]-1].route.isFeasible())
					{
						auxIn=solution[auxOut.getKnn()[j]-1];
						gain=feasibilityEvaluation.gainSHIFT(auxOut, auxIn);
						if(gain>0)
						{
							cost=evaluateCost.costSHIFT(auxOut, auxIn);
							calculateCost();
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.SHIFT, auxOut, auxIn,evaluationCost,gain);
							}
						}
					}
					
					// 从可行路线移动到不可行路线（通过K近邻）
					if(auxOut.getKnn()[j]!=0&&auxOut.route.isFeasible()&&!solution[auxOut.getKnn()[j]-1].route.isFeasible())
					{
						auxIn=solution[auxOut.getKnn()[j]-1];
						gain=feasibilityEvaluation.gainSHIFT(auxIn, auxOut);
						if(gain>0)
						{
							cost=evaluateCost.costSHIFT(auxIn, auxOut);
							calculateCost();
							
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.SHIFT, auxIn ,auxOut ,evaluationCost,gain);
							}
						}
					}
				}
			}
			auxOut=auxOut.next;
		}
		while(auxOut!=route.first);
	}
	
	
	private void searchBestCross(Route route) // 搜索最佳的CROSS移动
	{
		auxOut=route.first;
		do
		{
			if(auxOut.modified)
			{
				// 遍历所有路线
				for (int i = 0; i < numRoutes; i++) 
				{
					// 只考虑一条路线可行而另一条不可行的情况
					if(auxOut.route.isFeasible()^routes[i].isFeasible())
					{
						auxIn=routes[i].first;
						gain=feasibilityEvaluation.gainCross(auxOut, auxIn);
						if(gain>0)
						{
							cost=evaluateCost.costCross(auxOut, auxIn);
							calculateCost();
							
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.Cross, auxOut, auxIn,evaluationCost,gain);
							}
							
						}
						gain=feasibilityEvaluation.gainCrossInverted(auxOut, auxIn);
						if(gain>0)
						{
							cost=evaluateCost.inversedCostCross(auxOut, auxIn);
							calculateCost();
							
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.CrossInverted, auxOut, auxIn,evaluationCost,gain);
							}
						}
					}
				}
				
				// 遍历K近邻
				for (int j = 0; j < limitAdj; j++) 
				{
					if(auxOut.getKnn()[j]!=0&&auxOut.route.isFeasible()^solution[auxOut.getKnn()[j]-1].route.isFeasible())
					{
						auxIn=solution[auxOut.getKnn()[j]-1];
						gain=feasibilityEvaluation.gainCross(auxOut, auxIn);
						if(gain>0)
						{
							cost=evaluateCost.costCross(auxOut, auxIn);
							calculateCost();
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.Cross, auxOut, auxIn,evaluationCost,gain);
							}
						}
						gain=feasibilityEvaluation.gainCrossInverted(auxOut, auxIn);
						if(gain>0)
						{
							cost=evaluateCost.inversedCostCross(auxOut, auxIn);
							calculateCost();
							improvedNode=improvedMatrix[auxOut.route.nameRoute][auxIn.route.nameRoute];
							
							if(evaluationCost<improvedNode.evaluationCost)
							{
								if(!improvedNode.active)
									improvedMoves[topBest++]=improvedNode;
								
								improvedNode.setImprovedNode(cost, MovementType.CrossInverted, auxOut, auxIn,evaluationCost,gain);
							}
						}
					}
				}
			}
			auxOut=auxOut.next;
		}
		while(auxOut!=route.first);
	}
	
	private void intraLocalSearch(Route route) // 对单条路线进行内部本地搜索
	{
		f+=intraLocalSearch.intraLocalSearch(route, solution);
	}
	
	private void intraLocalSearch() // 对所有修改过的路线进行内部本地搜索
	{
		for (int i = 0; i < numRoutes; i++)
		{
			if(routes[i].modified)
				f+=intraLocalSearch.intraLocalSearch(routes[i], solution);
		}
	}
	
}
