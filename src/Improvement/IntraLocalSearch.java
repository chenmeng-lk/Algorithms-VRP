package Improvement;

// 内部局部搜索类：负责在单条路径内部进行节点操作优化，包括2-opt、移位和交换等邻域搜索操作
// 不停迭代，每轮遍历整条路径结点，找到最佳改进并执行，未找到改进则退出迭代
import Data.Instance;
import Evaluators.CostEvaluation;
import Evaluators.ExecuteMovement;
import Evaluators.MovementType;
import SearchMethod.Config;
import Solution.Node;
import Solution.Route;

public class IntraLocalSearch 
{
	private  CandidateNode improve; // 候选改进节点对象，用于存储找到的最佳改进操作
	Node first; // 路径起始节点
	int numElements=0; // 路径中节点数量
	MovementType moveType; // 移动类型枚举
	int iterator; // 迭代计数器
	double lowestCost; // 当前找到的最低成本改进值
	double prevF; // 改进前的路径总成本
	Node auxOut,auxIn; // 辅助节点：auxOut表示被移出/操作的节点，auxIn表示目标位置节点
	double cost; // 临时存储操作成本
	boolean changeFlag=false; // 标识是否找到改进
	CostEvaluation evaluateCost; // 成本评估器
	ExecuteMovement executeMovement; // 移动执行器
	int limitAdj; // 邻域搜索限制参数，限制每个节点考虑的最近邻数量
	
	// 构造函数：初始化局部搜索所需组件
	public IntraLocalSearch(Instance instance,Config config)
	{
		this.evaluateCost=new CostEvaluation(instance);
		this.executeMovement=new ExecuteMovement(instance);
		this.improve=new CandidateNode(evaluateCost);
		this.limitAdj=config.getVarphi(); // 获取配置中的邻域限制参数
	}
	
	// 设置当前处理的路径信息
	private void setRoute(Route route,Node solution[]) 
	{
		this.prevF=route.fRoute; // 记录改进前的路径成本
		this.first=route.first; // 记录路径起始节点
		this.numElements=route.numElements; // 记录路径节点数量
	}

	// 内部局部搜索主方法：在单条路径内部搜索改进机会并执行，返回成本改进值
	public double intraLocalSearch(Route route,Node solution[])
	{
		setRoute(route,solution); // 初始化路径信息
		
		iterator=0; // 迭代计数器清零
		changeFlag=true; // 初始标志设为true以进入循环
		while(changeFlag) // 当有改进发生时继续迭代
		{
			iterator++; // 迭代次数增加
			changeFlag=false; // 重置改进标志
			lowestCost=0; // 重置最低成本改进值
			auxOut=first; // 从路径起始节点开始遍历
			do
			{
				// 仅考虑标记为修改过的节点，减少搜索空间
				if(auxOut.modified)
				{
					// 遍历当前节点的有限个最近邻节点
					for (int j = 0; j < limitAdj; j++) 
					{
						// 获取第j个最近邻节点，索引为0时表示路径起始节点
						if(auxOut.getKnn()[j]==0)
							auxIn=first;
						else
							auxIn=solution[auxOut.getKnn()[j]-1];
						
						// 确保两个节点在同一路径中
						if(auxOut.route.nameRoute==auxIn.route.nameRoute)
						{
							//2Opt操作评估：反转节点间的路径段
							if(auxOut!=auxIn&&auxIn!=auxOut.next) // 避免无效操作
							{
								cost=evaluateCost.cost2Opt(auxOut,auxIn); // 计算2-opt操作成本
								if(lowestCost>cost) // 如果找到成本更低的改进
								{
									lowestCost=cost; // 更新最低成本
									moveType=MovementType.TwoOpt; // 记录移动类型
									improve.setImprovedNode(lowestCost, moveType, auxOut, auxIn,0,0,lowestCost); // 保存改进信息
									changeFlag=true; // 标记找到改进
								}
							}
							
							//SHIFT操作评估：将一个节点移动到另一节点之后
							if(numElements>2&&auxOut!=auxIn&&auxOut!=auxIn.next) // 路径至少3个节点且不是无效移动
							{
								cost=evaluateCost.costSHIFT(auxOut,auxIn); // 计算移位操作成本
								if(lowestCost>cost) // 如果找到成本更低的改进
								{
									lowestCost=cost;
									moveType=MovementType.SHIFT;
									improve.setImprovedNode(lowestCost, moveType, auxOut, auxIn,0,0,lowestCost);
									changeFlag=true;
								}
								
								// 尝试反向移位：将auxIn移动到auxOut之后
								if(auxIn!=auxOut.next) // 避免无效操作
								{
									cost=evaluateCost.costSHIFT(auxIn,auxOut);
									if(lowestCost>cost)
									{
										lowestCost=cost;
										moveType=MovementType.SHIFT;
										improve.setImprovedNode(lowestCost, moveType, auxIn, auxOut,0,0,lowestCost);
										changeFlag=true;
									}
								}
							}
							
							//SWAP操作评估：交换两个节点的位置
							if(numElements>2&&auxIn!=auxOut&&auxIn.next!=auxOut) // 路径至少3个节点且不是无效交换
							{
								cost=evaluateCost.costSWAP(auxOut,auxIn); // 计算交换操作成本
								if(lowestCost>cost) // 如果找到成本更低的改进
								{
									lowestCost=cost;
									moveType=MovementType.SWAP;
									improve.setImprovedNode(lowestCost, moveType, auxOut, auxIn,0,0,lowestCost);
									changeFlag=true;
								}
							}
						}
					}
				}
				
				auxOut=auxOut.next; // 移动到下一个节点
			}
			while(auxOut!=first); // 遍历整个路径直到回到起始节点
			
			// 如果本轮迭代找到改进，则执行最佳改进操作
			if(changeFlag)
				executeMovement.apply(improve);
		}
		
		// 返回路径的成本改进值（改进后成本 - 改进前成本）
		return route.fRoute-prevF;
	}
	
}
