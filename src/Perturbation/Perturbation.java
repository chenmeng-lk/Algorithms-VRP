// 扰动抽象类：定义扰动操作的通用接口和基本功能，用于在局部搜索中破坏当前解以跳出局部最优
package Perturbation;

import java.util.HashMap;
import java.util.Random;

import Data.Instance;
import DiversityControl.OmegaAdjustment;
import Improvement.IntraLocalSearch;
import SearchMethod.Config;
import Solution.Node;
import Solution.Route;
import Solution.Solution;

public abstract class Perturbation 
{
	protected Route routes[]; // 路线数组
	protected int numRoutes; // 路线数量
	protected Node solution[]; // 解中的节点数组
	protected double f=0; // 解的总成本
	protected Random rand=new Random(); // 随机数生成器
	public double omega; // 扰动强度参数
	OmegaAdjustment chosenOmega; // Omega调整器
	Config config; // 配置对象
	protected Node candidates[]; // 候选节点数组
	protected int countCandidates; // 候选节点数量

	InsertionHeuristic[]insertionHeuristics; // 插入启发式方法数组
	public InsertionHeuristic selectedInsertionHeuristic; // 当前选中的插入启发式方法
	
	Node node; // 当前节点
	
	public PerturbationType perturbationType; // 扰动类型
	int size; // 问题规模（节点数-1）
	HashMap<String, OmegaAdjustment> omegaSetup; // Omega配置映射
	
	double bestCost,bestDist; // 最佳成本和最佳距离
	int numIterUpdate; // 更新迭代次数
	int indexHeuristic; // 启发式方法索引
	
	double cost,dist; // 当前成本和距离
	double costPrev; // 前一个成本
	int indexA,indexB; // 随机索引
	Node bestNode,aux; // 最佳节点和辅助节点
	Instance instance; // 问题实例
	int limitAdj; // 相邻节点限制数
	
	IntraLocalSearch intraLocalSearch; // 内部本地搜索器
	
	public Perturbation(Instance instance,Config config,
	HashMap<String, OmegaAdjustment> omegaSetup, IntraLocalSearch intraLocalSearch) 
	{
		this.config=config;
		this.instance=instance;
		this.insertionHeuristics=config.getInsertionHeuristics();
		this.size=instance.getSize()-1;
		this.candidates=new Node[size];
		this.omegaSetup=omegaSetup;
		this.numIterUpdate=config.getGamma();
		this.limitAdj=config.getVarphi();
		this.intraLocalSearch=intraLocalSearch;
	}
	
	public void setOrder() // 随机打乱候选节点顺序
	{
		Node aux;
		for (int i = 0; i < countCandidates; i++)//每次循环随机挑选2个节点交换位置
		{
			indexA=rand.nextInt(countCandidates);
			indexB=rand.nextInt(countCandidates);
			
			aux=candidates[indexA];
			candidates[indexA]=candidates[indexB];
			candidates[indexB]=aux;
		}
	}
	//TODO:扰动分两种，同心扰动、顺序扰动
	public void applyPerturbation(Solution s){} // 应用扰动（抽象方法，由子类实现）
	
	protected void setSolution(Solution s) // 设置当前解
	{
		this.numRoutes=s.getNumRoutes();
		this.routes=s.routes;
		this.solution=s.getSolution();
		this.f=s.f;
		// 重置所有路线的修改标记
		for (int i = 0; i < numRoutes; i++) 
		{
			routes[i].modified=false;
			routes[i].first.modified=false;
		}
		
		// 重置所有节点的修改标记
		for (int i = 0; i < size; i++) 
			solution[i].modified=false;
	
		// 随机选择一个插入启发式方法
		indexHeuristic=rand.nextInt(insertionHeuristics.length);
		selectedInsertionHeuristic=insertionHeuristics[indexHeuristic];
		
		// 获取当前扰动类型的Omega调整器
		chosenOmega=omegaSetup.get(perturbationType+"");
		omega=chosenOmega.getActualOmega();
		omega=Math.min(omega, size); // Omega不超过问题规模
		
		countCandidates=0; // 重置候选节点计数
	}
	
	protected void assignSolution(Solution s) // 将结果赋值回解
	{
		s.f=f;
		s.numRoutes=this.numRoutes;
	}
	//TODO:随机选择两种插入策略之一：只找最近邻、遍历邻居列表（不在路线中的所有位置找，只看插入到邻居后）
	protected Node getNode(Node no) // 根据选定的启发式方法获取最佳插入位置
	{
		// 1. Distance策略：基于距离的最近邻插入
		//    - 只考虑节点的最近邻（limit=1）
		//    - 计算将节点插入到最近邻位置的成本，选择最小成本位置
		//    - 优点：计算速度快，适合快速扰动
		//    - 缺点：可能错过更优的插入位置
		//
		// 2. Cost策略：基于成本的K近邻插入  
		//    - 考虑节点的多个近邻（limit=limitAdj，通常>1）
		//    - 遍历多个候选位置，选择全局最小成本位置
		//    - 优点：能找到更优的插入位置，提高解质量
		//    - 缺点：计算量较大，耗时更多
		//
		// 两种策略都使用K近邻数据结构加速搜索
		// 核心思想：在移除节点后，找到使总成本增加最小的插入位置
		switch(selectedInsertionHeuristic)
		{
			case Distance: return getBestKNNNo2(no,1); // 基于距离的最近邻
			case Cost: return getBestKNNNo2(no,limitAdj); // 基于成本的K近邻
		}
		return null;
	}
	
	protected Node getBestKNNNo2(Node no,int limit) // 获取最佳K近邻节点作为插入位置
	{
		bestCost=Double.MAX_VALUE;
		boolean flag=false;
		bestNode=null;
		
		int count=0;
		flag=false;
		// 遍历K近邻列表，最多检查limit个有效近邻
		for (int i = 0; i < no.knn.length&&count<limit; i++) 
		{
			if(no.knn[i]==0) // 如果近邻为0，表示需要从所有路线中查找
			{
				for (int j = 0; j < numRoutes; j++) // 遍历所有路线的第一个节点作为候选插入位置
				{
					aux=routes[j].first;
					flag=true;
					// 计算插入成本：将节点no插入到aux之后
					cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name);
					if(cost<bestCost)
					{
						bestCost=cost;
						bestNode=aux;
					}
				}
				if(flag)
					count++;
			}
			else // 使用预计算的K近邻
			{
				aux=solution[no.knn[i]-1];
				if(aux.nodeBelong) // 检查节点是否属于当前解
				{
					count++;
					cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name);
					if(cost<bestCost)
					{
						bestCost=cost;
						bestNode=aux;
					}
				}
			}
		}
		
		// 如果未找到最佳节点（所有近邻都不属于当前解），从整个解中搜索
		if(bestNode==null)
		{
			for (int i = 0; i < solution.length; i++) 
			{
				aux=solution[i];
				if(aux.nodeBelong)
				{
					cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name);
					if(cost<bestCost)
					{
						bestCost=cost;
						bestNode=aux;
					}
				}
			}
		}
		
		// 再次检查（冗余代码，保险）
		if(bestNode==null)
		{
			for (int i = 0; i < solution.length; i++) 
			{
				aux=solution[i];
				if(aux.nodeBelong)
				{
					cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name);
					if(cost<bestCost)
					{
						bestCost=cost;
						bestNode=aux;
					}
				}
			}
		}
		
		// 确定在最佳节点的前面还是后面插入
		cost=instance.dist(bestNode.name,no.name)+instance.dist(no.name,bestNode.next.name)-instance.dist(bestNode.name,bestNode.next.name);
		costPrev=instance.dist(bestNode.prev.name,no.name)+instance.dist(no.name,bestNode.name)-instance.dist(bestNode.prev.name,bestNode.name);
		if(cost<costPrev)
		{
			return bestNode; // 在bestNode之后插入
		}
		else
		{
			return bestNode.prev; // 在bestNode之前插入
		}
	}
	
	public void addCandidates() // 将所有候选节点插入到最佳位置
	{
		for (int i = 0; i < countCandidates; i++) 
		{
			node=candidates[i];
			bestNode=getNode(node);// 根据选定的启发式方法获取最佳插入位置
			
			// 将节点插入到最佳位置并更新总成本
			f+=bestNode.route.addAfter(node, bestNode);
		}
	}
	
	public int getIndexHeuristic() {
		return indexHeuristic;
	}

	public OmegaAdjustment getChosenOmega() {
		return chosenOmega;
	}
	
	public PerturbationType getPerturbationType() {
		return perturbationType;
	}
	
}