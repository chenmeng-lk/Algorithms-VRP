// 顺序扰动类：通过移除连续节点然后重新插入的方式破坏当前解，以帮助跳出局部最优
package Perturbation;

import java.util.HashMap;

import Data.Instance;
import DiversityControl.OmegaAdjustment;
import Improvement.IntraLocalSearch;
import SearchMethod.Config;
import Solution.Node;
import Solution.Solution;

//Sequential removal of all at once to add the nodes
public class Sequential extends Perturbation
{
	
	public Sequential(Instance instance, Config config, 
	HashMap<String, OmegaAdjustment> omegaSetup, IntraLocalSearch intraLocalSearch)
	{
		super(instance, config,omegaSetup,intraLocalSearch);
		this.perturbationType=PerturbationType.Sequential; // 设置扰动类型为顺序扰动
	}

	public void applyPerturbation(Solution s)//对解应用顺序扰动
	{
		setSolution(s); // 初始化当前解
		
//		---------------------------------------------------------------------
		int contSizeString; // 当前已移除的连续节点计数
		double sizeString; // 每次连续移除的节点数
		Node initialNode; // 起始节点
		
		// 在路线中循环直到移除足够数量的节点（达到omega指定的数量）
		while(countCandidates<(int)omega)
		{
			// 计算本次连续移除的节点数，不超过剩余需要移除的数量
			sizeString=Math.min(Math.max(1, size),(int)omega-countCandidates);
			
			// 随机选择一个属于当前解的节点作为起始节点
			node=solution[rand.nextInt(size)];
			while(!node.nodeBelong)
				node=solution[rand.nextInt(size)];
			
			initialNode=node; // 记录起始节点
			
			contSizeString=0; // 重置计数器
			do
			{
				contSizeString++; // 增加计数器
				node=initialNode.next; // 移动到下一个节点
				if(node.name==0) // 如果遇到depot节点（name为0），跳过
					node=node.next;
				
				candidates[countCandidates++]=node; // 添加到候选列表
				
				node.prevOld=node.prev; // 保存原前驱节点
				node.nextOld=node.next; // 保存原后继节点
				
				f+=node.route.remove(node); // 从路线中移除节点并更新成本
			}
			while(initialNode.name!=node.name&&contSizeString<sizeString); // 直到回到起始节点或达到本次移除数量
		}
		
		setOrder(); // 随机打乱候选节点顺序
		
		addCandidates(); // 将候选节点重新插入到最佳位置
		
		assignSolution(s); // 将扰动重建后的解赋值回原解
	}

}
