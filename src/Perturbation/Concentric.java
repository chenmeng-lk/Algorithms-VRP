// 同心扰动类：通过移除一组相邻节点然后重新插入的方式破坏当前解，以帮助跳出局部最优
package Perturbation;

import java.util.HashMap;

import Data.Instance;
import DiversityControl.OmegaAdjustment;
import Improvement.IntraLocalSearch;
import SearchMethod.Config;
import Solution.Node;
import Solution.Solution;

//Concentric removal removes all nodes at once and then adds them into the route
public class Concentric extends Perturbation
{
	public Concentric(Instance instance, Config config,
	HashMap<String, OmegaAdjustment> omegaSetup, IntraLocalSearch intraLocalSearch)
	{
		super(instance, config, omegaSetup,intraLocalSearch);
		this.perturbationType=PerturbationType.Concentric; // 设置扰动类型为同心扰动
	}

	public void applyPerturbation(Solution s) // 应用同心扰动
	{
		setSolution(s); // 初始化当前解
		
//		---------------------------------------------------------------------
		// 选择随机参考节点
		Node reference=solution[rand.nextInt(solution.length)];
		// 移除参考节点的K近邻节点（数量由omega控制）
		for (int i = 0; i < omega&&i < reference.knn.length&&countCandidates<omega; i++) 
		{
			if(reference.knn[i]!=0)//邻居结点不是车场
			{
				node=solution[reference.knn[i]-1];
				candidates[countCandidates]=node; // 添加到候选列表
				countCandidates++;
				node.prevOld=node.prev; // 保存原前驱节点
				node.nextOld=node.next; // 保存原后继节点
				f+=node.route.remove(node); // 从路线中移除节点并更新成本
			}
		}
		
		setOrder(); // 随机打乱候选节点顺序
		
		addCandidates(); // 将候选节点重新插入到最佳位置
		
		assignSolution(s); // 将扰动重建后的解赋值回原解
	}
}
