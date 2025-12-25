package SearchMethod;

// 构造解类：负责构建初始解决方案，通过随机选择和启发式插入方法生成可行的车辆路径规划初始解
import java.util.Random;

import Data.Instance;
import Solution.Node;
import Solution.Route;
import Solution.Solution;


public class ConstructSolution 
{
	private Route routes[]; // 存储所有路径的数组
	private double f=0; // 当前解决方案的总成本
	private int numRoutes; // 当前使用的路径数量
	private Node []solution; // 所有节点的数组
	protected Random rand=new Random(); // 随机数生成器
	protected int size; // 客户节点数量（不包括仓库）
	Instance instance; // 问题实例数据
	Node notInserted[]; // 未插入路径的节点数组
	int countNotInserted=0; // 未插入节点计数器
	
	// 构造函数：初始化构造解决方案所需的数据结构
	public ConstructSolution(Instance instance,Config config)
	{
		this.instance=instance;
		this.routes=new Route[instance.getMaxNumberRoutes()]; // 根据最大路径数初始化路径数组
		this.size=instance.getSize()-1; // 客户节点数 = 总节点数 - 仓库节点
		this.notInserted=new Node[size]; // 初始化未插入节点数组
	}
	
	// 设置当前解决方案信息
	private void setSolution(Solution solution) 
	{
		this.numRoutes=solution.numRoutes; // 记录路径数量
		this.solution=solution.getSolution(); // 获取所有节点数组
		this.f=solution.f; // 记录当前总成本
		for (int i = 0; i < routes.length; i++) 
			this.routes[i]=solution.routes[i]; // 复制所有路径
	}

	// 将构造结果赋值回解决方案对象
	private void assignResult(Solution solution) 
	{
		solution.numRoutes=this.numRoutes; // 更新路径数量
		solution.f=this.f; // 更新总成本
		for (int i = 0; i < routes.length; i++) 
			solution.routes[i]=this.routes[i]; // 更新所有路径
	}

	// 初始解构造方法：构建一个初始可行的解决方案
	//TODO:初始解构造，每条路径随机插入一个初始节点，再遍历所有路径的所有位置找到最佳插入位置插入剩余的节点
	public void construct(Solution s)//初始解构造函数
	{
		setSolution(s); // 初始化解决方案信息
		
		// 清空所有路径
		for (int i = 0; i < routes.length; i++)//清空路径数组
			routes[i].clean();
		
		int index;
		Node node,bestNode;
		f=0; // 重置总成本
		countNotInserted=0; // 重置未插入节点计数器
		
		// 将所有节点初始化为未插入状态
		for (int i = 0; i < size; i++) //把每个结点加入未插入数组
			notInserted[countNotInserted++]=solution[i];
		
		// 为每条路径随机选择一个初始节点（确保每条路径至少有一个节点）
		for (int i = 0; i < numRoutes; i++)
		{
			index=rand.nextInt(countNotInserted); // 随机选择一个未插入节点
			f+=routes[i].addNodeEndRoute(notInserted[index]); // 将节点添加到路径末尾，并更新成本
			
			// 从未插入数组中移除已插入的节点（通过与最后一个元素交换）
			node=notInserted[index];
			notInserted[index]=notInserted[countNotInserted-1];
			notInserted[--countNotInserted]=node;
		}
		
		// 使用启发式方法插入剩余的节点
		while(countNotInserted>0) 
		{
			index=rand.nextInt(countNotInserted); // 随机选择一个未插入节点
			node=notInserted[index];
			// 为当前节点找到最佳插入位置（在所有路径中）
			bestNode=getBestNoRoutes(node);
			// 将节点插入到最佳位置，并更新成本
			f+=bestNode.route.addAfter(node, bestNode);//TODO:总成本更新是在每次执行插入等操作时进行的
			// 从未插入数组中移除已插入的节点
			notInserted[index]=notInserted[countNotInserted-1];
			notInserted[--countNotInserted]=node;
		}
		
		assignResult(s); // 将构造结果赋回原解决方案
		s.removeEmptyRoutes(); // 移除空路径
	}
	
	// 基于K最近邻的启发式插入：为节点找到最佳插入位置（在已插入的节点中）
	protected Node getBestKNNNo(Node no)
	{
		double bestCost=Double.MAX_VALUE;
		Node aux,bestNode=null;
		double cost,costPrev;
		
		// 遍历所有已插入的节点，计算插入成本
		for (int i = 0; i < solution.length; i++) 
		{
			aux=solution[i];
			if(aux.nodeBelong) // 只考虑已插入路径的节点
			{
				// 计算插入到aux和aux.next之间的成本增加
				cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name);
				if(cost<bestCost) // 找到成本更低的插入位置
				{
					bestCost=cost;
					bestNode=aux;
				}
			}
		}
		// 计算插入到bestNode之后的成本
		cost=instance.dist(bestNode.name,no.name)+instance.dist(no.name,bestNode.next.name)-instance.dist(bestNode.name,bestNode.next.name);
		// 计算插入到bestNode之前的成本
		costPrev=instance.dist(bestNode.prev.name,no.name)+instance.dist(no.name,bestNode.name)-instance.dist(bestNode.prev.name,bestNode.name);
		// 返回成本更小的插入位置
		if(cost<costPrev)
			return bestNode;
		
		return bestNode.prev;
	}
	
	// 为节点在所有路径中寻找最佳插入位置
	protected Node getBestNoRoutes(Node no)
	{
		double bestCost=Double.MAX_VALUE;
		Node aux,bestNode=null;
		
		// 遍历所有路径，在每个路径中寻找最佳插入位置
		for (int i = 0; i < numRoutes; i++) 
		{
			// 在当前路径中寻找最佳插入位置
			aux=routes[i].findBestPosition(no);//遍历路径每个顶点
			// 如果当前路径的插入成本更低，则更新最佳插入位置
			if(routes[i].lowestCost<bestCost)
			{
				bestCost=routes[i].lowestCost;
				bestNode=aux;
			}
		}
		
		return bestNode;
	}
	
}
