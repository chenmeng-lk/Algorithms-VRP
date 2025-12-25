package Solution;

import Data.Instance;
import SearchMethod.Config;

public class Route implements Comparable<Route>
{
	public int  capacity; // 路线容量
	public int depot; // 仓库节点名称

	public Node first; // 路线中的第一个节点
	public boolean modified; // 路线是否被修改
	public int totalDemand=0; // 路线总需求量
	public double fRoute=0,prevF; // 路线成本和之前的成本
	double increaseObjFunction; // 目标函数增加量
	public int numElements=0; // 路线中元素数量
	public int nameRoute; // 路线名称
	Node prev,next; // 临时节点指针
	
	//busca local best
	public double lowestCost=0; // 最低成本
	Node aux; // 辅助节点
	
	//------------------------------------------------
	public double cost,costRemoval; // 成本和移除成本
	public Node bestCost; // 最佳成本节点
	public double distance,lowestDist; // 距离和最低距离
	//------------------------------------------------
	public boolean update; // 是否更新
	
    Instance instance; // 实例对象
    int limitAdj; // 邻接限制
    
	public Route(Instance instance, Config config, Node depot,int nameRoute) // 构造函数，初始化路线
	{
		this.instance=instance; // 设置实例
		this.capacity = instance.getCapacity(); // 获取容量
		this.depot=depot.name; // 设置仓库名称
		this.nameRoute=nameRoute; // 设置路线名称
		this.first=null; // 初始化第一个节点为null
		this.limitAdj=config.getVarphi(); // 获取邻接限制
		addNodeEndRoute(depot.clone()); // 添加仓库节点到路线末尾
	}
	
    public Node findBestPosition(Node no) // 找到节点的最佳插入位置
	{
		bestCost=first; // 初始化最佳位置为第一个节点
		aux=first.next; // 辅助节点从下一个开始
		lowestCost=instance.dist(first.name,no.name)+instance.dist(no.name,first.next.name)-instance.dist(first.name,first.next.name); // 计算初始最低成本
		do
		{
			cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name); // 计算当前成本
			if(cost<lowestCost) // 如果更低
			{
				lowestCost=cost; // 更新最低成本
				bestCost=aux; // 更新最佳位置
			}
			aux=aux.next; // 移动到下一个
		}
		while(aux!=first); // 直到回到第一个
		return bestCost; // 返回最佳位置
	}
    
    public Node findBestPositionExceptAfterNode(Node no,Node exception) // 找到最佳位置，除了指定节点后
  	{
  		bestCost=first; // 初始化最佳位置
  		aux=first.next; // 辅助节点
  		
  		lowestCost=instance.dist(first.name,no.name)+instance.dist(no.name,first.next.name)-instance.dist(first.name,first.next.name); // 初始最低成本
  		do
  		{
  			cost=instance.dist(aux.name,no.name)+instance.dist(no.name,aux.next.name)-instance.dist(aux.name,aux.next.name); // 计算成本
  			if(cost<lowestCost&&aux.name!=exception.name) // 如果更低且不是例外
  			{
  				lowestCost=cost; // 更新
  				bestCost=aux; // 更新
  			}
  			aux=aux.next; // 下一个
  		}
  		while(aux!=first); // 循环
  		return bestCost; // 返回
  	}
    
    public Node findBestPositionExceptAfterNodeKNN(Node node,Node exception, Node solution[]) // 使用KNN找到最佳位置
   	{
    	lowestCost=Double.MAX_VALUE; // 初始化最低成本为最大值
    	
   		bestCost=first; // 初始化最佳位置
   		
   		if(numElements>limitAdj) // 如果元素数量超过限制
   		{
   			for (int j = 0; j < limitAdj; j++) // 遍历KNN
   			{
   	   			if(node.getKnn()[j]==0) // 如果是仓库
   	   			{
   	   				aux=first; // 辅助为第一个
   	   				cost=instance.dist(aux.name,node.name)+instance.dist(node.name,aux.next.name)-instance.dist(aux.name,aux.next.name); // 计算成本
   	   				if(cost<lowestCost) // 如果更低
   	   				{
   	   					lowestCost=cost; // 更新
   	   					bestCost=aux; // 更新
   	   				}   				
   	   			}
   	   			else if(node.getKnn()[j]!=exception.name&&solution[node.getKnn()[j]-1].route==this) // 如果不是例外且在同一路线
   	   			{
   	   				aux=solution[node.getKnn()[j]-1]; // 获取KNN节点
   	   				cost=instance.dist(aux.name,node.name)+instance.dist(node.name,aux.next.name)-instance.dist(aux.name,aux.next.name); // 计算成本
   	   				if(cost<lowestCost) // 如果更低
   	   				{
   	   					lowestCost=cost; // 更新
   	   					bestCost=aux; // 更新
   	   				}
   	   			}
   	   			
   			}
   		}
   		else // 否则使用普通方法
   		{
   			findBestPositionExceptAfterNode(node,exception); // 调用普通方法
   		}
   		
   		return bestCost; // 返回最佳位置
   	}
    
	public void clean() // 清理路线
	{
		first.prev=first; // 设置前节点为自身
		first.next=first; // 设置后节点为自身
		
		totalDemand=0; // 重置总需求
		fRoute=0; // 重置路线成本
		numElements=1; // 设置元素数量为1
		modified=true; // 标记为修改
	
	public void setAccumulatedDemand()//设置累计需求（前缀和）
	{
		first.accumulatedDemand=0;
		aux=first.next;
		do
		{
			aux.accumulatedDemand=aux.prev.accumulatedDemand+aux.demand;
			aux=aux.next;
		}
		while(aux!=first);
	}
	
	public void invertRoute() // 反转路线
	{
		aux=first; // 辅助节点为第一个
		Node prev=first.prev; // 保存前节点
		Node next=first.next; // 保存后节点
		do
		{
			aux.prev=next; // 反转前节点
			aux.next=prev; // 反转后节点
			aux=next; // 移动到下一个
			prev=aux.prev; // 更新prev
			next=aux.next; // 更新next
		}
		while(aux!=first); // 直到回到第一个
	}
	
	public double F() // 计算路线的总成本
	{
		double f=0; // 初始化成本
		Node node=first; // 从第一个节点开始
		Node next=node.next; // 下一个节点
		do
		{
			f+=instance.dist(node.name,next.name); // 累加距离
			node=next; // 移动节点
			next=node.next; // 更新下一个
		}
		while(next!=first); // 直到回到第一个
		
		f+=instance.dist(node.name,next.name); // 添加最后一段
		return f; // 返回总成本
	}
	
	public void findError() // 查找路线中的错误
	{
		int count=0; // 计数器
		Node aux=first; // 辅助节点
		
		do
		{
			if(aux.prev==null||aux.next==null) // 检查前或后节点是否为空
			{
				System.out.println("Erro no null No: "+aux+" em:\n"+this.toString());
				System.out.println(this);
			}
			if(aux.route!=this) // 检查路线是否正确
				System.out.println("Erro no : "+aux+" com route Errada:\n"+this.toString());

			count++; // 计数
			aux=aux.next; // 下一个
		}
		while(aux!=first); // 循环
		if(count!=numElements) // 检查计数是否匹配
			System.out.println("Error in numElements \n"+this.toString());

	}
	
	//------------------------Visualizacao-------------------------

	@Override
	public String toString() // 重写的toString方法
	{
		String str="Route: "+nameRoute; // 路线名称
		str+=" f: "+fRoute; // 成本
		str+=" space: "+availableCapacity(); // 可用容量
		str+=" size: "+numElements+" = "; // 元素数量
		str+=" modified: "+modified+" = "; // 是否修改

		Node aux=first; // 辅助节点
		do
		{
			str+=aux+"->"; // 添加节点
//			System.out.println(str);
			aux=aux.next; // 下一个
		}
		while(aux!=first); // 循环
		
		return str; // 返回字符串
	}
	
	public String toString2() // 另一种字符串表示
	{
		String str="Route #"+(nameRoute+1)+": "; // 路线编号
		Node aux=first.next; // 从下一个开始
		do
		{
			str+=aux.name+" "; // 添加节点名称
			aux=aux.next; // 下一个
		}
		while(aux!=first); // 循环
		
		return str; // 返回字符串
	}
	
	public boolean isFeasible() // 检查路线是否可行
	{
		if((capacity-totalDemand)>=0) // 如果容量足够
			return true; // 可行
		return false; // 不可行
	}
	
	public int availableCapacity() // 获取可用容量
	{
		return capacity-totalDemand; // 返回容量减去总需求
	}
	
	public void setIncreaseObjFunction(int increaseObjFunction) { // 设置目标函数增加量
		this.increaseObjFunction = increaseObjFunction; // 设置值
	}

	public int getNumElements() { // 获取元素数量
		return numElements; // 返回数量
	}

	public void setNumElements(int numElements) { // 设置元素数量
		this.numElements = numElements; // 设置值
	}
	
	@Override
	public int compareTo(Route x) // 比较路线
	{
		return nameRoute-x.nameRoute; // 返回名称差
	}

	public double remove(Node node) // 移除节点
	{
		double cost=node.costRemoval(); // 计算移除成本
		
		if(node==first) // 如果是第一个节点
			first=node.prev; // 更新第一个
		
		modified=true; // 标记修改
		node.modified=true; // 节点修改
		node.next.modified=true; // 下一个修改
		node.prev.modified=true; // 前一个修改
		
		node.nodeBelong=false; // 节点不属于路线
		
		fRoute+=cost; // 更新路线成本
		numElements--; // 减少元素数量
		
		prev=node.prev; // 保存前节点
		next=node.next; // 保存后节点
		
		prev.next=next; // 连接前和后
		next.prev=prev; // 连接后和前
		
		totalDemand-=node.demand; // 减少总需求
		
		node.route=null; // 设置node不属于任何路线
		node.prev=null; // 清空node前后索引
		node.next=null; 
		
		return cost; // 返回成本
	//------------------------Add No-------------------------
	
	public double addNodeEndRoute(Node node) // 添加节点到路线末尾
	{
		node.route=this; // 设置节点路线
		if(first==null) // 如果第一个为空，空路线，设置node为仓库
		{
			first=node; // 设置为第一个
			first.prev=node; // 前节点为自身
			first.next=node; // 后节点为自身
			numElements++; // 增加元素
			totalDemand=0; // 总需求为0
			fRoute=0; // 成本为0
			return 0; // 返回0
		}
		else if(node.name==0) // 非空路线，插入结点是仓库
		{
			aux=first.prev; // 辅助为前一个（最后一个客户节点）
			first=node; // 设置node为第一个
			return addAfter(node, aux); // 添加在后
		}
		else // 非空路线插入客户节点
		{
			aux=first.prev; // 辅助为前一个
			return addAfter(node, aux); // 添加在后
		}
	}
	
	public double addAfter(Node node1, Node node2) // 在节点2后添加节点1
	{
		increaseObjFunction=node1.costInsertAfter(node2); // 计算插入成本
		modified=true; // 标记修改
		node1.nodeBelong=true; // 节点属于路线
		node1.modified=true; // 节点修改
		node2.next.modified=true; // 下一个修改
		node2.prev.modified=true; // 前一个修改
		
		numElements++; // 增加元素
		fRoute+=increaseObjFunction; // 更新成本
		
		totalDemand+=node1.demand; // 增加总需求

		node1.route=this; // 设置路线
		
		next=node2.next; // 保存下一个
		node2.next=node1; // 插入节点
		node1.prev=node2; // 设置前节点
		
		next.prev=node1; // 设置前节点
		node1.next=next; // 设置后节点
		
		if(node1.name==0) // 如果是仓库
			first=node1; // 设置为第一个
		
		return increaseObjFunction; // 返回增加量
	}
}

