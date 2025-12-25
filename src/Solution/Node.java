package Solution;

import Data.Instance;
import Data.Point;

public class Node implements Cloneable
{
	public Node prev,prevOld; // 前一个节点和旧的前一个节点
	public Node next,nextOld; // 下一个节点和旧的下一个节点
	
	public Route route; // 该节点所属的路线
	public int name; // 节点名称（索引）
	public int demand; // 节点的需求量

	public int knn[]; // k近邻节点列表
	public int accumulatedDemand=0; // 累积需求量
	public boolean nodeBelong; // 节点是否属于某个路线
	public boolean fixedRoute; // 节点是否固定在路线中
	public boolean fixedPosition; // 节点是否固定位置
	Point point; // 节点对应的点对象
	Instance instance; // 实例对象
	public boolean modified; // 节点是否被修改
	
	public int nameDestinyRoute; // 目标路线名称
	public int nameOriginRoute; // 原始路线名称
	public int priority; // 优先级
	public Route destinyRoute; // 目标路线对象
	
	public Node(Point point, Instance instance) // 构造函数，初始化节点
	{
		this.point=point; // 设置点对象
		this.instance=instance; // 设置实例对象
		this.name = point.name; // 设置节点名称
		this.demand = point.demand; // 设置需求量

		this.next=null; // 初始化下一个节点为null
		this.prev=null; // 初始化前一个节点为null
		this.route=null; // 初始化路线为null
		this.knn=instance.getKnn()[name]; // 获取k近邻列表
	}
	
	public double dist(Node x) // 计算到另一个节点x的距离
	{
		return instance.dist(x.name,this.name); // 返回距离
	}
	
	public void clean() // 清理节点状态
	{
		this.nodeBelong=false; // 设置不属于路线
		this.route=null; // 清空路线
	}
	
	 public Node clone() // 克隆节点
	 {
		 Node clone = new Node(point, instance); // 创建新节点
		 clone.prev=prev; // 复制前节点
		 clone.next=next; // 复制后节点
		 clone.route=route; // 复制路线
		 return clone; // 返回克隆
	 }
	
	 public double costRemoval() // 计算移除节点的成本
	 {
		 return instance.dist(prev.name,next.name)-instance.dist(name,prev.name)-instance.dist(name,next.name); // 返回成本
	 }
	 
	 public double costInsertAfter(Node node) // 计算插入节点后的成本
	 {
		 if(node==null) // 检查节点是否为空
			 System.out.println("no null");
		 
		 return -instance.dist(node.name,node.next.name)+instance.dist(name,node.name)+instance.dist(name,node.next.name); // 返回成本
	 }
	 
	@Override
	public String toString() // 重写的toString方法
	{
		return "|n: "+name+" d: "+demand+"|"; // 返回节点信息字符串
	}

	public int getDemand() { // 获取需求量
		return demand; // 返回需求
	}

	public int[] getKnn() { // 获取k近邻列表
		return knn; // 返回列表
	}
	
	
}