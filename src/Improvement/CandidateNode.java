package Improvement;

import Evaluators.CostEvaluation;
import Evaluators.MovementType;
import Solution.Node;
import Solution.Route;
//TODO:表示和存储在局部搜索过程中生成的移动操作候选。
//有移动的有效性、成本、移动结点、目标位置等信息。
public class CandidateNode implements Comparable<CandidateNode>
{
	public boolean active;//移动是否有效
	public double cost;//成本变化
	public double evaluationCost;//评估成本(还考虑了负载变化)
	public MovementType moveType;//移动类型例如插入、交换等
	public Node a,b,prevA,prevB;//
	public int indexARoute, indexBRoute;
	public Route routeA,routeB;//涉及的路线
	public boolean intra;//是否为内部路线移动
	public int gain;//移动操作的容量增益
	CostEvaluation evaluateCost;
	
	public CandidateNode(CostEvaluation evaluateCost)
	{
		this.evaluateCost=evaluateCost;
		clean();
	}
	
	public void setImprovedNode(double cost, MovementType moveType, Node a, Node b,int indexARoute,int indexBRoute,double evaluationCost) 
	{
		this.active = true;
		this.cost = cost;
		this.moveType = moveType;
		this.a = a;
		this.b = b;
		this.indexARoute=indexARoute;
		this.indexBRoute=indexBRoute;
		this.evaluationCost=evaluationCost;
	}
	
	public void setImprovedNode(double cost, MovementType moveType, Node a, Node b,double evaluationCost) 
	{
		this.active = true;
		this.cost = cost;
		this.moveType = moveType;
		this.a = a;
		this.b = b;
		this.routeA=a.route;
		this.routeB=b.route;
		this.evaluationCost=evaluationCost;
	}
	
	public void setImprovedNode(double cost, MovementType moveType, Node a, Node b, Node prevA, Node prevB, double evaluationCost) 
	{
		this.active = true;
		this.cost = cost;
		this.moveType = moveType;
		this.a = a;
		this.b = b;
		this.prevA = prevA;
		this.prevB = prevB;
		this.routeA=a.route;
		this.routeB=b.route;
		this.evaluationCost=evaluationCost;
	}
	
	public void setImprovedNode(double cost, MovementType moveType, Node a, Node b, Node prevA, Node prevB, double evaluationCost,int gain) 
	{
		this.active = true;
		this.cost = cost;
		this.moveType = moveType;
		this.a = a;
		this.b = b;
		this.prevA = prevA;
		this.prevB = prevB;
		this.routeA=a.route;
		this.routeB=b.route;
		this.evaluationCost=evaluationCost;
		this.gain=gain;
	}
	
	public void setImprovedNode(double cost, MovementType moveType, Node a, Node b,double evaluationCost,int gain) 
	{
		this.active = true;
		this.cost = cost;
		this.moveType = moveType;
		this.a = a;
		this.b = b;
		this.routeA=a.route;
		this.routeB=b.route;
		this.evaluationCost=evaluationCost;
		this.gain=gain;
	}
	
	public void clone(CandidateNode x) 
	{
		this.active = x.active;
		this.cost = x.cost;
		this.moveType = x.moveType;
		this.a = x.a;
		this.b = x.b;
		this.routeA=a.route;
		this.routeB=b.route;
		this.indexARoute=x.indexARoute;
		this.indexBRoute=x.indexBRoute;
		this.evaluationCost=x.evaluationCost;
	}
	
	public void clean()
	{
		this.evaluationCost=Integer.MAX_VALUE;
		this.cost=Integer.MAX_VALUE;
		this.active=false;
	}

	@Override
	public String toString() 
	{
		if(active)
			return "CandidateNode [active=" + active + ", cost=" + cost + ", evaluationCost=" + evaluationCost + ", moveType="
				+ moveType + ", a=" + a + ", b=" + b + ", nameRouteA=" + a.route.nameRoute + ", nameRouteB=" + b.route.nameRoute
				+ ", intra=" + intra + ", gain=" + gain + "]";
		else
			return "CandidateNode [active=" + active + ", cost=" + cost + ", evaluationCost=" + evaluationCost;
	}

	public int compareTo(CandidateNode x) 
	{
		if(this.evaluationCost!=x.evaluationCost)
		{
			if( this.evaluationCost>x.evaluationCost)
				return 1;
			
			if( this.evaluationCost<x.evaluationCost)
				return -1;
		}
		else
		{
			if( this.gain>x.gain)
				return -1;
			
			if( this.gain<x.gain)
				return 1;
			
		}
		return 0;
	}
}
