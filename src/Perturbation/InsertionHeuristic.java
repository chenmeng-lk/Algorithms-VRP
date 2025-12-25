package Perturbation;
//添加节点的启发式方法
public enum InsertionHeuristic 
{
	Distance(1),
	Cost(2);
	
	final int heuristic;
	
	InsertionHeuristic(int heuristic)
	{
		this.heuristic=heuristic;
	}
}
