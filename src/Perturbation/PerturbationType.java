package Perturbation;
//移除节点的启发式方法
public enum PerturbationType 
{
	Sequential(0),
	Concentric(1);
	
	final int type;
	
	PerturbationType(int type)
	{
		this.type=type;
	}

}
