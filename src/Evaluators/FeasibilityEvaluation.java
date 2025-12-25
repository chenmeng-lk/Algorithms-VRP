// 可行性评估类：用于评估移动操作（SHIFT、SWAP、CROSS）对解可行性的改善程度，仅考虑了容量限制
package Evaluators;

import Solution.Node;

public class FeasibilityEvaluation 
{
	int capViolation=0; // 移动前的容量违反总量
	int capViolationMove=0; // 移动后的容量违反总量
	 
//	-------------------------------SHIFT------------------------------

	public int gainSHIFT(Node a, Node b) // 计算SHIFT移动的可行性改善程度
	{
		 capViolation=0; // 重置移动前的容量违反量
		 capViolationMove=0; // 重置移动后的容量违反量
		 
		 // 计算移动前两条路线的容量违反情况（负值表示容量不足）
		 if(a.route.availableCapacity()<0)
			capViolation+=a.route.availableCapacity(); // 路线a的容量违反
		 
		 if(b.route.availableCapacity()<0)
			capViolation+=b.route.availableCapacity(); // 路线b的容量违反
		 
		 // 计算移动后的容量违反情况
		 // 将节点a从路线a移动到路线b后：
		 // 路线a的剩余容量会增加a.demand（因为移除了节点a）
		 if((a.route.availableCapacity()+a.demand)<0)
			 capViolationMove+=(a.route.availableCapacity()+a.demand);
			 
		 // 路线b的剩余容量会减少a.demand（因为加入了节点a）
		 if((b.route.availableCapacity()-a.demand)<0)
			 capViolationMove+=(b.route.availableCapacity()-a.demand);
		 
		 // 返回值：移动前的容量违反总量（取负） + 移动后的容量违反总量
		 // 如果结果为正，表示可行性改善；如果结果为负，表示可行性恶化
		 return -capViolation+capViolationMove;
	}
	
//	-------------------------------SWAP------------------------------
	
	public int gainSWAP(Node a, Node b) // 计算SWAP移动的可行性改善程度
	{
		 capViolation=0; // 重置移动前的容量违反量
		 capViolationMove=0; // 重置移动后的容量违反量
		 
		 // 计算移动前两条路线的容量违反情况
		 if(a.route.availableCapacity()<0)
			capViolation+=a.route.availableCapacity(); // 路线a的容量违反
		 
		 if(b.route.availableCapacity()<0)
			capViolation+=b.route.availableCapacity(); // 路线b的容量违反
		 
		 // 计算移动后的容量违反情况
		 // SWAP操作：交换节点a和节点b的位置
		 // 路线a：移出节点a（需求a.demand），移入节点b（需求b.demand）
		 // 净需求变化：(a.demand - b.demand)，如果为负表示需求减少
		 if((a.route.availableCapacity()+(a.demand-b.demand))<0)
			 capViolationMove+=(a.route.availableCapacity()+(a.demand-b.demand));
			 
		 // 路线b：移出节点b（需求b.demand），移入节点a（需求a.demand）
		 // 净需求变化：-(a.demand - b.demand)
		 if((b.route.availableCapacity()-(a.demand-b.demand))<0)
			 capViolationMove+=(b.route.availableCapacity()-(a.demand-b.demand));
		 
		 // 返回值：移动前的容量违反总量（取负） + 移动后的容量违反总量
		 return -capViolation+capViolationMove;
	}
	
//	-------------------------------CROSS------------------------------

	
	public int gainCross(Node a, Node b) // 计算CROSS移动的可行性改善程度，仅考虑了容量限制
	{
		 capViolation=0; // 重置移动前的容量违反量
		 capViolationMove=0; // 重置移动后的容量违反量
		 
		 // 计算移动前两条路线的容量违反情况
		 if(a.route.availableCapacity()<0)
			capViolation+=a.route.availableCapacity(); // 路线a的容量违反
		 
		 if(b.route.availableCapacity()<0)
			capViolation+=b.route.availableCapacity(); // 路线b的容量违反
		 
		 // 计算移动后的容量违反情况
		 // CROSS操作：交换两条路线的后半段
		 // 移动后路线a的需求变化：从节点a之后的路段交换为从节点b之后的路段
		 // 需求变化 = (a路线a节点之后的总需求 - b路线b节点之后的总需求)
		 // 即：(a.route.totalDemand - a.accumulatedDemand) - (b.route.totalDemand - b.accumulatedDemand)
		 if(a.route.availableCapacity()+((a.route.totalDemand-a.accumulatedDemand)-(b.route.totalDemand-b.accumulatedDemand))<0)
			 capViolationMove+=a.route.availableCapacity()+((a.route.totalDemand-a.accumulatedDemand)-(b.route.totalDemand-b.accumulatedDemand));
			 
		 // 路线b的需求变化相反
		 if(b.route.availableCapacity()-((a.route.totalDemand-a.accumulatedDemand)-(b.route.totalDemand-b.accumulatedDemand))<0)
			 capViolationMove+=b.route.availableCapacity()-((a.route.totalDemand-a.accumulatedDemand)-(b.route.totalDemand-b.accumulatedDemand));
		 
		 // 返回值：移动前的容量违反总量（取负） + 移动后的容量违反总量
		 return -capViolation+capViolationMove;
	}
	
	public int gainCrossInverted(Node a, Node b) // 计算反向CROSS移动的可行性改善程度
	{
		 capViolation=0; // 重置移动前的容量违反量
		 capViolationMove=0; // 重置移动后的容量违反量
		 
		 // 计算移动前两条路线的容量违反情况
		 if(a.route.availableCapacity()<0)
			capViolation+=a.route.availableCapacity(); // 路线a的容量违反
		 
		 if(b.route.availableCapacity()<0)
			capViolation+=b.route.availableCapacity(); // 路线b的容量违反
		 
		 // 计算移动后的容量违反情况
		 // 反向CROSS操作：将路线a的后半段连接到路线b的前半段，反之亦然
		 // 移动后路线a的需求变化：将a节点之后的路线替换为b节点之前的路线
		 // 需求变化 = (a节点之后的需求 - b节点之前的需求)
		 // 即：(a.route.totalDemand - a.accumulatedDemand) - b.accumulatedDemand
		 if(a.route.availableCapacity()+((a.route.totalDemand-a.accumulatedDemand)-b.accumulatedDemand)<0)
			 capViolationMove+=a.route.availableCapacity()+((a.route.totalDemand-a.accumulatedDemand)-b.accumulatedDemand);
			 
		 // 路线b的需求变化相反
		 if(b.route.availableCapacity()-((a.route.totalDemand-a.accumulatedDemand)-b.accumulatedDemand)<0)
			 capViolationMove+=b.route.availableCapacity()-((a.route.totalDemand-a.accumulatedDemand)-b.accumulatedDemand);
		 
		 // 返回值：移动前的容量违反总量（取负） + 移动后的容量违反总量
		 return -capViolation+capViolationMove;
	}
	 
}
