package DiversityControl;

import SearchMethod.Config;
import SearchMethod.StoppingCriterionType;

public class DistAdjustment 
{
	int distMMin;
	int distMMax;
	int iterator;
	long ini;
	double executionMaximumLimit;
	double alpha=1;
	StoppingCriterionType stoppingCriterionType;
	IdealDist idealDist;

	public DistAdjustment(IdealDist idealDist,Config config,double executionMaximumLimit) 
	{
		this.idealDist=idealDist;
		this.executionMaximumLimit=executionMaximumLimit;
		this.distMMin=config.getDMin();
		this.distMMax=config.getDMax();
		this.idealDist.idealDist=distMMax;
		this.stoppingCriterionType=config.getStoppingCriterionType();
	}

	public void distAdjustment()//调整迭代过程中新解与参考解的距离，随着算法收敛距离越来越短，idealDist从30->15
	{
		if(iterator==0)
			ini=System.currentTimeMillis();
		
		iterator++;
		
		switch(stoppingCriterionType)
		{
			case Iteration: 	iterationAdjustment(); break;
			case Time: timeAdjustment(); break;
			default:
				break;
								
		}
		
		idealDist.idealDist*=alpha;
		idealDist.idealDist= Math.min(distMMax, Math.max(idealDist.idealDist, distMMin));
		
	}
	
	private void iterationAdjustment()
	{
		alpha=Math.pow((double)distMMin/(double)distMMax, (double) 1/executionMaximumLimit);
	}
	
	private void timeAdjustment()
	{
		double current=(double)(System.currentTimeMillis()-ini)/1000;
		double timePercentage=current/executionMaximumLimit;
		double total=(double)iterator/timePercentage;
		alpha=Math.pow((double)distMMin/(double)distMMax, (double) 1/total);
	}
}
