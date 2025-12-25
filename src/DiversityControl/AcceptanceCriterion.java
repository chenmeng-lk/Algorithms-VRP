package DiversityControl;

// 接受准则类：用于决定是否接受一个新的解决方案，通过动态调整接受阈值来平衡搜索的探索与利用
import Auxiliary.Mean;
import Data.Instance;
import SearchMethod.Config;
import SearchMethod.StoppingCriterionType;
import Solution.Solution;

public class AcceptanceCriterion 
{
	double thresholdOF; // 接受阈值，解决方案成本必须小于等于此阈值才能被接受
	double eta,etaMin,etaMax; // eta:当前阈值参数，etaMin/etaMax:eta的最小值和最大值限制
	long ini; // 记录算法开始时间
	double alpha=1; // 衰减因子，用于动态调整eta
	int globalIterator=0; // 全局迭代计数器
	StoppingCriterionType stoppingCriterionType; // 停止准则类型（迭代次数或时间）
	double executionMaximumLimit; // 最大执行限制（迭代次数或时间）
	int numIterUpdate; // 更新频率，每多少轮迭代更新一次参数
	double upperLimit=Integer.MAX_VALUE,updatedUpperLimit=Integer.MAX_VALUE; // 当前上限和待更新上限，用于计算阈值
	Mean averageLSfunction; // 局部搜索后解的质量（成本）平均值

	// 构造函数：初始化接受准则参数
	public AcceptanceCriterion(Instance instance, Config config, Double executionMaximumLimit)
	{
		this.eta=config.getEtaMax(); // 初始eta设置为最大值
		this.etaMin=config.getEtaMin(); // 设置eta最小值=0.01
		this.etaMax=config.getEtaMax(); // 设置eta最大值=1
		this.stoppingCriterionType=config.getStoppingCriterionType(); // 获取停止准则类型
		this.averageLSfunction=new Mean(config.getGamma()); // 初始化平均值计算器，窗口大小为更新频率
		this.numIterUpdate=config.getGamma(); // 设置更新频率
		this.executionMaximumLimit=executionMaximumLimit; // 设置最大执行限制
	}
	
	// 判断是否接受当前解决方案
	public boolean acceptSolution(Solution solution)
	{
		// 如果是第一次迭代，记录开始时间
		if(globalIterator==0)
			ini=System.currentTimeMillis();
		
		// 将当前解决方案的成本加入平均值计算器
		averageLSfunction.setValue(solution.f);
		
		globalIterator++; // 增加全局迭代计数
		
		// 每达到更新频率，更新上限值
		if(globalIterator%(numIterUpdate)==0)
		{
			upperLimit=updatedUpperLimit; // 将待更新上限设置为当前上限
			updatedUpperLimit=Integer.MAX_VALUE; // 重置待更新上限
		}
		
		// 如果当前解的成本小于待更新上限，则更新待更新上限
		if(solution.f<updatedUpperLimit)
			updatedUpperLimit=solution.f;
			
		// 如果当前解的成本小于当前上限，则更新当前上限
		if(solution.f<upperLimit)
			upperLimit=solution.f;
		
//		--------------------------------------------
		
		// 根据停止准则类型动态调整alpha衰减因子
		switch(stoppingCriterionType)
		{
			case Iteration: 	// 基于迭代次数的停止准则
							if(globalIterator%numIterUpdate==0) // 达到更新频率时
							{
								// 计算alpha衰减因子：使得eta从etaMax衰减到etaMin
								alpha=Math.pow(etaMin/etaMax, (double) 1/executionMaximumLimit);
							}
							break;
							
			case Time: 	// 基于时间的停止准则
							if(globalIterator%numIterUpdate==0)
							{
								double maxTime=executionMaximumLimit; // 最大允许时间
								double current=(double)(System.currentTimeMillis()-ini)/1000; // 当前已用时间（秒）
								double timePercentage=current/maxTime; // 时间进度百分比
								double total=(double)globalIterator/timePercentage; // 估计的总迭代次数
								
								// 基于估计的总迭代次数计算alpha衰减因子
								alpha=Math.pow(etaMin/etaMax, (double) 1/total);
							}
							break;
			default:
				break;
								
		}
		
		// 更新eta值：乘以衰减因子
		eta*=alpha;//从1衰减到0.01
		// 确保eta不低于最小值
		eta=Math.max(eta, etaMin);
		//TODO：不同的解接受策略	始终接受比阈值好的解，不随机接受劣解 随着迭代阈值越来越接近最佳解	使用滑动窗口平均解质量作为基准
		// 计算接受阈值：当前周期内的最佳解成本 + eta * (滑动窗口平均成本 - 当前周期内的最佳解成本) 
		// eta较大时，阈值更接近平均值；eta较小时，阈值更接近最佳解
		thresholdOF=(int)(upperLimit+(eta*(averageLSfunction.getDynamicAverage()-upperLimit)));
		// 如果当前解的成本小于等于阈值，则接受该解
		if(solution.f<=thresholdOF)
			return true;
		else
			return false;
	}
	
	// 获取当前eta值
	public double getEta() {
		return eta;
	}

	// 获取当前接受阈值
	public double getThresholdOF() {
		return thresholdOF;
	}

	// 设置eta值（用于测试或特殊调整）
	public void setEta(double eta) {
		this.eta = eta;
	}
	
	// 设置理想流方法（当前未实现）
	public void setIdealFlow(double idealFlow) {}
}
