package DiversityControl;

// Omega调整类：负责动态调整扰动强度参数omega，用于控制局部搜索后的解与当前最优解之间的距离，以平衡搜索的多样性和集中性
import java.text.DecimalFormat;
import java.util.Random;

import Auxiliary.Mean;
import Perturbation.PerturbationType;
import SearchMethod.Config;

public class OmegaAdjustment 
{
	double omega,omegaMin,omegaMax; // omega:当前扰动强度，omegaMin/omegaMax:omega的最小值和最大值限制
	Mean meanLSDist; // 用于计算局部搜索后解与当前最优解之间距离的动态平均值
	int iterator=0; // 迭代计数器，用于跟踪更新周期
	DecimalFormat deci2=new DecimalFormat("0.00"); // 格式化输出，保留两位小数
	double obtainedDist; // 实际获得的平均距离
	
	Mean averageOmega; // omega的平均值，用于监控omega的变化趋势
	
	double actualOmega; // 当前实际使用的omega值（经过边界限制后）
	Random rand=new Random(); // 随机数生成器
	PerturbationType perturbationType; // 扰动类型，用于标识不同的扰动策略
	int numIterUpdate; // Omega更新频率，每多少轮迭代更新一次
	IdealDist idealDist; // 理想距离对象，包含目标距离值
	
	// 构造函数：初始化Omega调整器
	public OmegaAdjustment(PerturbationType perturbationType, Config config, Integer size,IdealDist idealDist) 
	{
		this.perturbationType = perturbationType; // 设置扰动类型
		this.omega = idealDist.idealDist; // 初始omega设置为理想距离
		this.numIterUpdate = config.getGamma(); // 从配置中获取更新频率
		this.omegaMin=1; // 设置omega最小值
		this.omegaMax=size-2; // 设置omega最大值，通常为问题规模减2
		this.averageOmega=new Mean(config.getGamma()); // 初始化omega平均值计算器，窗口大小为更新频率
		this.meanLSDist=new Mean(config.getGamma()); // 初始化距离平均值计算器，窗口大小为更新频率

		this.idealDist=idealDist; // 设置理想距离对象
	}
	
	// 设置omega值：根据实际获得距离与理想距离的差异调整omega，使其趋向于理想距离
	//TODO：omega扰动参数控制破坏数量，随着迭代次数增加，期望新解与参考解的距离（破碎对距离）越来越小，从30减小到15。
	//FILO2是根据解S中弧的平均成本计算出强化上界下界阈值，然后根据新解的成本变化值决定相关顶点的omega的增减
	public void setupOmega()
	{
		// 获取局部搜索距离的动态平均值
		obtainedDist=meanLSDist.getDynamicAverage();

		// 调整公式：omega = omega + (omega/obtainedDist * idealDist - omega)
		// 相当于 omega = omega * (idealDist / obtainedDist)
		// 目的是使实际距离趋近于理想距离
		omega+=((omega/obtainedDist*idealDist.idealDist)-omega);

		// 将omega限制在最小值和最大值之间
		omega=Math.min(omegaMax, Math.max(omega, omegaMin));
		
		// 记录当前的omega值到平均值计算器
		averageOmega.setValue(omega);
		
		// 重置迭代计数器
		iterator=0;
	}
	
	// 设置距离值：收集每次局部搜索后的距离，并在达到更新频率时调整omega
	public void setDistance(double distLS)
	{
		iterator++; // 增加迭代计数
		
		// 将本次局部搜索后的距离加入平均值计算器
		meanLSDist.setValue(distLS);

		// 如果达到更新频率，则调整omega
		if(iterator%numIterUpdate==0)
			setupOmega();
	}
	
	// 获取当前实际使用的omega值（经过边界限制）
	public double getActualOmega() 
	{
		actualOmega=omega; // 获取当前omega值
		// 确保实际omega在允许范围内
		actualOmega=Math.min(omegaMax, Math.max(actualOmega, omegaMin));
		return actualOmega;
	}

	// 重写toString方法，用于输出当前状态信息
	@Override
	public String toString() {
		return 
		// 输出扰动类型（去掉"PerturbationType."前缀）、当前omega值
		"o"+String.valueOf(perturbationType).substring(4)+": " + deci2.format(omega) 
		// 输出扰动类型的局部搜索距离平均值
		+ " meanLSDist"+String.valueOf(perturbationType).substring(4)+": " + meanLSDist
		// 输出扰动类型的理想距离
		+ " dMI"+String.valueOf(perturbationType).substring(4)+": " + deci2.format(idealDist.idealDist)
		// 输出实际使用的omega值
		+ " actualOmega: "+deci2.format(actualOmega)
		// 输出实际获得的距离
		+ " obtainedDist: "+obtainedDist
		// 输出扰动类型的omega平均值
		+ " averageOmega"+String.valueOf(perturbationType).substring(4)+": " + averageOmega;
	}
	
	// 获取omega的平均值对象
	public Mean getAverageOmega() 
	{
		return averageOmega;
	}

	// 获取扰动类型
	public PerturbationType getPerturbationType() {
		return perturbationType;
	}

	// 设置实际omega值（通常用于测试或特殊调整）
	public void setActualOmega(double actualOmega) {
		this.actualOmega = actualOmega;
	}

}
