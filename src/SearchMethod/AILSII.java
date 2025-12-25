package SearchMethod;

import java.lang.reflect.InvocationTargetException;
import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.Random;

import Auxiliary.Distance;
import Data.Instance;
import DiversityControl.DistAdjustment;
import DiversityControl.OmegaAdjustment;
import DiversityControl.AcceptanceCriterion;
import DiversityControl.IdealDist;
import Improvement.LocalSearch;
import Improvement.IntraLocalSearch;
import Improvement.FeasibilityPhase;
import Perturbation.InsertionHeuristic;
import Perturbation.Perturbation;
import Solution.Solution;

public class AILSII 
{
	//----------Problema------------
	Solution solution,referenceSolution,bestSolution;//迭代解、参考解、最佳解
	
	Instance instance;
	Distance pairwiseDistance;
	double bestF=Double.MAX_VALUE;
	double executionMaximumLimit;
	double optimal;
	
	//----------caculoLimiar------------
	int numIterUpdate;

	//----------Metricas------------
	int iterator,iteratorMF;
	long first,ini;
	double timeAF,totalTime,time;
	
	Random rand=new Random();
	
	HashMap<String,OmegaAdjustment>omegaSetup=new HashMap<String,OmegaAdjustment>();

	double distanceLS;
	
	Perturbation[] pertubOperators;
	Perturbation selectedPerturbation;
	
	FeasibilityPhase feasibilityOperator;
	ConstructSolution constructSolution;
	
	LocalSearch localSearch;

	InsertionHeuristic insertionHeuristic;
	IntraLocalSearch intraLocalSearch;
	AcceptanceCriterion acceptanceCriterion;
//	----------Mare------------
	DistAdjustment distAdjustment;
//	---------Print----------
	boolean print=true;
	IdealDist idealDist;
	
	double epsilon;
	DecimalFormat deci=new DecimalFormat("0.0000");
	StoppingCriterionType stoppingCriterionType;
	
	public AILSII(Instance instance,InputParameters reader)
	{ 
		this.instance=instance;
		Config config=reader.getConfig();
		this.optimal=reader.getBest();
		this.executionMaximumLimit=reader.getTimeLimit();
		
		this.epsilon=config.getEpsilon();
		this.stoppingCriterionType=config.getStoppingCriterionType();
		this.idealDist=new IdealDist();
		this.solution =new Solution(instance,config);
		this.referenceSolution =new Solution(instance,config);
		this.bestSolution =new Solution(instance,config);
		this.numIterUpdate=config.getGamma();
		
		this.pairwiseDistance=new Distance();
		
		this.pertubOperators=new Perturbation[config.getPerturbation().length];
		
		this.distAdjustment=new DistAdjustment( idealDist, config, executionMaximumLimit);
		
		this.intraLocalSearch=new IntraLocalSearch(instance,config);
		
		this.localSearch=new LocalSearch(instance,config,intraLocalSearch);
		
		this.feasibilityOperator=new FeasibilityPhase(instance,config,intraLocalSearch);
		
		this.constructSolution=new ConstructSolution(instance,config);
		
		OmegaAdjustment newOmegaAdjustment;
		for (int i = 0; i < config.getPerturbation().length; i++) 
		{
			newOmegaAdjustment=new OmegaAdjustment(config.getPerturbation()[i], config,instance.getSize(),idealDist);
			omegaSetup.put(config.getPerturbation()[i]+"", newOmegaAdjustment);
		}
		
		this.acceptanceCriterion=new AcceptanceCriterion(instance,config,executionMaximumLimit);

		try 
		{
			for (int i = 0; i < pertubOperators.length; i++) 
			{
				this.pertubOperators[i]=(Perturbation) Class.forName("Perturbation."+config.getPerturbation()[i]).
				getConstructor(Instance.class,Config.class,HashMap.class,IntraLocalSearch.class).
				newInstance(instance,config,omegaSetup,intraLocalSearch);
			}
			
		} catch (InstantiationException | IllegalAccessException | IllegalArgumentException
				| InvocationTargetException | NoSuchMethodException | SecurityException
				| ClassNotFoundException e) {
			e.printStackTrace();
		}
		
	}

	public void search()
	{
		iterator=0;//迭代次数
		first=System.currentTimeMillis();//开始时间
		referenceSolution.numRoutes=instance.getMinNumberRoutes();//由实例文件读取过程计算，为ceil(total_demand/capacity)
		constructSolution.construct(referenceSolution);//构造初始解
		
		feasibilityOperator.makeFeasible(referenceSolution);//可行化
		localSearch.localSearch(referenceSolution,true);//局部搜索优化
		bestSolution.clone(referenceSolution);//记录最佳解
		while(!stoppingCriterion())//迭代未达到停止条件
		{
			iterator++;

			solution.clone(referenceSolution);//复制参考解，得到的迭代解进行修改，有概率接受新解
			
			selectedPerturbation=pertubOperators[rand.nextInt(pertubOperators.length)];//扰动参数
			selectedPerturbation.applyPerturbation(solution);//扰动
			feasibilityOperator.makeFeasible(solution);//可行化
			localSearch.localSearch(solution,true);//局部搜索
			distanceLS=pairwiseDistance.pairwiseSolutionDistance(solution,referenceSolution);//新解与参考解的距离
			
			evaluateSolution();//比较新解与最佳解看是否更新最佳解
			distAdjustment.distAdjustment();////调整迭代过程中新解与参考解的距离控制参数
			
			selectedPerturbation.getChosenOmega().setDistance(distanceLS);//update 更新扰动值
			
			if(acceptanceCriterion.acceptSolution(solution))//概率接受新解
				referenceSolution.clone(solution);
		}
		
		totalTime=(double)(System.currentTimeMillis()-first)/1000;//算法执行总时间
	}
	
	public void evaluateSolution()//比较新解与最佳解
	{
		if((solution.f-bestF)<-epsilon)
		{		
			bestF=solution.f;
			
			bestSolution.clone(solution);
			iteratorMF=iterator;
			timeAF=(double)(System.currentTimeMillis()-first)/1000;
				
			if(print)
			{
				System.out.println("solution quality: "+bestF
				+" gap: "+deci.format(getGap())+"%"
				+" K: "+solution.numRoutes
				+" iteration: "+iterator
				+" eta: "+deci.format(acceptanceCriterion.getEta())
				+" omega: "+deci.format(selectedPerturbation.omega)
				+" time: "+timeAF
				);
			}
		}
	}
	
	private boolean stoppingCriterion()
	{
		switch(stoppingCriterionType)
		{
			case Iteration: 	if(bestF<=optimal||executionMaximumLimit<=iterator)
									return true;
								break;
							
			case Time: 	if(bestF<=optimal||executionMaximumLimit<(System.currentTimeMillis()-first)/1000)
							return true;
						break;
		}
		return false;
	}
	
	public static void main(String[] args) 
	{
		InputParameters reader=new InputParameters();
		reader.readingInput(args);//读取命令行参数
		
		Instance instance=new Instance(reader);//读取实例文件
		
		AILSII ailsII=new AILSII(instance,reader);
		
		ailsII.search();//调用搜索
	}
	
	public Solution getBestSolution() {
		return bestSolution;
	}

	public double getBestF() {
		return bestF;
	}

	public double getGap()
	{
		return 100*((bestF-optimal)/optimal);
	}
	
	public boolean isPrint() {
		return print;
	}

	public void setPrint(boolean print) {
		this.print = print;
	}

	public Solution getSolution() {
		return solution;
	}

	public int getIterator() {
		return iterator;
	}

	public String printOmegas()
	{
		String str="";
		for (int i = 0; i < pertubOperators.length; i++) 
		{
			str+="\n"+omegaSetup.get(this.pertubOperators[i].perturbationType+""+referenceSolution.numRoutes);
		}
		return str;
	}
	
	public Perturbation[] getPertubOperators() {
		return pertubOperators;
	}
	
	public double getTotalTime() {
		return totalTime;
	}
	
	public double getTimePerIteration() 
	{
		return totalTime/iterator;
	}

	public double getTimeAF() {
		return timeAF;
	}

	public int getIteratorMF() {
		return iteratorMF;
	}
	
	public double getConvergenceIteration()
	{
		return (double)iteratorMF/iterator;
	}
	
	public double convergenceTime()
	{
		return (double)timeAF/totalTime;
	}
	
}
