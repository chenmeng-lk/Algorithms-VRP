package Solution;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import Data.File;
import Data.Instance;
import Data.Point;
import Improvement.IntraLocalSearch;
import SearchMethod.Config;

public class Solution
{
	private Point points[]; // 点数组，表示所有客户点和仓库点
	Instance instance; // 实例对象，包含问题数据
	Config config; // 配置对象，包含算法参数
	protected int size; // 客户点的数量（不包括仓库）
	Node solution[]; // 客户节点数组

	protected int first;//路线的首节点
	protected Node depot; // 仓库节点
	int capacity; // 车辆容量
	public Route routes[]; // 路线数组
	public int numRoutes; // 当前路线数量
	protected int numRoutesMin; // 最小路线数量
	protected int numRoutesMax; // 最大路线数量
	public double f = 0; // 总成本或目标函数值
	public int distance;
	double epsilon; // 浮点数比较的精度
//	-----------Comparadores-----------

	IntraLocalSearch intraLocalSearch; // 内部局部搜索对象

	// 构造函数，初始化解决方案对象
	public Solution(Instance instance, Config config)
	{
		this.instance = instance; // 设置实例对象
		this.config = config; // 设置配置对象
		this.points = instance.getPoints(); // 获取所有点（客户点和仓库点）
		int depot = instance.getDepot(); // 获取仓库点的索引
		this.capacity = instance.getCapacity(); // 获取车辆容量
		this.size = instance.getSize() - 1; // 计算客户点数量（总点数减去仓库）
		this.solution = new Node[size]; // 初始化客户节点数组
		this.numRoutesMin = instance.getMinNumberRoutes(); // 获取最小路线数量
		this.numRoutes = numRoutesMin; // 设置当前路线数量为最小值
		this.numRoutesMax = instance.getMaxNumberRoutes(); // 获取最大路线数量
		this.depot = new Node(points[depot], instance); // 创建仓库节点
		this.epsilon = config.getEpsilon(); // 获取浮点数比较精度

		this.routes = new Route[numRoutesMax]; // 初始化路线数组

		for(int i = 0; i < routes.length; i++) // 为每个路线创建Route对象
			routes[i] = new Route(instance, config, this.depot, i);

		int count = 0; // 计数器，用于填充solution数组
		for(int i = 0; i < (solution.length + 1); i++) // 遍历所有点，跳过仓库点，创建客户节点
		{
			if(i != depot)
			{
				solution[count] = new Node(points[i], instance);
				count++;
			}
		}
	}

	// 克隆方法，将参考解决方案复制到当前解决方案
	public void clone(Solution reference)
	{
		this.numRoutes = reference.numRoutes; // 复制路线数量
		this.f = reference.f; // 复制总成本

		for(int i = 0; i < routes.length; i++) // 设置路线名称
		{
			routes[i].nameRoute = i;
			reference.routes[i].nameRoute = i;
		}

		for(int i = 0; i < routes.length; i++) // 复制每个路线的属性
		{
			routes[i].totalDemand = reference.routes[i].totalDemand;
			routes[i].fRoute = reference.routes[i].fRoute;
			routes[i].numElements = reference.routes[i].numElements;
			routes[i].modified = reference.routes[i].modified;

			if(reference.routes[i].first.prev == null) // 处理前一个节点
				routes[i].first.prev = null;
			else if(reference.routes[i].first.prev.name == 0)
				routes[i].first.prev = routes[i].first;
			else
				routes[i].first.prev = solution[reference.routes[i].first.prev.name - 1];

			if(reference.routes[i].first.next == null) // 处理下一个节点
				routes[i].first.next = null;
			else if(reference.routes[i].first.next.name == 0)
				routes[i].first.next = routes[i].first;
			else
				routes[i].first.next = solution[reference.routes[i].first.next.name - 1];
		}

		for(int i = 0; i < solution.length; i++) // 复制每个客户节点的属性
		{
			solution[i].route = routes[reference.solution[i].route.nameRoute];
			solution[i].nodeBelong = reference.solution[i].nodeBelong;

			if(reference.solution[i].prev.name == 0) // 处理前一个节点
				solution[i].prev = routes[reference.solution[i].prev.route.nameRoute].first;
			else
				solution[i].prev = solution[reference.solution[i].prev.name - 1];

			if(reference.solution[i].next.name == 0) // 处理下一个节点
				solution[i].next = routes[reference.solution[i].next.route.nameRoute].first;
			else
				solution[i].next = solution[reference.solution[i].next.name - 1];
		}
	}

	// ------------------------Visualizacao-------------------------

	// 自定义的字符串表示方法，用于调试
	public String toStringMeu()
	{
		String str = "size: " + size; // 构建字符串，包含尺寸、仓库、路线数量、容量和总成本
		str += "\n" + "depot: " + depot;
		str += "\nnumRoutes: " + numRoutes;
		str += "\ncapacity: " + capacity;

		str += "\nf: " + f;
//		System.out.println(str);
		for(int i = 0; i < numRoutes; i++) // 遍历所有路线，添加到字符串中
		{
//			System.out.println(str);
			str += "\n" + routes[i];
		}

		return str;
	}

	// 重写的toString方法，返回解决方案的字符串表示
	@Override
	public String toString()
	{
		String str = ""; // 构建字符串，包含所有路线和总成本
		for(int i = 0; i < numRoutes; i++)
		{
			str += routes[i].toString2() + "\n";
		}
		str += "Cost " + f + "\n";
		return str;
	}

	// 计算不可行性，即容量违反的总和
	public int infeasibility()
	{
		int capViolation = 0; // 初始化容量违反量
		for(int i = 0; i < numRoutes; i++) // 遍历所有路线，累加负的可用容量
		{
			if(routes[i].availableCapacity() < 0)
				capViolation += routes[i].availableCapacity();
		}
		return capViolation;
	}

	// 检查解决方案的一致性和可行性
	public boolean checking(String local, boolean feasibility, boolean emptyRoute)
	{
		double f; // 临时变量，用于计算路线成本
		double sumF = 0; // 总成本累加
		int sumNumElements = 0; // 元素数量累加
		boolean erro = false; // 错误标志

		for(int i = 0; i < numRoutes; i++) // 遍历所有路线进行检查
		{
			routes[i].findError();
			f = routes[i].F();
			sumF += f;
			sumNumElements += routes[i].numElements;

			if(Math.abs(f - routes[i].fRoute) > epsilon) // 检查路线成本是否匹配
			{
				System.out.println("-------------------" + local + " ERROR-------------------" + "\n" + routes[i].toString() + "\nf esperado: " + f);
				erro = true;
			}

			if(emptyRoute && routes[i].first == routes[i].first.next) // 检查空路线
			{
				System.out.println("-------------------" + local + " ERROR-------------------" + "Empty route: " + routes[i].toString());
				erro = true;
			}

			if(routes[i].first.name != 0) // 检查路线是否以仓库开始
			{
				System.out.println("-------------------" + local + " ERROR-------------------" + " Route initiating without depot: " + routes[i].toString());
				erro = true;
			}

			if(feasibility && !routes[i].isFeasible()) // 检查可行性
			{
				System.out.println("-------------------" + local + " ERROR-------------------" + "Infeasible route: " + routes[i].toString());
				erro = true;
			}

		}
		if(Math.abs(sumF - this.f) > epsilon) // 检查总成本是否匹配
		{
			erro = true;
			System.out.println("-------------------" + local + " Error total sum-------------------");
			System.out.println("Expected: " + sumF + " obtained: " + this.f);
			System.out.println(this.toStringMeu());
		}

		if((sumNumElements - numRoutes) != size) // 检查元素数量是否匹配
		{
			erro = true;
			System.out.println("-------------------" + local + " ERROR quantity of Elements-------------------");
			System.out.println("Expected: " + size + " obtained : " + (sumNumElements - numRoutes));

			System.out.println(this);
		}
		return erro;
	}

	// 检查解决方案是否可行（所有路线容量不违反）
	public boolean feasible()
	{
		for(int i = 0; i < numRoutes; i++) // 遍历所有路线，检查可用容量
		{
			if(routes[i].availableCapacity() < 0)
				return false;
		}
		return true;
	}

	// 移除空路线
	public void removeEmptyRoutes()
	{
		for(int i = 0; i < numRoutes; i++) // 遍历路线，移除空路线
		{
			if(routes[i].first == routes[i].first.next)
			{
				removeRoute(i);
				i--;
			}
		}
	}

	// 移除指定索引的路线
	private void removeRoute(int index)
	{
		Route aux = routes[index]; // 临时保存要移除的路线
		if(index != numRoutes - 1) // 如果不是最后一个路线，则与最后一个交换
		{
			routes[index] = routes[numRoutes - 1];

			routes[numRoutes - 1] = aux;
		}
		numRoutes--; // 减少路线数量
	}

	// 从文件上传解决方案
	public void uploadSolution(String name)
	{
		BufferedReader in; // 文件读取器
		try
		{
			in = new BufferedReader(new FileReader(name));
			String str[] = null;
			String line;

			line = in.readLine();
			str = line.split(" ");

			for(int i = 0; i < 3; i++) // 跳过前三行
				in.readLine();

			int indexRoute = 0; // 路线索引
			line = in.readLine();
			str = line.split(" ");

			System.out.println("-------------- str.length: " + str.length);
			for(int i = 0; i < str.length; i++)
			{
				System.out.print(str[i] + "-");
			}
			System.out.println();

			do // 读取每行路线
			{
				routes[indexRoute].addNodeEndRoute(depot.clone());
				for(int i = 9; i < str.length - 1; i++) // 添加节点到路线
				{
					System.out.println("add: " + solution[Integer.valueOf(str[i].trim()) - 1] + " na route: " + routes[indexRoute].nameRoute);
					f += routes[indexRoute].addNodeEndRoute(solution[Integer.valueOf(str[i]) - 1]);
				}
				indexRoute++;
				line = in.readLine();
				if(line != null)
					str = line.split(" ");
			}
			while(line != null);

		}
		catch(IOException e)
		{
			System.out.println("File read Error");
		}
	}

	// 从文件上传解决方案的另一种方法
	public void uploadSolution1(String name)
	{
		BufferedReader in; // 文件读取器
		try
		{
			in = new BufferedReader(new FileReader(name));
			String str[] = null;

			str = in.readLine().split(" ");
			int indexRoute = 0; // 路线索引
			while(!str[0].equals("Cost")) // 读取直到"Cost"行
			{
				for(int i = 2; i < str.length; i++) // 添加节点到路线
				{
					f += routes[indexRoute].addNodeEndRoute(solution[Integer.valueOf(str[i]) - 1]);
				}
				indexRoute++;
				str = in.readLine().split(" ");
			}
		}
		catch(IOException e)
		{
			System.out.println("File read Error");
		}
	}

	// 获取路线数组
	public Route[] getRoutes()
	{
		return routes;
	}

	// 获取当前路线数量
	public int getNumRoutes()
	{
		return numRoutes;
	}

	// 获取仓库节点
	public Node getDepot()
	{
		return depot;
	}

	// 获取客户节点数组
	public Node[] getSolution()
	{
		return solution;
	}

	// 获取最大路线数量
	public int getNumRoutesMax()
	{
		return numRoutesMax;
	}

	// 设置最大路线数量
	public void setNumRoutesMax(int numRoutesMax)
	{
		this.numRoutesMax = numRoutesMax;
	}

	// 获取最小路线数量
	public int getNumRoutesMin()
	{
		return numRoutesMin;
	}

	// 设置最小路线数量
	public void setNumRoutesMin(int numRoutesMin)
	{
		this.numRoutesMin = numRoutesMin;
	}

	// 获取客户点数量
	public int getSize()
	{
		return size;
	}

	// 将解决方案打印到文件
	public void printSolution(String end)
	{
		File arq = new File(end);
		arq.write(this.toString());
		arq.close();
	}

}
