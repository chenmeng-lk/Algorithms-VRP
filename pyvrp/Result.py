import math  # 数学模块，用于inf常量
from dataclasses import dataclass  # 数据类装饰器
#求解结果类，存储求解结果和统计信息
from pyvrp.Statistics import Statistics  # 统计信息类
from pyvrp._pyvrp import CostEvaluator, Solution  # C++绑定的成本评估器和解类


@dataclass
class Result:  # 结果类：存储单次运行的求解结果
    """
    Stores the outcomes of a single run. An instance of this class is returned
    once the IteratedLocalSearch completes.

    Parameters
    ----------
    best
        The best observed solution.
    stats
        A Statistics object containing runtime statistics.
    num_iterations
        Number of iterations performed by the iterated local search algorithm.
    runtime
        Total runtime of the main iterated local search loop.

    Raises
    ------
    ValueError
        When the number of iterations or runtime are negative.
    """

    best: Solution  # 最优解，算法找到的最佳解
    stats: Statistics  # 统计信息对象，包含运行时的统计数据
    num_iterations: int  # 迭代次数，迭代局部搜索算法执行的迭代数
    runtime: float  # 运行时间（秒），主迭代局部搜索循环的总运行时间

    def __post_init__(self):  # 数据类初始化后验证方法
        if self.num_iterations < 0:  # 验证迭代次数不能为负
            raise ValueError("Negative number of iterations not understood.")

        if self.runtime < 0:  # 验证运行时间不能为负
            raise ValueError("Negative runtime not understood.")

    def cost(self) -> float:  # 获取成本方法：返回最优解的目标函数值
        """
        Returns the cost (objective) value of the best solution. Returns inf
        if the best solution is infeasible.
        """
        if not self.best.is_feasible():  # 如果最优解不可行
            return math.inf  # 返回无穷大

        num_load_dims = len(self.best.excess_load())  # 获取负载维度数量
        return CostEvaluator([0] * num_load_dims, 0, 0).cost(self.best)  # 使用零惩罚值计算成本（仅目标函数值）

    def is_feasible(self) -> bool:  # 检查可行性方法：返回最优解是否可行
        """
        Returns whether the best solution is feasible.
        """
        return self.best.is_feasible()  # 返回最优解的可行性

    def summary(self) -> str:  # 生成摘要方法：返回格式化的结果摘要字符串
        """
        Returns a nicely formatted result summary.
        """
        obj_str = f"{self.cost()}" if self.is_feasible() else "INFEASIBLE"  # 如果可行则显示成本，否则显示"INFEASIBLE"
        summary = [  # 构建摘要列表
            "Solution results",  # 标题
            "================",  # 分隔线
            f"    # routes: {self.best.num_routes()}",  # 路线数量
            f"     # trips: {self.best.num_trips()}",  # 行程数量
            f"   # clients: {self.best.num_clients()}",  # 客户数量
            f"   objective: {obj_str}",  # 目标函数值
            f"    distance: {self.best.distance()}",  # 总距离
            f"    duration: {self.best.duration()}",  # 总持续时间
            f"# iterations: {self.num_iterations}",  # 迭代次数
            f"    run-time: {self.runtime:.2f} seconds",  # 运行时间（保留2位小数）
        ]

        return "\n".join(summary)  # 用换行符连接并返回

    def __str__(self) -> str:  # 字符串表示方法：返回完整的结果字符串
        content = [  # 构建内容列表
            self.summary(),  # 摘要
            "",  # 空行
            "Routes",  # 路线标题
            "------",  # 分隔线
            str(self.best),  # 最优解的字符串表示
        ]

        return "\n".join(content)  # 用换行符连接并返回
