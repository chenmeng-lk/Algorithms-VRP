from __future__ import annotations  # 启用延迟类型注解
#求解入口，包含 solve() 和 SolveParams 类，用于配置求解参数
import tomllib  # TOML文件解析库，用于从TOML文件加载配置
from typing import TYPE_CHECKING  # 类型检查时的导入

import pyvrp.search  # 搜索模块，包含搜索算子和相关功能
from pyvrp.IteratedLocalSearch import (
    IteratedLocalSearch,  # 迭代局部搜索算法类
    IteratedLocalSearchParams,  # 迭代局部搜索参数类
)
from pyvrp.PenaltyManager import PenaltyManager, PenaltyParams  # 惩罚管理器和参数类
from pyvrp._pyvrp import ProblemData, RandomNumberGenerator, Solution  # C++绑定的核心类
from pyvrp.search import (
    NODE_OPERATORS,  # 默认节点算子列表
    ROUTE_OPERATORS,  # 默认路径算子列表
    LocalSearch,  # 局部搜索类
    NeighbourhoodParams,  # 邻域参数类
    NodeOperator,  # 节点算子基类
    PerturbationManager,  # 扰动管理器类
    PerturbationParams,  # 扰动参数类
    RouteOperator,  # 路径算子基类
    compute_neighbours,  # 计算邻域函数
)

if TYPE_CHECKING:  # 仅在类型检查时导入，避免运行时循环导入
    import pathlib  # 路径处理库

    from pyvrp.Result import Result  # 结果类
    from pyvrp.stop import StoppingCriterion  # 停止准则类


class SolveParams:  # 求解参数类：配置PyVRP迭代局部搜索算法的所有参数
    """
    Solver parameters for PyVRP's iterated local search algorithm.

    Parameters
    ----------
    ils
        Iterated local search parameters.
    penalty
        Penalty parameters.
    neighbourhood
        Neighbourhood parameters.
    node_ops
        Node operators to use in the search.
    route_ops
        Route operators to use in the search.
    display_interval
        Time (in seconds) between iteration logs. Default 5s.
    perturbation
        Perturbation parameters.
    initial_solution
        Solution to start the search from. If not provided, a default solution
        will be created.
    """

    def __init__(  # 构造函数：初始化求解参数
        self,
        ils: IteratedLocalSearchParams = IteratedLocalSearchParams(),  # 迭代局部搜索参数，默认值
        penalty: PenaltyParams = PenaltyParams(),  # 惩罚参数，默认值
        neighbourhood: NeighbourhoodParams = NeighbourhoodParams(),  # 邻域参数，默认值
        node_ops: list[type[NodeOperator]] = NODE_OPERATORS,  # 节点算子类型列表，默认使用所有节点算子
        route_ops: list[type[RouteOperator]] = ROUTE_OPERATORS,  # 路径算子类型列表，默认使用所有路径算子
        display_interval: float = 5.0,  # 显示间隔（秒），默认5秒
        perturbation: PerturbationParams = PerturbationParams(),  # 扰动参数，默认值
        initial_solution: Solution | None = None,  # 初始解，如果为None则自动生成
    ):
        self._ils = ils  # 存储迭代局部搜索参数
        self._penalty = penalty  # 存储惩罚参数
        self._neighbourhood = neighbourhood  # 存储邻域参数
        self._node_ops = node_ops  # 存储节点算子类型列表
        self._route_ops = route_ops  # 存储路径算子类型列表
        self._display_interval = display_interval  # 存储显示间隔
        self._perturbation = perturbation  # 存储扰动参数
        self._initial_solution = initial_solution  # 存储初始解

    def __eq__(self, other: object) -> bool:  # 相等性比较方法：比较两个SolveParams对象是否相等
        return (
            isinstance(other, SolveParams)  # 检查是否为SolveParams类型
            and self.ils == other.ils  # 比较迭代局部搜索参数
            and self.penalty == other.penalty  # 比较惩罚参数
            and self.neighbourhood == other.neighbourhood  # 比较邻域参数
            and self.node_ops == other.node_ops  # 比较节点算子列表
            and self.route_ops == other.route_ops  # 比较路径算子列表
            and self.display_interval == other.display_interval  # 比较显示间隔
            and self.perturbation == other.perturbation  # 比较扰动参数
            and self.initial_solution == other.initial_solution  # 比较初始解
        )

    @property
    def ils(self):  # 迭代局部搜索参数属性访问器
        return self._ils  # 返回迭代局部搜索参数

    @property
    def penalty(self):  # 惩罚参数属性访问器
        return self._penalty  # 返回惩罚参数

    @property
    def neighbourhood(self):  # 邻域参数属性访问器
        return self._neighbourhood  # 返回邻域参数

    @property
    def node_ops(self):  # 节点算子列表属性访问器
        return self._node_ops  # 返回节点算子类型列表

    @property
    def route_ops(self):  # 路径算子列表属性访问器
        return self._route_ops  # 返回路径算子类型列表

    @property
    def display_interval(self) -> float:  # 显示间隔属性访问器
        return self._display_interval  # 返回显示间隔（秒）

    @property
    def perturbation(self):  # 扰动参数属性访问器
        return self._perturbation  # 返回扰动参数

    @property
    def initial_solution(self) -> Solution | None:  # 初始解属性访问器
        return self._initial_solution  # 返回初始解（可能为None）

    @classmethod
    def from_file(cls, loc: str | pathlib.Path):  # 类方法：从TOML文件加载求解参数
        """
        Loads the solver parameters from a TOML file.

        .. note::
           The initial solution cannot be loaded from file, and will always be
           set to ``None``.
        """
        with open(loc, "rb") as fh:  # 以二进制模式打开TOML文件
            data = tomllib.load(fh)  # 加载TOML文件内容

        node_ops = NODE_OPERATORS  # 默认使用所有节点算子
        if "node_ops" in data:  # 如果文件中指定了节点算子
            node_ops = [getattr(pyvrp.search, op) for op in data["node_ops"]]  # 从搜索模块获取指定的算子类

        route_ops = ROUTE_OPERATORS  # 默认使用所有路径算子
        if "route_ops" in data:  # 如果文件中指定了路径算子
            route_ops = [getattr(pyvrp.search, op) for op in data["route_ops"]]  # 从搜索模块获取指定的算子类

        return cls(  # 创建并返回SolveParams实例
            IteratedLocalSearchParams(**data.get("ils", {})),  # 从数据中获取ILS参数，如果不存在则使用空字典
            PenaltyParams(**data.get("penalty", {})),  # 从数据中获取惩罚参数
            NeighbourhoodParams(**data.get("neighbourhood", {})),  # 从数据中获取邻域参数
            node_ops,  # 节点算子列表
            route_ops,  # 路径算子列表
            data.get("display_interval", 5.0),  # 显示间隔，默认5.0秒
            PerturbationParams(**data.get("perturbation", {})),  # 从数据中获取扰动参数
            None,  # initial solution cannot be loaded from file 初始解无法从文件加载，始终为None
        )


def solve(  # 求解函数：求解给定的VRP问题数据实例
    data: ProblemData,  # 问题数据实例，包含所有客户、仓库、车辆类型等信息
    stop: StoppingCriterion,  # 停止准则，定义何时停止求解
    seed: int = 0,  # 随机数种子，用于控制随机性，默认为0
    collect_stats: bool = True,  # 是否收集统计信息，默认为True
    display: bool = False,  # 是否显示求解进度，默认为False（需要collect_stats为True）
    params: SolveParams = SolveParams(),  # 求解参数，如果未提供则使用默认参数
) -> Result:  # 返回Result对象，包含统计信息和最优解
    """
    Solves the given problem data instance.

    Parameters
    ----------
    data
        Problem data instance to solve.
    stop
        Stopping criterion to use.
    seed
        Seed value to use for the random number stream. Default 0.
    collect_stats
        Whether to collect statistics about the solver's progress. Default
        ``True``.
    display
        Whether to display information about the solver progress. Default
        ``False``. Progress information is only available when
        ``collect_stats`` is also set, which it is by default.
    params
        Solver parameters to use. If not provided, a default will be used.

    Returns
    -------
    Result
        A Result object, containing statistics (if collected) and the best
        found solution.
    """
    rng = RandomNumberGenerator(seed=seed)  # 创建随机数生成器，使用指定的种子
    neighbours = compute_neighbours(data, params.neighbourhood)  # 计算邻域结构，用于限制搜索范围
    perturbation = PerturbationManager(params.perturbation)  # 创建扰动管理器，用于跳出局部最优
    ls = LocalSearch(data, rng, neighbours, perturbation)  # 创建局部搜索对象，包含问题数据、随机数生成器、邻域和扰动管理器

    for node_op in params.node_ops:  # 遍历所有节点算子类型
        if node_op.supports(data):  # 检查该算子是否支持当前问题数据
            ls.add_node_operator(node_op(data))  # 如果支持，则创建算子实例并添加到局部搜索中

    for route_op in params.route_ops:  # 遍历所有路径算子类型
        if route_op.supports(data):  # 检查该算子是否支持当前问题数据
            ls.add_route_operator(route_op(data))  # 如果支持，则创建算子实例并添加到局部搜索中

    pm = PenaltyManager.init_from(data, params.penalty)  # 从问题数据初始化惩罚管理器，自动计算初始惩罚值

    init = params.initial_solution  # 获取初始解
    if init is None:  # 如果未提供初始解
        init = ls.search(Solution(data, []), pm.max_cost_evaluator())  # 使用局部搜索从空解生成初始解，使用最大惩罚值确保可行性
        #局部搜索过程中所有节点都会被标记为有希望的

    algo = IteratedLocalSearch(data, pm, rng, ls, init, params.ils)  # 创建迭代局部搜索算法对象
    return algo.run(stop, collect_stats, display, params.display_interval)  # 运行算法并返回结果
