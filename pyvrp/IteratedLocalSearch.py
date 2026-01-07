from __future__ import annotations  # 启用延迟类型注解
#迭代局部搜索主算法
import time  # 时间模块，用于测量运行时间
from dataclasses import dataclass  # 数据类装饰器，用于创建简单的数据类
from typing import TYPE_CHECKING  # 类型检查时的导入

import numpy as np  # 数值计算库，用于数组操作

from pyvrp.ProgressPrinter import ProgressPrinter  # 进度打印器类，用于显示求解进度
from pyvrp.Result import Result  # 结果类，封装求解结果
from pyvrp.Statistics import Statistics  # 统计信息类，收集求解过程的统计数据

if TYPE_CHECKING:  # 仅在类型检查时导入，避免运行时循环导入
    from pyvrp.PenaltyManager import PenaltyManager  # 惩罚管理器类
    from pyvrp._pyvrp import ProblemData, RandomNumberGenerator, Solution  # C++绑定的核心类
    from pyvrp.search.SearchMethod import SearchMethod  # 搜索方法接口
    from pyvrp.stop.StoppingCriterion import StoppingCriterion  # 停止准则类


@dataclass
class IteratedLocalSearchParams:  # 迭代局部搜索参数类：配置迭代局部搜索算法的参数
    """
    Parameters for the iterated local search algorithm.

    Parameters
    ----------
    num_iters_no_improvement
        Number of iterations without any improvement needed before a restart
        occurs.
    initial_accept_weight
        Initial weight parameter used to determine the threshold value in the
        acceptance criterion. Larger values result in more accepted candidate
        solutions. Must be in [0, 1].
    history_length
        The number of recent candidate solutions to consider when computing the
        threshold value in the acceptance criterion. Must be positive.
    """

    num_iters_no_improvement: int = 20_000  # 无改进迭代次数阈值，达到此值后重启搜索，默认20000
    initial_accept_weight: float = 1  # 初始接受权重，用于计算接受准则的阈值，值越大接受越多候选解，范围[0,1]，默认1
    history_length: int = 500  # 历史长度，计算接受准则阈值时考虑的最近候选解数量，必须为正，默认500

    def __post_init__(self):  # 数据类初始化后验证方法
        if self.num_iters_no_improvement < 0:  # 验证无改进迭代次数不能为负
            raise ValueError("num_iters_no_improvement < 0 not understood.")

        if not (0 <= self.initial_accept_weight <= 1):  # 验证接受权重必须在[0,1]范围内
            raise ValueError("initial_accept_weight must be in [0, 1].")

        if self.history_length <= 0:  # 验证历史长度必须为正
            raise ValueError("history_length must be positive.")


class IteratedLocalSearch:  # 迭代局部搜索类：实现迭代局部搜索主算法
    """
    Creates an IteratedLocalSearch instance.

    Parameters
    ----------
    data
        The problem data instance.
    penalty_manager
        Penalty manager to use.
    rng
        Random number generator.
    search_method
        Search method to use.
    initial_solution
        Initial solution to start the search with.
    params
        Iterated local search parameters to use. If not provided, a default
        will be used.
    """

    def __init__(  # 构造函数：初始化迭代局部搜索对象
        self,
        data: ProblemData,  # 问题数据实例
        penalty_manager: PenaltyManager,  # 惩罚管理器，用于动态调整惩罚值
        rng: RandomNumberGenerator,  # 随机数生成器，用于随机化操作
        search_method: SearchMethod,  # 搜索方法，执行局部搜索操作
        initial_solution: Solution,  # 初始解，搜索的起点
        params: IteratedLocalSearchParams = IteratedLocalSearchParams(),  # 迭代局部搜索参数，默认值
    ):
        self._data = data  # 存储问题数据
        self._pm = penalty_manager  # 存储惩罚管理器
        self._rng = rng  # 存储随机数生成器
        self._search = search_method  # 存储搜索方法
        self._init = initial_solution  # 存储初始解
        self._params = params  # 存储参数

    def run(  # 运行方法：执行迭代局部搜索算法主循环
        self,
        stop: StoppingCriterion,  # 停止准则，定义何时停止搜索
        collect_stats: bool = True,  # 是否收集统计信息，默认为True
        display: bool = False,  # 是否显示进度，默认为False（需要collect_stats为True）
        display_interval: float = 5.0,  # 显示间隔（秒），默认5秒
    ) -> Result:  # 返回Result对象，包含统计信息和最优解
        """
        Runs the iterated local search algorithm with the provided stopping
        criterion.

        Parameters
        ----------
        stop
            Stopping criterion to use. The algorithm runs until the first time
            the stopping criterion returns ``True``.
        collect_stats
            Whether to collect statistics about the solver's progress. Default
            ``True``.
        display
            Whether to display information about the solver progress. Default
            ``False``. Progress information is only available when
            ``collect_stats`` is also set.
        display_interval
            Time (in seconds) between iteration logs. Defaults to 5s.

        Returns
        -------
        Result
            A Result object, containing statistics (if collected) and the best
            found solution.
        """
        print_progress = ProgressPrinter(display, display_interval)  # 创建进度打印器
        print_progress.start(self._data)  # 开始显示进度

        history = History(size=self._params.history_length)  # 创建历史记录对象，用于存储最近候选解的成本
        stats = Statistics(collect_stats=collect_stats)  # 创建统计信息对象

        start = time.perf_counter()  # 记录开始时间
        iters = iters_no_improvement = 0  # 初始化迭代计数器和无改进迭代计数器
        best = current = self._init  # 初始化最优解和当前解为初始解

        cost_eval = self._pm.cost_evaluator()  # 获取成本评估器
        while not stop(cost_eval.cost(best)):  # 主循环：直到停止准则满足
            iters += 1  # 增加迭代计数
            iters_no_improvement += 1  # 增加无改进迭代计数

            if iters_no_improvement == self._params.num_iters_no_improvement:  # 如果达到无改进迭代阈值
                print_progress.restart()  # 打印重启信息
                history.clear()  # 清空历史记录

                current = best  # 将当前解重置为最优解（重启）
                iters_no_improvement = 0  # 重置无改进迭代计数

            cost_eval = self._pm.cost_evaluator()  # 获取最新的成本评估器（惩罚值可能已更新）
            candidate = self._search(current, cost_eval)  # 对当前解执行迭代局部搜索，得到候选解(调用主搜索算子operator)
            self._pm.register(candidate)  # 向惩罚管理器注册候选解，用于更新惩罚值

            if cost_eval.cost(candidate) < cost_eval.cost(best):  # new best 如果候选解成本更低（新的最优解）
                best = candidate  # 更新最优解
                iters_no_improvement = 0  # 重置无改进迭代计数

            cand_cost = cost_eval.penalised_cost(candidate)  # 计算候选解的惩罚成本
            history.append(cand_cost)  # 将候选解成本添加到历史记录

            # Evaluate replacing the current solution with the candidate. A
            # candidate solution is accepted if it is better than a threshold
            # value based on the recent history of candidate objectives. This
            # threshold value is a convex combination of the recent best and
            # mean values. Based on Maximo and Nascimento (2021); see
            # https://doi.org/10.1016/j.ejor.2021.02.024 for more details.
            # 评估是否用候选解替换当前解。如果候选解优于基于最近历史的目标值阈值，则接受。
            # 阈值是最近最优值和平均值的凸组合。基于Maximo和Nascimento (2021)的论文。
            weight = self._params.initial_accept_weight  # 获取初始接受权重
            if (fraction := stop.fraction_remaining()) is not None:  # 如果停止准则支持剩余时间比例
                weight *= fraction  # 根据剩余时间比例调整权重（时间越少，权重越小，接受越严格）

            best_weight = (1 - weight) * history.min()  # 计算最优值权重部分
            mean_weight = weight * history.mean()  # 计算平均值权重部分
            if cand_cost <= best_weight + mean_weight:  # 如果候选解成本小于等于阈值
                current = candidate  # 接受候选解，更新当前解

            stats.collect(current, candidate, best, cost_eval)  # 收集统计信息
            print_progress.iteration(stats)  # 打印迭代进度

        runtime = time.perf_counter() - start  # 计算总运行时间
        res = Result(best, stats, iters, runtime)  # 创建结果对象

        print_progress.end(res)  # 打印结束信息

        return res  # 返回结果对象


class History:  # 历史记录类：管理最近候选解成本值的历史记录
    """
    Small helper class to manage a history of recent candidate solution values.
    """

    def __init__(self, size: int):  # 构造函数：初始化历史记录对象
        self._array = np.full(shape=(size,), fill_value=np.nan)  # 创建固定大小的数组，初始值为NaN
        self._idx = 0  # 当前写入索引

    def __len__(self) -> int:  # 长度方法：返回非NaN值的数量
        return np.count_nonzero(~np.isnan(self._array))  # 统计非NaN值的数量

    def clear(self):  # 清空方法：清空历史记录
        self._array.fill(np.nan)  # 将所有值设为NaN
        self._idx = 0  # 重置索引

    def append(self, value: int):  # 添加方法：添加新的成本值到历史记录
        self._array[self._idx % self._array.size] = value  # 使用循环缓冲区，覆盖最旧的值
        self._idx += 1  # 增加索引

    def min(self) -> float:  # 最小值方法：返回历史记录中的最小成本值
        return np.nanmin(self._array)  # 忽略NaN值，返回最小值

    def mean(self) -> float:  # 平均值方法：返回历史记录中的平均成本值
        return np.nanmean(self._array)  # 忽略NaN值，返回平均值
