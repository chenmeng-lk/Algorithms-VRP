from __future__ import annotations  # 启用延迟类型注解
#惩罚项管理，动态调整约束违反惩罚
from dataclasses import dataclass  # 数据类装饰器
from statistics import fmean  # 浮点平均值函数，用于计算可行性百分比
from warnings import warn  # 警告模块，用于发出警告

import numpy as np  # 数值计算库

from pyvrp._pyvrp import CostEvaluator, ProblemData, Solution  # C++绑定的核心类
from pyvrp.exceptions import PenaltyBoundWarning  # 惩罚边界警告异常类


@dataclass
class PenaltyParams:  # 惩罚参数类：配置惩罚管理器的参数
    """
    The penalty manager parameters.

    Parameters
    ----------
    solutions_between_updates
        Number of feasibility registrations between penalty value updates. The
        penalty manager updates the penalty terms every once in a while based
        on recent feasibility registrations. This parameter controls how often
        such updating occurs.
    penalty_increase
        Amount :math:`p_i \\ge 1` by which the current penalties are
        increased when insufficient feasible solutions (see
        ``target_feasible``) have been found amongst the most recent
        registrations. The penalty values :math:`v` are updated as
        :math:`v \\gets p_i v`.
    penalty_decrease
        Amount :math:`p_d \\in [0, 1]` by which the current penalties are
        decreased when sufficient feasible solutions (see ``target_feasible``)
        have been found amongst the most recent registrations. The penalty
        values :math:`v` are updated as :math:`v \\gets p_d v`.
    target_feasible
        Target percentage :math:`p_f \\in [0, 1]` of feasible registrations
        in the last ``solutions_between_updates`` registrations. This
        percentage is used to update the penalty terms: when insufficient
        feasible solutions have been registered, the penalties are increased;
        similarly, when too many feasible solutions have been registered, the
        penalty terms are decreased. This ensures a balanced search, with a
        fraction :math:`p_f` feasible and a fraction :math:`1 - p_f` infeasible
        solutions.
    feas_tolerance
        Deviation tolerance (in :math:`[0, 1]`) between actual and target
        percentage of feasible solutions between updates. If the deviation is
        smaller than this tolerance, the penalty terms are not updated. See
        also ``target_feasible`` and ``solutions_between_updates``.
    min_penalty
        Minimum penalty term value. Must not be negative.
    max_penalty
        Maximum penalty term value. Must not be negative.

        .. warning::
           Setting a (too) large maximum penalty value may cause integer
           overflow in PyVRP's native extensions.

    Attributes
    ----------
    solutions_between_updates
        Number of feasibility registrations between penalty value updates.
    penalty_increase
        Amount :math:`p_i \\ge 1` by which the current penalties are
        increased when insufficient feasible solutions (see
        ``target_feasible``) have been found amongst the most recent
        registrations.
    penalty_decrease
        Amount :math:`p_d \\in [0, 1]` by which the current penalties are
        decreased when sufficient feasible solutions (see ``target_feasible``)
        have been found amongst the most recent registrations.
    target_feasible
        Target percentage :math:`p_f \\in [0, 1]` of feasible registrations
        in the last ``solutions_between_updates`` registrations.
    feas_tolerance
        Deviation tolerance for ``target_feasible``.
    min_penalty
        Minimum penalty term value.
    max_penalty
        Maximum penalty term value.
    """

    solutions_between_updates: int = 500  # 更新间隔，每注册多少次可行性后更新惩罚值，默认500
    penalty_increase: float = 1.25  # 惩罚增加系数，当可行解不足时惩罚值乘以该系数，默认1.25
    penalty_decrease: float = 0.85  # 惩罚减少系数，当可行解过多时惩罚值乘以该系数，默认0.85
    target_feasible: float = 0.90  # 目标可行比例，期望的可行解比例，范围[0,1]，默认0.90
    feas_tolerance: float = 0.05  # 可行容差，实际可行比例与目标比例的偏差容差，范围[0,1]，默认0.05
    min_penalty: float = 0.1  # 最小惩罚值，惩罚值不能低于此值，默认0.1
    max_penalty: float = 100_000.0  # 最大惩罚值，惩罚值不能高于此值，默认100000.0

    def __post_init__(self):  # 数据类初始化后验证方法
        if not self.solutions_between_updates >= 1:  # 验证更新间隔至少为1
            raise ValueError("Expected solutions_between_updates >= 1.")

        if not self.penalty_increase >= 1.0:  # 验证惩罚增加系数至少为1.0
            raise ValueError("Expected penalty_increase >= 1.")

        if not (0.0 <= self.penalty_decrease <= 1.0):  # 验证惩罚减少系数在[0,1]范围内
            raise ValueError("Expected penalty_decrease in [0, 1].")

        if not (0.0 <= self.target_feasible <= 1.0):  # 验证目标可行比例在[0,1]范围内
            raise ValueError("Expected target_feasible in [0, 1].")

        if not (0.0 <= self.feas_tolerance <= 1.0):  # 验证可行容差在[0,1]范围内
            raise ValueError("Expected feas_tolerance in [0, 1].")

        if self.min_penalty < 0:  # 验证最小惩罚值不能为负
            raise ValueError("Expected min_penalty >= 0.")

        if self.max_penalty < self.min_penalty:  # 验证最大惩罚值不能小于最小惩罚值
            raise ValueError("Expected max_penalty >= min_penalty.")


class PenaltyManager:  # 惩罚管理器类：管理时间扭曲和负载惩罚，并根据历史动态调整惩罚值
    """
    Creates a PenaltyManager instance.

    This class manages time warp and load penalties, and provides penalty terms
    for given time warp and load values. It updates these penalties based on
    recent history.

    .. note::

       Consider initialising using :meth:`~init_from` to compute initial
       penalty values that are scaled according to the data instance.

    Parameters
    ----------
    initial_penalties
        Initial penalty values for units of load (idx 0), duration (1), and
        distance (2) violations. These values are clipped to the range
        [:attr:`~pyvrp.PenaltyManager.PenaltyParams.min_penalty`,
        :attr:`~pyvrp.PenaltyManager.PenaltyParams.max_penalty`].
    params
        PenaltyManager parameters. If not provided, a default will be used.
    """

    def __init__(  # 构造函数：初始化惩罚管理器
        self,
        initial_penalties: tuple[list[float], float, float],  # 初始惩罚值元组：(负载惩罚列表, 持续时间惩罚, 距离惩罚)
        params: PenaltyParams = PenaltyParams(),  # 惩罚参数，默认值
    ):
        self._params = params  # 存储参数
        self._penalties = np.clip(  # 将初始惩罚值裁剪到[min_penalty, max_penalty]范围内
            initial_penalties[0] + list(initial_penalties[1:]),  # 将元组转换为列表（负载列表+持续时间+距离）
            params.min_penalty,  # 最小值
            params.max_penalty,  # 最大值
        )

        # Tracks recent feasibilities for each penalty dimension.
        # 跟踪每个惩罚维度最近的可行性记录
        self._feas_lists: list[list[bool]] = [  # 为每个惩罚维度创建可行性列表
            [] for _ in range(len(self._penalties))  # 列表数量等于惩罚值数量
        ]

    def penalties(self) -> tuple[list[float], float, float]:  # 获取当前惩罚值方法：返回当前所有惩罚值
        """
        Returns the current penalty values.
        """
        return (
            self._penalties[:-2].tolist(),  # loads 负载惩罚列表（除最后两个外的所有值）
            self._penalties[-2],  # duration 持续时间惩罚（倒数第二个）
            self._penalties[-1],  # distance 距离惩罚（最后一个）
        )

    @classmethod
    def init_from(  # 类方法：从问题数据初始化惩罚管理器，自动计算初始惩罚值
        cls,
        data: ProblemData,  # 问题数据实例
        params: PenaltyParams = PenaltyParams(),  # 惩罚参数，默认值
    ) -> PenaltyManager:  # 返回PenaltyManager实例
        """
        Initialises from the given data instance and parameter object. The
        initial penalty values are computed from the problem data.

        Parameters
        ----------
        data
            Data instance to use when computing penalty values.
        params
            PenaltyManager parameters. If not provided, a default will be used.
        """
        distances = data.distance_matrices()  # 获取所有距离矩阵
        durations = data.duration_matrices()  # 获取所有持续时间矩阵

        # We first determine the elementwise minimum cost across all vehicle
        # types. This is the cheapest way any edge can be traversed.
        # 首先确定所有车辆类型的逐元素最小成本。这是遍历任何边的最便宜方式。
        unique_edge_costs = {  # 收集唯一的边成本配置（单位距离成本、单位持续时间成本、配置索引）
            (
                veh_type.unit_distance_cost,  # 单位距离成本
                veh_type.unit_duration_cost,  # 单位持续时间成本
                veh_type.profile,  # 配置索引
            )
            for veh_type in data.vehicle_types()  # 遍历所有车辆类型
        }

        first, *rest = unique_edge_costs  # 分离第一个和其余的成本配置
        unit_dist, unit_dur, prof = first  # 解包第一个配置
        edge_costs = unit_dist * distances[prof] + unit_dur * durations[prof]  # 计算第一个配置的边成本矩阵
        for unit_dist, unit_dur, prof in rest:  # 遍历其余配置
            mat = unit_dist * distances[prof] + unit_dur * durations[prof]  # 计算当前配置的边成本矩阵
            np.minimum(edge_costs, mat, out=edge_costs)  # 逐元素取最小值，得到最便宜的边成本

        # Best edge cost/distance/duration over all vehicle types and profiles,
        # and then average that for the entire matrix to obtain an "average
        # best" edge cost/distance/duration.
        # 在所有车辆类型和配置上的最佳边成本/距离/持续时间，然后对整个矩阵求平均，得到"平均最佳"值。
        avg_cost = edge_costs.mean()  # 计算平均边成本
        avg_distance = np.minimum.reduce(distances).mean()  # 计算所有距离矩阵的最小值，然后求平均
        avg_duration = np.minimum.reduce(durations).mean()  # 计算所有持续时间矩阵的最小值，然后求平均

        avg_load = np.zeros((data.num_load_dimensions,))  # 初始化平均负载数组
        if data.num_clients != 0 and data.num_load_dimensions != 0:  # 如果有客户和负载维度
            pickups = np.array([c.pickup for c in data.clients()])  # 获取所有客户的拾取量数组
            deliveries = np.array([c.delivery for c in data.clients()])  # 获取所有客户的配送量数组
            avg_load = np.maximum(pickups, deliveries).mean(axis=0)  # 对每个维度取最大值，然后求平均

        # Initial penalty parameters are meant to weigh an average increase
        # in the relevant value by the same amount as the average edge cost.
        # 初始惩罚参数旨在使相关值的平均增加量与平均边成本相同。
        init_load = avg_cost / np.maximum(avg_load, 1)  # 计算初始负载惩罚：平均成本除以平均负载（至少为1）
        init_tw = avg_cost / max(avg_duration, 1)  # 计算初始时间扭曲惩罚：平均成本除以平均持续时间（至少为1）
        init_dist = avg_cost / max(avg_distance, 1)  # 计算初始距离惩罚：平均成本除以平均距离（至少为1）
        return cls((init_load.tolist(), init_tw, init_dist), params)  # 创建并返回PenaltyManager实例

    def _compute(self, penalty: float, feas_percentage: float) -> float:  # 计算新惩罚值方法：根据当前值和可行性百分比
        # Computes and returns the new penalty value, given the current value
        # and the percentage of feasible solutions since the last update.
        # 计算并返回新的惩罚值，给定当前值和自上次更新以来的可行解百分比。
        diff = self._params.target_feasible - feas_percentage  # 计算目标可行比例与实际可行比例的差值

        if abs(diff) < self._params.feas_tolerance:  # 如果差值小于容差
            return penalty  # 不更新惩罚值，直接返回当前值

        if diff > 0:  # 如果可行解不足（实际比例低于目标）
            new_penalty = self._params.penalty_increase * penalty  # 增加惩罚值
        else:  # 如果可行解过多（实际比例高于目标）
            new_penalty = self._params.penalty_decrease * penalty  # 减少惩罚值

        if new_penalty >= self._params.max_penalty:  # 如果新惩罚值达到最大值
            msg = """
            A penalty parameter has reached its maximum value. This means PyVRP
            struggles to find a feasible solution for this instance, either
            because the instance has no feasible solution, or it is hard to
            find one - possibly due to large data scaling differences. Check
            the instance carefully to determine if a feasible solution exists.
            """
            warn(msg, PenaltyBoundWarning)  # 发出警告

        return np.clip(  # 将新惩罚值裁剪到[min_penalty, max_penalty]范围内
            new_penalty,
            self._params.min_penalty,
            self._params.max_penalty,
        )

    def _register(self, feas_list: list[bool], penalty: float, is_feas: bool):  # 注册可行性方法：记录解的可行性并可能更新惩罚值
        feas_list.append(is_feas)  # 将可行性添加到列表

        if len(feas_list) != self._params.solutions_between_updates:  # 如果未达到更新间隔
            return penalty  # 不更新，返回当前惩罚值

        avg = fmean(feas_list)  # 计算可行性平均值（百分比）
        feas_list.clear()  # 清空列表
        return self._compute(penalty, avg)  # 计算并返回新的惩罚值

    def register(self, sol: Solution):  # 注册解方法：注册解的可行性维度
        """
        Registers the feasibility dimensions of the given solution.
        """
        is_feasible = [  # 构建可行性列表
            *[excess == 0 for excess in sol.excess_load()],  # 每个负载维度的可行性（无超载）
            not sol.has_time_warp(),  # 时间扭曲可行性（无时间扭曲）
            not sol.has_excess_distance(),  # 距离可行性（无超距）
        ]

        for idx, is_feas in enumerate(is_feasible):  # 遍历每个可行性维度
            feas_list = self._feas_lists[idx]  # 获取该维度的可行性列表
            penalty = self._penalties[idx]  # 获取该维度的当前惩罚值
            self._penalties[idx] = self._register(feas_list, penalty, is_feas)  # 注册并可能更新惩罚值

    def cost_evaluator(self) -> CostEvaluator:  # 获取成本评估器方法：使用当前惩罚值创建成本评估器
        """
        Get a cost evaluator using the current penalty values.
        """
        *loads, tw, dist = self._penalties  # 解包惩罚值：负载列表、时间扭曲、距离
        return CostEvaluator(loads, tw, dist)  # 创建并返回成本评估器

    def max_cost_evaluator(self) -> CostEvaluator:  # 获取最大成本评估器方法：使用最大惩罚值创建成本评估器
        """
        Get a cost evaluator using the maximum penalty value.
        """
        penalties = np.full_like(self._penalties, self._params.max_penalty)  # 创建所有值为最大惩罚值的数组
        *loads, tw, dist = penalties  # 解包惩罚值
        return CostEvaluator(loads, tw, dist)  # 创建并返回成本评估器
