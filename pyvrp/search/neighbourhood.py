from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from pyvrp import ProblemData


@dataclass
class NeighbourhoodParams:
    """
    Configuration for calculating a granular neighbourhood.

    Attributes
    ----------
    weight_wait_time
        Penalty weight given to the minimum wait time aspect of the proximity
        calculation. A large wait time indicates the clients are far apart
        in duration/time.
    weight_time_warp
        Penalty weight given to the minimum time warp aspect of the proximity
        calculation. A large time warp indicates the clients are far apart in
        duration/time.
    num_neighbours
        Number of other clients that are in each client's granular
        neighbourhood. This parameter determines the size of the overall
        neighbourhood.
    symmetric_proximity
        Whether to calculate a symmetric proximity matrix. This ensures edge
        :math:`(i, j)` is given the same weight as :math:`(j, i)`.
    symmetric_neighbours
        Whether to symmetrise the neighbourhood structure. This ensures that
        when edge :math:`(i, j)` is in, then so is :math:`(j, i)`. Note that
        this is *not* the same as ``symmetric_proximity``.

    Raises
    ------
    ValueError
        When ``num_neighbours`` is non-positive.
    """
    # 配置用于计算细粒度邻域的参数类。

    weight_wait_time: float = 0.2  # 等待时间在邻近度计算中的权重系数
    weight_time_warp: float = 1.0  # 时间扭曲在邻近度计算中的权重系数
    num_neighbours: int = 60  # 每个客户的邻域中其他客户的数量，控制邻域大小
    symmetric_proximity: bool = True  # 是否计算对称的邻近度矩阵
    symmetric_neighbours: bool = False  # 是否对称化邻域结构

    def __post_init__(self):
        if self.num_neighbours <= 0:
            raise ValueError("num_neighbours <= 0 not understood.")  # 验证邻域数量必须为正数


def compute_neighbours(
    data: ProblemData, params: NeighbourhoodParams = NeighbourhoodParams()
) -> list[list[int]]:
    """
    Computes neighbours defining the neighbourhood for a problem instance.

    Parameters
    ----------
    data
        ProblemData for which to compute the neighbourhood.
    params
        NeighbourhoodParams that define how the neighbourhood is computed.

    Returns
    -------
    list
        A list of list of integers representing the neighbours for each client.
        The first lists in the lower indices are associated with the depots and
        are all empty.
    """
    # 为问题实例计算定义邻域的邻居列表
    proximity = _compute_proximity(
        data,
        params.weight_wait_time,
        params.weight_time_warp,
    )

    if params.symmetric_proximity:
        proximity = np.minimum(proximity, proximity.T)  # 对称化邻近度矩阵：取(i,j)和(j,i)的最小值

    for group in data.groups():
        if group.mutually_exclusive:
            # Clients in mutually exclusive groups cannot neighbour each other,
            # since only one of them can be in the solution at any given time.
            # We use max float, not infty, to ensure these clients are ordered
            # before the depots: we want to avoid same group neighbours, but it
            # is not problematic if we need to have them.
            idcs = np.ix_(group.clients, group.clients)
            proximity[idcs] = np.finfo(np.float64).max  # 互斥组内的客户间设置最大邻近度，避免成为邻居
    # 互斥组中的客户不能互为邻居，因为在任何给定时间解中只能包含其中一个。使用最大浮点数而非无穷大，
    # 以确保这些客户在排序时位于仓库之前：我们希望避免同组邻居，但必要时也可以接受。

    np.fill_diagonal(proximity, np.inf)  # 将对角线设为无穷大，避免客户成为自己的邻居
    proximity[: data.num_depots, :] = np.inf  # 仓库行设为无穷大：仓库没有邻居
    proximity[:, : data.num_depots] = np.inf  # 仓库列设为无穷大：客户不将仓库作为邻居

    k = min(params.num_neighbours, data.num_clients - 1)  # 实际考虑的邻居数，排除自己
    top_k = np.argsort(proximity, axis=1, kind="stable")[data.num_depots :, :k]  # 为每个客户选择k个最邻近的客户
    #argsort按行排序，返回索引而不是值，使用稳定排序(kind="stable")确保相同值的顺序不变
    if not params.symmetric_neighbours:
        return [[] for _ in range(data.num_depots)] + top_k.tolist()  # 非对称邻居：返回仓库的空列表和客户的邻居列表

    # Construct a symmetric adjacency matrix and return the adjacent clients
    # as the neighbourhood structure.
    # 构建对称邻接矩阵并将相邻客户作为邻域结构返回
    adj = np.zeros_like(proximity, dtype=bool)  # 创建布尔型邻接矩阵
    rows = np.expand_dims(np.arange(data.num_depots, len(proximity)), axis=1)  # 客户行的索引
    adj[rows, top_k] = True  # 将每个客户的top_k邻居标记为True
    adj = adj | adj.transpose()  # 对称化邻接矩阵：如果(i,j)为True则(j,i)也为True

    return [np.flatnonzero(row).tolist() for row in adj]  # 将邻接矩阵转换为每个位置的邻居列表


def _compute_proximity(
    data: ProblemData, weight_wait_time: float, weight_time_warp: float
) -> np.ndarray[float]:
    """
    Computes proximity for neighborhood. Proximity is based on [1]_, with
    modification for additional VRP variants.

    Parameters
    ----------
    data
        ProblemData for which to compute proximity.
    params
        NeighbourhoodParams that define how proximity is computed.

    Returns
    -------
    np.ndarray[float]
        An array of size :py:attr:`~pyvrp._pyvrp.ProblemData.num_locations`
        by :py:attr:`~pyvrp._pyvrp.ProblemData.num_locations`.

    References
    ----------
    .. [1] Vidal, T., Crainic, T. G., Gendreau, M., and Prins, C. (2013). A
           hybrid genetic algorithm with adaptive diversity management for a
           large class of vehicle routing problems with time-windows.
           *Computers & Operations Research*, 40(1), 475 - 489.
    """
    # 计算用于邻域构建的邻近度矩阵，基于文献[1]的方法并针对VRP变体进行了修改
    early = np.zeros((data.num_locations,), dtype=float)  # 创建时间窗开始时间数组
    early[data.num_depots :] = np.asarray([c.tw_early for c in data.clients()])  # 填充客户时间窗开始时间

    late = np.zeros_like(early)  # 创建时间窗结束时间数组
    late[data.num_depots :] = np.asarray([c.tw_late for c in data.clients()])  # 填充客户时间窗结束时间

    service = np.zeros_like(early)  # 创建服务时长数组
    service[data.num_depots :] = [c.service_duration for c in data.clients()]  # 填充客户服务时长

    prize = np.zeros_like(early)  # 创建奖励值数组
    prize[data.num_depots :] = [client.prize for client in data.clients()]  # 填充客户奖励值

    # We first determine the elementwise minimum cost across all vehicle types.
    # This is the cheapest way any edge can be traversed.
    # 首先确定所有车辆类型中的逐元素最小成本，这是任何边可以被遍历的最便宜方式
    distances = data.distance_matrices()  # 获取距离矩阵字典
    durations = data.duration_matrices()  # 获取时间矩阵字典
    unique_edge_costs = {
        (
            veh_type.unit_distance_cost,
            veh_type.unit_duration_cost,
            veh_type.profile,
        )
        for veh_type in data.vehicle_types()
    }  # 收集唯一的车辆成本配置（单位距离成本、单位时间成本、档案类型）

    first, *rest = unique_edge_costs  # 解包唯一成本配置
    unit_dist, unit_dur, prof = first  # 获取第一个配置
    edge_costs = unit_dist * distances[prof] + unit_dur * durations[prof]  # 计算初始边成本
    for unit_dist, unit_dur, prof in rest:
        mat = unit_dist * distances[prof] + unit_dur * durations[prof]  # 计算当前配置的边成本
        np.minimum(edge_costs, mat, out=edge_costs)  # 使用不同车辆类型计算边成本
        # 取所有车辆类型中的最小值作为最终边成本

    # Minimum wait time and time warp of visiting j directly after i.
    # 计算从i直接访问j的最小等待时间和时间扭曲
    min_duration = np.minimum.reduce(durations)  # 获取所有档案的最小持续时间矩阵，i到j的最短时间
    # j的最早开始时间-最短持续时间-服务时间-i的最晚开始时间
    min_wait = early[None, :] - min_duration - service[:, None] - late[:, None]  # 计算最小等待时间
    # i的最早开始时间+服务时间+最短持续时间-j的最晚开始时间
    min_tw = early[:, None] + service[:, None] + min_duration - late[None, :]  # 计算最小时间扭曲

    # Proximity is based on edge costs (and rewards) and penalties for known
    # time-related violations.
    # 邻近度基于边成本（和奖励）以及对已知时间违规的惩罚
    # TODO:不同的临近度规则
    # 邻近度 = 边成本 - 目标奖励 + 等待时间惩罚 + 时间扭曲惩罚
    edge_costs = edge_costs.astype(float)  # 转换为浮点类型
    edge_costs -= prize[None, :]  # 减去目标节点的奖励值（奖励越高，邻近度越小）
    edge_costs += weight_wait_time * np.maximum(min_wait, 0)  # 添加等待时间惩罚
    edge_costs += weight_time_warp * np.maximum(min_tw, 0)  # 添加时间扭曲惩罚

    return edge_costs  # 返回最终的邻近度矩阵