import pathlib  # 路径处理库
from collections import defaultdict  # 默认字典，用于分组
from itertools import count, pairwise  # 计数器和成对迭代器
from numbers import Number  # 数字基类，用于类型检查
from typing import Callable  # 可调用类型提示
from warnings import warn  # 警告模块

import numpy as np  # 数值计算库
import vrplib  # VRPLIB格式解析库
#读取问题实例文件
from pyvrp._pyvrp import (
    Client,  # 客户类（C++绑定）
    ClientGroup,  # 客户组类（C++绑定）
    Depot,  # 仓库类（C++绑定）
    ProblemData,  # 问题数据类（C++绑定）
    Route,  # 路线类（C++绑定）
    Solution,  # 解类（C++绑定）
    Trip,  # 行程类（C++绑定）
    VehicleType,  # 车辆类型类（C++绑定）
)
from pyvrp.constants import MAX_VALUE  # 最大常量值
from pyvrp.exceptions import ScalingWarning  # 缩放警告异常类

_RoundingFunc = Callable[[np.ndarray], np.ndarray]  # 类型别名：舍入函数类型

_INT_MAX = np.iinfo(np.int64).max  # 64位整数最大值
_UINT_MAX = np.iinfo(np.uint64).max  # 64位无符号整数最大值


ROUND_FUNCS: dict[str, _RoundingFunc] = {  # 预定义的舍入函数字典
    "round": lambda vals: np.round(vals).astype(np.int64),  # 四舍五入到最近整数
    "trunc": lambda vals: vals.astype(np.int64),  # 截断为整数
    "dimacs": lambda vals: (10 * vals).astype(np.int64),  # DIMACS格式：乘以10后截断
    "exact": lambda vals: np.round(1_000 * vals).astype(np.int64),  # 精确格式：乘以1000后四舍五入
    "none": lambda vals: vals,  # 不进行舍入
}


def read(  # 读取函数：从VRPLIB格式文件读取问题实例
    where: str | pathlib.Path,  # 文件路径，可以是字符串或Path对象
    round_func: str | _RoundingFunc = "none",  # 舍入函数，可以是字符串名称或函数对象，默认"none"
) -> ProblemData:  # 返回ProblemData对象
    """
    Reads the ``VRPLIB`` file at the given location, and returns a
    :class:`~pyvrp._pyvrp.ProblemData` instance.

    .. note::

       See the
       :doc:`VRPLIB format explanation <../dev/supported_vrplib_fields>` page
       for more details.

    Parameters
    ----------
    where
        File location to read. Assumes the data on the given location is in
        ``VRPLIB`` format.
    round_func
        Optional rounding function that is applied to all data values in the
        instance. This can either be a function or a string:

            * ``'round'`` rounds the values to the nearest integer;
            * ``'trunc'`` truncates the values to an integer;
            * ``'dimacs'`` scales by 10 and truncates the values to an integer;
            * ``'exact'`` scales by 1000 and rounds to the nearest integer.
            * ``'none'`` does no rounding. This is the default.

    Raises
    ------
    TypeError
        When ``round_func`` does not name a rounding function, or is not
        callable.
    ValueError
        When the data file does not provide information on the problem size.

    Returns
    -------
    ProblemData
        Data instance constructed from the read data.
    """
    if (key := str(round_func)) in ROUND_FUNCS:  # 如果舍入函数是字符串且在预定义字典中
        round_func = ROUND_FUNCS[key]  # 获取对应的舍入函数

    if not callable(round_func):  # 如果舍入函数不可调用
        raise TypeError(  # 抛出类型错误
            f"round_func = {round_func} is not understood. Can be a function,"
            f" or one of {ROUND_FUNCS.keys()}."
        )

    parser = _InstanceParser(vrplib.read_instance(where), round_func)  # 创建实例解析器，使用vrplib读取文件
    builder = _ProblemDataBuilder(parser)  # 创建问题数据构建器
    return builder.data()  # 构建并返回ProblemData对象


def read_solution(where: str | pathlib.Path, data: ProblemData) -> Solution:  # 读取解函数：从VRPLIB格式文件读取解
    """
    Reads a solution in ``VRPLIB`` format from the give file location, and
    returns the corresponding Solution object.

    Parameters
    ----------
    where
        File location to read. Assumes the solution in the file on the given
        location is in ``VRPLIB`` solution format.
    data
        Problem data instance that the solution is based on. See
        :meth:`~pyvrp.read` for details.

    Returns
    -------
    Solution
        Solution object constructed from the read data.
    """
    sol = vrplib.read_solution(str(where))  # 使用vrplib读取解决方案文件

    # We assume that the routes are listed in order of vehicle types as
    # determined by ``read()``. We particularly rely on the indices ``read()``
    # encodes in the vehicle type's name to map between vehicles and types.
    # 我们假设路线按照`read()`确定的车辆类型顺序列出。我们特别依赖`read()`编码在车辆类型名称中的索引来映射车辆和类型。
    veh2type = np.zeros((data.num_vehicles,), dtype=int)  # 创建车辆到类型映射的数组
    for idx, veh_type in enumerate(data.vehicle_types()):  # 遍历所有车辆类型
        idcs = list(map(int, veh_type.name.split(",")))  # 解析车辆类型名称中的车辆索引
        veh2type[idcs] = idx  # 将车辆索引映射到类型索引

    routes = []  # 存储所有路线
    for idx, route in enumerate(sol["routes"]):  # 遍历解决方案中的每条路线
        if not route:  # 如果路线为空则跳过
            continue

        route_visits = np.array(route, dtype=int)  # 将路线访问点转换为numpy数组
        depot_idcs = np.flatnonzero(route_visits < data.num_depots)  # 找到路线中所有仓库索引的位置

        trip_visits = np.split(route_visits, depot_idcs)  # 按仓库位置分割路线为多个行程
        trip_visits = [
            # These visits include the reload depots for later trips as the
            # first trip visit, which we need to skip.
            # 这些访问点包含了后续行程的重新装载仓库作为第一个行程访问点，我们需要跳过它。
            trip_visits[trip_idx > 0 :]  # 对于非第一个行程，跳过第一个重新装载仓库
            for trip_idx, trip_visits in enumerate(trip_visits)  # 遍历所有行程
        ]

        veh_type = data.vehicle_type(veh2type[idx])  # 获取当前车辆的车辆类型
        depots = [
            veh_type.start_depot,  # 起始仓库
            *route_visits[depot_idcs],  # 路线中的所有仓库访问点
            veh_type.end_depot,  # 结束仓库
        ]

        trips = [
            Trip(data, visits, veh2type[idx], start, end)  # 为每对仓库之间的访问点创建行程
            for visits, (start, end) in zip(trip_visits, pairwise(depots))  # 将行程访问点与仓库对配对
        ]

        routes.append(Route(data, trips, veh2type[idx]))  # 将多个行程组合成一条路线并添加到路线列表

    return Solution(data, routes)  # 返回解决方案对象


class _InstanceParser:
    """
    read() helper that parses VRPLIB data into meaningful parts for further
    processing.
    read()的辅助类，将VRPLIB数据解析为有意义的部分以供进一步处理。
    """

    def __init__(self, instance: dict, round_func: _RoundingFunc):
        self.instance = instance  # VRPLIB实例数据字典
        self.round_func = round_func  # 舍入函数

    @property
    def num_locations(self) -> int:
        return self.instance["dimension"]  # 返回总位置数（仓库+客户）

    @property
    def num_depots(self) -> int:
        return self.instance.get("depot", np.array([0])).size  # 返回仓库数量，默认为1

    @property
    def num_clients(self) -> int:
        return self.num_locations - self.num_depots  # 返回客户数量 = 总位置数 - 仓库数

    @property
    def num_vehicles(self) -> int:
        return self.instance.get("vehicles", self.num_locations - 1)  # 返回车辆数量，默认为总位置数-1

    def type(self) -> str:
        return self.instance.get("type", "")  # 返回问题类型字符串

    def edge_weight(self) -> np.ndarray:
        return self.round_func(self.instance["edge_weight"])  # 返回边权重矩阵并应用舍入

    def depot_idcs(self) -> np.ndarray:
        return self.instance.get("depot", np.array([0]))  # 返回仓库索引数组，默认为[0]

    def backhauls(self) -> np.ndarray:
        if "backhaul" not in self.instance:  # 如果没有回程数据
            return np.zeros((self.num_locations, 1), dtype=np.int64)  # 返回全零数组

        return self.round_func(self.instance["backhaul"])  # 返回回程数据并应用舍入

    def demands(self) -> np.ndarray:
        if "demand" not in self.instance and "linehaul" not in self.instance:  # 如果没有需求和线haul数据
            return np.zeros((self.num_locations, 1), dtype=np.int64)  # 返回全零数组

        return self.round_func(  # 返回需求数据并应用舍入
            self.instance.get("demand", self.instance.get("linehaul"))  # 获取需求或线haul数据
        )

    def coords(self) -> np.ndarray:
        if "node_coord" not in self.instance:  # 如果没有坐标数据
            return np.zeros((self.num_locations, 2), dtype=np.int64)  # 返回全零坐标数组

        return self.round_func(self.instance["node_coord"])  # 返回坐标数据并应用舍入

    def service_times(self) -> np.ndarray:
        service_times = self.instance.get("service_time", 0)  # 获取服务时间，默认为0

        if isinstance(service_times, Number):  # 如果服务时间是单个数值
            # Some instances describe a uniform service time as a single value
            # that applies to all clients.
            # 有些实例将统一的服务时间描述为适用于所有客户的单个值。
            service_times = np.full(self.num_locations, service_times)  # 创建所有位置服务时间相同的数组
            service_times[: self.num_depots] = 0  # 将仓库的服务时间设为0

        return self.round_func(service_times)  # 返回服务时间并应用舍入

    def time_windows(self) -> np.ndarray:
        if "time_window" not in self.instance:  # 如果没有时间窗数据
            time_windows = np.empty((self.num_locations, 2), dtype=np.int64)  # 创建空数组
            time_windows[:, 0] = 0  # 将开始时间设为0
            time_windows[:, 1] = _INT_MAX  # 将结束时间设为最大整数
            return time_windows

        return self.round_func(self.instance["time_window"])  # 返回时间窗数据并应用舍入

    def release_times(self) -> np.ndarray:
        release_times = self.instance.get("release_time", 0)  # 获取释放时间，默认为0
        shape = self.num_locations  # 数组形状为位置数量
        return self.round_func(np.broadcast_to(release_times, shape))  # 将释放时间广播到所有位置并应用舍入

    def reload_depots(self) -> list[tuple[int, ...]]:
        if "vehicles_reload_depot" not in self.instance:  # 如果没有重新装载仓库数据
            return [tuple() for _ in range(self.num_vehicles)]  # 返回每辆车空元组列表

        reload_depots = self.instance["vehicles_reload_depot"]  # 获取重新装载仓库数据

        if isinstance(reload_depots[0], Number):  # 如果每辆车只有一个重新装载仓库
            # Some instances describe only one reload depot per vehicle, so
            # we first cast it to a 2D array.
            # 有些实例每辆车只描述一个重新装载仓库，所以我们先将其转换为2D数组。
            reload_depots = np.atleast_2d(reload_depots).T  # 转换为2D数组并转置

        return [tuple(idx - 1 for idx in depots) for depots in reload_depots]  # 返回重新装载仓库索引元组列表（转换为0-based索引）

    def prizes(self) -> np.ndarray:
        if "prize" not in self.instance:  # 如果没有奖金数据
            return np.zeros(self.num_locations, dtype=np.int64)  # 返回全零数组

        return self.round_func(self.instance["prize"])  # 返回奖金数据并应用舍入

    def capacities(self) -> np.ndarray:
        if "capacity" not in self.instance:  # 如果没有容量数据
            return np.full(self.num_vehicles, _INT_MAX)  # 返回所有车辆容量为最大整数的数组

        capacities = self.instance["capacity"]  # 获取容量数据

        if isinstance(capacities, Number):  # 如果容量是单个数值
            # Some instances describe a uniform capacity as a single value
            # that applies to all vehicles.
            # 有些实例将统一的容量描述为适用于所有车辆的单个值。
            capacities = np.full(self.num_vehicles, capacities)  # 创建所有车辆容量相同的数组

        return self.round_func(capacities)  # 返回容量数据并应用舍入

    def allowed_clients(self) -> list[tuple[int, ...]]:
        if "vehicles_allowed_clients" not in self.instance:  # 如果没有允许客户数据
            client_idcs = tuple(range(self.num_depots, self.num_locations))  # 所有客户索引
            return [client_idcs for _ in range(self.num_vehicles)]  # 每辆车都可以访问所有客户

        allowed_clients = self.instance["vehicles_allowed_clients"]  # 获取允许客户数据

        if isinstance(allowed_clients[0], Number):  # 如果每辆车只有一个允许客户
            # Some instances describe only one allowed client per vehicle, so
            # we first cast it to a 2D array.
            # 有些实例每辆车只描述一个允许客户，所以我们先将其转换为2D数组。
            allowed_clients = np.atleast_2d(allowed_clients).T  # 转换为2D数组并转置

        return [
            tuple(idx - 1 for idx in clients) for clients in allowed_clients  # 返回允许客户索引元组列表（转换为0-based索引）
        ]

    def vehicles_depots(self) -> np.ndarray:
        if "vehicles_depot" not in self.instance:  # 如果没有车辆仓库数据
            depot_idcs = self.depot_idcs()  # 获取仓库索引
            return np.full(self.num_vehicles, depot_idcs[0])  # 所有车辆使用第一个仓库

        return self.instance["vehicles_depot"] - 1  # 返回车辆仓库索引并转换为0-based索引

    def max_distances(self) -> np.ndarray:
        if "vehicles_max_distance" not in self.instance:  # 如果没有最大距离数据
            return np.full(self.num_vehicles, _INT_MAX)  # 返回所有车辆最大距离为最大整数的数组

        max_distances = self.instance["vehicles_max_distance"]  # 获取最大距离数据
        shape = self.num_vehicles  # 数组形状为车辆数量
        return self.round_func(np.broadcast_to(max_distances, shape))  # 将最大距离广播到所有车辆并应用舍入

    def shift_durations(self) -> np.ndarray:
        # We call this field shift duration instead of a hard maximum duration.
        # 我们将此字段称为班次持续时间而不是硬最大持续时间。
        if "vehicles_max_duration" not in self.instance:  # 如果没有最大持续时间数据
            return np.full(self.num_vehicles, _INT_MAX)  # 返回所有车辆班次持续时间为最大整数的数组

        max_durations = self.instance["vehicles_max_duration"]  # 获取最大持续时间数据
        shape = self.num_vehicles  # 数组形状为车辆数量
        return self.round_func(np.broadcast_to(max_durations, shape))  # 将最大持续时间广播到所有车辆并应用舍入

    def max_reloads(self) -> np.ndarray:
        max_reloads = self.instance.get("vehicles_max_reloads", _UINT_MAX)  # 获取最大重新装载次数，默认为最大无符号整数
        return np.broadcast_to(max_reloads, self.num_vehicles)  # 将最大重新装载次数广播到所有车辆

    def fixed_costs(self) -> np.ndarray:
        fixed_costs = self.instance.get("vehicles_fixed_cost", 0)  # 获取固定成本，默认为0
        return self.round_func(np.broadcast_to(fixed_costs, self.num_vehicles))  # 将固定成本广播到所有车辆并应用舍入

    def unit_distance_costs(self) -> np.ndarray:
        # Unit distance costs are unrounded to prevent double scaling in the
        # total distance cost calculation (unit_distance_cost * distance).
        # 单位距离成本不进行舍入，以防止在总距离成本计算（单位距离成本 * 距离）中出现双重缩放。
        unit_cost = self.instance.get("vehicles_unit_distance_cost", 1)  # 获取单位距离成本，默认为1
        return np.broadcast_to(unit_cost, self.num_vehicles)  # 将单位距离成本广播到所有车辆

    def mutually_exclusive_groups(self) -> list[list[int]]:
        if "mutually_exclusive_group" not in self.instance:  # 如果没有互斥组数据
            return []  # 返回空列表

        groups = self.instance["mutually_exclusive_group"]  # 获取互斥组数据

        if isinstance(groups[0], Number):  # 如果每组只有一个客户
            # Some instances describe only one client per group, so we first
            # cast it to a 2D array.
            # 有些实例每组只描述一个客户，所以我们先将其转换为2D数组。
            groups = np.atleast_2d(groups).T  # 转换为2D数组并转置

        raw_groups = [[idx - 1 for idx in group] for group in groups]  # 转换为0-based索引

        # Only keep groups if they have more than one member. Empty groups or
        # groups with one member are trivial to decide, so there is no point
        # in keeping them.
        # 只保留有多个成员的组。空组或只有一个成员的组很容易决定，因此没有必要保留它们。
        return [group for group in raw_groups if len(group) > 1]  # 返回成员数大于1的组


class _ProblemDataBuilder:
    """
    read() helper that builds a ``ProblemData`` object from the instance
    attributes of the given parser.
    read()的辅助类，从给定解析器的实例属性构建`ProblemData`对象。
    """

    def __init__(self, parser: _InstanceParser):
        self.parser = parser  # 实例解析器

    def data(self) -> ProblemData:
        clients = self._clients()  # 创建客户对象列表
        depots = self._depots()  # 创建仓库对象列表
        vehicle_types = self._vehicle_types()  # 创建车辆类型对象列表
        distance_matrices = self._distance_matrices()  # 创建距离矩阵列表
        groups = self._groups()  # 创建客户组对象列表

        return ProblemData(  # 返回问题数据对象
            clients=clients,
            depots=depots,
            vehicle_types=vehicle_types,
            distance_matrices=distance_matrices,
            # VRPLIB instances typically do not have a duration data field, and
            # instead assume duration == distance.
            # VRPLIB实例通常没有持续时间数据字段，而是假设持续时间 == 距离。
            duration_matrices=distance_matrices,  # 使用距离矩阵作为持续时间矩阵
            groups=groups,
        )

    def _depots(self) -> list[Depot]:
        num_depots = self.parser.num_depots  # 仓库数量
        depot_idcs = self.parser.depot_idcs()  # 仓库索引

        contiguous_lower_idcs = np.arange(num_depots)  # 连续的较低索引
        if num_depots == 0 or (depot_idcs != contiguous_lower_idcs).any():  # 检查仓库索引是否连续且从0开始
            msg = """
            Source file should contain at least one depot in the contiguous
            lower indices, starting from 1.
            """
            raise ValueError(msg)  # 抛出值错误

        coords = self.parser.coords()  # 坐标数据
        return [
            Depot(x=coords[idx][0], y=coords[idx][1])  # 为每个仓库创建Depot对象
            for idx in range(num_depots)
        ]

    def _clients(self) -> list[Client]:
        groups = self.parser.mutually_exclusive_groups()  # 互斥组数据
        num_locs = self.parser.num_locations  # 总位置数

        idx2group: list[int | None] = [None for _ in range(num_locs)]  # 索引到组的映射列表
        for group, members in enumerate(groups):  # 遍历所有组
            for client in members:  # 遍历组内成员
                idx2group[client] = group  # 将客户索引映射到组编号

        coords = self.parser.coords()  # 坐标数据
        demands = self.parser.demands()  # 需求数据
        backhauls = self.parser.backhauls()  # 回程数据
        service_duration = self.parser.service_times()  # 服务时间数据
        time_windows = self.parser.time_windows()  # 时间窗数据
        release_times = self.parser.release_times()  # 释放时间数据
        prizes = self.parser.prizes()  # 奖金数据
        required = np.isclose(prizes, 0)  # 奖金接近0的客户是必需的

        return [
            Client(  # 为每个客户创建Client对象
                x=coords[idx][0],  # x坐标
                y=coords[idx][1],  # y坐标
                delivery=np.atleast_1d(demands[idx]),  # 送货量，确保至少1维
                pickup=np.atleast_1d(backhauls[idx]),  # 取货量，确保至少1维
                service_duration=service_duration[idx],  # 服务时间
                tw_early=time_windows[idx][0],  # 时间窗开始时间
                tw_late=time_windows[idx][1],  # 时间窗结束时间
                release_time=release_times[idx],  # 释放时间
                prize=prizes[idx],  # 奖金
                required=required[idx] and idx2group[idx] is None,  # 必需的且不在互斥组中
                group=idx2group[idx],  # 所属组
            )
            for idx in range(self.parser.num_depots, num_locs)  # 遍历所有客户（跳过仓库）
        ]

    def _vehicle_types(self) -> list[VehicleType]:
        num_vehicles = self.parser.num_vehicles  # 车辆数量
        vehicles_data = (  # 车辆数据元组
            self.parser.capacities(),  # 容量数据
            self.parser.allowed_clients(),  # 允许客户数据
            self.parser.reload_depots(),  # 重新装载仓库数据
            self.parser.vehicles_depots(),  # 车辆仓库数据
            self.parser.max_distances(),  # 最大距离数据
            self.parser.shift_durations(),  # 班次持续时间数据
            self.parser.max_reloads(),  # 最大重新装载次数数据
            self.parser.fixed_costs(),  # 固定成本数据
            self.parser.unit_distance_costs(),  # 单位距离成本数据
        )

        if any(len(attr) != num_vehicles for attr in vehicles_data):  # 检查所有车辆数据属性长度是否等于车辆数量
            msg = """
            The number of elements in the vehicles data attributes should be
            equal to the number of vehicles in the problem.
            """
            raise ValueError(msg)  # 抛出值错误

        # VRPLIB instances includes data for each available vehicle. We group
        # vehicles by their attributes to create unique vehicle types.
        # VRPLIB实例包含每辆可用车辆的数据。我们根据车辆属性对车辆进行分组以创建唯一的车辆类型。
        type2idcs = defaultdict(list)  # 创建默认字典，键为属性元组，值为车辆索引列表
        for vehicle, (capacity, *veh_type) in enumerate(zip(*vehicles_data)):  # 遍历所有车辆
            capacity = tuple(np.atleast_1d(capacity))  # 将容量转换为元组，确保至少1维
            type2idcs[(capacity, *veh_type)].append(vehicle)  # 将车辆索引添加到对应属性组的列表中

        client2profile = self._allowed2profile()  # 获取允许客户到配置索引的映射
        time_windows = self.parser.time_windows()  # 时间窗数据

        vehicle_types = []  # 车辆类型列表
        for attributes, vehicles in type2idcs.items():  # 遍历每种属性组合及其对应的车辆
            (
                capacity,
                clients,
                reloads,
                depot,
                max_distance,
                shift_duration,
                max_reloads,
                fixed_cost,
                unit_distance_cost,
            ) = attributes  # 解包属性元组

            vehicle_type = VehicleType(  # 创建车辆类型对象
                num_available=len(vehicles),  # 可用车辆数量
                capacity=capacity,  # 容量
                start_depot=depot,  # 起始仓库
                end_depot=depot,  # 结束仓库
                fixed_cost=fixed_cost,  # 固定成本
                # The literature specifies depot time windows. We instead set
                # those on the vehicles.
                # 文献指定仓库时间窗。我们改为在车辆上设置它们。
                tw_early=time_windows[depot][0],  # 时间窗开始时间（使用仓库的时间窗）
                tw_late=time_windows[depot][1],  # 时间窗结束时间（使用仓库的时间窗）
                shift_duration=shift_duration,  # 班次持续时间
                max_distance=max_distance,  # 最大距离
                unit_distance_cost=unit_distance_cost,  # 单位距离成本
                profile=client2profile[clients],  # 配置索引
                reload_depots=reloads,  # 重新装载仓库
                max_reloads=max_reloads,  # 最大重新装载次数
                # A bit hacky, but this csv-like name is really useful to track
                # the actual vehicles that make up this vehicle type.
                # 有点取巧，但这个类似csv的名称对于跟踪构成此车辆类型的实际车辆非常有用。
                name=",".join(map(str, vehicles)),  # 车辆索引的逗号分隔字符串作为名称
            )
            vehicle_types.append(vehicle_type)  # 将车辆类型添加到列表

        return vehicle_types  # 返回车辆类型列表

    def _distance_matrices(self) -> list[np.ndarray]:
        distances = self.parser.edge_weight()  # 边权重矩阵

        if self.parser.type() == "VRPB":  # 如果是VRPB问题类型
            # In VRPB, linehauls must be served before backhauls. This can be
            # enforced by setting a high value for the distance/duration from
            # depot to backhaul (forcing linehaul to be served first) and a
            # large value from backhaul to linehaul (avoiding linehaul after
            # backhaul clients).
            # 在VRPB中，线haul必须在回程之前服务。这可以通过设置从仓库到回程的高距离/持续时间值
            # （强制先服务线haul）以及从回程到线haul的大值（避免在回程客户之后服务线haul）来强制执行。
            linehaul = self.parser.demands() > 0  # 线haul客户（需求>0）
            backhaul = self.parser.backhauls() > 0  # 回程客户（回程>0）
            distances[0, backhaul] = MAX_VALUE  # 设置从仓库到回程客户的距离为最大值
            distances[np.ix_(backhaul, linehaul)] = MAX_VALUE  # 设置从回程客户到线haul客户的距离为最大值

        allowed2profile = self._allowed2profile()  # 获取允许客户到配置索引的映射
        num_profiles = len(allowed2profile)  # 配置数量
        dist_mats = [distances.copy() for _ in range(num_profiles)]  # 为每个配置创建距离矩阵副本

        for allowed_clients, type_idx in allowed2profile.items():  # 遍历每个配置及其索引
            if len(allowed_clients) == self.parser.num_clients:  # 如果允许所有客户
                # True if this feature is unused, and the distance matrix for
                # this profile does not have to be modified.
                # 如果此功能未使用，并且此配置的距离矩阵不需要修改，则为True。
                continue

            num_depots = self.parser.num_depots  # 仓库数量
            num_locations = self.parser.num_locations  # 总位置数

            # This profile is allowed to visit every depot and all clients in
            # its allowed clients section.
            # 此配置允许访问每个仓库及其允许客户部分中的所有客户。
            allowed = np.zeros((num_locations,), dtype=bool)  # 创建布尔数组表示允许访问的位置
            allowed[:num_depots] = True  # 所有仓库都允许访问
            allowed[list(allowed_clients)] = True  # 允许的客户设置为True

            # Some dtype trickery to ensure the MAX_VALUE assignment below does
            # not overflow.
            # 一些数据类型技巧，以确保下面的MAX_VALUE赋值不会溢出。
            dtype = np.promote_types(dist_mats[type_idx].dtype, np.int64)  # 提升数据类型以避免溢出
            dist_mats[type_idx] = dist_mats[type_idx].astype(dtype)  # 转换数据类型

            # Set MAX_VALUE to and from disallowed clients, preventing this
            # vehicle type from serving them.
            # 将不允许的客户的距离设置为MAX_VALUE，防止此车辆类型为他们服务。
            dist_mat = dist_mats[type_idx]  # 获取当前配置的距离矩阵
            dist_mat[:, ~allowed] = dist_mat[~allowed, :] = MAX_VALUE  # 设置不允许访问的位置之间的距离为最大值
            np.fill_diagonal(dist_mat, 0)  # 将对角线（自己到自己的距离）设置为0

        if any(dist.max() > MAX_VALUE for dist in dist_mats):  # 检查是否有距离矩阵的最大值超过MAX_VALUE
            msg = """
            The maximum distance value is very large. This might impact
            numerical stability. Consider rescaling your input data.
            """
            warn(msg, ScalingWarning)  # 发出缩放警告

        return dist_mats  # 返回距离矩阵列表

    def _groups(self) -> list[ClientGroup]:
        groups = self.parser.mutually_exclusive_groups()  # 互斥组数据
        return [ClientGroup(group) for group in groups]  # 为每个组创建ClientGroup对象

    def _allowed2profile(self) -> dict[tuple[int, ...], int]:
        allowed_clients2profile_idx = {}  # 允许客户到配置索引的映射字典
        profile_idx = count(0)  # 配置索引计数器
        for clients in self.parser.allowed_clients():  # 遍历每辆车的允许客户
            if clients not in allowed_clients2profile_idx:  # 如果此允许客户集合尚未分配配置索引
                allowed_clients2profile_idx[clients] = next(profile_idx)  # 分配新的配置索引

        return allowed_clients2profile_idx  # 返回允许客户到配置索引的映射