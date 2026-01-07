from __future__ import annotations  # 启用延迟类型注解，允许使用字符串类型提示
#问题建模接口，用于构建 VRP 实例
from typing import TYPE_CHECKING, Sequence  # TYPE_CHECKING用于类型检查时的导入，Sequence用于类型提示
from warnings import warn  # 警告模块，用于发出警告信息

import numpy as np  # 数值计算库，用于数组操作和数学运算

from pyvrp._pyvrp import (
    Client,  # 客户类（C++绑定）
    ClientGroup,  # 客户组类（C++绑定）
    Depot,  # 仓库类（C++绑定）
    ProblemData,  # 问题数据类（C++绑定）
    VehicleType,  # 车辆类型类（C++绑定）
)
from pyvrp.constants import MAX_VALUE  # 最大常量值，用于表示缺失的边
from pyvrp.exceptions import ScalingWarning  # 缩放警告异常类
from pyvrp.solve import SolveParams, solve  # 求解参数类和求解函数

if TYPE_CHECKING:  # 仅在类型检查时导入，避免运行时循环导入
    from pyvrp.Result import Result  # 结果类
    from pyvrp.stop import StoppingCriterion  # 停止准则类


class Edge:  # 边类：存储连接两个位置的边，包含距离和持续时间信息
    """
    Stores an edge connecting two locations.

    Raises
    ------
    ValueError
        When either distance or duration is a negative value, or when self
        loops have nonzero distance or duration values.
    """

    __slots__ = ["distance", "duration", "frm", "to"]  # 使用__slots__优化内存使用，限制只能有这四个属性

    def __init__(
        self,
        frm: Client | Depot,  # 起始位置（客户或仓库）
        to: Client | Depot,  # 目标位置（客户或仓库）
        distance: int,  # 边的距离值
        duration: int,  # 边的持续时间值
    ):
        if distance < 0 or duration < 0:  # 验证距离和持续时间不能为负
            raise ValueError("Cannot have negative edge distance or duration.")

        if id(frm) == id(to) and (distance != 0 or duration != 0):  # 自环（从自己到自己的边）的距离和持续时间必须为0
            raise ValueError("A self loop must have 0 distance and duration.")

        if max(distance, duration) > MAX_VALUE:  # 检查值是否过大，可能影响数值稳定性
            msg = """
            The given distance or duration value is very large. This may impact
            numerical stability. Consider rescaling your input data.
            """
            warn(msg, ScalingWarning)  # 发出缩放警告

        self.frm = frm  # 存储起始位置
        self.to = to  # 存储目标位置
        self.distance = distance  # 存储距离
        self.duration = duration  # 存储持续时间


class Profile:  # 路径规划配置类：存储一个路线配置，包含完整的距离和持续时间矩阵
    """
    Stores a routing profile.

    A routing profile is a collection of edges with distance and duration
    attributes that together define a complete distance and duration matrix.
    These can be used to model, for example, the road uses of different types
    of vehicles, like trucks, cars, or bicyclists. Each
    :class:`~pyvrp._pyvrp.VehicleType` is associated with a routing profile.
    """

    edges: list[Edge]  # 边列表，定义该配置的所有边
    name: str  # 配置名称

    def __init__(self, *, name: str = ""):  # 构造函数，name必须是关键字参数
        self.edges = []  # 初始化空的边列表
        self.name = name  # 设置配置名称

    def add_edge(
        self,
        frm: Client | Depot,  # 起始位置
        to: Client | Depot,  # 目标位置
        distance: int,  # 距离值
        duration: int = 0,  # 持续时间值，默认为0
    ) -> Edge:  # 返回创建的边对象
        """
        Adds a new edge to this routing profile.
        """
        edge = Edge(frm, to, distance, duration)  # 创建边对象
        self.edges.append(edge)  # 将边添加到列表中
        return edge  # 返回创建的边

    def __str__(self) -> str:  # 字符串表示方法
        return self.name  # 返回配置名称


class Model:  # 模型类：提供简单的接口用于建模车辆路径问题
    """
    A simple interface for modelling vehicle routing problems with PyVRP.
    """

    def __init__(self) -> None:  # 构造函数：初始化空的模型
        self._clients: list[Client] = []  # 客户列表，存储所有客户对象
        self._depots: list[Depot] = []  # 仓库列表，存储所有仓库对象
        self._edges: list[Edge] = []  # 基础边列表，存储所有位置的边（用于所有配置）
        self._groups: list[ClientGroup] = []  # 客户组列表，存储客户组对象
        self._profiles: list[Profile] = []  # 路径规划配置列表，存储不同的路线配置
        self._vehicle_types: list[VehicleType] = []  # 车辆类型列表，存储不同类型的车辆

    @property
    def clients(self) -> list[Client]:  # 客户属性访问器：返回模型中所有客户
        """
        Returns all clients currently in the model.
        """
        return self._clients  # 返回客户列表

    @property
    def depots(self) -> list[Depot]:  # 仓库属性访问器：返回模型中所有仓库
        """
        Returns all depots currently in the model.
        """
        return self._depots  # 返回仓库列表

    @property
    def locations(self) -> list[Client | Depot]:  # 位置属性访问器：返回所有位置（仓库+客户）
        """
        Returns all locations (depots and clients) in the current model. The
        clients in the routes of the solution returned by :meth:`~solve` can be
        used to index these locations.
        """
        return self._depots + self._clients  # 返回仓库列表和客户列表的拼接（仓库在前）

    @property
    def groups(self) -> list[ClientGroup]:  # 客户组属性访问器：返回模型中所有客户组
        """
        Returns all client groups currently in the model.
        """
        return self._groups  # 返回客户组列表

    @property
    def profiles(self) -> list[Profile]:  # 路径规划配置属性访问器：返回模型中所有配置
        """
        Returns all routing profiles currently in the model.
        """
        return self._profiles  # 返回配置列表

    @property
    def vehicle_types(self) -> list[VehicleType]:  # 车辆类型属性访问器：返回模型中所有车辆类型
        """
        Returns the vehicle types in the current model. The routes of the
        solution returned by :meth:`~solve` have a property
        :meth:`~pyvrp._pyvrp.Route.vehicle_type()` that can be used to index
        these vehicle types.
        """
        return self._vehicle_types  # 返回车辆类型列表

    @classmethod
    def from_data(cls, data: ProblemData) -> "Model":  # 类方法：从ProblemData对象构建Model实例
        """
        Constructs a model instance from the given data.

        .. tip::
           Only use this method if you intend to change the data using the
           model interface. If you only want to solve the given data instance,
           it is faster to directly call :meth:`~pyvrp.solve.solve`.

        Parameters
        ----------
        data
            Problem data to feed into the model.

        Returns
        -------
        Model
            A model instance representing the given data.
        """
        depots = data.depots()  # 获取所有仓库
        clients = data.clients()  # 获取所有客户
        locs = depots + clients  # 合并所有位置（仓库在前，客户在后）

        profiles = [Profile() for _ in range(data.num_profiles)]  # 为每个配置创建Profile对象
        for idx, profile in enumerate(profiles):  # 遍历每个配置
            distances = data.distance_matrix(profile=idx)  # 获取该配置的距离矩阵
            durations = data.duration_matrix(profile=idx)  # 获取该配置的持续时间矩阵
            profile.edges = [  # 为该配置创建所有边
                Edge(
                    frm=locs[frm],  # 起始位置对象
                    to=locs[to],  # 目标位置对象
                    distance=distances[frm, to],  # 从距离矩阵获取距离值
                    duration=durations[frm, to],  # 从持续时间矩阵获取持续时间值
                )
                for frm in range(data.num_locations)  # 遍历所有起始位置
                for to in range(data.num_locations)  # 遍历所有目标位置
            ]

        self = Model()  # 创建新的Model实例
        self._clients = clients  # 设置客户列表
        self._depots = depots  # 设置仓库列表
        self._groups = data.groups()  # 设置客户组列表
        self._profiles = profiles  # 设置配置列表
        self._vehicle_types = data.vehicle_types()  # 设置车辆类型列表

        return self  # 返回构建的Model实例

    def add_client(  # 添加客户方法：向模型中添加一个客户
        self,
        x: float,  # 客户的横坐标
        y: float,  # 客户的纵坐标
        delivery: int | list[int] = [],  # 配送量（可以是单个值或列表，支持多维度）
        pickup: int | list[int] = [],  # 拾取量（可以是单个值或列表，支持多维度）
        service_duration: int = 0,  # 服务持续时间，车辆在该客户处需要花费的时间
        tw_early: int = 0,  # 时间窗最早时间，可以开始服务的最早时间
        tw_late: int = np.iinfo(np.int64).max,  # 时间窗最晚时间，可以开始服务的最晚时间
        release_time: int = 0,  # 释放时间，车辆可以离开仓库前往该客户的最早时间
        prize: int = 0,  # 奖励值，访问该客户所获得的奖励
        required: bool = True,  # 是否必需，该客户是否必须在可行解中被访问
        group: ClientGroup | None = None,  # 客户组，该客户所属的客户组（可选）
        *,
        name: str = "",  # 客户名称（关键字参数）
    ) -> Client:  # 返回创建的Client对象
        """
        Adds a client with the given attributes to the model. Returns the
        created :class:`~pyvrp._pyvrp.Client` instance.

        Raises
        ------
        ValueError
            When ``group`` is not ``None``, and the given ``group`` is not part
            of this model instance, or when a required client is being added to
            a mutually exclusive client group.
        """
        if group is None:  # 如果未指定客户组
            group_idx = None  # 组索引为None
        elif (idx := _idx_by_id(group, self._groups)) is not None:  # 使用海象运算符查找组索引
            group_idx = idx  # 设置组索引
        else:
            raise ValueError("The given group is not in this model instance.")  # 组不在模型中，抛出异常

        if required and group is not None and group.mutually_exclusive:  # 如果客户是必需的且组是互斥的
            # Required clients cannot be part of a mutually exclusive client
            # group, since then there's nothing to decide about.
            raise ValueError("Required client in mutually exclusive group.")  # 必需客户不能在互斥组中

        client = Client(  # 创建Client对象
            x=x,  # 横坐标
            y=y,  # 纵坐标
            delivery=[delivery] if isinstance(delivery, int) else delivery,  # 将单个值转换为列表
            pickup=[pickup] if isinstance(pickup, int) else pickup,  # 将单个值转换为列表
            service_duration=service_duration,  # 服务持续时间
            tw_early=tw_early,  # 时间窗最早时间
            tw_late=tw_late,  # 时间窗最晚时间
            release_time=release_time,  # 释放时间
            prize=prize,  # 奖励值
            required=required,  # 是否必需
            group=group_idx,  # 组索引
            name=name,  # 名称
        )

        if group_idx is not None:  # 如果客户属于某个组
            client_idx = len(self._depots) + len(self._clients)  # 计算客户在模型中的索引（仓库数+当前客户数）
            self._groups[group_idx].add_client(client_idx)  # 将客户添加到组中

        self._clients.append(client)  # 将客户添加到模型客户列表
        return client  # 返回创建的客户对象

    def add_client_group(  # 添加客户组方法：向模型中添加一个新的客户组
        self, required: bool = True, *, name: str = ""  # required表示组是否必需，name是组名称（关键字参数）
    ) -> ClientGroup:  # 返回创建的ClientGroup对象
        """
        Adds a new, possibly optional, client group to the model. Returns the
        created group.
        """
        group = ClientGroup(required=required, name=name)  # 创建客户组对象
        self._groups.append(group)  # 将组添加到模型组列表
        return group  # 返回创建的组对象

    def add_depot(  # 添加仓库方法：向模型中添加一个仓库
        self,
        x: float,  # 仓库的横坐标
        y: float,  # 仓库的纵坐标
        tw_early: int = 0,  # 仓库开放时间（最早时间）
        tw_late: int = np.iinfo(np.int64).max,  # 仓库关闭时间（最晚时间）
        *,
        name: str = "",  # 仓库名称（关键字参数）
    ) -> Depot:  # 返回创建的Depot对象
        """
        Adds a depot with the given attributes to the model. Returns the
        created :class:`~pyvrp._pyvrp.Depot` instance.
        """
        depot = Depot(x=x, y=y, tw_early=tw_early, tw_late=tw_late, name=name)  # 创建仓库对象

        self._depots.append(depot)  # 将仓库添加到模型仓库列表

        for group in self._groups:  # new depot invalidates client indices
            # 新仓库的添加会使客户索引失效，需要清空所有组并重新添加客户
            group.clear()  # 清空组中的客户列表

        for idx, client in enumerate(self._clients, len(self._depots)):  # 重新计算客户索引并添加到组中
            if client.group is not None:  # 如果客户属于某个组
                self._groups[client.group].add_client(idx)  # 将客户添加到对应的组中

        return depot  # 返回创建的仓库对象

    def add_edge(  # 添加边方法：向模型中添加一条边
        self,
        frm: Client | Depot,  # 起始位置（客户或仓库）
        to: Client | Depot,  # 目标位置（客户或仓库）
        distance: int,  # 边的距离值
        duration: int = 0,  # 边的持续时间值，默认为0
        profile: Profile | None = None,  # 路径规划配置，如果指定则添加到该配置，否则作为基础边
    ) -> Edge:  # 返回创建的Edge对象
        """
        Adds an edge :math:`(i, j)` between ``frm`` (:math:`i`) and ``to``
        (:math:`j`). The edge can be given distance and duration attributes.
        Distance is required, but the default duration is zero. Returns the
        created edge.

        .. note::

           If ``profile`` is not provided, the edge is a base edge that will be
           set for all profiles in the model. Any profile-specific edge takes
           precedence over a base edge with the same ``frm`` and ``to``
           locations.

        .. note::

           If called repeatedly with the same ``frm``, ``to``, and ``profile``
           arguments, only the edge constructed last is used. PyVRP does not
           support multigraphs.
        """
        if profile is not None:  # 如果指定了配置
            return profile.add_edge(frm, to, distance, duration)  # 将边添加到指定配置并返回

        edge = Edge(frm=frm, to=to, distance=distance, duration=duration)  # 创建基础边对象
        self._edges.append(edge)  # 将边添加到基础边列表（用于所有配置）
        return edge  # 返回创建的边对象

    def add_profile(self, *, name: str = "") -> Profile:  # 添加路径规划配置方法：向模型中添加一个新的配置
        """
        Adds a new routing profile to the model.
        """
        profile = Profile(name=name)  # 创建配置对象
        self._profiles.append(profile)  # 将配置添加到模型配置列表
        return profile  # 返回创建的配置对象

    def add_vehicle_type(  # 添加车辆类型方法：向模型中添加一种车辆类型
        self,
        num_available: int = 1,  # 可用车辆数量，默认为1
        capacity: int | list[int] = [],  # 车辆容量（可以是单个值或列表，支持多维度）
        start_depot: Depot | None = None,  # 起始仓库，如果为None则使用第一个仓库
        end_depot: Depot | None = None,  # 结束仓库，如果为None则使用第一个仓库
        fixed_cost: int = 0,  # 固定成本，使用该类型车辆的固定费用
        tw_early: int = 0,  # 班次开始时间
        tw_late: int = np.iinfo(np.int64).max,  # 班次结束时间
        shift_duration: int = np.iinfo(np.int64).max,  # 名义最大班次持续时间
        max_distance: int = np.iinfo(np.int64).max,  # 最大路径距离
        unit_distance_cost: int = 1,  # 每单位距离的成本
        unit_duration_cost: int = 0,  # 每单位持续时间的成本
        profile: Profile | None = None,  # 路径规划配置，如果为None则使用第一个配置
        start_late: int | None = None,  # 班次最晚开始时间
        initial_load: int | list[int] = [],  # 初始负载（可以是单个值或列表）
        reload_depots: list[Depot] = [],  # 重新装载仓库列表，车辆可以在这些仓库重新装载
        max_reloads: int = np.iinfo(np.uint64).max,  # 最大重新装载次数
        max_overtime: int = 0,  # 允许的最大加班时间
        unit_overtime_cost: int = 0,  # 每单位加班时间的成本
        *,
        name: str = "",  # 车辆类型名称（关键字参数）
    ) -> VehicleType:  # 返回创建的VehicleType对象
        """
        Adds a vehicle type with the given attributes to the model. Returns the
        created :class:`~pyvrp._pyvrp.VehicleType` instance.

        .. note::

           The vehicle type is assigned to the first depot if no depot
           information is provided.

        Raises
        ------
        ValueError
            When the given ``depot`` or ``profile`` arguments are not in this
            model instance.
        """
        if start_depot is None:  # 如果未指定起始仓库
            start_idx = 0  # 使用第一个仓库（索引0）
        elif (idx := _idx_by_id(start_depot, self._depots)) is not None:  # 查找起始仓库索引
            start_idx = idx  # 设置起始仓库索引
        else:
            raise ValueError("The given start depot is not in this model.")  # 起始仓库不在模型中

        if end_depot is None:  # 如果未指定结束仓库
            end_idx = 0  # 使用第一个仓库（索引0）
        elif (idx := _idx_by_id(end_depot, self._depots)) is not None:  # 查找结束仓库索引
            end_idx = idx  # 设置结束仓库索引
        else:
            raise ValueError("The given end depot is not in this model.")  # 结束仓库不在模型中

        if profile is None:  # 如果未指定配置
            profile_idx = 0  # 使用第一个配置（索引0）
        elif (idx := _idx_by_id(profile, self._profiles)) is not None:  # 查找配置索引
            profile_idx = idx  # 设置配置索引
        else:
            raise ValueError("The given profile is not in this model.")  # 配置不在模型中

        reloads: list[int] = []  # 初始化重新装载仓库索引列表
        for depot in reload_depots:  # 遍历重新装载仓库列表
            depot_idx = _idx_by_id(depot, self._depots)  # 查找仓库索引
            if depot_idx is not None:  # 如果仓库在模型中
                reloads.append(depot_idx)  # 添加到重新装载列表
            else:
                msg = "The given reload depot is not in this model."  # 仓库不在模型中
                raise ValueError(msg)  # 抛出异常

        init_load = initial_load  # 复制初始负载
        if isinstance(init_load, int):  # 如果是单个值
            init_load = [init_load]  # 转换为列表

        vehicle_type = VehicleType(  # 创建车辆类型对象
            num_available=num_available,  # 可用数量
            capacity=[capacity] if isinstance(capacity, int) else capacity,  # 容量（转换为列表）
            start_depot=start_idx,  # 起始仓库索引
            end_depot=end_idx,  # 结束仓库索引
            fixed_cost=fixed_cost,  # 固定成本
            tw_early=tw_early,  # 班次开始时间
            tw_late=tw_late,  # 班次结束时间
            shift_duration=shift_duration,  # 班次持续时间
            max_distance=max_distance,  # 最大距离
            unit_distance_cost=unit_distance_cost,  # 单位距离成本
            unit_duration_cost=unit_duration_cost,  # 单位持续时间成本
            profile=profile_idx,  # 配置索引
            start_late=start_late,  # 最晚开始时间
            initial_load=init_load,  # 初始负载
            reload_depots=reloads,  # 重新装载仓库索引列表
            max_reloads=max_reloads,  # 最大重新装载次数
            max_overtime=max_overtime,  # 最大加班时间
            unit_overtime_cost=unit_overtime_cost,  # 单位加班成本
            name=name,  # 名称
        )

        self._vehicle_types.append(vehicle_type)  # 将车辆类型添加到模型车辆类型列表
        return vehicle_type  # 返回创建的车辆类型对象

    def data(self, missing_value: int = MAX_VALUE) -> ProblemData:  # 生成问题数据方法：从模型创建ProblemData实例
        """
        Creates and returns a :class:`~pyvrp._pyvrp.ProblemData` instance
        from this model's attributes.

        Parameters
        ----------
        missing_value
            Distance and duration value to use for missing edges. Defaults to
            :const:`~pyvrp.constants.MAX_VALUE`, a large number. Note that this
            value cannot exceed :const:`~pyvrp.constants.MAX_VALUE`.
        """
        locs = self.locations  # 获取所有位置（仓库+客户）
        loc2idx = {id(loc): idx for idx, loc in enumerate(locs)}  # 创建位置对象ID到索引的映射字典

        # First we create the base distance and duration matrices. These are
        # shared by all routing profiles.
        # 首先创建基础距离和持续时间矩阵。这些矩阵被所有路线配置共享。
        fill_value = min(missing_value, MAX_VALUE)  # 计算填充值（不能超过MAX_VALUE）
        base_distance = np.full((len(locs), len(locs)), fill_value, np.int64)  # 创建基础距离矩阵，初始值为fill_value
        base_duration = np.full((len(locs), len(locs)), fill_value, np.int64)  # 创建基础持续时间矩阵，初始值为fill_value
        np.fill_diagonal(base_distance, 0)  # 将对角线（自己到自己的距离）设为0
        np.fill_diagonal(base_duration, 0)  # 将对角线（自己到自己的持续时间）设为0

        for edge in self._edges:  # 遍历所有基础边
            frm = loc2idx[id(edge.frm)]  # 获取起始位置的索引
            to = loc2idx[id(edge.to)]  # 获取目标位置的索引
            base_distance[frm, to] = edge.distance  # 设置基础距离矩阵的值
            base_duration[frm, to] = edge.duration  # 设置基础持续时间矩阵的值

        # Now we create the profile-specific distance and duration matrices.
        # These are based on the base matrices.
        # 现在创建配置特定的距离和持续时间矩阵。这些矩阵基于基础矩阵。
        distances = []  # 初始化距离矩阵列表
        durations = []  # 初始化持续时间矩阵列表
        for profile in self._profiles:  # 遍历每个配置
            prof_distance = base_distance.copy()  # 复制基础距离矩阵
            prof_duration = base_duration.copy()  # 复制基础持续时间矩阵

            for edge in profile.edges:  # 遍历该配置的所有边
                frm = loc2idx[id(edge.frm)]  # 获取起始位置索引
                to = loc2idx[id(edge.to)]  # 获取目标位置索引
                prof_distance[frm, to] = edge.distance  # 覆盖配置特定的距离值
                prof_duration[frm, to] = edge.duration  # 覆盖配置特定的持续时间值

            distances.append(prof_distance)  # 将配置的距离矩阵添加到列表
            durations.append(prof_duration)  # 将配置的持续时间矩阵添加到列表

        # When the user has not provided any profiles, we create an implicit
        # first profile from the base matrices.
        # 当用户没有提供任何配置时，我们从基础矩阵创建一个隐式的第一个配置。
        if not self._profiles:  # 如果没有配置
            distances = [base_distance]  # 使用基础距离矩阵作为第一个配置
            durations = [base_duration]  # 使用基础持续时间矩阵作为第一个配置

        return ProblemData(  # 创建并返回ProblemData对象
            self._clients,  # 客户列表
            self._depots,  # 仓库列表
            self.vehicle_types,  # 车辆类型列表
            distances,  # 距离矩阵列表
            durations,  # 持续时间矩阵列表
            self._groups,  # 客户组列表
        )

    def solve(  # 求解方法：求解当前模型
        self,
        stop: StoppingCriterion,  # 停止准则，定义何时停止求解
        seed: int = 0,  # 随机数种子，用于控制随机性，默认为0
        collect_stats: bool = True,  # 是否收集统计信息，默认为True
        display: bool = True,  # 是否显示求解进度，默认为True（需要collect_stats为True）
        params: SolveParams = SolveParams(),  # 求解参数，如果未提供则使用默认参数
        missing_value: int = MAX_VALUE,  # 缺失边的距离和持续时间值，默认为MAX_VALUE
    ) -> Result:  # 返回Result对象，包含统计信息和最优解
        """
        Solve this model.

        Parameters
        ----------
        stop
            Stopping criterion to use.
        seed
            Seed value to use for the random number stream. Default 0.
        collect_stats
            Whether to collect statistics about the solver's progress. Default
            ``True``.
        display
            Whether to display information about the solver progress. Default
            ``True``. Progress information is only available when
            ``collect_stats`` is also set, which it is by default.
        params
            Solver parameters to use. If not provided, a default will be used.
        missing_value
            Distance and duration value to use for missing edges. Defaults to
            :const:`~pyvrp.constants.MAX_VALUE`, a large number.

        Returns
        -------
        Result
            A Result object, containing statistics (if collected) and the best
            found solution.
        """
        data = self.data(missing_value)  # 从模型生成ProblemData对象
        return solve(data, stop, seed, collect_stats, display, params)  # 调用solve函数进行求解并返回结果


def _idx_by_id(item: object, container: Sequence[object]) -> int | None:  # 辅助函数：通过对象ID（而非值相等）查找索引
    """
    Obtains the index of item in the container by identity rather than equality
    (as would happen with index()). This is important for various objects in
    the Model, because objects that compare equal may not be the same as the
    one intended. See #681 for a bug caused by this.
    """
    for idx, other in enumerate(container):  # 遍历容器中的每个元素
        if item is other:  # 使用is运算符比较对象ID（身份），而不是值相等
            return idx  # 找到匹配的对象，返回索引

    return None  # 未找到匹配的对象，返回None
