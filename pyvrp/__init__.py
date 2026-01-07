import logging  # 日志模块，用于记录程序运行信息
import sys  # 系统模块，用于访问标准输出流

from .IteratedLocalSearch import IteratedLocalSearch as IteratedLocalSearch  # 迭代局部搜索算法类
from .IteratedLocalSearch import (
    IteratedLocalSearchParams as IteratedLocalSearchParams,  # 迭代局部搜索参数类
)
from .Model import Edge as Edge  # 边类，表示两个位置之间的连接
from .Model import Model as Model  # 模型类，用于构建VRP问题实例
from .Model import Profile as Profile  # 路径规划配置类，存储距离和时间矩阵
from .PenaltyManager import PenaltyManager as PenaltyManager  # 惩罚管理器类，动态调整约束违反惩罚
from .PenaltyManager import PenaltyParams as PenaltyParams  # 惩罚参数类
from .Result import Result as Result  # 结果类，封装求解结果和统计信息
from .Statistics import Statistics as Statistics  # 统计信息类，收集求解过程的统计数据
from ._pyvrp import Client as Client  # 客户类（C++绑定），表示需要访问的客户
from ._pyvrp import ClientGroup as ClientGroup  # 客户组类（C++绑定），表示互斥的客户组
from ._pyvrp import CostEvaluator as CostEvaluator  # 成本评估器类（C++绑定），计算解的成本和惩罚项
from ._pyvrp import Depot as Depot  # 仓库类（C++绑定），表示车辆出发和返回的仓库
from ._pyvrp import DynamicBitset as DynamicBitset  # 动态位集合类（C++绑定），高效的布尔数组
from ._pyvrp import ProblemData as ProblemData  # 问题数据类（C++绑定），存储VRP问题的所有数据
from ._pyvrp import RandomNumberGenerator as RandomNumberGenerator  # 随机数生成器类（C++绑定）
from ._pyvrp import Route as Route  # 路线类（C++绑定），表示一条完整的车辆路径
from ._pyvrp import ScheduledVisit as ScheduledVisit  # 计划访问类（C++绑定），存储访问的时间安排
from ._pyvrp import Solution as Solution  # 解类（C++绑定），表示一个完整的VRP解
from ._pyvrp import Trip as Trip  # 行程类（C++绑定），表示从仓库出发到返回的一次行程
from ._pyvrp import VehicleType as VehicleType  # 车辆类型类（C++绑定），定义车辆的特征和约束
from .minimise_fleet import minimise_fleet as minimise_fleet  # 最小化车队函数，减少使用的车辆数量
from .read import read as read  # 读取函数，从VRPLIB格式文件读取问题实例
from .read import read_solution as read_solution  # 读取解函数，从文件读取解
from .show_versions import show_versions as show_versions  # 显示版本信息函数
from .solve import SolveParams as SolveParams  # 求解参数类，配置求解器的各种参数
from .solve import solve as solve  # 求解函数，执行VRP问题的求解

# Sets up basic logging to stdout for PyVRP, of INFO and up. This replaces
# previous print() statements, and allows easier integration into calling
# code's own logging configuration.
# 为PyVRP设置基本日志记录到标准输出，级别为INFO及以上。这替代了之前的print()语句，
# 并允许更容易地集成到调用代码自己的日志配置中。
_logger = logging.getLogger("pyvrp")  # 获取名为"pyvrp"的日志记录器
_logger.addHandler(logging.StreamHandler(stream=sys.stdout))  # 添加标准输出流处理器
_logger.setLevel(logging.INFO)  # 设置日志级别为INFO
