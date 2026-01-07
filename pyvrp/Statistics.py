import csv  # CSV模块，用于读写CSV文件
from dataclasses import dataclass, fields  # 数据类装饰器和字段函数
from pathlib import Path  # 路径处理库
from time import perf_counter  # 高精度计时器
from typing import Iterator, Literal  # 类型提示：迭代器和字面量类型

from pyvrp._pyvrp import CostEvaluator, Solution  # C++绑定的成本评估器和解类

#统计信息类，存储每次迭代的统计信息，用于记录每次迭代的成本、可行性等信息
@dataclass
class _Datum:  # 数据点类：存储单次迭代的数据点
    """
    Single iteration data point.
    """

    current_cost: int  # 当前解的惩罚成本
    current_feas: bool  # 当前解是否可行
    candidate_cost: int  # 候选解的惩罚成本
    candidate_feas: bool  # 候选解是否可行
    best_cost: int  # 最优解的惩罚成本
    best_feas: bool  # 最优解是否可行


class Statistics:  # 统计信息类：收集搜索过程的统计信息
    """
    Statistics about the search progress.

    Parameters
    ----------
    collect_stats
        Whether to collect statistics at all. This can be turned off to avoid
        excessive memory use on long runs.
    """

    runtimes: list[float]  # 运行时间列表，记录每次迭代的运行时间
    num_iterations: int  # 迭代次数计数器
    data: list[_Datum]  # 数据点列表，存储每次迭代的数据点

    def __init__(self, collect_stats: bool = True):  # 构造函数：初始化统计信息对象
        self.runtimes = []  # 初始化运行时间列表
        self.num_iterations = 0  # 初始化迭代次数为0
        self.data = []  # 初始化数据点列表

        self._clock = perf_counter()  # 初始化时钟，记录当前时间
        self._collect_stats = collect_stats  # 存储是否收集统计信息的标志

    def __eq__(self, other: object) -> bool:  # 相等性比较方法：比较两个Statistics对象是否相等
        return (
            isinstance(other, Statistics)  # 检查是否为Statistics类型
            and self._collect_stats == other._collect_stats  # 比较收集标志
            and self.runtimes == other.runtimes  # 比较运行时间列表
            and self.num_iterations == other.num_iterations  # 比较迭代次数
            and self.data == other.data  # 比较数据点列表
        )

    def __iter__(self) -> Iterator[_Datum]:  # 迭代器方法：遍历收集的数据点
        """
        Iterates over the collected data points.
        """
        yield from self.data  # 生成所有数据点

    def is_collecting(self) -> bool:  # 检查是否收集统计信息方法
        return self._collect_stats  # 返回收集标志

    def collect(  # 收集统计信息方法：收集一次迭代的统计信息
        self,
        current: Solution,  # 当前解
        candidate: Solution,  # 候选解
        best: Solution,  # 最优解
        cost_evaluator: CostEvaluator,  # 成本评估器，用于计算解的成本
    ):
        """
        Collect iteration statistics.

        Parameters
        ----------
        current
            The current solution.
        candidate
            The candidate solution.
        best
            The best solution.
        cost_evaluator
            CostEvaluator used to compute costs for solutions.
        """
        if not self._collect_stats:  # 如果不收集统计信息
            return  # 直接返回

        start = self._clock  # 记录开始时间
        self._clock = perf_counter()  # 更新时钟为当前时间

        self.runtimes.append(self._clock - start)  # 计算并添加本次迭代的运行时间
        self.num_iterations += 1  # 增加迭代次数

        datum = _Datum(  # 创建数据点对象
            cost_evaluator.penalised_cost(current),  # 当前解的惩罚成本
            current.is_feasible(),  # 当前解是否可行
            cost_evaluator.penalised_cost(candidate),  # 候选解的惩罚成本
            candidate.is_feasible(),  # 候选解是否可行
            cost_evaluator.penalised_cost(best),  # 最优解的惩罚成本
            best.is_feasible(),  # 最优解是否可行
        )
        self.data.append(datum)  # 将数据点添加到列表

    @classmethod
    def from_csv(cls, where: Path | str, delimiter: str = ",", **kwargs):
        """
        Reads a Statistics object from the CSV file at the given filesystem
        location.

        Parameters
        ----------
        where
            Filesystem location to read from.
        delimiter
            Value separator. Default comma.
        kwargs
            Additional keyword arguments. These are passed to
            :class:`csv.DictReader`.

        Returns
        -------
        Statistics
            Statistics object populated with the data read from the given
            filesystem location.
        """
        field2type = {field.name: field.type for field in fields(_Datum)}

        def make_datum(row) -> _Datum:
            datum = {}

            for name, value in row.items():
                if name in field2type:
                    if field2type[name] is bool:
                        datum[name] = bool(int(value))
                    else:
                        datum[name] = field2type[name](value)  # type: ignore

            return _Datum(**datum)

        with open(where) as fh:
            lines = fh.readlines()

        stats = cls()

        for row in csv.DictReader(lines, delimiter=delimiter, **kwargs):
            stats.runtimes.append(float(row["runtime"]))
            stats.num_iterations += 1
            stats.data.append(make_datum(row))

        return stats

    def to_csv(
        self,
        where: Path | str,
        delimiter: str = ",",
        quoting: Literal[0, 1, 2, 3] = csv.QUOTE_MINIMAL,
        **kwargs,
    ):
        """
        Writes this Statistics object to the given location, as a CSV file.

        Parameters
        ----------
        where
            Filesystem location to write to.
        delimiter
            Value separator. Default comma.
        quoting
            Quoting strategy. Default only quotes values when necessary.
        kwargs
            Additional keyword arguments. These are passed to
            :class:`csv.DictWriter`.
        """
        field_names = [f.name for f in fields(_Datum)]
        data = [
            {
                f: int(v) if isinstance(v, bool) else v  # store bool as 0/1
                for f, v in zip(field_names, vars(datum).values())
            }
            for datum in self.data
        ]

        with open(where, "w") as fh:
            header = ["runtime", *field_names]
            writer = csv.DictWriter(
                fh, header, delimiter=delimiter, quoting=quoting, **kwargs
            )
            writer.writeheader()

            for idx, (runtime, datum) in enumerate(zip(self.runtimes, data)):
                row = dict(runtime=runtime, **datum)
                writer.writerow(row)
