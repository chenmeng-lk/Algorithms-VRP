from pyvrp._pyvrp import (
    CostEvaluator,
    ProblemData,
    RandomNumberGenerator,
    Solution,
)
from pyvrp.search._search import LocalSearch as _LocalSearch
from pyvrp.search._search import (
    LocalSearchStatistics,
    NodeOperator,
    PerturbationManager,
    RouteOperator,
)


class LocalSearch:
    """
    Local search method. This search method explores a granular neighbourhood
    in a very efficient manner using user-provided node and route operators.
    This quickly results in much improved solutions.
    局部搜索方法。该方法使用用户提供的节点算子和路径算子，以非常高效的方式探索粒度邻域，从而快速获得显著改进的解。

    Parameters
    ----------
    data
        Data object describing the problem to be solved.
        描述待解决问题的数据对象。
    rng
        Random number generator.
        随机数生成器。
    neighbours
        List of lists that defines the local search neighbourhood.
        定义局部搜索邻域的列表的列表。
    perturbation_manager
        Perturbation manager that handles perturbation during each invocation.
        处理每次调用过程中扰动的扰动管理器。
    """

    def __init__(
        self,
        data: ProblemData,
        rng: RandomNumberGenerator,
        neighbours: list[list[int]],
        perturbation_manager: PerturbationManager = PerturbationManager(),
    ):
        # 初始化局部搜索对象，内部使用底层的 _LocalSearch 实例，并存储随机数生成器
        self._ls = _LocalSearch(data, neighbours, perturbation_manager)
        self._rng = rng

    def add_node_operator(self, op: NodeOperator):
        """
        Adds a node operator to this local search object. The node operator
        will be used by :meth:`~search` to improve a solution.
        向此局部搜索对象添加一个节点算子。该节点算子将被 :meth:`~search` 方法用于改进解。

        Parameters
        ----------
        op
            The node operator to add to this local search object.
            要添加到此局部搜索对象的节点算子。
        """
        self._ls.add_node_operator(op)

    def add_route_operator(self, op: RouteOperator):
        """
        Adds a route operator to this local search object. The route operator
        will be used by :meth:`~intensify` to improve a solution using more
        expensive route operators.
        向此局部搜索对象添加一个路径算子。该路径算子将被 :meth:`~intensify` 方法使用更昂贵的路径算子来改进解。

        Parameters
        ----------
        op
            The route operator to add to this local search object.
            要添加到此局部搜索对象的路径算子。
        """
        self._ls.add_route_operator(op)

    @property
    def neighbours(self) -> list[list[int]]:
        """
        Returns the granular neighbourhood currently used by the local search.
        返回当前局部搜索使用的粒度邻域。
        """
        return self._ls.neighbours

    @neighbours.setter
    def neighbours(self, neighbours: list[list[int]]):
        """
        Convenience method to replace the current granular neighbourhood used
        by the local search object.
        便捷方法，用于替换局部搜索对象当前使用的粒度邻域。
        """
        self._ls.neighbours = neighbours

    @property
    def node_operators(self) -> list[NodeOperator]:
        """
        Returns the node operators in use.
        返回正在使用的节点算子列表。
        """
        return self._ls.node_operators

    @property
    def route_operators(self) -> list[RouteOperator]:
        """
        Returns the route operators in use.
        返回正在使用的路径算子列表。
        """
        return self._ls.route_operators

    @property
    def statistics(self) -> LocalSearchStatistics:
        """
        Returns search statistics about the most recently improved solution.
        返回关于最近改进解的搜索统计信息。
        """
        return self._ls.statistics

    def __call__(
        self,
        solution: Solution,
        cost_evaluator: CostEvaluator,
    ) -> Solution:
        """
        This method uses the :meth:`~search` and :meth:`~intensify` methods to
        iteratively improve the given solution. First, :meth:`~search` is
        applied. Thereafter, :meth:`~intensify` is applied. This repeats until
        no further improvements are found. Finally, the improved solution is
        returned.
        此方法使用 :meth:`~search` 和 :meth:`~intensify` 方法来迭代改进给定的解。首先应用 :meth:`~search`，
        然后应用 :meth:`~intensify`。重复此过程直到找不到进一步的改进为止。最后返回改进后的解。

        Parameters
        ----------
        solution
            The solution to improve through local search.
            要通过局部搜索改进的解。
        cost_evaluator
            Cost evaluator to use.
            要使用的成本评估器。

        Returns
        -------
        Solution
            The improved solution. This is not the same object as the
            solution that was passed in.
            改进后的解。这不是传入的同一个解对象。
        """
        # 在搜索前打乱算子顺序以增加多样性，然后调用底层局部搜索的 __call__ 方法
        self._ls.shuffle(self._rng)
        return self._ls(solution, cost_evaluator)

    def intensify(
        self,
        solution: Solution,
        cost_evaluator: CostEvaluator,
    ) -> Solution:
        """
        This method uses the intensifying route operators on this local search
        object to improve the given solution.
        此方法使用此局部搜索对象上的强化路径算子来改进给定的解。

        Parameters
        ----------
        solution
            The solution to improve.
            要改进的解。
        cost_evaluator
            Cost evaluator to use.
            要使用的成本评估器。

        Returns
        -------
        Solution
            The improved solution. This is not the same object as the
            solution that was passed in.
            改进后的解。这不是传入的同一个解对象。
        """
        # 在强化搜索前打乱算子顺序，然后调用底层局部搜索的 intensify 方法
        self._ls.shuffle(self._rng)
        return self._ls.intensify(solution, cost_evaluator)

    def search(
        self, solution: Solution, cost_evaluator: CostEvaluator
    ) -> Solution:
        """
        This method uses the node operators on this local search object to
        improve the given solution.
        此方法使用此局部搜索对象上的节点算子来改进给定的解。

        Parameters
        ----------
        solution
            The solution to improve.
            要改进的解。
        cost_evaluator
            Cost evaluator to use.
            要使用的成本评估器。

        Returns
        -------
        Solution
            The improved solution. This is not the same object as the
            solution that was passed in.
            改进后的解。这不是传入的同一个解对象。
        """
        # 在搜索前打乱算子顺序，然后调用底层局部搜索的 search 方法
        self._ls.shuffle(self._rng)
        return self._ls.search(solution, cost_evaluator)
