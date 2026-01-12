"""
快速测试成本扰动的效果

这个脚本快速测试几个关键的成本扰动占比，用于初步验证新方法是否有效。
"""

import sys
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from pyvrp import RandomNumberGenerator, PenaltyManager, IteratedLocalSearch
from pyvrp.read import read
from pyvrp.search import (
    LocalSearch,
    PerturbationManager,
    PerturbationParams,
    compute_neighbours,
    Exchange10,
)
from pyvrp.stop import MaxRuntime


def test_ratio(data, ratio, seed=42, max_runtime=10.0):
    """测试单个占比，使用固定运行时间"""
    rng = RandomNumberGenerator(seed=seed)
    pm = PenaltyManager(initial_penalties=([20], 6, 6))
    neighbours = compute_neighbours(data)

    # 创建扰动管理器
    perturb_params = PerturbationParams(
        min_perturbations=1,
        max_perturbations=25,
        cost_based_ratio=ratio
    )
    perturbation_manager = PerturbationManager(perturb_params)

    # 创建 LocalSearch
    ls = LocalSearch(data, rng, neighbours, perturbation_manager)
    ls.add_node_operator(Exchange10(data))

    # 创建初始解
    from pyvrp import Solution
    init = Solution.make_random(data, rng)

    # 运行 ILS，使用固定时间
    ils = IteratedLocalSearch(data, pm, rng, ls, init)
    result = ils.run(MaxRuntime(max_runtime), collect_stats=False)

    cost_eval = pm.cost_evaluator()
    return {
        "ratio": ratio,
        "cost": cost_eval.cost(result.best),
        "iterations": result.num_iterations,
        "runtime": result.runtime,
        "feasible": result.best.is_feasible(),
    }


def main():
    # 读取测试实例
    instance_path = "instances/CVRP/XXL/X-n1001-k43.vrp"
    print(f"Loading instance: {instance_path}")
    data = read(instance_path)
    print(f"  Clients: {data.num_clients}, Vehicles: {data.num_vehicles}\n")
    
    # 测试不同的占比
    ratios = [0.0, 0.1, 0.15, 0.2, 0.3 , 0.5, 0.8, 0.9, 1.0]
    seeds = [42, 123, 456]
    max_runtime = 1.0  # 每次运行10秒

    print(f"Testing with {max_runtime}s runtime per run\n")
    print("=" * 85)
    print(f"{'Ratio':<10} {'Seed':<10} {'Cost':<15} {'Iters':<10} {'Runtime(s)':<12} {'Feasible':<10}")
    print("=" * 85)

    results_by_ratio = {r: [] for r in ratios}

    for ratio in ratios:
        for seed in seeds:
            result = test_ratio(data, ratio, seed=seed, max_runtime=max_runtime)
            results_by_ratio[ratio].append(result)

            print(f"{ratio:<10.2f} {seed:<10} {result['cost']:<15.2f} "
                  f"{result['iterations']:<10} {result['runtime']:<12.2f} {result['feasible']!s:<10}")
    
    print("=" * 85)
    print("\nSummary (Average over seeds):")
    print("=" * 85)
    print(f"{'Ratio':<10} {'Avg Cost':<15} {'Std Cost':<15} {'Avg Iters':<12} {'Iters/s':<10}")
    print("=" * 85)

    import numpy as np

    baseline_cost = None
    baseline_iters = None
    for ratio in ratios:
        costs = [r["cost"] for r in results_by_ratio[ratio]]
        iters = [r["iterations"] for r in results_by_ratio[ratio]]
        runtimes = [r["runtime"] for r in results_by_ratio[ratio]]

        avg_cost = np.mean(costs)
        std_cost = np.std(costs)
        avg_iters = np.mean(iters)
        avg_runtime = np.mean(runtimes)
        iters_per_sec = avg_iters / avg_runtime if avg_runtime > 0 else 0

        if ratio == 0.0:
            baseline_cost = avg_cost
            baseline_iters = avg_iters

        improvement = ""
        if baseline_cost and ratio > 0.0:
            pct = ((baseline_cost - avg_cost) / baseline_cost * 100)
            improvement = f" ({pct:+.2f}%)"

        # 添加迭代次数变化指示
        iter_change = ""
        if baseline_iters and ratio > 0.0:
            iter_pct = ((avg_iters - baseline_iters) / baseline_iters * 100)
            if abs(iter_pct) > 5:  # 只显示变化超过5%的
                iter_change = f" [{iter_pct:+.1f}%]"

        print(f"{ratio:<10.2f} {avg_cost:<15.2f} {std_cost:<15.2f} "
              f"{avg_iters:<12.0f}{iter_change:<8} {iters_per_sec:<10.1f}{improvement}")

    print("=" * 85)
    
    # 找出最佳占比
    avg_costs = {r: np.mean([res["cost"] for res in results_by_ratio[r]]) 
                 for r in ratios}
    best_ratio = min(avg_costs, key=avg_costs.get)
    
    print(f"\n✓ Best ratio: {best_ratio:.2f} with average cost {avg_costs[best_ratio]:.2f}")
    
    if best_ratio > 0.0:
        improvement = ((avg_costs[0.0] - avg_costs[best_ratio]) / avg_costs[0.0] * 100)
        print(f"✓ Improvement over baseline: {improvement:.2f}%")
        print("\n→ Cost-based perturbation shows promise! Run full evaluation for detailed analysis.")
    else:
        print("\n→ Cost-based perturbation may not be beneficial for this instance.")
        print("  Consider testing on larger/harder instances or adjusting parameters.")


if __name__ == "__main__":
    main()

