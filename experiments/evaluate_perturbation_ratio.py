"""
评估成本扰动占比对求解质量的影响

这个脚本会测试不同的成本扰动占比（0%, 10%, 20%, ..., 100%），
并记录每个占比下的：
- 最优解成本
- 迭代次数
- 运行时间
- 收敛速度

使用方法:
    python experiments/evaluate_perturbation_ratio.py [instance_path]
"""

import argparse
import json
import time
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from pyvrp import Model, ProblemData, RandomNumberGenerator, Solution
from pyvrp import CostEvaluator, PenaltyManager, IteratedLocalSearch
from pyvrp.read import read
from pyvrp.search import (
    LocalSearch,
    PerturbationManager,
    PerturbationParams,
    compute_neighbours,
    Exchange10,
)
from pyvrp.stop import MaxIterations, MaxRuntime


def run_experiment(
    data: ProblemData,
    cost_ratio: float,
    seed: int = 42,
    max_iterations: int = 1000,
    max_runtime: float = 60.0,
) -> Dict:
    """
    运行单次实验，使用指定的成本扰动占比

    Parameters
    ----------
    data
        问题数据
    cost_ratio
        成本扰动占比 (0.0 到 1.0)
    seed
        随机种子
    max_iterations
        最大迭代次数
    max_runtime
        最大运行时间（秒）

    Returns
    -------
    Dict
        包含实验结果的字典
    """
    rng = RandomNumberGenerator(seed=seed)
    pm = PenaltyManager(initial_penalties=([20], 6, 6))
    neighbours = compute_neighbours(data)

    # 创建带有指定成本扰动占比的 PerturbationManager
    perturb_params = PerturbationParams(
        min_perturbations=1,
        max_perturbations=25,
        cost_based_ratio=cost_ratio
    )
    perturbation_manager = PerturbationManager(perturb_params)

    # 创建 LocalSearch
    ls = LocalSearch(data, rng, neighbours, perturbation_manager)
    ls.add_node_operator(Exchange10(data))

    # 创建初始解
    init = Solution.make_random(data, rng)

    # 运行 ILS
    ils = IteratedLocalSearch(data, pm, rng, ls, init)

    start_time = time.perf_counter()
    result = ils.run(
        MaxIterations(max_iterations) & MaxRuntime(max_runtime),
        collect_stats=True
    )
    runtime = time.perf_counter() - start_time

    cost_eval = pm.cost_evaluator()
    best_cost = cost_eval.cost(result.best)

    # 收集统计信息
    stats_data = result.stats.data if result.stats else []

    return {
        "cost_ratio": cost_ratio,
        "seed": seed,
        "best_cost": best_cost,
        "num_iterations": result.num_iterations,
        "runtime": runtime,
        "is_feasible": result.best.is_feasible(),
        "num_routes": result.best.num_routes(),
        "num_clients": data.num_clients,
        "convergence_data": [
            {
                "iteration": i,
                "best_cost": stats_data[i].best_cost if i < len(stats_data) else best_cost,
            }
            for i in range(0, min(len(stats_data), max_iterations), max(1, len(stats_data) // 100))
        ] if stats_data else []
    }


def main():
    parser = argparse.ArgumentParser(description="评估成本扰动占比的影响")
    parser.add_argument(
        "instance",
        type=str,
        nargs="?",
        default="data/OkSmall.txt",
        help="VRPTW 实例文件路径"
    )
    parser.add_argument(
        "--ratios",
        type=str,
        default="0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0",
        help="要测试的成本扰动占比，逗号分隔"
    )
    parser.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=[42, 123, 456, 789, 1024],
        help="随机种子列表"
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=1000,
        help="每次运行的最大迭代次数"
    )
    parser.add_argument(
        "--max-runtime",
        type=float,
        default=60.0,
        help="每次运行的最大时间（秒）"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="experiments/perturbation_ratio_results.json",
        help="输出结果文件路径"
    )
    
    args = parser.parse_args()
    
    # 读取实例
    print(f"Loading instance: {args.instance}")
    data = read(args.instance)
    print(f"  Clients: {data.num_clients}, Vehicles: {data.num_vehicles}")
    
    # 解析占比
    ratios = [float(r) for r in args.ratios.split(",")]
    
    # 运行实验
    results = []
    total_runs = len(ratios) * len(args.seeds)
    current_run = 0
    
    print(f"\nRunning {total_runs} experiments...")
    print("=" * 60)

    for ratio in ratios:
        print(f"\nTesting cost_ratio = {ratio:.2f}")
        for seed in args.seeds:
            current_run += 1
            print(f"  [{current_run}/{total_runs}] ", end="")

            result = run_experiment(
                data,
                ratio,
                seed=seed,
                max_iterations=args.max_iterations,
                max_runtime=args.max_runtime,
            )
            results.append(result)

            print(f"    -> Cost: {result['best_cost']}, "
                  f"Iters: {result['num_iterations']}, "
                  f"Time: {result['runtime']:.2f}s")

    # 保存结果
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\n{'=' * 60}")
    print(f"Results saved to: {output_path}")

    # 生成分析报告
    analyze_results(results, output_path.parent)


def analyze_results(results: List[Dict], output_dir: Path):
    """分析实验结果并生成可视化"""
    df = pd.DataFrame(results)

    # 按占比分组统计
    summary = df.groupby("cost_ratio").agg({
        "best_cost": ["mean", "std", "min", "max"],
        "num_iterations": ["mean", "std"],
        "runtime": ["mean", "std"],
    }).round(2)

    print("\n" + "=" * 60)
    print("Summary Statistics:")
    print("=" * 60)
    print(summary)

    # 保存统计摘要
    summary.to_csv(output_dir / "perturbation_ratio_summary.csv")

    # 生成可视化
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Cost-Based Perturbation Ratio Analysis", fontsize=16, y=0.995)

    # 1. 最优解成本 vs 占比（带误差条）
    ax = axes[0, 0]
    grouped = df.groupby("cost_ratio")["best_cost"]
    means = grouped.mean()
    stds = grouped.std()
    ax.errorbar(means.index, means.values, yerr=stds.values,
                marker='o', capsize=5, capthick=2, linewidth=2, markersize=8)
    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Best Solution Cost", fontsize=11)
    ax.set_title("Solution Quality vs Perturbation Ratio", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_xticks(means.index)

    # 2. 迭代次数 vs 占比
    ax = axes[0, 1]
    grouped = df.groupby("cost_ratio")["num_iterations"]
    means = grouped.mean()
    stds = grouped.std()
    ax.errorbar(means.index, means.values, yerr=stds.values,
                marker='s', capsize=5, capthick=2, color='orange', linewidth=2, markersize=8)
    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Number of Iterations", fontsize=11)
    ax.set_title("Iterations vs Perturbation Ratio", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_xticks(means.index)

    # 3. 运行时间 vs 占比
    ax = axes[0, 2]
    grouped = df.groupby("cost_ratio")["runtime"]
    means = grouped.mean()
    stds = grouped.std()
    ax.errorbar(means.index, means.values, yerr=stds.values,
                marker='^', capsize=5, capthick=2, color='green', linewidth=2, markersize=8)
    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Runtime (seconds)", fontsize=11)
    ax.set_title("Runtime vs Perturbation Ratio", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_xticks(means.index)

    # 4. 箱线图：成本分布
    ax = axes[1, 0]
    ratios = sorted(df["cost_ratio"].unique())
    box_data = [df[df["cost_ratio"] == r]["best_cost"].values for r in ratios]
    bp = ax.boxplot(box_data, labels=[f"{r:.1f}" for r in ratios], patch_artist=True)
    for patch in bp['boxes']:
        patch.set_facecolor('lightblue')
    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Best Solution Cost", fontsize=11)
    ax.set_title("Cost Distribution by Ratio", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')

    # 5. 相对改进百分比（相对于纯随机扰动）
    ax = axes[1, 1]
    baseline_cost = df[df["cost_ratio"] == 0.0]["best_cost"].mean()
    grouped = df.groupby("cost_ratio")["best_cost"]
    means = grouped.mean()
    improvement = ((baseline_cost - means) / baseline_cost * 100)
    ax.bar(improvement.index, improvement.values, color='steelblue', alpha=0.7, edgecolor='black')
    ax.axhline(y=0, color='red', linestyle='--', linewidth=1)
    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Improvement over Baseline (%)", fontsize=11)
    ax.set_title("Relative Improvement (vs ratio=0.0)", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_xticks(improvement.index)

    # 6. 效率指标：成本改进 / 运行时间
    ax = axes[1, 2]
    cost_means = df.groupby("cost_ratio")["best_cost"].mean()
    time_means = df.groupby("cost_ratio")["runtime"].mean()
    # 归一化效率指标（越小越好的成本 / 时间）
    efficiency = (baseline_cost - cost_means) / time_means
    ax.plot(efficiency.index, efficiency.values, marker='D',
            linewidth=2, markersize=8, color='purple')
    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Efficiency (Cost Improvement / Time)", fontsize=11)
    ax.set_title("Search Efficiency", fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_xticks(efficiency.index)

    plt.tight_layout()
    plot_path = output_dir / "perturbation_ratio_analysis.png"
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved to: {plot_path}")

    # 找出最佳占比
    best_ratio_cost = summary["best_cost"]["mean"].idxmin()
    best_cost = summary["best_cost"]["mean"].min()
    best_ratio_efficiency = efficiency.idxmax()

    print(f"\n{'=' * 60}")
    print(f"Best Ratio (by cost): {best_ratio_cost:.2f}")
    print(f"  Average Cost: {best_cost:.2f}")
    print(f"Best Ratio (by efficiency): {best_ratio_efficiency:.2f}")
    print(f"  Efficiency: {efficiency[best_ratio_efficiency]:.4f}")

    # 统计显著性分析
    print(f"\n{'=' * 60}")
    print("Statistical Analysis:")
    print(f"{'=' * 60}")
    baseline_costs = df[df["cost_ratio"] == 0.0]["best_cost"].values
    for ratio in sorted(df["cost_ratio"].unique()):
        if ratio == 0.0:
            continue
        ratio_costs = df[df["cost_ratio"] == ratio]["best_cost"].values
        improvement_pct = ((baseline_costs.mean() - ratio_costs.mean()) / baseline_costs.mean() * 100)
        print(f"Ratio {ratio:.2f}: {improvement_pct:+.2f}% vs baseline (mean: {ratio_costs.mean():.2f})")

    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()

