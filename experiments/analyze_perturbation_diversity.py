"""
分析扰动多样性

这个脚本深入分析不同扰动占比下的搜索行为，特别关注：
1. 扰动后解的多样性
2. 接受率（acceptance rate）
3. 收敛曲线
4. 解空间探索范围

帮助理解为什么高成本扰动占比会导致迭代次数增加。
"""

import sys
from pathlib import Path
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

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
from pyvrp.stop import MaxIterations


def analyze_search_behavior(data, ratio, seed=42, max_iters=1000):
    """
    分析搜索行为，收集详细统计信息
    """
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
    
    # 运行 ILS 并收集统计信息
    ils = IteratedLocalSearch(data, pm, rng, ls, init)
    result = ils.run(MaxIterations(max_iters), collect_stats=True)
    
    cost_eval = pm.cost_evaluator()
    
    # 分析统计数据
    stats_data = result.stats.data if result.stats else []
    
    # 收集成本轨迹
    best_costs = [d.best_cost for d in stats_data]
    current_costs = [d.current_cost for d in stats_data]
    candidate_costs = [d.candidate_cost for d in stats_data]
    
    # 计算接受率
    acceptances = []
    for i in range(1, len(stats_data)):
        if current_costs[i] != current_costs[i-1]:
            acceptances.append(1)
        else:
            acceptances.append(0)
    
    acceptance_rate = np.mean(acceptances) if acceptances else 0.0
    
    # 计算解的多样性（通过成本标准差）
    cost_diversity = np.std(current_costs) if current_costs else 0.0
    
    # 计算收敛速度（达到最优解的迭代次数）
    final_best = best_costs[-1] if best_costs else float('inf')
    convergence_iter = next((i for i, c in enumerate(best_costs) if c == final_best), len(best_costs))
    
    # 计算改进次数
    improvements = sum(1 for i in range(1, len(best_costs)) if best_costs[i] < best_costs[i-1])
    
    return {
        "ratio": ratio,
        "seed": seed,
        "final_cost": cost_eval.cost(result.best),
        "acceptance_rate": acceptance_rate,
        "cost_diversity": cost_diversity,
        "convergence_iter": convergence_iter,
        "num_improvements": improvements,
        "best_costs": best_costs,
        "current_costs": current_costs,
        "candidate_costs": candidate_costs,
    }


def main():
    # 读取实例
    instance_path = "tests/data/E-n22-k4.txt"
    print(f"Loading instance: {instance_path}")
    data = read(instance_path)
    print(f"  Clients: {data.num_clients}, Vehicles: {data.num_vehicles}\n")
    
    # 测试不同占比
    ratios = [0.0, 0.2, 0.5, 0.8, 1.0]
    seed = 42
    max_iters = 1000
    
    print("Analyzing search behavior...")
    print("=" * 80)
    
    results = {}
    for ratio in ratios:
        print(f"Analyzing ratio {ratio:.2f}...", end=" ")
        result = analyze_search_behavior(data, ratio, seed=seed, max_iters=max_iters)
        results[ratio] = result
        print(f"Done. Cost: {result['final_cost']:.2f}, "
              f"Acceptance: {result['acceptance_rate']:.2%}, "
              f"Diversity: {result['cost_diversity']:.2f}")
    
    print("=" * 80)
    
    # 打印摘要
    print("\nSummary:")
    print("=" * 80)
    print(f"{'Ratio':<10} {'Cost':<12} {'Accept%':<12} {'Diversity':<12} "
          f"{'Conv.Iter':<12} {'#Improv':<10}")
    print("=" * 80)
    
    for ratio in ratios:
        r = results[ratio]
        print(f"{ratio:<10.2f} {r['final_cost']:<12.2f} {r['acceptance_rate']:<12.2%} "
              f"{r['cost_diversity']:<12.2f} {r['convergence_iter']:<12} {r['num_improvements']:<10}")
    
    print("=" * 80)

    # 生成可视化
    print("\nGenerating visualizations...")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Perturbation Diversity Analysis", fontsize=16)

    # 1. 收敛曲线
    ax = axes[0, 0]
    for ratio in ratios:
        r = results[ratio]
        ax.plot(r['best_costs'], label=f"Ratio {ratio:.1f}", linewidth=2, alpha=0.8)
    ax.set_xlabel("Iteration", fontsize=11)
    ax.set_ylabel("Best Cost", fontsize=11)
    ax.set_title("Convergence Curves", fontsize=12, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 当前解成本轨迹（显示探索范围）
    ax = axes[0, 1]
    for ratio in ratios:
        r = results[ratio]
        # 只显示前500次迭代以便看清
        ax.plot(r['current_costs'][:500], label=f"Ratio {ratio:.1f}",
                linewidth=1.5, alpha=0.6)
    ax.set_xlabel("Iteration", fontsize=11)
    ax.set_ylabel("Current Solution Cost", fontsize=11)
    ax.set_title("Solution Exploration (first 500 iters)", fontsize=12, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. 接受率和多样性
    ax = axes[1, 0]
    x = np.arange(len(ratios))
    width = 0.35

    accept_rates = [results[r]['acceptance_rate'] * 100 for r in ratios]
    diversities = [results[r]['cost_diversity'] for r in ratios]

    ax2 = ax.twinx()
    bars1 = ax.bar(x - width/2, accept_rates, width, label='Acceptance Rate (%)',
                   color='steelblue', alpha=0.7)
    bars2 = ax2.bar(x + width/2, diversities, width, label='Cost Diversity',
                    color='coral', alpha=0.7)

    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Acceptance Rate (%)", fontsize=11, color='steelblue')
    ax2.set_ylabel("Cost Diversity (Std)", fontsize=11, color='coral')
    ax.set_title("Acceptance Rate & Diversity", fontsize=12, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([f"{r:.1f}" for r in ratios])
    ax.tick_params(axis='y', labelcolor='steelblue')
    ax2.tick_params(axis='y', labelcolor='coral')
    ax.grid(True, alpha=0.3, axis='y')

    # 添加图例
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    # 4. 改进次数和收敛速度
    ax = axes[1, 1]
    x = np.arange(len(ratios))

    improvements = [results[r]['num_improvements'] for r in ratios]
    conv_iters = [results[r]['convergence_iter'] for r in ratios]

    ax2 = ax.twinx()
    bars1 = ax.bar(x - width/2, improvements, width, label='# Improvements',
                   color='green', alpha=0.7)
    bars2 = ax2.bar(x + width/2, conv_iters, width, label='Convergence Iter',
                    color='orange', alpha=0.7)

    ax.set_xlabel("Cost-Based Perturbation Ratio", fontsize=11)
    ax.set_ylabel("Number of Improvements", fontsize=11, color='green')
    ax2.set_ylabel("Convergence Iteration", fontsize=11, color='orange')
    ax.set_title("Improvements & Convergence Speed", fontsize=12, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([f"{r:.1f}" for r in ratios])
    ax.tick_params(axis='y', labelcolor='green')
    ax2.tick_params(axis='y', labelcolor='orange')
    ax.grid(True, alpha=0.3, axis='y')

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    plt.tight_layout()

    output_path = Path("experiments/perturbation_diversity_analysis.png")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")

    # 分析结论
    print("\n" + "=" * 80)
    print("ANALYSIS & INSIGHTS:")
    print("=" * 80)

    # 找出接受率最高和最低的
    max_accept_ratio = max(ratios, key=lambda r: results[r]['acceptance_rate'])
    min_accept_ratio = min(ratios, key=lambda r: results[r]['acceptance_rate'])

    print(f"\n1. Acceptance Rate:")
    print(f"   - Highest: ratio {max_accept_ratio:.2f} ({results[max_accept_ratio]['acceptance_rate']:.2%})")
    print(f"   - Lowest:  ratio {min_accept_ratio:.2f} ({results[min_accept_ratio]['acceptance_rate']:.2%})")

    if results[1.0]['acceptance_rate'] < results[0.0]['acceptance_rate']:
        print(f"   → Pure cost-based perturbation has LOWER acceptance rate")
        print(f"     This suggests it produces less diverse solutions!")

    # 找出多样性最高和最低的
    max_div_ratio = max(ratios, key=lambda r: results[r]['cost_diversity'])
    min_div_ratio = min(ratios, key=lambda r: results[r]['cost_diversity'])

    print(f"\n2. Solution Diversity:")
    print(f"   - Highest: ratio {max_div_ratio:.2f} (std: {results[max_div_ratio]['cost_diversity']:.2f})")
    print(f"   - Lowest:  ratio {min_div_ratio:.2f} (std: {results[min_div_ratio]['cost_diversity']:.2f})")

    if results[1.0]['cost_diversity'] < results[0.0]['cost_diversity']:
        print(f"   → Pure cost-based perturbation explores NARROWER solution space")
        print(f"     This explains why it may need more iterations!")

    # 最佳解质量
    best_ratio = min(ratios, key=lambda r: results[r]['final_cost'])
    print(f"\n3. Best Solution Quality:")
    print(f"   - Best ratio: {best_ratio:.2f} (cost: {results[best_ratio]['final_cost']:.2f})")

    # 收敛速度
    fastest_ratio = min(ratios, key=lambda r: results[r]['convergence_iter'])
    print(f"\n4. Convergence Speed:")
    print(f"   - Fastest: ratio {fastest_ratio:.2f} (iter: {results[fastest_ratio]['convergence_iter']})")

    print("\n" + "=" * 80)
    print("RECOMMENDATION:")
    print("=" * 80)

    # 综合评估
    if best_ratio == 0.0:
        print("❌ Cost-based perturbation does NOT improve solution quality.")
        print("   Possible reasons:")
        print("   - Removes the same high-cost clients repeatedly")
        print("   - Lacks diversity to explore different solution regions")
        print("   - May be stuck in similar local optima")
    elif 0.0 < best_ratio < 1.0:
        print(f"✓ Mixed strategy (ratio={best_ratio:.2f}) works BEST!")
        print("   - Balances exploitation (cost-based) and exploration (random)")
        print("   - Maintains solution diversity while targeting bottlenecks")
        print(f"   - Recommended ratio: {best_ratio:.2f}")
    else:
        print("✓ Pure cost-based perturbation works well!")
        print("   - Effectively targets bottlenecks")
        print("   - Instance may have clear cost structure")

    print("=" * 80)

