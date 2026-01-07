#ifndef PYVRP_SEARCH_PERTURBATIONMANAGER_H
#define PYVRP_SEARCH_PERTURBATIONMANAGER_H

#include "CostEvaluator.h"
#include "RandomNumberGenerator.h"
#include "SearchSpace.h"
#include "Solution.h"

#include <iosfwd>

namespace pyvrp::search
{
/**
 * PerturbationParams(min_perturbations: int = 1, max_perturbations: int = 25)
 *
 * Perturbation parameters.
 *
 * Parameters
 * ----------
 * min_perturbations
 *     Minimum number of perturbations to apply. Must not be negative.
 * max_perturbations
 *     Maximum number of perturbations to apply.
 */
struct PerturbationParams  // 扰动参数结构
{
    size_t const minPerturbations;  // 最小扰动次数
    size_t const maxPerturbations;  // 最大扰动次数

    PerturbationParams(size_t minPerturbations = 1,
                       size_t maxPerturbations = 25);

    bool operator==(PerturbationParams const &other) const = default;
};

/**
 * PerturbationManager(params: PerturbationParams)
 *
 * Handles perturbation during the search. In each iteration, it applies
 * :meth:`~num_perturbations` perturbations that strengthen (resp., weaken)
 * randomly selected neighbourhoods by inserting (removing) clients.
 *
 * Parameters
 * ----------
 * params
 *     Perturbation parameters for this manager.
 */
class PerturbationManager  // 扰动管理器类：处理搜索期间的扰动
{
    PerturbationParams const params_;  // owned by us 我们拥有的扰动参数
    size_t numPerturbations_;  // 当前扰动次数

public:
    PerturbationManager(PerturbationParams params = PerturbationParams());

    /**
     * Number of perturbations to apply.
     * 返回扰动次数
     */
    size_t numPerturbations() const;

    /**
     * Draws and sets a new random number of perturbations to apply.
     * 随机抽取并设置新的扰动次数
     */
    void shuffle(RandomNumberGenerator &rng);

    /**
     * Perturbs the given solution using the neighbourhood and ordering of the
     * given search space. Any perturbed clients are marked as promising in the
     * search space.
     *
     * Parameters
     * ----------
     * solution
     *     Solution to perturb. Perturbation happens in place.
     * search_space
     *     The search space to use for perturbation.
     * cost_evaluator
     *     Evaluator to use for insertions.
     */
    //对给定解执行扰动
    void perturb(Solution &solution,
                 SearchSpace &searchSpace,
                 CostEvaluator const &costEvaluator) const;
};
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_PERTURBATIONMANAGER_H
