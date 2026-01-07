#include "PerturbationManager.h"
#include "primitives.h"

#include <cassert>
#include <stdexcept>

using pyvrp::search::PerturbationManager;
using pyvrp::search::PerturbationParams;
using pyvrp::search::Route;

namespace
{
// 扰动操作类型：移除或插入节点
enum class PerturbType
{
    REMOVE,
    INSERT
};
}

PerturbationParams::PerturbationParams(size_t minPerturbations,
                                       size_t maxPerturbations)
    : minPerturbations(minPerturbations), maxPerturbations(maxPerturbations)
{
    if (minPerturbations > maxPerturbations)
        throw std::invalid_argument(
            "min_perturbations must be <= max_perturbations.");
}

PerturbationManager::PerturbationManager(PerturbationParams params)
    : params_(params), numPerturbations_(params_.minPerturbations)
{
}

size_t PerturbationManager::numPerturbations() const
{
    return numPerturbations_;
}

void PerturbationManager::shuffle(RandomNumberGenerator &rng)
{
    auto const range = params_.maxPerturbations - params_.minPerturbations;
    numPerturbations_ = params_.minPerturbations + rng.randint(range + 1);
}
// 对给定解执行扰动操作
void PerturbationManager::perturb(Solution &solution,
                                  SearchSpace &searchSpace,
                                  CostEvaluator const &costEvaluator) const
{
    size_t movesLeft = numPerturbations_;

    if (!movesLeft)  // nothing to do
        return;

    // Clear the set of promising nodes. Perturbation determines the initial
    // set of promising nodes for further (local search) improvement.
    searchSpace.unmarkAllPromising();

    // 用于跟踪哪些节点已经被扰动操作处理过，避免重复处理
    DynamicBitset perturbed = {solution.nodes.size()};
    // 定义扰动操作的lambda函数，根据action类型执行移除或插入操作
    auto const perturb = [&](auto *node, PerturbType action)
    {
        // This node has already been touched by a previous perturbation, so
        // we skip it here.
        // 如果该节点已经被之前的扰动操作处理过，则跳过
        if (perturbed[node->client()])
            return;

        // Remove if node is in a route and we are currently removing.
        // 如果节点在路线中且当前操作为移除，则从路线中移除该节点
        auto *route = node->route();
        if (route && action == PerturbType::REMOVE)
        {
            searchSpace.markPromising(node);
            route->remove(node->idx());
            route->update();
        }
        // Insert if node is not in a route and we are currently inserting.
        // 如果节点不在路线中且当前操作为插入，则尝试插入节点
        else if (!route && action == PerturbType::INSERT)
        {
            solution.insert(node, searchSpace, costEvaluator, true);
            node->route()->update();
            searchSpace.markPromising(node);
        }
        else  // no-op
            return;//其他情况不执行操作（如节点已在路线中但操作为插入，或节点不在路线中但操作为移除）

        perturbed[node->client()] = true;// 标记该节点已被处理
        movesLeft--;//减少剩余扰动操作次数
    };

    // We do numPerturbations if we can. We perturb the local neighbourhood of
    // randomly selected clients U: if U is in the solution, we remove it and
    // its neighbours, while if it is not, we try to insert instead. Each
    // removal or insertion counts as one perturbation.
    // 按照搜索空间的客户顺序遍历每个客户节点U,尝试执行扰动操作直到达到指定次数
    // TODO:扰动执行顺序取决于搜索空间中的客户顺序；移除插入都计入扰动次数中；
    // 客户被移除后这里没有插入；插入策略是插入到邻居之后或新路线中
    for (auto const uClient : searchSpace.clientOrder())
    {
        auto *U = &solution.nodes[uClient];
        // 根据U是否在路线中决定扰动类型
        auto action = U->route() ? PerturbType::REMOVE : PerturbType::INSERT;
        perturb(U, action);

        if (!movesLeft)// 如果扰动次数已用完，返回
            return;

        // 对U的邻居节点执行相同类型的扰动操作
        for (auto const vClient : searchSpace.neighboursOf(U->client()))
        {
            auto *V = &solution.nodes[vClient];
            perturb(V, action);

            if (!movesLeft)
                return;
        }
    }
}
