#include "PerturbationManager.h"
#include "ProblemData.h"
#include "primitives.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <vector>

using pyvrp::search::PerturbationManager;
using pyvrp::search::PerturbationParams;
using pyvrp::search::Route;

namespace
{
using pyvrp::CostEvaluator;
using pyvrp::ProblemData;
using pyvrp::search::SearchSpace;
using pyvrp::search::Solution;

enum class PerturbType
{
    REMOVE,
    INSERT
};

/**
 * Cost-based perturbation: evaluates removal cost for all clients in routes,
 * sorts by descending cost, and removes the top numPerturbations clients.
 * Then reinserts them using the neighbourhood structure.
 */
void costBasedPerturb(Solution &solution,
                      SearchSpace &searchSpace,
                      CostEvaluator const &costEvaluator,
                      ProblemData const &data,
                      size_t numPerturbations)
{
    using pyvrp::Cost;
    using pyvrp::search::removeCost;

    if (numPerturbations == 0)
        return;

    // Collect all clients currently in routes with their removal costs
    std::vector<std::pair<Cost, Route::Node *>> clientCosts;

    for (auto &route : solution.routes)
    {
        if (route.empty())
            continue;

        // Iterate through clients in the route (skip depots)
        for (auto *node : route)
        {
            if (!node->isDepot())
            {
                auto cost = removeCost(node, data, costEvaluator);
                clientCosts.push_back({cost, node});
            }
        }
    }

    if (clientCosts.empty())
        return;

    // Sort by removal cost in descending order (highest cost first)
    // Negative cost means removing the client improves the solution
    // We want to remove clients with the highest (most positive) removal cost
    // as they are the bottlenecks
    std::sort(clientCosts.begin(),
              clientCosts.end(),
              [](auto const &a, auto const &b) { return a.first > b.first; });

    // Remove top numPerturbations clients (or all if fewer available)
    size_t toRemove = std::min(numPerturbations, clientCosts.size());
    std::vector<Route::Node *> removedNodes;
    removedNodes.reserve(toRemove);

    for (size_t i = 0; i < toRemove; ++i)
    {
        auto *node = clientCosts[i].second;
        auto *route = node->route();

        if (route)  // Double-check node is still in a route
        {
            searchSpace.markPromising(node);
            removedNodes.push_back(node);
            route->remove(node->idx());
            route->update();
        }
    }

    // Reinsert removed clients using neighbourhood structure
    for (auto *node : removedNodes)
    {
        solution.insert(node, searchSpace, costEvaluator, true);
        if (node->route())
        {
            node->route()->update();
            searchSpace.markPromising(node);
        }
    }
}
}

PerturbationParams::PerturbationParams(size_t minPerturbations,
                                       size_t maxPerturbations,
                                       double costBasedRatio)
    : minPerturbations(minPerturbations),
      maxPerturbations(maxPerturbations),
      costBasedRatio(costBasedRatio)
{
    if (minPerturbations > maxPerturbations)
        throw std::invalid_argument(
            "min_perturbations must be <= max_perturbations.");

    if (costBasedRatio < 0.0 || costBasedRatio > 1.0)
        throw std::invalid_argument(
            "cost_based_ratio must be between 0.0 and 1.0.");
}

PerturbationManager::PerturbationManager(PerturbationParams params)
    : params_(params), numPerturbations_(params_.minPerturbations)
{
}

double PerturbationManager::costBasedRatio() const
{
    return params_.costBasedRatio;
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

void PerturbationManager::perturb(Solution &solution,
                                  SearchSpace &searchSpace,
                                  CostEvaluator const &costEvaluator,
                                  ProblemData const &data,
                                  RandomNumberGenerator &rng) const
{
    size_t movesLeft = numPerturbations_;

    if (!movesLeft)  // nothing to do
        return;

    // Clear the set of promising nodes. Perturbation determines the initial
    // set of promising nodes for further (local search) improvement.
    searchSpace.unmarkAllPromising();

    // Randomly choose between cost-based and random perturbation based on costBasedRatio
    if (rng.rand() < params_.costBasedRatio)
    {
        // Cost-based perturbation: remove high-cost clients (bottlenecks)
        costBasedPerturb(solution, searchSpace, costEvaluator, data, numPerturbations_);
        return;
    }

    // Original random perturbation method
    DynamicBitset perturbed = {solution.nodes.size()};
    auto const perturb = [&](auto *node, PerturbType action)
    {
        // This node has already been touched by a previous perturbation, so
        // we skip it here.
        if (perturbed[node->client()])
            return;

        // Remove if node is in a route and we are currently removing.
        auto *route = node->route();
        if (route && action == PerturbType::REMOVE)
        {
            searchSpace.markPromising(node);
            route->remove(node->idx());
            route->update();
        }
        // Insert if node is not in a route and we are currently inserting.
        else if (!route && action == PerturbType::INSERT)
        {
            solution.insert(node, searchSpace, costEvaluator, true);
            node->route()->update();
            searchSpace.markPromising(node);
        }
        else  // no-op
            return;

        perturbed[node->client()] = true;
        movesLeft--;
    };

    // We do numPerturbations if we can. We perturb the local neighbourhood of
    // randomly selected clients U: if U is in the solution, we remove it and
    // its neighbours, while if it is not, we try to insert instead. Each
    // removal or insertion counts as one perturbation.
    for (auto const uClient : searchSpace.clientOrder())
    {
        auto *U = &solution.nodes[uClient];
        auto action = U->route() ? PerturbType::REMOVE : PerturbType::INSERT;
        perturb(U, action);

        if (!movesLeft)
            return;

        for (auto const vClient : searchSpace.neighboursOf(U->client()))
        {
            auto *V = &solution.nodes[vClient];
            perturb(V, action);

            if (!movesLeft)
                return;
        }
    }
}
