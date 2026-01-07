#include "SearchSpace.h"

#include <cassert>
#include <numeric>
#include <stdexcept>

using pyvrp::search::Route;
using pyvrp::search::SearchSpace;

SearchSpace::SearchSpace(ProblemData const &data, Neighbours neighbours)// 初始化搜索空间，设置邻居结构并初始化订单
    : neighbours_(data.numLocations()),
      promising_(data.numLocations()),
      clientOrder_(data.numClients()),
      routeOrder_(data.numVehicles())
{
    setNeighbours(neighbours);

    std::iota(clientOrder_.begin(), clientOrder_.end(), data.numDepots());// 生成客户顺序
    std::iota(routeOrder_.begin(), routeOrder_.end(), 0);// 生成路线顺序

    size_t offset = 0;
    for (size_t vehType = 0; vehType != data.numVehicleTypes(); vehType++)
    {
        vehTypeOrder_.emplace_back(vehType, offset);
        offset += data.vehicleType(vehType).numAvailable;
    }
}

void SearchSpace::setNeighbours(Neighbours neighbours)// 设置邻居结构，验证其有效性
{
    if (neighbours.size() != neighbours_.size())
        throw std::runtime_error("Neighbourhood dimensions do not match.");

    size_t numDepots = neighbours_.size() - clientOrder_.size();// 计算仓库数量
    for (size_t client = numDepots; client != neighbours.size(); ++client)// 遍历每个客户，验证其邻居列表
    {
        auto const beginPos = neighbours[client].begin();
        auto const endPos = neighbours[client].end();

        auto const pred
            = [&](auto item) { return item == client || item < numDepots; };// 验证邻居列表中是否包含自身或仓库

        if (std::any_of(beginPos, endPos, pred))
        {
            throw std::runtime_error("Neighbourhood of client "
                                     + std::to_string(client)
                                     + " contains itself or a depot.");
        }
    }

    neighbours_ = neighbours;
}

SearchSpace::Neighbours const &SearchSpace::neighbours() const
{
    return neighbours_;
}

std::vector<size_t> const &SearchSpace::neighboursOf(size_t client) const
{
    return neighbours_[client];
}

bool SearchSpace::isPromising(size_t client) const
{
    assert(client < neighbours_.size());
    return promising_[client];
}

void SearchSpace::markPromising(size_t client)
{
    assert(client < neighbours_.size());
    promising_[client] = true;
}
//将给定节点(非仓库)及其直接客户邻居标记为有希望的
void SearchSpace::markPromising(Route::Node const *node)
{
    assert(node->route());

    if (!node->isDepot())
        markPromising(node->client());

    if (!node->isStartDepot() && !p(node)->isDepot())//前驱结点也标记为有希望的
        markPromising(p(node)->client());

    if (!node->isEndDepot() && !n(node)->isDepot())//后继结点也标记为有希望的
        markPromising(n(node)->client());
}

void SearchSpace::markAllPromising() { promising_.set(); }

void SearchSpace::unmarkAllPromising() { promising_.reset(); }

std::vector<size_t> const &SearchSpace::clientOrder() const
{
    return clientOrder_;
}

std::vector<size_t> const &SearchSpace::routeOrder() const
{
    return routeOrder_;
}

std::vector<std::pair<size_t, size_t>> const &SearchSpace::vehTypeOrder() const
{
    return vehTypeOrder_;
}

void SearchSpace::shuffle(RandomNumberGenerator &rng)
{
    rng.shuffle(clientOrder_.begin(), clientOrder_.end());
    rng.shuffle(routeOrder_.begin(), routeOrder_.end());
    rng.shuffle(vehTypeOrder_.begin(), vehTypeOrder_.end());
}
