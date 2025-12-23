#include "KDTree.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace cobra {

    // K-D树构造函数
    // 构建过程：
    // 1. 计算所有点的边界框
    // 2. 创建Point对象
    // 3. 递归构建K-D树
    KDTree::KDTree(const std::vector<double>& xcoords, const std::vector<double>& ycoords) {

        assert(xcoords.size() == ycoords.size());

        // 初始化边界框为极值
        std::array<double, 2> lobound = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        std::array<double, 2> hibound = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};

        // 遍历所有点，计算边界框并创建Point对象
        for (int i = 0; i < static_cast<int>(xcoords.size()); ++i) {
            lobound[0] = std::min(lobound[0], xcoords[i]);
            lobound[1] = std::min(lobound[1], ycoords[i]);
            hibound[0] = std::max(hibound[0], xcoords[i]);
            hibound[1] = std::max(hibound[1], ycoords[i]);
            nodes.emplace_back(i, xcoords[i], ycoords[i]);
        }

        // 递归构建K-D树
        root = BuildTree(0, 0, nodes.size(), lobound, hibound);
    }

    // 析构函数 - 递归删除所有节点
    KDTree::~KDTree() {
        delete root;
    }

    // Point构造函数
    KDTree::Point::Point(int index, double x, double y) : index(index), coords({x, y}) { }

    // Node构造函数和析构函数
    KDTree::Node::Node() = default;
    KDTree::Node::~Node() {
        // 递归删除左右子树
        delete left;
        delete right;
    }

    // HeapNode构造函数
    KDTree::HeapNode::HeapNode(int point_index, double distance) : point_index(point_index), distance(distance) { }

    // 堆节点比较器
    // 返回true如果a的距离小于b（用于构建最大堆）
    bool KDTree::HeapNodeComparator::operator()(const KDTree::HeapNode& a, const KDTree::HeapNode& b) const {
        return a.distance < b.distance;
    }

    // 递归构建K-D树
    //
    // 算法步骤：
    // 1. 根据深度确定分割维度（depth % 2）
    // 2. 创建节点并初始化
    // 3. 如果只有一个点，直接存储
    // 4. 否则找到中位数点作为分割点，递归构建左右子树
    //
    // 时间复杂度：O(n log n)
    KDTree::Node* KDTree::BuildTree(int depth, int begin, int end, const std::array<double, 2>& lobound,
                                    const std::array<double, 2>& hibound) {

        // 根据深度确定分割维度
        // depth为偶数时按x坐标分割，奇数时按y坐标分割
        const int dimension = depth % 2;

        // 创建新节点
        KDTree::Node* node = new KDTree::Node();
        node->cutdim = dimension;
        node->left = nullptr;
        node->right = nullptr;
        node->lobound = lobound;
        node->hibound = hibound;

        // 如果范围内只有一个点，直接存储
        if (end - begin <= 1) {
            node->point_index = begin;
        } else {
            // 找到中位数索引
            int median = (begin + end) / 2;
            // 使用nth_element找到中位数点
            // 这会重排nodes，使得nodes[median]是第median小的元素
            std::nth_element(nodes.begin() + begin, nodes.begin() + median, nodes.begin() + end,
                             [dimension](const Point& a, const Point& b) { return a.coords[dimension] < b.coords[dimension]; });
            node->point_index = median;

            // 获取分割值
            const int cutval = nodes[median].coords[dimension];

            // 递归构建左子树（小于分割值的点）
            if (median - begin > 0) {
                std::array<double, 2> next_hibound = hibound;
                next_hibound[dimension] = cutval;  // 更新上界
                node->left = BuildTree(depth + 1, begin, median, node->lobound, next_hibound);
            }

            // 递归构建右子树（大于分割值的点）
            if (end - median > 1) {
                std::array<double, 2> next_lobound = lobound;
                next_lobound[dimension] = cutval;  // 更新下界
                node->right = BuildTree(depth + 1, median + 1, end, next_lobound, hibound);
            }
        }

        return node;
    }

    // 获取k个最近邻居
    //
    // 算法步骤：
    // 1. 创建最大堆
    // 2. 递归搜索K-D树
    // 3. 从堆中提取结果（按距离从近到远）
    //
    // 时间复杂度：平均O(log n)，最坏O(n)
    std::vector<int> KDTree::GetNearestNeighbors(double x, double y, int k) const {

        // 创建最大堆，用于维护k个最近邻居
        KDTreeHeap heap;

        // 递归搜索K-D树
        SearchNeighbors(root, heap, {x, y}, k);

        // 从堆中提取结果
        std::vector<int> neighbors(k);

        // 堆是最大堆，所以从后往前填充结果
        // 这样结果数组就是按距离从近到远排序的
        while (!heap.empty()) {
            const HeapNode& heap_node = heap.top();
            neighbors[--k] = nodes[heap_node.point_index].index;
            heap.pop();
        }

        return neighbors;
    }

    // 计算两点之间的平方距离
    // 使用平方距离避免开方运算，提高效率
    double ComputeDistance(const std::array<double, 2>& a, const std::array<double, 2>& b) {
        return (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]);
    }

    // 计算一维坐标的平方距离
    double ComputeCoordinateDistance(double a, double b) {
        return (a - b) * (a - b);
    }

    // 检查节点边界框是否与查询球相交
    //
    // 算法：
    // 计算查询点到边界框的最小距离
    // 如果该距离小于等于查询半径，则相交
    //
    // 用途：剪枝优化，避免搜索不可能包含近邻的子树
    bool KDTree::BoundsOverlapBall(const std::array<double, 2>& point, double dist, KDTree::Node* node) const {

        double distsum = 0;

        // 计算查询点到边界框的平方距离
        for (int i = 0; i < static_cast<int>(point.size()); ++i) {
            if (point[i] < node->lobound[i]) {
                // 查询点在边界框下方
                distsum += ComputeCoordinateDistance(point[i], node->lobound[i]);
                if (distsum > dist) return false;  // 提前终止
            } else if (point[i] > node->hibound[i]) {
                // 查询点在边界框上方
                distsum += ComputeCoordinateDistance(point[i], node->hibound[i]);
                if (distsum > dist) return false;  // 提前终止
            }
            // 如果查询点在边界框内，该维度距离为0
        }

        return true;
    }

    // 检查查询球是否完全在节点边界内
    //
    // 算法：
    // 检查查询点到边界框每个边的距离
    // 如果所有边的距离都大于查询半径，则查询球完全在边界内
    //
    // 用途：提前终止搜索，因为不可能有更近的点在其他子树中
    bool KDTree::BallWithinBounds(const std::array<double, 2>& point, double dist, KDTree::Node* node) const {

        for (int i = 0; i < static_cast<int>(point.size()); ++i) {
            // 如果查询点到任何边界的距离小于等于查询半径
            // 说明查询球可能与边界相交，不能提前终止
            if (ComputeCoordinateDistance(point[i], node->lobound[i]) <= dist ||
                ComputeCoordinateDistance(point[i], node->hibound[i]) <= dist) {
                return false;
            }
        }

        return true;
    }

    // 递归搜索k近邻
    //
    // 算法步骤（深度优先搜索 + 剪枝）：
    // 1. 检查当前节点的点，更新堆
    // 2. 先搜索查询点所在的子树（更可能包含近邻）
    // 3. 如果另一侧子树可能包含更近的点，也搜索它
    // 4. 检查是否可以提前终止
    //
    // 剪枝策略：
    // - 如果子树边界框与查询球不相交，跳过该子树
    // - 如果查询球完全在当前节点边界内，提前终止
    bool KDTree::SearchNeighbors(KDTree::Node* node, KDTree::KDTreeHeap& heap, const std::array<double, 2>& point, int k) const {

        // 计算当前节点的点到查询点的距离
        double currdist = ComputeDistance(point, nodes[node->point_index].coords);

        // 更新堆
        if (static_cast<int>(heap.size()) < k) {
            // 堆未满，直接插入
            heap.push(HeapNode(node->point_index, currdist));
        } else if (currdist < heap.top().distance) {
            // 找到更近的点，替换堆顶（最远的点）
            heap.pop();
            heap.push(HeapNode(node->point_index, currdist));
        }

        // 第一步：先搜索查询点所在的子树
        // 根据查询点在分割维度上的位置，决定先搜索哪个子树
        if (point[node->cutdim] < nodes[node->point_index].coords[node->cutdim]) {
            // 查询点在分割线左侧，先搜索左子树
            if (node->left) {
                if (SearchNeighbors(node->left, heap, point, k)) {
                    return true;  // 提前终止
                }
            }
        } else {
            // 查询点在分割线右侧，先搜索右子树
            if (node->right) {
                if (SearchNeighbors(node->right, heap, point, k)) {
                    return true;  // 提前终止
                }
            }
        }

        // 获取当前查询半径（堆中最远点的距离）
        double dist = static_cast<int>(heap.size()) < k ? std::numeric_limits<double>::max() : heap.top().distance;

        // 第二步：检查另一侧子树是否可能包含更近的点
        // 如果查询球与另一侧子树的边界框相交，需要搜索它
        if (point[node->cutdim] < nodes[node->point_index].coords[node->cutdim]) {
            // 之前搜索了左子树，现在检查右子树
            if (node->right && BoundsOverlapBall(point, dist, node->right)) {
                if (SearchNeighbors(node->right, heap, point, k)) {
                    return true;  // 提前终止
                }
            }
        } else {
            // 之前搜索了右子树，现在检查左子树
            if (node->left && BoundsOverlapBall(point, dist, node->left)) {
                if (SearchNeighbors(node->left, heap, point, k)) {
                    return true;  // 提前终止
                }
            }
        }

        // 更新查询半径
        if (static_cast<int>(heap.size()) == k) {
            dist = heap.top().distance;
        }

        // 检查是否可以提前终止
        // 如果查询球完全在当前节点边界内，不可能有更近的点在其他地方
        return BallWithinBounds(point, dist, node);
    }

}  // namespace cobra