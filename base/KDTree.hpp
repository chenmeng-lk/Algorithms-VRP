// K-D树 (K-Dimensional Tree)
// 用于高效地进行k近邻搜索的空间数据结构
// 基于 https://github.com/cdalitz/kdtree-cpp 实现
#ifndef _FILO2_KDTREE_HPP_
#define _FILO2_KDTREE_HPP_

#include <array>
#include <queue>
#include <vector>

namespace cobra {

    // A simple implementation of a kd-tree based on https://github.com/cdalitz/kdtree-cpp.
    // K-D树类
    //
    // 功能：
    // - 高效的k近邻搜索（k-nearest neighbors search）
    // - 适用于2D点集（可扩展到更高维度）
    //
    // 时间复杂度：
    // - 构建：O(n log n)
    // - 查询：平均O(log n)，最坏O(n)
    //
    // 在FILO2中的应用：
    // - 预计算每个顶点的k个最近邻居
    // - 用于粒度规则（Granularity Rule）
    // - 减少搜索空间，提高算法效率
    class KDTree {
    public:
        // 构造函数
        // @param xcoords 所有点的x坐标
        // @param ycoords 所有点的y坐标
        KDTree(const std::vector<double>& xcoords, const std::vector<double>& ycoords);
        ~KDTree();

        // Retrieves the k nearest neighbors of point (x, y).
        // 获取点(x, y)的k个最近邻居
        // @param x 查询点的x坐标
        // @param y 查询点的y坐标
        // @param k 需要的邻居数量
        // @return 按距离从近到远排序的邻居索引列表
        std::vector<int> GetNearestNeighbors(double x, double y, int k) const;

    private:
        // A 2D point representation.
        // 2D点的表示
        //存储点的索引和2D坐标
        struct Point {
            Point(int index, double x, double y);
            // Point index.
            int index;  // 点的索引（在原始数据中的位置）
            // x and y coordinates.
            std::array<double, 2> coords;  // x和y坐标
        };

        // A KDTree node representation.
        // K-D树节点的表示
        //k-d树的节点
        //
        // K-D树是一种二叉搜索树，每层按不同维度分割空间
        // - 第0层按x坐标分割
        // - 第1层按y坐标分割
        // - 第2层又按x坐标分割，以此类推
        struct Node {
            Node();
            ~Node();
            // Cut dimension: 0 for the x coordinate, 1 for the y one. 分割维度（x或y）
            int cutdim;  // 分割维度：0表示x坐标，1表示y坐标
            // Child nodes. 左右子节点指针
            Node *left, *right;  // 左子树（小于分割值）和右子树（大于等于分割值）
            // Node bounding box. 节点的边界框（用于剪枝）
            std::array<double, 2> lobound, hibound;  // 节点覆盖的空间范围（用于剪枝优化）
            // Index of the point in nodes vector. 存储的点索引
            int point_index;  // 该节点存储的点在nodes向量中的索引
        };
        Node* root = nullptr;  // 树的根节点

        // Heap node representation. 用于优先队列的堆节点，存储距离
        // 堆节点表示
        // 用于维护k个最近邻居的最大堆
        struct HeapNode {
            HeapNode(int point_index, double distance);
            // Index of the point in nodes vector.
            int point_index;  // 点的索引
            // Distance of this neighbor from the input point.
            double distance;  // 该邻居到查询点的距离（平方距离）
        };

        // Heap node comparator.
        // 堆节点比较器
        // 用于构建最大堆（距离最大的在堆顶）
        // 这样可以方便地淘汰距离最远的邻居
        struct HeapNodeComparator {
            bool operator()(const HeapNode& a, const HeapNode& b) const;
        };

        // Max heap used to retrieve the set of nearest neighbors of a given point.
        // 最大堆类型定义
        // 用于维护k个最近邻居
        // 堆顶是当前k个邻居中距离最远的
        using KDTreeHeap = std::priority_queue<HeapNode, std::vector<HeapNode>, HeapNodeComparator>;

        // Builds the tree using nodes indexed from begin to end excluded.
        // 递归构建K-D树
        // @param depth 当前深度（决定分割维度）
        // @param begin 节点范围起始索引（包含）
        // @param end 节点范围结束索引（不包含）
        // @param lobound 当前节点的下界
        // @param hibound 当前节点的上界
        // @return 构建的子树根节点
        Node* BuildTree(int depth, int begin, int end, const std::array<double, 2>& lobound, const std::array<double, 2>& hibound);

        // 递归搜索k近邻
        // @param tree 当前搜索的子树
        // @param heap 维护k个最近邻居的最大堆
        // @param point 查询点
        // @param k 需要的邻居数量
        // @return 如果查询球完全在节点边界内则返回true（可以提前终止）
        bool SearchNeighbors(Node* tree, KDTreeHeap& heap, const std::array<double, 2>& point, int k) const;

        // 检查节点边界框是否与查询球相交
        // @param point 查询点
        // @param dist 查询半径（平方距离）
        // @param node 要检查的节点
        // @return 如果相交返回true
        bool BoundsOverlapBall(const std::array<double, 2>& point, double dist, Node* node) const;

        // 检查查询球是否完全在节点边界内
        // @param point 查询点
        // @param dist 查询半径（平方距离）
        // @param node 要检查的节点
        // @return 如果查询球完全在边界内返回true
        bool BallWithinBounds(const std::array<double, 2>& point, double dist, KDTree::Node* node) const;

        std::vector<Point> nodes;  // 存储所有点的向量
    };

}  // namespace cobra

#endif