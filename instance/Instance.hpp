// CVRP实例管理类
// 负责加载、存储和查询CVRP问题实例的所有信息
#ifndef _FILO2_INSTANCE_HPP_
#define _FILO2_INSTANCE_HPP_

#include <cassert>
#include <cmath>
#include <optional>
#include <string>

#include "../base/NonCopyable.hpp"
#include "Parser.hpp"

namespace cobra {
    namespace {
        // 快速四舍五入函数
        inline double fastround(double value) {
            return static_cast<int>(value + 0.5);
        }
    }  // namespace

    // Manages a CVRP instance by providing a set of methods to query its properties.
    // CVRP实例类
    // 存储问题的所有数据：客户位置、需求、距离、邻居列表等
    // 使用NonCopyable防止意外复制大对象
    class Instance : private NonCopyable<Instance> {
    public:
        // Returns an optional containing a properly built instance if the parsing of the input file completes correctly, nullopt otherwise.
        // The parameter `num_neighbors` specifies the number of neighbors that are precomputed for each vertice. These neighbors are then
        // accessibly by using the `get_neighbors_of` method.
        // 静态工厂方法：从文件创建实例
        // @param filepath 实例文件路径
        // @param num_neighbors 为每个顶点预计算的邻居数量
        // @return 如果解析成功返回Instance对象，否则返回nullopt
        static std::optional<Instance> make(const std::string& filepath, int num_neighbors);

        // Returns the instance size.
        // 返回实例大小（顶点总数 = 仓库 + 客户数）
        inline int get_vertices_num() const {
            return demands.size();
        }

        // Returns the depot's index.
        // 返回仓库索引（总是0）
        inline int get_depot() const {
            return 0;
        }

        // Returns the vehicle capacity.
        // 返回车辆容量
        inline int get_vehicle_capacity() const {
            return vehicle_capacity;
        };

        // Returns the number of customers.
        // 返回客户数量（不包括仓库）
        inline int get_customers_num() const {
            return get_vertices_num() - 1;
        };

        // Returns the index of the first customer.
        // 返回第一个客户的索引（总是1）
        inline int get_customers_begin() const {
            return 1;
        }

        // Returns the index after the last customer.
        // 返回最后一个客户之后的索引（用于循环终止条件）
        inline int get_customers_end() const {
            return get_vertices_num();
        }

        // Returns the index of the first vertex.
        // 返回第一个顶点的索引（即仓库，总是0）
        inline int get_vertices_begin() const {
            return get_depot();
        }

        // Returns the index after the last vertex.
        // 返回最后一个顶点之后的索引（用于循环终止条件）
        inline int get_vertices_end() const {
            return get_customers_end();
        }

        // Returns the cost of arc (i, j).
        // 返回弧(i, j)的成本（欧几里得距离，四舍五入到整数）
        // 使用欧几里得距离公式：sqrt((xi-xj)^2 + (yi-yj)^2)
        inline double get_cost(int i, int j) const {
            assert(i >= get_vertices_begin() && i < get_vertices_end());
            assert(j >= get_vertices_begin() && j < get_vertices_end());

            const double sqrt = std::sqrt((xcoords[i] - xcoords[j]) * (xcoords[i] - xcoords[j]) +
                                          (ycoords[i] - ycoords[j]) * (ycoords[i] - ycoords[j]));
            assert(static_cast<int>(std::round(sqrt)) == static_cast<int>(fastround(sqrt)));

            return fastround(sqrt);
        }

        // Returns the demand of vertex `i`. The demand is 0 for the depot.
        // 返回顶点i的需求量（仓库的需求为0）
        inline int get_demand(int i) const {
            return demands[i];
        };

        // Returns the x coordinate of vertex `i`.
        // 返回顶点i的x坐标
        inline double get_x_coordinate(int i) const {
            return xcoords[i];
        };

        // Returns the y coordinate of vertex `i`.
        // 返回顶点i的y坐标
        inline double get_y_coordinate(int i) const {
            return ycoords[i];
        };

        // Returns an array of vertices sorted according to increasing cost from `i`. This array always includes `i` in the first
        // position. The total number of elements in the array is defined by the `num_neighbors` parameter in the `Instance` constructor.
        // 返回顶点i的邻居列表（按距离升序排序）
        // 列表的第一个元素总是i自己
        // 列表长度由构造时的num_neighbors参数决定
        inline const std::vector<int>& get_neighbors_of(int i) const {
            return neighbors[i];
        };

    private:
        // 私有构造函数，只能通过make()方法创建实例
        Instance(const Parser::Data& data, int neighbors_num);

        // Maximum vehicle capacity.
        // 车辆最大容量
        int vehicle_capacity;

        // Vertices x coordinates.
        // 所有顶点的x坐标
        std::vector<double> xcoords;

        // Vertices y coordinates.
        // 所有顶点的y坐标
        std::vector<double> ycoords;

        // Vertices demands.
        // 所有顶点的需求量（仓库为0）
        std::vector<int> demands;

        // Neighbors for each vertex.
        // 每个顶点的邻居列表（预计算，按距离排序）
        std::vector<std::vector<int>> neighbors;
    };

}  // namespace cobra

#endif