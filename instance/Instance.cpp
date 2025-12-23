#include "Instance.hpp"

#include <algorithm>

#include "../base/KDTree.hpp"
#include "../base/Timer.hpp"

#ifdef VERBOSE
    #include <iostream>
#endif

namespace cobra {

    // static
    //读取CVRP实例文件，并初始化每个顶点的邻居列表
    std::optional<Instance> Instance::make(const std::string& filepath, int neighbors_num) {

        Parser parser(filepath);//复制路径

        std::optional<Parser::Data> maybe_data = parser.Parse();//读取CVRP实例文件
        if (!maybe_data.has_value()) {
            return std::nullopt;
        }

        return Instance(maybe_data.value(), neighbors_num);
    }

    Instance::Instance(const Parser::Data& data, int neighbors_num) {

        neighbors_num = std::min(neighbors_num, static_cast<int>(data.demands.size()));//最近邻数量不大于顶点总数

        // Copy info from parsed data.
        //复制解析的数据到实例对象
        vehicle_capacity = data.vehicle_capacity;
        xcoords = std::move(data.xcoords);
        ycoords = std::move(data.ycoords);
        demands = std::move(data.demands);

        // Identify the neighbors of each vertex by using a K-d tree, see again the paper cited above.
        //使用K-d树识别每个顶点的邻居
        neighbors.resize(get_vertices_num());

        KDTree kd_tree(xcoords, ycoords);

#ifdef VERBOSE
        Timer timer;
#endif

        for (int i = get_vertices_begin(); i < get_vertices_end(); ++i) {//遍历所有顶点

            neighbors[i] = kd_tree.GetNearestNeighbors(xcoords[i], ycoords[i], neighbors_num);

            // Make sure the first vertex is `i`. Since we are not using all neighbors, if several vertices overlap and the number of
            // neighbors is not large enough we might not have `i` in the neighbors set. Let's cross the fingers and hope it does not
            // happen.
            //确保第一个邻居是i自己
            if (neighbors[i][0] != i) {
                auto n = 1;
                while (n < static_cast<int>(neighbors[i].size())) {
                    if (neighbors[i][n] == i) {
                        break;
                    }
                    n++;
                }
                std::swap(neighbors[i][0], neighbors[i][n]);
            }

            assert(neighbors[i][0] == i);

#ifdef VERBOSE
            if (timer.elapsed_time<std::chrono::seconds>() > 10) {
                std::cout << "Progress: " << 100 * (i + 1) / get_vertices_num() << "%\n";
                timer.reset();
            }
#endif
        }
    }

}  // namespace cobra