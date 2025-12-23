#include "Parser.hpp"

#include <cstring>

namespace cobra {

    Parser::Parser(const std::string& filepath) : filepath(filepath) { }

    std::optional<Parser::Data> Parser::Parse() {//读取实例文件，存储到data中返回

        FILE* file = fopen(filepath.c_str(), "r");

        if (!file) {//打开失败
            return std::nullopt;
        }

        Parser::Data data;

        char name[64];
        char edgeWeightType[64];

        if (fscanf(file, "NAME : %s\n", name) != 1) return std::nullopt;
        if (fscanf(file, "COMMENT : %*[^\n]\n") != 0) return std::nullopt;//读取所有直到换行符的字符，但不保存它们，最后是换行符
        if (fscanf(file, "TYPE : %*[^\n]\n") != 0) return std::nullopt;

        int matrix_size;
        if (fscanf(file, "DIMENSION : %d\n", &matrix_size) != 1) return std::nullopt;
        if (fscanf(file, "EDGE_WEIGHT_TYPE : %s\n", edgeWeightType) != 1) return std::nullopt;

        if (fscanf(file, "CAPACITY : %d\n", &data.vehicle_capacity) != 1) return std::nullopt;//返回成功匹配和赋值的输入项数量

        data.xcoords.resize(matrix_size);
        data.ycoords.resize(matrix_size);

        if (fscanf(file, "NODE_COORD_SECTION\n") != 0) return std::nullopt;

        int vertex_index;
        //读取坐标
        for (int i = 0; i < matrix_size; ++i) {
            if (fscanf(file, "%d %lf %lf", &vertex_index, &data.xcoords[i], &data.ycoords[i]) != 3) return std::nullopt;
        }

        if (fscanf(file, "\n") != 0) return std::nullopt;
        if (fscanf(file, "DEMAND_SECTION\n") != 0) return std::nullopt;

        data.demands.resize(matrix_size);//读取需求
        for (int i = 0; i < matrix_size; i++) {
            if (fscanf(file, "%d %d", &vertex_index, &data.demands[i]) != 2) return std::nullopt;
        }

        fclose(file);

        return data;
    }

}  // namespace cobra