#ifndef _FILO2_BITMATRIX_HPP_
#define _FILO2_BITMATRIX_HPP_

// BitMatrix.hpp
// 位矩阵实现文件
// 实现一个稀疏位矩阵数据结构，用于高效存储和操作二元关系
// 使用SmallFlatSet实现每行的位集合，适合存储每行设置位较少的稀疏矩阵

#include <cmath>
#include <vector>

#include "NonCopyable.hpp"
#include "SmallFlatSet.hpp"

namespace cobra {

    // 位矩阵模板类
    // maxSize: 每行最大设置位数（即每行最多有多少位被置为1）
    // 继承NonCopyable禁止拷贝操作
    template <int maxSize>
    class BitMatrix : NonCopyable<BitMatrix<maxSize>> {
    public:
        // 构造函数：创建指定行数的位矩阵
        BitMatrix(int rows) : data(rows) { }

        // 重置行：将指定行的所有位清零
        inline void reset(int row) {
            data[row].clear();
        }

        // 设置位：在指定行和列的位置设置位为1
        inline void set(int row, int entry) {
            data[row].insert(entry);
        }

        // 检查位：检查指定行和列的位置是否被设置为1
        inline bool is_set(int row, int entry) {
            return static_cast<bool>(data[row].count(entry));
        }

        // 覆盖行：将源行的位集合复制到目标行
        inline void overwrite(int source_row, int destination_row) {
            data[destination_row] = data[source_row];
        }

        // 获取行中设置的位集合（可能包含重复项，但SmallFlatSet不存储重复）
        // 返回SmallFlatSet的引用，可用于遍历该行所有设置位
        inline auto& get_set_entries_possibly_with_duplicates(int row) {
            return data[row];
        }

    private:
        // 数据存储：每行是一个SmallFlatSet，存储该行中所有设置为1的列索引
        // 使用unsigned int作为列索引类型，~0（全1）作为空值标记
        std::vector<SmallFlatSet<unsigned int, static_cast<unsigned int>(~0), maxSize>> data;
    };

}  // namespace cobra

#endif
