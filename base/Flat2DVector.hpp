// 扁平化二维向量 (Flat 2D Vector)
// 使用一维数组模拟二维数组，提高缓存局部性
#ifndef _FILO2_FLAT2DVECTOR_HPP_
#define _FILO2_FLAT2DVECTOR_HPP_

#include <vector>

namespace cobra {

    // 扁平化二维向量类
    //
    // 实现原理：
    // - 使用一维数组存储二维数据
    // - 行主序存储：data[i][j] = flat_data[i * cols + j]
    //
    // 优点：
    // - 内存连续，缓存友好
    // - 避免指针间接访问
    // - 减少内存分配次数
    //
    // 缺点：
    // - 不支持动态改变行数或列数（需要重新分配）
    template <typename T>
    class Flat2DVector {
    public:
        Flat2DVector() : rows(0), cols(0) { }

        // 调整大小
        // @param rows_ 行数
        // @param cols_ 列数
        inline void resize(size_t rows_, size_t cols_) {
            rows = rows_;
            cols = cols_;
            data.resize(rows * cols, 0);
        }

        // 设置元素值
        // @param i 行索引
        // @param j 列索引
        // @param value 要设置的值
        inline void at(const int i, const int j, T value) {
            data[i * cols + j] = value;
        }

        // 获取元素值
        // @param i 行索引
        // @param j 列索引
        // @return 元素值
        inline T at(const int i, const int j) const {
            return data[i * cols + j];
        }

    private:
        std::vector<T> data;  // 扁平化的一维数组

        size_t rows;  // 行数
        size_t cols;  // 列数
    };

}  // namespace cobra

#endif