#ifndef PYVRP_MATRIX_H
#define PYVRP_MATRIX_H

#include <algorithm>
#include <cassert>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace pyvrp
{
template <typename T> class Matrix
{
    size_t cols_ = 0;           // The number of columns of the matrix 矩阵的列数
    size_t rows_ = 0;           // The number of rows of the matrix 矩阵的行数
    std::vector<T> data_ = {};  // Data vector 存储矩阵元素的一维向量

public:
    Matrix() = default;  // default is an empty matrix 默认构造函数，创建空矩阵

    /**
     * Creates a matrix of size nRows * nCols, initialised with the given
     * value.
     * 创建一个大小为 nRows * nCols 的矩阵，并用给定值初始化所有元素
     *
     * @param nRows Number of rows. 行数
     * @param nCols Number of columns. 列数
     * @param value Initial value to fill the matrix with. 用于填充矩阵的初始值
     */
    explicit Matrix(size_t nRows, size_t nCols, T value = {});

    // 从现有数据向量移动构造矩阵
    explicit Matrix(std::vector<T> &&data, size_t nRows, size_t nCols);

    bool operator==(Matrix const &other) const = default;

    // 访问矩阵元素的运算符重载，支持读写
    [[nodiscard]] decltype(auto) operator()(size_t row, size_t col);
    // 访问矩阵元素的运算符重载，只读版本
    [[nodiscard]] decltype(auto) operator()(size_t row, size_t col) const;

    // 返回指向矩阵数据起始位置的常量迭代器
    typename std::vector<T>::const_iterator begin() const;
    // 返回指向矩阵数据结束位置的常量迭代器
    typename std::vector<T>::const_iterator end() const;

    // 返回指向矩阵数据起始位置的迭代器
    typename std::vector<T>::iterator begin();
    // 返回指向矩阵数据结束位置的迭代器
    typename std::vector<T>::iterator end();

    // 返回指向矩阵底层数据的指针，支持修改
    [[nodiscard]] T *data();
    // 返回指向矩阵底层数据的常量指针，只读访问
    [[nodiscard]] T const *data() const;

    // 获取矩阵的列数
    [[nodiscard]] size_t numCols() const;

    // 获取矩阵的行数
    [[nodiscard]] size_t numRows() const;

    /**
     * @return Maximum element in the matrix.
     * 返回矩阵中的最大元素
     */
    [[nodiscard]] T max() const;

    /**
     * @return Matrix size.
     * 返回矩阵中元素的总数（行数 × 列数）
     */
    [[nodiscard]] size_t size() const;
};

template <typename T>
Matrix<T>::Matrix(size_t nRows, size_t nCols, T value)
    : cols_(nCols), rows_(nRows), data_(nRows * nCols, value)
{
    // 构造函数实现：初始化行列数，并用指定值填充数据向量
}

template <typename T>
Matrix<T>::Matrix(std::vector<T> &&data, size_t nRows, size_t nCols)
    : cols_(nCols), rows_(nRows), data_(std::forward<std::vector<T>>(data))
{
    assert(cols_ * rows_ == data_.size());  // 验证数据大小与矩阵尺寸匹配
}

template <typename T>
decltype(auto) Matrix<T>::operator()(size_t row, size_t col)
{
    return data_[cols_ * row + col];  // 通过行优先索引计算公式访问元素
}

template <typename T>
decltype(auto) Matrix<T>::operator()(size_t row, size_t col) const
{
    return data_[cols_ * row + col];  // 常量版本的矩阵元素访问
}

template <typename T>
typename std::vector<T>::const_iterator Matrix<T>::begin() const
{
    return data_.begin();  // 返回常量起始迭代器
}

template <typename T>
typename std::vector<T>::const_iterator Matrix<T>::end() const
{
    return data_.end();  // 返回常量结束迭代器
}

template <typename T> typename std::vector<T>::iterator Matrix<T>::begin()
{
    return data_.begin();  // 返回起始迭代器
}

template <typename T> typename std::vector<T>::iterator Matrix<T>::end()
{
    return data_.end();  // 返回结束迭代器
}

template <typename T> T *Matrix<T>::data() { return data_.data(); }  // 返回底层数据指针
template <typename T> T const *Matrix<T>::data() const { return data_.data(); }  // 返回常量底层数据指针

template <typename T> size_t Matrix<T>::numCols() const { return cols_; }  // 返回列数

template <typename T> size_t Matrix<T>::numRows() const { return rows_; }  // 返回行数

template <typename T> T Matrix<T>::max() const
{
    return *std::max_element(data_.begin(), data_.end());  // 返回矩阵中的最大值
}

template <typename T> size_t Matrix<T>::size() const { return data_.size(); }  // 返回矩阵元素总数
}  // namespace pyvrp

#endif  // PYVRP_MATRIX_H
