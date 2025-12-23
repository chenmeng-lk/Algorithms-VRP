// 稀疏整数集合 (Sparse Integer Set)
// 用于高效地存储和查询稀疏的整数集合
// 结合了位图和向量的优点：O(1)查询，O(n)清空（n为实际元素数）
#ifndef _FILO2_SPARSEINTSET_HPP_
#define _FILO2_SPARSEINTSET_HPP_

#include <cassert>
#include <cmath>
#include <vector>

namespace cobra {

    // 稀疏整数集合类
    //
    // 数据结构：
    // - flags: 位图，用于O(1)时间检查元素是否存在
    // - elements: 向量，存储实际的元素，用于快速遍历
    //
    // 优点：
    // - 插入：O(1)
    // - 查询：O(1)
    // - 清空：O(n)，其中n是实际元素数（而不是容量）
    // - 遍历：O(n)，只遍历实际存在的元素
    //
    // 在FILO2中的应用：
    // - 存储候选路径集合
    // - 存储需要更新的顶点集合
    class SparseIntSet {

    public:
        // 构造函数
        // @param entries_num 可能的最大元素值+1（即元素范围是[0, entries_num)）
        explicit SparseIntSet(unsigned int entries_num) {
            flags.resize(entries_num);
        }

        // 拷贝构造函数
        SparseIntSet(const SparseIntSet& other) {
            flags = other.flags;
        }

        // 赋值运算符
        SparseIntSet& operator=(const SparseIntSet& other) {
            flags = other.flags;
            return *this;
        }

        // 插入元素（带重复检查）
        // 如果元素已存在，不会重复插入
        inline void insert(int value) {
            const auto already_here = contains(value);
            if (!already_here) {
                insert_without_checking_existance(value);
            }
        }

        // 插入元素（不检查重复）
        // 调用者需要确保元素不存在，否则会导致重复
        inline void insert_without_checking_existance(int value) {
            assert(value < static_cast<int>(flags.size()));
            flags[value] = true;
            elements.push_back(value);
        }

        // 检查元素是否存在
        // @return true如果元素在集合中
        inline bool contains(int value) const {
            assert(value < static_cast<int>(flags.size()));
            return flags[value];
        }

        // 清空集合
        // 只需要遍历实际存在的元素，而不是整个flags数组
        void clear() {
            for (auto value : elements) {
                assert(value < static_cast<int>(flags.size()));
                flags[value] = false;
            }
            elements.clear();
        }

        // 获取所有元素
        const std::vector<int>& get_elements() const {
            return elements;
        }

        // 获取元素数量
        unsigned int size() const {
            return elements.size();
        }

    private:
        std::vector<bool> flags;        // 位图，用于快速查询
        std::vector<int> elements;      // 实际元素列表，用于遍历
    };

}  // namespace cobra

#endif