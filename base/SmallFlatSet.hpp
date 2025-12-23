#ifndef _FILO2_SMALLFLATSET_HPP_
#define _FILO2_SMALLFLATSET_HPP_

// SmallFlatSet.hpp
// 小型扁平集合容器实现文件
// 实现一个轻量级、内存紧凑的集合容器，使用开放寻址法解决冲突
// 适用于需要快速查找且元素数量较少（小于maxSize）的场景
// 基于SmallFlatMap实现，但只存储值，不存储键值对

#include <cstddef>
#include "functor.hpp"

namespace cobra {

    // 小型扁平集合模板类
    // Value: 值类型
    // emptyValue: 空值标记值
    // maxSize: 最大容量（实际内部缓冲区会更大以保持低负载因子）
    // op: 值转换函数对象（默认使用identity_functor，即不转换）
    template <typename Value, Value emptyValue, int maxSize, class op = identity_functor<Value>>
    class SmallFlatSet {

        // 静态断言：确保最大容量为正数
        static_assert(maxSize > 0, "Needs a positive value size.");
        // 静态断言：确保maxSize为整型
        static_assert(std::is_integral_v<decltype(maxSize)>, "Needs a integral type for maxSize");
        // 静态断言：确保容量不会过大（最大约65536字节）
        static_assert(maxSize * 5 / 4 <= (1 << 16), "Choose a smalle maxSize.");
        
        // 计算大于等于v的最小2的幂次（位运算技巧）
        static constexpr int next2pow(int v) {
            v--;
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;
            return ++v;
        }
        
        // 静态断言：确保总内存占用不超过65KB（保守限制）
        static_assert(next2pow(maxSize * 5 / 4) * sizeof(Value) <= (1 << 16), "Maximum memory occupation of 65KB.");

    public:
        // 自定义迭代器类：用于遍历非空元素
        class custom_iterator {
            friend class SmallFlatSet<Value, emptyValue, maxSize, op>;

        private:
            // 私有构造函数：由SmallFlatSet的begin()和end()调用
            custom_iterator(Value* _base, Value* _end) : base(_base), end(_end) {
                // 跳过空值，定位到第一个有效元素
                while (base != end && *base == emptyValue) ++base;
            };

        public:
            // 解引用操作符：返回当前值引用
            inline auto& operator*() {
                return *base;
            }

            // 箭头操作符：返回当前值指针
            inline auto operator->() {
                return base;
            }

            // 前缀递增操作符：移动到下一个非空元素
            inline auto& operator++() {
                do {
                    ++base;
                } while (base != end && *base == emptyValue);
                return *this;
            }

            // 不等操作符：比较两个迭代器
            inline auto operator!=(const custom_iterator x) {
                return x.base != base;
            }

            // 相等操作符：比较两个迭代器
            inline auto operator==(const custom_iterator x) {
                return x.base == base;
            }

        private:
            Value* base;        // 当前元素指针
            const Value* end;   // 结束位置指针
        };

    public:
        // 构造函数：初始化所有值为空标记
        SmallFlatSet() {
            for (auto& p : buffer) p = emptyValue;
        };

        // 查找函数：查找值为v的元素，返回值引用（线性探测法）
        inline Value& find(const Value v) {
            // 计算初始哈希索引（使用值转换函数和掩码）
            auto index = op()(v) & realSizem1;
            auto value = buffer[index];
            
            // 线性探测：直到找到匹配的值或空位置
            while (value != v && value != emptyValue) {
                index = (index + 1) & realSizem1;  // 使用掩码实现循环数组
                value = buffer[index];
            }
            return buffer[index];
        }

        // 插入函数：插入值v，如果值已存在则返回false
        inline bool insert(const Value v) {
            auto& candidate_place = find(v);
            if (candidate_place != emptyValue) return false;  // 元素已存在，插入失败

            candidate_place = v;  // 插入新元素
            return true;
        }

        // 插入或赋值函数：总是插入值v，如果已存在则覆盖
        inline bool insert_or_assign(const Value v) {
            auto& candidate_place = find(v);
            candidate_place = v;  // 插入或覆盖元素

            return true;
        }

        // 下标操作符：查找或创建值v对应的引用
        inline auto& operator[](Value v) {
            auto& value = find(v);
            value = v;  // 注意：不检查大小，调用者需确保不会超过容量
            return value;
        }

        // 清空函数：将所有值重置为空标记
        inline void clear() {
            for (auto& p : buffer) p = emptyValue;
        }

        // 计数函数：统计值v是否存在（0或1）
        inline size_t count(Value v) {
            return static_cast<size_t>(find(v) != emptyValue);
        }

        // 起始迭代器：返回指向第一个非空元素的迭代器
        inline auto begin() {
            return custom_iterator(buffer, buffer + realSize);
        };

        // 结束迭代器：返回指向末尾的迭代器
        inline auto end() {
            return custom_iterator(buffer + realSize, buffer + realSize);
        };

    private:
        // 实际缓冲区大小：取大于maxSize * 5/4的最小2的幂次（负载因子约0.8）
        // 为什么是5/4？这是为了使最大负载因子约为80%，减少哈希冲突
        constexpr static int realSize = next2pow(maxSize * 5 / 4);
        // 实际缓冲区大小减1，用于掩码运算（因为realSize是2的幂次）
        constexpr static int realSizem1 = realSize - 1;

    public:
        // 缓冲区数组：存储所有值
        Value buffer[realSize];
    };

}  // namespace cobra

#endif