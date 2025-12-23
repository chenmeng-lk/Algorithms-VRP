#ifndef _FILO2_SMALLFLATMAP_HPP_
#define _FILO2_SMALLFLATMAP_HPP_

// SmallFlatMap.hpp
// 小型扁平映射容器实现文件
// 实现一个轻量级、内存紧凑的键值对映射容器，使用开放寻址法解决哈希冲突
// 适用于需要快速查找且键值对数量较少（小于maxSize）的场景

#include <cstddef>
#include <utility>
#include "functor.hpp"

namespace cobra {

    // 小型扁平映射模板类
    // Key: 键类型
    // Value: 值类型
    // emptyKey: 空键标记值
    // maxSize: 最大容量（实际内部缓冲区会更大以保持低负载因子）
    // op: 键转换函数对象（默认使用identity_functor，即不转换）
    template <typename Key, typename Value, Key emptyKey, int maxSize, class op = identity_functor<Key>>
    class SmallFlatMap {

        // 键值对类型别名
        using KVpair = std::pair<Key, Value>;
        // 键值对指针类型别名
        using KVpair_ptr = KVpair*;

        // 静态断言：确保最大容量为正数
        static_assert(maxSize > 0, "Needs a positive value size.");
        // 静态断言：确保maxSize为整型
        static_assert(std::is_integral_v<decltype(maxSize)>, "Needs a integral type for maxSize");
        // 静态断言：确保容量不会过大（最大约65536字节）
        static_assert(maxSize * 5 / 4 <= (1 << 16), "Choose a smaller maxSize.");
        
        // 计算大于等于v的最小2的幂次（位运算技巧）
        static constexpr int next2pow(int v) {
            v--;//先减一再加一不然得到的是大于v的最小2的幂次-1
            //减1最高位若从1变0不影响，说明v已经是2的幂次了，位运算后得到的是v-1
            //最高位没变，则从这开始到最低位都变成1，加一即得到了大于等于v的最小2的幂次
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;//通俗来说就是把最高位的1及之后的所有位全部置为1
            return ++v;
        }
        
        // 静态断言：确保总内存占用不超过65KB（保守限制）
        static_assert(next2pow(maxSize * 5 / 4) * sizeof(KVpair) <= (1 << 16), "Maximum memory occupation of 65KB.");

    public:
        // 自定义迭代器类：用于遍历非空键值对
        class custom_iterator {
            friend class SmallFlatMap<Key, Value, emptyKey, maxSize, op>;

        private:
            // 私有构造函数：由SmallFlatMap的begin()和end()调用
            custom_iterator(KVpair_ptr _base, KVpair_ptr _end) : base(_base), end(_end) {
                // 跳过空键值对，定位到第一个有效元素
                while (base != end && base->first == emptyKey) ++base;
            };

        public:
            // 解引用操作符：返回当前键值对引用
            inline auto& operator*() {
                return *base;
            }

            // 箭头操作符：返回当前键值对指针
            inline auto operator->() {
                return base;
            }

            // 前缀递增操作符：移动到下一个非空元素
            inline auto& operator++() {
                do {
                    ++base;
                } while (base != end && base->first == emptyKey);
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
            KVpair_ptr base;       // 当前元素指针
            const KVpair_ptr end;  // 结束位置指针
        };

    public:
        // 构造函数：初始化所有键为空标记
        SmallFlatMap() {
            for (auto& p : buffer) p.first = emptyKey;
        };

        // 查找函数：查找键为k的元素，返回键值对引用（线性探测法）
        inline std::pair<Key, Value>& find(const Key k) {
            // 计算初始哈希索引（使用键转换函数和掩码）
            auto index = op()(k) & realSizem1;//op()(k)是将k转换为int，& realSizem1是将int转换为0~realSize-1的int
            auto key = buffer[index].first;
            
            // 线性探测：直到找到匹配的键或空位置
            while (key != k && key != emptyKey) {
                index = (index + 1) & realSizem1;  // 使用掩码实现循环数组
                key = buffer[index].first;
            }
            return buffer[index];
        }

        // 插入函数：插入键值对，如果键已存在则返回false
        inline bool insert(const Key k, const Value v) {
            auto& candidate_place = find(k);
            if (candidate_place.first != emptyKey) return false;  // 元素已存在，插入失败

            candidate_place = {k, v};  // 插入新元素
            return true;
        }

        // 下标操作符：查找或创建键k对应的值引用
        inline auto& operator[](Key k) {
            auto& kv = find(k);
            kv.first = k;  // 注意：不检查大小，调用者需确保不会超过容量
            return kv.second;
        }

        // 清空函数：将所有键重置为空标记
        inline void clear() {
            for (auto& p : buffer) p.first = emptyKey;
        }

        // 计数函数：统计键k是否存在（0或1）
        inline size_t count(Key k) {
            return static_cast<size_t>(find(k).first != emptyKey);
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
        // 缓冲区数组：存储所有键值对
        std::pair<Key, Value> buffer[realSize];
    };
}  // namespace cobra

#endif