// 二叉堆 (Binary Heap)
// 通用的二叉堆实现，支持自定义比较器和索引管理
// 用于实现优先队列，支持O(log n)的插入、删除和更新操作
#ifndef _FILO2_BINARYHEAP_HPP_
#define _FILO2_BINARYHEAP_HPP_

#include <cassert>
#include <iostream>
#include <vector>

#include "functor.hpp"

// 二叉堆的索引计算宏
#define LEFT(X) (2 * (X) + 1)      // 左子节点索引
#define RIGHT(X) (2 * (X) + 2)     // 右子节点索引
#define PARENT(X) (((X)-1) / 2)    // 父节点索引

namespace cobra {

    // Generic binary heap.
    // 通用二叉堆类
    //
    // 模板参数：
    // @tparam T 元素类型
    // @tparam Cmp 比较器，定义堆的顺序（最小堆或最大堆）
    // @tparam GetIdx 获取元素在堆中索引的函数对象
    // @tparam SetIdx 设置元素在堆中索引的函数对象
    // @tparam Updt 更新元素值的函数对象
    // @tparam unheaped 表示元素不在堆中的特殊索引值（默认-1）
    //
    // 在FILO2中的应用：
    // - MoveGeneratorsHeap使用此类维护移动生成器
    // - 按delta值排序，快速获取最优移动
    template <typename T, class Cmp, class GetIdx, class SetIdx, class Updt, int unheaped = -1>
    class BinaryHeap {
    public:
        BinaryHeap() { }
        BinaryHeap(const BinaryHeap& bh) : heap(bh.heap) { }
        BinaryHeap(BinaryHeap&& bh) : heap(std::move(bh.heap)) { }

        // Resets the data structure.
        // 重置堆 - 清空所有元素
        void reset() {
            for (auto& t : heap) {
                SetIdx()(t, unheaped);
            }
            heap.clear();
        }

        // Returns whether the heap is empty.
        // 检查堆是否为空
        bool empty() const {
            return heap.empty();
        }

        // Inserts an element.
        // 插入元素
        // 将元素添加到堆尾，然后上浮到正确位置
        void insert(T elem) {
            const int hindex = heap.size();
            SetIdx()(elem, hindex);
            heap.emplace_back(elem);
            upsift(hindex);  // 上浮操作

            assert(is_heap());
        }

        // Returns the heap head.
        // 获取并移除堆顶元素（最小或最大元素）
        T get() {
            assert(!heap.empty());

            SetIdx()(heap[0], unheaped);
            auto elem = std::move(heap[0]);

            if (heap.size() == 1) {
                heap.pop_back();
                return elem;
            }

            // 将最后一个元素移到堆顶，然后下沉
            SetIdx()(heap.back(), 0);
            heap[0] = std::move(heap.back());
            heap.pop_back();
            heapify(0);  // 下沉操作

            assert(is_heap());
            return elem;
        }

        // Removes a specific heaped entry given its heap index.
        // 移除指定索引的元素
        void remove(int hindex) {

            if (hindex < static_cast<int>(heap.size()) - 1) {
                auto last = std::move(heap.back());
                heap.pop_back();
                replace(hindex, std::move(last));
            } else {
                SetIdx()(heap[hindex], unheaped);
                heap.pop_back();
            }

            assert(is_heap());
        }

        // Replaces the element at `hindex` with the given `elem`.
        // 替换指定索引的元素
        // 根据新旧元素的比较结果，决定上浮或下沉
        void replace(int hindex, T elem) {
            assert(hindex >= 0 && hindex < static_cast<int>(heap.size()));

            const auto case3 = Cmp()(heap[hindex], elem);

            SetIdx()(heap[hindex], unheaped);
            SetIdx()(elem, hindex);
            heap[hindex] = elem;

            if (case3 > 0) {
                upsift(hindex);      // 新元素更小，上浮
            } else if (case3 < 0) {
                heapify(hindex);     // 新元素更大，下沉
            }

            assert(is_heap());
        }

        // Returns the number of heap elements.
        // 获取堆中元素数量
        auto size() const {
            return heap.size();
        }

        // Returns the element at the given `hindex` without modifying the heap.
        // 查看指定索引的元素（不修改堆）
        T& spy(int hindex) {
            return heap[hindex];
        }

        // 更新指定索引的元素值
        // 使用Updt函数对象更新元素，然后调整堆
        template <typename... Args>
        void update(int hindex, Args&&... args) {
            assert(hindex >= 0 && hindex < static_cast<int>(heap.size()));

            const auto case3 = Updt()(heap[hindex], std::forward<Args>(args)...);

            if (case3 > 0) {
                upsift(hindex);      // 值变小，上浮
            } else if (case3 < 0) {
                heapify(hindex);     // 值变大，下沉
            }

            assert(is_heap());
        }

        // Implement to print out some info about the heap.
        // 虚函数，用于打印堆的调试信息（子类可以重写）
        virtual void dump() { }

    private:
        // 找出父节点和两个子节点中的最小值
        // @return 最小子节点的索引，如果父节点最小则返回unheaped
        int inline min_lr(T& parent, int lindex, int rindex) {
            const int hsize = heap.size();
            auto smallest = lindex;
            if (rindex < hsize && Cmp()(heap[rindex], heap[lindex]) < 0) {
                smallest = rindex;
            }  // !! rindex < lindex always !!
            if (smallest < hsize && Cmp()(heap[smallest], parent) < 0) {
                return smallest;
            }
            return unheaped;
        }

        // 下沉操作（heapify）
        // 将指定索引的元素向下移动，直到满足堆性质
        // 用于删除或替换元素后恢复堆性质
        void heapify(int hindex) {

            auto smallest = min_lr(heap[hindex], LEFT(hindex), RIGHT(hindex));
            if (smallest == unheaped) {
                return;
            }

            auto elem = std::move(heap[hindex]);
            while (smallest != unheaped) {

                SetIdx()(heap[smallest], hindex);
                heap[hindex] = std::move(heap[smallest]);
                hindex = smallest;
                smallest = min_lr(elem, LEFT(hindex), RIGHT(hindex));
            }

            SetIdx()(elem, hindex);
            heap[hindex] = std::move(elem);
        }

        // 上浮操作（upsift）
        // 将指定索引的元素向上移动，直到满足堆性质
        // 用于插入新元素或更新元素值后恢复堆性质
        void upsift(int hindex) {

            if (hindex == 0) {
                return;
            }

            auto pindex = hindex;
            auto elem = std::move(heap[hindex]);
            while (hindex && Cmp()(elem, heap[pindex = PARENT(pindex)]) < 0) {

                SetIdx()(heap[pindex], hindex);
                heap[hindex] = std::move(heap[pindex]);
                hindex = pindex;
            }

            SetIdx()(elem, hindex);
            heap[hindex] = std::move(elem);
        }

        // 检查堆性质是否满足（用于调试）
        // 验证：
        // 1. 每个元素的索引是否正确
        // 2. 每个父节点是否小于等于其子节点
        bool is_heap() {

            const int hsize = heap.size();
            // 检查索引一致性
            for (int n = 0; n < hsize; ++n) {
                const auto& t = heap[n];
                if (auto idx = GetIdx()(t); idx != n) {
                    std::cout << "Heap index of element n is not n\n";
                    std::cout << idx << " " << n << "\n";
                    dump();
                    return false;
                }
            }

            // 检查堆性质
            for (int n = 0; n < hsize; ++n) {
                const auto lindex = LEFT(n);
                const auto rindex = RIGHT(n);
                if (lindex < hsize) {
                    if (Cmp()(heap[lindex], heap[n]) < 0) {
                        std::cout << "left: " << n << " > " << lindex << "\n";
                        dump();
                        return false;
                    }
                }
                if (rindex < hsize) {
                    if (Cmp()(heap[rindex], heap[n]) < 0) {
                        std::cout << "right: " << n << " > " << rindex << "\n";
                        dump();
                        return false;
                    }
                }
            }

            return true;
        }

        std::vector<T> heap;  // 堆数组
    };

    // 以下是用于指针类型元素的辅助函数对象
    // 通过成员指针访问对象的字段

    // 比较器 - 比较两个指针指向对象的指定字段
    template <typename N, auto field>
    struct CmpFieldPtr {
        auto operator()(N* r1, N* r2) {
            assert(r1 && r2);
            return access_field_functor<N, field>()(*r1) - access_field_functor<N, field>()(*r2);
        }
    };

    // 获取索引 - 获取指针指向对象的指定字段值
    template <typename N, auto field>
    struct GetIdxFieldPtr {
        auto operator()(N* r1) {
            assert(r1);
            return access_field_functor<N, field>()(*r1);
        }
    };

    // 设置索引 - 设置指针指向对象的指定字段值
    template <typename N, auto field>
    struct SetIdxFieldPtr {
        void operator()(N* r1, int idx) {
            assert(r1);
            access_field_functor<N, field>()(*r1) = idx;
        }
    };

    // 更新值 - 更新指针指向对象的指定字段值
    template <typename N, auto field>
    struct UpdtFieldPtr {
        auto operator()(N* r1, typename std::decay<decltype(std::declval<N>().*field)>::type val) {
            assert(r1);
            const auto res = access_field_functor<N, field>()(*r1) - val;
            access_field_functor<N, field>()(*r1) = val;
            return res;
        }
    };

    template <typename N, auto findex, auto fvalue>
    class BinaryHeapPtr
        : public BinaryHeap<N*, CmpFieldPtr<N, fvalue>, GetIdxFieldPtr<N, findex>, SetIdxFieldPtr<N, findex>, UpdtFieldPtr<N, fvalue>> { };

}  // namespace cobra

#endif