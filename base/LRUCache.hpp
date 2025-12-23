// LRU缓存 (Least Recently Used Cache)
// 用于跟踪最近访问的顶点，实现增量更新策略
// 在FILO2中用作SVC (Set of Visited Customers)
#ifndef _FILO2_LRUCACHE_HPP_
#define _FILO2_LRUCACHE_HPP_

#include <cassert>
#include <vector>


namespace cobra {

    // LRU缓存类
    // 使用双向链表实现，支持O(1)时间的插入、删除和访问
    //
    // 工作原理：
    // - 最近访问的元素放在链表头部
    // - 最久未访问的元素在链表尾部
    // - 当缓存满时，移除尾部元素
    //
    // 在FILO2中的应用：
    // - 跟踪最近修改的顶点（SVC）
    // - 只对这些顶点相关的移动生成器进行更新
    // - 大幅减少增量更新的计算量
    class LRUCache {

    public:
        // 构造函数
        // @param capacity_ 缓存容量（最多存储多少个元素）
        // @param vertices_num_ 顶点总数（用于初始化缓存数组）
        LRUCache(int capacity_, int vertices_num_) : cache_size(capacity_), cache(vertices_num_), counter(0) { }

        // 拷贝构造函数
        LRUCache(const LRUCache& other)
            : cache_size(other.cache_size), cache(other.cache), counter(other.counter), head(other.head), tail(other.tail) { }

        // 赋值运算符
        LRUCache& operator=(const LRUCache& other) {
            cache_size = other.cache_size;
            cache = other.cache;
            counter = other.counter;
            head = other.head;
            tail = other.tail;
            return *this;
        }

        // 插入或更新一个顶点
        // 如果顶点已在缓存中，将其移到头部
        // 如果顶点不在缓存中，添加到头部，必要时移除尾部元素
        void insert(int vertex) {

            if (cache[vertex].used) {
                // 顶点已在缓存中，移到头部

                remove(vertex);
                splay_on_top(vertex);

            } else {
                // 顶点不在缓存中

                // Reached the cache size, evict the least recently used entry.
                // 如果缓存已满，移除最久未使用的元素（尾部）
                if (counter == cache_size) {
                    remove(tail);
                } else {
                    counter++;
                }
                // Move the recently accessed entry to top.
                // 将新访问的元素移到头部
                splay_on_top(vertex);
            }
        }

        // 清空缓存 - 移除所有元素
        void clear() {
            counter = 0;

            auto curr = head;
            while (curr != Entry::dummy_vertex) {
                const auto next = cache[curr].next;
                cache[curr].used = false;
                cache[curr].next = Entry::dummy_vertex;
                cache[curr].prev = Entry::dummy_vertex;
                curr = next;
            }

            head = Entry::dummy_vertex;
            tail = Entry::dummy_vertex;
        }

        // 获取缓存中的元素数量
        int size() const {
            return counter;
        }

        // 检查缓存是否为空
        bool empty() const {
            return counter == 0;
        }

        // 获取链表头部（最近访问的元素）
        inline int begin() const {
            return head;
        }
        // 获取链表尾部（最久未访问的元素）
        inline int last() const {
            return tail;
        }
        // 获取指定顶点的下一个顶点
        inline int get_next(int vertex) const {
            return cache[vertex].next;
        }
        // 获取指定顶点的前一个顶点
        inline int get_prev(int vertex) const {
            return cache[vertex].prev;
        }

        // 获取链表结束标记
        inline int end() const {
            return Entry::dummy_vertex;
        }

    private:
        // 缓存条目类 - 双向链表节点
        class Entry {
        public:
            int prev = dummy_vertex;        // 前驱节点
            int next = dummy_vertex;        // 后继节点
            bool used = false;              // 是否在缓存中
            static constexpr int dummy_vertex = -1;  // 哨兵值
        };

        int cache_size;                     // 缓存容量
        std::vector<Entry> cache;           // 缓存数组（索引即为顶点ID）
        int counter = 0;                    // 当前缓存中的元素数量
        int head = Entry::dummy_vertex;     // 链表头部
        int tail = Entry::dummy_vertex;     // 链表尾部

        // 从缓存中移除一个顶点
        // 更新双向链表的指针
        inline void remove(int vertex) {

            assert(vertex != Entry::dummy_vertex);
            assert(cache[vertex].used);

            const auto prevEntry = cache[vertex].prev;
            const auto nextEntry = cache[vertex].next;

            // Head remove.
            // 如果移除的是头部节点
            if (prevEntry == Entry::dummy_vertex) {
                head = nextEntry;
            } else {
                cache[prevEntry].next = nextEntry;
            }

            // Tail remove.
            // 如果移除的是尾部节点
            if (nextEntry == Entry::dummy_vertex) {
                tail = prevEntry;
            } else {
                cache[nextEntry].prev = prevEntry;
            }

            // 重置节点状态
            cache[vertex].used = false;
            cache[vertex].prev = Entry::dummy_vertex;
            cache[vertex].next = Entry::dummy_vertex;
        }

        // 将顶点移到链表头部
        // 表示该顶点是最近访问的
        inline void splay_on_top(int vertex) {

            assert(vertex != Entry::dummy_vertex);
            assert(!cache[vertex].used);

            cache[vertex].used = true;

            // 插入到头部
            cache[vertex].next = head;
            if (head != Entry::dummy_vertex) {
                cache[head].prev = vertex;
            }
            head = vertex;
            cache[vertex].prev = Entry::dummy_vertex;

            // 如果链表原本为空，更新尾部指针
            if (tail == Entry::dummy_vertex) {
                tail = head;
            }
        }
    };

}  // namespace cobra

#endif