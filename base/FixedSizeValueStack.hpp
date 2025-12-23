#ifndef _FILO2_FIXEDSIZEVALUESTACK_HPP_
#define _FILO2_FIXEDSIZEVALUESTACK_HPP_

#include <cassert>
#include <functional>
//固定大小值栈，bigin指向最后进入的元素,capacity指向栈底
namespace cobra {

    template <class T>
    class FixedSizeValueStack {
    public:
        explicit FixedSizeValueStack(int dimension, std::function<T(int index)> array_initializer) {
            assert(dimension > 0);
            array.resize(dimension);
            capacity = dimension;
            begin = 0;
            initializer = array_initializer;
            reset();
        }

        ~FixedSizeValueStack() = default;

        FixedSizeValueStack<T>& operator=(const FixedSizeValueStack<T>& other) {
            assert(capacity == other.capacity);

            for (int i = 0; i < capacity; i++) {
                array[i] = other.array[i];
            }
            begin = other.begin;
            initializer = other.initializer;
            return *this;
        }

        T get() {//在第一次调用 push 之前，必须先调用至少一次 get 来腾出空间
            assert(begin < capacity);
            auto item = array[begin];
            begin++;
            return item;
        }

        void push(T item) {
            begin--;
            assert(begin >= 0);
            array[begin] = item;
        }

        void reset() {//初始满栈，begin指向栈顶
            for (int i = 0; i < capacity; i++) {
                array[i] = initializer(i);
            }
            begin = 0;
        }

        int size() const {
            return capacity - begin;
        }

        bool is_empty() {
            return begin == capacity;
        }

    private:
        std::vector<T> array;
        int begin = 0;
        int capacity;
        std::function<T(int)> initializer = nullptr;
    };

}  // namespace cobra

#endif