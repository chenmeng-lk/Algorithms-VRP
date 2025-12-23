// 向量视图 (Vector View)
// 对随机访问范围应用函数，在线转换值而不修改底层数据
// 类似于C++20的ranges::transform_view
#ifndef _FILO2_VECTORVIEW_HPP_
#define _FILO2_VECTORVIEW_HPP_



#include <iterator>
namespace cobra {

    // Apply a function to a random access range online, without modifying the underlying values.
    // 向量视图类
    //
    // 功能：
    // - 对底层容器的每个元素应用一元操作
    // - 不修改原始数据，只在访问时转换
    // - 支持随机访问迭代器
    //
    // 模板参数：
    // @tparam IterT 底层迭代器类型（必须是随机访问迭代器）
    // @tparam OpTmpl 一元操作模板（用于转换元素）
    template <typename IterT, template <typename T> class OpTmpl>
    class VectorView {
        using UnaryOp = OpTmpl<typename std::iterator_traits<IterT>::value_type>;

        static_assert(std::is_same_v<typename std::iterator_traits<IterT>::iterator_category, std::random_access_iterator_tag>,
                      "The iterator type need to be a random_access_iterator or "
                      "a pointer.");

    public:
        // 自定义常量迭代器
        // 在解引用时应用一元操作
        class custom_const_iterator {
        public:
            custom_const_iterator(IterT _original) : original(_original){};

            // 解引用 - 应用一元操作
            inline auto operator*() {
                return UnaryOp()(*original);
            }

            // 前置递增
            inline auto operator++() {
                ++original;
                return *this;
            }

            // 成员访问
            inline auto operator->() {
                return original;
            }

            // 减法（迭代器 - 整数）
            inline auto operator-(int x) const {
                return custom_const_iterator(original - x);
            }

            // 减法（迭代器 - 迭代器）
            inline auto operator-(custom_const_iterator x) const {
                return original - x.original;
            }

            // 不等于比较
            inline auto operator!=(custom_const_iterator x) {
                return x.original != original;
            }

            // 等于比较
            inline auto operator==(custom_const_iterator x) {
                return x.original == original;
            }

        private:
            IterT original;  // 底层迭代器
        };

        // 构造函数
        // @param _first 范围起始迭代器
        // @param _last 范围结束迭代器
        VectorView(IterT _first, IterT _last) : first(_first), last(_last){};

        // 下标访问 - 应用一元操作
        inline auto operator[](size_t index) const {
            return UnaryOp()(first[index]);
        }

        // at访问 - 应用一元操作
        inline auto at(size_t index) const {
            return UnaryOp()(first[index]);
        }

        // 获取起始迭代器
        inline auto begin() const {
            return custom_const_iterator(first);
        }

        // 获取结束迭代器
        inline auto end() const {
            return custom_const_iterator(last);
        }

        // 获取范围大小
        inline auto size() const {
            return std::distance(first, last);
        }

    private:
        IterT first;  // 范围起始
        IterT last;   // 范围结束
    };
}  // namespace cobra

#endif