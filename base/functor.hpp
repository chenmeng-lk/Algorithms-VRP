// 函数对象 (Functors)
// 提供各种通用的函数对象，用于模板编程
#ifndef _FILO2_FUNCTOR_HPP_
#define _FILO2_FUNCTOR_HPP_

#include <type_traits>

namespace cobra {

    // 恒等函数对象 - 返回输入值本身
    template <typename T>
    struct identity_functor {
        auto operator()(T x) {
            return x;
        }
    };

    // 孪生函数对象 - 返回x的孪生值
    // 对于整数x，返回x^1（异或1）
    // 例如：0->1, 1->0, 2->3, 3->2
    // 用于在成对的索引之间切换
    template <typename T>
    struct twin_functor {
        static_assert(std::is_integral_v<T>);
        auto operator()(T x) {
            return x ^ 1;
        }
    };

    // 基础函数对象 - 返回x的基础值
    // 对于整数x，返回x&(~1)（清除最低位）
    // 例如：0->0, 1->0, 2->2, 3->2
    // 用于将成对的索引映射到同一个基础值
    template <typename T>
    struct base_functor {
        static_assert(std::is_integral_v<T>);
        auto operator()(T x) {
            return x & (~1);
        }
    };

    // 字段访问函数对象基类 - 访问对象的指定成员
    // 使用成员指针访问对象的字段
    template <typename T, typename fieldType, fieldType T::*field>
    struct base_access_field_functor {
        auto& operator()(T& t) {
            return t.*field;
        }
    };

    // 字段访问函数对象 - 简化版本，自动推导字段类型
    template <typename T, auto field>
    struct access_field_functor : base_access_field_functor<T, typename std::decay<decltype(std::declval<T>().*field)>::type, field> { };

}  // namespace cobra

#endif