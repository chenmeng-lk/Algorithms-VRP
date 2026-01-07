#ifndef PYVRP_MEASURE_H
#define PYVRP_MEASURE_H

#include <cassert>
#include <cmath>
#include <compare>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <type_traits>

namespace pyvrp
{
// 测量类型枚举，定义不同的度量单位类型
enum class MeasureType
{
    COORD,     // 坐标类型
    DIST,      // 距离类型
    DURATION,  // 持续时间类型
    COST,      // 成本类型
    LOAD,      // 负载类型
};

// 概念定义：检查是否为数值类型
template <typename T>
concept NumberType = std::is_arithmetic_v<T>;

// 前向声明，以便早期定义相关的类型别名
template <MeasureType Type, NumberType Value> class Measure;

// 类型别名。这些在整个程序中使用
using Coordinate = Measure<MeasureType::COORD, double>;   // 坐标类型，使用double存储
using Cost = Measure<MeasureType::COST, int64_t>;         // 成本类型，使用int64_t存储
using Distance = Measure<MeasureType::DIST, int64_t>;     // 距离类型，使用int64_t存储
using Duration = Measure<MeasureType::DURATION, int64_t>; // 持续时间类型，使用int64_t存储
using Load = Measure<MeasureType::LOAD, int64_t>;         // 负载类型，使用int64_t存储

//
//                 EVERYTHING BELOW IS AN IMPLEMENTATION DETAIL
//

/**
 * The measure class is a thin wrapper around an underlying value. The measure
 * forms a strong type that is only explicitly castable to other arithmetic or
 * measure types.
 *
 * The measure is equipped with a ``MeasureType`` that specifies what it is
 * intended to model, as well as a ``NumberType`` storage class for the
 * underlying data. The latter can be used to e.g. differentiate integral and
 * floating point measures.
 */
// Measure类是一个围绕底层值的薄包装器，形成一个强类型，只能显式转换为其他算术类型或测量类型
template <MeasureType _, NumberType Value> class Measure
{
    Value value = 0;  // 底层存储的值

public:
    // 默认构造函数初始化为0
    Measure() = default;

    // 构造函数：接受任何算术类型并转换为底层Value类型
    template <NumberType T>
    Measure(T const value) : value(static_cast<Value>(value))
    {
    }

    // 显式转换：将底层值转换为其他算术类型
    template <NumberType T> explicit operator T() const
    {
        return static_cast<T>(value);
    }

    // 显式转换：转换为相同存储类型的其他测量类型（我们不希望因测量类型之间的转换而导致数据丢失）
    template <MeasureType Other> explicit operator Measure<Other, Value>() const
    {
        return value;
    }

    // 获取底层值
    [[nodiscard]] Value get() const;

    // 就地一元运算符
    Measure &operator+=(Measure const rhs);
    Measure &operator-=(Measure const rhs);
    Measure &operator*=(Measure const rhs);
    Measure &operator/=(Measure const rhs);

    // 比较运算符
    [[nodiscard]] bool operator==(Measure const other) const;
    [[nodiscard]] std::strong_ordering operator<=>(Measure const other) const;
};

// 获取底层值
template <MeasureType Type, NumberType Value>
Value Measure<Type, Value>::get() const
{
    return value;
}

// 就地加法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator+=(Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)  // 如果是整数类型，检查溢出
    {
        [[maybe_unused]] Value res = 0;
        assert(!__builtin_add_overflow(this->value, rhs.value, &res));
    }

    this->value += rhs.value;
    return *this;
}

// 就地减法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator-=(Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)  // 如果是整数类型，检查溢出
    {
        [[maybe_unused]] Value res = 0;
        assert(!__builtin_sub_overflow(this->value, rhs.value, &res));
    }

    this->value -= rhs.value;
    return *this;
}

// 就地乘法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator*=(Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)  // 如果是整数类型，检查溢出
    {
        [[maybe_unused]] Value res = 0;
        assert(!__builtin_mul_overflow(this->value, rhs.value, &res));
    }

    this->value *= rhs.value;
    return *this;
}

// 就地除法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator/=(Measure<Type, Value> const rhs)
{
    this->value /= rhs.value;
    return *this;
}

// 相等比较运算符
template <MeasureType Type, NumberType Value>
bool Measure<Type, Value>::operator==(Measure<Type, Value> const other) const
{
    return value == other.value;
}

// 三路比较运算符（C++20太空船运算符）
template <MeasureType Type, NumberType Value>
std::strong_ordering
Measure<Type, Value>::operator<=>(Measure<Type, Value> const other) const
{
    return value <=> other.value;
}

// 自由函数：加法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator+(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)  // 如果是整数类型，检查溢出
    {
        [[maybe_unused]] Value res = 0;
        assert(!__builtin_add_overflow(lhs.get(), rhs.get(), &res));
    }

    return lhs.get() + rhs.get();
}

// 自由函数：一元正号运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator+(Measure<Type, Value> const lhs)
{
    return +lhs.get();
}

// 自由函数：减法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator-(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)  // 如果是整数类型，检查溢出
    {
        [[maybe_unused]] Value res = 0;
        assert(!__builtin_sub_overflow(lhs.get(), rhs.get(), &res));
    }

    return lhs.get() - rhs.get();
}

// 自由函数：一元负号运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator-(Measure<Type, Value> const lhs)
{
    return -lhs.get();
}

// 自由函数：乘法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator*(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)  // 如果是整数类型，检查溢出
    {
        [[maybe_unused]] Value res = 0;
        assert(!__builtin_mul_overflow(lhs.get(), rhs.get(), &res));
    }

    return lhs.get() * rhs.get();
}

// 自由函数：除法运算符
template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator/(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    return lhs.get() / rhs.get();
}
}  // namespace pyvrp

// 用于打印的重载输出流运算符
template <pyvrp::MeasureType Type, pyvrp::NumberType Value>
std::ostream &operator<<(std::ostream &out,
                         pyvrp::Measure<Type, Value> const measure)
{
    return out << measure.get();
}

// 为哈希和数值限制提供的特化

// 哈希特化，使得Measure类型可用于哈希容器
template <pyvrp::MeasureType Type, pyvrp::NumberType Value>
struct std::hash<pyvrp::Measure<Type, Value>>
{
    size_t operator()(pyvrp::Measure<Type, Value> const measure) const
    {
        return std::hash<Value>()(measure.get());
    }
};

// 数值限制特化，使得Measure类型可以使用std::numeric_limits
template <pyvrp::MeasureType Type, pyvrp::NumberType Value>
class std::numeric_limits<pyvrp::Measure<Type, Value>>
{
public:
    static pyvrp::Measure<Type, Value> max()
    {
        return std::numeric_limits<Value>::max();
    }

    static pyvrp::Measure<Type, Value> min()
    {
        return std::numeric_limits<Value>::min();
    }
};

#endif  // PYVRP_MEASURE_H
