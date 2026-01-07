#include "Matrix.h"
#include "Measure.h"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <type_traits>

// Used to separate template parameters that otherwise do not work well with
// macros. 用于分隔模板参数，避免宏展开时出现问题。
#define PYVRP_PARAM_SEP ,

namespace pybind11::detail
{
// Type caster for matrices of integral measures.
// 用于整型度量值矩阵的类型转换器。主要作用：在C++的Matrix<Measure<T, V>>和Python的numpy整数数组之间进行双向转换。
template <pyvrp::MeasureType T, std::integral V>
struct type_caster<pyvrp::Matrix<pyvrp::Measure<T, V>>>
{
    PYBIND11_TYPE_CASTER(pyvrp::Matrix<pyvrp::Measure<T PYVRP_PARAM_SEP V>>,
                         _("numpy.ndarray[int]"));

    bool load(pybind11::handle src, bool convert)  // Python -> C++，主要作用：将Python的numpy数组转换为C++的Matrix<Measure<T, V>>
    {
        if (!convert && !pybind11::array_t<V>::check_(src))  // 如果不允许转换且输入不是正确的numpy数组类型，则返回false
            return false;

        auto const style
            = pybind11::array::c_style | pybind11::array::forcecast;  // 指定数组为C风格并强制类型转换
        auto const buf = pybind11::array_t<V, style>::ensure(src);  // 确保输入是一个numpy数组

        if (!buf || buf.ndim() != 2)  // 检查数组是否存在且为二维
            throw pybind11::value_error("Expected 2D np.ndarray argument!");

        if (buf.size() == 0)  // then the default constructed object is already 如果数组为空，则默认构造的对象已经是OK的
            return true;      // OK, and we have nothing to do. 无需进一步处理

        std::vector<pyvrp::Measure<T, V>> data
            = {buf.data(), buf.data() + buf.size()};  // 将numpy数组的数据复制到vector中

        value = pyvrp::Matrix<pyvrp::Measure<T, V>>(
            std::move(data), buf.shape(0), buf.shape(1));  // 使用数据、行数和列数构造Matrix对象

        return true;  // 转换成功
    }

    static pybind11::handle
    cast(pyvrp::Matrix<pyvrp::Measure<T, V>> const &src,  // C++ -> Python，主要作用：将C++的Matrix<Measure<T, V>>转换为Python的numpy数组
         [[maybe_unused]] pybind11::return_value_policy policy,
         pybind11::handle parent)
    {
        auto constexpr elemSize = sizeof(V);  // 计算元素类型V的大小

        pybind11::array_t<V> array
            = {{src.numRows(), src.numCols()},           // shape 数组形状
               {elemSize * src.numCols(), elemSize},     // strides 数组步幅
               reinterpret_cast<V const *>(src.data()),  // data 数据指针，注意重新解释为V类型
               parent};                                  // base 父对象

        // This is not pretty, but it makes the matrix non-writeable on the
        // Python side. That's needed because src is const, and we should
        // preserve that to avoid issues.
        // 这不够优雅，但它使得矩阵在Python端不可写。这是必需的，因为src是const的，我们应该保持这一点以避免问题。
        pybind11::detail::array_proxy(array.ptr())->flags
            &= ~pybind11::detail::npy_api::NPY_ARRAY_WRITEABLE_;  // 清除可写标志位

        return array.release();  // 释放所有权并返回Python对象
    }
};

// Caster for integral-valued measures.
// 用于整型度量值的类型转换器。主要作用：在C++的Measure<T, V>和Python的int之间进行双向转换。
template <pyvrp::MeasureType T, std::integral V>
struct type_caster<pyvrp::Measure<T, V>>
{
    PYBIND11_TYPE_CASTER(pyvrp::Measure<T PYVRP_PARAM_SEP V>, _("int"));

    bool load(pybind11::handle src, bool convert)  // Python -> C++，主要作用：将Python的int对象转换为C++的Measure<T, V>
    {
        static_assert(sizeof(long long) >= sizeof(V));  // 编译时检查：确保long long足够大以容纳V类型

        if (!convert && !PyLong_Check(src.ptr()))  // accept only int if no 如果不允许转换且输入不是Python整数
            return false;                          // conversion is allowed. 则返回false

        PyObject *tmp = PyNumber_Long(src.ptr());  // any argument for which 尝试将输入转换为Python长整型
        if (!tmp)                                  // int() succeeds. 如果转换失败
            return false;

        auto const raw = PyLong_AsLongLong(tmp);  // 将Python长整型转换为C++ long long
        Py_DECREF(tmp);  // 减少临时对象的引用计数

        // See https://docs.python.org/3/c-api/long.html#c.PyLong_AsLongLong:
        // -1 is returned on overflow, and OverflowError is set.
        // 参见Python文档：溢出时返回-1并设置OverflowError。
        if (raw == -1 && PyErr_Occurred())  // 检查是否发生溢出错误
            throw pybind11::error_already_set();  // 抛出已设置的Python错误

        value = pyvrp::Measure<T, V>(raw);  // 使用原始值构造Measure对象
        return !PyErr_Occurred();  // 返回是否成功（没有Python错误发生）
    }

    static pybind11::handle
    cast(pyvrp::Measure<T, V> const &src,  // C++ -> Python，主要作用：将C++的Measure<T, V>转换为Python的int对象
         [[maybe_unused]] pybind11::return_value_policy policy,
         [[maybe_unused]] pybind11::handle parent)
    {
        return PyLong_FromLongLong(src.get());  // 将Measure中的值转换为Python长整型
    }
};

// Caster for floating point measures.
// 用于浮点型度量值的类型转换器。主要作用：在C++的Measure<T, V>和Python的float之间进行双向转换。
template <pyvrp::MeasureType T, std::floating_point V>
struct type_caster<pyvrp::Measure<T, V>>
{
    PYBIND11_TYPE_CASTER(pyvrp::Measure<T PYVRP_PARAM_SEP V>, _("float"));

    bool load(pybind11::handle src, bool convert)  // Python -> C++，主要作用：将Python的float对象转换为C++的Measure<T, V>
    {
        if (!convert && !PyFloat_Check(src.ptr()))  // accept only float if no 如果不允许转换且输入不是Python浮点数
            return false;                           // conversion is allowed. 则返回false

        PyObject *tmp = PyNumber_Float(src.ptr());  // any argument for which 尝试将输入转换为Python浮点数
        if (!tmp)                                   // float() succeeds. 如果转换失败
            return false;

        auto const raw = PyFloat_AsDouble(tmp);  // 将Python浮点数转换为C++ double
        Py_DECREF(tmp);  // 减少临时对象的引用计数

        // See https://docs.python.org/3/c-api/float.html#c.PyFloat_AsDouble:
        // on error, returns -1.0 and sets PyErr_Occurred.
        // 参见Python文档：错误时返回-1.0并设置PyErr_Occurred。
        if (raw == -1.0 && PyErr_Occurred())  // 检查是否发生错误
            throw pybind11::error_already_set();  // 抛出已设置的Python错误

        value = pyvrp::Measure<T, V>(raw);  // 使用原始值构造Measure对象
        return !PyErr_Occurred();  // 返回是否成功（没有Python错误发生）
    }

    static pybind11::handle
    cast(pyvrp::Measure<T, V> const &src,  // C++ -> Python，主要作用：将C++的Measure<T, V>转换为Python的float对象
         [[maybe_unused]] pybind11::return_value_policy policy,
         [[maybe_unused]] pybind11::handle parent)
    {
        return PyFloat_FromDouble(src.get());  // 将Measure中的值转换为Python浮点数
    }
};
}  // namespace pybind11::detail
