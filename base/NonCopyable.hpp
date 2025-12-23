// 不可拷贝混入类 (Non-copyable Mixin)
// 用于禁止类的拷贝和赋值操作，防止意外的对象复制
// 参考: https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Non-copyable_Mixin
#ifndef _FILO2_NONCOPYABLE_HPP_
#define _FILO2_NONCOPYABLE_HPP_

namespace cobra {

    // Non-copyable mixin inspired by https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Non-copyable_Mixin.
    // 不可拷贝混入类
    //
    // 使用方法：
    // class MyClass : private NonCopyable<MyClass> { ... };
    //
    // 作用：
    // - 禁止拷贝构造和拷贝赋值
    // - 允许移动构造（但禁止移动赋值）
    // - 防止大对象的意外复制，提高性能
    //
    // 在FILO2中的应用：
    // - Instance, Solution, MoveGenerators等大对象都继承此类
    // - 避免昂贵的深拷贝操作
    template <class T>
    class NonCopyable {
    public:
        NonCopyable(const NonCopyable&) = delete;              // 禁止拷贝构造
        NonCopyable(NonCopyable&&) noexcept = default;         // 允许移动构造
        NonCopyable& operator=(const NonCopyable&) = delete;   // 禁止拷贝赋值
        NonCopyable& operator=(NonCopyable&&) noexcept = delete;  // 禁止移动赋值
        T& operator=(const T&) = delete;                       // 禁止派生类的拷贝赋值
        T& operator=(T&&) noexcept = delete;                   // 禁止派生类的移动赋值

    protected:
        NonCopyable() = default;
        ~NonCopyable() = default;  /// Protected non-virtual destructor
                                   /// 受保护的非虚析构函数（防止通过基类指针删除）
    };

}  // namespace cobra

#endif