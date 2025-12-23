#ifndef _FILO2_MACRO_HPP_
#define _FILO2_MACRO_HPP_
/*
这是一个编译器分支预测优化宏，主要用于提示编译器哪个
分支更可能被执行，从而优化指令流水线，减少分支预测失败的惩罚
 */
#define likely(condition) __builtin_expect(static_cast<bool>(condition), 1)//告诉编译器这个条件很可能为真（概率高）
#define unlikely(condition) __builtin_expect(static_cast<bool>(condition), 0)

#endif