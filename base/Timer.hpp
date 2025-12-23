// 计时器类 - 用于测量代码执行时间
// 使用高精度时钟，支持多种时间单位
#ifndef _FILO2_TIMER_HPP_
#define _FILO2_TIMER_HPP_

#include <chrono>

namespace cobra {
    // 计时器类
    // 基于C++11的chrono库实现高精度计时
    class Timer {
    public:
        // 构造函数 - 自动开始计时
        Timer() {
            reset();
        }

        // 获取从上次reset()以来经过的时间
        // @tparam Units 时间单位（默认为高精度时钟的原生单位）
        //               可以是std::chrono::seconds, std::chrono::milliseconds等
        // @return 经过的时间（以指定单位表示）
        template <typename Units = typename std::chrono::high_resolution_clock::duration>
        unsigned long elapsed_time() const {
            auto counted_time = std::chrono::duration_cast<Units>(std::chrono::high_resolution_clock::now() - start_point).count();
            return static_cast<unsigned long>(counted_time);
        }

        // 重置计时器 - 将起始时间点设置为当前时间
        void reset() {
            start_point = std::chrono::high_resolution_clock::now();
        }

    private:
        std::chrono::high_resolution_clock::time_point start_point;  // 起始时间点
    };
}  // namespace cobra

#endif