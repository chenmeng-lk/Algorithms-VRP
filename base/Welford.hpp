// Welford在线算法 - 用于计算均值和方差
// 这是一种数值稳定的在线算法，可以在数据流中增量计算统计量
// 参考: https://gist.github.com/alexalemi/2151722
#ifndef _FILO2_WELFORD_HPP_
#define _FILO2_WELFORD_HPP_

namespace cobra {

    // https://gist.github.com/alexalemi/2151722
    // Welford在线算法类
    // 用于增量计算均值，避免数值不稳定问题
    //
    // 算法原理：
    // mean(n) = mean(n-1) + (x(n) - mean(n-1)) / n
    //
    // 优点：
    // 1. 只需要一次遍历数据
    // 2. 数值稳定，不会因为大数相减导致精度损失
    // 3. 内存占用小，只需要存储当前均值和计数
    class Welford {

    public:
        Welford() = default;

        // 拷贝构造函数
        Welford(const Welford& other) {
            k = other.k;
            mean = other.mean;
        }

        Welford& operator=(const Welford& other) = default;

        // 更新统计量 - 添加一个新的数据点
        // @param x 新的数据点
        // 使用Welford算法增量更新均值
        void update(double x) {
            ++k;
            const auto new_mean = mean + (x - mean) * 1.0 / static_cast<double>(k);
            mean = new_mean;
        }

        // 获取当前均值
        double get_mean() const {
            return mean;
        }

        // 重置统计量 - 清空所有数据
        void reset() {
            k = 0;
            mean = 0.0;
        }

    private:
        unsigned long k = 0;    // 数据点数量
        double mean = 0.0;      // 当前均值
    };

}  // namespace cobra

#endif