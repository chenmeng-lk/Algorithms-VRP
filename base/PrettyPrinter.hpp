// 美化输出类 - 用于格式化打印表格数据
// 支持自动对齐、颜色样式、周期性打印表头等功能
#ifndef _FILO2_PRETTYPRINTER_HPP_
#define _FILO2_PRETTYPRINTER_HPP_

#include <string>
#include <vector>

namespace cobra {

    // 美化输出类
    // 用于在控制台输出格式化的表格数据
    // 支持整数、浮点数、字符串等多种数据类型
    class PrettyPrinter {

    public:
        // 字段类 - 定义表格的一列
        class Field {
        public:
            // 字段类型枚举
            enum Type { INTEGER, REAL, STRING };

            // 构造函数
            // @param name_ 字段名称（表头）
            // @param type_ 字段类型（整数/浮点数/字符串）
            // @param max_width_ 最大宽度
            // @param sep_ 分隔符
            // @param precision_ 浮点数精度
            Field(std::string name_, Type type_ = Type::REAL, int max_width_ = 8, std::string sep_ = "\t", int precision_ = 2)
                : name(std::move(name_)), type(type_), max_width(max_width_), sep(std::move(sep_)), precision(precision_){};

            const std::string& get_name() const {
                return name;
            }
            const Type& get_type() const {
                return type;
            }
            int get_max_width() const {
                return max_width;
            }
            const std::string& get_separator() const {
                return sep;
            }
            int get_precision() const {
                return precision;
            }

        private:
            std::string name;       // 字段名称
            Type type;              // 字段类型
            int max_width;          // 最大宽度
            std::string sep;        // 分隔符
            int precision;          // 浮点数精度
        };

        // 构造函数
        // @param args_ 字段列表，定义表格的列
        explicit PrettyPrinter(std::vector<Field> args_) : args(std::move(args_)) { }

        // 打印一行数据
        // 使用可变参数模板，支持任意数量的参数
        // 每隔max_header_count行会重新打印表头
        template <typename T, typename... Values>
        void print(T t, Values... values) {  // recursive variadic function

            // 如果需要打印表头
            if (!header_count) {
                header_count = max_header_count;

                std::cout << "\n";

                // 使用粗体打印表头
                std::cout << "\033[1m";

                // 打印表头名称
                for (auto header : args) {
                    std::cout << " ";
                    std::cout << std::fixed;
                    std::cout << std::setw(header.get_max_width());
                    std::cout << header.get_name();
                    std::cout << std::setprecision(10);
                    std::cout << std::defaultfloat;
                    std::cout << " ";
                    std::cout << header.get_separator();
                }

                // 重置样式
                std::cout << "\033[0m";

                std::cout << "\n";

                // 打印分隔线
                for (auto header : args) {
                    for (auto n = 0; n < header.get_max_width() + 2; n++) {
                        std::cout << " ";
                    }
                    std::cout << header.get_separator();
                }
                std::cout << "\n";
            }
            header_count--;

            // 应用样式（如果设置了）
            if (style != NONE) {
                std::cout << "\033[" + std::to_string(style) + "m";
            }

            // 递归打印所有值
            print_impl(0, t, values...);

            // 重置样式
            if (style != NONE) {
                std::cout << "\033[0m";
            }

            std::cout << "\n";
        }

        // 打印通知消息
        // @param message 要打印的消息
        void notify(const std::string& message) {
            std::cout << "\n";

            if (style != NONE) {
                std::cout << "\033[" + std::to_string(style) + "m";
            }

            std::cout << message << "\n";

            if (style != NONE) {
                std::cout << "\033[0m";
            }

            std::cout << "\n";
        }

        // 样式枚举 - 用于设置输出的颜色
        // 使用ANSI转义码实现终端颜色
        enum Style {
            NONE = 0,                   // 无样式
            FOREGROUND_BLACK = 30,      // 前景色：黑色
            FOREGROUND_RED,             // 前景色：红色
            FOREGROUND_GREEN,           // 前景色：绿色
            FOREGROUND_YELLOW,          // 前景色：黄色
            FOREGROUND_BLUE,            // 前景色：蓝色
            FOREGROUND_MAGENTA,         // 前景色：洋红色
            FOREGROUND_CYAN,            // 前景色：青色
            FOREGROUND_WHITE,           // 前景色：白色
            BACKGROUND_BLACK = 40,      // 背景色：黑色
            BACKGROUND_RED,             // 背景色：红色
            BACKGROUND_GREEN,           // 背景色：绿色
            BACKGROUND_YELLOW,          // 背景色：黄色
            BACKGROUND_BLUE,            // 背景色：蓝色
            BACKGROUND_MAGENTA,         // 背景色：洋红色
            BACKGROUND_CYAN,            // 背景色：青色
            BACKGROUND_WHITE            // 背景色：白色
        };

        // 设置输出样式
        void set_style(Style style_) {
            this->style = style_;
        }

        // 取消输出样式
        void unset_style() {
            this->style = NONE;
        }

    private:
        // 打印单个值
        // 根据字段类型进行格式化
        template <typename T>
        void just_print(T value, Field& header) {
            std::cout << " ";
            std::cout << std::fixed;
            std::cout << std::setprecision(header.get_precision());
            std::cout << std::setw(header.get_max_width());
            switch (header.get_type()) {
            case Field::INTEGER:
                std::cout << static_cast<long>(value);
                break;
            case Field::REAL:
                std::cout << static_cast<double>(value);
                break;
            case Field::STRING:
                std::cout << value;
                break;
            }
            std::cout << " ";
            std::cout << std::setprecision(10);
            std::cout << std::defaultfloat;
        }

        // 打印实现 - 递归终止条件（只有一个值）
        template <typename T>
        void print_impl(int n, T value) {

            if (static_cast<unsigned>(n) == args.size()) {
                std::cout << "Values do not correspond to headers\n";
                return;
            }

            just_print(value, args[n]);
        }

        // 打印实现 - 递归打印多个值
        template <typename T, typename... Values>
        void print_impl(int n, T value, Values... values)  // recursive variadic function
        {

            if (static_cast<unsigned>(n) == args.size()) {
                std::cout << "Values do not correspond to headers\n";
                return;
            }

            just_print(value, args[n]);
            std::cout << args[n].get_separator();

            // 递归打印剩余的值
            print_impl(n + 1, values...);
        }

        std::vector<Field> args;        // 字段列表
        int max_header_count = 15;      // 每隔多少行重新打印表头
        int header_count = 0;           // 当前计数器
        Style style = NONE;             // 当前样式
    };

}  // namespace cobra

#endif
