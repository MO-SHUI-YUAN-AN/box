#ifndef ARITHMETIC_PROBLEM_ARITHMETIC_UTILS_HPP
#define ARITHMETIC_PROBLEM_ARITHMETIC_UTILS_HPP

// 算式识别相关的纯函数工具
// 这些函数从原 arithmetic.cpp 中抽出，便于在不同模块（如 calculate 库）复用

#include <cctype>
#include <string>
#include <vector>

namespace arithmetic_problem {

// 模型类别索引（按 inference.h 中 classes 顺序）：
// 0-9: 数字 "0"-"9"，10: "+"，11: "-"，12: "×"，13: "÷"，14: "("，15: ")"
inline bool isOperator(int class_id) {
    // 运算符号 +, -, ×, ÷
    return class_id >= 10 && class_id <= 13;
}

inline char classIdToChar(int class_id) {
    if (class_id >= 0 && class_id <= 9) return static_cast<char>('0' + class_id);
    switch (class_id) {
        case 10: return '+';
        case 11: return '-';
        case 12: return '*';  // × 转为 *
        case 13: return '/';  // ÷ 转为 /
        case 14: return '(';
        case 15: return ')';
        default: return '?';
    }
}

// 按顺序修正括号：从左到右数到的第 1、3、5... 个括号一律改为 '('，
// 第 2、4、6... 个一律改为 ')'。如果总数为奇数（不配对），丢弃最后一个落单的。
inline std::string fixParentheses(const std::string& expr) {
    std::string result = expr;
    std::vector<size_t> parenPositions;
    for (size_t i = 0; i < result.size(); ++i) {
        if (result[i] == '(' || result[i] == ')') {
            parenPositions.push_back(i);
        }
    }
    for (size_t k = 0; k < parenPositions.size(); ++k) {
        result[parenPositions[k]] = (k % 2 == 0) ? '(' : ')';
    }
    if (parenPositions.size() % 2 == 1) {
        result.erase(parenPositions.back(), 1);
    }
    return result;
}

// 四则运算计算器（支持 + - * / 和括号，按运算优先级）
inline long long calcExpression(const std::string& expr) {
    std::vector<long long> nums;
    std::vector<char> ops;

    auto precedence = [](char op) {
        if (op == '(' || op == ')') return 0;
        if (op == '+' || op == '-') return 1;
        if (op == '*' || op == '/') return 2;
        return 0;
    };

    auto applyTop = [&]() {
        if (ops.empty()) return;
        char op = ops.back(); ops.pop_back();
        if (nums.size() < 2) return;
        long long b = nums.back(); nums.pop_back();
        long long a = nums.back(); nums.pop_back();
        long long r = 0;
        switch (op) {
            case '+': r = a + b; break;
            case '-': r = a - b; break;
            case '*': r = a * b; break;
            case '/': r = (b == 0) ? 0 : a / b; break;
            default: r = 0; break;
        }
        nums.push_back(r);
    };

    size_t i = 0;
    while (i < expr.size()) {
        char ch = expr[i];
        if (std::isdigit(static_cast<unsigned char>(ch))) {
            long long val = 0;
            while (i < expr.size() && std::isdigit(static_cast<unsigned char>(expr[i]))) {
                val = val * 10 + (expr[i] - '0');
                ++i;
            }
            nums.push_back(val);
        } else if (ch == '(') {
            ops.push_back(ch);
            ++i;
        } else if (ch == ')') {
            while (!ops.empty() && ops.back() != '(') {
                applyTop();
            }
            if (!ops.empty() && ops.back() == '(') ops.pop_back();
            ++i;
        } else if (ch == '+' || ch == '-' || ch == '*' || ch == '/') {
            while (!ops.empty() && ops.back() != '(' &&
                   precedence(ops.back()) >= precedence(ch)) {
                applyTop();
            }
            ops.push_back(ch);
            ++i;
        } else {
            ++i;
        }
    }
    while (!ops.empty()) applyTop();

    return nums.empty() ? 0 : nums.back();
}

// 把结果对 4 取模，得到 1-4 之间的正整数
inline int modTo1_4(long long result) {
    long long answer = result % 4;
    if (answer <= 0) answer += 4;
    return static_cast<int>(answer);
}

}  // namespace arithmetic_problem

#endif  // ARITHMETIC_PROBLEM_ARITHMETIC_UTILS_HPP
