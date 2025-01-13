#include "calculator.h"
#include <stdexcept>

double Calculator::add(double a, double b) {
    // Implementation needed to match the header declaration
    return a + b;
}

double Calculator::subtract(double a, double b) {
    return a - b;
}

double Calculator::multiply(double a, double b) {
    return a * b;
}

double Calculator::divide(double a, double b) {
    if (b == 0) {
        throw std::invalid_argument("Division by zero is undefined.");
    }
    return a / b;
}


