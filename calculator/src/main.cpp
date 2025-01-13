#include <iostream>
#include <sstream>
#include "calculator.h"

int main() {
    double num1, num2;
    char op;

    std::cout << "Welcome to the calculator!" << std::endl;
    std::cout << "Enter an operation in the format: number1 operator number2" << std::endl;
    std::cout << "For example: 2 + 3" << std::endl;

    Calculator calculator;
    while (true) {
        std::cout << "Enter your calculation ( or 'q' to quit): ";
        std::string input;
        std::getline(std::cin, input);

        if (input == "q" || input == "Q") {
            break;
        }

        std::istringstream iss(input);
        if (!(iss >> num1)) {
            std::cerr << "Error: Invalid input!" << std::endl;
            continue;
        }

        iss >> op;
        if (!(iss >> num2)) {
            std::cerr << "Error: Invalid input!" << std::endl;
            continue;
        }

        // Check if there are any extra characters in the input
        char extra;
        if (iss >> extra) {
            std::cerr << "Error: Invalid input!" << std::endl;
            continue;
        }

        try {
            double result;
            switch (op) {
                case '+':
                    result = calculator.add(num1, num2);
                    break;
                case '-':
                    result = calculator.subtract(num1, num2);
                    break;
                case '*':
                    result = calculator.multiply(num1, num2);
                    break;
                case '/':
                    result = calculator.divide(num1, num2);
                    break;
                default:
                    std::cerr << "Error: Invalid operator!" << std::endl;
                    continue;
            }
            std::cout << "Result: " << result << std::endl;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
    std::cout << "Goodbye!" << std::endl;
    return 0;
}