// This project uses the CMake build system.
#ifndef CALCULATOR_H

#define CALCULATOR_H



#include <stdexcept>



class Calculator {


public:

    double add(double a, double b);

    double subtract(double a, double b);

    double multiply(double a, double b);

    double divide(double a, double b);

};


#endif // CALCULATOR_H