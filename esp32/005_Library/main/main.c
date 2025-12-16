#include <stdio.h>
#include "calc.h"

void app_main(void)
{
    int a = 10;
    int b = 20;
    int result = 0;

    // Function pointer
    calc_func_t operation;

    // Addition
    operation = add;
    result = operation(a, b);
    printf("Add: %d\n", result);

    // Subtraction
    operation = sub;
    result = operation(a, b);
    printf("Sub: %d\n", result);

    // Multiplication
    operation = mul;
    result = operation(a, b);
    printf("Mul: %d\n", result);

    // Division
    operation = divide;
    result = operation(a, b);
    printf("Div: %d\n", result);
}
