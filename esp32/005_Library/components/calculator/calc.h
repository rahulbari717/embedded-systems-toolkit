#ifndef CALC_H
#define CALC_H

// Function pointer type definition
typedef int (*calc_func_t)(int, int);

// Function declarations
int add(int a, int b);
int sub(int a, int b);
int mul(int a, int b);
int divide(int a, int b);

#endif
