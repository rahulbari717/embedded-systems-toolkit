/*
 * call by value and call by referace 
 * call by value means : we are passing the values of arguments which are copied into the formal parameters of the functions.
 * which means original values remains unchanged 
 *
 * call by referance 
 * means we are passing address of argument into formal parameters
 *
 * */

#include <stdio.h>

/* function definition to swap the values */
void swap(int *x, int *y){
    int temp; 
    temp = *x; 
    *x = *y; 
    *y = temp; 
    return; 
}

int main(){
    int a = 33, b = 37; 
    printf("Value of a %d and b %d is \n", a,b);
    swap(&a, &b);
    printf("Value of a %d and b %d is \n", a,b);
    return 0;
}
