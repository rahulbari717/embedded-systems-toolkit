/*
 * Q) Why and how to pass array ?
 * ==> when we need to pass a list of values to a given funciotns. 
 *
 * print array functions 
 *
 * */


#include <stdio.h>


void print_array(int *arr)
{
    for(int i =0; i<10; i++){
        printf("The value of a %d is %d\n", i, arr[i]); 
    } 
}

int main()
{
    int arr1[] = {1,2,3,4,5,6,7,8,9,10}; 
    print_array(arr1);
    return 0; 
}
