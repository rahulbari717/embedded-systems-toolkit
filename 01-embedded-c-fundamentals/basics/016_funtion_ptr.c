/*
function pointers 
*/

#include <stdio.h>

int sum(int a, int b){
    return (a+b); 
}

int main(){
    int (*fptr) (int, int); 
    fptr = &sum; // creating a funciton pointer 
    int r = (*fptr) (4,5); // dereferancing funciton pointer.

    printf("Value of r is ==> %d \n", r); 



    return 0;
}