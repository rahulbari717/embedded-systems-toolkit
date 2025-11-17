#include <stdio.h>

void swap(int *x, int *y){

    int temp; 
    temp = *x;
    *x = *y;
    *y = temp;
}

int main(){

    int a = 10, b = 20; 
    printf("Swap by value \n ");
    printf("Before swap \n ");
    printf(" a = %d , b = %d  \n ", a,b);
    swap(&a,&b); // pass by ref : changes of values reflect here. 
    printf("After swap \n ");
    printf(" a = %d , b = %d  \n ", a,b);
    return 0; 
}