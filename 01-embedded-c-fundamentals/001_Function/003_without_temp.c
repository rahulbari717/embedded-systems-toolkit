#include <stdio.h>

void swap(int a, int b){
    /*
    a = a+b;
    b = a-b;
    a = a-b; 
    */
     a = a+b;
    b = a-b;
    a = a-b; 

    printf("After swap \n ");
    printf(" a = %d , b = %d  \n ", a,b);


}



int main(){

    int a = 10, b = 20; 
    printf("Swap by value \n ");
    printf("Before swap \n ");
    printf(" a = %d , b = %d  \n ", a,b);
    swap(a,b); 
    // printf("After swap \n ");
    // printf(" a = %d , b = %d  \n ", a,b);
    return 0; 
}