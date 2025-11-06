/*
 * What is pointer ? ==> variable which stores address of another variable 
 * size depend on architectures means  
 *      32-bit systems use 4-byte (32-bit) pointers.
 *      64-bit systems use 8-byte (64-bit) pointers.
 *
 *  Null Pointer :
 *        Null pointer is pointer that does not points to any object or function
 *        we can used it to initialize a pointer variable that isn't assinged any valied memory address yet 
 *
 *        int *ptr = NULL; 
 
 * */

#include <stdio.h>

int main(){
    
    int a = 10; 
        
    printf("Value of a is ==> %d \n", a); 

    int *ptr = &a; 

    printf("Value of a is ==> %d \n", *ptr); 

    printf("address of a is %p \n", ptr); 
    printf("address of a is %p \n", &a); 
        
    return 0; 
}


