/*
Q) Given array of marks of student if marks is less than 35 print its index
*/


#include <stdio.h>

void index(int *arr, int size){
    for(int i = 0; i<size; i++){
        if(arr[i] <= 35 ){
            printf("Index is ==>  %d \n", i); 
        }
    }
    return;
}

int main(){

    int marks[] = {95,91,25,100,50,65,89,27,30,100};
    int size = sizeof(marks)/sizeof(int); 
    // printf("%d", size); 
    index(marks, size); 

    return 0;
}