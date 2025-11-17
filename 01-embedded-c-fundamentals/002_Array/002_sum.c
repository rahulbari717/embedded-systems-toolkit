/*
Q) Given array of printf sum of array
*/


#include <stdio.h>

void sum(int *arr, int size){
    int rahu = 0; 
    for(int i = 0; i<size; i++){
        rahu = rahu +arr[i];
    }
    printf("Sum of array is ==> %d \n", rahu); 
    return;
}

int main(){

    int marks[] = {30,100};
    int size = sizeof(marks)/sizeof(int); 
    // printf("%d", size); 
    sum(marks, size); 

    return 0;
}