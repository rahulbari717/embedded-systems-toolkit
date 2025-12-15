/*
Given an integer array, reverse the array in-place (without using another array). Do it using a single loop by swapping elements from both ends.
Example Input:
[10, 20, 30, 40, 50]

Expected Output:
[50, 40, 30, 20, 10]

Conditions:

Do not create another array

Must use swapping

Use single loop only

It should work with negative numbers too

*/

#include <stdio.h>

void print_arr(int *arr, int size){
    for(int i = 0; i<size; i++ ){
        printf(" %d", arr[i]); 
    }
    return;
}


void reverse_arr(int *arr, int size){
    // 2 pointer approched
    
    int end = size-1;
    int temp;  
    for(int start = 0; start< end; start++){
        temp = arr[start]; 
        arr[start] = arr[end];
        arr[end] = temp; 
    }

    return; 
}

int main(){
    int arr[] = {10, 20, 30, 40, 50};
    int size = sizeof(arr)/sizeof(int);
    printf("before swap array \n");
    print_arr(arr,size);
    reverse_arr(arr,size);  
    printf("After swap array \n");
    printf("\n");
    print_arr(arr,size);   
    return 0;
}