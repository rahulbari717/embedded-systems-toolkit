/*

❓ Given an integer array, find the maximum value and the second maximum value in a single traversal, without using sorting and without using more than two variables for tracking.
 Also handle cases where all elements are equal or the array contains negative numbers.

Input: {-10, -3, -50, -1}
Output:

Max = -1  
*/

#include <stdio.h>

int max(int *arr, int size){
    int max = arr[0];
    for(int i =0; i<size; i++){
        if(max <= arr[i]){
            max = arr[i];
        }
    }
    return max; 
}

int main(){

    int arr[] = {-10, -3, -50, -1};

    int size = sizeof(arr)/sizeof(int); 

    printf("Maximu no. of this array is ==> %d \n", max(arr,size));

    return 0;
}