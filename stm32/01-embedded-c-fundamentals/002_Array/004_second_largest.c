/*

❓ Given an integer array, find the second largest value and the second maximum value in a single traversal, without using sorting and without using more than two variables for tracking.
 Also handle cases where all elements are equal or the array contains negative numbers.

Input: {-10, -3, -50, -1}
Output:

Max = -1  
*/

#include <stdio.h>

int max(int *arr, int size){
    int max = arr[0];
    int second_max = arr[0]; 
    for(int i =0; i<size; i++){
        if(max <= arr[i]){
            second_max = max;
            max = arr[i];
        } else if(second_max < arr[i] && max != arr[i]){
            second_max = arr[i];

        }

    }
   
    return second_max; 
}

int main(){

    int arr[] = {-10, -4, -200, -80, -19, -5, -12};

    int size = sizeof(arr)/sizeof(int); 

    printf("second max no. of this array is ==> %d \n", max(arr,size));

    return 0;
}