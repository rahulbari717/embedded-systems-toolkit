/*
 * Average of Elements
    Write a function that takes an array of floats and its size, and returns the average value of the elements.
 *
 * */

#include <stdio.h>

void average (int *arr, int size){
    static int sum = 0;
    
    for(int i =0; i<size; i++){
        sum += arr[i];
        printf("Sum of element no. %d is %d \n", i, sum); 
    }
    printf ("Final total sum is %d \n", sum);
    printf("so now we have to "); 
    printf(" avg is = %f", (float)(sum /size)); 
}

int main(){

int arr[] = {1,2,3,4,5,6,7,8,9,10};
int size = sizeof(arr) / sizeof(arr[0]);  

average(arr, size); 

return 0; 

}
