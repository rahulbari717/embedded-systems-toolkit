#include <stdio.h>

int main(){
    int size; 
    printf("Enter no. of student first : " );
    scanf("%d",&size);
    int marks[size]; 
    
    for(int i =0; i<size; i++){
        printf("Enter the value of %d element of the array \n", i);  
        scanf("%d", &marks[i]); 
    }

    for(int i=0; i<size; i++){
        printf("The value of %d element of the array is %d \n", i, marks[i]); 
    }
    
    return 0; 
}
