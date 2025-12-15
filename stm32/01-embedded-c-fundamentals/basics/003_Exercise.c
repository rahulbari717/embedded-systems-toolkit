/*
Print Multiplication Table of number entered by user

*/

#include <stdio.h>
int main(){
    int number;
    printf("Enter the number you want multiplication table of: ");
    scanf("%d", &number);
    printf("Multiplication table of %d is as follows: \n", number);
    for (int i = 1; i <= 10; i++){
        printf("%d X %d = %d \n", number, i, number * i);

    }
    return 0;

}