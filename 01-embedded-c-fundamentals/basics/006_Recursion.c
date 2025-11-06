// Recursion of code of simple factorial calculations


#include <stdio.h>

int fact(int num)
{
    // base case (stop condition)
    if(num == 0 || num == 1) return 1;
    else {
        // logic
        return (num * fact(num - 1)); 
    }
}

int main()
{
    int num; 
    printf("Enter the number you want the factorial of \n");
    scanf("%d", &num);

    printf("The factorial of %d is %d \n", num, fact(num)); 
    return 0; 
}
