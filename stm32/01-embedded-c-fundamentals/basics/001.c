/*
Dynamic memory allocations
*/

#include <stdio.h>
#include <stdlib.h>

void malloc()
{
    //
    int *ptr = (int *)malloc(10 * sizeof(int));
}

void calloc()
{
    //
    int n;
    printf("Enter the size of array : ");
    scanf("%d", &n);
    int *ptr = (int *)calloc(n, *sizeof(int));

    free(ptr);
}

int main()
{

    return 0;
}