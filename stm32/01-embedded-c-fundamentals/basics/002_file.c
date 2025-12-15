/*
file 
*/

#include <stdio.h>
int main(){
    FILE *fptr = NULL;
    char str [500]; 
    char str1 [500] = "this content was produce by me rahul";  
    
    // Reading file   
    // ptr = fopen("myfile.txt", "r");
    // fscanf(ptr, "%s", str);
    // printf("File content ==>  %s \n", str);

    // writng a file 
    fptr = fopen("myfile.txt", "w");
    fprintf(fptr, "%s", str1);
    // printf("File content ==>  %s \n", str1);
  
    fclose(fptr); 


    return 0;
}