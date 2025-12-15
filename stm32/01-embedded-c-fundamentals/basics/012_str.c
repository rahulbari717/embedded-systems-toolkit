#include <stdio.h>

int main(){
    char str[52];
    printf("Enter string : \n"); 
    fgets(str, sizeof(str), stdin); 
    printf("Entered string is %s", str);

    puts(str);
    return 0;

}
