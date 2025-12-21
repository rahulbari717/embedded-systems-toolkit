
#include <stdint.h>
#include <stdio.h>

int main(void)
{
	int count = 0;
	for(int i=0; i<10; i++){
		printf("Count = %d \n ", count++);
	}
	printf("Hello world \n ");
    /* Loop forever */
	for(;;);
}
