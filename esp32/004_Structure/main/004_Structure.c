#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    char firstName[50];
    char lastName[50];
    uint8_t age; 
} Person; 

void updatePerson(Person * robin){          // catch it using pointer only and used then arrow operator 
    strcpy(robin->firstName, "Ash");
    strcpy(robin->lastName, "Ketchum");
    robin->age = 25;
}

void app_main(void)
{
    Person robin;   // create a variable of this structure then memory will allocate 
    updatePerson(&robin);   // pass address of this struct so no additional copy will create     

    printf("Person : %s %s is %d years old \n",robin.firstName, robin.lastName, robin.age); 
}
