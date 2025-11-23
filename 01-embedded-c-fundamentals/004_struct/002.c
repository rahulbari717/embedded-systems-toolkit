/*

📌 Problem Statement

Create a program to store and display details of employees in a company.
Every employee has:

1. Basic Info – (Use struct)

char name[30]

int id

float salary

2. Address Info – (Use nested struct)

char city[20]

int pincode

This must be a nested struct inside Employee.

3. Employee Type – (Use enum)

Define an enum:

enum EmpType { INTERN, FULL_TIME, CONTRACT };


Store the employee type inside the Employee structure.

4. Union for Contact Info – (Use union)

Use a union Contact that can store either:

char phone[15]

char email[30]

Because the employee may provide only one of these.

5. typedef for cleaner definitions

Use typedef to alias:

the employee structure (Employee)

the union (Contact)

the enum (EmpType)

6. Use a Structure Pointer

In main(), create an array of 3 employees and fill it using a pointer to structure (Employee *ptr).

Use ptr->member syntax.

🎯 Your Task

Write a C program that:

Defines all required enum, union, struct, and typedefs

Takes input for 3 employees (name, id, salary, city, pincode, employee type, contact info)

Uses pointer-to-struct to store values

Prints all employee details in a clean format

Prints which contact type was provided (phone or email)

📌 Expected Example Output Format
Employee 1:
Name: Rahul Bari
ID: 101
Type: FULL_TIME
Salary: 55000
City: Mumbai
Pincode: 400001
Contact (Phone): 9876543210
-----------------------------------

*/

#include <stdio.h>
#include <string.h>

// Enum for Employee Type
typedef enum EmpType {
    INTERN,
    FULL_TIME,
    CONTRACT
} EmpType;

typedef union Contact {
    char phone[15];
    char email[30];
}Contact; 

// Nested struct for Address
typedef struct Address {
    char city[20];
    int pincode;
} Address;

// Main Employee structure (with nested Address)
typedef struct Employee {
    char name[30];
    int id;
    float salary;
    Address address;        // Nested struct
    EmpType type;          // Enum
    Contact contact;       // Union
    int contactType;       // 0 = phone, 1 = email (to track which one is used)
} Employee;


// Function to get employee type as string
const char* getEmpTypeString(EmpType type) {
    switch(type) {
        case INTERN: return "INTERN";
        case FULL_TIME: return "FULL_TIME";
        case CONTRACT: return "CONTRACT";
        default: return "UNKNOWN";
    }
}

// Function to input employee data using pointer
void inputEmployee(Employee *ptr, int num) {
    printf("\n=== Enter Details for Employee %d ===\n", num);
    
    printf("Name: ");
    scanf(" %[^\n]", ptr->name);
    
    printf("ID: ");
    scanf("%d", &ptr->id);
    
    printf("Salary: ");
    scanf("%f", &ptr->salary);
    
    printf("City: ");
    scanf(" %[^\n]", ptr->address.city);
    
    printf("Pincode: ");
    scanf("%d", &ptr->address.pincode);
    
    printf("Employee Type (0=INTERN, 1=FULL_TIME, 2=CONTRACT): ");
    int typeChoice;
    scanf("%d", &typeChoice);
    ptr->type = (EmpType)typeChoice;
    
    printf("Contact Type (0=Phone, 1=Email): ");
    scanf("%d", &ptr->contactType);
    
    if (ptr->contactType == 0) {
        printf("Phone: ");
        scanf(" %[^\n]", ptr->contact.phone);
    } else {
        printf("Email: ");
        scanf(" %[^\n]", ptr->contact.email);
    }
}

// Function to print employee data
void printEmployee(Employee *ptr, int num) {
    printf("\nEmployee %d:\n", num);
    printf("Name: %s\n", ptr->name);
    printf("ID: %d\n", ptr->id);
    printf("Type: %s\n", getEmpTypeString(ptr->type));
    printf("Salary: %.2f\n", ptr->salary);
    printf("City: %s\n", ptr->address.city);
    printf("Pincode: %d\n", ptr->address.pincode);
    
    if (ptr->contactType == 0) {
        printf("Contact (Phone): %s\n", ptr->contact.phone);
    } else {
        printf("Contact (Email): %s\n", ptr->contact.email);
    }
    printf("-----------------------------------\n");
}

int main() {
    Employee employees[3];  // Array of 3 employees
    Employee *ptr;          // Pointer to structure
    
    // Input data for 3 employees
    for (int i = 0; i < 3; i++) {
        ptr = &employees[i];  // Point to current employee
        inputEmployee(ptr, i + 1);
    }
    
    // Print all employee details
    printf("\n\n========== EMPLOYEE DETAILS ==========\n");
    for (int i = 0; i < 3; i++) {
        ptr = &employees[i];
        printEmployee(ptr, i + 1);
    }
    
    return 0;
}