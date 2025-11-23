/*
✅ 1. Basic – Structure Declaration & Access
**Q1: Create a structure Student with members: name, roll, and marks.

Write a program to input the data of 3 students and print it.**

Concepts Covered: basic struct, accessing members.

*/

#include <stdio.h>

typedef struct Student{
    char name[50];
    int roll;
    int marks;  
} x;

// Function to input data for a student
void input_data(x *s, int num) {
    printf("\nEnter details for Student %d:\n", num);
    printf("Name: ");
    scanf(" %[^\n]", s->name);  // Read string with spaces
    printf("Roll Number: ");
    scanf("%d", &s->roll);
    printf("Marks: ");
    scanf("%d", &s->marks);
}

// Function to print data for a student
void print_output_data(x s, int num) {
    printf("\n--- Student %d Details ---\n", num);
    printf("Name: %s\n", s.name);
    printf("Roll Number: %d\n", s.roll);
    printf("Marks: %d\n", s.marks);
}

int main(){
    x s1, s2, s3;  // Declare 3 student variables
    
    printf("=== Enter Information of 3 Students ===\n");
    
    // Input data for all 3 students
    input_data(&s1, 1);
    input_data(&s2, 2);
    input_data(&s3, 3);
    
    // Print data for all 3 students
    printf("\n\n=== Students Information ===\n");
    print_output_data(s1, 1);
    print_output_data(s2, 2);
    print_output_data(s3, 3);
    
    return 0;
}

/*

**Key Changes & Explanations:**

1. **Function declarations**: Added proper return types (`void`) and parameters
2. **Student declaration**: Changed `} s1, s2, s3;` to `} Student;` and declared variables in `main()`
3. **Pointer usage**: `input_data` takes a pointer so it can modify the actual struct
4. **String input**: Used `scanf(" %[^\n]", s->name)` to read names with spaces
5. **Arrow operator**: Used `s->name` instead of `s.name` when working with pointers
6. **Organized flow**: Input all students first, then print all their data

**Sample Output:**
```
=== Enter Information of 3 Students ===

Enter details for Student 1:
Name: John Doe
Roll Number: 101
Marks: 85

Enter details for Student 2:
Name: Jane Smith
Roll Number: 102
Marks: 92

*/