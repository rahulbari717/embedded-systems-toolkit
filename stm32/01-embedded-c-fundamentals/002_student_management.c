#include <stdio.h>
#include <string.h>

// 1) Define Structures and Constants
#define MAX_STUDENTS 100   // Maximum number of students
#define MAX_NAME_LENGTH 50 // Maximum length of student name

// Student structure
typedef struct
{
    int rollNumber;
    char name[MAX_NAME_LENGTH];
    float marks;
} Student;

// 2) Declare Function Prototypes
void addStudent(Student *students, int *count, int rollNumber, const char *name, float marks);
void displayStudents(Student *students, int count);
void searchStudent(Student *students, int count, int rollNumber);

// 3) Implement addStudent Function
void addStudent(Student *students, int *count, int rollNumber, const char *name, float marks)
{
    if (*count >= MAX_STUDENTS)
    {
        printf("Cannot add more students. Maximum limit reached.\n");
        return;
    }
    students[*count].rollNumber = rollNumber;
    strncpy(students[*count].name, name, MAX_NAME_LENGTH - 1);
    students[*count].name[MAX_NAME_LENGTH - 1] = '\0'; // Ensure null termination
    students[*count].marks = marks;
    (*count)++;
}

// 4) Implement displayStudents Function
void displayStudents(Student *students, int count)
{
    printf("\n--- Student List ---\n");
    for (int i = 0; i < count; i++)
    {
        printf("Roll Number: %d, Name: %s, Marks: %.2f\n",
               students[i].rollNumber, students[i].name, students[i].marks);
    }
}

// 5) Implement searchStudent Function
void searchStudent(Student *students, int count, int rollNumber)
{
    for (int i = 0; i < count; i++)
    {
        if (students[i].rollNumber == rollNumber)
        {
            printf("\n--- Student Found ---\n");
            printf("Roll Number: %d, Name: %s, Marks: %.2f\n",
                   students[i].rollNumber, students[i].name, students[i].marks);
            return;
        }
    }
    printf("\nStudent with Roll Number %d not found.\n", rollNumber);
}

// 6) Create the main Function
int main()
{
    Student students[MAX_STUDENTS]; // Array to store students
    int count = 0;                  // Number of students added

    // Adding some students
    addStudent(students, &count, 101, "Rahul", 88.5);
    addStudent(students, &count, 102, "Ananya", 92.0);
    addStudent(students, &count, 103, "Siddharth", 79.5);

    // Display all students
    displayStudents(students, count);

    // Search for specific students
    searchStudent(students, count, 102); // Existing student
    searchStudent(students, count, 200); // Non-existing student

    return 0;
}
