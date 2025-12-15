/*
 * Q) Take input from user and ask user to choose 0 for triangular star pattern and 1 for reverse star pattern
 *    Then print it accordingly
 *
 * */
#include <stdio.h>
#include <stdbool.h>

// Function prototypes
void get_user_input(bool *choice, int *n);
void pattern(bool choice, int n);

// Function to get user input
void get_user_input(bool *choice, int *n) {
    int c;

    printf("Enter 0 for triangular pattern or 1 for reverse pattern: ");
    scanf("%d", &c);

    if (c != 0 && c != 1) {
        printf("Invalid choice! Must be 0 or 1.\n");
        *choice = false;
        *n = 0;
        return;
    }

    *choice = (bool)c;

    printf("Enter number of rows: ");
    scanf("%d", n);
}

// Function to print pattern
void pattern(bool choice, int n) {
    if (choice == 0) {
        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= i; j++) {
                printf("*");
            }
            printf("\n");
        }
    } 
    else {
        for (int i = n; i >= 1; i--) {
            for (int j = 1; j <= i; j++) {
                printf("*");
            }
            printf("\n");
        }
    }
}

// Clean main()
int main() {
    bool choice;
    int n;

    get_user_input(&choice, &n);
    pattern(choice, n);

    return 0;
}

