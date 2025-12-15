#include <stdio.h>
#include <stdlib.h>
// ------------------------------
// Function Declarations
// ------------------------------
void show_menu();
int get_choice();
void print_1_to_5();
void print_1_to_5_skip_3();
void handle_choice(int choice);

// ------------------------------
// Entry Point: main()
// ------------------------------
int main() {
    int user_choice;

    while (1) {
        show_menu();                  // Display menu to user
        user_choice = get_choice();   // Get and validate input
        handle_choice(user_choice);   // Handle the selected action
    }

    return 0;
}

// ------------------------------
// Function: show_menu()
// Description: Prints available options
// ------------------------------
void show_menu() {
    printf("\n=== Simple Control Menu ===\n");
    printf("1. Print numbers 1 to 5\n");
    printf("2. Print numbers 1 to 5 (skip 3)\n");
    printf("3. Exit\n");
}

// ------------------------------
// Function: get_choice()
// Description: Reads user input and validates range
// ------------------------------
int get_choice() {
    int choice;
    printf("Enter your choice (1-3): ");
    scanf("%d", &choice);

    // Input validation using if-else
    if (choice < 1 || choice > 3) {
        printf("Invalid choice! Please enter between 1 and 3.\n");
        return 0; // Special return for invalid
    }

    return choice;
}

// ------------------------------
// Function: print_1_to_5()
// Description: Uses a for loop to print 1 to 5
// ------------------------------
void print_1_to_5() {
    for (int i = 1; i <= 5; i++) {
        printf("%d ", i);
    }
    printf("\n");
}

// ------------------------------
// Function: print_1_to_5_skip_3()
// Description: Uses continue to skip 3
// ------------------------------
void print_1_to_5_skip_3() {
    for (int i = 1; i <= 5; i++) {
        if (i == 3) {
            continue; // Skip printing 3
        }
        printf("%d ", i);
    }
    printf("\n");
}

// ------------------------------
// Function: handle_choice()
// Description: Controls logic using switch-case,
//              break, goto, and function calls
// ------------------------------
void handle_choice(int choice) {
    // Label to allow clean exit using goto
    static int running = 1;

    // Handle invalid choice (0 returned from get_choice)
    if (choice == 0) {
        return; // Just return to menu loop
    }

    switch (choice) {
        case 1:
            print_1_to_5();           // Normal print
            break;

        case 2:
            print_1_to_5_skip_3();    // Print skipping 3 using continue
            break;

        case 3:
            goto exit_program;        // Use goto to jump to exit label
    }

    return;

exit_program:
    printf("Exiting program... Goodbye!\n");
    // Optional: Do any cleanup here
    exit(0); // End the program
}

