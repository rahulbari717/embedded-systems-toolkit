/*
Problem 1: Reverse a Null-Terminated String In-Place
Issue Description
Reverse a null-terminated string in-place, e.g., "hello" becomes "olleh".
Handle edge cases like NULL or empty strings.
Problem Decomposition & Solution Steps
Input: Null-terminated string.
Output: Reversed string in-place.
Approach: Use two pointers to swap characters from ends.
Steps: Validate input, find length, swap characters using pointers.
Complexity: Time O(n), Space O(1).


*/

#include <stdio.h>
#include <string.h>

// Reverses a null-terminated string in-place.
void reverseString(char* str) {
if (str == NULL || str[0] == '\0') return; // Handle NULL or empty
int len = 0;
while (str[len] != '\0') len++; // Find length
    for (int i = 0, j = len - 1; i < j; i++, j--) { // Swap characters
        char temp = str[i];
        str[i] = str[j];
        str[j] = temp;
    }
}

int main() {

    char input[100];

    printf("Enter a string: ");
    scanf("%99s", input);  // Read string safely

    reverseString(input);

    printf("Reversed string: %s\n", input);

    return 0;
}