// 1. Define Structures and Constants
#include <stdio.h>
#include <string.h>

#define MAX_BOOKS               (100)
#define MAX_TITLE_LENGTH        (100)
#define MAX_AUTHOR_LENGTH       (50)

// Define Book structure
typedef struct
{
    const char *title;  // Title is read-only after adding
    const char *author; // Author is read-only after adding
    int year;
} Book;

// Define Library structure
typedef struct
{
    Book books[MAX_BOOKS]; // Array of books
    int count;             // Number of books currently in the library
} Library;

// 2. Declare Function Prototypes void initLibrary(Library *lib);
void addBook(Library *lib, const char *title, const char *author, int year);
void printBooks(const Library *lib);
void searchBooksByAuthor(const Library *lib, const char *author);

// 3. Implement initLibrary Function 
void initLibrary(Library *lib)
{
    lib->count = 0; // Initialize the book count to 0
}

// 4. Implement addBook Function 
void addBook(Library *lib, const char *title, const char *author, int year)
{
    if (lib->count >= MAX_BOOKS)
    {
        printf("Library is full!\n");
        return;
    }

    // Add book to the library
    lib->books[lib->count].title = title;
    lib->books[lib->count].author = author;
    lib->books[lib->count].year = year;
    lib->count++;
}

// 5. Implement printBooks Function 
void printBooks(const Library *lib)
{
    if (lib->count == 0)
    {
        printf("No books in the library.\n");
        return;
    }

    for (int i = 0; i < lib->count; i++)
    {
        printf("Book %d: %s by %s (%d)\n",
               i + 1,
               lib->books[i].title,
               lib->books[i].author,
               lib->books[i].year);
    }
}

// 6. Implement searchBooksByAuthor Function 
void searchBooksByAuthor(const Library *lib, const char *author)
{
    int found = 0;
    for (int i = 0; i < lib->count; i++)
    {
        if (strcmp(lib->books[i].author, author) == 0)
        {
            printf("%s by %s (%d)\n",
                   lib->books[i].title,
                   lib->books[i].author,
                   lib->books[i].year);
            found = 1;
        }`
    }
    if (!found)
    {
        printf("No books found by %s\n", author);
    }
}

// 7. Create the main Function 
int main()
{
    Library lib;

    // Initialize the library
    initLibrary(&lib);

    // Add books to the library
    addBook(&lib, "C Programming", "Dennis Ritchie", 1978);
    addBook(&lib, "The Pragmatic Programmer", "Andrew Hunt", 1999);
    addBook(&lib, "Clean Code", "Robert Martin", 2008);

    // Print all books
    printf("All books in the library:\n");
    printBooks(&lib);

    // Search books by author
    printf("\nBooks by Robert Martin:\n");
    searchBooksByAuthor(&lib, "Robert Martin");

    return 0;
}