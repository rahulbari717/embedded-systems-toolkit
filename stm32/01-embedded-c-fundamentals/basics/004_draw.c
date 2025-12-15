#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define WIDTH 500
#define HEIGHT 500

int main() {
    FILE *f = fopen("diamond.ppm", "w");
    if (!f) {
        perror("Unable to open file");
        return 1;
    }

    fprintf(f, "P3\n%d %d\n255\n", WIDTH, HEIGHT); // P3 format header

    int cx = WIDTH / 2;
    int cy = HEIGHT / 2;
    int size = 100;

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            int dx = abs(x - cx);
            int dy = abs(y - cy);

            if (dx + dy < size) {
                // Diamond area: white
                fprintf(f, "255 255 255 ");
            } else {
                // Background: black
                fprintf(f, "0 0 0 ");
            }
        }
        fprintf(f, "\n");
    }

    fclose(f);
    printf("Diamond image generated: diamond.ppm\n");
    return 0;
}
