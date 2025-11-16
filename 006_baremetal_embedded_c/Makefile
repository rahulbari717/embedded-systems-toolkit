cc=arm-none-eabi-gcc
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0

all: main.o led.o stm32_startup.o 

main.o:main.c
	$(cc) $(CFLAGS) $^ -o $@
	
led.o:led.c
	$(cc) $(CFLAGS) $^ -o $@

stm32_startup.o:stm32_startup.c
	$(cc) $(CFLAGS) $^ -o $@

clean:
	rm -rf *.o *.elf
