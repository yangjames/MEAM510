DEVICE = atmega32u4
CLOCK = 16000000
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

LIBRARIES += -lm

DIRECTORIES = $(sort $(dir $(wildcard */)))
LIBS = $(foreach DIRECTORIES,$(DIRECTORIES),-L./$(DIRECTORIES)) 
INCS = $(foreach DIRECTORIES,$(DIRECTORIES),-I./$(DIRECTORIES))
SRCS = $(foreach DIRECTORIES,$(DIRECTORIES),./$(shell find $(DIRECTORIES) -name '*.c'))
OBJS = $(filter-out ./,${SRCS:.c=.o})

all: precomp main.hex

precomp:
	for i in $(DIRECTORIES) ; do \
		if (cd $$i; make all) ; then :; fi; \
	done

.c.o:
	$(COMPILE) -c $< -o $@ $(LIBS) $(INCS)

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@ $(LIBS) $(INCS)

.c.s:
	$(COMPILE) -S $< -o $@ $(LIBS) $(INCS)

flashyflash: flash

install: flash

flash: all
	dfu-programmer $(DEVICE) erase
	dfu-programmer $(DEVICE) flash main.hex

main.elf: main.o
	$(COMPILE) -o $@ $< $(OBJS) $(LIBRARIES) $(LIBS) $(INCS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

disasm: main.elf
	avr-objdump -d main.elf

clean:
	if (rm *.o) ; then :; fi
	if (rm -f *~) ; then :; fi
	for i in $(DIRECTORIES) ; do \
		if ( cd $$i && make clean ) ; then :; fi; \
	done
