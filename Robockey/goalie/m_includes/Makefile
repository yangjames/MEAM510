DEVICE = atmega32u4
CLOCK = 16000000

COMPILE = avr-gcc -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
SRCS = $(shell find -name '*.c')
OBJS = ${SRCS:.c=.o}

all: $(OBJS)

$(OBJS): ${SRCS:.c=.h}

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

clean:
	if (rm *~) ; then :; fi
	if (rm -f *.o) ; then :; fi
