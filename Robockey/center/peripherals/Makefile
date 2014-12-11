DEVICE = atmega32u4
CLOCK = 16000000

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
SRCS = $(shell find -name '*.c')
OBJS = ${SRCS:.c=.o} 

M_DIR = ../m_includes
M_SRCS = $(shell find ../m_includes/ -name '*.c')
M_OBJS = $(filter-out ./,${M_SRCS:.c=.o})

all: $(OBJS)

$(OBJS): ${SRCS:.c=.h}

.c.o:
	$(COMPILE) -c $< -o $@ -I$(M_DIR) -L$(M_DIR) $(M_OBJS)

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@ -I$(M_DIR) -L$(M_DIR) $(M_OBJS)

.c.s:
	$(COMPILE) -S $< -o $@ -I$(M_DIR) -L$(M_DIR) $(M_OBJS)

clean:
	if (rm \#*) ; then :; fi
	if (rm *~) ; then :; fi
	if (rm -f *.o) ; then :; fi
