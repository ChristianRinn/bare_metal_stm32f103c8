########################################
## bare metal makefile for ARM Cortex ##
########################################

NAME      = hello_world
#DEBUG	  = 1

# run "make install" with OOCD_IFACE=stlink-v2-1 to use ST-Link V2.1, e.g. from Nucleo boards
OOCD_IFACE ?= stlink-v2

# run e.g. make EXAMPLE=example02 install to compile and upload example02_*
EXAMPLE ?= example01

SRCS      = $(wildcard src/STM32F103C8/*.c)
SRCS  	 += $(wildcard src/$(EXAMPLE)*/*.c)

INCDIRS   = src/STM32F103C8

LSCRIPT   = src/STM32F103C8/gcc_linker.ld

DEFINES   = $(EXDEFINES)

BUILDDIR  = build/

CFLAGS    = -ffunction-sections
CFLAGS   += -mlittle-endian
CFLAGS   += -mthumb
CFLAGS   += -mcpu=cortex-m3
CFLAGS   += -mfloat-abi=soft
CFLAGS   += -std=gnu11
CFLAGS   += -ggdb
CFLAGS   += -DSTM32F103xB

ifdef DEBUG
    CFLAGS   += -Og
    CFLAGS	 += -g3
else
    CFLAGS   += -Os -flto
endif

LFLAGS    = --specs=nano.specs 
LFLAGS   += --specs=nosys.specs 
LFLAGS   += -nostartfiles
LFLAGS   += -Wl,--gc-sections
LFLAGS   += -T$(LSCRIPT)
LFLAGS   += -lm

WFLAGS    = -Wall
WFLAGS   += -Wextra
WFLAGS   += -Wstrict-prototypes
WFLAGS   += -Werror -Wno-error=unused-function -Wno-error=unused-variable
WFLAGS   += -Wfatal-errors
WFLAGS   += -Warray-bounds
WFLAGS   += -Wno-unused-parameter

GCCPREFIX = arm-none-eabi-
CC        = $(GCCPREFIX)gcc
OBJCOPY   = $(GCCPREFIX)objcopy
OBJDUMP   = $(GCCPREFIX)objdump
SIZE      = $(GCCPREFIX)size

INCLUDE   = $(addprefix -I,$(INCDIRS))

OBJS      = $(addprefix $(BUILDDIR), $(addsuffix .o, $(basename $(SRCS))))
OBJDIR    = $(sort $(dir $(OBJS)))
BIN_NAME  = $(addprefix $(BUILDDIR), $(NAME))


###########
## rules ##
###########

.PHONY: all
all: $(BIN_NAME).elf
all: $(BIN_NAME).bin
all: $(BIN_NAME).s19
all: $(BIN_NAME).hex
all: $(BIN_NAME).lst
all: print_size


.PHONY: clean
clean:
	$(RM) -r $(wildcard $(BUILDDIR)*)

.PHONY: install
install: $(BIN_NAME).hex
	@echo;
	@echo [OpenOCD] program $<:
	openocd -d0 \
	-f interface/$(OOCD_IFACE).cfg \
	-f target/stm32f1x.cfg \
 	-c "program $<" \
 	-c "reset run" \
 	-c "shutdown"
	

###########################################
# Internal Rules                          #
###########################################

# create directories
$(OBJS): | $(OBJDIR)
$(OBJDIR):
	mkdir -p $@

# compiler
$(BUILDDIR)%.o: %.c
	@echo;
	@echo [CC] $<:
	$(CC) -MMD -c -o $@ $(INCLUDE) $(DEFINES) $(CFLAGS) $(WFLAGS) $<

# assembler
$(BUILDDIR)%.o: %.s
	@echo;
	@echo [AS] $<:
	$(CC) -c -x assembler-with-cpp -o $@ $(INCLUDE) $(DEFINES) $(CFLAGS) $(WFLAGS) $<

# linker
$(BUILDDIR)%.elf: $(OBJS)
	@echo;
	@echo [LD] $@:
	$(CC) -o $@ $^ $(CFLAGS) $(LFLAGS) -Wl,-Map=$(addsuffix .map, $(basename $@))

%.bin: %.elf
	@echo;
	@echo [objcopy] $@:
	$(OBJCOPY) -O binary -S $< $@

%.s19: %.elf
	@echo;
	@echo [objcopy] $@:
	$(OBJCOPY) -O srec -S $< $@

%.hex: %.elf
	@echo;
	@echo [objcopy] $@:
	$(OBJCOPY) -O ihex -S $< $@

%.lst: %.elf
	@echo;
	@echo [objdump] $@:
	$(OBJDUMP) -D $< > $@

.PHONY: print_size
print_size: $(BIN_NAME).elf
	@echo;
	@echo [SIZE] $<:
	$(SIZE) $<


#####################
## Advanced Voodoo ##
#####################

# try to include any compiler generated dependency makefile snippet *.d
# that might exist in BUILDDIR (but don't complain if it doesn't yet).
DEPS = $(patsubst %.o,%.d,$(OBJS))
-include $(DEPS)

# make the object files also depend on the makefile itself
$(OBJS): Makefile


