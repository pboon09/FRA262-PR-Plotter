# Make file generated with Waijung Blockset

PROJECT = io_toggle
DEBUG = 0
TOOLCHAIN = arm-none-eabi-
STACK_SIZE = 0xc00
HEAP_SIZE = 0
LD_SCRIPT = stm32f4xx_flash.ld

ELF = $(PROJECT).elf
HEX = $(PROJECT).hex
BIN = $(PROJECT).bin
LST = $(PROJECT).lss
DMP = $(PROJECT).dmp
MAP = $(PROJECT).map

# Tool chain definition
CXX = $(TOOLCHAIN)g++
CC = $(TOOLCHAIN)gcc
AS = $(TOOLCHAIN)gcc -x assembler-with-cpp
OBJCOPY = $(TOOLCHAIN)objcopy
OBJDUMP = $(TOOLCHAIN)objdump
SIZE = $(TOOLCHAIN)size

# Compiler and linker flags
CORE_FLAGS = $(patsubst %, -I%, .) -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffast-math -Wall -Wextra -O3
AS_FLAGS = $(CORE_FLAGS) -Wa,-amhls=$(<:.s=.lst) -D__STACK_SIZE=$(STACK_SIZE) -D__HEAP_SIZE=$(HEAP_SIZE)
C_FLAGS = $(CORE_FLAGS) -std=c99
CXX_FLAGS = $(CORE_FLAGS) -std=c++11
LD_FLAGS = -T$(LD_SCRIPT) -g -Wl,-Map=$(MAP),--cref,--no-warn-mismatch

AS_SRCS = $(wildcard *.s)
AS_OBJS = $(AS_SRCS:.s=.o)
C_SRCS = $(wildcard *.c)
C_OBJS = $(C_SRCS:.c=.o)

OBJS = $(AS_OBJS) $(C_OBJS)

ifeq ($(DEBUG), 1)
	AS_FLAGS += -g -ggdb3
	C_FLAGS += -g -ggdb3
	CXX_FLAGS += -g -ggdb3
	LD_FLAGS += -g -ggdb3
endif

LD_FLAGS += -nostartfiles

# Treat warning as error
CORE_FLAGS += -Werror


##############################################
# Make all
##############################################
all : $(ELF) $(HEX) $(BIN) $(LST) $(DMP) report

%.o : %.s
	@echo Assembling file: $<
	$(AS) -c $(AS_FLAGS) $< -o $@

%.o : %.c
	@echo Compiling: $<
	$(CC) -c $(C_FLAGS) $< -o $@

$(ELF) : $(OBJS)
	@echo Linking target: $(ELF)
	$(CXX) $(LD_FLAGS) $(OBJS) $(LIBS) -o $@

$(HEX) : $(ELF)
	@echo Creating HEX: $(HEX)
	$(OBJCOPY) -O ihex $< $@

$(BIN) : $(ELF)
	@echo Creating binary image: $(BIN)
	$(OBJCOPY) -O binary $< $@

$(LST) : $(ELF)
	@echo Creating listing: $(LST)
	$(OBJDUMP) -S $< > $@

$(DMP) : $(ELF)
	@echo Creating memory dump: $(DMP)
	$(OBJDUMP) -x --syms $< > $@

report :
	@echo.
	@echo ===================================================================
	@echo Target size:
	$(SIZE) -B $(ELF)
	@echo.
	@echo Objects size:
	$(SIZE) -B -t --common $(OBJS) $(USER_OBJS)
	@echo ===================================================================

clean-obj:
	del /Q /F *.o
	del /Q /F *.lst

