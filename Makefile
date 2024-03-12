###################################
# Generic Makefile (based on gcc) #
###################################

# --------------------------------------------------------------
# target: your project name, use your dir name if not specified
# --------------------------------------------------------------
TARGET := $(shell basename "$$PWD")

# --------------------------------------------------------------
# compiler settings
# --------------------------------------------------------------
# debug build?
DEBUG := 1
# optimization
OPT := -Og
# set the gcc used
GCC_PREFIX := arm-none-eabi-
# add the chip info
CPU := -mcpu=cortex-m4
FPU := -mfpu=fpv4-sp-d16
FLOAT-ABI := -mfloat-abi=hard
# macros for gcc
AS_DEFINES :=
C_DEFINES := \
USE_HAL_DRIVER \
STM32F407xx \
ARM_MATH_CM4

# link script
LDSCRIPT := STM32F407IGHX_FLASH.ld
LIBS := -lahrs
# figure out compiler settings
CC := $(GCC_PREFIX)gcc
AS := $(GCC_PREFIX)gcc -x assembler-with-cpp
CP := $(GCC_PREFIX)objcopy
SZ := $(GCC_PREFIX)size
GDB := $(GCC_PREFIX)gdb
HEX := $(CP) -O ihex
BIN := $(CP) -O binary
OOCD := openocd
OOCDFLAGS := -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg


# --------------------------------------------------------------
# add all your used files here 
# --------------------------------------------------------------
SRC_DIRS := \
Src \
Inc \
application \
bsp \
components \
Startup \
Middlewares/Third_Party/FreeRTOS \
Middlewares/ST/STM32_USB_Device_Library \
Drivers/STM32F4xx_HAL_Driver \
Drivers/CMSIS/Include \
Drivers/CMSIS/Device/ST/STM32F4xx/Include 

# where the output files are stored
BUILD_DIR := ./build
# separate your files
SRCS := $(shell find $(SRC_DIRS) -name '*.cpp' -or -name '*.c' -or -name '*.s')
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)
INC_DIRS := $(shell find $(SRC_DIRS) -type d)

# --------------------------------------------------------------
# generate flags for compiler
# --------------------------------------------------------------
MCU := $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
AS_DEFS := $(addprefix -D,$(AS_DEFINES))
C_DEFS := $(addprefix -D,$(C_DEFINES))
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
LIB_FLAGS := $(addprefix -L,$(INC_DIRS))
# ASM
ASFLAGS := $(MCU) $(AS_DEFS) $(INC_FLAGS) $(OPT) -Wall -fdata-sections -ffunction-sections
# CXX
CXXFLAGS = $(MCU) $(C_DEFS) $(INC_FLAGS) $(OPT) -Wall -fdata-sections -ffunction-sections -MMD -MP $(addprefix -MF,$(@:%.c.o=%.c.d))
# C
CFLAGS = $(MCU) $(C_DEFS) $(INC_FLAGS) $(OPT) -Wall -fdata-sections -ffunction-sections -MMD -MP $(addprefix -MF,$(@:%.c.o=%.c.d))
ifeq ($(DEBUG), 1)
	CFLAGS += -g -gdwarf-3
endif
# libraries
LIBS += -lc -lm 
LDFLAGS := $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIB_FLAGS) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections 

# --------------------------------------------------------------
# build your project!
# --------------------------------------------------------------
# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	
# Build step for C++ source
$(BUILD_DIR)/%.cpp.o: %.cpp Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo -n -e "\e[36m[CXX]\e[0m compiling "; echo -n $<; echo "..."
	@$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.cpp.lst)) $< -o $@

# Build step for C source
$(BUILD_DIR)/%.c.o: %.c Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo -n -e "\e[36m[CC]\e[0m compiling "; echo -n $<; echo "..."
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.c.lst)) $< -o $@

# Build step for ASM source
$(BUILD_DIR)/%.s.o: %.s Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo -n -e "\e[1;34m[AS]\e[0;0m compiling "; echo -n $<; echo "..."
	@$(AS) -c $(ASFLAGS) $< -o $@

# Build step for generate elf file 
$(BUILD_DIR)/$(TARGET).elf: $(OBJS) Makefile
	@mkdir -p $(dir $@)
	@echo -e "\e[1;36m[LD]\e[0;0m linking..." 
	@$(CC) $(OBJS) $(LDFLAGS) -o $@
	@$(SZ) $@
	@$(CP) --only-keep-debug $@ $@.gnu_debuglink.debug
	@$(CP) --strip-all $@ $@.gnu_debuglink
	@$(CP) --add-gnu-debuglink=$@.gnu_debuglink.debug $@.gnu_debuglink

# Build step for generate hex file 
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo -e "\e[1;32m[HEX]\e[0;0m generating hex..."
	@$(HEX) $< $@
	
# Build step for generate bin file 
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo -e "\e[1;32m[BIN]\e[0;0m generating bin..."
	@$(BIN) $< $@
	
# mkdir for build if it not exist
$(BUILD_DIR):
	@mkdir $@

# --------------------------------------------------------------
# clean up
# --------------------------------------------------------------
.PHONY: clean
clean:
	@echo -e "\e[0;31mcleaning...\e[0;0m"
	@-rm -fR $(BUILD_DIR)

.PHONY: flash
flash: $(BUILD_DIR)/$(TARGET).bin
	@echo -e "\e[1;33m[OpenOCD]\e[0;0m programming..."
	@$(OOCD) $(OOCDFLAGS) -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset exit"

# .PHONY: debug
# debug: all
# 	@printf "  GDB DEBUG $<\n"
#     $(Q)$(GDB) -iex 'target extended | $(OOCD) $(OOCDFLAGS) -c "gdb_port pipe"' \
#     -iex 'monitor reset halt' -ex 'load' -ex 'break main' $(BUILD_DIR)/$(TARGET).elf

# --------------------------------------------------------------
# dependencies
# --------------------------------------------------------------
# Include the .d makefiles. The - at the front suppresses the errors of missing
# Makefiles. Initially, all the .d files will be missing, and we don't want those
# errors to show up.
-include $(DEPS)
