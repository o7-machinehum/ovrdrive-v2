RM := rm -rf
COMPILER_PREFIX := riscv-none-elf

ifeq ($(COMPILER_PREFIX),riscv-none-embed)
    MARCH_OPT := -march=rv32imac
else ifeq ($(COMPILER_PREFIX),riscv-none-elf)
    MARCH_OPT := -march=rv32imac_zicsr
else ifeq ($(COMPILER_PREFIX),riscv64-elf)
    MARCH_OPT := -march=rv32imac_zicsr
else
    $(error Unknown COMPILER_PREFIX: $(COMPILER_PREFIX))
endif

# Define option(s) defined in pre-processor compiler option(s)
DEFINE_OPTS = -DMSC_DEVICE -DDEBUG=1
# Optimisation option(s)
OPTIM_OPTS = -O3
# Debug option(s)
#DEBUG = -g
DEBUG =

BUILD_DIR = ./build

PROJECT = $(BUILD_DIR)/HydraUSB3_MSC

RVMSIS_DIR  = ./wch-ch56x-bsp/rvmsis
RVMSIS_SRCS = $(wildcard $(RVMSIS_DIR)/*.c)
OBJS       += $(patsubst $(RVMSIS_DIR)/%.c,$(BUILD_DIR)/%.o,$(RVMSIS_SRCS))

DRV_DIR   = ./wch-ch56x-bsp/drv
DRV_SRCS  = $(wildcard $(DRV_DIR)/*.c)
OBJS     += $(patsubst $(DRV_DIR)/%.c,$(BUILD_DIR)/%.o,$(DRV_SRCS))

BOARD_DIR   = ./wch-ch56x-bsp/board
BOARD_SRCS  = ./wch-ch56x-bsp/board/hydrausb3_v1.c
OBJS     += $(patsubst $(BOARD_DIR)/%.c,$(BUILD_DIR)/%.o,$(BOARD_SRCS))

USB_DIR   = ./wch-ch56x-bsp/usb/usb_devbulk
USB_SRCS  = $(wildcard $(USB_DIR)/*.c)
OBJS        += $(patsubst $(USB_DIR)/%.c,$(BUILD_DIR)/%.o,$(USB_SRCS))

USER_DIR  = ./src
USER_SRCS = $(wildcard $(USER_DIR)/*.c)
OBJS     += $(patsubst $(USER_DIR)/%.c,$(BUILD_DIR)/%.o,$(USER_SRCS))

# All of the sources participating in the build are defined here
OBJS += $(BUILD_DIR)/startup_CH56x.o
DEPS  = $(subst .o,.d,$(OBJS))
LIBS  =

BASE_OPTS = $(MARCH_OPT) -mabi=ilp32 -msmall-data-limit=8 $(OPTIM_OPTS) -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections
C_OPTS    = $(BASE_OPTS) $(DEBUG) $(DEFINE_OPTS)\
              $(INCLUDES) -std=gnu99 -MMD -MP -MT"$(@)"
LD_OPTS   = -T "src/link.ld" -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"$(PROJECT).map" --specs=nano.specs --specs=nosys.specs

INCLUDES = \
  -I"$(RVMSIS_DIR)" \
  -I"$(DRV_DIR)" \
  -I"$(BOARD_DIR)" \
  -I"$(USB_DIR)" \
  -I"$(USER_DIR)"

# Add inputs and outputs from these tool invocations to the build variables
SECONDARY_FLASH += $(PROJECT).hex $(PROJECT).bin
SECONDARY_LIST  += $(PROJECT).lst
SECONDARY_SIZE  += $(PROJECT).siz
SECONDARY_MAP   += $(PROJECT).map

SECONDARY_OUTPUTS = $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE) $(SECONDARY_MAP)
secondary-outputs: $(SECONDARY_OUTPUTS)

# All Target
all: $(PROJECT).elf secondary-outputs

.PRECIOUS: $(BUILD_DIR)/. $(BUILD_DIR)%/.

$(BUILD_DIR)/.:
	mkdir -p $@

$(BUILD_DIR)%/.:
	mkdir -p $@

.SECONDEXPANSION:

$(BUILD_DIR)/startup_CH56x.o: ./wch-ch56x-bsp/startup/startup_CH56x.S | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -x assembler -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: ./src/%.c | $$(@D)/.
	@echo $(OBJS)
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: ./wch-ch56x-bsp/rvmsis/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: ./wch-ch56x-bsp/drv/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: ./wch-ch56x-bsp/board/%.c | $$(@D)/.
	@echo 'Building file: $<'
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

$(BUILD_DIR)/%.o: ./wch-ch56x-bsp/usb/usb_devbulk/%.c | $$(@D)/.
	@echo 'Building file: $<'
	mkdir -p $(@D)
	$(COMPILER_PREFIX)-gcc $(C_OPTS) -c -o "$@" "$<"
	@echo ' '

# Tool invocations
$(PROJECT).elf: $(OBJS)
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	$(COMPILER_PREFIX)-gcc $(BASE_OPTS) $(LD_OPTS) -o "$(PROJECT).elf" $(OBJS) $(LIBS)
	@echo ' '

$(PROJECT).hex: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	$(COMPILER_PREFIX)-objcopy -O ihex "$(PROJECT).elf"  "$(PROJECT).hex"
	@echo ' '

$(PROJECT).bin: $(PROJECT).elf
	-@echo 'Create Flash Image BIN'
	-$(COMPILER_PREFIX)-objcopy -O binary "$(PROJECT).elf"  "$(PROJECT).bin"
	-@echo ' '

$(PROJECT).lst: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	$(COMPILER_PREFIX)-objdump --source --all-headers --demangle --line-numbers --wide "$(PROJECT).elf" > "$(PROJECT).lst"
	@echo ' '

$(PROJECT).siz: $(PROJECT).elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	$(COMPILER_PREFIX)-size --format=berkeley "$(PROJECT).elf"
	@echo ' '

flash: all
	sudo ./wch-ch56x-isp/wch-ch56x-isp -f "$(PROJECT).bin"

# Other Targets
clean:
	-$(RM) $(OBJS) $(DEPS) $(SECONDARY_OUTPUTS) $(PROJECT).elf
	-@echo ' '

.PHONY: all clean dependents
