################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Raspberry_Pi_Quad.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Raspberry_Pi_Quad.list \

OBJCOPY_BIN += \
Raspberry_Pi_Quad.bin \


# All Target
all: Raspberry_Pi_Quad.elf secondary-outputs

# Tool invocations
Raspberry_Pi_Quad.elf: $(OBJS) $(USER_OBJS) D:\Users\callo\STM32CubeIDE\workspace_1.3.0\Raspberry_Pi_Quad\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Raspberry_Pi_Quad.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\Users\callo\STM32CubeIDE\workspace_1.3.0\Raspberry_Pi_Quad\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Raspberry_Pi_Quad.map" -Wl,--gc-sections -static -u _scanf_float --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Raspberry_Pi_Quad.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Raspberry_Pi_Quad.list"
	@echo 'Finished building: $@'
	@echo ' '

Raspberry_Pi_Quad.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Raspberry_Pi_Quad.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
