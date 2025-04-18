################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/TransformFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/SupportFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/SVMFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/QuaternionMathFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/InterpolationFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/FilteringFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/DistanceFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/ControllerFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/ComplexMathFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/CommonTables/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/BayesFunctions/subdir.mk
-include Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := verticle_plotter
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
verticle_plotter.elf \

MAP_FILES += \
verticle_plotter.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
verticle_plotter.list \


# All Target
all: main-build

# Main-build Target
main-build: verticle_plotter.elf secondary-outputs

# Tool invocations
verticle_plotter.elf verticle_plotter.map: $(OBJS) $(USER_OBJS) D:\What\ you\ need\ is\ in\ this\ folder\FRA262-PR-Plotter\verticle_plotter\STM32G474RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "verticle_plotter.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\What you need is in this folder\FRA262-PR-Plotter\verticle_plotter\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="verticle_plotter.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

verticle_plotter.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "verticle_plotter.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout verticle_plotter.elf verticle_plotter.list verticle_plotter.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
