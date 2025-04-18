################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/BasicMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/BayesFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/CommonTables" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/ComplexMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/ControllerFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/DistanceFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/FastMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/FilteringFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/InterpolationFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/MatrixFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/QuaternionMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/StatisticsFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/SupportFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/SVMFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/TransformFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

