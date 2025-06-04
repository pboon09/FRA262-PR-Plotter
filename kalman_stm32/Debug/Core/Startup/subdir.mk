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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/BasicMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/BayesFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/CommonTables" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/ComplexMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/ControllerFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/DistanceFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/FastMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/FilteringFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/InterpolationFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/MatrixFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/QuaternionMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/StatisticsFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/SupportFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/SVMFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/TransformFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

