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
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/BasicMathFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/BayesFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/CommonTables" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/ComplexMathFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/ControllerFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/DistanceFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/FastMathFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/FilteringFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/InterpolationFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/MatrixFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/QuaternionMathFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/StatisticsFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/SupportFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/SVMFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/TransformFunctions" -I"C:/Users/ASUS/code/GitHub/New folder/FRA262-PR-Plotter/verticle_plotter/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

