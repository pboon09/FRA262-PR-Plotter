################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/BasicMathFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/BayesFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/CommonTables" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/ComplexMathFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/ControllerFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/DistanceFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/FastMathFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/FilteringFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/InterpolationFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/MatrixFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/QuaternionMathFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/StatisticsFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/SupportFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/SVMFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/TransformFunctions" -I"C:/Users/HP/Downloads/FRA262-PR-Plotter/verticle_plotter/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FastMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FastMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/FastMathFunctions/FastMathFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-FastMathFunctions

