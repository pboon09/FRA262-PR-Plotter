################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/BasicMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/BayesFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/CommonTables" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/ComplexMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/ControllerFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/DistanceFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/FastMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/FilteringFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/InterpolationFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/MatrixFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/QuaternionMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/StatisticsFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/SupportFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/SVMFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/TransformFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-BasicMathFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-BasicMathFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/BasicMathFunctions/BasicMathFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-BasicMathFunctions

