################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/BasicMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/BayesFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/CommonTables" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/ComplexMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/ControllerFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/DistanceFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/FastMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/FilteringFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/InterpolationFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/MatrixFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/QuaternionMathFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/StatisticsFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/SupportFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/SVMFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/TransformFunctions" -I"D:/What you need is in this folder/FRA262-PR-Plotter/verticle_plotter/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-MatrixFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-MatrixFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/MatrixFunctions/MatrixFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-MatrixFunctions

