################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Controller.c \
../Core/Src/Cytron_MDXX.c \
../Core/Src/MathOperation.c \
../Core/Src/MotorKalman.c \
../Core/Src/MotorMatrixGenerator.c \
../Core/Src/PWM.c \
../Core/Src/QEI.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/motor_config.c \
../Core/Src/signal_generator.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/Controller.o \
./Core/Src/Cytron_MDXX.o \
./Core/Src/MathOperation.o \
./Core/Src/MotorKalman.o \
./Core/Src/MotorMatrixGenerator.o \
./Core/Src/PWM.o \
./Core/Src/QEI.o \
./Core/Src/adc.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/motor_config.o \
./Core/Src/signal_generator.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/Controller.d \
./Core/Src/Cytron_MDXX.d \
./Core/Src/MathOperation.d \
./Core/Src/MotorKalman.d \
./Core/Src/MotorMatrixGenerator.d \
./Core/Src/PWM.d \
./Core/Src/QEI.d \
./Core/Src/adc.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/motor_config.d \
./Core/Src/signal_generator.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/BasicMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/BayesFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/CommonTables" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/ComplexMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/ControllerFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/DistanceFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/FastMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/FilteringFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/InterpolationFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/MatrixFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/QuaternionMathFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/StatisticsFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/SupportFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/SVMFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/TransformFunctions" -I"D:/FRA262-PR-Plotter/kalman_stm32/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Controller.cyclo ./Core/Src/Controller.d ./Core/Src/Controller.o ./Core/Src/Controller.su ./Core/Src/Cytron_MDXX.cyclo ./Core/Src/Cytron_MDXX.d ./Core/Src/Cytron_MDXX.o ./Core/Src/Cytron_MDXX.su ./Core/Src/MathOperation.cyclo ./Core/Src/MathOperation.d ./Core/Src/MathOperation.o ./Core/Src/MathOperation.su ./Core/Src/MotorKalman.cyclo ./Core/Src/MotorKalman.d ./Core/Src/MotorKalman.o ./Core/Src/MotorKalman.su ./Core/Src/MotorMatrixGenerator.cyclo ./Core/Src/MotorMatrixGenerator.d ./Core/Src/MotorMatrixGenerator.o ./Core/Src/MotorMatrixGenerator.su ./Core/Src/PWM.cyclo ./Core/Src/PWM.d ./Core/Src/PWM.o ./Core/Src/PWM.su ./Core/Src/QEI.cyclo ./Core/Src/QEI.d ./Core/Src/QEI.o ./Core/Src/QEI.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_config.cyclo ./Core/Src/motor_config.d ./Core/Src/motor_config.o ./Core/Src/motor_config.su ./Core/Src/signal_generator.cyclo ./Core/Src/signal_generator.d ./Core/Src/signal_generator.o ./Core/Src/signal_generator.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

