################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/motors/StepperMotor.cpp 

OBJS += \
./Core/Src/motors/StepperMotor.o 

CPP_DEPS += \
./Core/Src/motors/StepperMotor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motors/%.o Core/Src/motors/%.su Core/Src/motors/%.cyclo: ../Core/Src/motors/%.cpp Core/Src/motors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I"C:/ST/STM32CubeIDE_1.16.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623/tools/arm-none-eabi/include/c++/12.3.1" -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-motors

clean-Core-2f-Src-2f-motors:
	-$(RM) ./Core/Src/motors/StepperMotor.cyclo ./Core/Src/motors/StepperMotor.d ./Core/Src/motors/StepperMotor.o ./Core/Src/motors/StepperMotor.su

.PHONY: clean-Core-2f-Src-2f-motors

