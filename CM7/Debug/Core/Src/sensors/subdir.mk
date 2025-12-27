################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/sensors/AS5048A.cpp 

OBJS += \
./Core/Src/sensors/AS5048A.o 

CPP_DEPS += \
./Core/Src/sensors/AS5048A.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su Core/Src/sensors/%.cyclo: ../Core/Src/sensors/%.cpp Core/Src/sensors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DARM_MATH_CM7 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I"C:/Users/fresh/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/DSP/Include" -I"C:/ST/STM32CubeIDE_2.0.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712/tools/arm-none-eabi/include/c++/13.3.1" -I"C:/_projects/FOC/cube/project3/Drivers/CMSIS/DSP/Include" -I"C:/_projects/FOC/cube/project3/CM7/Core/Inc" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/sensors" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/motors" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/common" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/common/base_classes" -I"C:/Users/fresh/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/Include" -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fno-common -ffunction-sections -fdata-sections -mno-unaligned-access -fno-section-anchors -Wl,--print-gc-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/AS5048A.cyclo ./Core/Src/sensors/AS5048A.d ./Core/Src/sensors/AS5048A.o ./Core/Src/sensors/AS5048A.su

.PHONY: clean-Core-2f-Src-2f-sensors

