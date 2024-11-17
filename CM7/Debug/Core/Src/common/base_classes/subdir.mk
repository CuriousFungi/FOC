################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/common/base_classes/CurrentSense.cpp 

OBJS += \
./Core/Src/common/base_classes/CurrentSense.o 

CPP_DEPS += \
./Core/Src/common/base_classes/CurrentSense.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/common/base_classes/%.o Core/Src/common/base_classes/%.su Core/Src/common/base_classes/%.cyclo: ../Core/Src/common/base_classes/%.cpp Core/Src/common/base_classes/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DARM_MATH_CM7 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/CMSIS/DSP/Include -I"C:/ST/STM32CubeIDE_1.16.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623/tools/arm-none-eabi/include/c++/12.3.1" -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Inc" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/sensors" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/motors" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/common" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/common/base_classes" -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fno-common -ffunction-sections -fdata-sections -mno-unaligned-access -fno-section-anchors -Wl,--print-gc-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-common-2f-base_classes

clean-Core-2f-Src-2f-common-2f-base_classes:
	-$(RM) ./Core/Src/common/base_classes/CurrentSense.cyclo ./Core/Src/common/base_classes/CurrentSense.d ./Core/Src/common/base_classes/CurrentSense.o ./Core/Src/common/base_classes/CurrentSense.su

.PHONY: clean-Core-2f-Src-2f-common-2f-base_classes

