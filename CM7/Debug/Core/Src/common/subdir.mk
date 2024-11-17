################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/common/foc_utils.cpp \
../Core/Src/common/lowpass_filter.cpp \
../Core/Src/common/pid.cpp \
../Core/Src/common/time_utils.cpp 

OBJS += \
./Core/Src/common/foc_utils.o \
./Core/Src/common/lowpass_filter.o \
./Core/Src/common/pid.o \
./Core/Src/common/time_utils.o 

CPP_DEPS += \
./Core/Src/common/foc_utils.d \
./Core/Src/common/lowpass_filter.d \
./Core/Src/common/pid.d \
./Core/Src/common/time_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/common/%.o Core/Src/common/%.su Core/Src/common/%.cyclo: ../Core/Src/common/%.cpp Core/Src/common/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DARM_MATH_CM7 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/CMSIS/DSP/Include -I"C:/ST/STM32CubeIDE_1.16.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.win32_1.0.200.202406191623/tools/arm-none-eabi/include/c++/12.3.1" -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Inc" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/sensors" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/motors" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/common" -I"C:/Users/fresh/STM32CubeIDE/workspace_1.16.0/project3/CM7/Core/Src/common/base_classes" -Og -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fno-common -ffunction-sections -fdata-sections -mno-unaligned-access -fno-section-anchors -Wl,--print-gc-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-common

clean-Core-2f-Src-2f-common:
	-$(RM) ./Core/Src/common/foc_utils.cyclo ./Core/Src/common/foc_utils.d ./Core/Src/common/foc_utils.o ./Core/Src/common/foc_utils.su ./Core/Src/common/lowpass_filter.cyclo ./Core/Src/common/lowpass_filter.d ./Core/Src/common/lowpass_filter.o ./Core/Src/common/lowpass_filter.su ./Core/Src/common/pid.cyclo ./Core/Src/common/pid.d ./Core/Src/common/pid.o ./Core/Src/common/pid.su ./Core/Src/common/time_utils.cyclo ./Core/Src/common/time_utils.d ./Core/Src/common/time_utils.o ./Core/Src/common/time_utils.su

.PHONY: clean-Core-2f-Src-2f-common

