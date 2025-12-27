################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h755zitx.s 

C_SRCS += \
../Core/Startup/handlers.c 

S_DEPS += \
./Core/Startup/startup_stm32h755zitx.d 

C_DEPS += \
./Core/Startup/handlers.d 

OBJS += \
./Core/Startup/handlers.o \
./Core/Startup/startup_stm32h755zitx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o Core/Startup/%.su Core/Startup/%.cyclo: ../Core/Startup/%.c Core/Startup/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/sensors" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/motors" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/common" -I"C:/_projects/FOC/cube/project3/CM7/Core/Src/common/base_classes" -I"C:/_projects/FOC/cube/project3/CM7/Core/Inc" -I"C:/Users/fresh/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1/Drivers/CMSIS/DSP/Include" -Og -ffunction-sections -fdata-sections -Wall -fno-section-anchors -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/handlers.cyclo ./Core/Startup/handlers.d ./Core/Startup/handlers.o ./Core/Startup/handlers.su ./Core/Startup/startup_stm32h755zitx.d ./Core/Startup/startup_stm32h755zitx.o

.PHONY: clean-Core-2f-Startup

