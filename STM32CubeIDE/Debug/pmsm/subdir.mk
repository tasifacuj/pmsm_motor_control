################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../pmsm/pmsm.c 

OBJS += \
./pmsm/pmsm.o 

C_DEPS += \
./pmsm/pmsm.d 


# Each subdirectory must supply rules for building sources it contributes
pmsm/%.o pmsm/%.su pmsm/%.cyclo: ../pmsm/%.c pmsm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../../Inc -I"C:/Users/t.shypytiak/work/projects/stm32/pmsm_motor_control/STM32CubeIDE" -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-pmsm

clean-pmsm:
	-$(RM) ./pmsm/pmsm.cyclo ./pmsm/pmsm.d ./pmsm/pmsm.o ./pmsm/pmsm.su

.PHONY: clean-pmsm

