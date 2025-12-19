################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/Set_home/Sethome.c 

C_DEPS += \
./Libraries/Set_home/Sethome.d 

OBJS += \
./Libraries/Set_home/Sethome.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Set_home/%.o Libraries/Set_home/%.su Libraries/Set_home/%.cyclo: ../Libraries/Set_home/%.c Libraries/Set_home/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/Set_home" -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/Robot_Mapping_xyz" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-Set_home

clean-Libraries-2f-Set_home:
	-$(RM) ./Libraries/Set_home/Sethome.cyclo ./Libraries/Set_home/Sethome.d ./Libraries/Set_home/Sethome.o ./Libraries/Set_home/Sethome.su

.PHONY: clean-Libraries-2f-Set_home

