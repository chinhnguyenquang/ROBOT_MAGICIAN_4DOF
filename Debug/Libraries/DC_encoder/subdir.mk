################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/DC_encoder/Dc_motor.c 

C_DEPS += \
./Libraries/DC_encoder/Dc_motor.d 

OBJS += \
./Libraries/DC_encoder/Dc_motor.o 


# Each subdirectory must supply rules for building sources it contributes
Libraries/DC_encoder/%.o Libraries/DC_encoder/%.su Libraries/DC_encoder/%.cyclo: ../Libraries/DC_encoder/%.c Libraries/DC_encoder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/Set_home" -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/Robot_Mapping_xyz" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-DC_encoder

clean-Libraries-2f-DC_encoder:
	-$(RM) ./Libraries/DC_encoder/Dc_motor.cyclo ./Libraries/DC_encoder/Dc_motor.d ./Libraries/DC_encoder/Dc_motor.o ./Libraries/DC_encoder/Dc_motor.su

.PHONY: clean-Libraries-2f-DC_encoder

