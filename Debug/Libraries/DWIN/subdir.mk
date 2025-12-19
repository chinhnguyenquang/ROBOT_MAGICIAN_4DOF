################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libraries/DWIN/Dwin.cpp 

OBJS += \
./Libraries/DWIN/Dwin.o 

CPP_DEPS += \
./Libraries/DWIN/Dwin.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/DWIN/%.o Libraries/DWIN/%.su Libraries/DWIN/%.cyclo: ../Libraries/DWIN/%.cpp Libraries/DWIN/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/DWIN" -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/STEP_MOTOR" -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/Set_home" -I"D:/DO_AN_ROBOT/codev2/v3/Libraries/Robot_Mapping_xyz" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-DWIN

clean-Libraries-2f-DWIN:
	-$(RM) ./Libraries/DWIN/Dwin.cyclo ./Libraries/DWIN/Dwin.d ./Libraries/DWIN/Dwin.o ./Libraries/DWIN/Dwin.su

.PHONY: clean-Libraries-2f-DWIN

