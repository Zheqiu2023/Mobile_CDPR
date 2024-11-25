################################################################################
# 自动生成的文件。不要编辑！
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
../User/Src/CDC_msg.c \
../User/Src/board.c \
../User/Src/bsp.c \
../User/Src/maxon_re35.c \
../User/Src/unitree_a1.c \
../User/Src/unitree_go.c \
../User/Src/utilities.c 

OBJS += \
./User/Src/CDC_msg.o \
./User/Src/board.o \
./User/Src/bsp.o \
./User/Src/maxon_re35.o \
./User/Src/unitree_a1.o \
./User/Src/unitree_go.o \
./User/Src/utilities.o 

C_DEPS += \
./User/Src/CDC_msg.d \
./User/Src/board.d \
./User/Src/bsp.d \
./User/Src/maxon_re35.d \
./User/Src/unitree_a1.d \
./User/Src/unitree_go.d \
./User/Src/utilities.d 


# 每个子目录必须为构建它所贡献的源提供规则
User/Src/%.o User/Src/%.su User/Src/%.cyclo: ../User/Src/%.c User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/Desktop/Mobile_CDPR/STM32/STM32/User/Inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User-2f-Src

clean-User-2f-Src:
	-$(RM) ./User/Src/CDC_msg.cyclo ./User/Src/CDC_msg.d ./User/Src/CDC_msg.o ./User/Src/CDC_msg.su ./User/Src/board.cyclo ./User/Src/board.d ./User/Src/board.o ./User/Src/board.su ./User/Src/bsp.cyclo ./User/Src/bsp.d ./User/Src/bsp.o ./User/Src/bsp.su ./User/Src/maxon_re35.cyclo ./User/Src/maxon_re35.d ./User/Src/maxon_re35.o ./User/Src/maxon_re35.su ./User/Src/unitree_a1.cyclo ./User/Src/unitree_a1.d ./User/Src/unitree_a1.o ./User/Src/unitree_a1.su ./User/Src/unitree_go.cyclo ./User/Src/unitree_go.d ./User/Src/unitree_go.o ./User/Src/unitree_go.su ./User/Src/utilities.cyclo ./User/Src/utilities.d ./User/Src/utilities.o ./User/Src/utilities.su

.PHONY: clean-User-2f-Src

