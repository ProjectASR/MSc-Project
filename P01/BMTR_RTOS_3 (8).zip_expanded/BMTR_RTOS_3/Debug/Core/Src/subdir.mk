################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/DFR_i2c.c \
../Core/Src/FLASH_SECTOR_F7.c \
../Core/Src/MCP4725.c \
../Core/Src/dob.c \
../Core/Src/fatfs_sd.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/motor_encoder.c \
../Core/Src/moving_average_int16.c \
../Core/Src/rtob.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c 

OBJS += \
./Core/Src/DFR_i2c.o \
./Core/Src/FLASH_SECTOR_F7.o \
./Core/Src/MCP4725.o \
./Core/Src/dob.o \
./Core/Src/fatfs_sd.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/motor_encoder.o \
./Core/Src/moving_average_int16.o \
./Core/Src/rtob.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o 

C_DEPS += \
./Core/Src/DFR_i2c.d \
./Core/Src/FLASH_SECTOR_F7.d \
./Core/Src/MCP4725.d \
./Core/Src/dob.d \
./Core/Src/fatfs_sd.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/motor_encoder.d \
./Core/Src/moving_average_int16.d \
./Core/Src/rtob.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/DFR_i2c.cyclo ./Core/Src/DFR_i2c.d ./Core/Src/DFR_i2c.o ./Core/Src/DFR_i2c.su ./Core/Src/FLASH_SECTOR_F7.cyclo ./Core/Src/FLASH_SECTOR_F7.d ./Core/Src/FLASH_SECTOR_F7.o ./Core/Src/FLASH_SECTOR_F7.su ./Core/Src/MCP4725.cyclo ./Core/Src/MCP4725.d ./Core/Src/MCP4725.o ./Core/Src/MCP4725.su ./Core/Src/dob.cyclo ./Core/Src/dob.d ./Core/Src/dob.o ./Core/Src/dob.su ./Core/Src/fatfs_sd.cyclo ./Core/Src/fatfs_sd.d ./Core/Src/fatfs_sd.o ./Core/Src/fatfs_sd.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/motor_encoder.cyclo ./Core/Src/motor_encoder.d ./Core/Src/motor_encoder.o ./Core/Src/motor_encoder.su ./Core/Src/moving_average_int16.cyclo ./Core/Src/moving_average_int16.d ./Core/Src/moving_average_int16.o ./Core/Src/moving_average_int16.su ./Core/Src/rtob.cyclo ./Core/Src/rtob.d ./Core/Src/rtob.o ./Core/Src/rtob.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_hal_timebase_tim.cyclo ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_hal_timebase_tim.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su

.PHONY: clean-Core-2f-Src

