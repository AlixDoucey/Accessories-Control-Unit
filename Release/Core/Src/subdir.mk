################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/NAU7802.c \
../Core/Src/app_threadx.c \
../Core/Src/displayUpdate.c \
../Core/Src/driveByWire.c \
../Core/Src/flashSector.c \
../Core/Src/gps.c \
../Core/Src/main.c \
../Core/Src/quickShifter.c \
../Core/Src/steeringLeds.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_hal_timebase_tim.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c 

S_UPPER_SRCS += \
../Core/Src/tx_initialize_low_level.S 

OBJS += \
./Core/Src/NAU7802.o \
./Core/Src/app_threadx.o \
./Core/Src/displayUpdate.o \
./Core/Src/driveByWire.o \
./Core/Src/flashSector.o \
./Core/Src/gps.o \
./Core/Src/main.o \
./Core/Src/quickShifter.o \
./Core/Src/steeringLeds.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_hal_timebase_tim.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o \
./Core/Src/tx_initialize_low_level.o 

S_UPPER_DEPS += \
./Core/Src/tx_initialize_low_level.d 

C_DEPS += \
./Core/Src/NAU7802.d \
./Core/Src/app_threadx.d \
./Core/Src/displayUpdate.d \
./Core/Src/driveByWire.d \
./Core/Src/flashSector.d \
./Core/Src/gps.d \
./Core/Src/main.d \
./Core/Src/quickShifter.d \
./Core/Src/steeringLeds.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_hal_timebase_tim.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H562xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.S Core/Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -DTX_SINGLE_MODE_NON_SECURE=1 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/NAU7802.cyclo ./Core/Src/NAU7802.d ./Core/Src/NAU7802.o ./Core/Src/NAU7802.su ./Core/Src/app_threadx.cyclo ./Core/Src/app_threadx.d ./Core/Src/app_threadx.o ./Core/Src/app_threadx.su ./Core/Src/displayUpdate.cyclo ./Core/Src/displayUpdate.d ./Core/Src/displayUpdate.o ./Core/Src/displayUpdate.su ./Core/Src/driveByWire.cyclo ./Core/Src/driveByWire.d ./Core/Src/driveByWire.o ./Core/Src/driveByWire.su ./Core/Src/flashSector.cyclo ./Core/Src/flashSector.d ./Core/Src/flashSector.o ./Core/Src/flashSector.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/quickShifter.cyclo ./Core/Src/quickShifter.d ./Core/Src/quickShifter.o ./Core/Src/quickShifter.su ./Core/Src/steeringLeds.cyclo ./Core/Src/steeringLeds.d ./Core/Src/steeringLeds.o ./Core/Src/steeringLeds.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_hal_timebase_tim.cyclo ./Core/Src/stm32h5xx_hal_timebase_tim.d ./Core/Src/stm32h5xx_hal_timebase_tim.o ./Core/Src/stm32h5xx_hal_timebase_tim.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su ./Core/Src/tx_initialize_low_level.d ./Core/Src/tx_initialize_low_level.o

.PHONY: clean-Core-2f-Src

