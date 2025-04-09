################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/morse.c \
../Src/stm32f429i_gpio_driver.c \
../Src/stm32f429i_i2c_driver.c \
../Src/stm32f429i_timer_driver.c \
../Src/syscall.c \
../Src/sysmem.c 

OBJS += \
./Src/main.o \
./Src/morse.o \
./Src/stm32f429i_gpio_driver.o \
./Src/stm32f429i_i2c_driver.o \
./Src/stm32f429i_timer_driver.o \
./Src/syscall.o \
./Src/sysmem.o 

C_DEPS += \
./Src/main.d \
./Src/morse.d \
./Src/stm32f429i_gpio_driver.d \
./Src/stm32f429i_i2c_driver.d \
./Src/stm32f429i_timer_driver.d \
./Src/syscall.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/morse.cyclo ./Src/morse.d ./Src/morse.o ./Src/morse.su ./Src/stm32f429i_gpio_driver.cyclo ./Src/stm32f429i_gpio_driver.d ./Src/stm32f429i_gpio_driver.o ./Src/stm32f429i_gpio_driver.su ./Src/stm32f429i_i2c_driver.cyclo ./Src/stm32f429i_i2c_driver.d ./Src/stm32f429i_i2c_driver.o ./Src/stm32f429i_i2c_driver.su ./Src/stm32f429i_timer_driver.cyclo ./Src/stm32f429i_timer_driver.d ./Src/stm32f429i_timer_driver.o ./Src/stm32f429i_timer_driver.su ./Src/syscall.cyclo ./Src/syscall.d ./Src/syscall.o ./Src/syscall.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

