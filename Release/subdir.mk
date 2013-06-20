################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ardy_Current_Meter.cpp 

OBJS += \
./ardy_Current_Meter.o 

CPP_DEPS += \
./ardy_Current_Meter.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/Applications/Arduino 1.0.app/Contents/Resources/Java/hardware/arduino/cores/arduino" -I"/Applications/Arduino 1.0.app/Contents/Resources/Java/hardware/arduino/variants/standard" -I"/Users/jacob/eclipse/ardy_LED_dimmer_eclipse" -D__IN_ECLIPSE__=1 -DUSB_VID= -DUSB_PID= -DARDUINO=103 -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '


