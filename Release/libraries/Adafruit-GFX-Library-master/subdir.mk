################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../libraries/Adafruit-GFX-Library-master/Adafruit_GFX.cpp 

C_SRCS += \
../libraries/Adafruit-GFX-Library-master/glcdfont.c 

OBJS += \
./libraries/Adafruit-GFX-Library-master/Adafruit_GFX.o \
./libraries/Adafruit-GFX-Library-master/glcdfont.o 

C_DEPS += \
./libraries/Adafruit-GFX-Library-master/glcdfont.d 

CPP_DEPS += \
./libraries/Adafruit-GFX-Library-master/Adafruit_GFX.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/Adafruit-GFX-Library-master/%.o: ../libraries/Adafruit-GFX-Library-master/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/Applications/Arduino 1.0.app/Contents/Resources/Java/hardware/arduino/cores/arduino" -I"/Applications/Arduino 1.0.app/Contents/Resources/Java/hardware/arduino/variants/standard" -I"/Volumes/Overflow HDD/Dropbox/eclipse/ardy_LED_dimmer_eclipse" -D__IN_ECLIPSE__=1 -DUSB_VID= -DUSB_PID= -DARDUINO=103 -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '

libraries/Adafruit-GFX-Library-master/%.o: ../libraries/Adafruit-GFX-Library-master/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"/Applications/Arduino 1.0.app/Contents/Resources/Java/hardware/arduino/cores/arduino" -I"/Applications/Arduino 1.0.app/Contents/Resources/Java/hardware/arduino/variants/standard" -I"/Volumes/Overflow HDD/Dropbox/eclipse/ardy_LED_dimmer_eclipse" -D__IN_ECLIPSE__=1 -DARDUINO=103 -DUSB_PID= -DUSB_VID= -Wall -Os -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


