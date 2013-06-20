################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include libraries/Adafruit-PCD8544-Nokia-5110-LCD-library-master/examples/pcdtest/subdir.mk
-include libraries/Adafruit-PCD8544-Nokia-5110-LCD-library-master/subdir.mk
-include libraries/Adafruit-GFX-Library-master/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
ardy_Current_Meter.lss \

FLASH_IMAGE += \
ardy_Current_Meter.hex \

EEPROM_IMAGE += \
ardy_Current_Meter.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: ardy_Current_Meter.elf secondary-outputs

# Tool invocations
ardy_Current_Meter.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-gcc -Os -Wl,--gc-sections  -L"/Users/jacob/eclipse/Arduino_Duemilanove_w__ATmega328/Release" -mmcu=atmega328p  -o"ardy_Current_Meter.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '

ardy_Current_Meter.lss: ardy_Current_Meter.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S ardy_Current_Meter.elf  >"ardy_Current_Meter.lss"
	@echo 'Finished building: $@'
	@echo ' '

ardy_Current_Meter.hex: ardy_Current_Meter.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex ardy_Current_Meter.elf  "ardy_Current_Meter.hex"
	@echo 'Finished building: $@'
	@echo ' '

ardy_Current_Meter.eep: ardy_Current_Meter.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex ardy_Current_Meter.elf  "ardy_Current_Meter.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: ardy_Current_Meter.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p ardy_Current_Meter.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(INO_DEPS)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) ardy_Current_Meter.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
