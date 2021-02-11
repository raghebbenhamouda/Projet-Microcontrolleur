################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Utilities/STM32F429I-Discovery/subdir.mk
-include Utilities/Common/subdir.mk
-include Libraries/STM32F4xx_StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECOUTPUTBUILDVAR += \
EXECUTABLES \


# All Target
all: Hello.elf secondary-outputs

# Tool invocations
Hello.elf: $(OBJS) $(USER_OBJS)
	@echo linking    .elf & arm-atollic-eabi-gcc -o "Hello.elf" $(OBJS) $(USER_OBJS) $(LIBS) -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T"../stm32f4_flash.ld" -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler "-Wl,-Map=Hello.map" -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs

EXECUTABLES: Hello.elf $(EXECUTABLES)
	"C:\Program Files (x86)\Atollic\TrueSTUDIO for STM32 9.3.0\ide\jre\bin\java" -jar "C:\Program Files (x86)\Atollic\TrueSTUDIO for STM32 9.3.0\Tools\arm-atollic-reports.jar" convert-hex sizeinfo list Hello.elf

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(SECOUTPUTBUILDVAR)$(OBJS)$(C_DEPS) Hello.elf
	-@echo ' '

secondary-outputs: $(SECOUTPUTBUILDVAR)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
