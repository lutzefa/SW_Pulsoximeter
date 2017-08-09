################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include USB/subdir.mk
-include Graphics/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/EFM32WG/subdir.mk
-include BSP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: STK3800_biometric.axf

# Tool invocations
STK3800_biometric.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "STK3800_biometric.ld" -L"C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/gecko_sdk_suite/v1.1//util/silicon_labs/sensor_si114xhrm/lib/" -Xlinker --gc-sections -Xlinker -Map="STK3800_biometric.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o STK3800_biometric.axf "./BSP/bsp_bcc.o" "./BSP/bsp_stk.o" "./BSP/bsp_trace.o" "./CMSIS/EFM32WG/startup_efm32wg.o" "./CMSIS/EFM32WG/system_efm32wg.o" "./Drivers/i2cspm.o" "./Drivers/rtcdriver.o" "./Drivers/segmentlcd.o" "./Drivers/si114x_functions.o" "./Drivers/si114x_sys_out.o" "./Drivers/si114x_uv_index.o" "./Drivers/si7013.o" "./Graphics/graphics.o" "./USB/descriptors.o" "./USB/em_usbd.o" "./USB/em_usbdch9.o" "./USB/em_usbdep.o" "./USB/em_usbdint.o" "./USB/em_usbhal.o" "./USB/em_usbtimer.o" "./USB/usb_debug.o" "./emlib/em_adc.o" "./emlib/em_assert.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_dma.o" "./emlib/em_ebi.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_lcd.o" "./emlib/em_prs.o" "./emlib/em_rtc.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./src/biometric_demo.o" "./src/heart_rate_monitor.o" -lsi114xHRM_GCC_CM4F -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: STK3800_biometric.hex'
	arm-none-eabi-objcopy -O ihex "STK3800_biometric.axf" "STK3800_biometric.hex"
	@echo ' '

	@echo 'Building bin file: STK3800_biometric.bin'
	arm-none-eabi-objcopy -O binary "STK3800_biometric.axf" "STK3800_biometric.bin"
	@echo ' '

	@echo 'Building s37 file: STK3800_biometric.s37'
	arm-none-eabi-objcopy -O srec "STK3800_biometric.axf" "STK3800_biometric.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "STK3800_biometric.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) STK3800_biometric.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
