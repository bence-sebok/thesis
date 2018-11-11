################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include Drivers/si4x6x/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/EZR32LG/subdir.mk
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
all: thesis_gecko.axf

# Tool invocations
thesis_gecko.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m3 -mthumb -T "thesis_gecko.ld" -Xlinker --gc-sections -Xlinker -Map="thesis_gecko.map" --specs=nano.specs -o thesis_gecko.axf "./CMSIS/EZR32LG/startup_gcc_ezr32lg.o" "./CMSIS/EZR32LG/system_ezr32lg.o" "./Drivers/BMP280.o" "./Drivers/DHT22.o" "./Drivers/FH1750.o" "./Drivers/display.o" "./Drivers/displayls013b7dh03.o" "./Drivers/displaypalemlib.o" "./Drivers/dmactrl.o" "./Drivers/dmadrv.o" "./Drivers/ezradio_api_lib.o" "./Drivers/ezradio_auto_ack_plugin.o" "./Drivers/ezradio_comm.o" "./Drivers/ezradio_crcerror_plugin.o" "./Drivers/ezradio_direct_receive_plugin.o" "./Drivers/ezradio_direct_transmit_plugin.o" "./Drivers/ezradio_hal.o" "./Drivers/ezradio_plugin_manager.o" "./Drivers/ezradio_pn9_plugin.o" "./Drivers/ezradio_receive_plugin.o" "./Drivers/ezradio_transmit_plugin.o" "./Drivers/ezradio_unmod_carrier_plugin.o" "./Drivers/gpiointerrupt.o" "./Drivers/retargetio.o" "./Drivers/retargettextdisplay.o" "./Drivers/rtcdriver.o" "./Drivers/spidrv.o" "./Drivers/textdisplay.o" "./Drivers/udelay.o" "./Drivers/ustimer.o" "./Drivers/si4x6x/ezradio_api_lib_add.o" "./emlib/em_assert.o" "./emlib/em_cmu.o" "./emlib/em_core.o" "./emlib/em_dma.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_pcnt.o" "./emlib/em_prs.o" "./emlib/em_rtc.o" "./emlib/em_system.o" "./emlib/em_timer.o" "./emlib/em_usart.o" "./src/main.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: thesis_gecko.hex'
	arm-none-eabi-objcopy -O ihex "thesis_gecko.axf" "thesis_gecko.hex"
	@echo ' '

	@echo 'Building bin file: thesis_gecko.bin'
	arm-none-eabi-objcopy -O binary "thesis_gecko.axf" "thesis_gecko.bin"
	@echo ' '

	@echo 'Building s37 file: thesis_gecko.s37'
	arm-none-eabi-objcopy -O srec "thesis_gecko.axf" "thesis_gecko.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "thesis_gecko.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) thesis_gecko.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
