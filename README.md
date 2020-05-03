STM32 based soldering station for JBC cartridge types

Introduction:

This code was written for the “Aliosensu STM32 JBC Controller v1.0” 
It features an SMT32F103C8T6 microcontroller, and uses the STM32duino core available here: https://github.com/stm32duino/Arduino_Core_STM32

Libraries used:

    • Display 	: https://github.com/Bodmer/TFT_eSPI
    • EEPROM	: https://github.com/stm32duino/Arduino_Core_STM32/tree/master/libraries/EEPROM 
    • Wire		: https://github.com/stm32duino/Arduino_Core_STM32/tree/master/libraries/Wire 


The design requires isolated power for the digital and heater sections. This is most easily accomplished with separate transformer windings, but an isolated SMPS, or separate transformers can also be used.
The design will accommodate a wide variety of transformer power and voltage specifications. 
Heater power should fall within 70-200 watts.
Heater voltage should fall within 25-50 volts. 
Digital side voltage should not exceed 25v AC when using on board linear regulation. 

Separate file “build_opt.h” required in sketch directory only when using large value NTC thermistor voltage divider for cold junction compensation. (100K or greater thermistor value)

Features of the controller are:

    • Interrupt based zero crossing detection
    • Zero volt power switching
    • Interrupt based rotary encoder control
    • Op-amp based amplification with cold junction compensation for thermocouple reading
    • 300µs ISR for heater management
    • Settable tip temp (EEPROM)
    • Tip temp calibration (EEPROM)
    • Potentiometer based transformer power throttling calibration (to accommodate a larger array of transformers)
    • U8G2 display library is currently used with a SSD1306_128X64 and I2C bus
	
Notes:
The STM32F103C8T6 makes for a great controller for such a station as it supports several interrupt lines required for zero crossing detection, as well as accurate rotary encoder control.
It’s speed is also beneficial for a quick ISR in order to successfully determine whether to switch power during a zero crossing event.  The speed is also helpful in running displays which are require heavier processing, such as the readily available OLED displays online.

A big thank you to Stmicroelectronics and other contributors who support the STM32duino core for the Arduino interface, making their controllers readily accessible to a large audience.
