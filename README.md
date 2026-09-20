# STM32 Real-Time Control and Positioning

STM32F103 firmware projects developed for an engineering-training competition robot. The repository contains a real-time motion-control program and a separate VL53L1X ranging/positioning program.

## Projects

- `工训 实时 - 第二圈/` - Main real-time robot control firmware. It combines motor control, encoders, PID loops, inertial sensing, serial communication, and competition-specific motion logic.
- `stm32-positioning/` - STM32F103 positioning firmware using the VL53L1X time-of-flight sensor, encoder feedback, PID control, and common peripherals such as UART, SPI, CAN, PWM, and OLED output.

## Hardware and tools

- MCU: STM32F103 (high-density device configuration)
- Toolchain: Keil MDK / Arm Compiler
- Project files: `USER/Template.uvprojx` in each project directory

## Build

1. Open the relevant `Template.uvprojx` file in Keil MDK.
2. Check the target device, clock, and board wiring before building.
3. Build and download the firmware with a supported ST-Link or compatible debugger.

## Notes

The code is provided as competition-project source. Pin assignments, sensor addresses, PID parameters, and motor polarity depend on the original hardware and may need adjustment for another board.

The STM32 standard peripheral library and the VL53L1X driver are included in the project tree. Their original license and attribution terms apply.
