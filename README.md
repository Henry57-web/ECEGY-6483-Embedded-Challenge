# ECEGY-6483-Embedded-Challenge
This program uses the LSM6DSL accelerometer on the STM32 Discovery board to detect three motion states using FFT analysis:

Tremor

Dyskinesia

Freezing of gait (FOG)

The program continuously collects accelerometer data, performs FFT, and lights different LEDs depending on the detected motion pattern.

🔧 Hardware

STM32L475VG Discovery board

LSM6DSL IMU (built-in)

LEDs (LED1, LED2, LED3 on board)

▶ How to run
Option 1 — PlatformIO

Open the project folder in VSCode

Install PlatformIO extension

Connect the board by USB

Click “Upload”

Click “Monitor” to see output
