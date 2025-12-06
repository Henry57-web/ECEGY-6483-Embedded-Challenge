# ECEGY-6483-Embedded-Challenge
This project detects three movement patterns using the on-board LSM6DSL IMU on the STM32L475VG Discovery board:

- Tremor  
- Dyskinesia  
- Freezing of gait (FOG)  

The program collects accelerometer data, performs FFT in real time, and lights different LEDs depending on the detected motion pattern.

---

## 🖥 Hardware

- STM32L475VG Discovery board  
- LSM6DSL IMU (built-in)
- On-board LEDs (LED1, LED2, LED3)

---

## 🚀 How to Run

### PlatformIO
1. Open folder in VSCode  
2. Install PlatformIO extension  
3. Connect board via USB  
4. Click **Upload**  
5. Click **Monitor** to view serial output  

---



## 🔔 LED Behavior

| LED | Meaning |
|---|---|
| LED1 | Tremor detected |
| LED2 | Dyskinesia detected |
| LED3 | Freezing of gait detected |

---


