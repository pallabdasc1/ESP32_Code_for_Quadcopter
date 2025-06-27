/*

===================================================================================

🚀 Open Source Quadcopter Flight Controller – ESP32 Port

===================================================================================



📜 License & Acknowledgement:

This code is inspired and adapted from the **Carbon Aeronautics Flight Controller Manual**, originally designed for the **Teensy 4.0** microcontroller. The core logic and architecture were retained while porting the implementation to the **ESP32-WROOM-DevKit** platform using the **Arduino framework**.



This project was created for **educational and experimental purposes only**. All credits to Carbon Aeronautics for their excellent open-source initiative. I do not intend to use any part of this code for commercial applications.



📡 Key Components:

- **ESP32-WROOM-DevKit v3**

- **MPU6050 IMU Sensor** (I2C)

- **FlySky PPM Receiver** (decoded via `PPMReader` library)

- **FreeRTOS Tasks** for flight control and battery monitoring

- Dual-loop **PID control** for roll, pitch, and yaw stabilization

- PWM motor control using `ledcWrite()` for ESCs



💡 License:

This code is released under the **MIT License**.

You are free to **fork**, **modify**, and **redistribute** this project for personal or educational use.



Please keep this notice intact and credit the original and ported sources.



📧 Contact:

Author of this ESP32 port: **Pallab Das**


===================================================================================



*/