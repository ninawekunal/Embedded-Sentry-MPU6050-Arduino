# Gesture-Recognition-ARM-Processor-MPU6050-Arduino
## This project was a part of the course ECE-GY-6483 - Real Time Embedded Systems(RTES).

**Technical Aspects:**
* **Board used:** Adafruit Feather M0 ESP8266 Huzzah
* **Accelerometer:** MPU6050
* **IDE:** Arduino IDE
* **Coding Language:** C
* **Processor:** ARM 

**Objectives:**
* Use the data collected from a single accelerometer and/or magnetometer to record a hand movement sequence as a means to generally “unlock” a resource.
* Recorded sequence must be saved on the microcontroller, using a “Record Key”
feature. (*I am using a serial monitor input for this. **NOTE:** The LED is continuously glowing when its recording*)
* User then must replicate the key sequence within sufficient tolerances to unlock
the resource. (*User must press detect pattern in the serial monitor*)
* A successful unlock must be indicated by a visual indication, such as an LED or
similar indicator. (*NOTE: The LED glows when it successfully detects the correct gesture and unlocks.*)

**Logic Used:**
* Using the MPU, I collected RAW data of the acceleration. (Ignored Gyroscope)
* Each movement will have 3 coordinates (i.e. aX, aY, aZ) 
* I am recording 10 gesture movements for better accuracy.
* Then normalizing the data based on linear algebra.
* And using this normalized data to make decision whether the entered gesture matches the saved gesture and how close the match is.

**Video Demonstration:** [Link](https://www.youtube.com/watch?v=0V10NohCUhY)
