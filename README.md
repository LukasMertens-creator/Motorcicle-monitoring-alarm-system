# Motorcicle-monitoring-alarm-system
![download](https://user-images.githubusercontent.com/61006702/77225272-eec95280-6b6d-11ea-8473-87b95f04ba34.jpg)

---
## Warning 
--- 
This project is made with an esp32 with ONLY a WiFi connection, if you want to use this project on a location where is no WiFi than you need a esp32 with 2G connection. (TTGO T-Call ESP32 SIM800L)
You will need to follow the instuctions (https://randomnerdtutorials.com/esp32-sim800l-publish-data-to-cloud/ ) and modify the code.

![ESP32-SIM800L-TTGO-T-Call-GPS-GPRS-f](https://user-images.githubusercontent.com/61006702/77245293-c4cf6900-6c1d-11ea-956c-971cef7f680f.jpg)

---

## description
### -MONITORING-
Monitor the outdoor temperature & humidity around your motorcycle and your motor temperature.
All these parameters are visible on an Oled display that you attached to your steering bar and on the app Blynk.
These values are also tracked and plotted in the Blynk app.

### -ALARM SYSTEM-
The alarm system can be monitored and controlled via the Blynk app. It has a gyro sensor that triggers the alarm if the moto moves while the alarm is on. When there is an alarm, the Blynk app sends a push message to notify you. You can see the location of the moto on a map in the app.

## Libraries and Resources

Title | Include | Link 
------|---------|------
Adafruit_SSD1306 | Adafruit_SSD1306.h | https://github.com/adafruit/Adafruit_SSD1306 
Adafruit-GFX-Library | Adafruit_GFX.h | https://github.com/adafruit/Adafruit-GFX-Library 
Adafruit-ADS1015 | Adafruit_ADS1015.h | https://github.com/adafruit/Adafruit_ADS1X15
Adafruit-BMP280 | Adafruit_BMP280.h | https://github.com/adafruit/Adafruit_BMP280_Library
Adafruit-Sensor | Adafruit_Sensor.h | https://github.com/adafruit/Adafruit_Sensor
Adafruit-MPU6050 | Adafruit_MPU6050.h | https://github.com/adafruit/Adafruit_MPU6050
Blynk | BlynkSimpleEsp32.h | https://github.com/blynkkk/blynk-library/blob/master/src/BlynkSimpleEsp32.h
Wifi | WiFi.h | https://github.com/arduino-libraries/WiFi
WifiClient | WiFiClient.h | https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiClient.h
Wire | Wire.h | https://github.com/esp8266/Arduino 
SPI | SPI.h | https://github.com/esp8266/Arduino 
EasyBuzzer | EasyBuzzer.h | https://github.com/evert-arias/EasyBuzzer

## Components

Microcontroller | esp32 |![esp](https://user-images.githubusercontent.com/61006702/77225982-b6794280-6b74-11ea-89eb-ba2c3c65d996.jpg)

Display | Oled | ![oled](https://user-images.githubusercontent.com/61006702/77226007-e6c0e100-6b74-11ea-9dff-47438e23a81a.jpg)

Sensor | BMP280 | ![bmp280](https://user-images.githubusercontent.com/61006702/77226020-225bab00-6b75-11ea-9e40-b6a22e855fad.jpg)

Sensor | MPU6050 | ![mpu6050](https://user-images.githubusercontent.com/61006702/77226033-461ef100-6b75-11ea-9bd4-dd8045d6a0cc.jpg)

ADC | ADC1115 |![adc1115](https://user-images.githubusercontent.com/61006702/77226066-a57d0100-6b75-11ea-89c8-86cf84a6cb76.jpg)

Sensor | KTY81 |![KTY81](https://user-images.githubusercontent.com/61006702/77226101-fab91280-6b75-11ea-9794-d01e3e729a7c.jpg)






