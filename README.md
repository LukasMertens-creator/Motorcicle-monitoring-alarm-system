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


# Code
```c++
//**********************************************LIBRARY*******************************************
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#define BLYNK_PRINT Serial
#include "arduino_secrets.h"
#include <BlynkSimpleEsp32.h>

//Gyro MPU6050
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#define maxval 2
#define minval -2

//Oled display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Temperature BMP280
#include <Adafruit_BMP280.h>

//Temperature ADS1115
#include <Adafruit_ADS1015.h>

//Alarm
#include "EasyBuzzer.h"

//map
WidgetMap myMap(V3);

//**********************************************NETWORK*******************************************

char auth[] = "$$$$"; // Fill in your personal Blync code.

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

//************************************************************************************************
//pinMode
int system_status = 2;
int sirene = 4;
int LimitationFlag = 0;
int MotionOnFlag;

//Gyro
Adafruit_MPU6050 mpu;

//Oled display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

//temperature
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

//Temperature ADS1115
Adafruit_ADS1115 ads(0x48);
float temp = 0.0;
float temp1 = 0.0;

//blynk timer
BlynkTimer timer;

//***********************************************SETUP********************************************
void setup() {
  Serial.begin(115200);
  ads.begin();
  EasyBuzzer.setPin(sirene);
  delay(10);
  
  int index = 0;
  float lat = 51.5074;
  float lon = 0.1278;
  myMap.location(index, lat, lon, "value");
//........................................Testing sensors & Oled

  Serial.println("MPU6050 OLED demo");

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
     Serial.println(F("SSD1306 allocation failed"));
     for(;;);
  }
  Serial.println(F("BMP280 Sensor event test"));
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  
  display.display();
  delay(500); // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

    
//........................................WiFi & Blynk connection
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);
  int wifi_ctr = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("WiFi connected");  
  Blynk.begin(auth, ssid, pass, "server.wyns.it", 8081);
  timer.setInterval(10L, MotionSensorRead);
  timer.setInterval(100L, OledDisplay);
//........................................PinMode..................

  pinMode(system_status, OUTPUT);
  pinMode(sirene, OUTPUT);
}
  

//***************************************************LOOP********************************************

void loop() {
  Blynk.run();
  timer.run();
  EasyBuzzer.update();
}

//***************************************************Blynk programs**********************************

BLYNK_WRITE(V0)                                    // Virtual pin to control the system.
{
  MotionOnFlag = param.asInt();
  int pinValue = param.asInt();                    // assigning incoming value from pin V5 to a variable
  if (param.asInt()) {
    Serial.println("Security system is ON ");
    digitalWrite(system_status, HIGH); 
    }else { 
    Serial.println("Security system is OFF ");
    Blynk.virtualWrite(V14, "Bike alarm OFF");
    digitalWrite(system_status, LOW);
    digitalWrite(sirene, LOW);
    }
}

void LimitationFlagReset() {
  LimitationFlag = 0;                              // Reset limitation flag
}

void MotionSensorRead() { 
 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
      
  if (MotionOnFlag == 1 && LimitationFlag == 0) {  // Check if both ON and that limit flag is inactive
    LimitationFlag = 1; // Set time limit flag for notifications
    timer.setTimeout(5000L, LimitationFlagReset);  // Start 5 second timer for limiting flag reset = sirene time on
    if (a.acceleration.x > maxval || a.acceleration.x < minval && a.acceleration.y > maxval || a.acceleration.y  < minval) {
      Blynk.virtualWrite(V14, "Bike Movement Detected");
      Blynk.notify("BIKE BEING STOLEN!");
      digitalWrite(sirene, HIGH);
    }  else {
      Blynk.virtualWrite(V14, "Bike Secure");
      digitalWrite(sirene, LOW);
    }
  }
}

void OledDisplay(){
  //ADC1115
  int16_t adc0;
  adc0 = ads.readADC_SingleEnded(0);
  temp = (adc0 * 0.1875) / 1000; // convert ADC value into voltage
  temp1 = temp * 10;
  
  //BMP280
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");
  Blynk.virtualWrite(V1, temp_event.temperature);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);
  display.println("Temperature= ");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(90, 30);
  display.println(temp_event.temperature);
  
  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
  Blynk.virtualWrite(V2, pressure_event.pressure);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 40);
  display.println("Pressure: ");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(80, 40);
  display.println(pressure_event.pressure);
  
  Serial.print("Temp Motor: ");
  Serial.println(temp1, 7);
  Serial.println();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 50);
  display.println("Temp motor:");

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(90, 50);
  display.println(temp1);
  Blynk.virtualWrite(V3, temp1);

  if (temp1 > 22 && temp1 < 24){
    Blynk.virtualWrite(V6, "Bike motor is warming up");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Motor is warming up");
  } else if (temp1 > 24 && temp1 < 28){
    Blynk.virtualWrite(V6, "ready to race");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("ready to race");
  } else if (temp1 > 28){
    Blynk.virtualWrite(V6, "Motor overheating");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Motor overheating");
  } else {
    Blynk.virtualWrite(V6, "Bike motor is cold");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Motor is cold"); 
  }
    display.display();
    Serial.println();
    delay(200);
}
```




