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
Wire | Wire.h | https://github.com/esp8266/Arduino/tree/master/libraries/Wire
SPI | SPI.h | https://github.com/esp8266/Arduino/tree/master/libraries/SPI 
EasyBuzzer | EasyBuzzer.h | https://github.com/evert-arias/EasyBuzzer

## Components

Microcontroller | esp32 |![esp](https://user-images.githubusercontent.com/61006702/77225982-b6794280-6b74-11ea-89eb-ba2c3c65d996.jpg)

ESP32 is a series of low-cost, low-power system on a chip microcontrollers with integrated Wi-Fi and dual-mode Bluetooth. The ESP32 series employs a Tensilica Xtensa LX6 microprocessor in both dual-core and single-core variations and includes built-in antenna switches, RF balun, power amplifier, low-noise receive amplifier, filters, and power-management modules. ESP32 is created and developed by Espressif Systems, a Shanghai-based Chinese company, and is manufactured by TSMC using their 40 nm process.[2] It is a successor to the ESP8266 microcontroller.

---
Display | Oled | ![oled](https://user-images.githubusercontent.com/61006702/77226007-e6c0e100-6b74-11ea-9dff-47438e23a81a.jpg)

Because the OLED display uses I2C communication protocol, wiring is very simple. You can use the following table as a reference.

Pin | ESP32 
------|------
Vin	| 3.3V
GND	| GND
SCL	| GPIO 22
SDA	| GPIO 21

---

Sensor | BMP280 | ![bmp280](https://user-images.githubusercontent.com/61006702/77226020-225bab00-6b75-11ea-9e40-b6a22e855fad.jpg)

This precision sensor from Bosch is the best low-cost sensing solution for measuring barometric pressure and temperature. Because pressure changes with altitude you can also use it as an altimeter!

Pin | ESP32 
------|------
Vcc	| 3.3V
GND	| GND
SCL	| GPIO 22
SDA	| GPIO 21

---

Sensor | MPU6050 | ![mpu6050](https://user-images.githubusercontent.com/61006702/77226033-461ef100-6b75-11ea-9bd4-dd8045d6a0cc.jpg)

The InvenSense MPU-6050 sensor contains a MEMS accelerometer and a MEMS gyro in a single chip. It is very accurate, as it contains 16-bits analog to digital conversion hardware for each channel. Therefor it captures the x, y, and z channel at the same time. The sensor uses the I2C-bus to interface with the Arduino.

The MPU-6050 is not expensive, especially given the fact that it combines both an accelerometer and a gyro.

Pin | ESP32 
------|------
Vcc	| 3.3V
GND	| GND
SCL	| GPIO 22
SDA	| GPIO 21

---

ADC | ADC1115 |![adc1115](https://user-images.githubusercontent.com/61006702/77226066-a57d0100-6b75-11ea-89c8-86cf84a6cb76.jpg)

The ADS1115 is a 16 bit Analog Digital Converter that can greatly improve your Arduino resolution and measurement accuracy.   It has four input channels that can be configured for Single Ended, Differential or Comparator Measurements.

Pin | ESP32 
------|------
VDD	| 3.3V
GND	| GND
SCL	| GPIO 22
SDA	| GPIO 21
ADDR | GND

---

Sensor | KTY81 |![KTY81](https://user-images.githubusercontent.com/61006702/77226101-fab91280-6b75-11ea-9794-d01e3e729a7c.jpg)

---
### And other small components:

pcs. | component 
------|------
1 | elco 220uF
2 | elco 10uF
2 | capacitor 0.1uF
1 | potentiometer 10K
1 | resistor 10 Ohm
1 | resistor 1K Ohm
1 | resistor 1K2 Ohm

## schematic

![schema motoalarm](https://user-images.githubusercontent.com/61006702/77251212-69b56a80-6c4d-11ea-93f9-a59be5d3f1ce.png)

### For the LM386 integrated power amplifier is a voltage of 9-12VDC necessary.
---
## Breadboard photo

![20200322_154613](https://user-images.githubusercontent.com/61006702/77252532-aa64b200-6c54-11ea-9f73-1e9f690c8f86.jpg)

---
## 3Dprinted case and Oled barholder
For this project I designed a case for the esp32, sensors, the siren circuit and for the OLED display a holder that can be mounted to the steering bar.
![bike alarm case](https://user-images.githubusercontent.com/61006702/77534196-7e397300-6e98-11ea-94be-a49eefc58169.png)

If you have 3D printed this design it looks like that.

![20200325_131056](https://user-images.githubusercontent.com/61006702/77535283-a9bd5d00-6e9a-11ea-9ac7-4715f0cc03a5.jpg)

### The red LED indicates that the esp32 has voltage. The blue LED indicates that the Alarm is active

![20200325_131134](https://user-images.githubusercontent.com/61006702/77535532-0587e600-6e9b-11ea-98bb-08fa405e3e34.jpg)


---
## Blynk app layout

The Alarm control page is where you can monitor the bike status and see on a mep where the bike is.
On this page you can also control the alarm and the siren with one press on your screen.

![Screenshot_20200325-122040_Blynk](https://user-images.githubusercontent.com/61006702/77532093-77a8fc80-6e94-11ea-83f1-1e8fd6490aac.jpg)

### On the second tab ( Monitoring temperature & humidity),
you can see the temperature, pressure, acceleration and gyro in a graph.
The temperatures and the status of the motor can also be seen on the OLED display.

![Screenshot_20200325-121942_Blynk](https://user-images.githubusercontent.com/61006702/77531943-1b45dd00-6e94-11ea-9435-631db3795387.jpg)

When detection is complete, the siren sounds and you receive a notification.

![Screenshot_20200325-122004_Blynk](https://user-images.githubusercontent.com/61006702/77532730-c3a87100-6e95-11ea-82f1-0079fce6b0a4.jpg)

---

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
#define maxval 0.5
#define minval -0.5
#define maxval1 0.2
#define minval1 -0.2

//Oled display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Temperature BMP280
#include <Adafruit_BMP280.h>

//Temperature ADS1115
#include <Adafruit_ADS1015.h>

//Alarm
#include "EasyBuzzer.h"
unsigned int frequency = 1000;
unsigned int beeps = 10;

//map
WidgetMap myMap(V3);

//**********************************************NETWORK*******************************************

char auth[] = "Jx-OJlhEJwCTkevLqW8uQ3ywjO05dA3V"; // Fill in your personal Blync code.

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
  int pinValue = param.asInt();                    // assigning incoming value from pin V0 to a variable
  if (param.asInt()) {
    Serial.println("Security system is ON ");
    digitalWrite(system_status, HIGH); 
 }else { 
    Serial.println("Security system is OFF ");
    Blynk.virtualWrite(V14, "Bike alarm OFF");
    digitalWrite(system_status, LOW);
    digitalWrite(sirene, LOW);
    EasyBuzzer.stopBeep();
    }
}

BLYNK_WRITE(V7)                                    // Virtual pin to control the system.
{
  int pinValue2 = param.asInt();                    // assigning incoming value from pin V0 to a variable
     if (param.asInt()) {
       EasyBuzzer.beep(
       frequency,  // Frequency in Hertz(HZ).
       beeps // The number of beeps.
       );
    }else {
    EasyBuzzer.stopBeep();
    }
  }
void LimitationFlagReset() {
  LimitationFlag = 0;                              // Reset limitation flag
}


void MotionSensorRead() { 
 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Blynk.virtualWrite(V8, a.acceleration.x);
  Blynk.virtualWrite(V9, a.acceleration.y);
  Blynk.virtualWrite(V10, a.acceleration.z);
  Blynk.virtualWrite(V11, g.gyro.x);
  Blynk.virtualWrite(V12, g.gyro.y);
  Blynk.virtualWrite(V13, g.gyro.z);
  if (MotionOnFlag == 1 && LimitationFlag == 0) {  // Check if both ON and that limit flag is inactive
    LimitationFlag = 1; // Set time limit flag for notifications
    timer.setTimeout(5000L, LimitationFlagReset);  // Start 5 second timer for limiting flag reset = sirene time on
    if (a.acceleration.x > maxval || a.acceleration.x < minval && a.acceleration.y > maxval || a.acceleration.y  < minval || g.gyro.x > maxval1 || g.gyro.x  < minval1 || g.gyro.y > maxval1 || g.gyro.y  < minval1) {
      Blynk.virtualWrite(V14, "Bike Movement Detected");
      Blynk.notify("BIKE BEING STOLEN!");
      EasyBuzzer.beep(
      frequency,  // Frequency in Hertz(HZ).
      beeps   // The number of beeps.
          );
   }  else {
      Blynk.virtualWrite(V14, "Bike Secure");
      digitalWrite(sirene, LOW);
      EasyBuzzer.stopBeep();
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

  if (temp1 > 22 && temp1 < 23){
    Blynk.virtualWrite(V6, "Bike motor is warming up");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Motor is warming up");
  } else if (temp1 > 23 && temp1 < 24){
    Blynk.virtualWrite(V6, "ready to race");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("ready to race");
  } else if (temp1 > 24){
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




