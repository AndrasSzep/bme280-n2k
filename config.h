/*********
  - ESP-WROOM-32 Bluetooth and WIFI Dual Core CPU with Low Power Consumption
    https://www.aliexpress.com/item/32864722159.html?spm=a2g0s.9042311.0.0.7ac14c4dv0nB1k
  - GY-BME280-3.3 precision altimeter atmospheric pressure BME280 sensor module
    https://www.aliexpress.com/item/32862445164.html?spm=a2g0s.9042311.0.0.7ac14c4dv0nB1k
  - DS18b20 Waterproof digital temperature sensor for external temperature
    https://www.ebay.co.uk/itm/1pc-New-DS18b20-Waterproof-digital-temperature-sensor-probe-Length-100cm/253644294739?hash=item3b0e60ca53:g:clgAAOSwSSNbn14~
  - SN65HVD230 CAN bus transceiver
    https://www.aliexpress.com/item/32686393467.html?spm=a2g0s.9042311.0.0.7ac14c4dv0nB1k
   
  by © SEKOM.com - Dr. András Szép 2020 using open source libraries v1.0
==================================================================================
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
===================================================================================

  ESP32 WROOM w BME280 to NMEA2000 and webserver for temp&pressure&humidity
  
  BME280 sensor connections:
  3.3v = VCC = SDO 
  GND  = CSB
  SCL =  GPIO5 on NodeMCU (D1) and SCL (GPIO22) on ESP32
  SDA =  GPIO4 on NodeMCU (D2) and SDA (GPIO21) on ESP32
  
  CAN bus connections TX2 - GPIO17
                      RX2 - GPIO16
 
********/

// Replace with your network credentials 
const char* ssid = "DrayTek";  // Enter SSID here
const char* password = "medved28";  //Enter Password here

// CAN BUS
#define ESP32_CAN_TX_PIN GPIO_NUM_17  //TX2 Set CAN TX GPIO15 =  D8
#define ESP32_CAN_RX_PIN GPIO_NUM_16  //RX2 Set CAN RX GPIO13 =  D7

#define LED 5    // builtin LED GPIO05
#define SWITCH 19   // ON/Off Switch for WIFI ~GPIO19
int buttonState = 0;         // variable for reading the pushbutton status

// which external temperature sensor is connected
//#define DS18B20_CONNECTED
#define DHT_CONNECTED

#ifdef DS18B20_CONNECTED
  #include <OneWire.h>
  #include <DallasTemperature.h>
  // GPIO where the DS18B20 is connected to
  const int oneWireBus = 4;     //GPIO4
  OneWire oneWire(oneWireBus);    // Setup a oneWire instance to communicate with any OneWire devices
  DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature sensor 
#endif

#ifdef DHT_CONNECTED
  #include "DHT.h"
  #define DHTPIN 4  //use insteasd of the failed DS18B20
  // Uncomment whatever type you're using!
  #define DHTTYPE DHT11   // DHT 11
  //#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  //#define DHTTYPE DHT21   // DHT 21 (AM2301)
  DHT dht(DHTPIN, DHTTYPE);
#endif

#define MQTT

#ifdef MQTT
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "sekom"
#define AIO_KEY         "aio_wKUC92mzxkfuwhfGNjLUuB01HqSe"
#endif
