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
const char* ssid = "YourWiFissid";  // Enter SSID here
const char* password = "WiFiPassword";  //Enter Password here

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

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
//#include <Seasmart.h>
#include <memory>
#include <N2kMessages.h>

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {130310L, // OutsideEnvironmental (obsolete!)
                                                  130311L, // EnvironmentalParameters (temperature/humidity/pressure)
                                                  130312L, // Temperature
                                                  130313L, // Humidity
                                                  130314L, // Pressure
                                                  130316L, // TemperatureExt
                                                  0
                                                 };
                                               
// Import required libraries
#ifdef ESP32
  #include <WiFi.h>
  #include <ESPAsyncWebServer.h>
#else
  #include <Arduino.h>
  #include <ESP8266WiFi.h>
  #include <Hash.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#endif

//ESP8266WebServer server(80);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <Adafruit_BMP280.h>
#include <string>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//declare global sensor data
float temperature = 18, humidity = 50, pressure = 1000, altitude = 0;
float OutTemp = 28, InTemp = 20, InHum = 45, OutHum = 55;


// sacn for I2C devices
void scan_I2C() {
  byte error, address;
  uint8_t ONE = 1;
  int nDevices;
  Serial.println("Scanning for I2C...");
  nDevices = 0;
  Wire.endTransmission();
  // for (address = 0x76; address < 0x77; address++ )
  for (address = 1; address < 128; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" !");
      nDevices++; // increment device count
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
    if (error == 0) {
      byte data1 = 0; byte error1=0;
      switch (address) {
        case 0x1D: 
          Serial.println(" ADXL345 digital accelerometer");
          break;
        case 0x1E:
          Serial.println(" HMC5883L 3-axis digital compass");
          break;
        case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
          Serial.println(" PCF8574 I/O expander");
          break;
        case 0x27:
          Serial.println(" LCD with I2C backpack");
          Serial.println(" PCF8574 I/O expander");
          break;
        case 0x38: case 0x39: case 0x3A: case 0x3B: case 0x3C: case 0x3D: case 0x3E:
          Serial.println(" PCF8574A I/O expander");
          break;
        case 0x3F:
          Serial.println(" LCD with I2C backpack");
          Serial.println(" PCF8574A I/O expander");
          break;
        case 0x40:
          Serial.println(" HTU21D digital humidity and temperature sensor");
          break;
        case 0x48: case 0x49: case 0x4A: case 0x4B: 
          Serial.println(" ADS1113, ADS1114, ADS1115, ADS1013, ADS1014, ADS1015");
          break;
        case 0x50: case 0x51: case 0x52: case 0x54: case 0x55: case 0x56: case 0x57:
          Serial.println(" AT24C32/64 Eeprom family");
          break;
        case 0x53:  
          Serial.println(" ADXL345 digital accelerometer");
          Serial.println(" or AT24C32/64 Eeprom family");
        case 0x68:
          Serial.println(" DS3231 or DS1307 Real Time Clock");
          Serial.println(" or MPU9250 gyroscope, accelerometer, magnetometer");
          Serial.println(" or L3G4200D gyroscope");
          break;
        case 0x69: // same device also on 0x68
        // also need to study pass-through mode of MPU9250
          Serial.println(" MPU9250 gyroscope, accelerometer, magnetometer");
          Serial.println(" or L3G4200D gyroscope");
          break;
        case 0x76: case 0x77:
          Serial.println(" BMP280 or BME280 or BME680 or MS5607,MS5611,MS5637");
          // note: address 0x77 may be BMP085,BMA180 and may not be MS5607 or MS5637 CHECK
          Wire.beginTransmission(address);
          // Select register
          Wire.write(0xD0); // 0xD0 hex address of chip_id
          // Stop I2C Transmission
          Wire.endTransmission();
          // Request 1 bytes of data
          Wire.requestFrom(address, ONE);
          // Read 1 byte of data
          if (Wire.available() == 1)  {
            data1 = Wire.read();
          } // end of if (Wire.available() == 3)
          Serial.print ("Device ID=");
          Serial.print(data1, HEX);
          if (data1 == 0x58) Serial.println(" = BMP280");
          else if (data1 == 0x60) Serial.println(" = BME280");
          else if (data1 == 0x61) Serial.println(" = BME680");
          else Serial.println(" ID not in list");
          break;
        default:
          Serial.println("device not in list");
          break;
      }
    }
  } // end of for (address = 1; address < 128; address++ )

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
} // end of void scan_I2C()

//==========================the HTML code====================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=UTF-8"/>
  <title>BME280->NMEA2000</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {
     font-family: Arial;
     display: inline-block;
     margin: 0px auto;
     text-align: center;
    }
  </style>
</head>
<body>
  <p>Inside: 
    <span class="ds-labels"> </span> 
    <span id="intemp">%INTEMP%</span>
    <sup class="units"></sup>&deg;C
  </p>  
  <p>Outside: 
    <span class="ds-labels"> </span> 
    <span id="outtemp">%OUTTEMP%</span>
    <sup class="units"></sup>&deg;C
  </p>
  <p>Pressure: 
    <span class="ds-labels"> </span>
    <span id="pressure">%PRESSURE%</span>
    <sup class="units"></sup>mBar
  </p>
  <p>Inside humidity: 
    <span class="ds-labels"> </span>
    <span id="inhum">%INHUM%</span>
    <sup class="units"></sup>&#037;
  </p>
  <p>Outside humidity: 
    <span class="ds-labels"> </span> 
    <span id="outhum">%OUTHUM%</span> 
    <sup class="units"></sup>&#037;
  </p>
  <h6 align="right">BME280-&gt;N2K ©2020 sekom.com</h6>
</body>
<script>
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("intemp").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/intemp", true);
  xhttp.send();
}, 10000) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("outtemp").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/outtemp", true);
  xhttp.send();
}, 10000) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("pressure").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/pressure", true);
  xhttp.send();
}, 10000) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("inhum").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/inhum", true);
  xhttp.send();
}, 10000) ;
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("outhum").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/outhum", true);
  xhttp.send();
}, 10000) ;
</script>
</html>)rawliteral";

// Replaces placeholder with  values
String processor(const String& var){
  //Serial.println(var);
  if(var == "INTEMP"){
    return String(InTemp);
  }  
  else if(var == "OUTTEMP"){
    return String(OutTemp);
  }
  else if(var == "PRESSURE"){
    return String(pressure);
  }
  else if(var == "INHUM"){
    return String(InHum);
  }
  else if(var == "OUTHUM"){
    return String(OutHum);
  }
  return String();
}

//============================= send temperature/humidity/pressure to NMEA2000 ===================================

void SendN2kTempHumPress(double intemp, double outtemp, double inhum, double outhum, double pressure) {
  tN2kMsg N2kMsg;

    // Select the right PGN for your MFD and set the PGN value also in "TransmitMessages[]"

   SetN2kEnvironmentalParameters(N2kMsg, 0, N2kts_OutsideTemperature, CToKelvin(outtemp),           // PGN130311, uncomment the PGN to be used 
                    N2khs_OutsideHumidity, outhum, pressure);
  NMEA2000.SendMsg(N2kMsg);
  
  SetN2kTemperature(N2kMsg, 0, 1, N2kts_InsideTemperature, CToKelvin(intemp), N2kDoubleNA) ;
  NMEA2000.SendMsg(N2kMsg);

  SetN2kHumidity(N2kMsg, 0, 1, N2khs_InsideHumidity, inhum, N2kDoubleNA) ;
  NMEA2000.SendMsg(N2kMsg);

  SetN2kPressure(N2kMsg, 0, 1, N2kps_Atmospheric, hPAToPascal(pressure));
  NMEA2000.SendMsg(N2kMsg);
}
int switchOn() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(SWITCH); Serial.print(buttonState);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(LED, LOW); 
  } else {
    // turn LED off:
    digitalWrite(LED, HIGH); 
  }
  return buttonState;
}
//========================SETUP=================================================

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println("\n====================\nSEKOM - THP-01 v1.1\n====================\nCombined inside/outside temperature, humidity and pressure monitor\n");
  Serial.println("\nhttp://....ip.../intemp; /outtemp; /pressure; /humidity\n");
  
  pinMode(LED, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED, LOW);   // Turn the LED on by making the voltage LOW 
  pinMode(SWITCH, INPUT);   // initialize the switch pin as an input:
  delay(1000);

#ifdef DS18B20_CONNECTED
  // Start the DS18B20 sensor
  sensors.begin();
#endif  
  Wire.begin(); // initialise I2C protocol
  
#ifdef DHT_CONNECTED
    dht.begin(); //start DHT11
#endif
  
  scan_I2C();  //scan for sensors
  bme.begin(0x76);
   
 if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    // while (1) delay(10); //we don't want to stop here even if it is not working
 }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
  
  if( switchOn() == HIGH ){  
   // Connect to Wi-Fi
   WiFi.begin(ssid, password);
   Serial.print("BME WiFi connecting to: ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
    }
  
    // Print ESP Local IP Address
    Serial.print("\nConnected IP:");
   Serial.println(WiFi.localIP());
 
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/intemp", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send_P(200, "text/plain", String(InTemp).c_str() );
    });  
    server.on("/outtemp", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send_P(200, "text/plain", String(OutTemp).c_str() );
    });
    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send_P(200, "text/plain", String(pressure).c_str());
    });
    server.on("/inhum", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send_P(200, "text/plain", String(InHum).c_str());
    });  
    server.on("/outhum", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send_P(200, "text/plain", String(OutHum).c_str());
    });
  
    // Start server
   server.begin();  
  }  
  
  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 101, // Manufacturer's product code
                                 "BME280-N2K",  // Manufacturer's Model ID
                                 "1.0.0.1 (2020-03-07)",  // Manufacturer's Software version code
                                 "1.0.0.1 (2020-03-07)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(1001, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2804 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 32);
//  NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);     // Uncomment this, so you can test code without CAN bus chips on Arduino Mega

  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.Open();
  Serial.println("\nNMEA2000 CAN BUS initialized\n");
}
 
void loop(){
  
  sensors_event_t temp_event, pressure_event, humidity_event;
  
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  humidity = humidity_event.relative_humidity;
  
#ifdef DS18B20_CONNECTED  
  sensors.requestTemperatures(); 
  OutTemp = sensors.getTempCByIndex(0);
  OutHum = humidity; //there is no humidity sensor outside
#endif

#ifdef DHT_CONNECTED
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  OutHum = dht.readHumidity();
  // Read temperature as Celsius (the default)
  OutTemp = dht.readTemperature();
  InHum = humidity; //is the inside humidity
#endif

  InTemp = temp_event.temperature;
  pressure = pressure_event.pressure;

  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  SendN2kTempHumPress(InTemp, OutTemp, InHum, OutHum, hPAToPascal(pressure)); //send data to NMEA2000
  
  Serial.print("Pressure = ");
  Serial.print(pressure,2); // print with 2 decimal places
  Serial.print(" hPa,  Temperature (in/outside) = ");
  Serial.print(InTemp,1); // print with 1 decimal places
  Serial.print(" / ");
  Serial.print(OutTemp,1);
  Serial.print( " ˚C,   Humidity (in/outside) = ");
  Serial.print(InHum,1); // print with 1 decimal places
  Serial.print( " / ");
  Serial.print(OutHum,1);
  Serial.println(" %");
  delay(5000);
}
