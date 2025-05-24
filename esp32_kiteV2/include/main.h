#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "Adafruit_HX711.h"
// #include <Adafruit_MCP23X17.h>
#include <MCP23017.h>

#define BUZZZER_PIN  33 // ESP32 pin GPIO18 connected to piezo buzzer
#define SWITCH_PIN 39  // GPIO39 (VN)
#define GPS_PPS 35
#define INT_A 34

// on GPIO extender
#define STATUS_LED 0 // GPA0 // red
#define LOGGING_LED 1 // GPA1  // yellow
#define LBO GPA2 2 // GPA2
#define GPS_FIX 3 // GPA3
#define GPS_EN 4 // GPA4 
#define RESET_IMU 5 // GPA5
#define SPARE_BUTTON 6 // GPA6 


int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

// Wi-Fi Configuration
const char* ssid = "Kite"; // Wi-Fi name
const char* password = "12345678"; // Wi-Fi password

// HTTP Server
WebServer server(80);

// SD Card
File dataFile;
const char* filePrefix = "imu_log_";
char currentFileName[16];

// IMU Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Pin extender
MCP23017 mcp = MCP23017(0x20);  // default adress

// GPS

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial GPS_Serial(2);

// #define GPSSerial Serial2
// Adafruit_GPS GPS(&Serial2);
// #define GPSECHO false
//uint32_t timer = millis();

// Définir les broches pour chaque HX711
const int32_t CALIBRATION = 13;

// Arrière droit
const uint8_t DATA_PIN1 = 25;
const uint8_t CLOCK_PIN1 = 26;

// Arrière gauche, n°4 on schematics
const uint8_t DATA_PIN2 = 13;
const uint8_t CLOCK_PIN2 = 12;

//Arrière centre
const uint8_t DATA_PIN3 = 27;
const uint8_t CLOCK_PIN3 = 14;

//Avant Centre, n°2 on schematics
const uint8_t DATA_PIN4 = 36;
const uint8_t CLOCK_PIN4 = 15;

//Avant Droit
const uint8_t DATA_PIN5 = 4;
const uint8_t CLOCK_PIN5 = 0;

//Avant Gauche
const uint8_t DATA_PIN6 = 32;
const uint8_t CLOCK_PIN6 = 2;

// Instances des modules HX711
Adafruit_HX711 hx711_1(DATA_PIN1, CLOCK_PIN1);  
Adafruit_HX711 hx711_2(DATA_PIN2, CLOCK_PIN2);
Adafruit_HX711 hx711_3(DATA_PIN3, CLOCK_PIN3);
Adafruit_HX711 hx711_4(DATA_PIN4, CLOCK_PIN4);
Adafruit_HX711 hx711_5(DATA_PIN5, CLOCK_PIN5);
Adafruit_HX711 hx711_6(DATA_PIN6, CLOCK_PIN6);


struct {
  unsigned long cpu_timestamp;
  unsigned long last_pps;
  char timestamp[24];
  float latitude, longitude;
  float euler_x, euler_y, euler_z;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  int32_t weight1;
  int32_t weight2;
  int32_t weight3;
  int32_t weight4;
  int32_t weight5;
  int32_t weight6;
} imuData;


// State variables

bool loggingActive = false;  
unsigned long lastBeepTime = 0;
unsigned long lastLogLEDTime = 0;