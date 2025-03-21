#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Arduino.h>
#include "Adafruit_HX711.h"

#include "pitches.h"
#define BUZZZER_PIN  33 // ESP32 pin GPIO18 connected to piezo buzzer
#define SWITCH_PIN 39  // GPIO39 (VN)

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

// GPS
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();

// Définir les broches pour chaque HX711
const int32_t CALIBRATION = 13;

// Arrière droit
const uint8_t DATA_PIN1 = 25;
const uint8_t CLOCK_PIN1 = 26;

// Arrière gauche
const uint8_t DATA_PIN2 = 13;
const uint8_t CLOCK_PIN2 = 12;

//Arrière centre
const uint8_t DATA_PIN3 = 27;
const uint8_t CLOCK_PIN3 = 14;

//Avant Centre
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


// State variables
bool switchState = false;    
bool lastSwitchReading = true;  
bool loggingActive = false;  
unsigned long lastBeepTime = 0;

// Function to List Files on SD Card
String listFiles(fs::FS &fs, const char* dirname) {
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) {
    return "Cannot open directory.";
  }

  String html = "<h1>Files</h1><ul>";
  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    html += "<li>" + fileName;
    html += " <a href=\"/download?file=" + fileName + "\">[Download]</a>";
    html += " <a href=\"/delete?file=" + fileName + "\" onclick=\"return confirm('Delete " + fileName + " ?')\">[Delete]</a>";
    html += "</li>";
    file = root.openNextFile();
  }
  html += "</ul>";
  html += "<button onclick=\"if(confirm('Delete all files?')) window.location.href='/delete_all'\">Delete All</button>";
  return html;
}

// HTTP Handlers
void handleRoot() {
  String html = listFiles(SD, "/");
  server.send(200, "text/html", html);
}

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Error: No file specified.");
    return;
  }

  String fileName = server.arg("file");
  fileName = "/" + fileName;
  File file = SD.open(fileName.c_str(), FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "Cannot open file.");
    return;
  }

  server.setContentLength(file.size());
  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendHeader("Content-Disposition", "attachment; filename=" + fileName);
  server.sendHeader("Connection", "close");
  server.streamFile(file, "application/octet-stream");
  file.close();
}

void handleDelete() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Error: No file specified.");
    return;
  }

  String fileName = server.arg("file");
  fileName = "/" + fileName;

  if (SD.exists(fileName.c_str())) {
    SD.remove(fileName.c_str());
  }
  server.send(200, "text/html", "<script>window.location.href='/'</script>");
}

void handleDeleteAll() {
  File root = SD.open("/");
  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    fileName = "/" + fileName;
    file.close();  // Close the current file before deleting
    SD.remove(fileName.c_str());
    file = root.openNextFile();
  }
  root.close();  // Close the root directory
  server.sendHeader("Location", "/");
  server.send(303);
}


void setupServer() {
  Serial.println("Setting up HTTP server...");
  server.on("/", handleRoot);
  server.on("/download", handleDownload);
  server.on("/delete", handleDelete);
  server.on("/delete_all", handleDeleteAll);

  Serial.println("Starting server...");
  server.begin();
  Serial.println("HTTP server started.");
}


// Initialize GPS
void setupGPS() {

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  Serial.println("GPS initialized.");
}

void createNewLogFile() {
  int fileIndex = 0;
  do {
    sprintf(currentFileName, "/%s%04d.bin", filePrefix, fileIndex++);
  } while (SD.exists(currentFileName));

  dataFile = SD.open(currentFileName, FILE_WRITE);
  if (dataFile) {
    Serial.print("Created new log file: ");
    Serial.println(currentFileName);
  } else {
    Serial.println("Failed to create log file!");
  }
}

void setupLoadCells() {
  // Initialisation de chaque HX711
  hx711_1.begin();
  hx711_2.begin();
  hx711_3.begin();
  hx711_4.begin();
  hx711_5.begin();
  hx711_6.begin();

  // Tare pour chaque module
  Serial.println("Tare des modules HX711...");
  
  for (uint8_t t = 0; t < 5; t++) {
    hx711_1.tareA(hx711_1.readChannelRaw(CHAN_A_GAIN_64));
    hx711_2.tareA(hx711_2.readChannelRaw(CHAN_A_GAIN_64));
    hx711_3.tareA(hx711_3.readChannelRaw(CHAN_A_GAIN_64));
    hx711_4.tareA(hx711_4.readChannelRaw(CHAN_A_GAIN_64));
    hx711_5.tareA(hx711_5.readChannelRaw(CHAN_A_GAIN_64));
    hx711_6.tareA(hx711_6.readChannelRaw(CHAN_A_GAIN_64));
  }

  Serial.println("Tare terminée !");
}

// Update GPS timestamp
String getGPSTimestamp() {
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return ""; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  char timestamp[24];
  sprintf(timestamp, "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
          GPS.year, GPS.month, GPS.day,
          GPS.hour, GPS.minute, GPS.seconds,
          GPS.milliseconds);

  return String(timestamp);
  /*
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 2); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 2); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
  }
  */
}


// IMU Data Logging
void logIMUData() { 
  sensors_event_t orientationData, angVelocityData, linearAccelData;

  // Get IMU data
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Read load cell data // ! -------------
  int32_t weight1 = hx711_1.readChannelBlocking(CHAN_A_GAIN_64)/CALIBRATION;
  int32_t weight2 = hx711_2.readChannelBlocking(CHAN_A_GAIN_64)/CALIBRATION;
  int32_t weight3 = hx711_3.readChannelBlocking(CHAN_A_GAIN_64)/CALIBRATION;
  int32_t weight4 = hx711_4.readChannelBlocking(CHAN_A_GAIN_64)/CALIBRATION;
  int32_t weight5 = hx711_5.readChannelBlocking(CHAN_A_GAIN_64)/CALIBRATION;
  int32_t weight6 = hx711_6.readChannelBlocking(CHAN_A_GAIN_64)/CALIBRATION;



  // Prepare data buffer
  struct {
    unsigned long cpu_timestamp;
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

  imuData.cpu_timestamp = millis();

  String gpsTimestamp = getGPSTimestamp();
  if (!gpsTimestamp.isEmpty()) {
    strncpy(imuData.timestamp, gpsTimestamp.c_str(), sizeof(imuData.timestamp));
  } else {
    strncpy(imuData.timestamp, "No timestamp", sizeof(imuData.timestamp));
  }

  if (GPS.fix) {
    imuData.latitude = GPS.latitude;
    imuData.longitude = GPS.longitude;
  } else {
    imuData.latitude = 0.0;
    imuData.longitude = 0.0;
  }

  imuData.euler_x = orientationData.orientation.x;
  imuData.euler_y = orientationData.orientation.y;
  imuData.euler_z = orientationData.orientation.z;
  imuData.gyro_x = angVelocityData.gyro.x;
  imuData.gyro_y = angVelocityData.gyro.y;
  imuData.gyro_z = angVelocityData.gyro.z;
  imuData.accel_x = linearAccelData.acceleration.x;
  imuData.accel_y = linearAccelData.acceleration.y;
  imuData.accel_z = linearAccelData.acceleration.z;

  imuData.weight1 = weight1;
  imuData.weight2 = weight2;
  imuData.weight3 = weight3;
  imuData.weight4 = weight4;
  imuData.weight5 = weight5;
  imuData.weight6 = weight6;
  
  Serial.print("Value loadcell 1 : ");
  Serial.println(weight1);
  Serial.print("Value loadcell 2 : ");
  Serial.println(weight2);
  Serial.print("Value loadcell 3 : ");
  Serial.println(weight3);
  Serial.print("Value loadcell 4 : ");
  Serial.println(weight4);
  Serial.print("Value loadcell 5 : ");
  Serial.println(weight5);
  Serial.print("Value loadcell 6 : ");
  Serial.println(imuData.weight6);
  // Serial.print("GPS : ");
  // Serial.println(imuData.timestamp);
  // Serial.print("IMU x: ");
  // Serial.print(imuData.euler_x);
  // Serial.print(", y: ");
  // Serial.print(imuData.euler_y);
  // Serial.print(", z: ");
  // Serial.println(imuData.euler_z);

  // Write binary data to SD card
  if (dataFile) {
    dataFile.write((uint8_t*)&imuData, sizeof(imuData));
    dataFile.flush(); // Ensure data is written
  }
}

void checkGPSFix() {
  static unsigned long lastBeepTime = 0;
  unsigned long currentTime = millis();

  if (!GPS.fix && (currentTime - lastBeepTime >= 2000)) { // Bip long toutes les 2s si pas de fix
    lastBeepTime = currentTime;
    tone(BUZZZER_PIN, NOTE_C4, 500);
  }
}

bool readToggleSwitch(int pin) {
  int currentReading = digitalRead(pin); // Read the physical switch state

  if (lastSwitchReading == 1 && currentReading == 0) {
    switchState = !switchState;  // Toggle state (0 → 1 or 1 → 0)
  }

  lastSwitchReading = currentReading; // Save last reading
  return switchState;  // Return the toggled state
}

void setup() {
  Serial.begin(115200);

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZZER_PIN, melody[thisNote], noteDuration);

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZZER_PIN);
  }


  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // while (1);
  }
  Serial.println("IMU initialized!");

  // Initialize SD Card
  if (!SD.begin()) {
    Serial.println("Cannot initialize SD card.");
    // while (1);
  }
  else {
    Serial.println("SD card initialized.");
  }

  // Create a unique log file
  int fileIndex = 0;
  do {
    sprintf(currentFileName, "/%s%04d.bin", filePrefix, fileIndex++);
  } while (SD.exists(currentFileName));

  dataFile = SD.open(currentFileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Cannot create file!");
    // while (1);
  } else {
    Serial.print("Created file: ");
    Serial.println(currentFileName);
  }

  // Initialize GPS
  setupGPS();

  setupLoadCells();

  // Start Wi-Fi
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Wi-Fi created: ");
  Serial.println(ssid);
  Serial.print("IP: ");
  Serial.println(IP);

  // Setup HTTP Server
  setupServer();

  pinMode(SWITCH_PIN, INPUT);

  Serial.println("Setup finished"); 
}


void loop() {
  static unsigned long lastLogTime = 0;
  unsigned long currentTime = millis();
  bool currentState = readToggleSwitch(SWITCH_PIN);

  if (currentState) {
    if (!loggingActive) {
      loggingActive = true;
      createNewLogFile();
    }
    
    if (currentTime - lastLogTime >= 100) {
      lastLogTime = currentTime;
      logIMUData();
    }

    if (currentTime - lastBeepTime >= 5000) {
      lastBeepTime = currentTime;
      tone(BUZZZER_PIN, NOTE_C4, 200);
    }
  } else {
    if (loggingActive) {
      loggingActive = false;
      if (dataFile) dataFile.close();
    }
  }

  checkGPSFix();

  server.handleClient();
}