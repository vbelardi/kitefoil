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
#include <MCP23017.h>

#include "pitches.h"

#include "main.h"

volatile unsigned long last_pps = 0;

# define WIFI_ON true

void PPS_int(){
  last_pps = millis();
}

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

  // Append IMU data section
  html += "<h2>IMU Data</h2>";
  html += "<pre id=\"imuData\">Loading...</pre>";
  html += R"rawliteral(
    <script>
      async function fetchIMUData() {
        try {
          const response = await fetch('/imu_data');
          const data = await response.json();
          document.getElementById('imuData').innerText = JSON.stringify(data, null, 2);
        } catch (error) {
          console.error('Error fetching IMU data:', error);
        }
      }

      setInterval(fetchIMUData, 1400); // Fetch data every 0.5 seconds
      window.onload = fetchIMUData; // Fetch data on page load
    </script>
  )rawliteral";

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

void handleIMUData() {
  // Prepare JSON response
  String json = "{";
  json += "\"cpu_timestamp\":" + String(imuData.cpu_timestamp) + ",";
  json += "\"timestamp\":\"" + String(imuData.timestamp) + "\",";
  json += "\"PPS\":\"" + String(last_pps) + "\",";
  json += "\"Fix\":\"" + String(gps.location.isValid()) + "\",";
  json += "\"latitude\":" + String(imuData.latitude, 6) + ",";
  json += "\"longitude\":" + String(imuData.longitude, 6) + ",";
  json += "\"euler_x\":" + String(imuData.euler_x, 2) + ",";
  json += "\"euler_y\":" + String(imuData.euler_y, 2) + ",";
  json += "\"euler_z\":" + String(imuData.euler_z, 2) + ",";
  json += "\"gyro_x\":" + String(imuData.gyro_x, 2) + ",";
  json += "\"gyro_y\":" + String(imuData.gyro_y, 2) + ",";
  json += "\"gyro_z\":" + String(imuData.gyro_z, 2) + ",";
  json += "\"accel_x\":" + String(imuData.accel_x, 2) + ",";
  json += "\"accel_y\":" + String(imuData.accel_y, 2) + ",";
  json += "\"accel_z\":" + String(imuData.accel_z, 2) + ",";
  json += "\"weight1\":" + String(imuData.weight1) + ",";
  json += "\"weight2\":" + String(imuData.weight2) + ",";
  json += "\"weight3\":" + String(imuData.weight3) + ",";
  json += "\"weight4\":" + String(imuData.weight4) + ",";
  json += "\"weight5\":" + String(imuData.weight5) + ",";
  json += "\"weight6\":" + String(imuData.weight6);
  json += "}";

  server.send(200, "application/json", json);
}


void setupServer() {
  Serial.println("Setting up HTTP server...");
  server.on("/", handleRoot);
  server.on("/download", handleDownload);
  server.on("/delete", handleDelete);
  server.on("/delete_all", handleDeleteAll);
  server.on("/imu_data", handleIMUData);

  Serial.println("Starting server...");
  server.begin();
  Serial.println("HTTP server started.");
}


// Initialize GPS
void setupGPS() {

  // GPS.begin(9600);
  GPS_Serial.begin(GPSBaud);
  // 
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // // GPS.sendCommand(PGCMD_ANTENNA);
  // GPS.sendCommand(PMTK_SET_BAUD_9600);
  // // to get data updated at 5 Hz
  // GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);


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
  static unsigned long last_GPS_print = millis();

  while (GPS_Serial.available())
      gps.encode(GPS_Serial.read());


  // // read data from the GPS in the 'main loop'
  // char c = GPS.read();
  // // if you want to debug, this is a good time to do it!
  // if (GPSECHO)
  //   if (c) Serial.print(c);
  // // if a sentence is received, we can check the checksum, parse it...
  // if (GPS.newNMEAreceived()) {
  //   // a tricky thing here is if we print the NMEA sentence, or data
  //   // we end up not listening and catching other sentences!
  //   // so be very wary if using OUTPUT_ALLDATA and trying to print out data
  //   //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
  //   if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
  //     return ""; // we can fail to parse a sentence in which case we should just wait for another
  // }
  
  // char timestamp[24];
  // sprintf(timestamp, "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
  //         GPS.year, GPS.month, GPS.day,
  //         GPS.hour, GPS.minute, GPS.seconds,
  //         GPS.milliseconds);

  

  
    if (millis() - 1000 > last_GPS_print ){
      // Serial.print("Location: ");
      // Serial.print(GPS.latitude, 2); Serial.print(GPS.lat);
      // Serial.print(", ");
      // Serial.print(GPS.longitude, 2); Serial.println(GPS.lon);
      // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      // Serial.print("Angle: "); Serial.println(GPS.angle);
      // Serial.print("Altitude: "); Serial.println(GPS.altitude);
      // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      // Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);

      // char sz[32];
      // sprintf(sz, "%02d/%02d/%04d ", gps.date.month(), gps.date.day(), gps.date.year());
      // Serial.print("Date : ");
      // Serial.println(sz);

      // char sz1[32];
      // sprintf(sz1, "%02d:%02d:%02d ", gps.time.hour(), gps.time.minute(), gps.time.second());
      // Serial.print("Time : ");
      // Serial.println(sz1);

      // Serial.print(gps.location.lat(), 2); Serial.println(gps.location.lng());

      last_GPS_print = millis();
    }

  char timestamp[24];
  sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d.%02d",
          gps.date.year(),
          gps.date.month(), gps.date.day(), 
          gps.time.hour(), gps.time.minute(), gps.time.second(),
          gps.time.centisecond());
 

  return String(timestamp);

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

  imuData.cpu_timestamp = millis();

  String gpsTimestamp = getGPSTimestamp();
  if (!gpsTimestamp.isEmpty()) {
    strncpy(imuData.timestamp, gpsTimestamp.c_str(), sizeof(imuData.timestamp));
  } else {
    strncpy(imuData.timestamp, "No timestamp", sizeof(imuData.timestamp));
  }

  // update from value changed from interrupt
  imuData.last_pps = last_pps;

  // ! need to add gps pos still

  if (gps.location.isValid()) {
    imuData.latitude = gps.location.lat();
    imuData.longitude = gps.location.lng();
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
  
  // Serial.print("Value loadcell 1 : ");
  // Serial.println(weight1);
  // Serial.print("Value loadcell 2 : ");
  // Serial.println(weight2);
  // Serial.print("Value loadcell 3 : ");
  // Serial.println(weight3);
  // Serial.print("Value loadcell 4 : ");
  // Serial.println(weight4);
  // Serial.print("Value loadcell 5 : ");
  // Serial.println(weight5);
  // Serial.print("Value loadcell 6 : ");
  // Serial.println(imuData.weight6);
  Serial.print("GPS : ");
  Serial.println(imuData.timestamp);
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

  // if (!GPS.fix && (currentTime - lastBeepTime >= 2000)) { // Bip long toutes les 2s si pas de fix
  //   lastBeepTime = currentTime;
  //   tone(BUZZZER_PIN, NOTE_C4, 500);
  // }
}

bool readToggleSwitch(int pin) {
  static bool switchState = false;    
  static bool lastSwitchReading = true; 
  static bool already_changed = false; 
  static unsigned long last_switch_time = millis();

  int currentReading = digitalRead(pin); // Read the physical switch state
  
  if (lastSwitchReading == 1 && currentReading == 0) {
    last_switch_time = millis();
    already_changed = false;
    //Serial.println("Last switchtime reset");
  }
  // when button is pressed, it has to be stayed pressed for a bit
  if (currentReading == 0 && (millis() - 500 > last_switch_time)){
    // avoid that when being pressed it continues to change
    if (already_changed == false){
      switchState = !switchState;  // Toggle state (0 → 1 or 1 → 0)
      already_changed = true;
      //Serial.println("Switched logging state");
    } 
  }

  lastSwitchReading = currentReading; // Save last reading
  return switchState;  // Return the toggled state
}


void toogle_leds(){
  // read state of led pins
  // write the oposite
  mcp.digitalWrite(LOGGING_LED, !mcp.digitalRead(LOGGING_LED));
  mcp.digitalWrite(STATUS_LED, !mcp.digitalRead(STATUS_LED));
}


// * -------------------- SETUP and LOOP --------------------

void setup() {
  Serial.begin(115200);
  // GPS Serial
  Serial2.begin(9600);

  // for (int thisNote = 0; thisNote < 8; thisNote++) {
  //   int noteDuration = 1000 / noteDurations[thisNote];
  //   tone(BUZZZER_PIN, melody[thisNote], noteDuration);

  //   int pauseBetweenNotes = noteDuration * 1.30;
  //   delay(pauseBetweenNotes);
  //   noTone(BUZZZER_PIN);
  // }


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
    Serial.print("First available file: ");
    Serial.println(currentFileName);
    SD.remove(currentFileName);
  }

  

  // Initialize GPS
  setupGPS();

  // add PPS interupt
  attachInterrupt(GPS_PPS, PPS_int, RISING);

  setupLoadCells();

  

  // Setup HTTP Server
  if(WIFI_ON){
    // Start Wi-Fi
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Wi-Fi created: ");
    Serial.println(ssid);
    Serial.print("IP: ");
    Serial.println(IP);


    setupServer();
  }
  

  pinMode(SWITCH_PIN, INPUT);

  // setting pin modes for MCP extender
  mcp.pinMode(STATUS_LED, OUTPUT);
  mcp.pinMode(LOGGING_LED, OUTPUT);

  Serial.println("Setup finished"); 
}

unsigned long last_led_togle = 0;

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

    // if (currentTime - lastBeepTime >= 5000) {
    //   lastBeepTime = currentTime;
    //   tone(BUZZZER_PIN, NOTE_C4, 200);
    // }
    // handling the logging led
    if (currentTime - lastLogLEDTime >= 500) {
      lastLogLEDTime = currentTime;
      mcp.digitalWrite(LOGGING_LED, !mcp.digitalRead(LOGGING_LED));
    }
  } else {
    if (loggingActive) {
      loggingActive = false;
      if (dataFile) dataFile.close();
      mcp.digitalWrite(LOGGING_LED, 0);
    }
  }

  // checkGPSFix();

  if (WIFI_ON){
    server.handleClient();
  }
  

  // if ((millis() - last_led_togle) > 2000){
  //   last_led_togle = millis();
  //   toogle_leds();
  // }
}

