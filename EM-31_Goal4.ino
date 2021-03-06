// Libraries for GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Libraries for SD Card
#include <SPI.h>
#include <SD.h>

// Set Up for GPS
TinyGPSPlus tinyGPS;                                                                    // Name for GPS
SoftwareSerial ssGPS(8, 9);                                                             // Initiate software serial communication with GPS

char logFileName[13];                                                                   // Char to store the current file name
#define LOG_COLUMN_COUNT 8                                                              // Number of columns of GPS and timing information
char * log_col_names[LOG_COLUMN_COUNT] = {                                              // Names to be printed at head of columns
  "longitude", "latitude", "altitude", "date", "time", "satellites", "range", "reading"
};
#define LOG_RATE 2000 ////////////////////!SAMPLING RATE!////////////////////////       // Define the sampling rate
unsigned long l = 0;                                                                    // Set the time of the last log to 0
int j;                                                                                  // Define variable to indicate if the button was pressed

void setup() {
  ssGPS.begin(9600);                                                                    // Start GPS communication
  SD.begin(10);                                                                         // Start micro SD communication
  pinMode(2, INPUT);                                                                    // Pin D - Indicates Range (green wire)
  pinMode(3, INPUT);                                                                    // Pin E - Indicates Range (blue wire)
  pinMode(4, INPUT);                                                                    // Pin F - Indicates Range (red wire)
  pinMode(5, OUTPUT);                                                                   // Green LED - Auto Data
  pinMode(6, OUTPUT);                                                                   // Blue LED - Button Data
  pinMode(7, OUTPUT);                                                                   // Red LED - Power
  digitalWrite(7, HIGH);                                                                // Turn red LED on to indicate there is power
  
  // Begin a New File and Print Header to the File
  while (!tinyGPS.location.isUpdated()) {                                               // Wait for valid GPS data before creating the file name
    while (ssGPS.available()) {                                                         // While waiting, if GPS communication is available
      tinyGPS.encode(ssGPS.read());                                                     // Read GPS data from the GPS
    }
  }
  while (tinyGPS.date.day() == 0) {                                                     // Make sure the day is not zero (the first time is sometimes is)
    while (ssGPS.available()) {                                                         // While waiting, if GPS communication is available
      tinyGPS.encode(ssGPS.read());                                                     // Read GPS data from the GPS
    }
  }
  updateFileName();                                                                     // Create a new file
}

void loop() {
  int G = analogRead(A1);                                                               // Read pin G - Used to see if button was pressed (black wire with 2 silver bands)
  float G2 = G * (5.0 / 1023.0);                                                        // Convert reading to voltage
  if (G2 > 2) {                                                                         // If voltage is above 2, the button is pressed
    j = 1;                                                                              // Set button indicator to indicate button was pressed
    delay(2000);                                                                        // Wait for voltage to stabilize
    if (tinyGPS.location.isUpdated()) {                                                 // If there is GPS data
      if (logGPSData()) {                                                               // Log the data
        digitalWrite(6, HIGH);                                                          // Blink the blue light to indicate data was logged
        delay(250);
        digitalWrite(6, LOW);
      }
    }
    j = 0;                                                                              // Reset the button indicator
  }
  else if ((l + LOG_RATE) <= millis()) {                                                // If the button was not pressed log the data at the sampling rate, check if enough time has passed since last data was logged
    if (logGPSData()) {                                                                 // Log the data
      l = millis();                                                                     // Update the time of the last data log
      digitalWrite(5, HIGH);                                                            // Blink the green light to indicate data was logged
      delay(250);
      digitalWrite(5, LOW);
    }
  }
  while (ssGPS.available())                                                             // Continue to get data from GPS
    tinyGPS.encode(ssGPS.read());
}

byte logGPSData() {                                                                     // Log the data
  int range = 0;                                                                        // Define a variable for the range
  int D = digitalRead(2);                                                               // Check if pin D is high or low
  int E = digitalRead(3);                                                               // Check if pin E is high or low
  int F = digitalRead(4);                                                               // Check if pin F is high or low
  int A = analogRead(A1);                                                               // Read pin A (white wire) to get a conductivity reading
  float A2 = A * (5.0 / 1023.0);                                                        // Convert reading to a voltage
  if (D == 1) {                                                                         // Use the combination of highs and lows for pind D, E, and F to determine the range (mS/m)
    if (E == 1) {
      range = 30;
    }
    else {
      if (F == 1) {
        range = 300;
      }
      else {
        range = 3;
      }
    }
  }
  else {
    if (E == 1) {
      if (F == 1) {
        range = 1000;
      }
      else {
        range = 10;
      }
    }
    else {
      range = 100;
    }
  }
  float A3 = A2 * (range / 0.5);                                                        // Convert the voltage to conductivity (mS/m)

  File logFile = SD.open(logFileName, FILE_WRITE);                                      // Open the file
  if (logFile) {                                                                        // Log the GPS, range, and conductivity data
    if (tinyGPS.location.isUpdated()) {                                                 // If GPS data is available, log the GPS data
    logFile.print(tinyGPS.location.lng(), 6);                                           // Log longitude in degrees
    logFile.print(',');
    logFile.print(tinyGPS.location.lat(), 6);                                           // Log latitude in degrees
    logFile.print(',');
    logFile.print(tinyGPS.altitude.meters(), 1);                                        // Log altitude in meters
    logFile.print(',');
    logFile.print(tinyGPS.date.value());                                                // Log date in DDMMYY format
    logFile.print(',');
    logFile.print(tinyGPS.time.value());                                                // Log time in HHMMSSCC format
    logFile.print(',');
    logFile.print(tinyGPS.satellites.value());                                          // Log the number of satellites
    logFile.print(',');
    }
    else{                                                                               // Otherwise log nothing for each value missed
      logFile.print(",,,,,,")
    }
    logFile.print(range);                                                               // Log the range in mS/m
    logFile.print(',');
    logFile.print(A3);                                                                  // Log the conductivity in mS/m
    if (j == 1) {                                                                       // Indicate if the button was pressed
      logFile.print(',');
      logFile.print("B");
    }
    logFile.println();
    logFile.close();                                                                    // Close the file
    return 1;                                                                           // Indicate data was successfully logged
  }
  return 0;                                                                             // Otherwise, indicate data was not logged
}

void printHeader() {                                                                    // Print the column headers
  File logFile = SD.open(logFileName, FILE_WRITE);                                      // Open the file
  if (logFile) {
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++) {                                                 // Print each of the column headers
      logFile.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1)
        logFile.print(',');                                                             // With a comma between each
      else
        logFile.println();
    }
    logFile.close();                                                                    // Close the file
  }
}

void updateFileName() {                                                                 // Name the file
    memset(logFileName, 0, strlen(logFileName));                                        // Clear the char used to store the file name
    //String m = String(tinyGPS.date.month());
    String d = String(tinyGPS.date.day());                                              // Get day, hour, minutes, and seconds from the GPS
    String h = String(tinyGPS.time.hour());
    String mn = String(tinyGPS.time.minute());
    String s = String(tinyGPS.time.second());
    //  if (m.length() < 2)
    //  {
    //    m = "0" + m;
    //  }
    if (d.length() < 2)                                                                 // If day, hour, minutes, or seconds has only one digit, add a 0 in front
    {
      d = "0" + d;
    }
    if (h.length() < 2)
    {
      h = "0" + h;
    }
    if (mn.length() < 2)
    {
      mn = "0" + mn;
    }
    if (s.length() < 2)
    {
      s = "0" + s;
    }
    String FileName = d + h + mn + s + ".csv";                                          // Put file name together in a string
    FileName.toCharArray(logFileName, 13);                                              // Convert file name string to char
    printHeader();                                                                      // Print the column headers
}

