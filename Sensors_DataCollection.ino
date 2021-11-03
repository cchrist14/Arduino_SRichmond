// Libraries for GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Library for SD Card
#include <SD.h>

// Library for Vernier Probes (Nitrate, Chloride, and Thermistor)
#include <math.h>

// Libraries for ADS1115 ADC (for Thermocouples)
#include <Adafruit_ADS1015.h>
#include <Wire.h>

// LEDs
#define powerLight 2                                                // Red LED to turn on when system has power
#define gpsLight 3                                                  // Blue LED to blink when GPS data was logged
#define dataLight 4                                                 // Blue LED to blink when sensor data was logged

// Pin Definitions
#define SD_PIN 53                                                   // Digital pin 53
#define GPS_Rx_PIN 10                                               // Digital pin 10
#define GPS_Tx_PIN 11                                               // Digital pin 11
#define Chloride_PIN A0                                             // Analog pin A0
#define Nitrate_PIN A1                                              // Analog pin A1
#define TypeJ_PIN 0                                                 // ADS1115 pin 0
#define TypeK_PIN 1                                                 // ADS1115 pin 1
#define Turbidity_PIN A3                                            // Analog pin A3

// Set Up for GPS
SoftwareSerial ssGPS(GPS_Rx_PIN, GPS_Tx_PIN);                       // Create a software serial communication
Adafruit_GPS GPS(&ssGPS);

// Set Up for ADS1115 ADC (for Thermocouples)
Adafruit_ADS1115 ads1115;

String f = "";                                                      // String for reading from files
int lr = 1000;                                                      // Log rate in ms
unsigned long ll = 0;                                               // Time of last log
char logfilename[13];                                               // Char to hold file name for data

// Variables for GPS
float lt, lg, a;                                                    // lt = latitude (ddmm.mmmm), lg = longitude (dddmm.mmmm), a = altitude (m)
uint8_t y, m, d, h, mn, s, sv;                                      // y = year, m = month, d = day, h = hour, mn = minute, s = second (UTC time), sv = number of satellites
uint16_t ms;                                                        // ms = millisecond
boolean GPS_complete = false;                                       // Indicator for if GPS data was valid
float timeCheck1;                                                   // Variable to check if time data is new
float timeCheck2;                                                   // Variable to check if time data is new and valid
float dateCheck;                                                    // Variable to check if date is valid
double mins = 0.0;                                                  // Variable to hold minutes when converting from ddmm.mmmm to decimal degrees
int degs = 0;                                                       // Variable to hold degrees when converting from ddmm.mmmm to decimal degrees
double lt_dd, lg_dd;                                                // lt_dd = latitude (deg), lg_dd = longitude (deg)

// Variables for Nitrate
int n, nvh, nvl;                                                    // n = analog value of nitrate, nvh = analog value of high standard, nvl = analog value of low standard
float nh, nl, nvhmV, nvlmV, nEo, nm;                                // Variables for Nitrate Calibration, nh = high standard concentration (mg/L), nl = low standard concentration (mg/L), nvhmV and nvlmV = analog values of standard converted to mV, nEo and nm = calibration parameters
float nV, nmV, nC;                                                  // Variables for Nitrate Readings, nV = n converted to V, nmV = nV converted to mV, nC = nmV converted to mg/L

// Variables for Chloride
int c, cvh, cvl;                                                    // c = analog value of chloride, cvh = analog value of high standard, nvl = analog value of low standard
float ch, cl, cvhmV, cvlmV, cEo, cm;                                // Variables for Chloride Calibration, ch = high standard concentration (mg/L), cl = low standard concentration (mg/L), cvhmV and cvlmV = analog values of standards converted to mV, cEo and cm = calibration parameters
float cV, cmV, cC;                                                  // Variables for Chloride Readings, cV = c converted to V, cmV = cV converted to mV, cC = cmV converted to mg/L

// Variables for Electrical Conductivity Communication
String iEC = "";                                                    // String to hold strings sent from serial to EC
String sEC = "";                                                    // String to hold strings sent from EC to be read
boolean iEC_complete = false;                                       // Indicator for if string is ready to be sent to EC
boolean sEC_complete = false;                                       // Indicator for if EC string is ready to be read

// Variables for pH Communication
String ipH = "";                                                    // String to hold strings sent from serial to pH
String spH = "";                                                    // String to hold strings sent from pH to be read
boolean ipH_complete = false;                                       // Indicator for if string is ready to be sent to pH
boolean spH_complete = false;                                       // Indicator for if pH string is ready to be read

// Variables for Dissolved Oxygen Communication
String iDO = "";                                                    // String to hold strings sent from serial to DO
String sDO = "";                                                    // String to hold strings sent from DO to be read
boolean iDO_complete = false;                                       // Indicator for if string is ready to be sent to DO
boolean sDO_complete = false;                                       // Indicator for if DO string is ready to be read

// Variables for Type J Thermocouple
int16_t t_J;                                                        // t_J = analog value of temperature
float tl_J, th_J, tvl_J, tvh_J, tm_J, tb_J, tC_J;                   // tl_J = low calibration temperature, th_J = high calibration temperature, tvl_J = analog value of low calibration temperature, tvh_J = analog value of high calibration temperature, tm_J = slope of linear calibration, tb_J = intercept of linear calibration, tC_J = temperature reading in deg. C

// Variables for Type K Thermocouple
int16_t t_K;                                                        // t_K = analog value of temperature
float tl_K, th_K, tvl_K, tvh_K, tm_K, tb_K, tC_K;                   // tl_K = low calibration temperature, th_K = high calibration temperature, tvl_K = analog value of low calibration temperature, tvh_K = analog value of high calibration temperature, tm_K = slope of linear calibration, tb_K = intercept of linear calibration, tC_K = temperature reading in deg. C

// Variables for Turbidity
int tbd;                                                            // Variable for analog value of turbidity
float tbdV;                                                         // Variable for Voltage of turbidity

boolean error;                                                      // Variable to indicate if any errors occured during setup

void setup() {
  // Set up LEDs
  pinMode(powerLight, OUTPUT);                                      // Set up LED to indicate if power is supplied
  pinMode(gpsLight, OUTPUT);                                        // Set up LED to indicate if GPS data is being logged
  pinMode(dataLight, OUTPUT);                                       // Set up LED to indicate if data is being logged

  digitalWrite(powerLight, HIGH);                                   // Turn on power LED to indicate system has power

  // Begin Communications
  Serial.begin(9600);                                               // Begin communication with computer
  GPS.begin(9600);                                                  // Begin communication with GPS
  Serial1.begin(9600);                                              // Begin communication with EC (Rx - 19; Tx - 18);
  Serial2.begin(9600);                                              // Begin communication with pH (Rx - 17; Tx - 16);
  Serial3.begin(9600);                                              // Begin communication with DO (Rx - 15; Tx - 14);
  ads1115.begin();                                                  // Begin communication with ADS1115 ADC (for Thermocouples)
  if (!SD.begin(SD_PIN)) {                                          // Initiate microSD card
    Serial.println("SD Error");
    for (int i = 0; i < 5; i++) {                                   // If there was an error with initiation, blink blue LEDs 5 times
      digitalWrite(gpsLight, HIGH);
      digitalWrite(dataLight, HIGH);
      delay(250);
      digitalWrite(gpsLight, LOW);
      digitalWrite(dataLight, LOW);
      delay(250);
    }
  }

  // ADS1115 Settings
  ads1115.setGain(GAIN_TWO);                                        // Sets gain to 2 (+/- 2.048 V)

  // GPS Settings
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                     // Enables RMC and GGA strings
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                        // Sets GPS update rate to 1 Hz
  //  GPS.sendCommand(PGCMD_ANTENNA);                               // Enables updates on Antenna status

  // Reserve Bytes for Communication with EC, pH, and DO
  iEC.reserve(10);
  sEC.reserve(30);
  ipH.reserve(10);
  spH.reserve(30);
  iDO.reserve(10);
  sDO.reserve(30);

  // If Calibration Data Exists, Calculate Calibration Parameters
  if (SD.exists("nvh.csv") && SD.exists("nvl.csv") && SD.exists("nh.csv") && SD.exists("nl.csv")) { // Check if all nitrate calibration files exist, and if they do
    File f_nvh = SD.open("nvh.csv");                                // Open the calibration file for the high standard nitrate calibration value
    if (f_nvh) {                                                    // If the file was opened
      f = f_nvh.readStringUntil(13);                                // Get the high standard calibration value for nitrate
      nvh = f.toFloat();                                            // Convert that value to a number
      f_nvh.close();                                                // Close the file
    }
    File f_nvl = SD.open("nvl.csv");                                // Open the calibration file for the low standard nitrate calibration value
    if (f_nvl) {                                                    // If the file was opened
      f = f_nvl.readStringUntil(13);                                // Get the low standard calibration value for nitrate
      nvl = f.toFloat();                                            // Convert that value to a number
      f_nvl.close();                                                // Close the file
    }
    File f_nh = SD.open("nh.csv");                                  // Open the calibration file for the high standard nitrate concentration
    if (f_nh) {                                                     // If the file was opened
      f = f_nh.readStringUntil(13);                                 // Get the high standard concentration for nitrate
      nh = f.toFloat();                                             // Convert that value to a number
      f_nh.close();                                                 // Close the file
    }
    File f_nl = SD.open("nl.csv");                                  // Open the calibration file for the low standard nitrate concentration
    if (f_nl) {                                                     // If the file was opened
      f = f_nl.readStringUntil(13);                                 // Get the low standard concentration for nitrate
      nl = f.toFloat();                                             // Convert that value to a number
      f_nl.close();                                                 // Close the file
    }
    nvhmV = 137.55 * (nvh / 1023.0 * 5.0) - 0.1682;                 // Using the nitrate calibration values, calculate the nitrate calibration parameters
    nvlmV = 137.55 * (nvl / 1023.0 * 5.0) - 0.1682;                 // These equations come from the manual for the vernier nitrate probe
    nm = (nvhmV - nvlmV) / log(nh / nl);
    nEo = nvlmV - (nm * log(nl));
  }
  else {                                                            // If no nitrate calibration data, indicate there was an error
    error = true;
    Serial.println("Need Nitrate Calibration");
  }
  if (SD.exists("cvh.csv") && SD.exists("cvl.csv") && SD.exists("ch.csv") && SD.exists("cl.csv")) { // Check if all chloride calibration files exists, and if they do,
    File f_cvh = SD.open("cvh.csv");                                // Open the calibration file for the high standard chloride calibration value
    if (f_cvh) {                                                    // If the file was opened
      f = f_cvh.readStringUntil(13);                                // Get the high standard calibration value for chloride
      cvh = f.toFloat();                                            // Convert that value to a number
      f_cvh.close();                                                // Close the file
    }
    File f_cvl = SD.open("cvl.csv");                                // Open the calibration file for the low standard chloride calibration value
    if (f_cvl) {                                                    // If the file was opened
      f = f_cvl.readStringUntil(13);                                // Get the low standard calibration value for chloride
      cvl = f.toFloat();                                            // Convert that value to a number
      f_cvl.close();                                                // Close the file
    }
    File f_ch = SD.open("ch.csv");                                  // Open the calibration file for the high standard chloride concentration
    if (f_ch) {                                                     // If the file was opened
      f = f_ch.readStringUntil(13);                                 // Get the high standard concentration for chloride
      ch = f.toFloat();                                             // Convert that value to a number
      f_ch.close();                                                 // Close the file
    }
    File f_cl = SD.open("cl.csv");                                  // Open the calibration file for the low standard chloride concentration
    if (f_cl) {                                                     // If the file was opened
      f = f_cl.readStringUntil(13);                                 // Get the low standard concentration for chloride
      cl = f.toFloat();                                             // Convert that value to a number
      f_cl.close();                                                 // Close the file
    }
    cvhmV = 137.55 * (cvh / 1023.0 * 5.0) - 0.1682;                 // Using the chloride calibration values, calculate the chloride calibration parameters
    cvlmV = 137.55 * (cvl / 1023.0 * 5.0) - 0.1682;                 // These equations come from the manual for the vernier chloride probe
    cm = (cvhmV - cvlmV) / log(ch / cl);
    cEo = cvlmV - (cm * log(cl));
  }
  else {                                                            // If no chloride calibration data, indicate there was an error
    error = true;
    Serial.println("Need Chloride Calibration");
  }
  if (SD.exists("tvl_J.csv") && SD.exists("tvh_J.csv") && SD.exists("tl_J.csv") && SD.exists("th_J.csv")) { // Check if all type J thermocouple calibration files exists, and if they do,
    File f_tvl_J = SD.open("tvl_J.csv");                            // Open the calibration file for the lower temperature calibration value
    if (f_tvl_J) {                                                  // If the file was opened
      f = f_tvl_J.readStringUntil(13);                              // Get the lower temperature calibration value
      tvl_J = f.toFloat();                                          // Convert that value to a number
      f_tvl_J.close();                                              // Close the file
    }
    File f_tvh_J = SD.open("tvh_J.csv");                            // Open the calibration file for the higher temperature calibration value
    if (f_tvh_J) {                                                  // If the file was opened
      f = f_tvh_J.readStringUntil(13);                              // Get the higher temperature calibration value
      tvh_J = f.toFloat();                                          // Convert that value to a number
      f_tvh_J.close();                                              // Close the file
    }
    File f_tl_J = SD.open("tl_J.csv");                              // Open the calibration file for the lower temperature value
    if (f_tl_J) {                                                   // If the file was opened
      f = f_tl_J.readStringUntil(13);                               // Get the lower temperature value
      tl_J = f.toFloat();                                           // Convert that value to a number
      f_tl_J.close();                                               // Close the file
    }
    File f_th_J = SD.open("th_J.csv");                              // Open the calibration file for the higher temperature value
    if (f_th_J) {                                                   // If the file was opened
      f = f_th_J.readStringUntil(13);                               // Get the higher temperature value
      th_J = f.toFloat();                                           // Convert that value to a number
      f_th_J.close();                                               // Close the file
    }
    tm_J = (th_J - tl_J) / (tvh_J - tvl_J);                         // Using the temperature calibration and temperature values, calculate the type J thermocouple calibration parameters
    tb_J = th_J - (tm_J * tvh_J);                                   // These equations assume a linear relationship
  }
  else {                                                            // If no type J thermocouple calibration data, indicate there was an error
    error = true;
    Serial.println("Need Type J  Thermocouple Calibration");
  }
  if (SD.exists("tvl_K.csv") && SD.exists("tvh_K.csv") && SD.exists("tl_K.csv") && SD.exists("th_K.csv")) { // Check if all type K thermocouple calibration files exists, and if they do,
    File f_tvl_K = SD.open("tvl_K.csv");                            // Open the calibration file for the lower temperature calibration value
    if (f_tvl_K) {                                                  // If the file was opened
      f = f_tvl_K.readStringUntil(13);                              // Get the lower temperature calibration value
      tvl_K = f.toFloat();                                          // Convert that value to a number
      f_tvl_K.close();                                              // Close the file
    }
    File f_tvh_K = SD.open("tvh_K.csv");                            // Open the calibration file for the higher temperature calibration value
    if (f_tvh_K) {                                                  // If the file was opened
      f = f_tvh_K.readStringUntil(13);                              // Get the higher temperature calibration value
      tvh_K = f.toFloat();                                          // Convert that value to a number
      f_tvh_K.close();                                              // Close the file
    }
    File f_tl_K = SD.open("tl_K.csv");                              // Open the calibration file for the lower temperature value
    if (f_tl_K) {                                                   // If the file was opened
      f = f_tl_K.readStringUntil(13);                               // Get the lower temperature value
      tl_K = f.toFloat();                                           // Convert that value to a number
      f_tl_K.close();                                               // Close the file
    }
    File f_th_K = SD.open("th_K.csv");                              // Open the calibration file for the higher temperature value
    if (f_th_K) {                                                   // If the file was opened
      f = f_th_K.readStringUntil(13);                               // Get the higher temperature value
      th_K = f.toFloat();                                           // Convert that value to a number
      f_th_K.close();                                               // Close the file
    }
    tm_K = (th_K - tl_K) / (tvh_K - tvl_K);                         // Using the temperature calibration and temperature values, calculate the type K thermocouple calibration parameters
    tb_K = th_K - (tm_K * tvh_K);                                   // These equations assume a linear relationship
  }
  else {                                                            // If no type K thermocouple calibration data, indicate there was an error
    error = true;
    Serial.println("Need Type K Thermocouple Calibration");
  }

  if (error == true) {                                              // If there was an error during set up (missing calibration data)
    digitalWrite(powerLight, LOW);                                  // Turn power light off
    delay(250);
    for (int i = 0; i < 10; i++) {                                  // Blink all three LEDs 10 times
      digitalWrite(powerLight, HIGH);
      digitalWrite(gpsLight, HIGH);
      digitalWrite(dataLight, HIGH);
      delay(250);
      digitalWrite(powerLight, LOW);
      digitalWrite(gpsLight, LOW);
      digitalWrite(dataLight, LOW);
      delay(250);
    }
  }
  digitalWrite(powerLight, HIGH);                                   // Turn power light back on

  // Begin a New File and Print Header to the File
  Serial.println("GPS looking for signal...");
  while (!GPS.newNMEAreceived()) {                                  // Wait for GPS data before continuing
    GPS.read();
  }
  Serial.println("GPS waiting for good data...");
  while (timeCheck2 == 0 || dateCheck == 0) {                       // Wait until GPS data is valid before naming file
    GPS.read();
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
      timeCheck2 = float(GPS.hour) * 10000 + float(GPS.minute) * 100 + float(GPS.seconds) + float(GPS.milliseconds) / 1000;
      dateCheck = float(GPS.year) * 10000 + float(GPS.month) * 100 + float(GPS.day);
    }
  }
  updateFileName();                                                 // Run the function that updates the file name
  File f_dat = SD.open(logfilename, FILE_WRITE);                    // Open the new data file
  if (f_dat) {                                                      // If the file was opened
    f_dat.println("Year,Month,Day,Hour,Minute,Second,Millisecond,Latitude,Longitude,Altitude,Satellites,Millis,Nitrate,Chloride,EC,pH,DO,Type J Thermocouple,Type K Thermocouple,Turbidity");  // Print the column headers for the data
    f_dat.close();                                                  // Close the file
  }
  Serial.print("Year");
  Serial.print('\t');
  Serial.print("Month");
  Serial.print('\t');
  Serial.print("Day");
  Serial.print('\t');
  Serial.print("Hour");
  Serial.print('\t');
  Serial.print("Minute");
  Serial.print('\t');
  Serial.print("Second");
  Serial.print('\t');
  Serial.print("Millisecond");
  Serial.print('\t');
  Serial.print("Latitude");
  Serial.print('\t');
  Serial.print("Longitude");
  Serial.print('\t');
  Serial.print("Altitude");
  Serial.print('\t');
  Serial.print("Satellites");
  Serial.print('\t');
  Serial.print("Millis");
  Serial.print('\t');
  Serial.print("Nitrate");
  Serial.print('\t');
  Serial.print("Chloride");
  Serial.print('\t');
  Serial.print("EC");
  Serial.print('\t');
  Serial.print("pH");
  Serial.print('\t');
  Serial.print("DO");
  Serial.print('\t');
  Serial.print("Type J");
  Serial.print('\t');
  Serial.print("Type K");
  Serial.print('\t');
  Serial.println("Turbidity");
}

// Get Data from EC
void serialEvent1() {                                               // Triggered if there is an incoming string from EC
  sEC = "";                                                         // Clear the EC sensor string variable
  sEC = Serial1.readStringUntil(13);                                // Store the incoming string in the EC sensor string variable, to be sent to the serial and/or logged
  sEC_complete = true;                                              // Indicate that there is a string to be sent to the serial and/or data to be logged
  if (sEC.length() > 5) {                                           // If there is an error and the data is too long (two data points mashed together)
    sEC_complete = false;                                           // Indicate that there is no data to be logged instead
  }
}

// Get Data from pH
void serialEvent2() {                                               // Triggered if there is an incoming string from from pH
  spH = "";                                                         // Clear the pH sensor string variable
  spH = Serial2.readStringUntil(13);                                // Store the incoming string in the pH sensor string variable, to be sent to the serial and/or logged
  spH_complete = true;                                              // Indicate that there is a string to be sent to the serial and/or data to be logged
  if (spH.length() > 6) {                                           // If there is an error and the data is too long (two data points mashed together)
    spH_complete = false;                                           // Indicate that there is no data to be logged instead
  }
}

// Get Data from DO
void serialEvent3() {                                               // Triggered if there is an incoming string from DO
  sDO = "";                                                         // Clear the DO sensor string variable
  sDO = Serial3.readStringUntil(13);                                // Store the incoming string in the DO sensor string variable, to be sent to the serial and/or logged
  sDO_complete = true;                                              // Indicate that there is a string to be sent to the serial and/or data to be logged
  if (sDO.length() > 5) {                                           // If there is an error and the data is too long (two data points mashed together)
    sDO_complete = false;                                           // Indicate that there is no data to be logged instead
  }
}

void loop() {
  // Get GPS Data
  while (!GPS.newNMEAreceived()) {                                  // Wait for next NMEA string
    GPS.read();
  }
  if (GPS.newNMEAreceived()) {                                      // If there is new GPS data to be read
    GPS.parse(GPS.lastNMEA());                                      // Parse the new GPS data
    timeCheck2 = float(GPS.hour) * 10000 + float(GPS.minute) * 100 + float(GPS.seconds) + float(GPS.milliseconds) / 1000; // Determine the new time in HHMMSS.SSS format
    dateCheck = float(GPS.year) * 10000 + float(GPS.month) * 100 + float(GPS.day);  // Determine the new date in YYMMDD format
    if (timeCheck1 != timeCheck2 && timeCheck2 != 0 && dateCheck != 0) {  // If the new data is valid
      y = GPS.year;                                                 // Get the year (YY)
      m = GPS.month;                                                // Get the month (MM)
      d = GPS.day;                                                  // Get the day (DD)
      h = GPS.hour;                                                 // Get the hour (HH)
      mn = GPS.minute;                                              // Get the minute (MM)
      s = GPS.seconds;                                              // Get the seconds (SS)
      ms = GPS.milliseconds;                                        // Get the milliseconds (MMM)
      lt = GPS.latitude;                                            // Get the latitude in ddmm.mmmm format
      lg = GPS.longitude;                                           // Get the longitude in dddmm.mmmm format
      a = GPS.altitude;                                             // Get the altitude in meters
      sv = GPS.satellites;                                          // Get the number of satellites
      timeCheck1 = timeCheck2;                                      // Set the new time as the last recorded time

      // Convert latitude and longitude to decimal degrees
      mins = fmod((double)lt, 100.0);                               // Determine the minutes of the latitude
      degs = (int)(lt / 100);                                       // Determine the degrees of the latitude
      lt_dd = degs + (mins / 60);                                   // Compute the latitude in decimal degrees

      mins = fmod((double)lg, 100.0);                               // Determine the minutes of the longitude
      degs = (int)(lg / 100);                                       // Determine the degrees of the longitude
      lg_dd = degs + (mins / 60);                                   // Compute the longitude in decimal degrees

      GPS_complete = true;                                          // Indicate that there is valid GPS data to be logged
    }
  }

  // Get Nitrate Data
  n = analogRead(Nitrate_PIN);                                      // Read the analog value from the nitrate probe
  nV = n / 1023.0 * 5.0;                                            // Convert that value to V using 10 bit conversion from Arduino ADC
  nmV = 137.55 * nV - 0.1682;                                       // Convert that value to mV using equation given in the vernier nitrate probe manual
  double(nval) = ((nmV - nEo) / nm);                                // Using the equation given in the vernier nitrate probe manual and the calibration parameters,
  nC = exp(nval);                                                   // Calculate the concentration of nitrate in mg/L

  // Get Chloride Data
  c = analogRead(Chloride_PIN);                                     // Read the analog value from the chloride probe
  cV = c / 1023.0 * 5.0;                                            // Convert that value to V using 10 bit conversion from Arduino ADC
  cmV = 137.55 * cV - 0.1682;                                       // Convert that value to mV using equation given in the vernier nitrate probe manual
  double(cval) = ((cmV - cEo) / cm);                                // Using the equation given in the vernier chloride probe manual and the calibration parameters,
  cC = exp(cval);                                                   // Calculate the concentration of chloride in mg/L

  // Get Type J Thermocouple Data
  t_J = ads1115.readADC_SingleEnded(TypeJ_PIN);                     // Read the analog value from the type J thermocouple
  tC_J = tm_J * t_J + tb_J;                                         // Using the calibration parameters and an assumed linear relationship, calculate temperature in deg. C from the average analog value

  // Get Type K Thermocouple Data
  t_K = ads1115.readADC_SingleEnded(TypeK_PIN);                     // Read the analog value from the type K thermocouple
  tC_K = tm_K * t_K + tb_K;                                         // Using the calibration parameters and an assumed linear relationship, calculate temperature in deg. C of the average analog value

  // Get Turbidity Data
  tbd = analogRead(Turbidity_PIN);                                  // Read the analog value from the turbidity sensor
  tbdV = tbd * (5.0 / 1023.0);                                      // Convert that value to V using 10 bit conversion from Arduino ADC

  // Log Data to SD Card
  if ((ll + lr) <= millis() && millis() >= 5000) {                  // Check if it's time for next data point
    if (logData()) {                                                // If data was logged
      digitalWrite(dataLight, HIGH);                                // Blink the data LED to indicate data was logged
      delay(100);
      digitalWrite(dataLight, LOW);
    }
  }
}

// Function that logs the data to the micro SD card and prints it in the serial
byte logData() {                                                    // Triggered when function to log the data is run
  File f_dat = SD.open(logfilename, FILE_WRITE);                    // Open the data file in write mode
  if (f_dat) {                                                      // If the file was opened
    if (GPS_complete == true) {                                     // If there is valid GPS data to log
      f_dat.print(y + 2000);                                        // Log the year (YYYY)
      f_dat.print(',');
      f_dat.print(m);                                               // Log the month (MM)
      f_dat.print(',');
      f_dat.print(d);                                               // Log the day (DD)
      f_dat.print(',');
      f_dat.print(h);                                               // Log the hour (HH, UTC time)
      f_dat.print(',');
      f_dat.print(mn);                                              // Log the minute (MM, UTC time)
      f_dat.print(',');
      f_dat.print(s);                                               // Log the seconds (SS, UTC time)
      f_dat.print(',');
      f_dat.print(ms);                                              // Log the milliseconds (MMM, UTC time)
      f_dat.print(',');
      f_dat.print(lt_dd, 6);                                        // Log latitude in degrees
      f_dat.print(',');
      f_dat.print(lg_dd, 6);                                        // Log longitude in degrees
      f_dat.print(',');
      f_dat.print(a, 1);                                            // Log altitude in meters
      f_dat.print(',');
      f_dat.print(sv);                                              // Log the number of satellites
      f_dat.print(',');
      Serial.print(y + 2000);                                       // Log the year (YYYY)
      Serial.print('\t');
      Serial.print(m);                                              // Log the month (MM)
      Serial.print('\t');
      Serial.print(d);                                              // Log the day (DD)
      Serial.print('\t');
      Serial.print(h);                                              // Log the hour (HH, UTC time)
      Serial.print('\t');
      Serial.print(mn);                                             // Log the minute (MM, UTC time)
      Serial.print('\t');
      Serial.print(s);                                              // Log the seconds (SS, UTC time)
      Serial.print('\t');
      Serial.print(ms);                                             // Log the milliseconds (MMM, UTC time)
      Serial.print('\t');
      Serial.print(lt_dd, 6);                                       // Log latitude in degrees
      Serial.print('\t');
      Serial.print(lg_dd, 6);                                       // Log longitude in degrees
      Serial.print('\t');
      Serial.print(a, 1);                                           // Log altitude in meters
      Serial.print('\t');
      Serial.print(sv);                                             // Log the number of satellites
      Serial.print('\t');
      GPS_complete = false;                                         // Indicate that GPS data was printed to the serial
      digitalWrite(gpsLight, HIGH);                                 // Blink the GPS LED indicating GPS data was logged
      delay(100);
      digitalWrite(gpsLight, LOW);
    }
    else {                                                          // If there is not valid GPS data to log, log error value (-9999) for each value missed
      f_dat.print("-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,");
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
      Serial.print('\t');
    }
    f_dat.print(millis());                                          // Log the Arduino time in millis
    f_dat.print(',');
    f_dat.print(nC);                                                // Log the nitrate concentration in mg/L
    f_dat.print(',');
    f_dat.print(cC);                                                // Log the chloride concentration in mg/L
    f_dat.print(',');
    Serial.print(millis());                                         // Log the Arduino time in millis
    Serial.print('\t');
    Serial.print(nC);                                               // Log the nitrate concentration in mg/L
    Serial.print('\t');
    Serial.print(cC);                                               // Log the chloride concentration in mg/L
    Serial.print('\t');
    if (sEC_complete == true) {                                     // If there is a string to read from EC
      f_dat.print(sEC);                                             // Log EC string/data
      f_dat.print(',');
      Serial.print(sEC);                                            // Log EC string/data
      Serial.print('\t');
      sEC_complete = false;                                         // Indicate string from EC was read
    }
    else {                                                          // If there is not a string to read from EC
      f_dat.print("-9999,");                                        // Log error value (-9999)
      Serial.print('\t');                                           // Log nothing
    }
    if (spH_complete == true) {                                     // If there is a string to read from pH
      f_dat.print(spH);                                             // Log pH string/data
      f_dat.print(',');
      Serial.print(spH);                                            // Log pH string/data
      Serial.print('\t');
      spH_complete = false;                                         // Indicate string from pH was read
    }
    else {                                                          // If there is not a string to read from pH
      f_dat.print("-9999,");                                        // Log error value (-9999)
      Serial.print('\t');                                           // Log nothing
    }
    if (sDO_complete == true) {                                     // If there is a string to read from DO
      f_dat.print(sDO);                                             // Log DO string/data
      f_dat.print(',');
      Serial.print(sDO);                                            // Log DO string/data
      Serial.print('\t');
      sDO_complete = false;                                         // Indicate string from DO was read
    }
    else {                                                          // If there is not a string to read from DO
      f_dat.print("-9999,");                                        // Log error value (-9999)
      Serial.print('\t');                                           // Log nothing
    }
    f_dat.print(tC_J);                                              // Log the type J thermocouple temperature in deg. C
    f_dat.print(',');
    f_dat.print(tC_K);                                              // Log the type K thermocouple temperature in deg. C
    f_dat.print(',');
    Serial.print(tC_J);                                             // Log the type J thermocouple temperature in deg. C
    Serial.print('\t');
    Serial.print(tC_K);                                             // Log the type K thermocouple temperature in deg. C
    Serial.print('\t');
    f_dat.println(tbdV);                                            // Log the turbidity voltage in V
    Serial.println(tbdV);                                           // Log the turbidity voltage in V
    f_dat.close();                                                  // Close the file
    ll = millis();                                                  // Save the time as the time of the last log
    return 1;                                                       // Return that data was logged successfully
  }
  return 0;                                                         // If the file was not opened, return that data was not logged
}

void updateFileName() {                                             // Triggered when function to update the file name is run
  memset(logfilename, 0, strlen(logfilename));                      // Clear the char used to store the file name
  //  String mon = String(GPS.month);                               // Get the month from the GPS and convert to a string (Can be switched with seconds for MMDDMMHH format)
  String dy = String(GPS.day);                                      // Get the day from the GPS and convert to a string
  String hr = String(GPS.hour);                                     // Get the hour from the GPS and convert to a string
  String mnt = String(GPS.minute);                                  // Get the minute from the GPS and convert to a string
  String sec = String(GPS.seconds);                                 // Get the second from the GPS and convert to a string
  //  if (mon.length() < 2)                                         // If the month is less than 10, add a '0' as the first digit
  //  {
  //    mon = "0" + mon;
  //  }
  if (dy.length() < 2)                                              // If the day is less than 10, add a '0' as the first digit
  {
    dy = "0" + dy;
  }
  if (hr.length() < 2)                                              // If the hour is less than 10, add a '0' as the first digit
  {
    hr = "0" + hr;
  }
  if (mnt.length() < 2)                                             // If the minute is less than 10, add a '0' as the first digit
  {
    mnt = "0" + mnt;
  }
  if (sec.length() < 2)                                             // If the second is less than 10, add a '0' as the first digit
  {
    sec = "0" + sec;
  }
  String FileName = dy + hr + mnt + sec + ".csv";                   // Combine the day, hour, minute and second string to DDHHMMSS format and add a '.csv' to create the file name string
  FileName.toCharArray(logfilename, 13);                            // Convert the file name string to a char array
}

