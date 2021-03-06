// Library for SD Card
#include <SD.h>

// Library for Vernier Probes (Nitrate, Chloride, and Thermistor)
#include <math.h>

// Libraries for ADS1115 ADC (for Thermocouples)
#include <Adafruit_ADS1015.h>
#include <Wire.h>

// Pin Definitions
#define SD_PIN 53                                                   // Digital pin 53
#define Chloride_PIN A0                                             // Analog pin A0
#define Nitrate_PIN A1                                              // Analog pin A1
#define TypeJ_PIN 0                                                 // ADS1115 pin 0
#define TypeK_PIN 1                                                 // ADS1115 pin 1

// Set Up for ADS1115 ADC (for Thermocouples)
Adafruit_ADS1115 ads1115;

String i = "";                                                      // String for reading from serial
String f = "";                                                      // String for reading from files
int id = 0;                                                         // ID used to switch communication between EC, pH and DO
int lr = 1000;                                                      // Log rate in ms
unsigned long ll = 0;                                               // Time of last log

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

void setup() {
  // Begin Communications
  Serial.begin(9600);                                               // Begin communication with computer
  Serial1.begin(9600);                                              // Begin communication with EC (Rx - 19; Tx - 18);
  Serial2.begin(9600);                                              // Begin communication with pH (Rx - 17; Tx - 16);
  Serial3.begin(9600);                                              // Begin communication with DO (Rx - 15; Tx - 14);
  ads1115.begin();                                                  // Begin communication with ADS1115 ADC (for Thermocouples)
  if (!SD.begin(SD_PIN)) {                                          // Indicate if micro SD card was successfully initiated
    Serial.println("SD Error");
  }
  else {
    Serial.println("SD Ready");
  }

  // ADS1115 Settings
  ads1115.setGain(GAIN_TWO);                                        // Sets gain to 2 (+/- 2.048 V)

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
  else {                                                            // If not all nitrate calibration files exists, indicate that nitrate needs calibration
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
  else {                                                            // If not all chloride files exist, indicate that chloride needs calibration
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
  else {                                                            // If not all type J thermocouple files exist, indicate that type J thermocouple needs calibration
    Serial.println("Need Type J Thermocouple Calibration");
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
  else {                                                            // If not all type K thermocouple files exist, indicate the type K thermocouple needs calibration
    Serial.println("Need Type K Thermocouple Calibration");
  }

  // Print Headers
  Serial.print("EC");
  Serial.print('\t');
  Serial.print("pH");
  Serial.print('\t');
  Serial.print("DO");
  Serial.print('\t');
  Serial.print("NO3-");
  Serial.print('\t');
  Serial.print("Cl-");
  Serial.print('\t');
  Serial.print("Type J");
  Serial.print('\t');
  Serial.println("Type K");
}

void serialEvent() {                                                // Triggered if there is an incoming string sent from the serial
  i = Serial.readStringUntil(13);                                   // Read the incoming string from serial

  // Change Communication Between EC, pH and DO                     // These are the commands to tell the Arduino which sensor you want to send commands to, needed for EC, pH, and DO
  if (i == "EC") {                                                  // If the string was "EC" change the id to 3 to switch communication to EC
    id = 1;
    Serial.println("Communicating with EC");
  }
  else if (i == "pH") {                                             // If the string was "pH" change the id to 2 to switch communication to pH
    id = 2;
    Serial.println("Communicating with pH");
  }
  else if (i == "DO") {                                             // If the string was "DO" change the id to 1 to switch communication to DO
    id = 3;
    Serial.println("Communicating with DO");
  }

  // Begin Nitrate Calibration
  else if (i == "n cal") {                                          // If the string was "n cal"
    Serial.println("Begin Nitrate Calibration");                    // Begin nitrate calibration
    Caln();                                                         // Run the function for nitrate calibration
  }

  // Begin Chloride Calibration
  else if (i == "c cal") {                                          // If the string was "c cal"
    Serial.println("Begin Chloride Calibration");                   // Begin chloride calibration
    Calc();                                                         // Run the function for chloride calibration
  }

  // Begin Type J Thermocouple Calibration
  else if (i == "tJ cal") {                                         // If the string was "tJ cal"
    Serial.println("Begin Type J Thermocouple Calibration");        // Begin type J thermocouple calibration
    CaltJ();                                                        // Run the function for type J thermocouple calibration
  }

  // Begin Type K Thermocouple Calibration
  else if (i == "tK cal") {                                         // If the string was "tK cal"
    Serial.println("Begin Type K Thermocouple Calibration");        // Begin type K thermocouple calibration
    CaltK();                                                        // Run the function for type K thermocouple calibration
  }

  // If None of the Above, Send String to EC, pH, or DO Depending on Current ID
  else {
    if (id == 1) {                                                  // If the current ID is 1
      iEC = i;                                                      // Set the EC incoming string variable equal to the incoming string, to be sent to EC
      iEC_complete = true;                                          // Indicate that there is a string to be sent to EC
    }
    else if (id == 2) {                                             // If the current ID is 2
      ipH = i;                                                      // Set the pH incoming string variable equal to the incoming string, to be sent to pH
      ipH_complete = true;                                          // Indicate that there is a string to be sent to pH
    }
    else if (id == 3) {                                             // If the current ID is 3
      iDO = i;                                                      // Set the DO incoming string variable equal to the incoming string, to be sent to DO
      iDO_complete = true;                                          // Indicate that there is a string to be sent to DO
    }
  }
}

// Get Data from EC
void serialEvent1() {                                               // Triggered if there is an incoming string from EC
  sEC = "";                                                         // Clear the EC sensor string variable
  sEC = Serial1.readStringUntil(13);                                // Store the incoming string in the EC sensor string variable, to be sent to the serial and/or logged
  sEC_complete = true;                                              // Indicate that there is a string to be sent to the serial and/or data to be logged
  if (sEC.length() > 4) {                                           // If there is an error and the data is too long (two data points mashed together)
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
  t_J = ads1115.readADC_SingleEnded(TypeJ_PIN);                     // Read the analog value from the thermocouple
  tC_J = tm_J * t_J + tb_J;                                         // Using the calibration parameters and an assumed linear relationship, calculate temperature in deg. C from the average analog value

  // Get Type K Thermocouple Data
  t_K = ads1115.readADC_SingleEnded(TypeK_PIN);                     // Read the analog value from the thermocouple
  tC_K = tm_K * t_K + tb_K;                                         // Using the calibration parameters and an assumed linear relationship, calculate temperature in deg. C of the average analog value

  // Print Data to Serial
  if ((ll + lr) <= millis() && millis() >= 5000) {                  // Check if it's time for next data point
    if (sEC_complete == true) {                                     // If there is a string to read from EC
      Serial.print(sEC);                                            // Log EC string/data
      Serial.print('\t');
      sEC_complete = false;                                         // Indicate string from EC was read
    }
    else {                                                          // If there is not a string to read from EC
      Serial.print('\t');                                           // Log nothing
    }
    if (spH_complete == true) {                                     // If there is a string to read from pH
      Serial.print(spH);                                            // Log pH string/data
      Serial.print('\t');
      spH_complete = false;                                         // Indicate string from pH was read
    }
    else {                                                          // If there is not a string to read from pH
      Serial.print('\t');                                           // Log nothing
    }
    if (sDO_complete == true) {                                     // If there is a string to read from DO
      Serial.print(sDO);                                            // Log DO string/data
      Serial.print('\t');
      sDO_complete = false;                                         // Indicate string from DO was read
    }
    else {                                                          // If there is not a string to read from DO
      Serial.print('\t');                                           // Log nothing
    }
    Serial.print(nC);                                               // Log the nitrate concentration in mg/L
    Serial.print('\t');
    Serial.print(cC);                                               // Log the chloride concentration in mg/L
    Serial.print('\t');
    Serial.print(tC_J);                                             // Log the type J thermocouple temperature in deg. C
    Serial.print('\t');
    Serial.println(tC_K);                                           // Log the type K thermocouple temperature in deg. C
    ll = millis();                                                  // Save the time as the time of the last log
  }

  // Send String to EC
  if (iEC_complete == true) {                                       // If there is a string to send to EC
    Serial1.print(iEC);                                             // Send the string to EC
    Serial1.print('\r');                                            // Send a return indicating string is complete
    iEC = "";                                                       // Clear the EC incoming string variable
    iEC_complete = false;                                           // Indicate string was sent to EC
  }

  //Send String to pH
  if (ipH_complete == true) {                                       // If there is a string to send to pH
    Serial2.print(ipH);                                             // Send the string to pH
    Serial2.print('\r');                                            // Send a return indicating string is complete
    ipH = "";                                                       // Clear the pH incoming string variable
    ipH_complete = false;                                           // Indicate string was sent to pH
  }

  // Send String to DO
  if (iDO_complete == true) {                                       // If there is a string to send to DO
    Serial3.print(iDO);                                             // Send the string to DO
    Serial3.print('\r');                                            // Send a return indicating string is complete
    iDO = "";                                                       // Clear the DO incoming string variable
    iDO_complete = false;                                           // Indicate string was sent to DO
  }
}

// Nitrate Calibration
void Caln() {                                                       // Triggered when nitrate calibration fuction is run
  Serial.println("Soak electrode in High Standard solution for 30 minutes");

  // Calibrate Higher Value
  Serial.println("Enter concentration of high standard solution (mg/L)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new high standard value
  nh = i.toFloat();                                                 // Convert to a number
  if (SD.exists("nh.csv")) {                                        // If a file already exists,
    SD.remove("nh.csv");                                            // delete the old file
  }
  File f_nh = SD.open("nh.csv", FILE_WRITE);                        // Create and open a new file
  if (f_nh) {                                                       // If the file was opened
    f_nh.print(nh);                                                 // Save the new high standard value
    f_nh.close();                                                   // Close the file
    Serial.print("high standard: ");                                // Print the high standard value to the serial
    Serial.println(nh);
  }
  Serial.println("Send any key when ready to calibrate high standard solution");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    n = analogRead(Nitrate_PIN);                                    // While waiting, get the analog value from nitrate
    Serial.println(n);                                              // and print the analog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  nvh = analogRead(Nitrate_PIN);                                    // Once a key has been pressed, get new calibration value
  if (SD.exists("nvh.csv")) {                                       // If a file already exists,
    SD.remove("nvh.csv");                                           // delete the old file
  }
  File f_nvh = SD.open("nvh.csv", FILE_WRITE);                      // Create and open a new file
  if (f_nvh) {                                                      // If the file was opened
    f_nvh.print(nvh);                                               // Save the new calibration value
    f_nvh.close();                                                  // Close the file
    Serial.print("nvh = ");                                         // Print the new calibration value to the serial
    Serial.println(nvh);
  }

  // Calibrate Lower value
  Serial.println("Enter concentration of low standard solution (mg/L)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new low standard value
  nl = i.toFloat();                                                 // Convert to a number
  if (SD.exists("nl.csv")) {                                        // If a file already exists,
    SD.remove("nl.csv");                                            // delete the old file
  }
  File f_nl = SD.open("nl.csv", FILE_WRITE);                        // Create and open a new file
  if (f_nl) {                                                       // If the file was opened
    f_nl.print(nl);                                                 // Save the new low standard value
    f_nl.close();                                                   // Close the file
    Serial.print("low standard: ");                                 // Print the low standard value to the serial
    Serial.println(nl);
  }
  Serial.println("Send any key when ready to calibrate low standard solution");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    n = analogRead(Nitrate_PIN);                                    // While waiting, get the analog value from nitrate
    Serial.println(n);                                              // and print the analog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  nvl = analogRead(Nitrate_PIN);                                    // Once a key has been pressed, get new calibration value
  if (SD.exists("nvl.csv")) {                                       // If a file already exists,
    SD.remove("nvl.csv");                                           // delete the old file
  }
  File f_nvl = SD.open("nvl.csv", FILE_WRITE);                      // Create and open a new file
  if (f_nvl) {                                                      // If the file was opened
    f_nvl.print(nvl);                                               // Save the new calibration value
    f_nvl.close();                                                  // Close the file
    Serial.print("nvl = ");                                         // Print the new calibration value to the serial
    Serial.println(nvl);
  }

  // Calculate new calibration parameters
  nvhmV = 137.55 * (nvh / 1023.0 * 5.0) - 0.1682;
  nvlmV = 137.55 * (nvl / 1023.0 * 5.0) - 0.1682;
  nm = (nvhmV - nvlmV) / log(nh / nl);
  nEo = nvlmV - (nm * log(nl));

  Serial.println("Nitrate Calibration Done");
}

// Chloride Calibration
void Calc() {                                                       // Triggered when chloride calibration function is run
  Serial.println("Soak electrode in high standard solution for 30 minutes");

  // Calibrate Higher Value
  Serial.println("Enter concentration of high standard solution (mg/L)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new high standard value
  ch = i.toFloat();                                                 // Convert to a number
  if (SD.exists("ch.csv")) {                                        // If a file already exists,
    SD.remove("ch.csv");                                            // delete the old file
  }
  File f_ch = SD.open("ch.csv", FILE_WRITE);                        // Create and open a new file
  if (f_ch) {                                                       // If the file was opened
    f_ch.print(ch);                                                 // Save the new high standard value
    f_ch.close();                                                   // Close the file
    Serial.print("high standard: ");                                // Print the high standard value to the serial
    Serial.println(ch);
  }
  Serial.println("Send any key when ready to calibrate high standard solution");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    c = analogRead(Chloride_PIN);                                   // While waiting, get the analog value from chloride
    Serial.println(c);                                              // and print the analog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  cvh = analogRead(Chloride_PIN);                                   // Get new calibration value
  if (SD.exists("cvh.csv")) {                                       // If a file already exists,
    SD.remove("cvh.csv");                                           // delete the old file
  }
  File f_cvh = SD.open("cvh.csv", FILE_WRITE);                      // Create and open a new file
  if (f_cvh) {                                                      // If the file was opened
    f_cvh.print(cvh);                                               // Save the new calibration value
    f_cvh.close();                                                  // Close the file
    Serial.print("cvh = ");                                         // Print the new calibration value to the serial
    Serial.println(cvh);
  }

  // Calibrate Lower Value
  Serial.println("Enter concentration of low standard solution (mg/L)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new low standard value
  cl = i.toFloat();                                                 // Convert to a number
  if (SD.exists("cl.csv")) {                                        // If a file already exists,
    SD.remove("cl.csv");                                            // delete the old file
  }
  File f_cl = SD.open("cl.csv", FILE_WRITE);                        // Create and open a new file
  if (f_cl) {                                                       // If the file was opened
    f_cl.print(cl);                                                 // Save the new low standard value
    f_cl.close();                                                   // Close the file
    Serial.print("low standard: ");                                 // Print the low standard value to the serial
    Serial.println(cl);
  }
  Serial.println("Send any key when ready to calibrate low standard solution");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    c = analogRead(Chloride_PIN);                                   // While waiting, get the analog value from chloride
    Serial.println(c);                                              // and print the analog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  cvl = analogRead(Chloride_PIN);                                   // Get new calibration value
  if (SD.exists("cvl.csv")) {                                       // If a file already exists,
    SD.remove("cvl.csv");                                           // delete the old file
  }
  File f_cvl = SD.open("cvl.csv", FILE_WRITE);                      // Create and open a new file
  if (f_cvl) {                                                      // If the file was opened
    f_cvl.print(cvl);                                               // Save the new calibration value
    f_cvl.close();                                                  // Close the file
    Serial.print("cvl = ");                                         // Print the new calibration value to the serial
    Serial.println(cvl);
  }

  // Calculate new calibration parameters
  cvhmV = 137.55 * (cvh / 1023.0 * 5.0) - 0.1682;
  cvlmV = 137.55 * (cvl / 1023.0 * 5.0) - 0.1682;
  cm = (cvhmV - cvlmV) / log(ch / cl);
  cEo = cvlmV - (cm * log(cl));

  Serial.println("Chloride Calibration Done");
}

// Type J Thermocouple Calibration
void CaltJ() {                                                      // Triggered when type J thermocouple calibration function is run
  // Calibrate Lower Value
  Serial.println("Enter Low Temperature (deg. C)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new temperature value
  tl_J = i.toFloat();                                               // Convert to a number
  if (SD.exists("tl_J.csv")) {                                      // If a file already exists,
    SD.remove("tl_J.csv");                                          // delete the old file
  }
  File f_tl_J = SD.open("tl_J.csv", FILE_WRITE);                    // Create and open a new file
  if (f_tl_J) {                                                     // If the file was opened
    f_tl_J.print(tl_J);                                             // Save the new temperature value
    f_tl_J.close();                                                 // Close the file
    Serial.print("low temp: ");                                     // Print the low temperature value to the serial
    Serial.println(tl_J);
  }
  Serial.println("Send any key when ready to calibrate low temperature");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    t_J = ads1115.readADC_SingleEnded(TypeJ_PIN);                   // While waiting, get the analog value from type J thermocouple
    Serial.println(t_J);                                            // and print the anlog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  tvl_J = ads1115.readADC_SingleEnded(TypeJ_PIN);                   // Get new calibration value
  if (SD.exists("tvl_J.csv")) {                                     // If a file already exists,
    SD.remove("tvl_J.csv");                                         // delete the old file
  }
  File f_tvl_J = SD.open("tvl_J.csv", FILE_WRITE);                  // Create and open a new file
  if (f_tvl_J) {                                                    // If the file was opened
    f_tvl_J.print(tvl_J);                                           // Save the new calibration value
    f_tvl_J.close();                                                // Close the file
    Serial.print("tvl_J = ");                                       // Print the new calibration value to the serial
    Serial.println(tvl_J);
  }

  // Calibrate Higher Value
  Serial.println("Enter High Temperature (deg. C)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new temperature value
  th_J = i.toFloat();                                               // Convert to a number
  if (SD.exists("th_J.csv")) {                                      // If a file alrady exists,
    SD.remove("th_J.csv");                                          // delete the old file
  }
  File f_th_J = SD.open("th_J.csv", FILE_WRITE);                    // Create and open a new file
  if (f_th_J) {                                                     // If the file was opened
    f_th_J.print(th_J);                                             // Save the new temperature value
    f_th_J.close();                                                 // Close the file
    Serial.print("high temp: ");                                    // Print the high temperature value to the serial
    Serial.println(th_J);
  }
  Serial.println("Send any key when ready to calibrate high temperature");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    t_J = ads1115.readADC_SingleEnded(TypeJ_PIN);                   // While waiting, get the anlog value from type J thermocouple
    Serial.println(t_J);                                            // and print the analog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  tvh_J = ads1115.readADC_SingleEnded(TypeJ_PIN);                   // Get new calibration value
  if (SD.exists("tvh_J.csv")) {                                     // If a file already exists,
    SD.remove("tvh_J.csv");                                         // delete the old file
  }
  File f_tvh_J = SD.open("tvh_J.csv", FILE_WRITE);                  // Create and open a new file
  if (f_tvh_J) {                                                    // If the file was opened
    f_tvh_J.print(tvh_J);                                           // Save the new calibration value
    f_tvh_J.close();                                                // Close the file
    Serial.print("tvh_J = ");                                       // Print the new calibration value to the serial
    Serial.println(tvh_J);
  }

  // Calculate new calibration parameters
  tm_J = (th_J - tl_J) / (tvh_J - tvl_J);
  tb_J = th_J - (tm_J * tvh_J);

  Serial.println("Type J Thermocouple Calibration Done");
}

// Type K Thermocouple Calibration
void CaltK() {                                                      // Triggered when type K thermocouple calibration function is run
  // Calibrate Lower Value
  Serial.println("Enter Low Temperature (deg. C)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new temperature value
  tl_K = i.toFloat();                                               // Convert to a number
  if (SD.exists("tl_K.csv")) {                                      // If a file already exists,
    SD.remove("tl_K.csv");                                          // delete the old file
  }
  File f_tl_K = SD.open("tl_K.csv", FILE_WRITE);                    // Create and open a new file
  if (f_tl_K) {                                                     // If the file was opened
    f_tl_K.print(tl_K);                                             // Save the new temperature value
    f_tl_K.close();                                                 // Close the file
    Serial.print("low temp: ");                                     // Print the low temperature value to the serial
    Serial.println(tl_K);
  }
  Serial.println("Send any key when ready to calibrate low temperature");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    t_K = ads1115.readADC_SingleEnded(TypeK_PIN);                   // While waiting, get the analog value from type K thermocouple
    Serial.println(t_K);                                            // and print the anlog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  tvl_K = ads1115.readADC_SingleEnded(TypeK_PIN);                   // Get new calibration value
  if (SD.exists("tvl_K.csv")) {                                     // If a file already exists,
    SD.remove("tvl_K.csv");                                         // delete the old file
  }
  File f_tvl_K = SD.open("tvl_K.csv", FILE_WRITE);                  // Create and open a new file
  if (f_tvl_K) {                                                    // If the file was opened
    f_tvl_K.print(tvl_K);                                           // Save the new calibration value
    f_tvl_K.close();                                                // Close the file
    Serial.print("tvl_K = ");                                       // Print the new calibration value to the serial
    Serial.println(tvl_K);
  }

  // Calibrate Higher Value
  Serial.println("Enter High Temperature (deg. C)");
  while (!Serial.available()) {                                     // Wait for a value
  }
  i = Serial.readStringUntil(13);                                   // Get new temperature value
  th_K = i.toFloat();                                               // Convert to a number
  if (SD.exists("th_K.csv")) {                                      // If a file alrady exists,
    SD.remove("th_K.csv");                                          // delete the old file
  }
  File f_th_K = SD.open("th_K.csv", FILE_WRITE);                    // Create and open a new file
  if (f_th_K) {                                                     // If the file was opened
    f_th_K.print(th_K);                                             // Save the new temperature value
    f_th_K.close();                                                 // Close the file
    Serial.print("high temp: ");                                    // Print the high temperature value to the serial
    Serial.println(th_K);
  }
  Serial.println("Send any key when ready to calibrate high temperature");
  while (!Serial.available()) {                                     // Wait for anything to be sent from serial
    t_K = ads1115.readADC_SingleEnded(TypeK_PIN);                   // While waiting, get the anlog value from type K thermocouple
    Serial.println(t_K);                                            // and print the analog value to the serial
    delay(500);                                                     // every half second
  }
  i = Serial.readStringUntil(13);                                   // Read the key that was sent to clear the serial buffer (or it will just wait there to be read)
  tvh_K = ads1115.readADC_SingleEnded(TypeK_PIN);                   // Get new calibration value
  if (SD.exists("tvh_K.csv")) {                                     // If a file already exists,
    SD.remove("tvh_K.csv");                                         // delete the old file
  }
  File f_tvh_K = SD.open("tvh_K.csv", FILE_WRITE);                  // Create and open a new file
  if (f_tvh_K) {                                                    // If the file was opened
    f_tvh_K.print(tvh_K);                                           // Save the new calibration value
    f_tvh_K.close();                                                // Close the file
    Serial.print("tvh_K = ");                                       // Print the new calibration value to the serial
    Serial.println(tvh_K);
  }

  // Calculate new calibration parameters
  tm_K = (th_K - tl_K) / (tvh_K - tvl_K);
  tb_K = th_K - (tm_K * tvh_K);

  Serial.println("Type K Thermocouple Calibration Done");
}

