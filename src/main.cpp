
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino.h>
#include <Thread.h>
#include <DFRobotDFPlayerMini.h>

// ---------------------------------------------------------------------------
//     GLOBAL SETTINGS & CONSTANTS
// ---------------------------------------------------------------------------
const bool BLUETOOTH_ENABLED   = false;
const bool CALIBRATION_ENABLED = false;

// DFPlayer Mini declarations
DFRobotDFPlayerMini mp3;
const int MP3_BUSY_PIN = 4;  // Busy pin connected to Arduino pin 4
#define MP3_SERIAL Serial1    // Using hardware Serial1 (pins 13/RX, 14/TX)

// Gesture Enumeration
enum Gesture {
  GESTURE_UNKNOWN,
  GESTURE_HELLO,
  GESTURE_OKAY,
  GESTURE_LOVE,
  GESTURE_YES
};

// Threshold Values
int THUMB_LOW   = 100;
int INDEX_LOW   = 100;
int MIDDLE_LOW  = 150;
int RING_LOW    = 150;
int PINKY_LOW   = 200;

int THUMB_MID   = 200;
int INDEX_MID   = 300;
int MIDDLE_MID  = 300;
int RING_MID    = 300;
int PINKY_MID   = 500;

int THUMB_HIGH  = 600;
int INDEX_HIGH  = 700;
int MIDDLE_HIGH = 700;
int RING_HIGH   = 700;
int PINKY_HIGH  = 750;

// BLE Service and Characteristics
BLEService sensorService("180C");
BLEStringCharacteristic sensorCharacteristic("2A57", BLERead | BLENotify, 150);

// Global Variables
int calibrationOffsets[5] = {0, 0, 0, 0, 0};
String consumerBitstreams[5];
int consumerBitstreamsCount = 0;

// ---------------------------------------------------------------------------
//     FUNCTION DECLARATIONS
// ---------------------------------------------------------------------------
void initIMU();
void initBluetooth();
bool checkForCentral(bool &runReadingLoop);
void calibrateSensors();
void readFingerSensors(int fingerReadings[5]);
void getFingerThresholds(int fingerIndex, int &lowVal, int &midVal, int &highVal);
String encodeFingerReadings(const int fingerReadings[5]);
void readIMU(float &accX, float &accY, float &accZ,
             float &gyroX, float &gyroY, float &gyroZ,
             float &magX, float &magY, float &magZ);
String buildSensorData(const int fingerReadings[5],
                       const String &encodedFingers,
                       float accX, float accY, float accZ,
                       float gyroX, float gyroY, float gyroZ,
                       float magX, float magY, float magZ);
void readAndSendData();
void initDFPlayer();
void playGestureSound(Gesture gesture);
bool isMP3Busy();
Gesture GestureFromBits(uint16_t gestureBits);

// ---------------------------------------------------------------------------
//     THREADS
// ---------------------------------------------------------------------------
void producer() {
  readAndSendData();
}

int binaryStringToInt(String s) {
  int result = 0;
  for (int i = 0; i < s.length(); i++) {
    result = result * 2 + (s.charAt(i) - '0');
  }
  return result;
}

int computeMode(int arr[], int n) {
  int frequency[3] = {0};
  for (int i = 0; i < n; i++) {
    int val = arr[i];
    frequency[val]++;      
  }
  int mode = 0;
  int maxCount = frequency[0];
  for (int i = 1; i < 3; i++) {
    if (frequency[i] > maxCount) {
      maxCount = frequency[i];
      mode = i;
    }
  }
  return mode;
}

uint16_t buildGestureInt(const int fingerModes[5]) {
  uint16_t gestureBits = 0;
  gestureBits |= (fingerModes[0] & 0x3) << 8;
  gestureBits |= (fingerModes[1] & 0x3) << 6;
  gestureBits |= (fingerModes[2] & 0x3) << 4;
  gestureBits |= (fingerModes[3] & 0x3) << 2;
  gestureBits |= (fingerModes[4] & 0x3) << 0;
  return gestureBits;
}

Gesture GestureFromBits(uint16_t gestureBits) {
  Gesture currentGesture = GESTURE_UNKNOWN;
  
  switch(gestureBits) {
    case 0b0000000000:
      Serial.println("Hello");
      currentGesture = GESTURE_HELLO;
      break;
    case 0b0101000000:
      Serial.println("Okay");
      currentGesture = GESTURE_OKAY;
      break;
    case 0b0000101000:
      Serial.println("I Love You");
      currentGesture = GESTURE_LOVE;
      break;
    case 0b0010101010:
    case 0b0110101010:
      Serial.println("Yes");
      currentGesture = GESTURE_YES;
      break;
  }

  if(currentGesture != GESTURE_UNKNOWN && !isMP3Busy()) {
    playGestureSound(currentGesture);
  }
  
  return currentGesture;
}

void consumer() {
  if (consumerBitstreamsCount < 5) return;
  
  int fingerModes[5];
  
  for (int finger = 0; finger < 5; finger++) {
    int states[5];  
    
    for (int reading = 0; reading < 5; reading++) {
      int startIndex = (finger == 0) ? 0 : finger * 3;
      String pairStr = consumerBitstreams[reading].substring(startIndex, startIndex + 2);
      states[reading] = binaryStringToInt(pairStr);
    }
    
    fingerModes[finger] = computeMode(states, 5);
  }
  
  uint16_t finalBits = buildGestureInt(fingerModes);
  GestureFromBits(finalBits);
  consumerBitstreamsCount = 0;
}

Thread producerThread(producer, 400);
Thread consumerThread(consumer, 450);

// ---------------------------------------------------------------------------
//     SETUP
// ---------------------------------------------------------------------------
// Set volume (0-30)


void setup() {  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect - CRUCIAL FOR NANO 33 BLE
  
  // Add debug message to confirm serial is working
  Serial.println("Serial communication initialized");
  
  // Rest of your setup code...
  pinMode(MP3_BUSY_PIN, INPUT_PULLUP);
  initDFPlayer();

  mp3.volume(30);
  if (CALIBRATION_ENABLED) {
    calibrateSensors();
  }

  initIMU();

  if (BLUETOOTH_ENABLED) {
    initBluetooth();
  } else {
    Serial.println("Bluetooth is DISABLED. Skipping BLE setup.");
  }
}

// ---------------------------------------------------------------------------
//     LOOP
// ---------------------------------------------------------------------------
void loop() {
  bool runReadingLoop = false;

  if (BLUETOOTH_ENABLED) {
    checkForCentral(runReadingLoop);
    while (runReadingLoop) {
      producerThread.run();
      consumerThread.run();
      if (!BLE.central().connected()) {
        runReadingLoop = false;
        Serial.println("Disconnected from central");
      }
      delay(200);
    }
  } else {
    producerThread.run();
    consumerThread.run();
    delay(200);
  }
}

// ---------------------------------------------------------------------------
//     DFPlayer Functions
// ---------------------------------------------------------------------------
#define MP3_SERIAL Serial1  // Use hardware Serial1

void initDFPlayer() {
  MP3_SERIAL.begin(9600);
  Serial.println("Initializing DFPlayer...");
  delay(1000); // Add initial delay
  
  if(!mp3.begin(MP3_SERIAL)) {
    Serial.println("DFPlayer initialization failed!");
    Serial.println("Check connections and reset");
    while(1) { // Blink LED if available
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }
  
  mp3.volume(30);
  Serial.println("DFPlayer ready");
  delay(500); // Add post-init delay
}

bool isMP3Busy() {
  return digitalRead(MP3_BUSY_PIN) == LOW;
}

void playGestureSound(Gesture gesture) {
  switch(gesture) {
    case GESTURE_HELLO:
      mp3.play(1);  // Play track 0001.mp3
      break;
    case GESTURE_OKAY:
      mp3.play(2);  // Play track 0002.mp3
      break;
    case GESTURE_LOVE:
      mp3.play(3);  // Play track 0003.mp3
      break;
    case GESTURE_YES:
      mp3.play(4);  // Play track 0004.mp3
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
//  Sensor Calibration Routine
//  - The user holds their hand fully straight.
//  - Collects sensor readings over 5 seconds to calculate a baseline for each finger.
//  - Offsets the default threshold values by subtracting the baseline for index, middle, ring, and pinky.
//  - Thumb thresholds remain unchanged.
//  - Clamps any negative thresholds to 0 and prints calibration information.
// ---------------------------------------------------------------------------
void calibrateSensors() {
  Serial.println("Starting calibration. Please hold your hand fully straight.");
  const unsigned long calibrationTime = 5000; // 5 seconds
  unsigned long startTime = millis();
  long sum[5] = {0, 0, 0, 0, 0};
  int count = 0;

  // Collect samples for 5 seconds
  while (millis() - startTime < calibrationTime) {
    int fingerReadings[5];
    fingerReadings[0] = analogRead(A7);
    fingerReadings[1] = analogRead(A5);
    fingerReadings[2] = analogRead(A3);
    fingerReadings[3] = analogRead(A1);
    fingerReadings[4] = analogRead(A0);

    for (int i = 0; i < 5; i++) {
      sum[i] += fingerReadings[i];
    }
    count++;
    delay(50);  // Sample every 50ms
  }

  // Calculate and store the average (baseline) for each finger
  Serial.print("Calibration offsets (baseline): ");
  for (int i = 0; i < 5; i++) {
    calibrationOffsets[i] = sum[i] / count;
    Serial.print(calibrationOffsets[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Apply calibration to thresholds for all fingers except the thumb (finger index 0)

  // Index finger
  INDEX_LOW   -= calibrationOffsets[1];
  INDEX_MID   -= calibrationOffsets[1];
  INDEX_HIGH  -= calibrationOffsets[1];

  // Middle finger
  MIDDLE_LOW  -= calibrationOffsets[2];
  MIDDLE_MID  -= calibrationOffsets[2];
  MIDDLE_HIGH -= calibrationOffsets[2];

  // Ring finger
  RING_LOW    -= calibrationOffsets[3];
  RING_MID    -= calibrationOffsets[3];
  RING_HIGH   -= calibrationOffsets[3];

  // Pinky finger
  PINKY_LOW   -= calibrationOffsets[4];
  PINKY_MID   -= calibrationOffsets[4];
  PINKY_HIGH  -= calibrationOffsets[4];

  // Clamp any negative threshold values to 0 for index, middle, ring, and pinky
  if (INDEX_LOW < 0)   INDEX_LOW = 0;
  if (INDEX_MID < 0)   INDEX_MID = 0;
  if (INDEX_HIGH < 0)  INDEX_HIGH = 0;

  if (MIDDLE_LOW < 0)  MIDDLE_LOW = 0;
  if (MIDDLE_MID < 0)  MIDDLE_MID = 0;
  if (MIDDLE_HIGH < 0) MIDDLE_HIGH = 0;

  if (RING_LOW < 0)    RING_LOW = 0;
  if (RING_MID < 0)    RING_MID = 0;
  if (RING_HIGH < 0)   RING_HIGH = 0;

  if (PINKY_LOW < 0)   PINKY_LOW = 0;
  if (PINKY_MID < 0)   PINKY_MID = 0;
  if (PINKY_HIGH < 0)  PINKY_HIGH = 0;

  // Print the new adjusted thresholds
  Serial.println("Adjusted threshold values after calibration:");
  Serial.print("Thumb (unchanged):  LOW=");
  Serial.print(THUMB_LOW);
  Serial.print(", MID=");
  Serial.print(THUMB_MID);
  Serial.print(", HIGH=");
  Serial.println(THUMB_HIGH);

  Serial.print("Index:  LOW=");
  Serial.print(INDEX_LOW);
  Serial.print(", MID=");
  Serial.print(INDEX_MID);
  Serial.print(", HIGH=");
  Serial.println(INDEX_HIGH);

  Serial.print("Middle: LOW=");
  Serial.print(MIDDLE_LOW);
  Serial.print(", MID=");
  Serial.print(MIDDLE_MID);
  Serial.print(", HIGH=");
  Serial.println(MIDDLE_HIGH);

  Serial.print("Ring:   LOW=");
  Serial.print(RING_LOW);
  Serial.print(", MID=");
  Serial.print(RING_MID);
  Serial.print(", HIGH=");
  Serial.println(RING_HIGH);

  Serial.print("Pinky:  LOW=");
  Serial.print(PINKY_LOW);
  Serial.print(", MID=");
  Serial.print(PINKY_MID);
  Serial.print(", HIGH=");
  Serial.println(PINKY_HIGH);
}

// ----------------------
//  Initialize the IMU
// ----------------------
void initIMU() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1); // Stop here
  }
  Serial.println("IMU initialized.");
}

// ------------------------------------------
//  Initialize Bluetooth (if enabled)
// ------------------------------------------
void initBluetooth() {
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1); // Stop here
  }

  BLE.setDeviceName("Arduino_BLE_Sensor");
  BLE.setLocalName("Arduino_BLE_Sensor");

  sensorService.addCharacteristic(sensorCharacteristic);
  BLE.addService(sensorService);

  BLE.setAdvertisingInterval(100);
  BLE.advertise();

  Serial.println("BLE is now advertising with IMU data!");
}

// ------------------------------------------------------------------
//  Check for a newly connected central device
// ------------------------------------------------------------------
bool checkForCentral(bool &runReadingLoop) {
  BLE.advertise();
  BLEDevice central = BLE.central();
  if (central) {
    runReadingLoop = true;
    Serial.print("Connected to: ");
    Serial.println(central.address());
    return true;
  }
  return false;
}

// ----------------------------------------------------
//  Read finger sensors into fingerReadings[5]
//  (Subtract calibration offsets from readings)
// ----------------------------------------------------
void readFingerSensors(int fingerReadings[5]) {
  // Subtract calibration offsets from the raw sensor values
  fingerReadings[0] = analogRead(A7) - calibrationOffsets[0]; // thumb
  fingerReadings[1] = analogRead(A5) - calibrationOffsets[1]; // index
  fingerReadings[2] = analogRead(A3) - calibrationOffsets[2]; // middle
  fingerReadings[3] = analogRead(A1) - calibrationOffsets[3]; // ring
  fingerReadings[4] = analogRead(A0) - calibrationOffsets[4]; // pinky

  // Clamp negative values to 0
  for (int i = 0; i < 5; i++) {
    if (fingerReadings[i] < 0) {
      fingerReadings[i] = 0;
    }
  }
}

// ---------------------------------------------------------------------------
//  Retrieve the thresholds (low, mid, high) for a given finger index.
// ---------------------------------------------------------------------------
void getFingerThresholds(int fingerIndex, int &lowVal, int &midVal, int &highVal) {
  switch(fingerIndex) {
    case 0: // thumb
      lowVal  = THUMB_LOW;
      midVal  = THUMB_MID;
      highVal = THUMB_HIGH;
      break;
    case 1: // index
      lowVal  = INDEX_LOW;
      midVal  = INDEX_MID;
      highVal = INDEX_HIGH;
      break;
    case 2: // middle
      lowVal  = MIDDLE_LOW;
      midVal  = MIDDLE_MID;
      highVal = MIDDLE_HIGH;
      break;
    case 3: // ring
      lowVal  = RING_LOW;
      midVal  = RING_MID;
      highVal = RING_HIGH;
      break;
    case 4: // pinky
      lowVal  = PINKY_LOW;
      midVal  = PINKY_MID;
      highVal = PINKY_HIGH;
      break;
    default:
      lowVal  = 0;
      midVal  = 0;
      highVal = 0;
      break;
  }
}

// ---------------------------------------------------------------------------
//  Encode finger readings directly into a 10-bit binary string using thresholds.
// ---------------------------------------------------------------------------
String encodeFingerReadings(const int fingerReadings[5]) {
  String bitString;
  bitString.reserve(15);  // Reserve enough space for 10 bits + 4 spaces

  for (int i = 0; i < 5; i++) {
    int lowVal, midVal, highVal;
    getFingerThresholds(i, lowVal, midVal, highVal);

    int val = fingerReadings[i];

    // Determine bit pair based on thresholds
    if (val < midVal) {
      bitString += "00";
    } else if (val < highVal) {
      bitString += "01";
    } else if (val >= highVal) {
      bitString += "10";
    } else {
      bitString += "11";  // Fallback case, should not occur
    }
    
    // Add a space after each finger's bits except for the last one
    if (i < 4) {
      bitString += " ";
    }
  }
  
  return bitString;  // e.g. "00 01 10 00 01"
}

// -----------------------------------------------------
//  Read IMU data and populate references
// -----------------------------------------------------
void readIMU(float &accX, float &accY, float &accZ,
             float &gyroX, float &gyroY, float &gyroZ,
             float &magX, float &magY, float &magZ) {
  IMU.readAcceleration(accX, accY, accZ);
  IMU.readGyroscope(gyroX, gyroY, gyroZ);
  IMU.readMagneticField(magX, magY, magZ);
}

// --------------------------------------------------------------------------
//  Build the combined data string with sensor readings and IMU data
// --------------------------------------------------------------------------
String buildSensorData(const int fingerReadings[5],
                       const String &encodedFingers,
                       float accX, float accY, float accZ,
                       float gyroX, float gyroY, float gyroZ,
                       float magX, float magY, float magZ) {
  String sensorData = "Fingers:" +
                      String(fingerReadings[0]) + "," +
                      String(fingerReadings[1]) + "," +
                      String(fingerReadings[2]) + "," +
                      String(fingerReadings[3]) + "," +
                      String(fingerReadings[4]) +
                      "|Encoded:" + encodedFingers +
                      "|Acc:"   + String(accX, 2) + "," + String(accY, 2) + "," + String(accZ, 2) +
                      "|Gyro:"  + String(gyroX, 2) + "," + String(gyroY, 2) + "," + String(gyroZ, 2) +
                      "|Mag:"   + String(magX, 2) + "," + String(magY, 2) + "," + String(magZ, 2);
  return sensorData;
}

// --------------------------------------------------------------------------
//  Read all data, build the string, and send via BLE if connected
// --------------------------------------------------------------------------
void readAndSendData() {
  if(consumerBitstreamsCount < 5) {
    int fingerReadings[5];
    readFingerSensors(fingerReadings);
    consumerBitstreams[consumerBitstreamsCount] = encodeFingerReadings(fingerReadings);

    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    readIMU(accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);

    String sensorData = buildSensorData(fingerReadings, consumerBitstreams[consumerBitstreamsCount],
                                        accX, accY, accZ,
                                        gyroX, gyroY, gyroZ,
                                        magX, magY, magZ);

    if (BLUETOOTH_ENABLED && BLE.central().connected()) {
      sensorCharacteristic.writeValue(sensorData);
    }
    Serial.println(sensorData);
    consumerBitstreamsCount++;
  }
}