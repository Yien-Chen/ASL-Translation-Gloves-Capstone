#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>  // Include the IMU library

// Threshold constants
const int THRESHOLD_NOT_BENT  = 0;
const int THRESHOLD_HALF_BENT = 500;
const int THRESHOLD_FULLY_BENT = 1000;

// Define BLE Service and Characteristics
BLEService sensorService("180C");
// Increased buffer size to accommodate IMU data
BLEStringCharacteristic sensorCharacteristic("2A57", BLERead | BLENotify, 150);

String encode_finger_readings(const int fingerReadings[5]) {
  uint16_t code = 0;

  for (int i = 0; i < 5; i++) {
    int classification;
    if (fingerReadings[i] < THRESHOLD_HALF_BENT) {
      classification = 0;  // 00
    } else if (fingerReadings[i] < THRESHOLD_FULLY_BENT) {
      classification = 1;  // 01
    } else if (fingerReadings[i] >= THRESHOLD_FULLY_BENT) {
      classification = 2;  // 10
    } else {
      classification = 3;  // 11 (default/error)
    }

    uint16_t bits = 0;
    switch (classification) {
      case 0:
        bits = 0; // 00
        break;
      case 1:
        bits = 1; // 01
        break;
      case 2:
        bits = 2; // 10
        break;
      default:
        bits = 3; // 11 (error)
        break;
    }

    // Shift the current code to the left by 2 to make room for this finger's bits
    code <<= 2;
    code |= bits;
  }

  // Convert the 10-bit value in 'code' to a 10-character binary string
  String bitString;
  for (int i = 9; i >= 0; i--) {
    bitString += ((code >> i) & 1) ? "1" : "0";
  }

  return bitString;
}

void setup() {
  Serial.begin(9600);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  BLE.setDeviceName("Arduino_BLE_Sensor");
  BLE.setLocalName("Arduino_BLE_Sensor");

  sensorService.addCharacteristic(sensorCharacteristic);
  BLE.addService(sensorService);

  BLE.setAdvertisingInterval(100);
  BLE.advertise();
  Serial.println("BLE is now advertising with IMU data!");
}

void loop() {
  BLEDevice central = BLE.central();
  BLE.advertise();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Read finger sensors into an array
      int fingerReadings[5];
      fingerReadings[0] = analogRead(A7); // thumb
      fingerReadings[1] = analogRead(A5); // index
      fingerReadings[2] = analogRead(A3); // middle
      fingerReadings[3] = analogRead(A1); // ring
      fingerReadings[4] = analogRead(A0); // pinky

      // Encode the finger readings into a 10-bit binary string
      String encoded_bitstream = encode_finger_readings(fingerReadings);
      sensorCharacteristic.writeValue(encoded_bitstream);
      Serial.println(encoded_bitstream);
      // Read IMU data
      float accX, accY, accZ;
      float gyroX, gyroY, gyroZ;
      float magX, magY, magZ;
      
      IMU.readAcceleration(accX, accY, accZ);
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      IMU.readMagneticField(magX, magY, magZ);

      // Create combined data string
      String sensorData = 
          "Fingers:" +
          String(fingerReadings[0]) + "," +
          String(fingerReadings[1]) + "," +
          String(fingerReadings[2]) + "," +
          String(fingerReadings[3]) + "," +
          String(fingerReadings[4]) + "|" +
          "Acc:" + String(accX, 2) + "," + String(accY, 2) + "," + String(accZ, 2) + "|" +
          "Gyro:" + String(gyroX, 2) + "," + String(gyroY, 2) + "," + String(gyroZ, 2) + "|" +
          "Mag:" + String(magX, 2) + "," + String(magY, 2) + "," + String(magZ, 2);

      // Send data over BLE characteristic
      // sensorCharacteristic.writeValue(sensorData);
      //Serial.println(sensorData);

      delay(200); // A delay to control how often data is sent
    }

    Serial.println("Disconnected from central");
  }
}
