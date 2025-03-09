#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>  // Include the IMU library

// Define BLE Service and Characteristics
BLEService sensorService("180C");
// Increased buffer size to accommodate IMU data
BLEStringCharacteristic sensorCharacteristic("2A57", BLERead | BLENotify, 150);

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
      // Read finger sensors
      int thumb = analogRead(A7);
      int index = analogRead(A5);
      int middle = analogRead(A3);
      int ring = analogRead(A1);
      int pinky = analogRead(A0);

      // Read IMU data
      float accX, accY, accZ;
      float gyroX, gyroY, gyroZ;
      float magX, magY, magZ;
      
      // Get acceleration (m/s²)
      IMU.readAcceleration(accX, accY, accZ);
      
      // Get angular velocity (rad/s)
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      
      // Get magnetic field (μT)
      IMU.readMagneticField(magX, magY, magZ);

      // Create combined data string
      String sensorData = 
        "Fingers:" + String(thumb) + "," + String(index) + "," + 
        String(middle) + "," + String(ring) + "," + String(pinky) + "|" +
        "Acc:" + String(accX, 2) + "," + String(accY, 2) + "," + String(accZ, 2) + "|" +
        "Gyro:" + String(gyroX, 2) + "," + String(gyroY, 2) + "," + String(gyroZ, 2) + "|" +
        "Mag:" + String(magX, 2) + "," + String(magY, 2) + "," + String(magZ, 2);

      // Send data
      sensorCharacteristic.writeValue(sensorData);
      Serial.println(sensorData);
      delay(200);  // Reduced delay for better IMU responsiveness
    }

    Serial.println("Disconnected from central");
  }
}