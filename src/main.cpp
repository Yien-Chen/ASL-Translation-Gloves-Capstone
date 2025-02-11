#include <ArduinoBLE.h>

// Define BLE Service and Characteristics globally
BLEService sensorService("180C"); // Custom Service UUID
BLEStringCharacteristic sensorCharacteristic("2A57", BLERead | BLENotify, 50); // Allows sending strings (up to 50 bytes)


void setup() {
  Serial.begin(9600);  // Begin serial communication for debugging

  // Remove while(!Serial) so the BLE stack initializes immediately
  // while (!Serial);  // This line is no longer needed

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);  // Halt the program if BLE initialization fails
  }

  // Set device name (visible in BLE scanning apps)
  BLE.setDeviceName("Arduino_BLE_Sensor");
  BLE.setLocalName("Arduino_BLE_Sensor");

  // Add characteristic to service
  sensorService.addCharacteristic(sensorCharacteristic);
  BLE.addService(sensorService);

  // Start advertising
  BLE.setAdvertisingInterval(100);  // 100ms advertising interval
  BLE.advertise();  // Begin advertising the BLE service
  Serial.println("BLE is now advertising as 'Arduino_BLE_Sensor'!");
}

void loop() {
  BLEDevice central = BLE.central(); // Wait for a connection
  BLE.advertise();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) { // Keep sending data while connected
      int thumb = analogRead(A7);
      int index = analogRead(A5);
      int middle = analogRead(A3);
      int ring = analogRead(A1);
      int pinky = analogRead(A0);

      // Format sensor values into a string
      String sensorData = "Thumb:" + String(thumb) + ", " +
                          "Index:" + String(index) + ", " +
                          "Middle:" + String(middle) + ", " +
                          "Ring:" + String(ring) + ", " +
                          "Pinky:" + String(pinky);

      // Send data over BLE
      sensorCharacteristic.writeValue(sensorData);
      Serial.println(sensorData);
      delay(500);
    }

    Serial.println("Disconnected from central");
  }
}
