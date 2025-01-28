#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for the Serial Monitor to be ready
}

void loop() {
  int thumb = analogRead(A0);
  int index = analogRead(A1);
  int middle = analogRead(A2);
  int ring = analogRead(A3);
  int pinky = analogRead(A4);

  Serial.print("Thumb: ");
  Serial.print(thumb);
  Serial.print("\tIndex: ");
  Serial.print(index);
  Serial.print("\tMiddle: ");
  Serial.print(middle);
  Serial.print("\tRing: ");
  Serial.print(ring);
  Serial.print("\tPinky: ");
  Serial.println(pinky);

  delay(500);
}
