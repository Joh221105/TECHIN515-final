#include <Arduino.h>

void setup() {
  Serial.begin(9600);   
  while (!Serial) {}  
  Serial.println("Toolchain ready!");
}

void loop() {
  Serial.println("Hello World");
  delay(1000);
}