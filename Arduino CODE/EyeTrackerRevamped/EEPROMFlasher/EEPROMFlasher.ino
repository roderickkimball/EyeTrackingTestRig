#include <EEPROM.h>

long int defaultPosition = 0;
float defaultAngle = 0.0;

void setup() {
  // put your setup code here, to run once:
  defaultPosition = 0;
  // putting default stepper positions in EEPROM
  EEPROM.put(0, defaultPosition);
  EEPROM.put(4, defaultPosition);
  EEPROM.put(8, defaultPosition);
  
  defaultPosition = 1500;
  // putting eye center positions in EEPROM
  EEPROM.put(12, defaultPosition);
  EEPROM.put(16, defaultPosition);
  EEPROM.put(20, defaultPosition);
  EEPROM.put(24, defaultPosition);

  // putting default neck parameters in EEPROM
  defaultAngle = 0.0;
  EEPROM.put(28, defaultAngle);
  EEPROM.put(32, defaultAngle);
  EEPROM.put(36, defaultAngle);
  EEPROM.put(40, defaultPosition);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("EEPROM variables loaded successfully!");
}
