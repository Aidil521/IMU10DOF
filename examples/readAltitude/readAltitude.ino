#include <IMU10DOF.h>

uint32_t oldT;

void setup() {
  Serial.begin(115200);
  IMU.begin();
  IMU.calcOffsets();
}

void loop() {
  // put your main code here, to run repeatedly:
  float Altitude;

  IMU.update(); //update measurement IMU Sensor
  if ((millis() - oldT) > 500) {
    oldT = millis();

    //read value altitude
    Altitude = IMU.getAltitude();

    //Display for serial monitor
    Serial.println("Altitude : " + String(Altitude));
    Serial.println();
  }
}