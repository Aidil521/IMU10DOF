#include <IMU10DOF.h>

uint32_t oldT;

void setup() {
  Serial.begin(115200);
  IMU.begin();
  IMU.calcOffsets();
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.update(); //update measurement IMU Sensor

  if ((millis() - oldT) > 500) {
    oldT = millis();
    Serial.print("GyroX     : " + String(IMU.getGyroXoffset()));
    Serial.print("\tGyroY    : " + String(IMU.getGyroYoffset()));
    Serial.println("\tGyroZ      : " + String(IMU.getGyroZoffset()));
    Serial.print("Acc X  : " + String(IMU.getAccXoffset()));
    Serial.print("\tAcc Y  : " + String(IMU.getAccYoffset()));
    Serial.println("\tAcc Z : " + String(IMU.getAccZoffset()));
    Serial.println();
  }
}