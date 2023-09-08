#include <IMU10DOF.h>

uint32_t oldT;

void setup() {
  Serial.begin(115200);
  IMU.begin();
  IMU.setDeclination(0, 15);
  IMU.setGyroOffsets(-2.08, 0.51, -0.15);
  IMU.setAccOffsets(0.06, -0.02, -0.08);
  IMU.calcOffsets(false);
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.update(); //update measurement IMU Sensor

  if ((millis() - oldT) > 500) {
    oldT = millis();
    Serial.println("Roll     : " + String(IMU.getAngleX()));
    Serial.println("Pitch    : " + String(IMU.getAngleY()));
    Serial.println("Yaw      : " + String(IMU.getAngleZ()));
    Serial.println("Heading  : " + String(IMU.getHeading()));
    Serial.println("Compass  : " + String(IMU.getAzimuth()));
    Serial.println("Altitude : " + String(IMU.getAltitude()));
    Serial.println("Pressure : " + String(IMU.getPressure()));
    Serial.println("Temp     : " + String(IMU.getTemperature()));
    Serial.println();
  }
}