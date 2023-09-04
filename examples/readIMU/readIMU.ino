#include <IMU10DOF.h>

uint32_t oldT;

void setup() {
  Serial.begin(115200);
  IMU.begin();
  IMU.setDeclination(0, 15);
  IMU.setGyroOffsets(-1.48, 0.28, -0.21);
  IMU.setAccOffsets(0.05, -0.03, -0.07);
  IMU.calcOffsets();

}

void loop() {
  // put your main code here, to run repeatedly:
  IMU.update();
  if ((millis() - oldT) > 500) {
    oldT = millis();
    Serial.println("Roll     : " + String(IMU.getAngleX()));
    Serial.println("Pitch    : " + String(IMU.getAngleY()));
    Serial.println("Yaw      : " + String(IMU.getHeading()));
    Serial.println("Compass  : " + String(IMU.getAzimuth()));
    Serial.println("Altitude : " + String(IMU.getAltitude()));
    Serial.println("Temp     : " + String(IMU.getTemp()));
    Serial.println("Pressure : " + String(IMU.getPressure()));
    Serial.println();
  }
}