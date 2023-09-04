#include <IMU10DOF.h>

uint32_t oldT;

void setup() {
  Serial.begin(115200);
  IMU.begin();
  //setting your location with value magnetic declination
  //Link view your declination https://www.magnetic-declination.com/
  IMU.setDeclination(0, 15); // example The magnetic declination is: +0º 15'
  IMU.calcOffsets();
}

void loop() {
  // put your main code here, to run repeatedly:
  float Heading, Azimuth;

  IMU.update(); //update measurement IMU Sensor

  if ((millis() - oldT) > 500) {
    oldT = millis();

    //read value heading and azimuth
    Heading = IMU.getHeading();
    Azimuth = IMU.getAzimuth();

    //Display for serial monitor
    Serial.println("Heading : " + String(Heading));
    Serial.println("Azimuth : " + String(Azimuth));
    Serial.println();
  }
}