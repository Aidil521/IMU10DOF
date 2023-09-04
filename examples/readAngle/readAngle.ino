#include <IMU10DOF.h>

uint32_t oldT;

void setup() {
  Serial.begin(115200);
  IMU.begin();
  IMU.calcOffsets();
}

void loop() {
  // put your main code here, to run repeatedly:
  float Roll, Pitch, Yaw;
  
  IMU.update(); //update measurement IMU Sensor

  if ((millis() - oldT) > 500) {
    oldT    = millis();

    //read angle x, y, z
    Roll  = IMU.getAngleX();
    Pitch = IMU.getAngleY();
    Yaw   = IMU.getAngleZ(); 

    //Display for serial monitor
    Serial.println("Roll    : " + String(Roll));
    Serial.println("Pitch   : " + String(Pitch));
    Serial.println("Yaw     : " + String(Yaw));
    Serial.println();
  }
}