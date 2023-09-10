/* Library Reference
  https://github.com/rfetick/MPU6050_light.git
  https://github.com/mprograms/QMC5883LCompass.git
  https://github.com/MartinL1/BMP280_DEV.git
*/

#include "IMU10DOF.h"

float wrap(float angle, float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

float LPF(float value, float newValue) {
  return (value * LPF_OFFSET) + ((1.0f - LPF_OFFSET) * newValue);
}

IMUSensor::IMUSensor(TwoWire &w){
  wire = &w;
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
}

void IMUSensor::begin(){
  wire->begin();
  wire->setClock(400000);

  // Configuration MPU6050
  uint8_t statusMPU = writeByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REGISTER, 0x01); 
  writeByte(MPU6050_ADDR, MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeByte(MPU6050_ADDR, MPU6050_GYRO_CONFIG_REGISTER, 0x08); //65.5f
  writeByte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REGISTER, 0x10); //4096.0f
  writeByte(MPU6050_ADDR, MPU6050_CONFIG_REGISTER, 0x03);
  delay(10);

  // Configuration QMC5883L
  uint8_t statusQMC = writeByte(QMC5883L_ADDR, QMC5583L_CONFIG_REGISTER, 0x01);
  writeByte(QMC5883L_ADDR, QMC5883L_MODE_REGISTER, 0x01 | 0x0C | 0x10 | 0x00);
  delay(10);

  // Configuration BMP280
  if (readByte(BMP280_ADDR, BMP280_MDICE_ID) != DEVICE_ID);
  writeByte(BMP280_ADDR, BMP280_RESET, RESET_CODE);
  delay(10);
  readBMP(BMP280_TRIM_PARAMS, (uint8_t *)&params ,sizeof(params));
  uint8_t statusBMP = writeByte(BMP280_ADDR, BMP280_CONFIG_REGISTER, 0xC0 | 0x00);
  writeByte(BMP280_ADDR, BMP280_CTRL_REGISTER, 0x20 | 0x14 | 0x03);
  delay(100);

  this->update();
  angleX = angleAccX;
  angleY = angleAccY;
  preInterval = micros();
}

void IMUSensor::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void IMUSensor::setAccOffsets(float x, float y, float z){
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void IMUSensor::setDeclination(int16_t degree, uint8_t minute, char dir) {
  switch (dir) {
  case 'E':
    _declination = (degree + minute) / 60;
    break;
  case 'W':
    _declination = 0 - (degree + minute) / 60;
    break;
  }
}

void IMUSensor::calcOffsets(bool _offsetMPU, bool _offsetQMC, bool _offsetBMP) {
  if (_offsetMPU == true) {
    setGyroOffsets(0,0,0);
    setAccOffsets(0,0,0);
    float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
    for(uint8_t i = 0; i < CALIB_OFFSET_NB_MES; i++){
      this->calcDataMPU();
      ag[0] += accX;
      ag[1] += accY;
      ag[2] += (accZ-1.0f);
      ag[3] += gyroX;
      ag[4] += gyroY;
      ag[5] += gyroZ;
    }
    accXoffset  = ag[0] / (float)CALIB_OFFSET_NB_MES;
    accYoffset  = ag[1] / (float)CALIB_OFFSET_NB_MES;
    accZoffset  = ag[2] / (float)CALIB_OFFSET_NB_MES;
    gyroXoffset = ag[3] / (float)CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / (float)CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / (float)CALIB_OFFSET_NB_MES;
  }
  
  if (_offsetQMC == true) {
    startHeading = 0; 
    this->calcDataQMC(); 
    startHeading = Heading;
  }
  if (_offsetBMP == true) {
    startAltitude = 0; 
    this->calcDataBMP(); 
    startAltitude = Altitude;
  }
}

/* UPDATE */
void IMUSensor::update(){
  this->calcDataMPU();
  this->calcDataQMC();
  this->calcDataBMP();
}

void IMUSensor::calcDataMPU() {
  int16_t rawAG[7]; // [ax,ay,az,temp,gx,gy,gz]
  readMPU(MPU6050_ACCEL_OUT_REGISTER, 14);
  for(uint8_t i = 0; i < 7; i++){
    rawAG[i]  = wire->read() << 8 | wire->read();
  }

  accX  = ((float)rawAG[0] / ACC_LSB_2_G) - accXoffset;
  accY  = ((float)rawAG[1] / ACC_LSB_2_G) - accYoffset;
  accZ  = ((float)rawAG[2] / ACC_LSB_2_G) - accZoffset;
  temp  = ((float)rawAG[3] / TEMP_LSB_2_DEGREE) + TEMP_LSB_OFFSET;
  gyroX = ((float)rawAG[4] / GYRO_LSB_2_DEG_SEC) - gyroXoffset;
  gyroY = ((float)rawAG[5] / GYRO_LSB_2_DEG_SEC) - gyroYoffset;
  gyroZ = ((float)rawAG[6] / GYRO_LSB_2_DEG_SEC) - gyroZoffset;
  
  float sgZ  = accZ < 0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  angleAccX  = LPF(angleAccX, (atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG)); // [-180, +180] deg
  angleAccY  = LPF(angleAccY, (-atan2(accX, sgZ*sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG)); // [-180, +180] deg
  angleGyroX = LPF(angleGyroX, gyroX);
  angleGyroY = LPF(angleGyroY, gyroY);
  angleGyroZ = LPF(angleGyroZ, gyroZ);

  // estimate tilt angles: this is an approximation for small angles!
  float _dt = (micros() - preInterval) * 1e-6f;
  angleX = wrap((0.28f + LPF_OFFSET) * (angleAccX + wrap(angleX + angleGyroX * _dt - angleAccX, 180)) + (1.0f - (0.28f + LPF_OFFSET)) * angleAccX, 180);
  angleY = wrap((0.28f + LPF_OFFSET) * (angleAccY + wrap(angleY + angleGyroY * _dt - angleAccY, 180)) + (1.0f - (0.28f + LPF_OFFSET)) * angleAccY, 180);
  angleZ = wrap((0.28f + LPF_OFFSET) * (Heading   + wrap(angleZ + angleGyroZ * _dt - Heading,   180)) + (1.0f - (0.28f + LPF_OFFSET)) * Heading,   180);
  preInterval = micros();
}

void IMUSensor::calcDataQMC() {  
  float rawMG[6];
  readQMC(QMC5883L_OUT_REGISTER, 6);
  for(uint8_t i = 0; i < 3; i++){
    rawMG[i] = (int)(int16_t)(wire->read() | wire->read() << 8);
  }
  float a = (atan2(rawMG[1], rawMG[0]) * RAD_2_DEG) + _declination;
  Azimuth = a < 0 ? 360 + a : a;
  Heading = wrap(a - startHeading, 180);
}

void IMUSensor::calcDataBMP() {  
  uint8_t rawBMP[6];
  readBMP(BMP280_PRES_MSB, rawBMP, 6);
  int32_t adcTemp = (int32_t)rawBMP[3] << 12 | (int32_t)rawBMP[4] << 4 | (int32_t)rawBMP[5] >> 4; 
  int32_t adcPres = (int32_t)rawBMP[0] << 12 | (int32_t)rawBMP[1] << 4 | (int32_t)rawBMP[2] >> 4;

  int32_t vaT1 = ((((adcTemp >> 3) - ((int32_t)params.dig_T1 << 1))) * ((int32_t)params.dig_T2)) >> 11;
  int32_t vaT2 = (((((adcTemp >> 4) - ((int32_t)params.dig_T1)) * ((adcTemp >> 4) - ((int32_t)params.dig_T1))) >> 12) * ((int32_t)params.dig_T3)) >> 14;
  int32_t t_fine = vaT1 + vaT2;
  TempB = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)params.dig_P6;
  var2 = var2 + ((var1 * (int64_t)params.dig_P5) << 17);
  var2 = var2 + (((int64_t)params.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)params.dig_P3) >> 8) + ((var1 * (int64_t)params.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)params.dig_P1) >> 33;
  int64_t p = 1048576 - adcPres;
  if (var1 == 0) {p = 0;}
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)params.dig_P9) * (p >> 13) * (p>>13)) >> 25;
  var2 = (((int64_t)params.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)params.dig_P7) << 4);
  Pressure = p / 256.0f / 100.0f;

  Altitude = (Altitude * 0.98f) + (((((float)powf(1013.23f / Pressure, 0.190223f) - 1.0f) * (TempB + 273.15f) / 0.0065f) - startAltitude) * 0.02);
}

void IMUSensor::readMPU(uint8_t reg, int bitData) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(false);
  wire->requestFrom(MPU6050_ADDR, bitData);
}

void IMUSensor::readQMC(uint8_t reg, int bitData) {
  wire->beginTransmission(QMC5883L_ADDR);
  wire->write(reg);
  wire->endTransmission();
  wire->requestFrom(QMC5883L_ADDR, bitData);
}

void IMUSensor::readBMP(uint8_t reg, uint8_t* data, int16_t bitData) {
  wire->beginTransmission(BMP280_ADDR);          
  wire->write(reg);                   
  wire->endTransmission(false); 
  uint8_t i = 0;
  wire->requestFrom(BMP280_ADDR, bitData);
  while (wire->available()) {
    data[i++] = wire->read();
  }
}

uint8_t IMUSensor::writeByte(uint8_t add, uint8_t reg, uint8_t data){
  wire->beginTransmission(add);
  wire->write(reg);
  wire->write(data);
  uint8_t status = wire->endTransmission();
  return status; 
}

uint8_t IMUSensor::readByte(uint8_t add, uint8_t reg) {
  wire->beginTransmission(add);
  wire->write(reg);
  wire->endTransmission(false);
  wire->requestFrom(add, (uint8_t)1);
  int data = wire->read(); 
  return data;
}

IMUSensor IMU;