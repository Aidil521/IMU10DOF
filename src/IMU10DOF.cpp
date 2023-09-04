#include "IMU10DOF.h"

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

IMUSensor::IMUSensor(TwoWire &w){
  wire = &w;
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
}

void IMUSensor::begin(){
  wire->begin();
  wire->setClock(400000);
  // Configuration MPU6050
  uint8_t statusMPU = writeData(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REGISTER, 0x01); 
  writeData(MPU6050_ADDR, MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_ADDR, MPU6050_CONFIG_REGISTER, 0x00);
  writeData(MPU6050_ADDR, MPU6050_GYRO_CONFIG_REGISTER, 0x00);
  writeData(MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
  delay(100);

  // Configuration QMC5883L
  uint8_t statusQMC = writeData(QMC5883L_ADDR, QMC5583L_CONFIG_REGISTER, 0x01);
  writeData(QMC5883L_ADDR, QMC5883L_MODE_REGISTER, 0x01 | 0x0C | 0x10 | 0x00);
  delay(100);

  // Configuration BMP280
  if (readByte(BMP280_ADDR, BMP280_MDICE_ID) != DEVICE_ID);
  writeData(BMP280_ADDR, BMP280_RESET, RESET_CODE);
  delay(10);
  readBMP(BMP280_TRIM_PARAMS, (uint8_t *)&params ,sizeof(params));
  uint8_t statusBMP = writeData(BMP280_ADDR, BMP280_CONFIG_REGISTER, 0xC0 | 0x00);
  writeData(BMP280_ADDR, BMP280_CTRL_REGISTER, 0x20 | 0x14 | 0x03);
  delay(100);

  this->calcDataMPU();
  angleX = angleAccX;
  angleY = angleAccY;
  preInterval = millis(); // may cause lack of angular accuracy if begin() is much before the first update()
}

uint8_t IMUSensor::writeData(uint8_t add, uint8_t reg, uint8_t data){
  wire->beginTransmission(add);
  wire->write(reg);
  wire->write(data);
  uint8_t status = wire->endTransmission();
  return status; // 0 if success
}

uint8_t IMUSensor::readByte(uint8_t add, uint8_t reg) {
  wire->beginTransmission(add);
  wire->write(reg);
  wire->endTransmission(false);
  wire->requestFrom(add, (uint8_t)1);
  int data = wire->read(); 
	return data;
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

/* SETTER */
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

void IMUSensor::calcOffsets(bool _offsetMPU, bool _offsetQMC, bool _offsetBMP) {
  if (_offsetMPU == true) {
    setGyroOffsets(0,0,0);
    setAccOffsets(0,0,0);
    float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
    for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
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

void IMUSensor::setDeclination(int16_t degree, uint8_t minute) {
	_declination = (degree + minute) / 60;
}

/* UPDATE */
void IMUSensor::update(){
  this->calcDataMPU();
  this->calcDataQMC();
  this->calcDataBMP();
}

void IMUSensor::calcDataMPU(){
  // retrieve raw data
  this->readMPU(MPU6050_ACCEL_OUT_REGISTER, 14);
  int16_t rawAG[7]; // [ax,ay,az,temp,gx,gy,gz]
  for(uint8_t i = 0; i < 7; i++){
	  rawAG[i]  = wire->read() << 8;
    rawAG[i] |= wire->read();
  }
  accX = ((float)rawAG[0]) / 4096.0f - accXoffset;
  accY = ((float)rawAG[1]) / 4096.0f - accYoffset;
  accZ = ((float)rawAG[2]) / 4096.0f - accZoffset;
  temp = (rawAG[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawAG[4]) / 131.0f - gyroXoffset;
  gyroY = ((float)rawAG[5]) / 131.0f - gyroYoffset;
  gyroZ = ((float)rawAG[6]) / 131.0f - gyroZoffset;
  
  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  angleAccX = atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
  angleAccY = -atan2(accX, sgZ*sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 180,+ 180] deg

  float dt = (millis() - preInterval) * 1e-3f;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyIMUSensor/issues/6
  angleX = wrap(0.98f*(angleAccX + wrap(angleX + gyroX*dt - angleAccX, 180)) + (1.0f-0.98f)*angleAccX, 180);
  angleY = wrap(0.98f*(angleAccY + wrap(angleY + gyroY*dt - angleAccY, 180)) + (1.0f-0.98f)*angleAccY, 180);
  angleZ += gyroZ*dt; // not wrapped
  preInterval = millis();
}
/**
	GET AZIMUTH
	Calculate the azimuth (in degrees);
	
	@since v0.1;
	@return int azimuth
**/
void IMUSensor::calcDataQMC(){
  float rawMG[6];
  this->readQMC(QMC5883L_OUT_REGISTER, 6);
  for(uint8_t i = 0; i < 3; i++){
      rawMG[i] = (int)(int16_t)(wire->read() | wire->read() << 8);
  }
	float a = (atan2(rawMG[1], rawMG[0]) * 180.0f / PI) + _declination;
	Azimuth = a < 0 ? 360 + a : a;
	float h = (int)((a - startHeading) * 10) % 3600;
	Heading = (h < (-1790)) ? ((h / 10.0f) + 360) : (h / 10.0f);
}

void IMUSensor::calcDataBMP(){
  uint8_t data[6];
  this->readBMP(BMP280_PRES_MSB, &data[0], 6);
  int32_t adcTemp = (int32_t)data[3] << 12 | (int32_t)data[4] << 4 | (int32_t)data[5] >> 4;  // Copy the temperature and pressure data into the adc variables
	int32_t adcPres = (int32_t)data[0] << 12 | (int32_t)data[1] << 4 | (int32_t)data[2] >> 4;
    
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

  Altitude = (((float)powf(1013.23f / Pressure, 0.190223f) - 1.0f) * (TempB + 273.15f) / 0.0065f) - startAltitude;
}

IMUSensor IMU;