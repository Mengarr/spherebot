/****************************************************************************
* ITG3200.cpp - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with ITG-3200 Breakout                             *
* SCL     -> pin 21     (no pull up resistors)                              *
* SDA     -> pin 20     (no pull up resistors)                              *
* CLK & GND -> pin GND                                                      *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V                                                     *
*****************************************************************************/
#include "../include/dfr_10_dof_pkg/FIMU_ITG3200.h"

ITG3200::ITG3200(uint8_t address)
  : _i2cBus(address)
  {
  setGains(1.0,1.0,1.0);
  setOffsets(0.0,0.0,0.0);
  setRevPolarity(0,0,0);    
}

ITG3200::~ITG3200() {
  // Close I2C file on destructor
  
}

void ITG3200::init() {
  // Uncomment or change your default ITG3200 initialization
  // Initialise reading from bus
  // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
  init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);
  
  // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);
  
  // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);
  
  // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);

  
}

void ITG3200::init(uint8_t  _SRateDiv, uint8_t  _Range, uint8_t  _filterBW, uint8_t  _ClockSrc, bool _ITGReady, bool _INTRawDataReady) {
  setSampleRateDiv(_SRateDiv);
  setFSRange(_Range);
  setFilterBW(_filterBW);
  setClockSource(_ClockSrc);
  setITGReady(_ITGReady);
  setRawDataReady(_INTRawDataReady);  
  std::this_thread::sleep_for(std::chrono::milliseconds(GYROSTART_UP_DELAY));  // startup 
}

// uint8_t ITG3200::getDevAddr() {
//   /*_i2cBus.readmem(WHO_AM_I, 1, &_buff[0]); 
//   return _buff[0];  */
//   return _dev_address;
// }

// void ITG3200::setDevAddr(unsigned int  _addr) {
//   writemem(WHO_AM_I, _addr); 
//   _dev_address = _addr;
// }

uint8_t  ITG3200::getSampleRateDiv() {
  _i2cBus.readmem(SMPLRT_DIV, 1, &_buff[0]);
  return _buff[0];
}

void ITG3200::setSampleRateDiv(uint8_t  _SampleRate) {
  _i2cBus.writemem(SMPLRT_DIV, _SampleRate);
}

uint8_t  ITG3200::getFSRange() {
  _i2cBus.readmem(DLPF_FS, 1, &_buff[0]);
  return ((_buff[0] & DLPFFS_FS_SEL) >> 3);
}

void ITG3200::setFSRange(uint8_t  _Range) {
  _i2cBus.readmem(DLPF_FS, 1, &_buff[0]);   
  _i2cBus.writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) ); 
}

uint8_t  ITG3200::getFilterBW() {  
  _i2cBus.readmem(DLPF_FS, 1, &_buff[0]);
  return (_buff[0] & DLPFFS_DLPF_CFG); 
}

void ITG3200::setFilterBW(uint8_t  _BW) {   
  _i2cBus.readmem(DLPF_FS, 1, &_buff[0]);
  _i2cBus.writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW)); 
}

bool ITG3200::isINTActiveOnLow() {  
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ACTL) >> 7);
}

void ITG3200::setINTLogiclvl(bool _State) {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  _i2cBus.writemem(INT_CFG, ((_buff[0] & ~INTCFG_ACTL) | (_State << 7))); 
}

bool ITG3200::isINTOpenDrain() {  
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_OPEN) >> 6);
}

void ITG3200::setINTDriveType(bool _State) {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  _i2cBus.writemem(INT_CFG, ((_buff[0] & ~INTCFG_OPEN) | _State << 6)); 
}

bool ITG3200::isLatchUntilCleared() {    
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}

void ITG3200::setLatchMode(bool _State) {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  _i2cBus.writemem(INT_CFG, ((_buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5)); 
}

bool ITG3200::isAnyRegClrMode() {    
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void ITG3200::setLatchClearMode(bool _State) {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  _i2cBus.writemem(INT_CFG, ((_buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4)); 
}

bool ITG3200::isITGReadyOn() {   
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}

void ITG3200::setITGReady(bool _State) {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  _i2cBus.writemem(INT_CFG, ((_buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2)); 
}

bool ITG3200::isRawDataReadyOn() {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  return (_buff[0] & INTCFG_RAW_RDY_EN);
}

void ITG3200::setRawDataReady(bool _State) {
  _i2cBus.readmem(INT_CFG, 1, &_buff[0]);
  _i2cBus.writemem(INT_CFG, ((_buff[0] & ~INTCFG_RAW_RDY_EN) | _State)); 
}

bool ITG3200::isITGReady() {
  _i2cBus.readmem(INT_STATUS, 1, &_buff[0]);
  return ((_buff[0] & INTSTATUS_ITG_RDY) >> 2);
}

bool ITG3200::isRawDataReady() {
  _i2cBus.readmem(INT_STATUS, 1, &_buff[0]);
  return (_buff[0] & INTSTATUS_RAW_DATA_RDY);
}

void ITG3200::readTemp(float *_Temp) {
  _i2cBus.readmem(TEMP_OUT,2,_buff);
  *_Temp = 35 + (((_buff[0] << 8) | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32  
}

void ITG3200::readGyroRaw(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ){
  _i2cBus.readmem(GYRO_XOUT, 6, _buff);
  *_GyroX = (((int16_t)_buff[0] << 8) | _buff[1]);
  *_GyroY = (((int16_t)_buff[2] << 8) | _buff[3]); 
  *_GyroZ = (((int16_t)_buff[4] << 8) | _buff[5]);
}

void ITG3200::readGyroRaw(int16_t *_GyroXYZ){
  readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
  polarities[0] = _Xpol ? -1 : 1;
  polarities[1] = _Ypol ? -1 : 1;
  polarities[2] = _Zpol ? -1 : 1;
}

void ITG3200::setGains(float _Xgain, float _Ygain, float _Zgain) {
  gains[0] = _Xgain;
  gains[1] = _Ygain;
  gains[2] = _Zgain;
}

void ITG3200::setOffsets(int _Xoffset, int _Yoffset, int _Zoffset) {
  offsets[0] = _Xoffset;
  offsets[1] = _Yoffset;
  offsets[2] = _Zoffset;
}

void ITG3200::zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  int16_t xyz[3]; 
  float tmpOffsets[] = {0,0,0};

  for (unsigned int i = 0;i < totSamples;i++){
    std::this_thread::sleep_for(std::chrono::milliseconds(sampleDelayMS));
    readGyroRaw(xyz);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];  
  }
  setOffsets(-tmpOffsets[0] / totSamples, -tmpOffsets[1] / totSamples, -tmpOffsets[2] / totSamples);
}

void ITG3200::readGyroRawCal(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ) {
  readGyroRaw(_GyroX, _GyroY, _GyroZ);
  *_GyroX += offsets[0];
  *_GyroY += offsets[1];
  *_GyroZ += offsets[2];
}

void ITG3200::readGyroRawCal(int16_t *_GyroXYZ) {
  readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::readGyro(float *_GyroX, float *_GyroY, float *_GyroZ){
  int16_t x, y, z;
  
  readGyroRawCal(&x, &y, &z); // x,y,z will contain calibrated integer values from the sensor
  *_GyroX =  (float)(x / 14.375 * polarities[0] * gains[0]);
  *_GyroY =  (float)(y / 14.375 * polarities[1] * gains[1]);
  *_GyroZ =  (float)(z / 14.375 * polarities[2] * gains[2]);
}

void ITG3200::readGyro(float *_GyroXYZ){
  readGyro(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::reset() {     
  _i2cBus.writemem(PWR_MGM, PWRMGM_HRESET); 
  std::this_thread::sleep_for(std::chrono::milliseconds(GYROSTART_UP_DELAY)); //gyro startup 
}

bool ITG3200::isLowPower() {   
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_SLEEP) >> 6;
}
  
void ITG3200::setPowerMode(bool _State) {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  _i2cBus.writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_SLEEP) | _State << 6));  
}

bool ITG3200::isXgyroStandby() {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_XG) >> 5;
}

bool ITG3200::isYgyroStandby() {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_YG) >> 4;
}

bool ITG3200::isZgyroStandby() {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_ZG) >> 3;
}

void ITG3200::setXgyroStandby(bool _Status) {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  _i2cBus.writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}

void ITG3200::setYgyroStandby(bool _Status) {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  _i2cBus.writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}

void ITG3200::setZgyroStandby(bool _Status) {
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  _i2cBus.writemem(PWR_MGM, ((_buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}

uint8_t  ITG3200::getClockSource() {  
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_CLK_SEL);
}

void ITG3200::setClockSource(uint8_t  _CLKsource) {   
  _i2cBus.readmem(PWR_MGM, 1, &_buff[0]);
  _i2cBus.writemem(PWR_MGM, ((_buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource)); 
}

// Arduino implementation
// void ITG3200::writemem(uint8_t _addr, uint8_t _val) {
//   Wire.beginTransmission(_dev_address);   // start transmission to device 
//   Wire.write(_addr); // send register address
//   Wire.write(_val); // send value to write
//   Wire.endTransmission(); // end transmission
// }

// void ITG3200::_i2cBus.readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
//   Wire.beginTransmission(_dev_address); // start transmission to device 
//   Wire.write(_addr); // sends register address to read from
//   Wire.endTransmission(); // end transmission
  
//   Wire.beginTransmission(_dev_address); // start transmission to device 
//   Wire.requestFrom(_dev_address, _nbytes);// send data n-bytes read
//   uint8_t i = 0; 
//   while (Wire.available()) {
//     __buff[i] = Wire.read(); // receive DATA
//     i++;
//   }
//   Wire.endTransmission(); // end transmission
// }

