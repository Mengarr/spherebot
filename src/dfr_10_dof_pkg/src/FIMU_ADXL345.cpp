/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for pi5*                                                 *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

#include "../include/dfr_10_dof_pkg/FIMU_ADXL345.h"

#define TO_READ (6)      // num of bytes we are going to read each time (two bytes for each axis)

ADXL345::ADXL345(uint8_t address)
    : _i2cBus(address)
{
  status = ADXL345_OK;
  error_code = ADXL345_NO_ERROR;

  gains[0] = 0.00376390;
  gains[1] = 0.00376009;
  gains[2] = 0.00349265;
}

void ADXL345::init() {
  powerOn();
  setRangeSetting(2);
}

void ADXL345::powerOn() {
  //Turning on the ADXL345
  //_i2cBus.writemem(ADXL345_POWER_CTL, 0);      
  //_i2cBus.writemem(ADXL345_POWER_CTL, 16);
  _i2cBus.writemem(ADXL345_POWER_CTL, 8);
}

// Reads the acceleration into an array of three places
void ADXL345::readAccel(int16_t *xyz){
  readAccel(xyz, xyz + 1, xyz + 2);
}

// Reads the acceleration into three variable x, y and z
void ADXL345::readAccel(int16_t *x, int16_t *y, int16_t *z) {
  _i2cBus.readmem(ADXL345_DATAX0, TO_READ, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  *x = (((int16_t)_buff[1]) << 8) | _buff[0];  
  *y = (((int16_t)_buff[3]) << 8) | _buff[2];
  *z = (((int16_t)_buff[5]) << 8) | _buff[4];
}

void ADXL345::get_Gxyz(float *xyz){
  int i;
  int16_t xyz_int[3];
  readAccel(xyz_int);
  for(i=0; i<3; i++){
    xyz[i] = xyz_int[i] * gains[i];
  }
}

// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void ADXL345::getRangeSetting(uint8_t* rangeSetting) {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_DATA_FORMAT, 1, &_b);
  *rangeSetting = _b & 0b00000011;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void ADXL345::setRangeSetting(int val) {
  uint8_t _s;
  uint8_t _b;

  switch (val) {
  case 2:  
    _s = 0b00000000;
    break;
  case 4:  
    _s = 0b00000001;
    break;
  case 8:  
    _s = 0b00000010;
    break;
  case 16:
    _s = 0b00000011;
    break;
  default:
    _s = 0b00000000;
  }
  _i2cBus.readmem(ADXL345_DATA_FORMAT, 1, &_b);
  _s |= (_b & 0b11101100);
  _i2cBus.writemem(ADXL345_DATA_FORMAT, _s);
}
// gets the state of the SELF_TEST bit
bool ADXL345::getSelfTestBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void ADXL345::setSelfTestBit(bool selfTestBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

// Gets the state of the SPI bit
bool ADXL345::getSpiBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// Sets the SPI bit
// if set to 1 it sets the device to 3-wire mode
// if set to 0 it sets the device to 4-wire SPI mode
void ADXL345::setSpiBit(bool spiBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

// Gets the state of the INT_INVERT bit
bool ADXL345::getInterruptLevelBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void ADXL345::setInterruptLevelBit(bool interruptLevelBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

// Gets the state of the FULL_RES bit
bool ADXL345::getFullResBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void ADXL345::setFullResBit(bool fullResBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

// Gets the state of the justify bit
bool ADXL345::getJustifyBit() {
  return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void ADXL345::setJustifyBit(bool justifyBit) {
  setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void ADXL345::setTapThreshold(int tapThreshold) {
  tapThreshold = std::min(std::max(tapThreshold,0),255);
  uint8_t _b = uint8_t (tapThreshold);
  _i2cBus.writemem(ADXL345_THRESH_TAP, _b);  
}

// Gets the THRESH_TAP byte value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int ADXL345::getTapThreshold() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_THRESH_TAP, 1, &_b);  
  return int (_b);
}

// set/get the gain for each axis in Gs / count
void ADXL345::setAxisGains(float *_gains){
  int i;
  for(i = 0; i < 3; i++){
    gains[i] = _gains[i];
  }
}
void ADXL345::getAxisGains(float *_gains){
  int i;
  for(i = 0; i < 3; i++){
    _gains[i] = gains[i];
  }
}
 

// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between
void ADXL345::setAxisOffset(int x, int y, int z) {
  _i2cBus.writemem(ADXL345_OFSX, uint8_t (x));  
  _i2cBus.writemem(ADXL345_OFSY, uint8_t (y));  
  _i2cBus.writemem(ADXL345_OFSZ, uint8_t (z));  
}

// Gets the OFSX, OFSY and OFSZ bytes
void ADXL345::getAxisOffset(int* x, int* y, int*z) {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_OFSX, 1, &_b);  
  *x = int (_b);
  _i2cBus.readmem(ADXL345_OFSY, 1, &_b);  
  *y = int (_b);
  _i2cBus.readmem(ADXL345_OFSZ, 1, &_b);  
  *z = int (_b);
}

// Sets the DUR byte
// The DUR byte contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/float tap funcitons. Max value is 255.
void ADXL345::setTapDuration(int tapDuration) {
  tapDuration = std::min(std::max(tapDuration,0),255);
  uint8_t _b = uint8_t (tapDuration);
  _i2cBus.writemem(ADXL345_DUR, _b);  
}

// Gets the DUR byte
int ADXL345::getTapDuration() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_DUR, 1, &_b);  
  return int (_b);
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the float tap function.
// It accepts a maximum value of 255.
void ADXL345::setDoubleTapLatency(int floatTapLatency) {
  uint8_t _b = uint8_t (floatTapLatency);
  _i2cBus.writemem(ADXL345_LATENT, _b);  
}

// Gets the Latent value
int ADXL345::getDoubleTapLatency() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_LATENT, 1, &_b);  
  return int (_b);
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the float tap function. The maximum value is 255.
void ADXL345::setDoubleTapWindow(int floatTapWindow) {
  floatTapWindow = std::min(std::max(floatTapWindow,0),255);
  uint8_t _b = uint8_t (floatTapWindow);
  _i2cBus.writemem(ADXL345_WINDOW, _b);  
}

// Gets the Window register
int ADXL345::getDoubleTapWindow() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_WINDOW, 1, &_b);  
  return int (_b);
}

// Sets the THRESH_ACT byte which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void ADXL345::setActivityThreshold(int activityThreshold) {
  activityThreshold = std::min(std::max(activityThreshold,0),255);
  uint8_t _b = uint8_t (activityThreshold);
  _i2cBus.writemem(ADXL345_THRESH_ACT, _b);  
}

// Gets the THRESH_ACT byte
int ADXL345::getActivityThreshold() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_THRESH_ACT, 1, &_b);  
  return int (_b);
}

// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void ADXL345::setInactivityThreshold(int inactivityThreshold) {
  inactivityThreshold = std::min(std::max(inactivityThreshold,0),255);
  uint8_t _b = uint8_t (inactivityThreshold);
  _i2cBus.writemem(ADXL345_THRESH_INACT, _b);  
}

// Gets the THRESH_INACT byte
int ADXL345::getInactivityThreshold() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_THRESH_INACT, 1, &_b);  
  return int (_b);
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void ADXL345::setTimeInactivity(int timeInactivity) {
  timeInactivity = std::min(std::max(timeInactivity,0),255);
  uint8_t _b = uint8_t (timeInactivity);
  _i2cBus.writemem(ADXL345_TIME_INACT, _b);  
}

// Gets the TIME_INACT register
int ADXL345::getTimeInactivity() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_TIME_INACT, 1, &_b);  
  return int (_b);
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void ADXL345::setFreeFallThreshold(int freeFallThreshold) {
  freeFallThreshold = std::min(std::max(freeFallThreshold,0),255);
  uint8_t _b = uint8_t (freeFallThreshold);
  _i2cBus.writemem(ADXL345_THRESH_FF, _b);  
}

// Gets the THRESH_FF register.
int ADXL345::getFreeFallThreshold() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_THRESH_FF, 1, &_b);  
  return int (_b);
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void ADXL345::setFreeFallDuration(int freeFallDuration) {
  freeFallDuration = std::min(std::max(freeFallDuration,0),255);  
  uint8_t _b = uint8_t (freeFallDuration);
  _i2cBus.writemem(ADXL345_TIME_FF, _b);  
}

// Gets the TIME_FF register.
int ADXL345::getFreeFallDuration() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_TIME_FF, 1, &_b);  
  return int (_b);
}

bool ADXL345::isActivityXEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 6);
}
bool ADXL345::isActivityYEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 5);
}
bool ADXL345::isActivityZEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 4);
}
bool ADXL345::isInactivityXEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 2);
}
bool ADXL345::isInactivityYEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 1);
}
bool ADXL345::isInactivityZEnabled() {  
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 0);
}

void ADXL345::setActivityX(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}
void ADXL345::setActivityY(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}
void ADXL345::setActivityZ(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}
void ADXL345::setInactivityX(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}
void ADXL345::setInactivityY(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}
void ADXL345::setInactivityZ(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}

bool ADXL345::isActivityAc() {
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 7);
}
bool ADXL345::isInactivityAc(){
  return getRegisterBit(ADXL345_ACT_INACT_CTL, 3);
}

void ADXL345::setActivityAc(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}
void ADXL345::setInactivityAc(bool state) {  
  setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state);
}

bool ADXL345::getSuppressBit(){
  return getRegisterBit(ADXL345_TAP_AXES, 3);
}
void ADXL345::setSuppressBit(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 3, state);
}

bool ADXL345::isTapDetectionOnX(){
  return getRegisterBit(ADXL345_TAP_AXES, 2);
}
void ADXL345::setTapDetectionOnX(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 2, state);
}
bool ADXL345::isTapDetectionOnY(){
  return getRegisterBit(ADXL345_TAP_AXES, 1);
}
void ADXL345::setTapDetectionOnY(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 1, state);
}
bool ADXL345::isTapDetectionOnZ(){
  return getRegisterBit(ADXL345_TAP_AXES, 0);
}
void ADXL345::setTapDetectionOnZ(bool state) {  
  setRegisterBit(ADXL345_TAP_AXES, 0, state);
}

bool ADXL345::isActivitySourceOnX(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 6);
}
bool ADXL345::isActivitySourceOnY(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 5);
}
bool ADXL345::isActivitySourceOnZ(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 4);
}

bool ADXL345::isTapSourceOnX(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 2);
}
bool ADXL345::isTapSourceOnY(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 1);
}
bool ADXL345::isTapSourceOnZ(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 0);
}

bool ADXL345::isAsleep(){
  return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}

bool ADXL345::isLowPower(){
  return getRegisterBit(ADXL345_BW_RATE, 4);
}
void ADXL345::setLowPower(bool state) {  
  setRegisterBit(ADXL345_BW_RATE, 4, state);
}

float ADXL345::getRate(){
  uint8_t _b;
  _i2cBus.readmem(ADXL345_BW_RATE, 1, &_b);
  _b &= 0b00001111;
  return (std::pow(2,((int) _b)-6)) * 6.25;
}

void ADXL345::setRate(float rate){
  uint8_t _b,_s;
  int v = (int) (rate / 6.25);
  int r = 0;
  while (v >>= 1)
  {
    r++;
  }
  if (r <= 9) {
    _i2cBus.readmem(ADXL345_BW_RATE, 1, &_b);
    _s = (uint8_t) (r + 6) | (_b & 0b11110000);
    _i2cBus.writemem(ADXL345_BW_RATE, _s);
  }
}

void ADXL345::set_bw(uint8_t bw_code){
  if((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)){
    status = false;
    error_code = ADXL345_BAD_ARG;
  }
  else{
    _i2cBus.writemem(ADXL345_BW_RATE, bw_code);
  }
}

uint8_t ADXL345::get_bw_code(){
  uint8_t bw_code;
  _i2cBus.readmem(ADXL345_BW_RATE, 1, &bw_code);
  return bw_code;
}

uint8_t ADXL345::getInterruptSource() {
  uint8_t _b;
  _i2cBus.readmem(ADXL345_INT_SOURCE, 1, &_b);
  return _b;
}

bool ADXL345::getInterruptSource(uint8_t interruptBit) {
  return getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

bool ADXL345::getInterruptMapping(uint8_t interruptBit) {
  return getRegisterBit(ADXL345_INT_MAP,interruptBit);
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345::setInterruptMapping(uint8_t interruptBit, bool interruptPin) {
  setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

bool ADXL345::isInterruptEnabled(uint8_t interruptBit) {
  return getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

void ADXL345::setInterrupt(uint8_t interruptBit, bool state) {
  setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void ADXL345::setRegisterBit(uint8_t regAdress, int bitPos, bool state) {
  uint8_t _b;
  _i2cBus.readmem(regAdress, 1, &_b);
  if (state) {
    _b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
  }
  else {
    _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
  }
  _i2cBus.writemem(regAdress, _b);  
}

bool ADXL345::getRegisterBit(uint8_t regAdress, int bitPos) {
  uint8_t _b;
  _i2cBus.readmem(regAdress, 1, &_b);
  return ((_b >> bitPos) & 1);
}
