/*
    bq769x0.cpp - Battery management system based on bq769x0 for Arduino
    Copyright (C) 2015  Martin Jäger (m.jaeger@posteo.de)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program. If not, see 
    <http://www.gnu.org/licenses/>.
*/

/*
  TODO:
  - Balancing algorithm
  - SOC calculation + coulomb counting
  - Autobalancing (what is this?)
*/

#include <Arduino.h>
#include <Wire.h>     // I2C/TWI (for Battery Management IC)
#include <math.h>     // log for thermistor calculation

#include "bq769x0.h"
#include "registers.h"

// for the ISR to know the bq769x0 instance
bq769x0* bq769x0::instancePointer = 0;


#if BQ769X0_DEBUG
  const char *byte2char(int x)
  {
    static char b[9];
    b[0] = '\0';
    int z;
    for (z = 128; z > 0; z >>= 1) strcat(b, ((x & z) == z) ? "1" : "0");
    return b;
  }
#endif

// CRC calculation taken from LibreSolar mbed firmware
uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;
  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else data <<= 1;
  }

  return data;
}

//----------------------------------------------------------------------------

/*
bq769x0::bq769x0(byte bqType, int bqI2CAddress)
{
  type = bqType;
  I2CAddress = bqI2CAddress;
  
  if (type == bq76920) {
    _cellBal[0].cellBalAddress = CELLBAL1;
    numberOfCells = 5;
  }
  else if (type == bq76930) {
    _cellBal[0].cellBalAddress = CELLBAL1;
    _cellBal[1].cellBalAddress = CELLBAL2;
    numberOfCells = 10;  
  }
  else {
    _cellBal[0].cellBalAddress = CELLBAL1;
    _cellBal[1].cellBalAddress = CELLBAL2;
    _cellBal[2].cellBalAddress = CELLBAL3;
    numberOfCells = 15;
  }
  
  // prevent errors if someone reduced MAX_NUMBER_OF_CELLS accidentally
  if (numberOfCells > MAX_NUMBER_OF_CELLS) {
    numberOfCells = MAX_NUMBER_OF_CELLS;
  }
  setCellConfiguration(CELL_15);
  disableBalancingProtection();
}
*/

bq769x0::bq769x0(byte bqType, int bqI2CAddress, uint8_t channel)
{
  _channel = channel;
  type = bqType;
  I2CAddress = bqI2CAddress;
  
  if (type == bq76920) {
    _cellBal[0].cellBalAddress = CELLBAL1;
    numberOfCells = 5;
  }
  else if (type == bq76930) {
    _cellBal[0].cellBalAddress = CELLBAL1;
    _cellBal[1].cellBalAddress = CELLBAL2;
    numberOfCells = 10;  
  }
  else {
    _cellBal[0].cellBalAddress = CELLBAL1;
    _cellBal[1].cellBalAddress = CELLBAL2;
    _cellBal[2].cellBalAddress = CELLBAL3;
    numberOfCells = 15;
  }
  
  // prevent errors if someone reduced MAX_NUMBER_OF_CELLS accidentally
  if (numberOfCells > MAX_NUMBER_OF_CELLS) {
    numberOfCells = MAX_NUMBER_OF_CELLS;
  }
  setCellConfiguration(CELL_15);
  disableBalancingProtection();
}

int bq769x0::getTCAChannel()
{
  return _channel;
}

void bq769x0::setI2C(TwoWire *wire)
{
  _wire = wire;
}

//-----------------------------------------------------------------------------

int bq769x0::begin(byte alertPin, byte bootPin)
{
  // Wire.begin();        // join I2C bus
  _listener = 0;
  _bootPin = bootPin;
  _alertPin = alertPin;
  // initialize variables
  for (byte i = 0; i < numberOfCells; i++) {
    cellVoltages[i] = 0;
  }
  
  if (_wire <= 0 || _wire == NULL)
  {
    // Serial.println("BMS I2C pin not defined");
    return 1;
  }

  // Boot IC if pin is defined (else: manual boot via push button has to be done before calling this method)
  wake();
 
  if (determineAddressAndCrc())
  {
    LOG_PRINTLN("Address and CRC detection successful");
    LOG_PRINT("Address: ");
    LOG_PRINTLN(I2CAddress);
    LOG_PRINT("CRC Enabled: ");
    LOG_PRINTLN(crcEnabled);

    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, B00011000);  // switch external thermistor (TEMP_SEL) and ADC on (ADC_EN)
    writeRegister(SYS_CTRL2, B01000000);  // switch CC_EN on

    // attach ALERT interrupt to this instance
    instancePointer = this;
    attachInterrupt(digitalPinToInterrupt(_alertPin), bq769x0::alertISR, RISING);

    // get ADC offset and gain
    adcOffset = (signed int) readRegister(ADCOFFSET);  // convert from 2's complement
    adcGain = 365 + (((readRegister(ADCGAIN1) & B00001100) << 1) | ((readRegister(ADCGAIN2) & B11100000) >> 5)); // uV/LSB
    
    return 0;
  }

  else
  {
    LOG_PRINTLN("BMS communication error");
    return 1;
  }
}

/**
 * begin method with listener for tca function
 */
int bq769x0::begin(byte alertPin, byte bootPin, void (*listener) (uint8_t))
{
  // Wire.begin();        // join I2C bus
  _listener = listener;
  _bootPin = bootPin;
  _alertPin = alertPin;
  if (_listener > 0)
  {
    _listener(_channel);
  }
  
  // initialize variables
  for (byte i = 0; i < numberOfCells; i++) {
    cellVoltages[i] = 0;
  }
  
  // Boot IC if pin is defined (else: manual boot via push button has to be done before calling this method)
  wake();
 
  if (determineAddressAndCrc())
  {
    LOG_PRINTLN("Address and CRC detection successful");
    LOG_PRINT("Address: ");
    LOG_PRINTLN(I2CAddress);
    LOG_PRINT("CRC Enabled: ");
    LOG_PRINTLN(crcEnabled);

    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, B00011000);  // switch external thermistor (TEMP_SEL) and ADC on (ADC_EN)
    writeRegister(SYS_CTRL2, B01000000);  // switch CC_EN on

    // attach ALERT interrupt to this instance
    instancePointer = this;
    attachInterrupt(digitalPinToInterrupt(_alertPin), bq769x0::alertISR, RISING);

    // get ADC offset and gain
    adcOffset = (signed int) readRegister(ADCOFFSET);  // convert from 2's complement
    adcGain = 365 + (((readRegister(ADCGAIN1) & B00001100) << 1) | ((readRegister(ADCGAIN2) & B11100000) >> 5)); // uV/LSB
    
    return 0;
  }

  else
  {
    LOG_PRINTLN("BMS communication error");
    return 1;
  }
}

/**
 * Method for set Listener, use this method to change listener
 */
void bq769x0::setListener(void (*listener)(uint8_t))
{
  _listener = listener;
}

/**
 * Method to clear listener
 */
void bq769x0::removeListener()
{
  _listener = 0;
}

/**
 * Method to set bq769x0 cell configuration such as 9 cell, 10 cell, 11 cell, 12 cell, 13 cell, 14 cell, 15 cell
 * This method is for balancing protection to prevent adjacent cell from being balanced at the same time
 */
void bq769x0::setCellConfiguration(int cellConfiguration)
{
  if (cellConfiguration >= 9 && cellConfiguration <= 15)
  {
    _cellConfiguration = cellConfiguration;
  }
}

int bq769x0::getCellConfiguration()
{
  return _cellConfiguration;
}

/**
 * Method to enable hardware balancing protection to prevent adjacent cell from being balanced at the same time
 */
void bq769x0::enableBalancingProtection()
{
  _isBalancingProtectionEnabled = true;
}


/**
 * Method to disable hardware balancing protection
 */
void bq769x0::disableBalancingProtection()
{
  _isBalancingProtectionEnabled = false;
}

//----------------------------------------------------------------------------
// automatically find out address and CRC setting

bool bq769x0::determineAddressAndCrc(void)
{
  LOG_PRINTLN("Determining i2c address and whether CRC is enabled");

  // check for each address and CRC combination while also set CC_CFG to 0x19 as per datasheet
  I2CAddress = 0x08;
  crcEnabled = false;
  crcEnabled = true;  //add as a test method
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) return true;

  I2CAddress = 0x18;
  crcEnabled = false;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) return true;

  I2CAddress = 0x08;
  crcEnabled = true;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) return true;

  I2CAddress = 0x18;
  crcEnabled = true;
  writeRegister(CC_CFG, 0x19);
  if (readRegister(CC_CFG) == 0x19) return true;

  return false;
}

//----------------------------------------------------------------------------
// Fast function to check whether BMS has an error
// (returns 0 if everything is OK)

int bq769x0::checkStatus()
{
   LOG_PRINT("checkStatus: ");
   LOG_PRINTLN(errorStatus);
  if (alertInterruptFlag == false && errorStatus == 0) {
    return 0;
  }
  else {
    regSYS_STAT_t sys_stat;
    sys_stat.regByte = readRegister(SYS_STAT);

    if (sys_stat.bits.CC_READY == 1) {
      //LOG_PRINTLN("Interrupt: CC ready");
      updateCurrent(true);  // automatically clears CC ready flag	
    }
    
    // Serious error occured
    if (sys_stat.regByte & B00111111)
    {
      if (alertInterruptFlag == true) {
        secSinceErrorCounter = 0;
      }
      errorStatus = sys_stat.regByte;
      
      int secSinceInterrupt = (millis() - interruptTimestamp) / 1000;

      // check for overrun of millis() or very slow running program
      if (abs((int) (secSinceInterrupt - secSinceErrorCounter)) > 2) {
        secSinceErrorCounter = secSinceInterrupt;
      }
      
      // called only once per second
      if (secSinceInterrupt >= secSinceErrorCounter)
      {
        if (sys_stat.regByte & B00100000) { // XR error
          // datasheet recommendation: try to clear after waiting a few seconds
          if (secSinceErrorCounter % 3 == 0) {
            LOG_PRINTLN(F("Clearing XR error"));
            writeRegister(SYS_STAT, B00100000);
          }
        }
        if (sys_stat.regByte & B00010000) { // Alert error
          if (secSinceErrorCounter % 10 == 0) {
            LOG_PRINTLN(F("Clearing Alert error"));
            writeRegister(SYS_STAT, B00010000);
          }
        }
        if (sys_stat.regByte & B00001000) { // UV error
          updateVoltages();
          if (cellVoltages[idCellMinVoltage] > minCellVoltage) {
            LOG_PRINTLN(F("Clearing UV error"));
            writeRegister(SYS_STAT, B00001000);
          }
        }
        if (sys_stat.regByte & B00000100) { // OV error
          updateVoltages();
          if (cellVoltages[idCellMaxVoltage] < maxCellVoltage) {
            LOG_PRINTLN(F("Clearing OV error"));
            writeRegister(SYS_STAT, B00000100);
          }
        }
        if (sys_stat.regByte & B00000010) { // SCD
          if (secSinceErrorCounter % 60 == 0) {
            LOG_PRINTLN(F("Clearing SCD error"));
            writeRegister(SYS_STAT, B00000010);
          }
        }
        if (sys_stat.regByte & B00000001) { // OCD
          if (secSinceErrorCounter % 60 == 0) {
            LOG_PRINTLN(F("Clearing OCD error"));
            writeRegister(SYS_STAT, B00000001);
          }
        }
        
        secSinceErrorCounter++;
      }
    }
    else {
      errorStatus = 0;
    }
    
    return errorStatus;

  }

}

//----------------------------------------------------------------------------
// should be called at least once every 250 ms to get correct coulomb counting

void bq769x0::update()
{
  LOG_PRINTLN("update");
  if(_isSleep)
  {
    return;
  }
  if(_wire <= 0 || _wire == NULL)
  {
    return;
  }
  if (_listener > 0)
  {
    _listener(_channel);
  }
  // updateCurrent();  // will only read new current value if alert was triggered
  updateVoltages();
  updateTemperatures();
  updateBalanceSwitches();
  // 
  // updateBalancingSwitches();
}

//----------------------------------------------------------------------------
// puts BMS IC into SHIP mode (i.e. switched off)
/**
 * Puts BMS IC into SHIP mode (i.e. switched off)
 */
void bq769x0::shutdown()
{
  writeRegister(SYS_CTRL1, 0x0);
  writeRegister(SYS_CTRL1, 0x1);
  writeRegister(SYS_CTRL1, 0x2);
  delay(250); //Give time for IC to perform sleep sequence, minimum 250ms, the longer the time, the better
  int data = readRegister(CELLBAL1);
  // Serial.println("BMS Channel " + String(_channel) + " Shutdown");
  // Serial.println("Data Register Check : " + String(data));
  if (data < 0)
  {
    _isSleep = true;
  }
}

/**
 * Method to wake the BMS from SHIP mode into NORMAL mode
 */
void bq769x0::wake()
{
  _isSleep = true;
  if (_bootPin >= 0)
  {
    pinMode(_bootPin, OUTPUT);
    digitalWrite(_bootPin, HIGH);
    delay(5);   // wait 5 ms for device to receive boot signal (datasheet: max. 2 ms)
    pinMode(_bootPin, INPUT);     // don't disturb temperature measurement
    delay(10);  // wait for device to boot up completely (datasheet: max. 10 ms)
  }
  // delay(100);
  // Serial.println("Read register cellbal");
  
  int data = -1;
  
  // Serial.println("Data : " + String(data));
  // Serial.println("After reading");
  for (size_t i = 0; i < 10; i++)
  {
    data = readRegister(CELLBAL1);
    writeRegister(SYS_CTRL1, B00011000);  // switch external thermistor (TEMP_SEL) and ADC on (ADC_EN)
    writeRegister(SYS_CTRL2, B01000000);  // switch CC_EN on
  }
  
  _isSleep = false;
  if(data >= 0)
  {
    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, B00011000);  // switch external thermistor (TEMP_SEL) and ADC on (ADC_EN)
    writeRegister(SYS_CTRL2, B01000000);  // switch CC_EN on
    _isSleep = false;
  }
}

bool bq769x0::isDeviceSleep()
{
  return _isSleep;
}

//----------------------------------------------------------------------------

bool bq769x0::enableCharging()
{
  LOG_PRINTLN("enableCharging");
  if (checkStatus() == 0 &&
    cellVoltages[idCellMaxVoltage] < maxCellVoltage &&
    temperatures[0] < maxCellTempCharge &&
    temperatures[0] > minCellTempCharge)
  {
    byte sys_ctrl2;
    sys_ctrl2 = readRegister(SYS_CTRL2);
    writeRegister(SYS_CTRL2, sys_ctrl2 | B00000001);  // switch CHG on
    LOG_PRINTLN("enableCharging: enabled");
    return true;
  }
  else {
    LOG_PRINTLN("enableCharging: failed");
    return false;
  }
}

//----------------------------------------------------------------------------

bool bq769x0::enableDischarging()
{
  LOG_PRINTLN("enableDischarging");
  if (checkStatus() == 0 &&
    cellVoltages[idCellMinVoltage] > minCellVoltage &&
    temperatures[0] < maxCellTempDischarge &&
    temperatures[0] > minCellTempDischarge)
  {
    byte sys_ctrl2;
    sys_ctrl2 = readRegister(SYS_CTRL2);
    writeRegister(SYS_CTRL2, sys_ctrl2 | B00000010);  // switch DSG on
    LOG_PRINTLN("enableDischarging: enabled");
    return true;
  }
  else {
    LOG_PRINTLN("enableDischarging: failed");
    return false;
  }
}

//----------------------------------------------------------------------------

void bq769x0::enableAutoBalancing(void)
{
  autoBalancingEnabled = true;
}


//----------------------------------------------------------------------------

void bq769x0::setBalancingThresholds(int idleTime_min, int absVoltage_mV, byte voltageDifference_mV)
{
  balancingMinIdleTime_s = idleTime_min * 60;
  balancingMinCellVoltage_mV = absVoltage_mV;
  balancingMaxVoltageDifference_mV = voltageDifference_mV;
}

//----------------------------------------------------------------------------
// sets balancing registers if balancing is allowed 
// (sufficient idle time + voltage)

byte bq769x0::updateBalancingSwitches(void)
{
  LOG_PRINTLN("updateBalancingSwitches");
  long idleSeconds = (millis() - idleTimestamp) / 1000;
  byte numberOfSections = numberOfCells/5;
  
  // check for millis() overflow
  if (idleSeconds < 0) {
    idleTimestamp = 0;
    idleSeconds = millis() / 1000;
  }
    
  // check if balancing allowed
  if (checkStatus() == 0 &&
    idleSeconds >= balancingMinIdleTime_s && 
    cellVoltages[idCellMaxVoltage] > balancingMinCellVoltage_mV &&
    (cellVoltages[idCellMaxVoltage] - cellVoltages[idCellMinVoltage]) > balancingMaxVoltageDifference_mV)
  {
    balancingActive = true;
    //LOG_PRINTLN("Balancing enabled!");
    
    regCELLBAL_t cellbal;
    byte balancingFlags;
    byte balancingFlagsTarget;
    
    for (int section = 0; section < numberOfSections; section++)
    {
      balancingFlags = 0;
      for (int i = 0; i < 5; i++)
      {
        if ((cellVoltages[section*5 + i] - cellVoltages[idCellMinVoltage]) > balancingMaxVoltageDifference_mV) {
          
          // try to enable balancing of current cell
          balancingFlagsTarget = balancingFlags | (1 << i);

          // check if attempting to balance adjacent cells
          bool adjacentCellCollision = 
            ((balancingFlagsTarget << 1) & balancingFlags) ||
            ((balancingFlags << 1) & balancingFlagsTarget);
            
          if (adjacentCellCollision == false) {
            balancingFlags = balancingFlagsTarget;
          }          
        }
      }
      LOG_PRINT("Setting CELLBAL");
      LOG_PRINT(section+1);
      LOG_PRINT(" register to: ");
      LOG_PRINTLN(byte2char(balancingFlags));
      
      // set balancing register for this section
      writeRegister(CELLBAL1+section, balancingFlags);
    }
  }
  else if (balancingActive == true)
  {  
    // clear all CELLBAL registers
    for (int section = 0; section < numberOfSections; section++)
    {
      LOG_PRINT("Clearing Register CELLBAL");
      LOG_PRINTLN(section+1);
      writeRegister(CELLBAL1+section, 0x0);
    }
    
    balancingActive = false;
  }
}

byte bq769x0::updateBalancingSwitch()
{
  LOG_PRINTLN("updateBalancingSwitches");
  return 0;
  // writeRegister(CELLBAL1+section, 0x0);
}

bool bq769x0::testBalancing(uint8_t cellBal, uint8_t pos, bool switchState, int data)
{
  if (pos > 4)
  {
    return 0;
  }

  if (cellBal == CELLBAL1)
  {
    data = _dataCell[0];
  }
  if (cellBal == CELLBAL2)
  {
    data = _dataCell[1];
  }
  if (cellBal == CELLBAL3)
  {
    data = _dataCell[2];
  }

  if (data < 0)
  {
    return 0;
  }
  
  if (switchState)
  {
    if (!isValidPosition(pos, data, cellBal))
    {
      return 0;
    }
  }  
  uint8_t state = 1;
  if (switchState)
  {
    state = switchState << pos;
    state = data | state;
  }
  else
  {
    state = state << pos;
    uint8_t temp = 0b11111111;
    state = temp ^ state;
    state = data & state;
  }
  if (cellBal == CELLBAL1)
  {
    _cellBal[0].cellBalAddress = CELLBAL1;
    _cellBal[0].databal = state;
  }
  else if (cellBal == CELLBAL2)
  {
    _cellBal[1].cellBalAddress = CELLBAL2;
    _cellBal[1].databal = state;
  }
  else if (cellBal == CELLBAL3)
  {
    _cellBal[2].cellBalAddress = CELLBAL3;
    _cellBal[2].databal = state;
  }
  return 1;
}


/**
 * Method to set balancing register by bit, it takes CELLBAL address, bit position, and switch state (true to activated, false to deactivated)
 */
bool bq769x0::setBalanceSwitch(uint8_t cellBal, uint8_t pos, bool switchState)
{
  // Serial.println("Set Single Balancing");
  int data = readRegister(cellBal);
  if (pos > 4 || data < 0)
  {
    return 0;
  }
  if (cellBal == CELLBAL1)
  {
    // _cellBal[0].databal = readRegister(cellBal);
    // _cellBal[0].databal = _dataCell[0];
    data = _cellBal[0].databal;
  }
  else if (cellBal == CELLBAL2)
  {
    // _cellBal[1].databal = readRegister(cellBal);
    // _cellBal[1].databal = _dataCell[1];
    data = _cellBal[1].databal;
  }
  else if (cellBal == CELLBAL3)
  {
    // _cellBal[2].databal = readRegister(cellBal);
    // _cellBal[2].databal = _dataCell[2];
    data = _cellBal[2].databal;
  }
  if (data < 0)
  {
    return 0;
  }

  if(_isBalancingProtectionEnabled)
  {
    if (switchState)
    {
      if (!isValidPosition(pos, data, cellBal))
      {
        return 0;
      }
    }
  }
  
  uint8_t state = 1;
  if (switchState)
  {
    state = switchState << pos;
    state = data | state;
  }
  else
  {
    state = state << pos;
    uint8_t temp = 0b11111111;
    state = temp ^ state;
    state = data & state;
  }
  if (cellBal == CELLBAL1)
  {
    _cellBal[0].cellBalAddress = CELLBAL1;
    _cellBal[0].databal = state;
  }
  else if (cellBal == CELLBAL2)
  {
    _cellBal[1].cellBalAddress = CELLBAL2;
    _cellBal[1].databal = state;
  }
  else if (cellBal == CELLBAL3)
  {
    _cellBal[2].cellBalAddress = CELLBAL3;
    _cellBal[2].databal = state;
  }
  return 1;
}

/**
 * Method to set CELLBAL register by byte (8 bit), Note that only 6 bit is used
 * this method will fill the register from LSB to MSB
 * depends on how the cell configuration is set and if the balancing protection is activated,
 * some register may not be set 1 to prevent balancing adjacent cell
 */
bool bq769x0::setBalanceSwitches(uint8_t cellBal, uint8_t switchState)
{
  
  // int data = 0;
  int data = readRegister(cellBal);
  if (data < 0)
  {
    return 0;
  }

  for(int i = 0; i < 6; i ++)
  {
    int state = getBit(i, switchState);
    if (state < 0)
    {
      break;
    }
    setBalanceSwitch(cellBal, i, state);  
  }
  return 1;
}

/**
 * Method to deactivate all CELLBAL register by writing 0 on each bit
 */
void bq769x0::clearBalanceSwitches()
{
  if (type == bq76920) 
  {  
    _cellBal[0].databal = 0;    
  }
  else if (type == bq76930) 
  {
    for (int i = 0; i < 2; i++)
    {
      _cellBal[i].databal = 0;
    }
  }
  else 
  {
    for (int i = 0; i < 3; i++)
    {
      _cellBal[i].databal = 0;
    }
  }
}

/**
 * Method to send the local cellbal register through i2c communication, thus will write the value into the hardware
 * as long as this method is not called, the hardware register will not updated
 */
void bq769x0::updateBalanceSwitches()
{
  int data = readRegister(_cellBal[0].cellBalAddress);
  // int data = _dataCell[0];
  if (data < 0)
  {
    return;
  }
  if (type == bq76920) 
  {  
    // Serial.print("CelBal address = ");
    // Serial.print("0x");
    // Serial.println(_cellBal[0].cellBalAddress, HEX);
    // Serial.print("Data bal = ");
    // Serial.println(_cellBal[0].databal, BIN);
    _dataCell[0] = _cellBal[0].databal;
    writeRegister(_cellBal[0].cellBalAddress, _cellBal[0].databal);
    _cellBal[0].databal = readRegister(_cellBal[0].cellBalAddress);
  }
  else if (type == bq76930) 
  {
    for (int i = 0; i < 2; i++)
    {
      // Serial.print("CelBal address = ");
      // Serial.print("0x");
      // Serial.println(_cellBal[i].cellBalAddress, HEX);
      // Serial.print("Data bal = ");
      // Serial.println(_cellBal[i].databal, BIN);
      _dataCell[i] = _cellBal[i].databal;
      writeRegister(_cellBal[i].cellBalAddress, _cellBal[i].databal);
      _cellBal[i].databal = readRegister(_cellBal[i].cellBalAddress);
    }
  }
  else 
  {
    for (int i = 0; i < 3; i++)
    {
      // Serial.print("CelBal address = ");
      // Serial.print("0x");
      // Serial.println(_cellBal[i].cellBalAddress, HEX);
      // Serial.print("Data bal = ");
      // Serial.println(_cellBal[i].databal, BIN);
      _dataCell[i] = _cellBal[i].databal;
      writeRegister(_cellBal[i].cellBalAddress, _cellBal[i].databal);
      _cellBal[i].databal = readRegister(_cellBal[i].cellBalAddress);
    }
  }
}

/**
 * Method to get bit value from int data type, it takes argument of bit position and data
 * return -1 if the pos is more than 7 or below 0
 */
int bq769x0::getBit(int pos, int data)
{
  if (pos > 7 & pos < 0)
  {
    return -1;
  }
  int temp = data >> pos;
  temp = temp & 0x01;
  return temp;
}

/**
 * Method to get a local cell data, this method is used for simulation only
 */
int bq769x0::getDataCell(int cellBal)
{
  if (cellBal > 0 && cellBal < 4)
  {
    // return _dataCell[cellBal-1];
    return _cellBal[cellBal-1].databal;
  }
  return 0;
}

/**
 * Method to check the value after n bit, will return 1 if the bit value is 0, otherwise return 0
 */
bool bq769x0::checkPosAfter(int pos, int data)
{
  int comparator = 1;
  int isValid = true;
  if (pos < 4)
  {
    comparator = comparator << (pos + 1);
  }
  else
  {
    return isValid;
  }
  if ((data & comparator) != 0)
  {
    isValid = false;
  }
  return isValid;
}

/**
 * Method to check the value before n bit, will return 1 if the bit value is 0, otherwise return 0
 */
bool bq769x0::checkPosBefore(int pos, int data)
{
  int comparator = 1;
  int isValid = true;
  if (pos - 1 >= 0)
  {
    comparator = comparator << (pos - 1);
  }
  else
  {
    return isValid;
  }
  if ((data & comparator) != 0)
  {
    isValid = false;
  }
  return isValid;
}

/**
 * Method to check if the shorting pins (9,10,11,12,13,14 cell configuration) is inactive
 * return 1 if inactive, otherwise return 0
 */
bool bq769x0::isShortingPinsInactive(int data, int shortingPinConfiguration)
{
  int isShortingPinInactive = true;
  if (shortingPinConfiguration == _TRIPLE_SHORT)
  {
    for (int x : _shortingPins.pinList1)
    {
      int bitState = getBit(x,data);
      if (bitState)
      {
        isShortingPinInactive = false;
        break;
      }
    }
  }
  if (shortingPinConfiguration == _DOUBLE_SHORT)
  {
    for (int x : _shortingPins.pinList2)
    {
      int bitState = getBit(x,data);
      if (bitState)
      {
        isShortingPinInactive = false;
        break;
      }
    }
  }
  return isShortingPinInactive;
}

/**
 * Method to check bit value before the shorting pins (9,10,11,12,13,14 cell configuration)
 * return 1 if bit is 0, otherwise return 0
 */
bool bq769x0::posCheckBeforeShortingPins(int pos, int data, int shortingPinConfiguration)
{
  return(checkPosBefore(pos, data) && isShortingPinsInactive(data, shortingPinConfiguration));
}

/**
 * Method to check bit value after the shorting pins (9,10,11,12,13,14 cell configuration)
 * return 1 if bit is 0, otherwise return 0
 */
bool bq769x0::posCheckAfterShortingPins(int pos, int data, int shortingPinConfiguration)
{
  return(isShortingPinsInactive(data, shortingPinConfiguration) && checkPosAfter(pos, data));
}

/**
 * Method to check whether bit is in shorting pins, return 1 if the bit is in shorting pins, otherwise return 0
 */
bool bq769x0::isPosInShortingPins(int pos, int shortingPinConfiguration)
{
  if (shortingPinConfiguration == _TRIPLE_SHORT)
  {
    for (int x : _shortingPins.pinList1)
    {
      if (x == pos)
      {
        return 1;
      }
    }
  }
  else if (shortingPinConfiguration == _DOUBLE_SHORT)
  {
    for (int x : _shortingPins.pinList2)
    {
      if (x == pos)
      {
        return 1;
      }
    }
  }
  return 0;    
}

/**
 * if the pos is in shorting pins, this method will check before the first shorting pins and after the last shorting pins
 * it will also check if any of the pin state in shorting pin. It will bitwise AND every of them, if one of them is not valid
 * it will return 0, otherwise return 1
 */
bool bq769x0::posCheckInShortingPins(int pos, int data, int shortingPinConfiguration)
{
  if (shortingPinConfiguration == _TRIPLE_SHORT)
  {
    return(checkPosBefore(_shortingPins.pinList1[0], data) && checkPosAfter(_shortingPins.pinList1[2], data) && isShortingPinsInactive(data, shortingPinConfiguration));
  }
  else if (shortingPinConfiguration == _DOUBLE_SHORT)
  {
    return(checkPosBefore(_shortingPins.pinList2[0], data) && checkPosAfter(_shortingPins.pinList2[1], data) && isShortingPinsInactive(data, shortingPinConfiguration));
  }
  return 0;    
}

/**
 * if the pos is in normal position, it will only check on adjacent pin, after and before the selected position
 */
bool bq769x0::posCheckNormalPins(int pos, int data)
{
  // Normal Check
  if ((pos < 4) && (pos > 0))
  {
    return(checkPosAfter(pos, data) && checkPosBefore(pos, data));
  }
  else if (pos == 4)
  {
    return(checkPosBefore(pos, data));
  }
  else if (pos == 0)
  {
    return(checkPosAfter(pos, data));
  }
  return 0;
}

/**
 * Method to check for triple short pins configuration, ex: 9 cell configuration
 */
bool bq769x0::tripleShortCheck(int pos, int data)
{
  if (pos == (_shortingPins.pinList1[0] - 1))
  {
    return posCheckBeforeShortingPins(pos, data, _TRIPLE_SHORT);
  }
  else if (pos == (_shortingPins.pinList1[2] + 1))
  {
    return posCheckAfterShortingPins(pos, data, _TRIPLE_SHORT);
  }
  else if (isPosInShortingPins(pos, _TRIPLE_SHORT))
  {
    return(posCheckInShortingPins(pos, data, _TRIPLE_SHORT));
  }
  else
  {
    return posCheckNormalPins(pos, data);
  }
}

/**
 * Method to check for double short pins configuration, ex: 10 cell configuration on CELLBAL1
 */
bool bq769x0::doubleShortCheck(int pos, int data)
{
  if (pos == (_shortingPins.pinList2[0] - 1))
  {
    return posCheckBeforeShortingPins(pos, data, _DOUBLE_SHORT);
  }
  else if (pos == (_shortingPins.pinList2[1] + 1))
  {
    return posCheckAfterShortingPins(pos, data, _DOUBLE_SHORT);
  }
  else if (isPosInShortingPins(pos, _DOUBLE_SHORT))
  {
    return(posCheckInShortingPins(pos, data, _DOUBLE_SHORT));
  }
  else
  {
    return posCheckNormalPins(pos, data);
  }
}

/**
 * Method to check for no short pins configuration, ex: 15 cell configuration
 */
bool bq769x0::noShortCheck(int pos, int data)
{
  return posCheckNormalPins(pos, data);
}

/**
 * Method to check if the requested position is valid (no adjacent cell is active)
 */
bool bq769x0::isValidPosition(int pos, int data, int cellBallAddr)
{
  return pinCheck(pos, data, cellBallAddr);
}

/**
 * Method to check the validity pin position on every cell configuration
 */
bool bq769x0::pinCheck(int pos, int data, int cellBalAddr)
{
  if (_cellConfiguration == CELL_9) //9 cell
  {
    // Serial.println("CELL 9");
    return tripleShortCheck(pos, data);
  }

  if (_cellConfiguration == CELL_10) //10 cell
  {
    // Serial.println("CELL 10");
    if(cellBalAddr != 0x01) //First Cell Stack
    {
      return tripleShortCheck(pos, data);
    }
    else
    {
      return doubleShortCheck(pos, data);
    }
  }
  if (_cellConfiguration == CELL_11) //11 cell
  {
    // Serial.println("CELL 11");
    if(cellBalAddr != 0x03) //Third Cell Stack
    {
      return doubleShortCheck(pos, data);
    }
    else
    {
      return tripleShortCheck(pos, data);
    }
  }
  if (_cellConfiguration == CELL_12) //12 cell
  {
    // Serial.println("CELL 12");
    return doubleShortCheck(pos, data);
  }
  if (_cellConfiguration == CELL_13) //13 cell
  {
    // Serial.println("CELL 13");
    if (cellBalAddr != 0x01)
    {
      return doubleShortCheck(pos, data);
    }
    else
    {
      return noShortCheck(pos, data);
    }
  }
  if (_cellConfiguration == CELL_14) //14 cell
  {
    // Serial.println("CELL 14");
    if (cellBalAddr != 0x03)
    {
      return noShortCheck(pos, data);
    }
    else
    {
      return doubleShortCheck(pos, data);
    }
  }
  if (_cellConfiguration == CELL_15) //15 cell
  {
    // Serial.println("CELL 15");
    return noShortCheck(pos, data);
  }
}

void bq769x0::setShuntResistorValue(int res_mOhm)
{
  shuntResistorValue_mOhm = res_mOhm;
}

void bq769x0::setThermistorBetaValue(int beta_K)
{
  thermistorBetaValue = beta_K;
}

void bq769x0::setTemperatureLimits(int minDischarge_degC, int maxDischarge_degC, 
  int minCharge_degC, int maxCharge_degC)
{
  // Temperature limits (°C/10)
  minCellTempDischarge = minDischarge_degC * 10;
  maxCellTempDischarge = maxDischarge_degC * 10;
  minCellTempCharge = minCharge_degC * 10;
  maxCellTempCharge = maxCharge_degC * 10;  
}

void bq769x0::setIdleCurrentThreshold(int current_mA)
{
  idleCurrentThreshold = current_mA;
}


//----------------------------------------------------------------------------

long bq769x0::setShortCircuitProtection(long current_mA, int delay_us)
{
  LOG_PRINTLN("setSCD");
  regPROTECT1_t protect1;
  
  // only RSNS = 1 considered
  protect1.bits.RSNS = 1;

  protect1.bits.SCD_THRESH = 0;
  for (int i = sizeof(SCD_threshold_setting) / sizeof(SCD_threshold_setting[0]) - 1; i > 0; i--) {
    if (current_mA * shuntResistorValue_mOhm / 1000 >= SCD_threshold_setting[i]) {
      protect1.bits.SCD_THRESH = i;
      LOG_PRINT("SCD threshold: ");
      LOG_PRINTLN(i);
      break;
    }
  }
  
  protect1.bits.SCD_DELAY = 0;
  for (int i = sizeof(SCD_delay_setting) / sizeof(SCD_delay_setting[0]) - 1; i > 0; i--) {
    if (delay_us >= SCD_delay_setting[i]) {
      protect1.bits.SCD_DELAY = i;
      LOG_PRINT("SCD delay: ");
      LOG_PRINTLN(i);
      break;
    }
  }
  
  writeRegister(PROTECT1, protect1.regByte);
  
  // returns the actual current threshold value
  return (long)SCD_threshold_setting[protect1.bits.SCD_THRESH] * 1000 / 
    shuntResistorValue_mOhm;
}

//----------------------------------------------------------------------------

long bq769x0::setOvercurrentChargeProtection(long current_mA, int delay_ms)
{
  // ToDo: Software protection for charge overcurrent
}

//----------------------------------------------------------------------------

long bq769x0::setOvercurrentDischargeProtection(long current_mA, int delay_ms)
{
  LOG_PRINTLN("setOCD");
  regPROTECT2_t protect2;
  
  // Remark: RSNS must be set to 1 in PROTECT1 register

  protect2.bits.OCD_THRESH = 0;
  for (int i = sizeof(OCD_threshold_setting) / sizeof(OCD_threshold_setting[0]) - 1; i > 0; i--) {
    if (current_mA * shuntResistorValue_mOhm / 1000 >= OCD_threshold_setting[i]) {
      protect2.bits.OCD_THRESH = i;
      LOG_PRINT("OCD threshold: ");
      LOG_PRINTLN(i);
      break;
    }
  }
  
  protect2.bits.OCD_DELAY = 0;
  for (int i = sizeof(OCD_delay_setting) / sizeof(OCD_delay_setting[0]) - 1; i > 0; i--) {
    if (delay_ms >= OCD_delay_setting[i]) {
      protect2.bits.OCD_DELAY = i;
      LOG_PRINT("OCD delay: ");
      LOG_PRINTLN(i);
      break;
    }
  }
  
  writeRegister(PROTECT2, protect2.regByte);
 
  // returns the actual current threshold value
  return (long)OCD_threshold_setting[protect2.bits.OCD_THRESH] * 1000 / 
    shuntResistorValue_mOhm;
}


//----------------------------------------------------------------------------

int bq769x0::setCellUndervoltageProtection(int voltage_mV, int delay_s)
{
  LOG_PRINTLN("setUVP");
  regPROTECT3_t protect3;
  byte uv_trip = 0;
  
  minCellVoltage = voltage_mV;
  
  protect3.regByte = readRegister(PROTECT3);
  
  uv_trip = ((((long)voltage_mV - adcOffset) * 1000 / adcGain) >> 4) & 0x00FF;
  uv_trip += 1;   // always round up for lower cell voltage
  writeRegister(UV_TRIP, uv_trip);
  
  protect3.bits.UV_DELAY = 0;
  for (int i = sizeof(UV_delay_setting)-1; i > 0; i--) {
    if (delay_s >= UV_delay_setting[i]) {
      protect3.bits.UV_DELAY = i;
      LOG_PRINT("UV_DELAY: ");
      LOG_PRINTLN(i);
      break;
    }
  }
  
  writeRegister(PROTECT3, protect3.regByte);
  
  // returns the actual current threshold value
  return ((long)1 << 12 | uv_trip << 4) * adcGain / 1000 + adcOffset;
}

//----------------------------------------------------------------------------

int bq769x0::setCellOvervoltageProtection(int voltage_mV, int delay_s)
{
  LOG_PRINTLN("setOVP");
  regPROTECT3_t protect3;
  byte ov_trip = 0;

  maxCellVoltage = voltage_mV;
  
  protect3.regByte = readRegister(PROTECT3);
  
  ov_trip = ((((long)voltage_mV - adcOffset) * 1000 / adcGain) >> 4) & 0x00FF;
  writeRegister(OV_TRIP, ov_trip);
    
  protect3.bits.OV_DELAY = 0;
  for (int i = sizeof(OV_delay_setting)-1; i > 0; i--) {
    if (delay_s >= OV_delay_setting[i]) {
      protect3.bits.OV_DELAY = i;
      LOG_PRINT("OV_DELAY: ");
      LOG_PRINTLN(i);
      break;
    }
  }
  
  writeRegister(PROTECT3, protect3.regByte);
 
  // returns the actual current threshold value
  return ((long)1 << 13 | ov_trip << 4) * adcGain / 1000 + adcOffset;
}


//----------------------------------------------------------------------------

long bq769x0::getBatteryCurrent()
{
  return batCurrent;
}

//----------------------------------------------------------------------------

long bq769x0::getBatteryVoltage()
{
  return batVoltage;
}

//----------------------------------------------------------------------------

int bq769x0::getMaxCellVoltage()
{
  return cellVoltages[idCellMaxVoltage];
}

//----------------------------------------------------------------------------

int bq769x0::getCellVoltage(byte idCell)
{
  return cellVoltages[idCell-1];
}


//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegC(byte channel)
{
  if (channel >= 1 && channel <= 3) {
    return (float)temperatures[channel-1] / 10.0;
  }
  else
    return -273.15;   // Error: Return absolute minimum temperature
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegF(byte channel)
{
  return getTemperatureDegC(channel) * 1.8 + 32;
}


//----------------------------------------------------------------------------
/*
void bq769x0::updateTemperatures()
{
  float tmp = 0;
  int adcVal = 0;
  int vtsx = 0;
  unsigned long rts = 0;
  
  Wire.beginTransmission(I2CAddress);
  Wire.write(0x2C);
  Wire.endTransmission();
  
  if (Wire.requestFrom(I2CAddress, 2) == 2)
  {
    // calculate R_thermistor according to bq769x0 datasheet
    adcVal = ((Wire.read() & B00111111) << 8) | Wire.read();
    vtsx = adcVal * 0.382; // mV
    rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
        
    // Temperature calculation using Beta equation
    // - According to bq769x0 datasheet, only 10k thermistors should be used
    // - 25°C reference temperature for Beta equation assumed
    tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue*log(rts/10000.0)); // K
    
    temperatures[0] = (tmp - 273.15) * 10.0;
  }
}
*/

/**
 * Method to get the temperature data from device, it is then stored into local storage
 * to get the value, call getTemperatureDegC or getTemperatureDegF
 */
void bq769x0::updateTemperatures()
{
  float tmp = 0;
  int adcVal = 0;
  int vtsx = 0;
  unsigned long rts = 0;
  
  if (type == bq76940)
  {
    for (int i = 0; i < 3; i ++)
    {
      _wire->beginTransmission(I2CAddress);
      _wire->write(TS1_HI_BYTE + (i * 2));
      _wire->endTransmission();
      
      if (_wire->requestFrom(I2CAddress, 2) == 2)
      {
        // calculate R_thermistor according to bq769x0 datasheet
        adcVal = ((_wire->read() & B00111111) << 8) | _wire->read();
        vtsx = adcVal * 0.382; // mV
        rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
            
        // Temperature calculation using Beta equation
        // - According to bq769x0 datasheet, only 10k thermistors should be used
        // - 25°C reference temperature for Beta equation assumed
        tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue*log(rts/10000.0)); // K
        
        temperatures[i] = (tmp - 273.15) * 10.0;
      }
    }
  }
  if (type == bq76930)
  {
    for (int i = 0; i < 2; i ++)
    {
      _wire->beginTransmission(I2CAddress);
      _wire->write(TS1_HI_BYTE + (i * 2));
      _wire->endTransmission();
      
      if (_wire->requestFrom(I2CAddress, 2) == 2)
      {
        // calculate R_thermistor according to bq769x0 datasheet
        adcVal = ((_wire->read() & B00111111) << 8) | _wire->read();
        vtsx = adcVal * 0.382; // mV
        rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
            
        // Temperature calculation using Beta equation
        // - According to bq769x0 datasheet, only 10k thermistors should be used
        // - 25°C reference temperature for Beta equation assumed
        tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue*log(rts/10000.0)); // K
        
        temperatures[i] = (tmp - 273.15) * 10.0;
      }
    }
  }
  if (type == bq76920)
  {
    _wire->beginTransmission(I2CAddress);
    _wire->write(TS1_HI_BYTE );
    _wire->endTransmission();
    
    if (_wire->requestFrom(I2CAddress, 2) == 2)
    {
      // calculate R_thermistor according to bq769x0 datasheet
      adcVal = ((_wire->read() & B00111111) << 8) | _wire->read();
      vtsx = adcVal * 0.382; // mV
      rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
          
      // Temperature calculation using Beta equation
      // - According to bq769x0 datasheet, only 10k thermistors should be used
      // - 25°C reference temperature for Beta equation assumed
      tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue*log(rts/10000.0)); // K
      
      temperatures[0] = (tmp - 273.15) * 10.0;
    }
  }
}


//----------------------------------------------------------------------------
// If ignoreCCReadFlag == true, the current is read independent of an interrupt
// indicating the availability of a new CC reading

void bq769x0::updateCurrent(bool ignoreCCReadyFlag)
{
  LOG_PRINTLN("updateCurrent");
  int16_t adcVal = 0;
  regSYS_STAT_t sys_stat;
  sys_stat.regByte = readRegister(SYS_STAT);
  
  if (ignoreCCReadyFlag == true || sys_stat.bits.CC_READY == 1)
  {
    adcVal = (readRegister(0x32) << 8) | readRegister(0x33);
    batCurrent = adcVal * 8.44 / shuntResistorValue_mOhm;  // mA

    if (batCurrent > -10 && batCurrent < 10)
    {
      batCurrent = 0;
    }
    
    // reset idleTimestamp
    if (batCurrent > idleCurrentThreshold) {
      idleTimestamp = millis();
    }

    // no error occured which caused alert
    if (!(sys_stat.regByte & B00111111)) {
      alertInterruptFlag = false;
    }

    writeRegister(SYS_STAT, B10000000);  // Clear CC ready flag	
    LOG_PRINTLN("updateCurrent: updated, CC flag cleared");
  }
}

//----------------------------------------------------------------------------
// reads all cell voltages and updates batVoltage
// now supports CRC, taken from mbed version of LibreSolar firmware (thanks to mikethezipper)

/**
 * Method to get the data voltages from device, it is then stored into local storage
 * to get the value, call getCellVoltage
 */
void bq769x0::updateVoltages()
{
  long adcVal = 0;
  char buf[4];
  int connectedCells = 0;
  idCellMaxVoltage = 0; //resets to zero before writing values to these vars
  idCellMinVoltage = 0;

  uint8_t crc;
  crc = 0;  
  // Serial.println("========Voltage Measurement==========");
  // Serial.println("TCA Channel : " + String(_channel));

  /****************************************************\
    Note that each cell voltage is 14 bits stored across two 8 bit register locations in the BQ769x0 chip.
    This means that first we need to read register HI (in first instance this is VC1_HI_BYTE), 
    however this 8 bit piece of data has two worthless first digits - garbage.
    To remove the first two bits, the bitwise & is used. By saying A & 00111111, only the last 6 bits of A are used. 
    Meanwhile all of the 8 bits on the low side are used. So the overall reading is the last 6 bits of high in front of the 8 bits from low.
    To add the hi and lo readings together, the << is used to shift the hi value over by 8 bits, then adding it to the 8 bits.
    This is done by using the OR operator |. So the total thing looks like: adcVal = (VC_HI_BYTE & 0b00111111) << 8 | VC_LO_BYTE;
  \****************************************************/

  // will run once for each cell up to the total num of cells
  for (int i = 0; i < numberOfCells; i++) {
    
    if (crcEnabled == true) {
      // refer to datasheet at 10.3.1.4 "Communications Subsystem"
      buf[0] = (char) VC1_HI_BYTE + (2*i); // start with the first cell
      _wire->beginTransmission(I2CAddress);
      _wire->write(buf[0]);     // tell slave that this is the address it is interested in
      // Wire.write(buf[0] + (i*2));     // tell slave that this is the address it is interested in
      _wire->endTransmission(); // end transmission so that read can begin
      _wire->requestFrom(I2CAddress, 4);  // request 4 bytes: 1) VCx_HI - 2) VCx_HI CRC - 3) VCx_LO - 4) VCx_LO CRC
      buf[0] = _wire->read();             // VCx_HI - note that only bottom 6 bits are good
      buf[1] = _wire->read();             // VCx_HI CRC - done on address and data
      buf[2] = _wire->read();             // VCx_LO - all 8 bits are used
      buf[3] = _wire->read();             // VCx_LO CRC - done on just the data byte
      
      // check if CRC matches data bytes
      // CRC of first byte includes slave address (including R/W bit) and data
      crc = _crc8_ccitt_update(0, (I2CAddress << 1) | 1);
      crc = _crc8_ccitt_update(crc, buf[0]);
      // if (crc != buf[1]){
      //   LOG_PRINTLN("VCxHI CRC doesn't match");
      //   return; // to exit without saving corrupted value
      // }
        
      // CRC of subsequent bytes contain only data
      crc = _crc8_ccitt_update(0, buf[2]);
      // if (crc != buf[3]) {
      //   LOG_PRINTLN("VCxLO CRC doesn't match");
      //   return; // to exit without saving corrupted value
      // }
    } 

    // if CRC is disabled only read 2 bytes and call it a day :)
    else { 
      // Serial.println("No CRC");
      _wire->requestFrom(I2CAddress, 2);
      buf[0] = _wire->read(); // VCx_HI - note that only bottom 6 bits are good
      buf[2] = _wire->read(); // VCx_LO - all 8 bits are used
    }

    // combine VCx_HI and VCx_LO bits and calculate cell voltage
    adcVal = (buf[0] & 0b00111111) << 8 | buf[2];           // read VCx_HI bits and drop the first two bits, shift left then append VCx_LO bits
    cellVoltages[i] = adcVal * adcGain / 1000 + adcOffset;  // calculate real voltage in mV
    // Serial.print("Cell Voltage " + String(i+1) + " : ");
    // Serial.println(cellVoltages[i]);
  
    // filter out voltage readings from unconnected cell(s)
    if (cellVoltages[i] > 500) {  
      connectedCells++; // add one to the temporary cell counter var - only readings above 500mV are counted towards real cell count
    }

    if (cellVoltages[i] > cellVoltages[idCellMaxVoltage]) {
      idCellMaxVoltage = i;
    }

    if (cellVoltages[i] < cellVoltages[idCellMinVoltage] && cellVoltages[i] > 500) {
      idCellMinVoltage = i;
    }
  }
  
  long adcValPack = ((readRegister(BAT_HI_BYTE) << 8) | readRegister(BAT_LO_BYTE)) & 0b1111111111111111;
  batVoltage = 4 * adcGain * adcValPack / 1000 + (connectedCells * adcOffset); // in original LibreSolar, connectedCells is converted to byte, maybe to reduce bit size
  // Serial.print("Pack Voltage : ");
  // Serial.println(batVoltage);
  // Serial.println("===============================");
}

//----------------------------------------------------------------------------
// now supports CRC, taken from mbed version of LibreSolar firmware (thanks to mikethezipper)

void bq769x0::writeRegister(byte address, int data)
{
  if(_wire <= 0 || _wire == NULL)
  {
    // Serial.println("I2C not defined");
    return;
  }
  if (_listener > 0)
  {
    _listener(_channel);
  }
  LOG_PRINT("write: ");
  LOG_PRINT(byte2char(address));
  LOG_PRINT(" --> ");
  LOG_PRINT(byte2char(data));
  uint8_t crc = 0;
  char buf[3];
  buf[0] = (char) address;
  buf[1] = data;

  // note that writes to the bq769x0 IC are: 1) start - 2) address - 3) address - 4) data - 5) CRC8 - 6) stop bit
  _wire->beginTransmission(I2CAddress); // writes start bit - the first step
  _wire->write(buf[0]);                 // writes register address
  _wire->write(buf[1]);                 // writes data - the fourth step
 
  if (crcEnabled == true) {
    // CRC is calculated over the slave address (including R/W bit), register address, and data.
    crc = _crc8_ccitt_update(crc, (I2CAddress << 1) | 0);
    crc = _crc8_ccitt_update(crc, buf[0]);
    crc = _crc8_ccitt_update(crc, buf[1]);
    buf[2] = crc;

    _wire->write(buf[2]); // writes CRC
    LOG_PRINT(" CRC:");
    LOG_PRINT(byte2char(buf[2]));
  }

  _wire->endTransmission();
  LOG_PRINTLN();
}

/**
 * Public method for writing register into device
 */
void bq769x0::writeReg(byte address, int data)
{
  writeRegister(address, data);
}


//----------------------------------------------------------------------------

int bq769x0::readRegister(byte address)
{  
  if(_wire <= 0 || _wire == NULL)
  {
    Serial.println("i2c null");
    return -1;
  }
  if (_listener > 0)
  {
    _listener(_channel);
  }
  _wire->beginTransmission(I2CAddress);
  _wire->write(address);
  _wire->endTransmission();
  _wire->requestFrom(I2CAddress, 1);
  return _wire->read();
}


/**
 * Public method for reading register from device
 */
int bq769x0::readReg(byte address)
{  
  return readRegister(address);
}

//----------------------------------------------------------------------------
// supports CRC, taken from mbed version of LibreSolar firmware
// however often freeze momentarily at "while (crc != buf[1]);"

// int bq769x0::readRegister(byte address)
// {
//   LOG_PRINT("read: ");
//   LOG_PRINTLN(byte2char(address));

//   uint8_t crc = 0;
//   char buf[2];
//   buf[0] = (char) address;

//   Wire.beginTransmission(I2CAddress);
//   Wire.write(buf[0]);
//   Wire.endTransmission();

//   if (crcEnabled == true) {
//     do {
//       Wire.requestFrom(I2CAddress, 2);
//       buf[0] = Wire.read();
//       buf[1] = Wire.read();
//       // CRC is calculated over the slave address (including R/W bit) and data.
//       crc = _crc8_ccitt_update(crc, (I2CAddress << 1) | 1);
//       crc = _crc8_ccitt_update(crc, buf[0]);
//     } while (crc != buf[1]);
//     return buf[0];
//   }
//   else {
//     Wire.requestFrom(I2CAddress, 1);
//     return Wire.read();
//   }
// }

//----------------------------------------------------------------------------
// the actual ISR, called by static function alertISR()

void bq769x0::setAlertInterruptFlag()
{
  interruptTimestamp = millis();
  alertInterruptFlag = true;
}

//----------------------------------------------------------------------------
// The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
// a new value (either new CC reading or an error)

void bq769x0::alertISR()
{
  if (instancePointer != 0)
  {
    instancePointer->setAlertInterruptFlag();
  }
}

//----------------------------------------------------------------------------
// for debug purposes
#if BQ769X0_DEBUG
  void bq769x0::printRegisters()
  {
    LOG_PRINT(F("0x00 SYS_STAT:  "));
    LOG_PRINTLN(byte2char(readRegister(SYS_STAT)));

    LOG_PRINT(F("0x01 CELLBAL1:  "));
    LOG_PRINTLN(byte2char(readRegister(CELLBAL1)));

    LOG_PRINT(F("0x02 CELLBAL2:  "));
    LOG_PRINTLN(byte2char(readRegister(CELLBAL2)));

    LOG_PRINT(F("0x03 CELLBAL3:  "));
    LOG_PRINTLN(byte2char(readRegister(CELLBAL3)));

    LOG_PRINT(F("0x04 SYS_CTRL1: "));
    LOG_PRINTLN(byte2char(readRegister(SYS_CTRL1)));
    
    LOG_PRINT(F("0x05 SYS_CTRL2: "));
    LOG_PRINTLN(byte2char(readRegister(SYS_CTRL2)));
    
    LOG_PRINT(F("0x06 PROTECT1:  "));
    LOG_PRINTLN(byte2char(readRegister(PROTECT1)));
    
    LOG_PRINT(F("0x07 PROTECT2:  "));
    LOG_PRINTLN(byte2char(readRegister(PROTECT2)));
    
    LOG_PRINT(F("0x08 PROTECT3   "));
    LOG_PRINTLN(byte2char(readRegister(PROTECT3)));
    
    LOG_PRINT(F("0x09 OV_TRIP:   "));
    LOG_PRINTLN(byte2char(readRegister(OV_TRIP)));
    
    LOG_PRINT(F("0x0A UV_TRIP:   "));
    LOG_PRINTLN(byte2char(readRegister(UV_TRIP)));
    
    LOG_PRINT(F("0x0B CC_CFG:    "));
    LOG_PRINTLN(byte2char(readRegister(CC_CFG)));

    LOG_PRINT(F("0x2A BAT_HI:     "));
    LOG_PRINTLN(byte2char(readRegister(BAT_HI_BYTE)));

    LOG_PRINT(F("0x2B BAT_LO:     "));
    LOG_PRINTLN(byte2char(readRegister(BAT_LO_BYTE)));

    LOG_PRINT(F("0x32 CC_HI:     "));
    LOG_PRINTLN(byte2char(readRegister(CC_HI_BYTE)));

    LOG_PRINT(F("0x33 CC_LO:     "));
    LOG_PRINTLN(byte2char(readRegister(CC_LO_BYTE)));

    // LOG_PRINT(F("0x50 ADCGAIN1:  "));
    // LOG_PRINTLN(byte2char(readRegister(ADCGAIN1)));

    // LOG_PRINT(F("0x51 ADCOFFSET: "));
    // LOG_PRINTLN(byte2char(readRegister(ADCOFFSET)));

    // LOG_PRINT(F("0x59 ADCGAIN2:  "));
    // LOG_PRINTLN(byte2char(readRegister(ADCGAIN2)));
  }
#endif