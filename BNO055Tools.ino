
void BNOWriteRegister(uint8_t regAddress, uint8_t val) {
  Wire.beginTransmission(BNO055_Address); // start transmission to device
  Wire.write(regAddress);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
  delay(10);
}
void BNORestoreCalibration(uint8_t* calibData)
{
  uint8_t lastMode = BNOreadRegister(OPR_MODE_REG);
  // bno.setMode(0x00);
  SetBNO055Mode(0x00);
  delay(25);
  BNOWriteRegister(ACCEL_OFFSET_X_LSB_REG, calibData[0]);
  BNOWriteRegister(ACCEL_OFFSET_X_MSB_REG, calibData[1]);
  BNOWriteRegister(ACCEL_OFFSET_Y_LSB_REG, calibData[2]);
  BNOWriteRegister(ACCEL_OFFSET_Y_MSB_REG, calibData[3]);
  BNOWriteRegister(ACCEL_OFFSET_Z_LSB_REG, calibData[4]);
  BNOWriteRegister(ACCEL_OFFSET_Z_MSB_REG, calibData[5]);

  BNOWriteRegister(GYRO_OFFSET_X_LSB_REG, calibData[6]);
  BNOWriteRegister(GYRO_OFFSET_X_MSB_REG, calibData[7]);
  BNOWriteRegister(GYRO_OFFSET_Y_LSB_REG, calibData[8]);
  BNOWriteRegister(GYRO_OFFSET_Y_MSB_REG, calibData[9]);
  BNOWriteRegister(GYRO_OFFSET_Z_LSB_REG, calibData[10]);
  BNOWriteRegister(GYRO_OFFSET_Z_MSB_REG, calibData[11]);

  BNOWriteRegister(MAG_OFFSET_X_LSB_REG, calibData[12]);
  BNOWriteRegister(MAG_OFFSET_X_MSB_REG, calibData[13]);
  BNOWriteRegister(MAG_OFFSET_Y_LSB_REG, calibData[14]);
  BNOWriteRegister(MAG_OFFSET_Y_MSB_REG, calibData[15]);
  BNOWriteRegister(MAG_OFFSET_Z_LSB_REG, calibData[16]);
  BNOWriteRegister(MAG_OFFSET_Z_MSB_REG, calibData[17]);

  BNOWriteRegister(ACCEL_RADIUS_LSB_REG, calibData[18]);
  BNOWriteRegister(ACCEL_RADIUS_MSB_REG, calibData[19]);

  BNOWriteRegister(MAG_RADIUS_LSB_REG, calibData[20]);
  BNOWriteRegister(MAG_RADIUS_MSB_REG, calibData[21]);
  delay(20);
  BNOPrintCalibration();
  SetBNO055Mode(lastMode);
}
void BNOPrintCalibration()
{
  uint8_t savMode = BNOreadRegister(OPR_MODE_REG);
  uint8_t buff[22];
  SetBNO055Mode(0x00);
  BNOreadLenRegister(ACCEL_OFFSET_X_LSB_REG, &buff[0], 22);

  Serial.print("accel offset x: ");
  uint16_t accOffsetX = (((uint16_t)buff[1] << 8) | (uint16_t)buff[0]);
  Serial.print(accOffsetX);
  Serial.print(" y: ");
  uint16_t accOffsetY = (((uint16_t)buff[3] << 8) | (uint16_t) buff[2]);
  Serial.print(accOffsetY);
  Serial.print(" z: ");
  uint16_t accOffsetZ = (((uint16_t)buff[5] << 8) | (uint16_t) buff[4]);
  Serial.println(accOffsetZ);
  Serial.print("mag offset x:");
  uint16_t magOffsetX = (((uint16_t)buff[7] << 8) | (uint16_t)buff[6]);
  Serial.print(magOffsetX);
  Serial.print(" y: ");
  uint16_t magOffsetY = (((uint16_t)buff[9] << 8) | (uint16_t)buff[8]);
  Serial.print(magOffsetY);
  Serial.print(" z: ");
  uint16_t magOffsetZ = (((uint16_t)buff[11] << 8) | (uint16_t)buff[10]);
  Serial.println(magOffsetZ);
  Serial.print("gyro offset x:");
  uint16_t gyroOffsetX = (((uint16_t)buff[13] << 8) | (uint16_t)buff[12]);
  Serial.print(gyroOffsetX);
  Serial.print(" y:");
  uint16_t gyroOffsetY = (((uint16_t)buff[15] << 8) | (uint16_t)buff[14]);
  Serial.print(gyroOffsetY);
  Serial.print("  z:");
  uint16_t gyroOffsetZ = (((uint16_t)buff[17] << 8) | (uint16_t) buff[16]);
  Serial.println(gyroOffsetZ);
  Serial.print("accel radius:");
  uint16_t accelRadius = (((uint16_t)buff[19] << 8) | (uint16_t)buff[18]);
  Serial.println(accelRadius);
  Serial.print("mag radius:");
  uint16_t magRadius = (((uint16_t)buff[21] << 8) | (uint16_t) buff[20]);
  Serial.println(magRadius);

  SetBNO055Mode(savMode);
}
uint8_t BNOreadRegister( uint8_t regAddress) {
  uint8_t v;
  Wire.beginTransmission(BNO055_Address);
  Wire.write(regAddress); // register to read
  Wire.endTransmission();
  Wire.requestFrom(BNO055_Address, 1); // read a byte
  while (!Wire.available()) {
    // waiting
  }
  v = Wire.read();
  return v;
}

boolean BNOreadLenRegister(uint8_t regAddress, byte * buff, uint8_t len)
{
  Wire.beginTransmission(BNO055_Address);
  Wire.write(regAddress); // register to read
  Wire.endTransmission();
  Wire.requestFrom(BNO055_Address, (byte)len); // read a byte
  for (uint8_t i = 0; i < len; i++)
  {
    buff[i] = Wire.read();
  }
  return true;
}

boolean BNOInit()
{
  Wire.begin();
  int eeAddress = 0;
  uint8_t id = BNOreadRegister(CHIP_ID_REG);
  uint8_t calibData[22];

  //  Serial.println(id, HEX);
  if (id != BNO055_ID)
  {
    delay(1000); // hold on for boot
    id = BNOreadRegister(CHIP_ID_REG);
    //   Serial.println(id, HEX);
    if (id != BNO055_ID) {
      return false;  // still not? ok bail
    }

  }
  BNOWriteRegister(OPR_MODE_REG, OP_MODE_CONFIG);      // set config mode
  delay(100);
  BNOWriteRegister(SYS_TRIGGER_REG, 0x20);           // reset
  delay(1000);
  id = 0x00;
  while (id != BNO055_ID)
  {
    delay(100);
    id = BNOreadRegister(CHIP_ID_REG);
  }
  BNOWriteRegister(PWR_MODE_REG, POWER_MODE_NORMAL);
  delay(10);
  BNOWriteRegister(PAGE_ID_REG, 0);
  BNOWriteRegister(SYS_TRIGGER_REG, 0x00);           // reset
  delay(10);
  //  SetBNO055Mode(OP_MODE_NDOF);
  delay(20);
  return true;
}

void SetBNO055Mode(uint8_t mode)
{
  BNOWriteRegister(OPR_MODE_REG, OP_MODE_CONFIG);      // set config mode
  delay(100);                                         // wait for BNO
  BNOWriteRegister(OPR_MODE_REG, mode);      // set  mode
  delay(100);                               // wait for BNO
  // WriteRegister(BNO055_Address, Adafruit_BNO055::BNO055_OPR_MODE_ADDR, mode);
  pendingInitRelativeHeading = true;
  Serial.print("new mode:0x");
  Serial.println(BNOreadRegister(OPR_MODE_REG), HEX );
//  delay(30);

}



void BNOSetInterruptEnable(uint8_t intValue)
{
  uint8_t savMode = BNOreadRegister(OPR_MODE_REG);
  Serial.print("sav mode:0x");
  Serial.println(savMode, HEX);
  SetBNO055Mode(0x00);
  delay(25);
  BNOWriteRegister(PAGE_ID_REG, 1);
  Serial.print("\t");
  Serial.print(BNOreadRegister(INT_EN_REG), HEX);
  Serial.print("\t");
  Serial.println(BNOreadRegister(INT_MSK_REG), HEX);
  BNOWriteRegister(INT_EN_REG, intValue);
  BNOWriteRegister(INT_MSK_REG, intValue);
  BNOWriteRegister(ACC_INT_settings_REG, 0b00011101);
  BNOWriteRegister(ACC_NM_SET_REG, 0x01);
  BNOWriteRegister(ACC_NM_THRE_REG, 0x02);
  Serial.print("\t");
  Serial.print(BNOreadRegister(INT_EN_REG), HEX);
  Serial.print("\t");
  Serial.println(BNOreadRegister(INT_MSK_REG), HEX);
  BNOWriteRegister(PAGE_ID_REG, 0);
  //  delay(15);
  /* Set the requested operating mode (see section 3.3) */
  SetBNO055Mode(savMode);
  delay(20);
}


vector GetVector(uint8_t vectorType)
{
  uint8_t buff[6];
  memset (buff, 0, 6);

  double x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  BNOreadLenRegister(vectorType, buff, 6);

  x = ((int16_t)buff[0]) | (((int16_t)buff[1]) << 8);
  y = ((int16_t)buff[2]) | (((int16_t)buff[3]) << 8);
  z = ((int16_t)buff[4]) | (((int16_t)buff[5]) << 8);

  /* Convert the value to an appropriate range (section 3.6.4) */
  /* and assign the value to the Vector type */
  switch (vectorType)
  {
    case vector_MAGNETOMETER:
      /* 1uT = 16 LSB */
      x = ((double)x) / 16.0;
      y = ((double)y) / 16.0;
      z = ((double)z) / 16.0;
      break;
    case vector_GYROSCOPE:
      /* 1rps = 900 LSB */
      x = ((double)x) / 900.0;
      y = ((double)y) / 900.0;
      z = ((double)z) / 900.0;
      break;
    case vector_EULER:
      /* 1 degree = 16 LSB */
      x = ((double)x) / 16.0;
      y = ((double)y) / 16.0;
      z = ((double)z) / 16.0;
      break;
    case vector_ACCELEROMETER:
    case vector_LINEARACCEL:
    case vector_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      x = ((double)x) / 100.0;
      y = ((double)y) / 100.0;
      z = ((double)z) / 100.0;
      break;
  }

  return {x, y, z};
}

void BNOdisplayCalStatus()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */


  uint8_t calStat = BNOreadRegister(CALIB_STAT_REG);
  Serial.print(" Calib_Status:");
  Serial.print(calStat, HEX);
  Serial.print(" Sys:");
  Serial.print(((calStat & 0xc0 >> 1) & 0xef) >> 5, HEX);
  Serial.print(" G:");
  Serial.print((calStat & 0x30) >> 4, HEX);
  Serial.print(" A:");
  Serial.print((calStat & 0x0c) >> 2, HEX);
  Serial.print(" M:");
  Serial.println(calStat & 0x03, HEX);

}

void BNOdisplaySensorStatus(void)
{


  /* Display the results in the Serial Monitor */

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(BNOreadRegister(SYS_STAT_REG), HEX);
  Serial.print("Self Test:     0x");
  Serial.println(BNOreadRegister(SELFTEST_RESULT_REG), HEX);
  Serial.print("System Error:  0x");
  Serial.println(BNOreadRegister(SYS_ERR_REG), HEX);
  Serial.println("");
  delay(500);
}
