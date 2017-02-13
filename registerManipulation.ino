uint8_t readRegister(uint8_t deviceAddress, byte address) {
  uint8_t v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1); // read a byte
  while (!Wire.available()) {
    // waiting
  }
  v = Wire.read();
  return v;
}
void PrintRegisters()
{
  for (uint8_t reg = BNO055RegistersCopySubsystemFirst; reg < BNO055RegistersCopySubsystemFirst + BNO055RegistersCopySubsystemNumber; reg++)
  {
    byte regValue = readRegister(LOCAL_CTRL_REG[BNO055_Address_Reg], reg);
    uint8_t idx = BNO055RegistersCopySubsystemMapping + reg;
    LOCAL_CTRL_REG[idx] = regValue;
    Serial.print("BNO055 register 0x");
    Serial.print(reg, HEX);
    Serial.print(":");
    Serial.println(regValue, BIN);
    delay(50); //wait for the sensor to be ready
  }
}
void WriteRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}
