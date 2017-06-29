/*
   I2C communication as master
   one connection with BNO055
   one connection with robot (robot can be polled with timer or can request token with InputRobotRequestPin
*/

/*
   Fully calibrated!
  --------------------------------
  Calibration Results:
  Accelerometer: 65508 65523 10
  Gyro: 65533 65534 1
  Mag: 239 65240 65265
  Accel Radius: 1000
  Mag Radius: 457

*/
/* PINs configuration define the starting mode
    (PIN 1,PIN 2)=> (0,0) = NDOF / (1,1) = IMU / (1,0) = Compas / / (0,1) = Config
*/
//#define debugOn
//#define debugL2On
//#define debugL3On
//#define debugLEDOn
#include <Wire.h>

#include <BNO055SubsystemCommonDefine.h>
#include <BNO055Definitions.h>
#include <EEPROM.h>
//-- accelerometer and magnetometer

uint8_t sysStatusFlag = 0x00;
uint8_t sysCalFlag = 0x00;
uint8_t magCalFlag = 0x00;
unsigned long refAccX;
unsigned long UpdateBNOStatus_time;
unsigned long UpdateLEDStatus_time;
float X;
float Y = 0;
int NOBeforeRotation = 0; // keep NO before rotation
int NOAfterRotation = 0;  // keep NO after rotation
int NOBeforeMoving = 0; // keep NO before moving straight
int NOAfterMoving = 0; // keep NO after moving straight
// compass calibration to be made

volatile boolean dataToRead = false;
uint8_t LOCAL_CTRL_REG[150];        // to store parameters

int x;
int y;
int z;
uint8_t statusBNO055;
float bias = 0;
unsigned int count = 0;
int translatedHeading = 0;
int relativeHeading = 0;
int startHeading = 0;
int locationHeading = 0;
unsigned long avg = 0;
unsigned long savTime = 0;
uint8_t inputData[256];
uint8_t receivedCount = 0x00;
boolean flagOnRequest = false;
uint8_t dataIn[33];
uint8_t pollResponseExpectedLenght = 10;
boolean req = false;
unsigned long prevPollTimer;
unsigned long prevBNO055Timer;
unsigned long prevSentData;
unsigned long updateNOTimer;
boolean monitGyro = true;
boolean monitMagneto = true;
boolean statRobotRequest = 0;
uint8_t currentStatus = 0x00;
unsigned long startInterruptTime;
unsigned long interruptCount;
//Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_Address);
unsigned long countLoop = 0;
uint8_t calibData[22];
boolean pendingInitRelativeHeading = true;
/*
   relative position computation
*/
//float posLeftX = 0;
//float posLeftY = 0;
//float posRightX = 0;
//float posRightY = 0;
//int deltaLeftPosX = 0;
//int deltaLeftPosY = 0;
//int deltaRightPosX = 0;
//int deltaRightPosY = 0;
float InitLocationHeading = 0;
float fDeltaLeftPosX = 0;
float fDeltaLeftPosY = 0;
float fDeltaRightPosX = 0;
float fDeltaRightPosY = 0;
int shiftHeadingLocation = 0;
float locationHeadingRad  = 0.;
typedef struct {
  double x;
  double y;
  double z;
} vector;
typedef enum
{
  vector_ACCELEROMETER = 0x08,
  vector_MAGNETOMETER  = 0x0e,
  vector_GYROSCOPE     = 0x14,
  vector_EULER         = 0x1a,
  vector_LINEARACCEL   = 0x28,
  vector_GRAVITY       = 0x2e
} vector_type_t;

//Adafruit_BNO055::adafruit_bno055_reg_t regDef;
void setup() {
  // Wire.begin();
  Serial.begin(38400);
  pinMode(systemLED, OUTPUT);
  pinMode(sysCalLED, OUTPUT);
  pinMode(magCalLED, OUTPUT);
  pinMode(pin1OpMode, INPUT);
  pinMode(pin2OpMode, INPUT);
  digitalWrite(systemLED, 1);
  InitSubsystemParameters(true, 0, 0);
  //  LOCAL_CTRL_REG[selectedRange_Reg] = 0x01;         // default selected range
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  /* Initialise the sensor */

  if (!BNOInit())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
    LOCAL_CTRL_REG[BNO055Mode_Reg] = 0x00;
  }
  else
  {
    LOCAL_CTRL_REG[BNO055Mode_Reg] = BNOreadRegister(OPR_MODE_REG);
  }

  Serial.print("op mode:");
  Serial.println(LOCAL_CTRL_REG[BNO055Mode_Reg], HEX);
  delay(1000);
  EEPROM.get(eeAddress, bnoID);


  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
     This isn't foolproof, but it's better than nothing.
  */

  if (bnoID != sensor_id)
  {
    Serial.print("\nNo Calibration Data in EEPROM for this sensor ID:");
    Serial.println(bnoID);
    delay(500);
  }
  else
  {
    Serial.print("\nFound Calibration in EEPROM for this sensor ID:");
    Serial.println(bnoID);
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibData);
    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    BNORestoreCalibration(&calibData[0]);
    Serial.println("\n\nCalibration data loaded into BNO055");
    delay(200);
    BNOPrintCalibration();
    foundCalib = true;
  }
  UpdateBNOStatus();
  UpdateLEDStatus();
  delay(1000);
#if defined(debugOn)
  BNOdisplayCalStatus();
  BNOdisplaySensorStatus();
#endif
  if (digitalRead(pin1OpMode) == 1 && digitalRead(pin2OpMode) == 0)
  {
    Serial.println("starting compas mode");
    SetBNO055Mode(OP_MODE_COMPASS);
  }
  if (digitalRead(pin1OpMode) == 0 && digitalRead(pin2OpMode) == 0)
  {
    Serial.println("startingt NDOF mode");
    SetBNO055Mode(OP_MODE_NDOF);
  }
  if (digitalRead(pin1OpMode) == 1 && digitalRead(pin2OpMode) == 1)
  {
    Serial.println("starting IMU mode");
    SetBNO055Mode(OP_MODE_IMUPLUS);
  }
  if (digitalRead(pin1OpMode) == 0 && digitalRead(pin2OpMode) == 1)
  {
    Serial.println("starting config mode");
    SetBNO055Mode(OP_MODE_CONFIG);
  }
  //  SetBNO055Mode(OP_MODE_NDOF);
  LOCAL_CTRL_REG[BNO055Mode_Reg] = BNOreadRegister(OPR_MODE_REG);
  if (foundCalib  && digitalRead(pin1OpMode) == 0 && digitalRead(pin2OpMode) == 0) {
    Serial.println("Move sensor slightly to calibrate magnetometers");
    unsigned int count = 0;
    while (BNOreadRegister(CALIB_STAT_REG) != 0xff && count < 60)
    {
#if defined(debugOn)
      BNOdisplayCalStatus();
      BNOdisplaySensorStatus();
#endif
      UpdateBNOStatus();
      UpdateLEDStatus();
      delay(1000);
      count++;
    }
    Serial.println("end calibration");
  }
  UpdateBNOStatus();
  UpdateLEDStatus();
  delay(500);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Optional: Display current status */
  // displaySensorStatus();
  //  WriteRegister(BNO055_Address, 0x10, 0b01000000);
  delay(20);
  Serial.println("BNO055 is started");
  pinMode(SensorOutputReadyPin, OUTPUT);
#if defined(BNO055PinInterrupt)
  pinMode(BNO055PinInterrupt, INPUT);
  attachInterrupt(BNO055PinInterrupt, Robot_BNO055DataReady, RISING);
#endif
  //    pinMode(MagnetoPowerPin, OUTPUT);

#if defined(SensorInputRobotRequestPin)
  pinMode(SensorInputRobotRequestPin, INPUT);
#endif
  Serial.println("ready");
  delay(1000);
  digitalWrite(SensorOutputReadyPin, HIGH);
  UpdateBNOStatus();
  UpdateLEDStatus();

  //  Serial.println("set IMU mode");
  //  SetBNO055Mode(OP_MODE_IMUPLUS);
}

void loop() {
  delayMicroseconds(1);

  if ((millis() - prevBNO055Timer) >= LOCAL_CTRL_REG[BNO055cycleDuration_Reg] && monitGyro == true ) //LOCAL_CTRL_REG[cycleDuration_Reg]
  {
    dataToRead = false;
    if (LOCAL_CTRL_REG[BNO055Mode_Reg] == OP_MODE_IMUPLUS)
    {
      double x, y, z;
      vector euler = {x, y, z};
      euler = GetVector(vector_EULER);
      //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      if (pendingInitRelativeHeading)
      {
        Serial.println("initR1");
        startHeading = -euler.x;
        pendingInitRelativeHeading = false;
      }
      relativeHeading = (int)round((euler.x + startHeading)) % 360 ;

      locationHeading = (int)round((euler.x + InitLocationHeading + shiftHeadingLocation)) % 360;
      //     locationHeadingRad = round((locationHeading) * PI / 180);
      uint16_t uLocationHeading = (uint16_t)(round(360 + RobotRotationClockWise * locationHeading) % 360); // adjust rotation sens & store for robot
      LOCAL_CTRL_REG[locationHeading_reg1] = ((uint8_t)((uLocationHeading & 0x7fff) >> 8) | ((uLocationHeading & 0x8000) >> 8));
      LOCAL_CTRL_REG[locationHeading_reg2] = (((uint8_t)uLocationHeading ) );
#if defined(debugOn)
      countLoop++;
      if (countLoop % 5000 == 0)
      {
        Serial.print("IMU r:");
        Serial.print(relativeHeading);
        Serial.print(" l:");
        Serial.println(locationHeading);
      }
#endif
      if (relativeHeading >= 0)
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[relativeHeading_Reg2] = uint8_t (abs(relativeHeading / 256));
      LOCAL_CTRL_REG[relativeHeading_Reg3] = uint8_t (abs(relativeHeading));
      //   prevtranslatedHeading=translatedHeading;
    }
    if (LOCAL_CTRL_REG[BNO055Mode_Reg] == OP_MODE_COMPASS)
    {
      double x, y, z;
      vector euler = {x, y, z};
      euler = GetVector(vector_EULER);
      //     imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      int compassHeading = euler.x;
      compassHeading = round((compassHeading + LOCAL_CTRL_REG[permanentNOShift_Reg1] * 256 + LOCAL_CTRL_REG[permanentNOShift_Reg2])) % 360;
      LOCAL_CTRL_REG[compasHeading_Reg1] = uint8_t (compassHeading / 256);
      LOCAL_CTRL_REG[compasHeading_Reg2] = uint8_t (compassHeading);
#if defined(debugOn)
      countLoop++;
      if (countLoop % 5000 == 0)
      {
        Serial.print("compass:");
        Serial.println(compassHeading);
      }
#endif
    }
    if (LOCAL_CTRL_REG[BNO055Mode_Reg] == OP_MODE_NDOF)
    {
      double x, y, z;
      vector euler = {x, y, z};
      euler = GetVector(vector_EULER);
      //      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      locationHeading = (int)round((euler.x + InitLocationHeading + shiftHeadingLocation)) % 360;
      //      locationHeadingRad = round((locationHeading) * PI / 180);
      uint16_t uLocationHeading = (uint16_t)(round(360 + RobotRotationClockWise * locationHeading) % 360);  // adjust rotation sens & store for robot
      LOCAL_CTRL_REG[locationHeading_reg1] = ((uint8_t)((uLocationHeading & 0x7fff) >> 8) | ((uLocationHeading & 0x8000) >> 8));
      LOCAL_CTRL_REG[locationHeading_reg2] = (((uint8_t)uLocationHeading ) );
      int absoluteHeading = euler.x;
      if (pendingInitRelativeHeading)
      {
        Serial.println("initR2");
        startHeading = -euler.x;
        pendingInitRelativeHeading = false;
      }
      relativeHeading = round(absoluteHeading + startHeading + BNOPermanentShiftHeading);
      absoluteHeading = round((absoluteHeading + LOCAL_CTRL_REG[permanentNOShift_Reg1] * 256 + LOCAL_CTRL_REG[permanentNOShift_Reg2])) % 360;
      if (absoluteHeading >= 0)
      {
        LOCAL_CTRL_REG[absoluteHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[absoluteHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[absoluteHeading_Reg2] = uint8_t (abs(absoluteHeading / 256));
      LOCAL_CTRL_REG[absoluteHeading_Reg3] = uint8_t (abs(absoluteHeading));
      if (relativeHeading >= 0)
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
      }
      else
      {
        LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x01;
      }
      LOCAL_CTRL_REG[relativeHeading_Reg2] = uint8_t (abs(relativeHeading / 256));
      LOCAL_CTRL_REG[relativeHeading_Reg3] = uint8_t (abs(relativeHeading));
      //   prevtranslatedHeading=translatedHeading;
#if defined(debugOn)
      countLoop++;
      if (countLoop % 5000 == 0)
      {
        Serial.print("NDOF absolute heading:");
        Serial.print(absoluteHeading);
        Serial.print(" relative heading:");
        Serial.println(relativeHeading);
      }
#endif
    }
#if defined(debugOn)
    if (countLoop % 20000 == 0)
    {
      BNOdisplayCalStatus();
      //            getCalibrationData();
      BNOdisplaySensorStatus();
#if defined(debugOn)
      Serial.print("lX:");
      Serial.print(fDeltaLeftPosX);
      Serial.print(" lY:");
      Serial.print(fDeltaLeftPosY);
      Serial.print(" rX:");
      Serial.print(fDeltaRightPosX);
      Serial.print(" rY:");
      Serial.print(fDeltaRightPosY);
      Serial.print(" H:");
      Serial.println(locationHeading);
#endif
    }
#endif
    //   imu::Vector<3> absoluteOrientation = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    //Serial.println(absoluteOrientation.y());
  }

  if ( (digitalRead(SensorInputRobotRequestPin) == HIGH) )
    /*
       The robot ask for the word
    */
  {
    int dataLen = Robot_PollSlave(LOCAL_CTRL_REG[robotAddress_Reg], currentStatus);  // request data from robot
    uint8_t receiveAddress = dataIn[0];
    if (receiveAddress == LOCAL_CTRL_REG[robotAddress_Reg])                     // check data is comming from robot
    {
      uint8_t cmd = dataIn[1];                                                  // this byte is a command switch
      uint8_t numberRegs = dataIn[2];
      uint8_t parameter = dataIn[2];
      switch (cmd)
      {
        case idleRequest:
          break;
        case setRegisterRequest:                                    // request is set registers values
          switch (numberRegs)                                       // number of consecutive registers to set
          {
            case 0xff:
              InitSubsystemParameters(true, 0, 0);                                        // reset default value
              break;
            default:
              if (numberRegs <= maxRegsNumberUpdate)
              {
                for (int i = 0; i < min(numberRegs, maxRegsNumberUpdate); i++)
                {
                  InitSubsystemParameters(false, dataIn[2 * i + 3], dataIn[2 * i + 4]);
                }
              }
              break;
          }
          break;
        case setMoveRegisters:                                    // request is set registers values
#if defined(debugOn)
          Serial.print("SetMoveRegisters:");
#endif
          if (numberRegs <= maxRegsNumberUpdate)
          {
            for (int i = 0; i < min(numberRegs, maxRegsNumberUpdate); i++)
            {
              LOCAL_CTRL_REG[dataIn[2 * i + 3]] = dataIn[2 * i + 4];
#if defined(debugL2On)
              Serial.print(" reg:");
              Serial.print(dataIn[2 * i + 3]);
              Serial.print(" value:");
              Serial.print(dataIn[2 * i + 4]);
              Serial.print(" read:");
              Serial.print(LOCAL_CTRL_REG[dataIn[2 * i + 3]]);
#endif
            }
#if defined(debugOn)
            Serial.println();

#endif
          }
          break;
        case readRegisterRequest:
          {
            if (numberRegs <= maxRegsNumberRead)
            {
              Robot_SendRegistersValue(receiveAddress, numberRegs, dataIn);
            }
            break;
          }

        case startInitMonitorGyro:
          {
            double x, y, z;
            vector euler = {x, y, z};
            euler = GetVector(vector_EULER);
            //           imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            startHeading = -euler.x;
            //           SetBNO055Mode(IMUMode);
            monitGyro = true;
            LOCAL_CTRL_REG[relativeHeading_Reg1] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg2] = 0x00;
            LOCAL_CTRL_REG[relativeHeading_Reg3] = 0x00;
            Serial.print("StartInitGyro:");
            Serial.println(startHeading);
            bitWrite(currentStatus, monitGyroStatusBit, 1);
            interruptCount = 0;
            break;
          }

        case startMonitorMagneto:
          {
            //           digitalWrite(MagnetoPowerPin, HIGH);
            //           SetBNO055Mode(compassMode);
            //          BNO055CalibrationStatus();
            monitMagneto = true;
            Serial.println("StartCompas");
            bitWrite(currentStatus, monitMagnetoStatusBit, 1);
            break;
          }
        case setBNO055Mode:
          {
            //           digitalWrite(MagnetoPowerPin, HIGH);
            SetBNO055Mode(dataIn[2]);
            break;
          }
        case initLocation:
          {
            InitLocation();
            break;
          }
        default:
          {

          }
      }
    }
    req = !req;
  }

  if (millis() - UpdateBNOStatus_time >=  UpdateBNOStatusDelay)
  {
    UpdateBNOStatus();
    UpdateLEDStatus();
  }
  if (LOCAL_CTRL_REG[requestCompute_Reg] == 0x01)
  {
    LOCAL_CTRL_REG[requestCompute_Reg] = 0x00;
    //  uint16_t deltaLeftPosX = (uint16_t)LOCAL_CTRL_REG[deltaLeftPosX_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaLeftPosX_reg2];
    //  uint16_t deltaLeftPosY = (uint16_t)LOCAL_CTRL_REG[deltaLeftPosY_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaLeftPosY_reg2];
    //   uint16_t deltaRightPosX = (uint16_t)LOCAL_CTRL_REG[deltaRightPosX_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaRightPosX_reg2];
    //  uint16_t deltaRightPosY = (uint16_t)LOCAL_CTRL_REG[deltaRightPosY_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaRightPosY_reg2];
    locationHeadingRad = ((int)(360 - RobotRotationClockWise * locationHeading) % 360) * PI / 180;   // convert & adjust rotation sens
    fDeltaLeftPosX = fDeltaLeftPosX + ((float)(LOCAL_CTRL_REG[leftDistance_Reg])) * cos(RobotRotationClockWise * locationHeadingRad );
    fDeltaLeftPosY = fDeltaLeftPosY + ((float)(LOCAL_CTRL_REG[leftDistance_Reg])) * sin(RobotRotationClockWise * locationHeadingRad );
    fDeltaRightPosX = fDeltaRightPosX + ((float)LOCAL_CTRL_REG[rightDistance_Reg]) * cos(RobotRotationClockWise * locationHeadingRad );
    fDeltaRightPosY = fDeltaRightPosY + ((float)LOCAL_CTRL_REG[rightDistance_Reg]) * sin(RobotRotationClockWise * locationHeadingRad );
#if defined(debugOn)
    //   Serial.println(fDeltaLeftPosX);
    //   Serial.println(fDeltaLeftPosY);
    //   Serial.println(fDeltaRightPosX);
    //   Serial.println(fDeltaRightPosY);
#endif
    uint16_t deltaLeftPosX = (uint16_t)(round(fDeltaLeftPosX));
    uint16_t deltaRightPosX = (uint16_t)(round(fDeltaRightPosX));
    uint16_t deltaLeftPosY = (uint16_t)(round(fDeltaLeftPosY));
    uint16_t deltaRightPosY = (uint16_t)(round(fDeltaRightPosY));
    // uint16_t uLocationHeading = (uint16_t)(round(locationHeading));
    //  Serial.println(deltaLeftPosX, BIN);
    //   Serial.println(((uLocationHeading & 0x7fff) >> 8) | ((uLocationHeading & 8000) >> 8), BIN);
    LOCAL_CTRL_REG[deltaLeftPosX_reg1] = ((uint8_t)((deltaLeftPosX & 0x7fff) >> 8) | ((deltaLeftPosX & 0x8000) >> 8));
    LOCAL_CTRL_REG[deltaLeftPosX_reg2] = ((uint8_t)deltaLeftPosX )  ;
    LOCAL_CTRL_REG[deltaLeftPosY_reg1] = ((uint8_t)((deltaLeftPosY & 0x7fff) >> 8) | ((deltaLeftPosY & 0x8000) >> 8));
    LOCAL_CTRL_REG[deltaLeftPosY_reg2] = (((uint8_t)deltaLeftPosY )) ;
    LOCAL_CTRL_REG[deltaRightPosX_reg1] = ((uint8_t)((deltaRightPosX & 0x7fff) >> 8) | ((deltaRightPosX & 0x8000) >> 8));
    LOCAL_CTRL_REG[deltaRightPosX_reg2] = (((uint8_t)deltaRightPosX ) ) ;
    LOCAL_CTRL_REG[deltaRightPosY_reg1] = ((uint8_t)((deltaRightPosY & 0x7fff) >> 8) | ((deltaRightPosY & 0x8000) >> 8));
    LOCAL_CTRL_REG[deltaRightPosY_reg2] = (((uint8_t)deltaRightPosY ) ) ;
    //  LOCAL_CTRL_REG[locationHeading_reg1] = ((uint8_t)((uLocationHeading & 0x7fff) >> 8) | ((uLocationHeading & 0x8000) >> 8));
    //  LOCAL_CTRL_REG[locationHeading_reg2] = (((uint8_t)uLocationHeading ) ) ;
#if defined(debugOn)
    Serial.print("holes left:");
    Serial.print( LOCAL_CTRL_REG[leftDistance_Reg]);
    Serial.print(" right:");
    Serial.println( LOCAL_CTRL_REG[rightDistance_Reg]);
    Serial.print("heading:");
    Serial.print(locationHeading);
    Serial.print(" left delta x:");
    int xx = (uint16_t)LOCAL_CTRL_REG[deltaLeftPosX_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaLeftPosX_reg2];
    Serial.print(xx);
    Serial.print(" y:");
    int yy = (uint16_t)LOCAL_CTRL_REG[deltaLeftPosY_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaLeftPosY_reg2];
    Serial.print(yy);
    Serial.print(" right delta x:");
    xx = (uint16_t)LOCAL_CTRL_REG[deltaRightPosX_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaRightPosX_reg2];
    Serial.print(xx);
    Serial.print(" y:");
    yy = (uint16_t)LOCAL_CTRL_REG[deltaRightPosY_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[deltaRightPosY_reg2];
    Serial.println(yy);
#endif
  }
}

void Robot_BNO055DataReady()
{
  //  detachInterrupt(BNO055InterruptNumber);
  Serial.print("i..");
  dataToRead = true;
  interruptCount++;
  // Serial.print(".");
}
void Robot_SendRegistersValue(uint8_t deviceAddress,  uint8_t numberRegs, uint8_t dataIn[33])
{
  Wire.beginTransmission(deviceAddress); // transmit to device #8
  Wire.write(deviceAddress);
  Wire.write(readRegisterResponse);
  Wire.write(numberRegs);
  for (int i = 0; i < numberRegs; i++)
  {
    Wire.write(dataIn[i + 3]);
    Wire.write(LOCAL_CTRL_REG[dataIn[i + 3]]);            // sends one byte
  }
  Wire.endTransmission();    // stop transmitting
}
int Robot_PollSlave(int deviceAddress, byte parameter) {

  int idx = 0;
  prevPollTimer = millis();
  //
  Wire.beginTransmission(deviceAddress);
  Wire.write(deviceAddress);
  Wire.write(idleRequest);
  Wire.write(parameter);
  Wire.write(pollResponseExpectedLenght);                   // Command Data = dummy zeroes
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, pollResponseExpectedLenght); // read a byte
  while (Wire.available()) {
    dataIn[idx] = Wire.read();
    idx++;
  }
#if defined(debugL3On)
  Serial.print("request from robot:" );
  Serial.println(deviceAddress, HEX);
  //  Serial.println(float(interruptCount * 1000 / (millis() - startInterruptTime)));
#endif
  return idx;
}
/*
  int UpdateNorthOrientation()
  {
  int NO = 0;
  #if defined(magnetoInstalled)
  compass.read();
  float northOrientation = compass.heading();
  #else
  return 0;
  #endif
  #if defined(debugMagnetoOn)
  Serial.print(" magneto orientation: ");
  Serial.println(northOrientation);

  NO = int(northOrientation);
  LOCAL_CTRL_REG[headingNorthOrientation_Reg1] = uint8_t(NO / 256);
  LOCAL_CTRL_REG[headingNorthOrientation_Reg2] = uint8_t(NO);
  #endif
  updateNOTimer = millis();
  return NO;

  }
*/
void InitSubsystemParameters(boolean allReg, uint8_t regNumber, uint8_t regValue)
{
  if (allReg || regNumber == BNO055cycleDuration_Reg)
  {
    Serial.print("BNO055 polling cycle:");
    if (allReg)
    {
      LOCAL_CTRL_REG[BNO055cycleDuration_Reg] = BNO055PollingCycle;         // default selected range
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = uint8_t(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[BNO055cycleDuration_Reg]);
  }
  //
  if (allReg || regNumber == robotAddress_Reg)
  {
    Serial.print("Robot addrsss:");
    if (allReg)
    {
      LOCAL_CTRL_REG[robotAddress_Reg] = robotI2CAddress;         // robot I2 address
    }
    else
    {
      LOCAL_CTRL_REG[regNumber] = int(regValue);
    }
    Serial.println(LOCAL_CTRL_REG[robotAddress_Reg], HEX);
  }

  Serial.print("Permanent NO Shift:");
  LOCAL_CTRL_REG[permanentNOShift_Reg1] = permanentNOShift / 256;
  LOCAL_CTRL_REG[permanentNOShift_Reg2] = permanentNOShift;
  Serial.println(LOCAL_CTRL_REG[permanentNOShift_Reg1] * 256 + LOCAL_CTRL_REG[permanentNOShift_Reg2]);
}


void receiveEvent(int howMany) {
  //  Serial.println(howMany);
  while (Wire.available()) { // loop through all but the last
    inputData[receivedCount] = Wire.read(); // receive byte as a character
    //   Serial.print(inputData[receivedCount], HEX);        // print the character
    //   Serial.print("-");
    receivedCount++;
  }
  //  Serial.println();
}

void Robot_CalibrateGyro(uint8_t deviceAddress)
{

  Wire.beginTransmission(deviceAddress); // transmit to device #8
  Wire.write(deviceAddress);
  Wire.write(calibrateGyro);
  Wire.endTransmission();    // stop transmitting
}

void InitLocation()
{
  if (LOCAL_CTRL_REG[BNO055Mode_Reg] != OP_MODE_IMUPLUS)
  {
    //    SetBNO055Mode(OP_MODE_IMUPLUS);
  }
  double x, y, z;
  vector euler = {x, y, z};
  euler = GetVector(vector_EULER);
  InitLocationHeading = ((uint16_t)LOCAL_CTRL_REG[initHeading_reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[initHeading_reg2]);
  InitLocationHeading = (int)(360 + RobotRotationClockWise * InitLocationHeading) % 360;  // adjust rotation sens
  shiftHeadingLocation = -(euler.x ) ;
  fDeltaLeftPosX = (int)((uint16_t)LOCAL_CTRL_REG[initPosX_Reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[initPosX_Reg2]);
  fDeltaLeftPosY = (int)((uint16_t)LOCAL_CTRL_REG[initPosY_Reg1] << 8 | (uint16_t) LOCAL_CTRL_REG[initPosY_Reg2]);
  fDeltaRightPosX = fDeltaLeftPosX;
  fDeltaRightPosY = fDeltaLeftPosY;
  LOCAL_CTRL_REG[deltaLeftPosX_reg1] = LOCAL_CTRL_REG[initPosX_Reg1] ;
  LOCAL_CTRL_REG[deltaLeftPosX_reg2] = LOCAL_CTRL_REG[initPosX_Reg2] ;
  LOCAL_CTRL_REG[deltaLeftPosY_reg1] = LOCAL_CTRL_REG[initPosY_Reg1];
  LOCAL_CTRL_REG[deltaLeftPosY_reg2] = LOCAL_CTRL_REG[initPosY_Reg2] ;
  LOCAL_CTRL_REG[deltaRightPosX_reg1] = LOCAL_CTRL_REG[initPosX_Reg1];
  LOCAL_CTRL_REG[deltaRightPosX_reg2] = LOCAL_CTRL_REG[initPosX_Reg2] ;
  LOCAL_CTRL_REG[deltaRightPosY_reg1] = LOCAL_CTRL_REG[initPosY_Reg1];
  LOCAL_CTRL_REG[deltaRightPosY_reg2] = LOCAL_CTRL_REG[initPosY_Reg2];
  LOCAL_CTRL_REG[locationHeading_reg1] = LOCAL_CTRL_REG[initHeading_reg1];
  LOCAL_CTRL_REG[locationHeading_reg2] = LOCAL_CTRL_REG[initHeading_reg2] ;
  pendingInitRelativeHeading = true;
#if defined(debugOn)
  Serial.print("init:");
  Serial.print(shiftHeadingLocation);
  Serial.print(" h:");
  Serial.print( InitLocationHeading);
  Serial.print(" e:");
  Serial.print(euler.x);
  Serial.print(" x:");
  Serial.print(fDeltaLeftPosX);
  Serial.print(" y:");
  Serial.println(fDeltaLeftPosY);

#endif

}
void UpdateBNOStatus()
{
  LOCAL_CTRL_REG[BNO055SysStatus_Reg] = BNOreadRegister(SYS_STAT_REG);
  LOCAL_CTRL_REG[BNO055CalStatus_Reg] = BNOreadRegister(CALIB_STAT_REG);
  LOCAL_CTRL_REG[BNO055TestStatus_Reg] = BNOreadRegister(SELFTEST_RESULT_REG);
  LOCAL_CTRL_REG[BNO055SysError_Reg] = BNOreadRegister(SYS_ERR_REG);
  UpdateBNOStatus_time = millis();
}
void UpdateLEDStatus()
{
  // uint8_t sysStat = LOCAL_CTRL_REG[BNO055SysStatus_Reg];
  LOCAL_CTRL_REG[BNO055Mode_Reg] = BNOreadRegister(OPR_MODE_REG);
  uint8_t maskCal = 0x00;
  magCalFlag++;
  switch (LOCAL_CTRL_REG[BNO055Mode_Reg])
  { // 1 blink compas 2 IMU 3 NDOF
    case MODE_IMUPLUS:
      maskCal = 0x30;
      if (magCalFlag % 50 == 0 || magCalFlag % 52 == 0 )
      {
        digitalWrite(sysCalLED, 1);
      }
      else
      {
        digitalWrite(sysCalLED, 0);
      }
      break;
    case MODE_COMPASS:
      maskCal = 0x03;
      if (magCalFlag % 50 == 0  )
      {
        digitalWrite(sysCalLED, 1);
      }
      else
      {
        digitalWrite(sysCalLED, 0);
      }
      break;
    case MODE_NDOF:
      maskCal = 0xc0;
      if (magCalFlag % 50 == 0 || magCalFlag % 52 == 0 || magCalFlag % 54 == 0 )
      {
        digitalWrite(sysCalLED, 1);
      }
      else
      {
        digitalWrite(sysCalLED, 0);
      }
      maskCal = 0xff;
      break;
    default:
      break;
  }
#if defined(debugLEDOn)
  Serial.print("LED Sys:");
  Serial.print(LOCAL_CTRL_REG[BNO055SysStatus_Reg], HEX);
  Serial.print(" Cal:");
  Serial.println(LOCAL_CTRL_REG[BNO055CalStatus_Reg], HEX);
#endif
  switch (LOCAL_CTRL_REG[BNO055SysStatus_Reg])
  {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
      digitalWrite(systemLED, 1);
      break;
    case 0x05:
      digitalWrite(systemLED, 0);
      break;
    case 0x06:
      digitalWrite(systemLED, !digitalRead(systemLED));
      break;
  }

  switch (LOCAL_CTRL_REG[BNO055CalStatus_Reg] & maskCal)
  {
    case 0x00:
      digitalWrite(magCalLED, 0);
      break;
    case 0x03:
    case 0x30:
    case 0xff:
      digitalWrite(magCalLED, 1);
      break;
    case 0x01:
    case 0x10:
      //    case 0x40:
      if (magCalFlag % 3 == 0)
      {
        digitalWrite(magCalLED, !digitalRead(magCalLED));
      }
      break;
    case 0x02:
    case 0x20:
    //   case 0x80:
    case 0xc0:
      if (magCalFlag % 2 == 0)
      {
        digitalWrite(magCalLED, !digitalRead(magCalLED));
      }
      magCalFlag++;
      break;
    default:
      digitalWrite(magCalLED, !digitalRead(magCalLED));
      break;
  }

  UpdateLEDStatus_time = millis();
}

