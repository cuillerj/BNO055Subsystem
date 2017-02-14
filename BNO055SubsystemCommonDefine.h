//#define LocalCtrlRegNumber 6
/*
Sensor subsystem that manage gyroscope and magneto sensors
	             that comunicate with the robot
The subsystem is a SI2C master station
The robot and sensors are slave stations
There are 2 ways for comminucation between subsystem and robot
			polling way: the subsystem frenquently give the word to the robot
			interrupt way
*/
/*
BNO055 sensor parameters
*/
#define BNO055_Address 0x29     // default value  gyro
#define BNO055_Address_Reg 0    // stored in this register
#define BNO055Interrupt 			//  the gyroscope will set DRDY (data ready) (INT2)  interrupt Therefore the subsystem will read available data
#define BNO055PollingCycle 5  		// polling cycle in ms 
//#define selectedRange 0        // default gyroscope selected range value (0 1 or 2) meaning {245, 500, 2000}
//#define selectedRange_Reg 1      // stored in this register
#define RobotRotationClockWise -1  // 1  when robot rotateclockwise and -1 when ribot rotateanticlockwise
#define BNOPermanentShiftHeading 0  // in positive degres contains difference between sensor X orientztion and subsystem heading
#define IMUMode 0x08
#define compassMode 0x09
#define NDOFMode 0x0c
/*
Sensors subsystem internal registers
define subsytem register that contain on byte data
*/
#define robotAddress_Reg 2         // data stored in this register is robot I2C address
#define BNO055cycleDuration_Reg 3   // exclusive with L3GInterrupt2 - only use if DRDY interrupt not available
#define robotPollingTimer_Reg1 4     	// stored in this register
#define robotPollingTimer_Reg2 5     	// stored in this register
#define relativeHeading_Reg1 6     // data stored in this register is sign of gyroscope heading
#define relativeHeading_Reg2 7     //data stored in this register is high byte of gyroscope heading
#define relativeHeading_Reg3 8     // data stored in this register is low byte of gyroscope heading
//#define magnetoInstalled true
#define MagnetoCycleDuration 5000         // default magneto polling timer value ms 
#define absoluteHeading_Reg1 9   // data stored in this register is high byte of magneto heading
#define absoluteHeading_Reg2 10  // data stored in this register is low byte of magneto heading
#define absoluteHeading_Reg3 11      // data stored in this register is
#define savedNorthOrientationBefore_Reg1 12
#define savedNorthOrientationBefore_Reg2 13
#define savedNorthOrientationBefore_Reg3 14
#define savedNorthOrientationAfter_Reg1 15
#define savedNorthOrientationAfter_Reg2 16
#define savedNorthOrientationAfter_Reg3 17
#define permanentNOShift_Reg1 18
#define permanentNOShift_Reg2 19

#define compasHeading_Reg1 20  // data stored in this register is high byte of magneto heading
#define compasHeading_Reg2 21  // data stored in this register is low byte of magneto heading

#define BNO055Mode_Reg 22                // contains a copy of the current BNO running mode
#define BNO055CalStatus_Reg 23                // contains a copy of BNO calibration status
#define BNO055SysStatus_Reg 24               // contains a copy of BNO status
#define BNO055TestStatus_Reg 25              // contains a copy of BNO status
#define BNO055SysError_Reg 26               // contains a copy of BNO status

/*
dynamique localisation
*/
#define requestCompute_Reg 27
#define leftDistance_Reg 28
#define rightDistance_Reg 29
#define initPosX_Reg1 30
#define initPosX_Reg2 31
#define initPosY_Reg1 32
#define initPosY_Reg2 33
#define deltaLeftPosX_reg1 34
#define deltaLeftPosX_reg2 35
#define deltaLeftPosY_reg1 36
#define deltaLeftPosY_reg2 37
#define deltaRightPosX_reg1 38
#define deltaRightPosX_reg2 39
#define deltaRightPosY_reg1 40
#define deltaRightPosY_reg2 41
#define initHeading_reg1 42
#define initHeading_reg2 43
#define locationHeading_reg1 44
#define locationHeading_reg2 45

/*
*/


/*
parameters used for I2C communication
*/
#define robotI2CAddress 8          		// define the robot I2C address
#define maxRegsNumberUpdate 3            // maximum number of registers that can be set at a time
#define maxRegsNumberRead 6              // maixum number of registers that can be read a time
#define robotPollingTimer 500       	// default robot polling timer value only used if InputRobotRequestPin not defined (soft polling)
#define pollResponseLenght 10       	// define frame lenght for polling request and response
#define minimumDurationBeetwenPolling 10 // in ms used by robot when hard polling is used to avoid to maintain RobotOutputRobotRequestPin high to long

/*
define request (byte) from robot to sensor subsystem
*/
#define idleRequest 0x00               // nothing to do
#define setRegisterRequest 0x01        // robot request to set registers
#define readRegisterRequest 0x02       // robot request to read registers
#define startMonitorGyro 0x04
#define stopMonitorGyro 0x05
#define startInitMonitorGyro 0x06
#define stopInitMonitorGyro 0x07
#define calibrateGyro 0x08
#define startMonitorMagneto 0x09
#define stopMonitorMagneto 0x0a
#define setBNO055Mode 0x0b
#define printGyroRegisters 0x0c
#define setMoveRegisters 0x0d  
#define initLocation 0x0e  
//#define setGyroODR 0x0d

/*
define response byte from sensor subsytem to robot
*/
#define readRegisterResponse 0x03
/*
used by robot to construct request and analyse response
*/
uint8_t relativeHeadingResponse[3]={relativeHeading_Reg1,relativeHeading_Reg2,relativeHeading_Reg3};
uint8_t absoluteHeadingResponse[3]={absoluteHeading_Reg1,absoluteHeading_Reg2,absoluteHeading_Reg3};
uint8_t compassResponse[2]={compasHeading_Reg1,compasHeading_Reg2};
uint8_t beforeNOResponse[3]={savedNorthOrientationBefore_Reg1,savedNorthOrientationBefore_Reg2,savedNorthOrientationBefore_Reg3};
uint8_t afterNOResponse[3]={savedNorthOrientationAfter_Reg1,savedNorthOrientationAfter_Reg2,savedNorthOrientationAfter_Reg3};
uint8_t beforeAfterNOResponse[6]={savedNorthOrientationBefore_Reg1,savedNorthOrientationBefore_Reg2,savedNorthOrientationBefore_Reg3,savedNorthOrientationAfter_Reg1,savedNorthOrientationAfter_Reg2,savedNorthOrientationAfter_Reg3};
uint8_t calibrateGyroResponse[1]={relativeHeading_Reg1};
uint8_t BNO055StatusResponse[4]={BNO055Mode_Reg,BNO055CalStatus_Reg,BNO055SysStatus_Reg,BNO055SysError_Reg};
uint8_t BNO055LeftLocationResponse[4]={deltaLeftPosX_reg1,deltaLeftPosX_reg2,deltaLeftPosY_reg1,deltaLeftPosY_reg2};
uint8_t BNO055RightLocationResponse[4]={deltaRightPosX_reg1,deltaRightPosX_reg2,deltaRightPosY_reg1,deltaRightPosY_reg2};
uint8_t BNO055LocationHeading[2]={locationHeading_reg1,locationHeading_reg2};
/*
Sensor subsytem parameters
*/
#define nbReadForBiasComputing 1000         // used to calibrate gyroscope
#define BNO055PinInterrupt 2                   // sensor subsystem PIN connected to L3G DRDY  
#define BNO055InterruptNumber 0				// interrupt number corresponding with L3GInterruptNumber (same as digitalPinToInterrupt(L3GPinInterrupt))
#define SensorOutputReadyPin 11              // Sensor subsytem Pin that is set high when Sensor subsytem is ready to work
#define SensorInputRobotRequestPin 12        // sensor subsytem pin that is high when robot is asking for polling (if defined means hard polling is used if not soft polling is used)
#define GyroBiasMicrosec 193                 // bias in micro second taken into account when computing the rotation - max 225
#define BNO055RegistersCopySubsystemMapping 100   // define the offset in the subsystem registers where storing a copy of BNO055 registers
#define BNO055RegistersCopySubsystemFirst 0x39  // define the first BNO055 register to be storesd on the subsystem
#define BNO055RegistersCopySubsystemNumber 6    // define the number of BNO055 register to be storesd on the subsystem
/* 
robot parameters
*/
#define RobotOutputRobotRequestPin 40        // robot pin connected to SensorInputRobotRequestPin used by the robot to request for polling
#define RobotInputReadyPin 41                 // robot pin connected to SensorOutputReadyPin used by the robot to check that the substem is ready
//#define MagnetoPowerPin 10  

/*
Sensor subsytem status byte description
*/
#define monitGyroStatusBit 0                 // bit 0 gyroscope monitoring running
#define monitMagnetoStatusBit 1               // bit 1 magneto monitoring running
#define I2CConnectionBit 2

/*
Used BNO modes
*/
#define   MODE_IMUPLUS                                 0X08
#define   MODE_COMPASS                                 0X09
#define   MODE_NDOF                                    0X0C