      /* Page id register definition */
#define PAGE_ID_REG                                      0X07

      /* PAGE0 REGISTER DEFINITION START*/
#define  CHIP_ID_REG                                     0x00
/* Accelerometer Offset registers */
#define  ACCEL_OFFSET_X_LSB_REG                                 0X55
#define  ACCEL_OFFSET_X_MSB_REG                                 0X56
#define  ACCEL_OFFSET_Y_LSB_REG                                 0X57
#define  ACCEL_OFFSET_Y_MSB_REG                                 0X58
#define  ACCEL_OFFSET_Z_LSB_REG                                 0X59
#define  ACCEL_OFFSET_Z_MSB_REG                                 0X5A

/* Magnetometer Offset registers */
#define  MAG_OFFSET_X_LSB_REG                                   0X5B
#define  MAG_OFFSET_X_MSB_REG                                   0X5C
#define  MAG_OFFSET_Y_LSB_REG                                   0X5D
#define  MAG_OFFSET_Y_MSB_REG                                   0X5E
#define  MAG_OFFSET_Z_LSB_REG                                   0X5F
#define  MAG_OFFSET_Z_MSB_REG                                   0X60

/* Gyroscope Offset register s*/
#define  GYRO_OFFSET_X_LSB_REG                                  0X61
#define  GYRO_OFFSET_X_MSB_REG                                  0X62
#define  GYRO_OFFSET_Y_LSB_REG                                  0X63
#define  GYRO_OFFSET_Y_MSB_REG                                  0X64
#define  GYRO_OFFSET_Z_LSB_REG                                  0X65
#define  GYRO_OFFSET_Z_MSB_REG                                  0X66

/* Radius registers */
#define  ACCEL_RADIUS_LSB_REG                                   0X67
#define ACCEL_RADIUS_MSB_REG                                    0X68
#define MAG_RADIUS_LSB_REG                                      0X69
#define MAG_RADIUS_MSB_REG                                      0X6A

#define BNO055_ID												 0xA0
#define  OPR_MODE_REG                                    0X3D
#define  PWR_MODE_REG                                    0X3E

 /* Operation mode settings*/
#define   OP_MODE_CONFIG                                  0X00
#define   OP_MODE_ACCONLY                                 0X01
#define   OP_MODE_MAGONLY                                 0X02
#define	  OP_MODE_GYRONLY                                 0X03
#define   OP_MODE_ACCMAG                                  0X04
#define   OP_MODE_ACCGYRO                                 0X05
#define   OP_MODE_MAGGYRO                                 0X06
#define   OP_MODE_AMG                                     0X07
#define   OP_MODE_IMUPLUS                                 0X08
#define   OP_MODE_COMPASS                                 0X09
#define   OP_MODE_M4G                                     0X0A
#define   OP_MODE_NDOF_FMC_OFF                            0X0B
#define   OP_MODE_NDOF                                    0X0C

#define   SYS_TRIGGER_REG                          0X3F

#define  POWER_MODE_NORMAL                                 0X00
#define  POWER_MODE_LOWPOWER                               0X01
#define  POWER_MODE_SUSPEND                                0X02

     /* BNO Status registers */
#define  CALIB_STAT_REG                      0X35
#define  SELFTEST_RESULT_REG                   0X36
#define  INTR_STAT_REG                        0X37
#define  SYS_CLK_STAT_REG                     0X38
#define  SYS_STAT_REG                         0X39
#define  SYS_ERR_REG                           0X3A

/*
Subsytem charactristics
*/

/* Interrupt registers
page 0 */
#define INT_STA_REG										0x37

/*
page 1*/
#define ACC_NM_SET_REG									0x16
#define ACC_NM_THRE_REG									0x15
#define INT_EN_REG										0x10
#define INT_MSK_REG										0x0f
#define ACC_INT_settings_REG							0x12

/*
Physical characteristics
*/
#define permanentNOShift							    81

/*
Timers
*/
#define UpdateBNOStatusDelay							250

/*
Identification
*/
#define sensor_id 0x01

/*
PIN description
*/

#define systemLED 5
#define sysCalLED 6
#define magCalLED 4
#define pin1OpMode 8
#define pin2OpMode 7