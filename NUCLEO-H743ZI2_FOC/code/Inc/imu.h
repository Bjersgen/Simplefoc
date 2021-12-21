#include "main.h"

/*Defalt Settings
LED Status----------Blinking
Angle Unit----------deg
Angle Output Range-- +-180 deg
Baudrate------------115200 bps
Data Output Rate----100 Hz
*/

/*IMUÖ¸Áî´úÂë*/
//Acknowledged / Not-acknowledged Identifiers
#define REPLY_ACK 0x00
#define REPLY_NACK 0x01

//Sensor Info
#define GET_FIRMWARE_VERSION 0x04
#define GET_HARDWARE_VERSION 0x05
#define GET_SERIAL_NUMBER 0x06

//Configuration and Status Commands
#define GET_CONFIG 0x07
#define GET_STATUS 0x08

//Mode Switching Commands
#define GOTO_COMMAND_MODE 0x09
#define GOTO_STREAM_MODE 0x0A

//Data Transmission Commands
#define GET_SENSOR_DATA 0x0B
#define GET_STREAM_FREQ 0x12
#define SET_STREAM_FREQ 0x13
#define GET_UART_BAUDRATE 0x14
#define SET_UART_BAUDRATE 0x15

//Sensor reset
#define RESET_BIAS 0x17
#define RESET_HEADING 0x18
#define RESET_SENSOR 0x19

//Set Angle Output Range
#define SET_360_OUTPUT 0x0E
#define SET_180_OUTPUT 0x0F

//LED Control Commands
#define ENABLE_LED 0x1A
#define DISABLE_LED 0x1B


typedef struct _sensorData
{
// float32_t gAngle;
uint16_t gAngle;  //Heading angle (deg) *100
int16_t gRate;   //Angular speed (deg/s) *50
int16_t accX;    //Calibrated acceleration data (g) *1000
int16_t accY;
int16_t accZ;
} sensorData;

uint32_t IMU_Parse_Stream(uint8_t len);

uint32_t IMU_Init(void);

void IMU_Set_Baudrate(void);
void IMU_Set_Angle_Range(void);
void IMU_Go_To_Command_Mode(void);
void IMU_Go_To_Stream_Mode(void);
void IMU_Heading_Reset(void);

uint8_t Calculate_LRC(uint8_t *ptr, uint8_t len);

extern sensorData IMUData;
