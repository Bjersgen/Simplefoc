#include <stdio.h>
#include "string.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "main.h"
#include "imu.h"

#define IMU_BAUDRATE 115200

// Private Var
uint8_t IMU_Stream[IMU_BUFFER_SIZE];
uint8_t IMU_TX_Buffer[IMU_BUFFER_SIZE];
uint8_t IMU_RX_Command[IMU_BUFFER_SIZE];

const uint8_t IMU_ACK_RESPONSE[] = {0x3a, 0x00, 0x01, 0x00, 0x01, 0x0D, 0x0A};
const uint8_t IMU_NACK_RESPONSE[] = {0x3a, 0x01, 0x01, 0x00, 0x02, 0x0D, 0x0A};

//校验数据并输出到SensorData
uint32_t IMU_Parse_Stream(uint8_t len){
    memcpy(IMU_Stream, IMU_RX_Buffer, IMU_BUFFER_SIZE);
    if(len != 17){ return HAL_ERROR; }
     if (IMU_Stream[0] != 0x3A)  return HAL_ERROR;
    uint8_t function = IMU_Stream[1];
    uint8_t index = IMU_Stream[2];
    uint8_t length = IMU_Stream[3];
    uint8_t checksum = 0;
   // memcpy(c2i.c, dataBuffer + 4, 10);
    for (int i = 1; i < 14; ++i)
         checksum += IMU_Stream[i];
    if(checksum != IMU_Stream[14])  return HAL_ERROR;
    IMUData.gAngle = (IMU_Stream[4]<<8) + IMU_Stream[5];
    IMUData.gRate  = (IMU_Stream[6]<<8) + IMU_Stream[7];
    IMUData.accX   = (IMU_Stream[8]<<8) + IMU_Stream[9];
    IMUData.accY   = (IMU_Stream[10]<<8) + IMU_Stream[11];
    IMUData.accZ   = (IMU_Stream[12]<<8) + IMU_Stream[13];
    return HAL_OK;
}
/*
uint32_t IMU_Parse_Response(){
    if(strstr(IMU_RX_Buffer, IMU_ACK_RESPONSE)==0) return 0; //ACK
    else if(strstr(IMU_RX_Buffer, IMU_NACK_RESPONSE)==0) return 1; //NACK
    return 2; //EXCEPTION
}

*/

//IMU初始化
uint32_t IMU_Init(){
    IMU_TX_Buffer[0] = 0x3A;
    memset(IMU_TX_Buffer + 1, 0, IMU_BUFFER_SIZE - 1);
	  IMU_Go_To_Command_Mode();
	  IMU_Set_Angle_Range();
	  IMU_Go_To_Stream_Mode();
}

//IMU波特率设置 
void IMU_Set_Baudrate(void){
    IMU_TX_Buffer[1]=0x15;
    IMU_TX_Buffer[2]=0x04;
    for(uint8_t i = 3; i<6; i++)
    {
        IMU_TX_Buffer[i] = (uint8_t)((IMU_BAUDRATE>>((i<<1) - 6)) & 0xFF);
    }
    IMU_TX_Buffer[6]=0x00;
    IMU_TX_Buffer[7]=Calculate_LRC(IMU_TX_Buffer,6);
    IMU_TX_Buffer[8]=0x0D;
    IMU_TX_Buffer[9]=0x0A;
    HAL_UART_Transmit(&huart8,IMU_TX_Buffer,10,0xFF);
		//HAL_Delay(10);//temporary
    while (!IMU_RX_End_Flag);  //TODO:SET TIMEOUT
    //if(HAL_OK != IMU_Parse_Response)
    //return HAL_OK;
}

//IMU进入命令模式
void IMU_Go_To_Command_Mode(void){
	IMU_TX_Buffer[1] = GOTO_COMMAND_MODE;
	IMU_TX_Buffer[2] = 0x00;
	IMU_TX_Buffer[3] = Calculate_LRC(IMU_TX_Buffer, 2);
	IMU_TX_Buffer[4] = 0x0D;
	IMU_TX_Buffer[5] = 0x0A;
	HAL_UART_Transmit(&huart8,IMU_TX_Buffer,6,0xFF);
	while (!IMU_RX_End_Flag);  //TODO:SET TIMEOUT
}

void IMU_Go_To_Stream_Mode(void){
	IMU_TX_Buffer[1] = GOTO_STREAM_MODE;
	IMU_TX_Buffer[2] = 0x00;
	IMU_TX_Buffer[3] = Calculate_LRC(IMU_TX_Buffer, 2);
	IMU_TX_Buffer[4] = 0x0D;
	IMU_TX_Buffer[5] = 0x0A;
	HAL_UART_Transmit(&huart8,IMU_TX_Buffer,6,0xFF);
	while (!IMU_RX_End_Flag);  //TODO:SET TIMEOUT
}


//IMU角度范围设置
void IMU_Set_Angle_Range(void){
	IMU_TX_Buffer[1] = SET_180_OUTPUT;
	IMU_TX_Buffer[2] = 0x00;
	IMU_TX_Buffer[3] = Calculate_LRC(IMU_TX_Buffer, 2);
	IMU_TX_Buffer[4] = 0x0D;
	IMU_TX_Buffer[5] = 0x0A;
	HAL_UART_Transmit(&huart8,IMU_TX_Buffer,6,0xFF);
	while (!IMU_RX_End_Flag);  //TODO:SET TIMEOUT
	//HAL_Delay(10);//temporary
}

//IMU角度零点设置
void IMU_Heading_Reset(void){
	IMU_TX_Buffer[1] = RESET_HEADING;
	IMU_TX_Buffer[2] = 0x00;
	IMU_TX_Buffer[3] = Calculate_LRC(IMU_TX_Buffer, 2);
  IMU_TX_Buffer[4] = 0x0D;
	IMU_TX_Buffer[5] = 0x0A;
	HAL_UART_Transmit(&huart8,IMU_TX_Buffer,6,0xFF);
	while (!IMU_RX_End_Flag);
}

//LRC校验位计算
uint8_t Calculate_LRC(uint8_t *ptr, uint8_t len){
    uint8_t sum = 0x00;
	  uint8_t i;
    for(i = 1; i <=len;i++) sum+= *(ptr + i);
    return sum;
}
