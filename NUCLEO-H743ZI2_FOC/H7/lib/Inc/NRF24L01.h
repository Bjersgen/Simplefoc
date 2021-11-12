#ifndef __NRF24L01_H
#define __NRF24L01_H	

#include <stdint.h>
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "stm32h7xx_hal.h"
#include "spi.h"

//NRF24L01 �������� 
/****************************************************************************************************/
//NRF24L01�Ĵ�����������
#define SPI_READ_REG     0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define SPI_WRITE_REG    0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD      0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD      0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX         0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX         0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL      0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP              0xFF  //�ղ���,����������״̬�Ĵ���	 
#define R_RX_PL_WID      0x60
#define W_ACK_PLOAD			 0xA8
#define WR_TX_PLOAD_NACK 0xB0

//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  	    0x10  //�ﵽ����ʹ����ж�
#define TX_OK       	0x20  //TX��������ж�
#define RX_OK   	    0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS     0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
#define DYNPD						0x1C
#define FEATRUE					0x1D
/**********************************************************************************************************/

#define NRF24L01_TIME_OUT 0xFF

//NRF2401Ƭѡ�ź�
#define Clr_NRF24L01_TX_CE      HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_RESET)
#define Set_NRF24L01_TX_CE      HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_SET)

//SPIƬѡ�ź�	
#define Clr_NRF24L01_TX_CSN     HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)
#define Set_NRF24L01_TX_CSN     HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)
    
//NRF2401_IRQ��������
#define Clr_NRF24L01_TX_IRQ     HAL_GPIO_WritePin(SPI1_IRQ_GPIO_Port, SPI1_IRQ_Pin, GPIO_PIN_RESET) 
#define Set_NRF24L01_TX_IRQ     HAL_GPIO_WritePin(SPI1_IRQ_GPIO_Port, SPI1_IRQ_Pin, GPIO_PIN_SET) 

#define READ_NRF24L01_TX_IRQ    HAL_GPIO_ReadPin(SPI1_IRQ_GPIO_Port, SPI1_IRQ_Pin) 


//NRF2401Ƭѡ�ź�
#define Clr_NRF24L01_RX_CE      HAL_GPIO_WritePin(SPI2_CE_GPIO_Port, SPI2_CE_Pin, GPIO_PIN_RESET)
#define Set_NRF24L01_RX_CE      HAL_GPIO_WritePin(SPI2_CE_GPIO_Port, SPI2_CE_Pin, GPIO_PIN_SET)

//SPIƬѡ�ź�	
#define Clr_NRF24L01_RX_CSN     HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET)
#define Set_NRF24L01_RX_CSN     HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET)
    
//NRF2401_IRQ��������
#define Clr_NRF24L01_RX_IRQ     HAL_GPIO_WritePin(SPI2_IRQ_GPIO_Port, SPI2_IRQ_Pin, GPIO_PIN_RESET) 
#define Set_NRF24L01_RX_IRQ     HAL_GPIO_WritePin(SPI2_IRQ_GPIO_Port, SPI2_IRQ_Pin, GPIO_PIN_SET) 

#define READ_NRF24L01_RX_IRQ    HAL_GPIO_ReadPin(SPI2_IRQ_GPIO_Port, SPI2_IRQ_Pin) 


//NRF2401F27����ʹ��
#define Set_NRF24L01_F27_TXEN    HAL_GPIO_WritePin(F27_TXEN_GPIO_Port, F27_TXEN_Pin, GPIO_PIN_SET) 
#define Clr_NRF24L01_F27_TXEN    HAL_GPIO_WritePin(F27_TXEN_GPIO_Port, F27_TXEN_Pin, GPIO_PIN_RESET) 

//NRF2401F27����ʹ��
#define Set_NRF24L01_F27_RXEN    HAL_GPIO_WritePin(F27_RXEN_GPIO_Port, F27_RXEN_Pin, GPIO_PIN_SET) 
#define Clr_NRF24L01_F27_RXEN    HAL_GPIO_WritePin(F27_RXEN_GPIO_Port, F27_RXEN_Pin, GPIO_PIN_RESET) 

//NRF24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5                               //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5                               //5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  25                              //25�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  25                              //25�ֽڵ��û����ݿ��
#define MAX_TIME_INTO_IDEL		10						//����IDELģʽ�����ʱ�䣬��λ��S
extern 	unsigned char idel_mode_flag;								   	   
extern 	unsigned char mode_time_counter;								   	   

void NRF24L01_RX_Init(void);                                //NRF24l01��ʼ��
void NRF24L01_TX_Init(void);                                //NRF24l01��ʼ��
void RX_Mode(void);                                      //����Ϊ����ģʽ
void TX_Mode(void);                                      //����Ϊ����ģʽ
uint8_t NRF24L01_RX_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen); //д������
uint8_t NRF24L01_TX_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen); //д������
void NRF24L01_RX_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);  //��������	
void NRF24L01_TX_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);  //��������
uint8_t NRF24L01_RX_Read_Reg(uint8_t regaddr);		                 //���Ĵ���
uint8_t NRF24L01_TX_Read_Reg(uint8_t regaddr);		                 //���Ĵ���
uint8_t NRF24L01_RX_Write_Reg(uint8_t regaddr, uint8_t data);              //д�Ĵ���
uint8_t NRF24L01_TX_Write_Reg(uint8_t regaddr, uint8_t data);              //д�Ĵ���
uint8_t NRF24L01_RX_Check(void);                                 //���NRF24L01�Ƿ���λ
uint8_t NRF24L01_RX_DMA_Check(void);                                 //���NRF24L01�Ƿ���λ
uint8_t NRF24L01_TX_Check(void);                                 //���NRF24L01�Ƿ���λ
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);                         //����һ����������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);                         //����һ����������
#endif











