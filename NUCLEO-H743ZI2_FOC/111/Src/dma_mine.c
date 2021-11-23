#include "dma_mine.h"

DMA_HandleTypeDef  I2C3TxDMA_Handler;      //DMA���

//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx)
{ 
	if((int)DMA_Streamx>(int)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2ʱ��ʹ��	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1ʱ��ʹ�� 
	}
    
    __HAL_LINKDMA(&hi2c3,hdmatx,I2C3TxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
    
    //Tx DMA����
    I2C3TxDMA_Handler.Instance=DMA_Streamx;                            //������ѡ��
		I2C3TxDMA_Handler.Init.Request=DMA_REQUEST_I2C3_TX;				//USART1����DMA
    I2C3TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //�洢��������
    I2C3TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
    I2C3TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
    I2C3TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
    I2C3TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
    I2C3TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //��������ģʽ
    I2C3TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
    I2C3TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    I2C3TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    I2C3TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
    I2C3TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
    
    HAL_DMA_DeInit(&I2C3TxDMA_Handler);   
    HAL_DMA_Init(&I2C3TxDMA_Handler);
		//MYDMA_Config(DMA2_Stream7);				//��ʼ��DMA
} 
