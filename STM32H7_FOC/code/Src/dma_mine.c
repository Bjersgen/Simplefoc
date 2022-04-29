#include "dma_mine.h"

DMA_HandleTypeDef  I2C3TxDMA_Handler;      //DMA句柄

//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx)
{ 
	if((int)DMA_Streamx>(int)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	}
    
    __HAL_LINKDMA(&hi2c3,hdmatx,I2C3TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
    
    //Tx DMA配置
    I2C3TxDMA_Handler.Instance=DMA_Streamx;                            //数据流选择
		I2C3TxDMA_Handler.Init.Request=DMA_REQUEST_I2C3_TX;				//USART1发送DMA
    I2C3TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
    I2C3TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    I2C3TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    I2C3TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    I2C3TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    I2C3TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
    I2C3TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    I2C3TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    I2C3TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    I2C3TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    I2C3TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
    
    HAL_DMA_DeInit(&I2C3TxDMA_Handler);   
    HAL_DMA_Init(&I2C3TxDMA_Handler);
		//MYDMA_Config(DMA2_Stream7);				//初始化DMA
} 
