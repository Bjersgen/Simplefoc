#include "NRF24L01.h"

//NRF24L01 Çı¶¯º¯Êı 

uint8_t ff = 0xFF;
HAL_StatusTypeDef ERRORERROR;

unsigned char idel_mode_flag = 0;
unsigned char mode_time_counter = 0;	

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x11,0xa9,0x56,0x82,0x21}; //·¢Éä»ú½ÓÊÕµØÖ·
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x00,0x98,0x45,0x71,0x10}; //Ğ¡³µ½ÓÊÕµØÖ·
//const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x00,0x98,0x45,0x71,0x10}; //·¢ËÍµØÖ·
//const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x00,0x98,0x45,0x71,0x10}; //·¢ËÍµØÖ·	

extern uint8_t tx_freq;	//24L01ÆµÂÊ³õÊ¼»¯Îª90
extern uint8_t rx_freq;	//24L01ÆµÂÊ³õÊ¼»¯Îª90
#ifdef bandwidth_1Mbps
uint8_t bandwidth = 0x06;  //´ø¿í³õÊ¼»¯1Mbps
#endif
#ifdef bandwidth_250kbps
uint8_t bandwidth = 0x26;  //´ø¿í³õÊ¼»¯Îª0.25Mbps
#endif

//³õÊ¼»¯24L01µÄIO¿Ú
void NRF24L01_RX_Init(void)
{
	Set_NRF24L01_F27_RXEN;
	Clr_NRF24L01_F27_TXEN;
	
	Set_NRF24L01_RX_CE;                                    //³õÊ¼»¯Ê±ÏÈÀ­¸ß
  Set_NRF24L01_RX_CSN;                                   //³õÊ¼»¯Ê±ÏÈÀ­¸ß

	MX_SPI2_Init();                                     //³õÊ¼»¯SPI
	Clr_NRF24L01_RX_CE; 	                                  //Ê¹ÄÜ24L01
	Set_NRF24L01_RX_CSN;                                   //SPIÆ¬Ñ¡È¡Ïû
}
void NRF24L01_TX_Init(void)
{
	Set_NRF24L01_TX_CE;                                    //³õÊ¼»¯Ê±ÏÈÀ­¸ß
  Set_NRF24L01_TX_CSN;                                   //³õÊ¼»¯Ê±ÏÈÀ­¸ß

	MX_SPI1_Init();                                     //³õÊ¼»¯SPI
	Clr_NRF24L01_TX_CE; 	                                  //Ê¹ÄÜ24L01
	Set_NRF24L01_TX_CSN;                                   //SPIÆ¬Ñ¡È¡Ïû
}
//ÉÏµç¼ì²âNRF24L01ÊÇ·ñÔÚÎ»
//Ğ´5¸öÊı¾İÈ»ºóÔÙ¶Á»ØÀ´½øĞĞ±È½Ï£¬
//ÏàÍ¬Ê±·µ»ØÖµ:0£¬±íÊ¾ÔÚÎ»;·ñÔò·µ»Ø1£¬±íÊ¾²»ÔÚÎ»	
uint8_t NRF24L01_RX_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t rx_buf1[5];
	uint8_t i; 
	NRF24L01_RX_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//Ğ´Èë5¸ö×Ö½ÚµÄµØÖ·.	
	NRF24L01_RX_Read_Buf(TX_ADDR,rx_buf1,5);              //¶Á³öĞ´ÈëµÄµØÖ·
	for(i=0;i<5;i++){
		if(rx_buf1[i] != 0XA5)
			break;
	};
	if(i!=5)
		return 1;                               //NRF24L01²»ÔÚÎ»	
	return 0;		                                //NRF24L01ÔÚÎ»
}	

//////NRF2401 DMA CHECK
//////
//////uint8_t NRF24L01_RX_DMA_Check_flag = 0;
//////uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
//////uint8_t rx_buf1[5] = {0, 0, 0, 0, 0};
//////uint8_t rx_temp;
//////uint8_t rx_status;
//////uint8_t NRF24L01_RX_DMA_Check(void)
//////{
//////	NRF24L01_RX_DMA_Check_flag = 1;
////////	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
////////	uint8_t buf1[5];
////////	uint8_t i; 
////////	uint8_t status, temp;
////// 	Clr_NRF24L01_RX_CSN;                                    //Ê¹ÄÜSPI´«Êä
//////	rx_temp = SPI_WRITE_REG+TX_ADDR;
//////	HAL_SPI_TransmitReceive_DMA(&hspi1, &rx_temp, &rx_status, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ
//////	return 0;
////////	HAL_SPI_Transmit_DMA(&hspi1, buf, 5);     //Ğ´ÈëÊı¾İ
////////  Set_NRF24L01_RX_CSN;                                    //¹Ø±ÕSPI´«Êä
////////	
////////	Clr_NRF24L01_RX_CSN;                     //Ê¹ÄÜSPI´«Êä
////////	temp = TX_ADDR;
////////	HAL_SPI_Transmit_DMA(&hspi1, &temp, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ?
////////	HAL_SPI_Receive_DMA(&hspi1, buf1, 5);     //¶Á³öÊı¾İ
////////  Set_NRF24L01_RX_CSN;                     //¹Ø±ÕSPI´«Êä
////////	
////////	for(i=0;i<5;i++){
////////		if(buf1[i] != 0XA5)
////////			break;
////////	};
////////	if(i!=5)
////////		return 1;                               //NRF24L01²»ÔÚÎ»	
////////	return 0;		                                //NRF24L01ÔÚÎ»
//////}	

//////void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//////{
//////	if(hspi->Instance == SPI1){
//////		if(NRF24L01_RX_DMA_Check_flag == 1){
//////			NRF24L01_RX_DMA_Check_flag ++;
//////			HAL_SPI_Transmit_DMA(&hspi1, buf, 5);     //Ğ´ÈëÊı¾İ
//////		}
//////	}
//////}
//////void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
//////{
//////	if(hspi->Instance == SPI1){
//////		if(NRF24L01_RX_DMA_Check_flag == 3){
//////			NRF24L01_RX_DMA_Check_flag ++;
//////			HAL_SPI_Receive_DMA(&hspi1, rx_buf1, 5);     //¶Á³öÊı¾İ
//////		}
//////		if(NRF24L01_RX_DMA_Check_flag == 2){
//////			Set_NRF24L01_RX_CSN; 
//////			NRF24L01_RX_DMA_Check_flag ++;
//////			Clr_NRF24L01_RX_CSN;  
//////			rx_temp = TX_ADDR;
//////			HAL_SPI_Transmit_DMA(&hspi1, &rx_temp, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ?
//////		}
//////	}
//////}

//////void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//////{
//////	uint8_t i = 0;
//////	if(hspi->Instance == SPI1){
//////		if(NRF24L01_RX_DMA_Check_flag == 4){
//////			NRF24L01_RX_DMA_Check_flag = 0;
//////			for(i=0;i<5;i++){
//////				if(rx_buf1[i] != 0XA5)
//////					break;
//////			};
//////			if(i == 5)	
//////				HAL_GPIO_TogglePin(RX_COM_GPIO_Port, RX_COM_Pin);								//NRF24L01ÔÚÎ»
//////		}
//////	}
//////}

uint8_t NRF24L01_TX_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t tx_buf1[5];
	uint8_t i; 
	NRF24L01_TX_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//Ğ´Èë5¸ö×Ö½ÚµÄµØÖ·.	
	NRF24L01_TX_Read_Buf(TX_ADDR,tx_buf1,5);              //¶Á³öĞ´ÈëµÄµØÖ·
	for(i=0;i<5;i++){
		if(tx_buf1[i] != 0XA5)
			break;
	};
	if(i!=5)
		return 1;                               //NRF24L01²»ÔÚÎ»	
	return 0;		                                //NRF24L01ÔÚÎ»
}	 
//Í¨¹ıSPIĞ´¼Ä´æÆ÷
uint8_t NRF24L01_RX_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;
  Clr_NRF24L01_RX_CSN;                    //Ê¹ÄÜSPI´«Êä
	
	//×èÈûĞÍ
	HAL_SPI_TransmitReceive(&hspi2, &regaddr, &status, 1, NRF24L01_TIME_OUT); //·¢ËÍ¼Ä´æÆ÷ºÅ 
	HAL_SPI_Transmit(&hspi2, &data, 1, NRF24L01_TIME_OUT);           //Ğ´Èë¼Ä´æÆ÷µÄÖµ
	
//////	//DMA
//////	HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1); //·¢ËÍ¼Ä´æÆ÷ºÅ 
//////	HAL_SPI_Transmit_DMA(&hspi1, &data, 1);           //Ğ´Èë¼Ä´æÆ÷µÄÖµ
	
  Set_NRF24L01_RX_CSN;                    //½ûÖ¹SPI´«Êä	   
  return(status);       		         //·µ»Ø×´Ì¬Öµ
}
uint8_t NRF24L01_TX_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;
  Clr_NRF24L01_TX_CSN;                    //Ê¹ÄÜSPI´«Êä
	
	//×èÈûĞÍ
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT); //·¢ËÍ¼Ä´æÆ÷ºÅ 
	HAL_SPI_Transmit(&hspi1, &data, 1, NRF24L01_TIME_OUT);           //Ğ´Èë¼Ä´æÆ÷µÄÖµ
	
//////	//DMA
//////	HAL_SPI_TransmitReceive_DMA(&hspi2, &regaddr, &status, 1); //·¢ËÍ¼Ä´æÆ÷ºÅ 
//////	HAL_SPI_Transmit_DMA(&hspi2, &data, 1);           //Ğ´Èë¼Ä´æÆ÷µÄÖµ
	
  Set_NRF24L01_TX_CSN;                    //½ûÖ¹SPI´«Êä	   
  return(status);       		         //·µ»Ø×´Ì¬Öµ
}
//¶ÁÈ¡SPI¼Ä´æÆ÷Öµ £¬regaddr:Òª¶ÁµÄ¼Ä´æÆ÷
uint8_t NRF24L01_RX_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val, status;    
 	Clr_NRF24L01_RX_CSN;                //Ê¹ÄÜSPI´«Êä
	
	//×èÈûĞÍ
	HAL_SPI_TransmitReceive(&hspi2, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //·¢ËÍ¼Ä´æÆ÷ºÅ
  HAL_SPI_Receive(&hspi2, &reg_val, 1, NRF24L01_TIME_OUT);		//¶ÁÈ¡¼Ä´æÆ÷ÄÚÈİ
	
//////	//DMA
//////	HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1); //·¢ËÍ¼Ä´æÆ÷ºÅ
//////	HAL_SPI_Receive_DMA(&hspi1, &reg_val, 1);           //Ğ´Èë¼Ä´æÆ÷µÄÖµ
	
  Set_NRF24L01_RX_CSN;                //½ûÖ¹SPI´«Êä		    
  return reg_val;                 //·µ»Ø×´Ì¬Öµ
}	
uint8_t NRF24L01_TX_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val, status;    
 	Clr_NRF24L01_TX_CSN;                //Ê¹ÄÜSPI´«Êä
	
	//×èÈûĞÍ
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //·¢ËÍ¼Ä´æÆ÷ºÅ
  HAL_SPI_Receive(&hspi1, &reg_val, 1, NRF24L01_TIME_OUT);		//¶ÁÈ¡¼Ä´æÆ÷ÄÚÈİ
	
//////	//DMA
//////	HAL_SPI_TransmitReceive_DMA(&hspi2, &regaddr, &status, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ
//////  HAL_SPI_Receive_DMA(&hspi2, &reg_val, 1);		//¶ÁÈ¡¼Ä´æÆ÷ÄÚÈİ
	
  Set_NRF24L01_TX_CSN;                //½ûÖ¹SPI´«Êä		    
  return reg_val;                 //·µ»Ø×´Ì¬Öµ
}	
//ÔÚÖ¸¶¨Î»ÖÃ¶Á³öÖ¸¶¨³¤¶ÈµÄÊı¾İ
//*pBuf:Êı¾İÖ¸Õë
//·µ»ØÖµ,´Ë´Î¶Áµ½µÄ×´Ì¬¼Ä´æÆ÷Öµ 
void NRF24L01_RX_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{      
//	uint8_t status;
	Clr_NRF24L01_RX_CSN;                     //Ê¹ÄÜSPI´«Êä

	//×èÈûĞÍ
	HAL_SPI_Transmit(&hspi2, &regaddr, 1, NRF24L01_TIME_OUT);     //·¢ËÍ¼Ä´æÆ÷ºÅÖ
	HAL_SPI_Receive(&hspi2, pBuf, datalen, NRF24L01_TIME_OUT);     //¶Á³öÊı¾İ
	
////////	//DMA
////////	HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ?
////////	HAL_Delay(1000);
////////	HAL_SPI_Receive_DMA(&hspi1, pBuf, datalen);     //¶Á³öÊı¾İ
////////	HAL_Delay(1000);
	
  Set_NRF24L01_RX_CSN;                     //¹Ø±ÕSPI´«Êä
}
void NRF24L01_TX_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{      
//	uint8_t status;
	Clr_NRF24L01_TX_CSN;                     //Ê¹ÄÜSPI´«Êä

	//×èÈûĞÍ
	HAL_SPI_Transmit(&hspi1, &regaddr, 1, NRF24L01_TIME_OUT);     //·¢ËÍ¼Ä´æÆ÷ºÅ
	HAL_SPI_Receive(&hspi1, pBuf, datalen, NRF24L01_TIME_OUT);     //¶Á³öÊı¾İ
	
//////	//DMA
//////	HAL_SPI_Transmit_DMA(&hspi2, &regaddr, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ
//////	HAL_SPI_Receive_DMA(&hspi2, pBuf, datalen);     //¶Á³öÊı¾İ
//////	HAL_Delay(0);
	
  Set_NRF24L01_TX_CSN;                     //¹Ø±ÕSPI´«Êä
}
//ÔÚÖ¸¶¨Î»ÖÃĞ´Ö¸¶¨³¤¶ÈµÄÊı¾İ
//*pBuf:Êı¾İÖ¸Õë
//·µ»ØÖµ,´Ë´Î¶Áµ½µÄ×´Ì¬¼Ä´æÆ÷Öµ
uint8_t NRF24L01_RX_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status;
 	Clr_NRF24L01_RX_CSN;                                    //Ê¹ÄÜSPI´«Êä

	//×èÈûĞÍ
	HAL_SPI_TransmitReceive(&hspi2, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //·¢ËÍ¼Ä´æÆ÷ºÅ
	HAL_SPI_Transmit(&hspi2, pBuf, datalen, NRF24L01_TIME_OUT);     //Ğ´ÈëÊı¾İ	

////////	//DMA
////////	HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ
////////	HAL_Delay(1000);
////////	HAL_SPI_Transmit_DMA(&hspi1, pBuf, datalen);     //Ğ´ÈëÊı¾İ
////////	HAL_Delay(1000);
	
	
  Set_NRF24L01_RX_CSN;                                    //¹Ø±ÕSPI´«Êä
  return status;                                       //·µ»Ø¶Áµ½µÄ×´Ì¬Öµ
}		
uint8_t NRF24L01_TX_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status;
 	Clr_NRF24L01_TX_CSN;                                    //Ê¹ÄÜSPI´«Êä

	//×èÈûĞÍ
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //·¢ËÍ¼Ä´æÆ÷ºÅ
	HAL_SPI_Transmit(&hspi1, pBuf, datalen, NRF24L01_TIME_OUT);     //Ğ´ÈëÊı¾İ	
	 
//////	//DMA
//////	HAL_SPI_TransmitReceive_DMA(&hspi2, &regaddr, &status, 1);     //·¢ËÍ¼Ä´æÆ÷ºÅ
//////	HAL_SPI_Transmit_DMA(&hspi2, pBuf, datalen);     //Ğ´ÈëÊı¾İ
//////	HAL_Delay(0);
	
  Set_NRF24L01_TX_CSN;                                    //¹Ø±ÕSPI´«Êä
  return status;                                       //·µ»Ø¶Áµ½µÄ×´Ì¬Öµ
}	
//Æô¶¯NRF24L01·¢ËÍÒ»´ÎÊı¾İ
//txbuf:´ı·¢ËÍÊı¾İÊ×µØÖ·
//·µ»ØÖµ:·¢ËÍÍê³É×´¿ö
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{ 
	NRF24L01_TX_Write_Reg(FLUSH_TX,0xff);               //Çå³ıTX FIFO¼Ä´æÆ÷  
	Clr_NRF24L01_TX_CE;
  NRF24L01_TX_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//Ğ´Êı¾İµ½TX BUF  25¸ö×Ö½Ú
 	Set_NRF24L01_TX_CE;                                     //Æô¶¯·¢ËÍ	   
	return TX_OK;                                         //·¢ËÍÍê³É
}

//Æô¶¯NRF24L01·¢ËÍÒ»´ÎÊı¾İ
//txbuf:´ı·¢ËÍÊı¾İÊ×µØÖ·
//·µ»ØÖµ:0£¬½ÓÊÕÍê³É£»ÆäËû£¬´íÎó´úÂë
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{	
	uint8_t state;		    							      
	state=NRF24L01_RX_Read_Reg(STATUS);                //¶ÁÈ¡×´Ì¬¼Ä´æÆ÷µÄÖµ   
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+STATUS,state); //Çå³ıTX_DS»òMAX_RTÖĞ¶Ï±êÖ¾
	if(state&RX_OK)                                 //½ÓÊÕµ½Êı¾İ
	{
		NRF24L01_RX_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//¶ÁÈ¡Êı¾İ
		NRF24L01_RX_Write_Reg(FLUSH_RX,0xff);          //Çå³ıRX FIFO¼Ä´æÆ÷ 
		return 0; 
	}	  	
	return 1;                                      //Ã»ÊÕµ½ÈÎºÎÊı¾İ
}

//¸Ãº¯Êı³õÊ¼»¯NRF24L01µ½RXÄ£Ê½
//ÉèÖÃRXµØÖ·,Ğ´RXÊı¾İ¿í¶È,Ñ¡ÔñRFÆµµÀ,²¨ÌØÂÊºÍLNA HCURR
//µ±CE±ä¸ßºó,¼´½øÈëRXÄ£Ê½,²¢¿ÉÒÔ½ÓÊÕÊı¾İÁË		   
void RX_Mode(void)
{
	Clr_NRF24L01_RX_CE;	  
	NRF24L01_RX_Write_Reg( SETUP_AW, 0x3 );
	
  //Ğ´RX½ÚµãµØÖ·
  NRF24L01_RX_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);

  //Disable Í¨µÀ0µÄ×Ô¶¯Ó¦´ğ
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);    
  //Ê¹ÄÜÍ¨µÀ0µÄ½ÓÊÕµØÖ·  	 
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
  //ÉèÖÃRFÍ¨ĞÅÆµÂÊ		  
  NRF24L01_RX_Write_Reg(SPI_WRITE_REG+RF_CH,rx_freq);	     
  //Ñ¡ÔñÍ¨µÀ0µÄÓĞĞ§Êı¾İ¿í¶È 	    
  NRF24L01_RX_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  //ÉèÖÃTX·¢Éä²ÎÊı,20dbÔöÒæ,0.25Mbps,µÍÔëÉùÔöÒæ¿ªÆô   
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+RF_SETUP,bandwidth);
  //ÅäÖÃ»ù±¾¹¤×÷Ä£Ê½µÄ²ÎÊı;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX½ÓÊÕÄ£Ê½ 
  NRF24L01_RX_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0f); 
  //CEÎª¸ß,½øÈë½ÓÊÕÄ£Ê½ 
	Set_NRF24L01_RX_CE;      
}			

//¸Ãº¯Êı³õÊ¼»¯NRF24L01µ½TXÄ£Ê½
//ÉèÖÃTXµØÖ·,Ğ´TXÊı¾İ¿í¶È,ÉèÖÃRX×Ô¶¯Ó¦´ğµÄµØÖ·,Ìî³äTX·¢ËÍÊı¾İ,
//Ñ¡ÔñRFÆµµÀ,²¨ÌØÂÊºÍLNA HCURR PWR_UP,CRCÊ¹ÄÜ
//µ±CE±ä¸ßºó,¼´½øÈëRXÄ£Ê½,²¢¿ÉÒÔ½ÓÊÕÊı¾İÁË		   
//CEÎª¸ß´óÓÚ10us,ÔòÆô¶¯·¢ËÍ.	 
void TX_Mode(void)
{														 
	Clr_NRF24L01_TX_CE;	    
	//Set up Address Width
	NRF24L01_TX_Write_Reg( SETUP_AW, 0x3 );
      
	//Ğ´TX½ÚµãµØÖ· 
  NRF24L01_TX_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    
  //ÉèÖÃTX½ÚµãµØÖ·,Ö÷ÒªÎªÁËÊ¹ÄÜACK	  
	NRF24L01_TX_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); 
	NRF24L01_TX_Write_Reg(SPI_WRITE_REG+RX_PW_P0, TX_PLOAD_WIDTH );
  //Disable Í¨µÀ0µÄ×Ô¶¯Ó¦´ğ
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+EN_AA,0x00);     
  //Disable All Í¨µÀµÄ½ÓÊÕµØÖ·
	NRF24L01_TX_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x00); 
  //ÉèÖÃ×Ô¶¯ÖØ·¢¼ä¸ôÊ±¼ä:500us + 86us;×î´ó×Ô¶¯ÖØ·¢´ÎÊı:10´Î
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0);
	//ÉèÖÃRFÍ¨µÀÎª24
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+RF_CH,tx_freq);
	//NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,frequency + );
  //ÉèÖÃTX·¢Éä²ÎÊı,20dbÔöÒæ,0.25Mbps,µÍÔëÉùÔöÒæ¿ªÆô   
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+RF_SETUP,bandwidth);
  //ÅäÖÃ»ù±¾¹¤×÷Ä£Ê½µÄ²ÎÊı;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX·¢ËÍÄ£Ê½,¿ªÆôËùÓĞÖĞ¶Ï
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+CONFIG, ( 1 << 3 ) | //Enable CRC
                                      ( 1 << 1 )| // PWR_UP
																			( 1 << 2)); // 16bit CRC
  // CEÎª¸ß,10usºóÆô¶¯·¢ËÍ
	Set_NRF24L01_TX_CE;    
}	


