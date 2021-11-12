/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include "arm_math.h"
#include "NRF24L01.h"
#include "dma_mine.h"
#ifdef IMU_TEST_1
#include "imu.h"
#include "stm32h7xx_ll_dma.h"
#endif
//#include <cstdio>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void Init_Timers(void);
void Init_PCA9539(void);  // PCA9539初始化
void Buzzer_Ring(void);   //蜂鸣器鸣叫
void Buzzer_Off(void);    //蜂鸣器关闭
void Is_Infrared(void);
uint8_t Shoot_Chip(void);
void Is_BatteryLow_BootCharged(void);
void dribber(void);
void pack(uint8_t *);
//void PWM_SET_VALUE(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t value);
void unpack(uint8_t *Packet);
void motion_planner(void);

#ifdef AngleTest
double normalize(double angle);
double setEdge(double angle, double edge);
uint8_t time_flag = 0;
uint8_t time_flag_2 = 0;
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Robot参数
uint8_t tx_freq, tx_mode, rx_freq, rx_mode;  //频点
uint8_t robot_num;
uint8_t robot_set = 0x08;
uint8_t Robot_Is_Infrared;      //红外触发
uint8_t Robot_Is_Boot_charged;  //电容充电到60V
uint8_t Robot_drib;
uint8_t Robot_Chip_Or_Shoot;    //chip:1  shoot:0
uint8_t Robot_Is_Shoot;
uint8_t Robot_Is_Chip;
uint8_t shoot_chip_flag = 0;
uint8_t Robot_Boot_Power = 0;
uint8_t Comm_Down_Flag = 0;
uint8_t Robot_Is_Report;
uint8_t Robot_Chipped = 0, Robot_Shooted = 0;
uint8_t Robot_Status = 0, Last_Robot_Status = 0;
int8_t Left_Report_Package = 0;
uint8_t Kick_Count = 0;

uint16_t Init2401_count = 0;
uint8_t sbcount = 0;

uint16_t drib_power, drib_power_set[4] = {0, 3000, 3000, 3000};
//uint8_t selftest_vel_mode = 0x02, selftest_drib_mode = 0x03, selftest_chip_mode = 0x04, selftest_shoot_mode = 0x05, selftest_discharge_mode = 0x06; 
//ALIGN_32BYTES(__attribute__((section (".RAM_D2"))) )
uint16_t ADC_value[32];  //存放ADC采集数据uint16_t ADC_value[32];
int16_t Self_Test_Discharge_Flag = 1000;

//电池电压与电容电压累计值
uint32_t AD_Battery = 0, AD_Battery_Last = 217586, AD_Boot_Cap = 0, AD_Boot_Cap_Last, Total_Missed_Package_Num = 0, Period_2ms_Since_Last_Zero_Motion_Planner = 0;
uint8_t AD_Battery_i = 0;

//uint8_t bat_bool = 0;
int temp_bat;
int temp_boot;


/*IMU Data*/
sensorData IMUData;

//uint8_t TX_frequency = 0x18;			//NRF24L01发射频率   6号频点时为0x18， 8号频点时为0x5a
//uint8_t RX_frequency = 0x5a;			//NRF24L01接收频率

int16_t Vx_package = 0, Vy_package = 0, Vr_package = 0;  //下发机器人速度
int16_t Vx_package_last = 0, Vy_package_last = 0,
        Vr_package_last = 0;  //上一帧下发机器人速度
double Vr_cal = 0.0;
double rot_factor = 0.0;

uint8_t acc_set = 150;  //加速度限制 16ms内合成加速度最大值，单位cm/s
uint16_t acc_r_set = 60;  //
uint8_t RX_Packet[25];    //收包
uint8_t TX_Packet[25] = {
    0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,  //[0-8]
    0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,        //[9-16]
    0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};       //[17-24] //回包

uint16_t Received_packet = 0;
uint16_t transmitted_packet = 0;
uint8_t received_packet_flag = 0;
uint8_t    packet_flag = 0;
uint8_t    to_transmit_packet = 0;
uint16_t miss_packet = 0;  //多久未收包或丢失了多少包
uint8_t    stop_flag = 0;
uint16_t heart_toggle_flag = 0;
uint8_t ADC_circle_i = 0;    

#ifdef NEW_CAR
double sin_angle[4] = {0.7071067811865476, -0.7071067811865476,
                       -0.7071067811865476,
                       0.7071067811865476};  // double sin_angle[4] = {sin(58),
                                             // -sin(58), -sin(45), sin(45)};
double cos_angle[4] = {-0.7071067811865476, -0.7071067811865476,
                       0.7071067811865476,
                       0.7071067811865476};  // double cos_angle[4] = {-cos(58),
                                              // -cos(58), cos(45), cos(45)};
#else
double sin_angle[4] = {0.8480480961564261, -0.8480480961564261,
                       -0.7071067811865476,
                       0.7071067811865476};  // double sin_angle[4] = {sin(58),
                                             // -sin(58), -sin(45), sin(45)};
double cos_angle[4] = {-0.5299192642332049, -0.5299192642332049,
                       0.7071067811865476,
                       0.7071067811865476};  // double cos_angle[4] = {-cos(58),
                                              // -cos(58), cos(45), cos(45)};
#endif    
//轮子角度



//调试输出
uint8_t temp[200];
uint8_t temp_count = 0;

//计时器计数
uint32_t Time_count = 0;
uint32_t Time_count_last = 0;
uint32_t Interval_count = 0;
uint64_t timer_time = 0;
uint16_t Time_PID_flag = 0;
uint32_t vel_do_flag = 0;

uint16_t chipshoot_timerdelay_flag = 0;

uint16_t selftest_timer_flag = 0;

//时间间隔，单位：ms
double Interval = 0;

//编码器计数
int Encoder_count_Motor1 = 0;
int Encoder_count_Motor2 = 0;
int Encoder_count_Motor3 = 0;
int Encoder_count_Motor4 = 0;

//控制速度与实际速度
int Vel_Motor1_aim = 0, Vel_Motor1 = 0, Vel_Now_Motor1 = 0, Vel_last_Motor1[8], Vel_last_sum_Motor1;																//转速 单位：rpm
int Vel_Motor2_aim = 0, Vel_Motor2 = 0, Vel_Now_Motor2 = 0, Vel_last_Motor2[8], Vel_last_sum_Motor2;																//转速 单位：rpm
int Vel_Motor3_aim = 0, Vel_Motor3 = 0, Vel_Now_Motor3 = 0, Vel_last_Motor3[8], Vel_last_sum_Motor3;																//转速 单位：rpm
int Vel_Motor4_aim = 0, Vel_Motor4 = 0, Vel_Now_Motor4 = 0, Vel_last_Motor4[8], Vel_last_sum_Motor4;																//转速 单位：rpm
//uint8_t vel_filter_flag = 0, vel_filter_temp;

//占空比
double PWM_Pulse_Motor1, PWM_Pulse_Motor2, PWM_Pulse_Motor3, PWM_Pulse_Motor4, PWM_Pulse_Motord;	

//实际PWM 占空value
int PWM_Pulse_Motor1_value, PWM_Pulse_Motor2_value, PWM_Pulse_Motor3_value, PWM_Pulse_Motor4_value;

////速度转换系数
//double Vel_k2 = 2.727 *  4 * 360 / 2 / 3.1415926 / 2.3 / 1000 * 2;		//  cm/s  ----->>>>>>>   count/2ms

#ifdef ENCODER_1000
const double Vel_k2 = 1.446036111111111;			// normal
//const double Vel_k2 = 0.8514063970211044;
// const double Vel_k2 = -0.7954321118415024;		// 45degree
#endif

#ifdef ENCODER_360
const double Vel_k2 = 0.520573;
#endif

#if defined(ENCODER_1000) && defined(ROBOT_50W)
//const uint32_t Motor_KP = 40, Motor_KI = 11;
//const int PID_I_Limit = 720; //7995 / Motor_KI;
const uint32_t Motor_KP = 20, Motor_KI = 2;
const int PID_I_Limit = 1400; //7995 / Motor_KI;
#endif

#if defined(ENCODER_360) && defined(ROBOT_50W)
const uint32_t Motor_KP = 110, Motor_KI = 30;
const int PID_I_Limit = 266; //7995 / Motor_KI;
#endif

#if defined(ENCODER_1000) && defined(ROBOT_70W)
const uint32_t Motor_KP = 40, Motor_KI = 11;         //Under Testing
const int PID_I_Limit = 720; //7995 / Motor_KI;
#endif

#if defined(ENCODER_360) && defined(ROBOT_70W)
const uint32_t Motor_KP = 110, Motor_KI = 30;         //Under Testing
const int PID_I_Limit = 266; //7995 / Motor_KI;
#endif
//PID中I积分项
int Motor_EK_Motor1 = 0, Motor_EK_Motor2 = 0, Motor_EK_Motor3 = 0,Motor_EK_Motor4 = 0;
int Motor_KS_Motor1 = 0, Motor_KS_Motor2 = 0, Motor_KS_Motor3 = 0,Motor_KS_Motor4 = 0;

//USART调试
uint16_t USART_flag = 0, USART_flag2 = 0;

#ifdef IMU_TEST_1
uint8_t IMU_RX_End_Flag = 0;
uint8_t IMU_RX_Buffer[IMU_BUFFER_SIZE];
#endif


#ifdef AngleTest
//PID DIR
double dirKP = 1.5, dirKI = 0.2, dirKD = 0.3 * 75;
double lastRotVel = 0;
double dirNow = 0;
double lastdirEK = 0;
double dirKS = 0;
double dirEK = 0;
double dirD = 0;
double lastDir = 0;
double max_dir_acc = 1.5;
double max_dir_speed = 2.5;
double rotVel = 0;
uint8_t DEC_FRAME = 0;
uint8_t use_dir = 0;
#endif

//保存码盘数据和
int Encoder_count_Motor1_sum = 0;
int Encoder_count_Motor2_sum = 0;
int Encoder_count_Motor3_sum = 0;
int Encoder_count_Motor4_sum = 0;
uint8_t speed_count = 0;
double Encoder_count_Motor1_avg = 0;
double Encoder_count_Motor2_avg = 0;
double Encoder_count_Motor3_avg = 0;
double Encoder_count_Motor4_avg = 0;
float motor_input_u[4] = {0.0, 0.0, 0.0, 0.0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_HRTIM_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM12_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_TIM5_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */

    Init_Timers();                        //电机及其他Timer和PWM初始化
    Init_PCA9539();                   //PCA9539初始化
    
    NRF24L01_RX_Init();                //NRF24L01接收初始化
    NRF24L01_TX_Init();                //NRF24L01发送初始化
    RX_Mode();                                //NRF24L01设置为接收模式
    TX_Mode();                                //NRF24L01设置为发送模式
    HAL_TIM_Base_Start_IT(&htim5);
		
#ifdef IMU_TEST_1
		__HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart8,IMU_RX_Buffer,IMU_BUFFER_SIZE);
		HAL_GPIO_WritePin(IMU_NRST_GPIO_Port,IMU_NRST_Pin,GPIO_PIN_SET);//拉高复位脚
		HAL_GPIO_WritePin(UART8_RTS_GPIO_Port, UART8_RTS_Pin, GPIO_PIN_RESET);//拉低RTS(不使用)
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    if((robot_set == 1) && (tx_mode == 0)){                    // Self Test Mode
        
        Buzzer_Ring();
        HAL_Delay(100);
        Buzzer_Off();
        HAL_Delay(200);
        Buzzer_Ring();
        HAL_Delay(100);
        Buzzer_Off();
        HAL_Delay(300);
        Buzzer_Ring();
        HAL_Delay(100);
        Buzzer_Off();
        
        while(1){
            Init_PCA9539();                                               //PCA9539 Get Set Num Tx Rx Number
            Is_Infrared();
            HAL_Delay(1);
            switch(robot_set){
                case 2 :
                        drib_power = 0;
                        Robot_Boot_Power = 0;
                        if(tx_mode <= 8)
                            Vx_package = 10 * tx_mode;
                        else
                            Vx_package = 10 * (tx_mode - 17);
                        motion_planner();
                break;
                        
                case 3 :
                    Robot_Boot_Power = 0;
                    Vx_package = 0;
                    motion_planner();
                    if(Robot_Is_Infrared == 0){
                        drib_power = drib_power_set[1];
                    }
                    else{
                        drib_power = drib_power_set[3];
                    }
                break;

                case 4 :
                    drib_power = 0;
                    Vx_package = 0;
                    motion_planner();
                    Is_BatteryLow_BootCharged();			//电池电压是否低于15.2V，电容是否充电到60V
                    Robot_Boot_Power = 10 * tx_mode;
                    if (Robot_Boot_Power > 127) {
                        Robot_Boot_Power = 127;
                    }
                    if (Robot_Is_Boot_charged && (Robot_Boot_Power != 0) && (Robot_Is_Infrared == 1) && (Self_Test_Discharge_Flag == 0)){
                        htim12.Instance = TIM12;
                        htim12.Init.Period = Robot_Boot_Power * 250 + 10;
                        htim12.Instance->CCR1 = 10;
                        htim12.Instance->CCR2 = Robot_Boot_Power * 250 + 10;
                        HAL_TIM_PWM_Init(&htim12);
                        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
                        Self_Test_Discharge_Flag = 1000;
                    }
                    else{
                        Self_Test_Discharge_Flag --;
                        if (Self_Test_Discharge_Flag<0) Self_Test_Discharge_Flag = 0;
                    }
                break;
                
                case 5 :
                    drib_power = 0;
                    Vx_package = 0;
                    motion_planner();
                    Is_BatteryLow_BootCharged();			//电池电压是否低于15.2V，电容是否充电到60V
                    Robot_Boot_Power = 10 * tx_mode;
                    if (Robot_Boot_Power > 127) {
                        Robot_Boot_Power = 127;
                    }
                    if (Robot_Is_Boot_charged && (Robot_Boot_Power != 0) && (Robot_Is_Infrared == 1) && (Self_Test_Discharge_Flag == 0)){
                        htim12.Instance = TIM12;
                        htim12.Init.Period = Robot_Boot_Power * 250 + 10;
                        htim12.Instance->CCR1 = Robot_Boot_Power * 250 + 10;
                        htim12.Instance->CCR2 = 10;
                        HAL_TIM_PWM_Init(&htim12);
                        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
                        Self_Test_Discharge_Flag = 1000;
                    }
                    else{
                        Self_Test_Discharge_Flag --;
                        if (Self_Test_Discharge_Flag<0) Self_Test_Discharge_Flag = 0;
                    }
                break;
                
                case 6 :
                    drib_power = 0;
                    Vx_package = 0;
                    motion_planner();
                    Is_BatteryLow_BootCharged();
                    Robot_Boot_Power = 50;
                    if (Robot_Is_Boot_charged && (Robot_Boot_Power != 0) && (Self_Test_Discharge_Flag == 0)){
                        htim12.Instance = TIM12;
                        htim12.Init.Period = Robot_Boot_Power * 250 + 10;
                        htim12.Instance->CCR1 = 10;
                        htim12.Instance->CCR2 = Robot_Boot_Power * 250 + 10;
                        HAL_TIM_PWM_Init(&htim12);
                        HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
                        Self_Test_Discharge_Flag = 1000;
                    }
                    else{
                        Self_Test_Discharge_Flag --;
                        if (Self_Test_Discharge_Flag<0) Self_Test_Discharge_Flag = 0;
                    }
                break;    
                default :
                    drib_power = 0;
                    Vx_package = 0;
                    Robot_Boot_Power = 0;
                    motion_planner();
                break;
            }
        }
    }
    
  while(1)
  {

//		//调试输出
//		sprintf(temp, "%d", AD_Battery_Last);	
//		HAL_UART_Transmit(&huart3, &robot_num, 1, 0xffff);
		
//////		//接收2401 check
//////		if(NRF24L01_TX_Check() == 0){
//////			HAL_GPIO_TogglePin(TX_COM_GPIO_Port, TX_COM_Pin);
//////			HAL_Delay(100);
//////		}
//////		if(NRF24L01_RX_Check() == 0){
//////			HAL_GPIO_TogglePin(RX_COM_GPIO_Port, RX_COM_Pin);
//////			HAL_Delay(100);
//////		}
		
//////		NRF2401DMA调试
//////		NRF24L01_RX_DMA_Check();
//////		HAL_Delay(100);

        if(Received_packet > 30){
            HAL_GPIO_TogglePin(RX_COM_GPIO_Port, RX_COM_Pin);
            Received_packet = 0;
        };
        if(transmitted_packet > 30){
            HAL_GPIO_TogglePin(TX_COM_GPIO_Port, TX_COM_Pin);
            transmitted_packet = 0;
        }

		Is_Infrared();				//是否触发红外
		dribber();
		Is_BatteryLow_BootCharged();			//电池电压是否低于15.2V，电容是否充电到60V
////		平挑射放到2ms中断里面
////		Shoot_Chip();					//平挑射
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART8|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 32;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  PeriphClkInitStruct.Hrtim1ClockSelection = RCC_HRTIM1CLK_CPUCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//时钟中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim5.Instance)
	{
			//码盘读数与计时器读数  差分码盘
            #ifdef ENCODER_1000
            Encoder_count_Motor1 =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim1));
            __HAL_TIM_SET_COUNTER(&htim1,0);
            Encoder_count_Motor2 =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim2));
            __HAL_TIM_SET_COUNTER(&htim2,0);
            Encoder_count_Motor3 =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim3)) ;
            __HAL_TIM_SET_COUNTER(&htim3,0);
            Encoder_count_Motor4 =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim4));
            __HAL_TIM_SET_COUNTER(&htim4,0);
            #endif

            #ifdef ENCODER_360
            Encoder_count_Motor1 =  -(int16_t)(__HAL_TIM_GET_COUNTER(&htim1));
            __HAL_TIM_SET_COUNTER(&htim1,0);
            Encoder_count_Motor2 =  -(int16_t)(__HAL_TIM_GET_COUNTER(&htim2));
            __HAL_TIM_SET_COUNTER(&htim2,0);
            Encoder_count_Motor3 =  -(int16_t)(__HAL_TIM_GET_COUNTER(&htim3)) ;
            __HAL_TIM_SET_COUNTER(&htim3,0);
            Encoder_count_Motor4 =  -(int16_t)(__HAL_TIM_GET_COUNTER(&htim4));
            __HAL_TIM_SET_COUNTER(&htim4,0);
            #endif
			//计算轮子转速
			Vel_Now_Motor1 = Encoder_count_Motor1;
			Vel_Now_Motor2 = Encoder_count_Motor2;
			Vel_Now_Motor3 = Encoder_count_Motor3;
			Vel_Now_Motor4 = Encoder_count_Motor4;
			
			//PID EK项
			Motor_EK_Motor1 = Vel_Motor1 - Vel_Now_Motor1;
			Motor_EK_Motor2 = Vel_Motor2 - Vel_Now_Motor2;
			Motor_EK_Motor3 = Vel_Motor3 - Vel_Now_Motor3;
			Motor_EK_Motor4 = Vel_Motor4 - Vel_Now_Motor4;
			
			//PID I积分项
			Motor_KS_Motor1 += Motor_EK_Motor1;
			Motor_KS_Motor2 += Motor_EK_Motor2;
			Motor_KS_Motor3 += Motor_EK_Motor3;
			Motor_KS_Motor4 += Motor_EK_Motor4;
			
			if ((Motor_KS_Motor1)> PID_I_Limit) Motor_KS_Motor1= PID_I_Limit;
			if ((Motor_KS_Motor1)<-PID_I_Limit) Motor_KS_Motor1=-PID_I_Limit;

			if ((Motor_KS_Motor2)> PID_I_Limit) Motor_KS_Motor2= PID_I_Limit;
			if ((Motor_KS_Motor2)<-PID_I_Limit) Motor_KS_Motor2=-PID_I_Limit;

			if ((Motor_KS_Motor3)> PID_I_Limit) Motor_KS_Motor3= PID_I_Limit;
			if ((Motor_KS_Motor3)<-PID_I_Limit) Motor_KS_Motor3=-PID_I_Limit;
			
			if ((Motor_KS_Motor4)> PID_I_Limit) Motor_KS_Motor4= PID_I_Limit;
			if ((Motor_KS_Motor4)<-PID_I_Limit) Motor_KS_Motor4=-PID_I_Limit;
			
			//PID
			PWM_Pulse_Motor1_value = Motor_KP * Motor_EK_Motor1 + Motor_KI * Motor_KS_Motor1;
			PWM_Pulse_Motor2_value = Motor_KP * Motor_EK_Motor2 + Motor_KI * Motor_KS_Motor2;
			PWM_Pulse_Motor3_value = Motor_KP * Motor_EK_Motor3 + Motor_KI * Motor_KS_Motor3;
			PWM_Pulse_Motor4_value = Motor_KP * Motor_EK_Motor4 + Motor_KI * Motor_KS_Motor4;					
			
			//最大占空比限制
			PWM_Pulse_Motor1_value = (PWM_Pulse_Motor1_value >= 7995) ? 7995 : PWM_Pulse_Motor1_value;
			PWM_Pulse_Motor1_value = (PWM_Pulse_Motor1_value <= -7995) ? -7995 : PWM_Pulse_Motor1_value;
			PWM_Pulse_Motor2_value = (PWM_Pulse_Motor2_value >= 7995) ? 7995 : PWM_Pulse_Motor2_value;
			PWM_Pulse_Motor2_value = (PWM_Pulse_Motor2_value <= -7995) ? -7995 : PWM_Pulse_Motor2_value;
			PWM_Pulse_Motor3_value = (PWM_Pulse_Motor3_value >= 7995) ? 7995 : PWM_Pulse_Motor3_value;
			PWM_Pulse_Motor3_value = (PWM_Pulse_Motor3_value <= -7995) ? -7995 : PWM_Pulse_Motor3_value;
			PWM_Pulse_Motor4_value = (PWM_Pulse_Motor4_value >= 7995) ? 7995 : PWM_Pulse_Motor4_value;
			PWM_Pulse_Motor4_value = (PWM_Pulse_Motor4_value <= -7995) ? -7995 : PWM_Pulse_Motor4_value;

            /*********************************************************************************************************/
            //skuba开环力矩模型
#if defined ROBOT_50W
            if (speed_count <= 4){
                Encoder_count_Motor1_sum += abs(Encoder_count_Motor1);
                Encoder_count_Motor2_sum += abs(Encoder_count_Motor2);
                Encoder_count_Motor3_sum += abs(Encoder_count_Motor3);
                Encoder_count_Motor4_sum += abs(Encoder_count_Motor4);
                speed_count ++;
            }
            
            if (speed_count == 5){
                Encoder_count_Motor1_avg = Encoder_count_Motor1_sum / 5.0;
                Encoder_count_Motor2_avg = Encoder_count_Motor2_sum / 5.0;
                Encoder_count_Motor3_avg = Encoder_count_Motor3_sum / 5.0;
                Encoder_count_Motor4_avg = Encoder_count_Motor4_sum / 5.0;
                speed_count = 0;
                Encoder_count_Motor1_sum = 0;
                Encoder_count_Motor2_sum = 0;
                Encoder_count_Motor3_sum = 0;
                Encoder_count_Motor4_sum = 0;
            }
            
            //计算电机的最终输入信号u    u = torgue / [(k_m / R) * Vcc - (k_m / R / k_n) * omega]   k_m = torgue constant;  k_n = speed constant
            // 50W 电机 k_m = 33.5 mNm/A, k_n = 285 rpm/V, R = 28.82mm
            motor_input_u[0] = (PWM_Pulse_Motor1_value) / (0.0541 * 16.8 - 0.000142355 * Encoder_count_Motor1_avg * 500 * 2 * 3.1415926 / 1000);
            motor_input_u[1] = (PWM_Pulse_Motor2_value) / (0.0541 * 16.8 - 0.000142355 * Encoder_count_Motor2_avg * 500 * 2 * 3.1415926 / 1000);
            motor_input_u[2] = (PWM_Pulse_Motor3_value) / (0.0541 * 16.8 - 0.000142355 * Encoder_count_Motor3_avg * 500 * 2 * 3.1415926 / 1000);
            motor_input_u[3] = (PWM_Pulse_Motor4_value) / (0.0541 * 16.8 - 0.000142355 * Encoder_count_Motor4_avg * 500 * 2 * 3.1415926 / 1000);
            
            //放大输入信号为PWM值
            PWM_Pulse_Motor1_value = motor_input_u[0];
            PWM_Pulse_Motor2_value = motor_input_u[1];
            PWM_Pulse_Motor3_value = motor_input_u[2];
            PWM_Pulse_Motor4_value = motor_input_u[3];
#endif
            
            /*********************************************************************************************************/
            
            //控制
            //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 4000 + PWM_Pulse_Motor1_value;
            if (PWM_Pulse_Motor1_value<0) {HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);}
                else {HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);}
            if (abs(PWM_Pulse_Motor1_value)<=3) PWM_Pulse_Motor1_value = 3;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = abs(PWM_Pulse_Motor1_value);
            
            if (PWM_Pulse_Motor2_value<0) {HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);}
              else {HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_SET);}
            if (abs(PWM_Pulse_Motor2_value)<=3) PWM_Pulse_Motor2_value = 3;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = abs(PWM_Pulse_Motor2_value);
            
            if (PWM_Pulse_Motor3_value<0) {HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_RESET);}
                else {HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_SET);}
            if (abs(PWM_Pulse_Motor3_value)<=3) PWM_Pulse_Motor3_value = 3;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = abs(PWM_Pulse_Motor3_value);
            
            if (PWM_Pulse_Motor4_value<0) {HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_Port, MOTOR4_DIR_Pin, GPIO_PIN_RESET);}
                else {HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_Port, MOTOR4_DIR_Pin, GPIO_PIN_SET);}
            if (abs(PWM_Pulse_Motor4_value)<=3) PWM_Pulse_Motor4_value = 3;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = abs(PWM_Pulse_Motor4_value);
            
            heart_toggle_flag ++;
            if(heart_toggle_flag == 500){
                HAL_GPIO_TogglePin(HEART_GPIO_Port, HEART_Pin);
                heart_toggle_flag = 0;
            }

            Is_Infrared();
                
			      htim14.Instance->CCR1 = drib_power;				//吸球
            

            

            if(NRF24L01_RxPacket(RX_Packet) == 0){
                unpack(RX_Packet);
                if(received_packet_flag == 1){
                    motion_planner();
                    Received_packet++;
                    Total_Missed_Package_Num = 0;
                    Comm_Down_Flag = 0;
									
                    if ((Robot_Status != Last_Robot_Status) || (Robot_Is_Infrared) || (Robot_Is_Report == 1)) {
                        Left_Report_Package = 4;
                        Last_Robot_Status = Robot_Status;
                    }
          
//                    else{
//                        if(Left_Report_Package >= 0)
//                            Left_Report_Package --;
//                        Last_Robot_Status = Robot_Status;
//                    }
//                    if(Robot_Is_Report == 1){
//                        Left_Report_Package = 5;
//                    }
											if(Kick_Count > 0) {
												  Kick_Count--;
											}
                      else	{
												Robot_Status &= 0xCF;
											}		   			
                    
                    if(Left_Report_Package > 0 || Kick_Count > 0){
                        pack(TX_Packet);
                        NRF24L01_TxPacket(TX_Packet);
                        transmitted_packet++;
                    }       
										if(Left_Report_Package > 0)   Left_Report_Package --;
										
										
//                    else{
//                        Robot_Status = 0;
//                        Last_Robot_Status = 0;
//                    }
                }
            }
                else {
                    received_packet_flag = 0;
                }
            if (Comm_Down_Flag == 1){
                Period_2ms_Since_Last_Zero_Motion_Planner ++;
                if (Period_2ms_Since_Last_Zero_Motion_Planner>=8){
                    Init2401_count++;
                    if ( Init2401_count == 10 )
                    {
                        // 0519 test 2401 initial
                        NRF24L01_RX_Init();                //NRF24L01接收初始化
                        NRF24L01_TX_Init();                //NRF24L01发送初始化
                        RX_Mode();                                //NRF24L01设置为接收模式
                        TX_Mode();                                //NRF24L01设置为发送模式
                        
                        // Buzzer_Ring();
                        // HAL_Delay(100);
                        // Buzzer_Off();
                        Init2401_count = 0;
                    }
                    

                    // Init2401_count++;
                    // while ( Init2401_count == 10 && sbcount == 0 )
                    // {
                    //     sbcount++;
                    //     if ( sbcount >= 5 )
                    //     {
                    //         continue;
                    //     }        
                        
                    //     Vy_package = 10;
                    //     Vx_package = 10;
                    //     Vr_package = 10;
                    //     use_dir = 0;
                    //     Robot_drib = 0;
                    //     motion_planner();
                    // }
                    
                    Vy_package = 0;
                    Vx_package = 0;
                    Vr_package = 0;
                    use_dir = 0;
                    Robot_drib = 0;
                    motion_planner();
                    Period_2ms_Since_Last_Zero_Motion_Planner = 0;
                }
            }
            else{
                if(received_packet_flag == 0){
                    Total_Missed_Package_Num ++;
                    if (Total_Missed_Package_Num >= 500){ // 500ms Missing Package -> Every 16ms Do Motion Planner
                        Comm_Down_Flag = 1;
                        Vy_package = 0;
                        Vx_package = 0;
                        Vr_package = 0;
                        use_dir = 0;
                        Robot_drib = 0;
                        Period_2ms_Since_Last_Zero_Motion_Planner = 0;
                        motion_planner();
                    }
                }
            }
						
						
            if(chipshoot_timerdelay_flag < 1000)
                chipshoot_timerdelay_flag++;
            
            Shoot_Chip();                    //平挑射
						
						
//            if(selftest_timer_flag <= 8)
//                selftest_timer_flag++;
            #ifdef IMU_TEST_1
            //IMU 上传了一帧数据
            if(IMU_RX_End_Flag == 1){
                //if(IMU_Parse_Stream(IMU_RX_Len)==HAL_OK){
                    //TODO:print(IMUData.gAngle);
                    
                    //sprintf("IMU_DATA");
                  //HAL_UART_Transmit_DMA(&huart3,IMU_RX_Buffer,IMU_RX_Len);
//                  IMU_RX_Len = 0;
                  IMU_RX_End_Flag = 0;
                HAL_UART_Receive_DMA(&huart8, IMU_RX_Buffer, IMU_BUFFER_SIZE);    
            };
            #endif
    }
    time_flag_2 ++;
    if (time_flag_2 == 0xFF)
        time_flag_2 = 0;
}



//蜂鸣器鸣叫
void Buzzer_Ring(){							//蜂鸣器PWM通道打开，频率2K/0.6，占空比0.5
	//HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	htim16.Instance->CCR1=300;
};

//蜂鸣器关闭
void Buzzer_Off(){							//蜂鸣器PWM通道打开，频率2K/0.6，占空比0
	htim16.Instance->CCR1=0;
};

//PCA9539初始化函数
void Init_PCA9539(){
//    uint8_t i;
//    uint8_t PCA9539_REG67_test[2] = {0xa5, 0xa5};
//    uint8_t PCA9539_REG67[2] = {0xff, 0xff};
    uint8_t Pca9539_Read[2];
    uint8_t Pca9539_convert[16] = {15, 7, 11, 3, 13, 5, 9, 1, 14, 6, 10, 2, 12, 4, 8, 0};
    HAL_I2C_Mem_Read(&hi2c3, 0xe8 + (0x03<<1), 0, I2C_MEMADD_SIZE_8BIT, Pca9539_Read, 2, 0x10);
    tx_mode = Pca9539_convert[(Pca9539_Read[0] & 0x0f)];
    rx_mode = Pca9539_convert[(Pca9539_Read[0] >> 4)];
    if(tx_mode < 0x08){
        tx_freq = 4 * tx_mode;
    }
    else{
        tx_freq = 58 + 4 * tx_mode;
    }
    if (tx_mode == 1) tx_freq = 88;
    if (tx_mode == 2) tx_freq = 100;
    if(rx_mode < 0x08){
        rx_freq = 4 * rx_mode;
    }
    else{
        rx_freq = 58 + 4 * rx_mode;
    }
    if (rx_mode == 1) rx_freq = 88;
    if (rx_mode == 2) rx_freq = 100;
    robot_num = Pca9539_convert[(Pca9539_Read[1] & 0x0f)];
    robot_set = Pca9539_convert[(Pca9539_Read[1] >> 4)];
}

//红外是否触发
void Is_Infrared(){
    if(HAL_GPIO_ReadPin(INFRAIN_GPIO_Port, INFRAIN_Pin) == 0){
        HAL_GPIO_WritePin(INFRA_LED_GPIO_Port, INFRA_LED_Pin, GPIO_PIN_SET);
        Robot_Is_Infrared = 0;
        Robot_Status = Robot_Status & 0x30;
    }
    else{
        HAL_GPIO_WritePin(INFRA_LED_GPIO_Port, INFRA_LED_Pin, GPIO_PIN_RESET);
        Robot_Is_Infrared = 1;
        Robot_Status = Robot_Status | (1 << 6);
    }
}

//ADC采集
//电池电压是否低于15.2V，电容是否充电到60V
//Battery_Voltage  分压比22/3.3
//Boot_Cap_Voltage 分压比1000/10
void Is_BatteryLow_BootCharged(){
    //SCB_InvalidateDCache_by_Addr ((uint32_t *)ADC_value, 2);
    AD_Battery = AD_Battery + ADC_value[0];
    AD_Boot_Cap = ADC_value[1];
    
    if(AD_Battery_i >= 5){
        AD_Battery_Last = ((AD_Battery << 2) + (AD_Battery << 1) + AD_Battery + AD_Battery_Last) >> 3;
        AD_Battery = 0;
        AD_Battery_i = 0;
    }
    AD_Battery_i ++;
    
	//电池电压低于15.2V，196864 = 15.2/25.3*3.3/3.3*65535*5
	//电池电压高于3.3V，42740 = 3.3/25.3*3.3/3.3*65535*5   189600
	if(AD_Battery_Last < 189600)
//	if((AD_Battery_Last > 122740)&&(AD_Battery_Last < 189600))
        htim16.Instance->CCR1=300;
    else
        htim16.Instance->CCR1=0;
        
    
    //电容升压到60V，11797 = 60/1010*10/3.3*65535
		//电容最高升压约为195V。
    if(AD_Boot_Cap > 12000){
        Robot_Is_Boot_charged = 1;
        HAL_GPIO_WritePin(BOOT_DONE_GPIO_Port, BOOT_DONE_Pin, GPIO_PIN_RESET);
    }
    else{
        Robot_Is_Boot_charged = 0;
        HAL_GPIO_WritePin(BOOT_DONE_GPIO_Port, BOOT_DONE_Pin, GPIO_PIN_SET);
    }
}

//吸球
void dribber(void){
//	Robot_drib = 3;
	if (Robot_drib == 0){
		drib_power = 0;
		return;
	}
	if (Robot_Is_Infrared == 1){
		drib_power = drib_power_set[Robot_drib];
	}
	else {
		drib_power = drib_power_set[1];
	}	
}

//平挑射
uint8_t Shoot_Chip(){
//////    Robot_Chip_Or_Shoot = 1;
//////    Robot_Boot_Power = 50;
//////    if((Robot_Boot_Power > 0) && (chipshoot_timerdelay_flag >= 1000)){
    
//////    
//////    //不用升压test
//////    if((Robot_Boot_Power > 0) && (Robot_Is_Infrared == 1) && (chipshoot_timerdelay_flag >= 1000)){
        
    //inuse
    if((Robot_Boot_Power > 0) && (Robot_Is_Boot_charged == 1) && (Robot_Is_Infrared == 1) && (chipshoot_timerdelay_flag >= 500)){
        Kick_Count = 8 + 1;
				if(Robot_Chip_Or_Shoot == 1){
                Robot_Is_Chip = 1;
                
//////                htim12.Instance->CCR1 = 32000 - Robot_Boot_Power * 250;
//////                htim12.Instance->CCR2 = 32000;
//////                htim12.Instance = TIM12;
            
                htim12.Instance = TIM12;
                htim12.Init.Period = Robot_Boot_Power * 250 + 10;
                htim12.Instance->CCR1 = 10;
                htim12.Instance->CCR2 = Robot_Boot_Power * 250 + 10;
            
                HAL_TIM_PWM_Init(&htim12);
                HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
                
                Robot_Status = Robot_Status | (1 << 4);
                Robot_Boot_Power = 0;
                chipshoot_timerdelay_flag  = 0;
                Robot_Chip_Or_Shoot = 2;   
            
                return 0;    
        }
        if(Robot_Chip_Or_Shoot == 0){
                Robot_Is_Shoot = 1;
            
//////                htim12.Instance->CCR2 = 32000 - Robot_Boot_Power * 250;
//////                htim12.Instance->CCR1 = 32000;
//////                htim12.Instance = TIM12;

                htim12.Instance = TIM12;
                htim12.Init.Period = Robot_Boot_Power * 250 + 0 + 10;
                htim12.Instance->CCR2 = 10;
                htim12.Instance->CCR1 = Robot_Boot_Power * 250 + 0 + 10;
            
                HAL_TIM_PWM_Init(&htim12);
                HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
                
                Robot_Chip_Or_Shoot = 2;
                Robot_Status = Robot_Status | (1 << 5);
                Robot_Boot_Power = 0;
                chipshoot_timerdelay_flag  = 0;
                
                return 0;
        }		
    }
    return 0;
}

//电机及其他Timer和PWM初始化
void Init_Timers(){
	//Motor1对应的Timer打开及PWM设置
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 3;
	HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1);//通道打开
	HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_A);//开启定时器
    HAL_GPIO_WritePin(MOTOR1_RESET_GPIO_Port, MOTOR1_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR1_MODE_GPIO_Port, MOTOR1_MODE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
    
    
	//Motor2对应的Timer打开及PWM设置
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = 3;
	HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TC2);//通道打开
	HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_C);//开启定时器
    HAL_GPIO_WritePin(MOTOR2_RESET_GPIO_Port, MOTOR2_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_MODE_GPIO_Port, MOTOR2_MODE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_SET);
    
	//Motor3对应的Timer打开及PWM设置
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = 3;
	HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TD2);//通道打开
	HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_D);//开启定时器
	HAL_GPIO_WritePin(MOTOR3_RESET_GPIO_Port, MOTOR3_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR3_MODE_GPIO_Port, MOTOR3_MODE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_SET);
	
	//Motor4对应的Timer打开及PWM设置
	HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = 3;
	HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TB1);//通道打开
	HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_TIMER_B);//开启定时器
	HAL_GPIO_WritePin(MOTOR4_RESET_GPIO_Port, MOTOR4_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR4_MODE_GPIO_Port, MOTOR4_MODE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_Port, MOTOR4_DIR_Pin, GPIO_PIN_SET);
	
	//MotorD对应的Timer打开及PWM设置						
	//PWM_SET_VALUE(,TIM_CHANNEL_1,2000);						//通道打开
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	htim14.Instance->CCR1 = 0;
	HAL_GPIO_WritePin(MOTORD_RESET_GPIO_Port, MOTORD_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTORD_MODE_GPIO_Port, MOTORD_MODE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTORD_DIR_GPIO_Port, MOTORD_DIR_Pin, GPIO_PIN_RESET);
    
	//4路电机Encoder对应的Timer打开
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	//4路电机对应的Timer计时打开
	
	
	//红外对应的Timer打开
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);							//红外PWM通道打开，频率38K，占空比0.2
	
	//蜂鸣器叫100ms
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	htim16.Instance->CCR1 = 300;
	//PWM_SET_VALUE(&htim16,TIM_CHANNEL_1,300);
	HAL_Delay(100);
	//PWM_SET_VALUE(&htim16,TIM_CHANNEL_1,0);
	htim16.Instance->CCR1 = 0;
	
	//启动ADC转换
//	memset(ADC_value, 0, sizeof(ADC_value));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_value, 32);
	
//	HAL_TIM_Base_Start(&htim12);
//	Robot_Boot_Power = 0;
//	htim12.Instance->CCR1 = 65535 - Robot_Boot_Power * 50;
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

////////////设置占空比PWM
//////////void PWM_SET_VALUE(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t value)
//////////{
//////////	TIM_OC_InitTypeDef sConfigOC;
//////////	uint32_t pluse = value;
//////////	sConfigOC.OCMode = TIM_OCMODE_PWM1;
//////////	sConfigOC.Pulse = pluse;
//////////	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//////////	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//////////	if(htim == &htim16){
//////////		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//////////		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//////////	}
//////////	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);
//////////	HAL_TIM_PWM_Start(htim, Channel);   
//////////}

//解包，到每个轮子的速度
void unpack(uint8_t *Packet){
    if((Packet[0] & 0xf0) == 0x40){
        if((robot_num == (Packet[1] & 0x0f)) && (Packet[0] & 0x08)){
            received_packet_flag = 1;
            Vx_package = (Packet[2] & 0x7f) + ((Packet[17] & 0xc0) << 1);
            Vx_package = (Packet[2] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[3] & 0x7f) + ((Packet[17] & 0x30) << 3);
            Vy_package = (Packet[3] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[4] & 0x7f) + ((Packet[17] & 0x0f) << 7);
            Vr_package = (Packet[4] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[1] >> 7;
            Robot_drib = (Packet[1] >> 4) & 0x03;
            Robot_Chip_Or_Shoot = ( Packet[1] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[21] & 0x7f;
            use_dir = Packet[21] & 0x80;
        }
        else if((robot_num == (Packet[5] & 0x0f)) && (Packet[0] & 0x04)){
            received_packet_flag = 1;
            Vx_package = (Packet[6] & 0x7f) + ((Packet[18] & 0xc0) << 1);
            Vx_package = (Packet[6] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[7] & 0x7f) + ((Packet[18] & 0x30) << 3);
            Vy_package = (Packet[7] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[8] & 0x7f) + ((Packet[18] & 0x0f) << 7);
            Vr_package = (Packet[8] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[5] >> 7;
            Robot_drib = (Packet[5] >> 4) & 0x03;
            Robot_Chip_Or_Shoot = ( Packet[5] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[22] & 0x7f;
            use_dir = Packet[22] & 0x80;
        }
        else if((robot_num == (Packet[9] & 0x0f)) && (Packet[0] & 0x02)){
            received_packet_flag = 1;
            Vx_package = (Packet[10] & 0x7f) + ((Packet[19] & 0xc0) << 1);
            Vx_package = (Packet[10] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[11] & 0x7f) + ((Packet[19] & 0x30) << 3);
            Vy_package = (Packet[11] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[12] & 0x7f) + ((Packet[19] & 0x0f) << 7);
            Vr_package = (Packet[12] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[9] >> 7;
            Robot_drib = (Packet[9] >> 4) & 0x03;
            Robot_Chip_Or_Shoot = ( Packet[9] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[23] & 0x7f;
            use_dir = Packet[23] & 0x80;
        }
        else if((robot_num == (Packet[13] & 0x0f)) && (Packet[0] & 0x01)){
            received_packet_flag = 1;
            Vx_package = (Packet[14] & 0x7f) + ((Packet[20] & 0xc0) << 1);
            Vx_package = (Packet[14] & 0x80) ? ( -Vx_package ) : Vx_package;
            Vy_package = (Packet[15] & 0x7f) + ((Packet[20] & 0x30) << 3);
            Vy_package = (Packet[15] & 0x80) ? ( -Vy_package ) : Vy_package;
            Vr_package = (Packet[16] & 0x7f) + ((Packet[20] & 0x0f) << 7);
            Vr_package = (Packet[16] & 0x80) ? ( -Vr_package ) : Vr_package;
            Robot_Is_Report = Packet[13] >> 7;
            Robot_drib = (Packet[13] >> 4) & 0x03;                                      
            Robot_Chip_Or_Shoot = ( Packet[13] >> 6 ) & 0x01;
            Robot_Boot_Power = Packet[24] & 0x7f;
            use_dir = Packet[24] & 0x80;
        }
        else 
            received_packet_flag = 0;
    }
        else
            received_packet_flag = 0;
};
 
void pack(uint8_t *TX_Packet){
    memset(TX_Packet, 0, 25);
    TX_Packet[0] = 0xff;
    TX_Packet[1] = 0x02;
    TX_Packet[2] = robot_num;
    TX_Packet[3] = Robot_Status;
    
    temp_bat = ((int32_t)AD_Battery_Last - 189600) / 50;
    
    if(temp_bat > 255){
        temp_bat = 255;
    }
    else if(temp_bat < 0){
        temp_bat = 0;
    }
    
    TX_Packet[4] = temp_bat;
    
		// Capacitor voltage: AD_Boot_Cap / 3.3 * 65536 / 10 * 1010
		temp_boot = ((int32_t)AD_Boot_Cap * 1010 / 196608);
		
		if (temp_boot < 0){
				temp_boot = 0;
		}
		else if (temp_boot > 255){
				temp_boot = 255;
		}
		
    #ifdef IMU_TEST_1
      memcpy(TX_Packet+14, IMU_RX_Buffer+4, 10);
    TX_Packet[24]= IMU_RX_Buffer[2];
  #endif
    
    TX_Packet[5] = temp_boot;

    //TX_Packet[6] = Robot_Boot_Power;
    //TX_Packet[7] = transmitted_packet;
    //uint32_t tick = HAL_GetTick();
    //TX_Packet[8] = tick >> 8 & 0xFF;
    //TX_Packet[9] = tick & 0xFF;
    //TX_Packet[10] = Left_Report_Package;
    int temp1 = Encoder_count_Motor1_avg;
    int temp2 = Encoder_count_Motor2_avg;
    int temp3 = Encoder_count_Motor3_avg;
    int temp4 = Encoder_count_Motor4_avg;
	TX_Packet[6] = (temp1 & 0xFF00)>>8;
    TX_Packet[7] = (temp1 & 0xFF);
	TX_Packet[8] = (temp2 & 0xFF00)>>8;
	TX_Packet[9] = (temp2 & 0xFF);
	TX_Packet[10] = (temp3 & 0xFF00)>>8;
	TX_Packet[11] = (temp3 & 0xFF);
	TX_Packet[12] = (temp4 & 0xFF00)>>8;
	TX_Packet[13] = (temp4 & 0xFF);
		


}

double normalize(double angle)
{
    const double M_2PI = PI * 2;
    // 快速粗调整
    angle -= (int)(angle / M_2PI) * M_2PI; 
    
    // 细调整 (-PI,PI]
    while( angle > PI ) {
        angle -= M_2PI;
    }

    while( angle <= -PI ) {
        angle += M_2PI;
    }

    return angle;
}

double setEdge(double angle, double edge) {
    // 角度限幅
    if (angle > edge)
    {
        angle = edge;
    }
    else if (angle < -edge)
    {
        angle = -edge;
    }
    return angle;
}

void motion_planner(void){
    int16_t acc_x = 0;
    int16_t acc_y = 0;
    double acc_whole = 0;
    double sin_x = 0;
    double sin_y = 0;
    if(sqrt(Vx_package_last * Vx_package_last + Vy_package_last * Vy_package_last) > 325.0)
    {
        acc_set = 25.0;	// 4.18 test
        DEC_FRAME++;
    }
    else
    {
        DEC_FRAME = 0;
        acc_set = 20.0;
    }
    acc_x = Vx_package - Vx_package_last;
    acc_y = Vy_package - Vy_package_last; 
    acc_whole = acc_x * acc_x + acc_y * acc_y ;
    acc_whole = sqrt(acc_whole);
    sin_x = acc_x / acc_whole;
    sin_y = acc_y / acc_whole;

    if (acc_whole > acc_set)
    {
        acc_whole = acc_set;
        acc_x = acc_whole * sin_x;
        acc_y = acc_whole * sin_y;
        Vx_package = Vx_package_last + acc_x;
        Vy_package = Vy_package_last + acc_y; 
    }
    
    //if((Vr_package - Vr_package_last)  >  acc_r_set) Vr_package = Vr_package_last + acc_r_set;
    //if((Vr_package - Vr_package_last)  < -acc_r_set) Vr_package = Vr_package_last - acc_r_set;
     //Vr_fuck = Vr_package;
    //Vr_package = Vr_package / 160.0;
    // Vr_fucked = Vr_package;
    
    // double type
    Vr_cal = Vr_package / 160.0;
    if(!DEC_FRAME)
    {
        Vx_package_last = Vx_package;
        Vy_package_last = Vy_package;
    }
    else
    {
        Vx_package_last += acc_x;
        Vy_package_last += acc_y;
    }
    
    // no use of v_r limit
  // Vr_package_last = Vr_package;
    
#ifdef IMU_TEST_1
    if(use_dir)
    {
        if(Robot_Is_Infrared == 0){
            rot_factor = powf(fabs((Vx_package_last)/100.0),2)+powf(fabs((Vy_package_last)/100.0),2);
            max_dir_acc = 6.5 - setEdge(rot_factor,4.5);
            max_dir_speed = 12 - setEdge(rot_factor,5.5);
        }
        else{
            rot_factor = powf(fabs((Vx_package_last)/75.0),2)+powf(fabs((Vy_package_last)/75.0),2);
            max_dir_acc = 5.0 - setEdge(rot_factor,4.5);
            max_dir_speed = 12 - setEdge(rot_factor,6.5);
        }
        dirKP = 7;
        dirKI = 0.5;
        dirKD = 5.5;

    dirNow = (IMU_RX_Buffer[4]) + 256 * (IMU_RX_Buffer[5]);
    if (dirNow > 32767) {
        dirNow -= 65535;
    }
        dirNow /= 100.0;
		dirNow = dirNow * PI / 180.0;
		dirEK = normalize(Vr_cal - dirNow);
    if(dirEK < 0.01 && dirEK > -0.01) dirEK = 0;
    dirD = dirEK - lastdirEK;
    lastdirEK = dirEK;
    dirKS += dirEK;
    dirKS = setEdge(dirKS, 2*PI);
    if (dirKS * dirEK < 0)
    {
        dirKS = 0;
    }
    rotVel = dirKP * dirEK + dirKI * dirKS + dirKD * dirD;
    rotVel = lastRotVel + setEdge(rotVel - lastRotVel, max_dir_acc);
    rotVel = setEdge(rotVel, max_dir_speed);
        
        
    Vr_cal = rotVel; // for compensation
    }
    else 
    {
        use_dir = 0;
    }
#endif
    
    Vel_Motor1 = ((sin_angle[0]/*+0.40108349*/) * Vx_package + (cos_angle[0]/*-0.01842361*/) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
    Vel_Motor2 = ((sin_angle[1]/*+0.27548794*/) * Vx_package + (cos_angle[1]/*-0.01842361*/) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
    Vel_Motor3 = ((sin_angle[2]/*-0.04107562*/) * Vx_package + (cos_angle[2]) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
    Vel_Motor4 = ((sin_angle[3]/*-0.64915591*/) * Vx_package + (cos_angle[3]) * Vy_package - 8.2 * Vr_cal) * Vel_k2;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
