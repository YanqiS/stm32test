/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/**
 * version .3.1
 * last change: add KL15 channel
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
//#include <Device_V1.h>
#include <OLED.h>
#include <USBPD.h>

#include <stm32g0xx_hal_fdcan.h>

////mylib
#include <EasySyslib.h>
//#include "ModBus.h"
//#include "TCHUBv3.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef *Flash_SPI;

UART_HandleTypeDef *Serial_Num;


#define ADC_CHANNELS 	6
#define LightSensr_Gate 	50
uint16_t adc_buffer[ADC_CHANNELS]={0};

int Version_A		= 0	;	//Ver  A.BC
int Version_B		= 1 ;
int Version_C		= 0 ;	//0.08

//#define Serial_Data_LENGTH		24	//24-1
uint16_t	Serial_Data_LENGTH = 24;

uint16_t CAN1_2Ser_ID[32];
uint16_t CAN2_2Ser_ID[32];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


//////// ////////system level

//sys time
uint8_t Enp_YEAR, Enp_MONTH, Enp_DAY, Enp_HOUR, Enp_MINUTE, Enp_SECOND;
uint8_t EncrypKey;

uint8_t dateTimeBuffer[7];

//W25Q64 	sector		block		page
//8M/64m	 256	x	 64		x	 256
int Sys_CNT, Sys_RunCNT,	Sys_CNT1, Sys_RunCNT1;
////	DispX0,DispX1,DispY0,DispY1;
uint32_t Sys_Addr_DispX0 	= 	0x001100;
uint32_t Sys_Addr_DispX1 	= 	0x001110;
uint32_t Sys_Addr_DispY0	=	0x001200;		//
uint32_t Sys_Addr_DispY1	=	0x001210;		//

uint32_t Sys_TIM_TICK;	//1-1000ms
bool	Sys_TIM_Flag;

//config
int8_t BoardID;
bool SW1_1, SW1_2, SW2_1, SW2_2;
int Mode_ID;	//IO_CFG_1/2/3/4	电阻上拉，拨码开关off时高电平，on时与GND导�?�，低电�??????????????????????????????????????????????????????????????????

//state
int sys_state = 0; //0 - 6; 	0 - none; 1- green	;2 - cyan(青色，天蓝）;3 - blue	;4 - yellow	;5 - purple	;6 - red
bool ERR = 0;
int Remote_state = 0;
uint8_t Web_ConnectSts;



//////// ////////app level

//Motor Control
struct MotorCtrl_TypeDef MotorCtrl_M1, MotorCtrl_M2, MotorCtrl_M3, MotorCtrl_M4;
uint16_t HostID = 0x01;
uint16_t M1_ID = 0x0D1;//D1	D2,	X
uint16_t M2_ID = 0x0D2;//D1	D2,	X
uint16_t M3_ID = 0x0D3;//D3	D4,	Y
uint16_t M4_ID = 0x0D5;//D5,	Z

uint8_t MotorInit_M1, MotorInit_M2, MotorInit_M3, MotorInit_M4;
bool MotorERR_M1, MotorERR_M2, MotorERR_M3, MotorERR_M4;

#define XmaxLimit 	200		//
#define YmaxLimit 	220		//

//#define DispX0 	100		//
//#define DispX1 	120		//
//#define DispY0 	100		//
//#define DispY1 	120		//
uint8_t	DispX0[4],DispX1[4],DispY0[4],DispY1[4];
int	DispX0_32b,DispX1_32b,DispY0_32b,DispY1_32b;

#define CANMsg_MNumb		32



//web232
struct Ser2CAN_Msg_TypeDef Ser2CAN_Msg[CANMsg_MNumb];
struct SerLoCtrl_Msg_TypeDef SerLoCtrl_Msg;
struct Bench_AckInfo_TypeDef Bench_Info;

//TA531
struct TA531_env_TypeDef TA531SysEnv;
struct TA531_TimCallback_TypeDef TA531TimCallback;
struct TA531_RobotCtrl_TypeDef TA531_RC1;

uint8_t	TA531_RC1_fg;	// 0-Error;	1-no task;	2-command received;	3-command accomplish;
uint8_t	TA531_RC1_x_ready,TA531_RC1_y_ready,TA531_RC1_z_ready;
uint8_t	TA531_RC1_Ack, TA531_Lock;

//stm32 header

FDCAN_TxHeaderTypeDef CAN_TxHeader[CANMsg_MNumb];
FDCAN_TxHeaderTypeDef FDCAN_TxHeader[CANMsg_MNumb];

FDCAN_TxHeaderTypeDef MotrCtrl_1_TxHeader,MotrCtrl_2_TxHeader,MotrCtrl_3_TxHeader,MotrCtrl_4_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader,FDCAN2_RxHeader,MotrCtrl_1_RxHeader,MotrCtrl_2_RxHeader,MotrCtrl_3_RxHeader;
FDCAN_FilterTypeDef FDCAN_Filter1[28],FDCAN_Filter2[28]; //
FDCAN_TxHeaderTypeDef TSA_Ack_Header,TSA1_ADC_Header, TSA2_LS_Header, TSA_Ack_RC_Header;

//ccommunication

uint8_t CH1RxData[8], CH2RxData[8];
uint8_t CH1TxData[8], CH2TxData[8], TSA1_ADC_DATA[8], TSA2_LS_DATA[8], TSA_Ack_DATA[8], TSA_Ack_RC_DATA[8];
uint8_t MotrCtrl_1_DATA[8], MotrCtrl_2_DATA[8], MotrCtrl_3_DATA[8], MotrCtrl_4_DATA[8];

uint8_t I2CRxData[REC_LENGTH];
uint8_t u1RxData[REC_LENGTH], u2RxData[REC_LENGTH], u3RxData[REC_LENGTH],
		u4RxData[REC_LENGTH], u5RxData[REC_LENGTH], u6RxData[REC_LENGTH],
		RxData[REC_LENGTH];
uint8_t Serial_TxData[REC_LENGTH] = {0};
uint8_t Bench_Info_TxData[REC_LENGTH] = {0};


uint32_t len;


bool rxflag;
bool U1RXFlag = 0, U2RXFlag = 0, U3RXFlag = 0, U4RXFlag = 0, U5RXFlag = 0,
		U6RXFlag = 0;//

bool TSA3_0x52_Flag = 0, TSA4_0x53_Flag = 0;


uint32_t adc_value;
uint32_t mVoltage;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void UART_RESET(UART_HandleTypeDef *huart);
void UART_Init(UART_HandleTypeDef *huart, uint32_t data_length);
void Ser2CAN(void);
uint32_t mRead_ADC1_ch(uint8_t ch);
void Sys_tune1();
void Sys_tune2();

void MotoCtrl_PackSend12();
//void MotoCtrl_PackSend2();
void MotoCtrl_PackSend3();
void MotoCtrl_PackSend4();
void MotoCtrl_PositionLoop(int PositionX_mm, int PositionY_mm);

//void CAN2Ser_Config(void);

uint8_t ByteEncryp(uint8_t byteData);

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

//
//  CAN2Ser_ID[3]=0xdf;
//  CAN2Ser_ID[5]=0x2;
//  CAN2Ser_ID[31]=0xff;
//  CAN2Ser_ID[1]=0xff;
//  CAN2Ser_ID[22]=0x11;
//
//  Order_MinF(32,CAN2Ser_ID);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */



  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_ADC1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  ///////use OLED for debug msg display

    HAL_Delay(300);
    OLED_Init(OLED_I2C_ch,OLED_type);
    OLED_Fill(OLED_I2C_ch,OLED_type, 0xff);
    HAL_Delay(1000);
    OLED_Fill(OLED_I2C_ch,OLED_type, 0x00);
    HAL_Delay(1000);

	HAL_FDCAN_MspInit(&hfdcan1);
	HAL_FDCAN_MspInit(&hfdcan2);

	Flash_SPI = &hspi1;

	TSA_Ack_Header.Identifier = 0x531;

	TSA_Ack_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA_Ack_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA_Ack_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA_Ack_Header.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	TSA_Ack_Header.IdType = FDCAN_STANDARD_ID;
	TSA_Ack_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA_Ack_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA_Ack_Header.MessageMarker = 0;


	TSA_Ack_RC_Header.Identifier = 0x532;

	TSA_Ack_RC_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA_Ack_RC_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA_Ack_RC_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA_Ack_RC_Header.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	TSA_Ack_RC_Header.IdType = FDCAN_STANDARD_ID;
	TSA_Ack_RC_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA_Ack_RC_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA_Ack_RC_Header.MessageMarker = 0;


	TSA2_LS_Header.Identifier = 0x51;

	TSA2_LS_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA2_LS_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA2_LS_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA2_LS_Header.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	TSA2_LS_Header.IdType = FDCAN_STANDARD_ID;
	TSA2_LS_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA2_LS_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA2_LS_Header.MessageMarker = 0;


	TSA1_ADC_Header.Identifier = 0x50;

	TSA1_ADC_Header.DataLength = FDCAN_DLC_BYTES_8;
	TSA1_ADC_Header.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	TSA1_ADC_Header.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	TSA1_ADC_Header.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	TSA1_ADC_Header.IdType = FDCAN_STANDARD_ID;
	TSA1_ADC_Header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	TSA1_ADC_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TSA1_ADC_Header.MessageMarker = 0;



	MotrCtrl_1_TxHeader.Identifier = M1_ID;

	MotrCtrl_1_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	MotrCtrl_1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	MotrCtrl_1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	MotrCtrl_1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	MotrCtrl_1_TxHeader.IdType = FDCAN_STANDARD_ID;
	MotrCtrl_1_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	MotrCtrl_1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	MotrCtrl_1_TxHeader.MessageMarker = 0;

	MotrCtrl_2_TxHeader.Identifier = M2_ID;

	MotrCtrl_2_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	MotrCtrl_2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	MotrCtrl_2_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	MotrCtrl_2_TxHeader.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	MotrCtrl_2_TxHeader.IdType = FDCAN_STANDARD_ID;
	MotrCtrl_2_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	MotrCtrl_2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	MotrCtrl_2_TxHeader.MessageMarker = 0;


	MotrCtrl_3_TxHeader.Identifier = M3_ID;

	MotrCtrl_3_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	MotrCtrl_3_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;	//FDCAN_FD_CAN;
	MotrCtrl_3_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;	//FDCAN_BRS_ON; //
	MotrCtrl_3_TxHeader.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	MotrCtrl_3_TxHeader.IdType = FDCAN_STANDARD_ID;
	MotrCtrl_3_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	MotrCtrl_3_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	MotrCtrl_3_TxHeader.MessageMarker = 0;




////////TIM6定时器，20ms周期

	HAL_TIM_Base_Start_IT(&htim6);

////////TIM7定时器，100ms周期

	HAL_TIM_Base_Start_IT(&htim7);

////////TIM7定时器，1000ms周期

	HAL_TIM_Base_Start_IT(&htim14);

////////TIM16定时器，5ms周期

	HAL_TIM_Base_Start_IT(&htim16);

	////////TIM17定时器，1ms周期

	HAL_TIM_Base_Start_IT(&htim17);


	// 读取PCF8563的时间和日期
	//PCF8563_ReadDateTime(dateTimeBuffer);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//Mode check
	bool id1,id2,id3,id4;
	id1 = HAL_GPIO_ReadPin(IO_CFG_1_GPIO_Port,IO_CFG_1_Pin);
	id2 = HAL_GPIO_ReadPin(GPIOA,IO_CFG_2_Pin);
	id3 = HAL_GPIO_ReadPin(GPIOA,IO_CFG_3_Pin);
	id4 = HAL_GPIO_ReadPin(GPIOA,IO_CFG_4_Pin);


	Mode_ID = (id1<<3) + (id2<<2) + (id3<<1) + (id4<<0);

	UART_Init(&huart1,Serial_Data_LENGTH);


	HAL_Delay(2000);
	//Version_A
	char *str = "MoC ";
	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 0, str );
	char str1[16] = {0};
	itoa(Version_A,str1,10);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,4, 0, str1 );
	OLED_ShowString(OLED_I2C_ch ,OLED_type,5, 0, ".");
	itoa(Version_B,str1,10);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,6, 0, str1);
	itoa(Version_C,str1,10);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,7, 0, str1);

//	str = "|cfg: ";
//	OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 0, str );
//	OLED_ShowString(OLED_I2C_ch ,OLED_type,14, 0," .");


//	HAL_UART_Transmit(&huart2, "ATP_TD_mini Ready~", strlen("ATP_TD_mini Ready~") ,0xff);
//	HAL_UART_Transmit_IT(&huart1, "Hello TA531~", strlen("Hello TA531~") );

	for (int i = 0; i < CANMsg_MNumb ; i++)	//
	{
		CAN_TxHeader[i].TxFrameType = FDCAN_DATA_FRAME;
		CAN_TxHeader[i].ErrorStateIndicator = FDCAN_ESI_PASSIVE;
		CAN_TxHeader[i].IdType = FDCAN_STANDARD_ID;
		CAN_TxHeader[i].MessageMarker = 0;
		CAN_TxHeader[i].TxEventFifoControl = FDCAN_NO_TX_EVENTS;

		FDCAN_TxHeader[i].TxFrameType = FDCAN_DATA_FRAME;
		FDCAN_TxHeader[i].ErrorStateIndicator = FDCAN_ESI_PASSIVE;
		FDCAN_TxHeader[i].IdType = FDCAN_STANDARD_ID;
		FDCAN_TxHeader[i].MessageMarker = 0;
		FDCAN_TxHeader[i].TxEventFifoControl = FDCAN_NO_TX_EVENTS;


		Ser2CAN_Msg[i].DataCycle = 0xffff;

		//		FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
		//		FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
		//		FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
		//		FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		//		FDCAN1_TxHeader.MessageMarker = 0;
		//		FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;	///*!< Data frame > < Remote frame */
	}

//	  HAL_Delay(500);


//	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 0, "DeviceID:");
//	uint16_t FlashID;
//	FlashID = SPI_Flash_GetID();
//	itoa(FlashID,str1,16);
//	OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 0, str1);

	uint8_t temp1[4],temp2[4];
	temp1[0] = 123;
	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Flash Test");
	while(temp1[0] != temp2[0])
	{
		SPI_Flash_WriteSomeBytes(temp1, Sys_Addr_DispX1, sizeof(int));
		HAL_Delay(1);
		SPI_Flash_ReadBytes(temp2, Sys_Addr_DispX1, sizeof(int));
		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Flash Test Err!");

		itoa(temp1[0],str1,10);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str1);	//123
		itoa(temp2[0],str1,10);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 2, str1);	//0

	}
	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Flash Test OK!");


//// ADC
	HAL_Delay(50);
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
    	char *str0 = "ADC_Calib_Error! ";
    	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, str0 );
        // 校准失败，处理错�????????????????????????????????????
        Error_Handler();
        while(1);
    }

    MX_DMA_Init();
    HAL_Delay(20);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);

    Web_ConnectSts = 255;



//    Sys_beep();
    Sys_tune1();


////42 MotorCtrl init

    MotorInit_M1 = 0;
    MotorInit_M2 = 0;
    MotorInit_M3 = 0;
    MotorInit_M4 = 0;


    MotorCtrl_M1.MotorCtrl_HostID = HostID;
    MotorCtrl_M1.MotorCtrl_FuncType = 0x01;
    MotorCtrl_M1.MotorCtrl_FuncCode = 0x01;
    MotorCtrl_M1.MotorCtrl_ByteData = 0x01;		//06/07
    MotorCtrl_M1.MotorCtrl_DataCode = 0x000000;

    MotorCtrl_M2.MotorCtrl_HostID = HostID;
    MotorCtrl_M2.MotorCtrl_FuncType = 0x01;
    MotorCtrl_M2.MotorCtrl_FuncCode = 0x01;
    MotorCtrl_M2.MotorCtrl_ByteData = 0x01;		//06/07
    MotorCtrl_M2.MotorCtrl_DataCode = 0x000000;

    MotorInit_M1 = 1;	//init wait
    MotorInit_M2 = 1;	//init wait
    MotoCtrl_PackSend12();
    HAL_Delay(500);

    while((MotorInit_M1 != 2)|(MotorInit_M2 != 2))
    {
    	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "M1&2 Init Wait " );
    	HAL_Delay(500);
    	MotoCtrl_PackSend12();
    }

    OLED_ShowString(OLED_I2C_ch ,OLED_type,9, 0, "1 2" );	// 9 11 13 15
    OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "M1&2 Init ok! " );

	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, "X: ");
	itoa(M1_ID,str1,16);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,4, 2, str1);
	itoa(TA531_RC1.TA531_RC_X_act ,str1,10);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 2, str1);


    MotorCtrl_M3.MotorCtrl_HostID = HostID;
    MotorCtrl_M3.MotorCtrl_FuncType = 0x01;
    MotorCtrl_M3.MotorCtrl_FuncCode = 0x01;
    MotorCtrl_M3.MotorCtrl_ByteData = 0x01;		//06/07
    MotorCtrl_M3.MotorCtrl_DataCode = 0x000000;

    MotorInit_M3 = 1;	//init wait
    MotoCtrl_PackSend3();
    HAL_Delay(500);

    while(MotorInit_M3 != 2)
    {
    	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "M3 Init Wait " );
    	HAL_Delay(500);
    	MotoCtrl_PackSend3();
    }

    OLED_ShowString(OLED_I2C_ch ,OLED_type,13, 0, "3 " );	// 9 11 13 15
    OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "M3 Initialized! " );

	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, "Y: ");
	itoa(M3_ID,str1,16);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,4, 3, str1);
	itoa(TA531_RC1.TA531_RC_Y_act ,str1,10);
	OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 3, str1);

    OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Check TouchPen! " );
	HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 1);	//F
	HAL_Delay(700);
	HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 0);	//F
    HAL_Delay(1000);


    OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "PuhUp Rst DispXY" );
    HAL_Delay(1000);

    if (HAL_GPIO_ReadPin(GPIOB,SW_UP_Pin) == 0)////reset display xy
	{
		Sys_tune1();
		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Hold Up 2s Reset" );
		HAL_Delay(800);
		if (HAL_GPIO_ReadPin(GPIOB,SW_UP_Pin) == 0)////reset display xy
		{
			Sys_tune1();
			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Hold Up 1s Reset" );
			HAL_Delay(800);
			Sys_tune2();
			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Release & Reset" );
			HAL_Delay(2000);

			if (HAL_GPIO_ReadPin(GPIOB,SW_UP_Pin) == 1)	// into reset display xy
			{
				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Up/Dw to Set X0" );
				HAL_Delay(800);
				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, "X0:");

				while(HAL_GPIO_ReadPin(GPIOF,SW_BUTTON_Pin) != 0)	//no push down
				{
					if (HAL_GPIO_ReadPin(GPIOB,SW_UP_Pin) == 0)
					{
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act +5;

						TA531_RC1_fg = 2;
					}
					else if (HAL_GPIO_ReadPin(GPIOB,SW_DOWN_Pin) == 0)
					{
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act -5;

						if(TA531_RC1.TA531_RC_X_trg < 0)
						{
							TA531_RC1.TA531_RC_X_trg = 0;
						}
						TA531_RC1_fg = 2;
					}

					MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);
					HAL_Delay(800);

					itoa(TA531_RC1.TA531_RC_X_trg ,str1,10);
					OLED_ShowString(OLED_I2C_ch ,OLED_type,3, 2, str1);

				}

				////push down
				DispX0_32b = TA531_RC1.TA531_RC_X_trg;
				DispX0[0] = TA531_RC1.TA531_RC_X_trg & 0xff;
				DispX0[1] = (TA531_RC1.TA531_RC_X_trg >>8) & 0xff;
				DispX0[2] = (TA531_RC1.TA531_RC_X_trg >>16) & 0xff;
				DispX0[3] = (TA531_RC1.TA531_RC_X_trg >>24) & 0xff;

				SPI_Flash_WriteSomeBytes(DispX0, Sys_Addr_DispX0, sizeof(int));
				HAL_Delay(2);
				uint8_t temp1[4];//temp3,temp4;
				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX0, sizeof(int));
				if ((temp1[0] != DispX0[0])	|(temp1[1] != DispX0[1])|(temp1[2] != DispX0[2])|(temp1[3] != DispX0[3]))
				{
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash Err!");
					while((temp1[0] != DispX0[0])	|(temp1[1] != DispX0[1])|(temp1[2] != DispX0[2])|(temp1[3] != DispX0[3]))
					{
						SPI_Flash_WriteSomeBytes(DispX0, Sys_Addr_DispX0, sizeof(int));
						HAL_Delay(1);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX0, sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash OK!");
				}


				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Up/Dw to Set X1" );
				HAL_Delay(800);
				OLED_ShowString(OLED_I2C_ch ,OLED_type,7, 2, "Xmax:");

				while(HAL_GPIO_ReadPin(GPIOF,SW_BUTTON_Pin) != 0)	//no push down
				{
					if (HAL_GPIO_ReadPin(GPIOB,SW_UP_Pin) == 0)
					{
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act +5;

						TA531_RC1_fg = 2;
					}
					else if (HAL_GPIO_ReadPin(GPIOB,SW_DOWN_Pin) == 0)
					{
						TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act -5;

						if(TA531_RC1.TA531_RC_X_trg < 0)
						{
							TA531_RC1.TA531_RC_X_trg = 0;
						}
						TA531_RC1_fg = 2;
					}

					MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);
					HAL_Delay(800);

					itoa(TA531_RC1.TA531_RC_X_trg ,str1,10);
					OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 2, str1);

				}


				////push down
				DispX1_32b = TA531_RC1.TA531_RC_X_trg;
				DispX1[0] = TA531_RC1.TA531_RC_X_trg & 0xff;
				DispX1[1] = (TA531_RC1.TA531_RC_X_trg >>8) & 0xff;
				DispX1[2] = (TA531_RC1.TA531_RC_X_trg >>16) & 0xff;
				DispX1[3] = (TA531_RC1.TA531_RC_X_trg >>24) & 0xff;

				SPI_Flash_WriteSomeBytes(DispX1, Sys_Addr_DispX1, sizeof(int));
				HAL_Delay(2);
//				int temp1;//temp3,temp4;
				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX1, sizeof(int));
				if ((temp1[0] != DispX1[0])	|(temp1[1] != DispX1[1])|(temp1[2] != DispX1[2])|(temp1[3] != DispX1[3]))
				{
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash Err!");
					while((temp1[0] != DispX1[0])	|(temp1[1] != DispX1[1])|(temp1[2] != DispX1[2])|(temp1[3] != DispX1[3]))
					{
						SPI_Flash_WriteSomeBytes(DispX1, Sys_Addr_DispX1, sizeof(int));
						HAL_Delay(1);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispX1, sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash OK!");
				}


				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "L/R to Set Y0" );
				HAL_Delay(800);
				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, "Y0:");

				while(HAL_GPIO_ReadPin(GPIOF,SW_BUTTON_Pin) != 0)	//no push down
				{
					if (HAL_GPIO_ReadPin(GPIOB,SW_LEFT_Pin) == 0)
					{
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act -5;

						if(TA531_RC1.TA531_RC_Y_trg < 0)
						{
							TA531_RC1.TA531_RC_Y_trg = 0;
						}

						TA531_RC1_fg = 2;
					}
					else if (HAL_GPIO_ReadPin(GPIOF,SW_RIGHT_Pin) == 0)
					{
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act +5;

						TA531_RC1_fg = 2;

					}

					MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);
					HAL_Delay(800);

					itoa(TA531_RC1.TA531_RC_Y_trg ,str1,10);
					OLED_ShowString(OLED_I2C_ch ,OLED_type,3, 3, str1);

				}

				////push down
				DispY0_32b = TA531_RC1.TA531_RC_Y_trg;
				DispY0[0] = TA531_RC1.TA531_RC_Y_trg & 0xff;
				DispY0[1] = (TA531_RC1.TA531_RC_Y_trg >>8) & 0xff;
				DispY0[2] = (TA531_RC1.TA531_RC_Y_trg >>16) & 0xff;
				DispY0[3] = (TA531_RC1.TA531_RC_Y_trg >>24) & 0xff;
				SPI_Flash_WriteSomeBytes(DispY0, Sys_Addr_DispY0, sizeof(int));
				HAL_Delay(2);
//				int temp1;//temp3,temp4;
				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY0, sizeof(int));
				if ((temp1[0] != DispY0[0])	|(temp1[1] != DispY0[1])|(temp1[2] != DispY0[2])|(temp1[3] != DispY0[3]))
				{
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash Err!");
					while ((temp1[0] != DispY0[0])	|(temp1[1] != DispY0[1])|(temp1[2] != DispY0[2])|(temp1[3] != DispY0[3]))
					{
						SPI_Flash_WriteSomeBytes(DispY0, Sys_Addr_DispY0, sizeof(int));
						HAL_Delay(1);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY0, sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash OK!");
				}


				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "L/R to Set Ymax" );
				HAL_Delay(800);
				OLED_ShowString(OLED_I2C_ch ,OLED_type,7, 3, "Ymax:");

				while(HAL_GPIO_ReadPin(GPIOF,SW_BUTTON_Pin) != 0)	//no push down
				{
					if (HAL_GPIO_ReadPin(GPIOB,SW_LEFT_Pin) == 0)
					{
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act -5;

						if(TA531_RC1.TA531_RC_Y_trg < 0)
						{
							TA531_RC1.TA531_RC_Y_trg = 0;
						}

						TA531_RC1_fg = 2;
					}
					else if (HAL_GPIO_ReadPin(GPIOF,SW_RIGHT_Pin) == 0)
					{
						TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act +5;

						TA531_RC1_fg = 2;

					}

					MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);
					HAL_Delay(800);

					itoa(TA531_RC1.TA531_RC_Y_trg ,str1,10);
					OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 3, str1);

				}
				////push down
				DispY1_32b = TA531_RC1.TA531_RC_Y_trg;
				DispY1[0] = TA531_RC1.TA531_RC_Y_trg & 0xff;
				DispY1[1] = (TA531_RC1.TA531_RC_Y_trg >>8) & 0xff;
				DispY1[2] = (TA531_RC1.TA531_RC_Y_trg >>16) & 0xff;
				DispY1[3] = (TA531_RC1.TA531_RC_Y_trg >>24) & 0xff;
				SPI_Flash_WriteSomeBytes(DispY1, Sys_Addr_DispY1, sizeof(int));
				HAL_Delay(2);
//				int temp1;//temp3,temp4;
				SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY1, sizeof(int));
				if ((temp1[0] != DispY1[0])	|(temp1[1] != DispY1[1])|(temp1[2] != DispY1[2])|(temp1[3] != DispY1[3]))
				{
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash Err!");
					while((temp1[0] != DispY1[0])	|(temp1[1] != DispY1[1])|(temp1[2] != DispY1[2])|(temp1[3] != DispY1[3]))
					{
						SPI_Flash_WriteSomeBytes(DispY1, Sys_Addr_DispY1, sizeof(int));
						HAL_Delay(1);
						SPI_Flash_ReadBytes(temp1, Sys_Addr_DispY1, sizeof(int));
					}
					OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "WriteFlash OK!");
				}


				////Display Area reset finish		//DispX0,DispX1,DispY0,DispY1;

				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Reset XY finish!");




				TA531_RC1.TA531_RC_X_trg =	DispX0_32b;
				TA531_RC1.TA531_RC_Y_trg =	DispY0_32b;
//				TA531_RC1.TA531_RC_Z_code = 1;
//				TA531_RC1.TA531_RC_Z_code2 = 0;
				TA531_RC1_fg = 2;
				MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);

				HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 1);	//touch pen push
				HAL_Delay(700);
				HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 0);	//touch pen release
			    HAL_Delay(1000);


				TA531_RC1.TA531_RC_X_trg =	DispX1_32b;
				TA531_RC1.TA531_RC_Y_trg =	DispY1_32b;
				TA531_RC1_fg = 2;
				MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);

				HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 1);	//touch pen push
				HAL_Delay(700);
				HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 0);	//touch pen release
			    HAL_Delay(1000);

			}	//finish reset display xy
		}	//reset display xy comfirm
	}	//reset display xy comfirm
    else
    {	//DispX0,DispX1,DispY0,DispY1;
    	SPI_Flash_ReadBytes(DispX0, Sys_Addr_DispX0, sizeof(int));
    	SPI_Flash_ReadBytes(DispY0, Sys_Addr_DispY0, sizeof(int));
    	SPI_Flash_ReadBytes(DispX1, Sys_Addr_DispX1, sizeof(int));
    	SPI_Flash_ReadBytes(DispY1, Sys_Addr_DispY1, sizeof(int));

    	DispX0_32b = DispX0[0] + (DispX0[1]<<8) + (DispX0[2]<<16) + (DispX0[3]<<24);
    	DispX1_32b = DispX1[0] + (DispX1[1]<<8) + (DispX1[2]<<16) + (DispX1[3]<<24);
    	DispY0_32b = DispY0[0] + (DispY0[1]<<8) + (DispY0[2]<<16) + (DispY0[3]<<24);
    	DispY1_32b = DispY1[0] + (DispY1[1]<<8) + (DispY1[2]<<16) + (DispY1[3]<<24);

    	HAL_Delay(10);
    	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Read XY fm Flash");
    	HAL_Delay(200);

    	if ((DispX0_32b > 0 ) &(DispX0_32b < DispX1_32b )&(DispX1_32b < XmaxLimit )&(DispY0_32b > 0) &(DispY0_32b < DispY1_32b) &(DispY1_32b < YmaxLimit))
    	{
    		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "XY Check Pass!");
    	}
    	else
    	{
    		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "XY Check Fail!");
    		Sys_tune1();
    		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, "Pls Reboot &");
    		Sys_tune1();
    		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, "Reset XY Area");
    		Sys_tune2();

    		while(1);
    	}
    }

//////finish reset display xy


	TA531_RC1_fg = 2;
	TA531_RC1.TA531_RC_X_trg = 0;
	TA531_RC1.TA531_RC_Y_trg = 0;
	MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);


	while (1)
	{
		////
		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Rds x");
		itoa(TA531_RC1_x_ready ,str1,10);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,5, 1, str1);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,6, 1, "y");
		itoa(TA531_RC1_y_ready ,str1,10);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,7, 1, str1);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 1, "z");
		itoa(TA531_RC1_z_ready ,str1,10);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,9, 1, str1);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 1, "Fg");
		itoa(TA531_RC1_fg ,str1,10);
		OLED_ShowString(OLED_I2C_ch ,OLED_type,15, 1, str1);


		if ((TA531_RC1.TA531_RC_X_act == TA531_RC1.TA531_RC_X_trg))
		{
			TA531_RC1_x_ready = 1;
		}
		else
		{
			TA531_RC1_x_ready = 0;
		}
		if ((TA531_RC1.TA531_RC_Y_act == TA531_RC1.TA531_RC_Y_trg))
		{
			TA531_RC1_y_ready = 1;
		}
		else
		{
			TA531_RC1_y_ready = 0;
		}


		while ((TA531_RC1_fg == 2)&((TA531_RC1_x_ready& TA531_RC1_y_ready) !=1 ))
		{

			MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);

			itoa(TA531_RC1.TA531_RC_X_trg ,str1,10);
			OLED_ShowString(OLED_I2C_ch ,OLED_type,13, 2, str1);
			itoa(TA531_RC1.TA531_RC_Y_trg ,str1,10);
			OLED_ShowString(OLED_I2C_ch ,OLED_type,13, 3, str1);

			HAL_Delay(100);

			if ((TA531_RC1.TA531_RC_X_act == TA531_RC1.TA531_RC_X_trg))
			{
				TA531_RC1_x_ready = 1;
			}
			else
			{
				TA531_RC1_x_ready = 0;
			}
			if ((TA531_RC1.TA531_RC_Y_act == TA531_RC1.TA531_RC_Y_trg))
			{
				TA531_RC1_y_ready = 1;
			}
			else
			{
				TA531_RC1_y_ready = 0;
			}
		}

//		MotoCtrl_PositionLoop(TA531_RC1.TA531_RC_X_trg , TA531_RC1.TA531_RC_Y_trg);
//		HAL_Delay(20);


		if((TA531_RC1_x_ready == 1 )&(TA531_RC1_y_ready == 1 )&(TA531_RC1_fg < 3) )
		{	//reach 1st point

			itoa(TA531_RC1.TA531_RC_X_act ,str1,10);
			OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 2, str1);
			itoa(TA531_RC1.TA531_RC_Y_act ,str1,10);
			OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 3, str1);


			if(TA531_RC1.TA531_RC_Z_code > 0)	//(TA531_RC1.TA531_RC_Z_code2 != TA531_RC1.TA531_RC_Z_code)
			{
				switch(TA531_RC1.TA531_RC_Z_code)
				{
					case 0:		//none
						HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 0);
						break;
					case 1:
						HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 1);	//F
						HAL_Delay(250);
						break;
					case 2:
						HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 1);	//F
						HAL_Delay(2000);
						break;
					case 3:
						HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 1);	//F
						HAL_Delay(5000);
						break;
				}

				if ((TA531_RC1.TA531_RC_X_Mov != 0)|(TA531_RC1.TA531_RC_Y_Mov != 0))
				{
					MotoCtrl_PositionLoop((int)(TA531_RC1.TA531_RC_X_trg + TA531_RC1.TA531_RC_X_Mov), (int)(TA531_RC1.TA531_RC_Y_trg  + TA531_RC1.TA531_RC_Y_Mov));
					HAL_Delay(1200);
				}

				HAL_GPIO_WritePin(GPIOF, KL15_RELAY_Pin, 0);	//F
				TA531_RC1.TA531_RC_Z_code2 = TA531_RC1.TA531_RC_Z_code;		// 0
				TA531_RC1.TA531_RC_X_Mov = 0;
				TA531_RC1.TA531_RC_Y_Mov = 0;


				TA531_RC1.TA531_RC_Z_code = 0;
			}

			if (TA531_RC1.TA531_RC_Reset == 1)
			{
				TA531_RC1.TA531_RC_X_trg = 0;
				TA531_RC1.TA531_RC_Y_trg = 0;
				MotoCtrl_PositionLoop( 0 , 0 );
				HAL_Delay(1000);
			}

			TA531_RC1_fg = 3;

			TSA_Ack_RC_DATA[0] = TA531_RC1_Ack & 0x0f;
			TSA_Ack_RC_DATA[3] = TA531_RC1.TA531_RC_X_act & 0xff;
			TSA_Ack_RC_DATA[4] = (TA531_RC1.TA531_RC_X_act >> 8) & 0xff;
			TSA_Ack_RC_DATA[5] = TA531_RC1.TA531_RC_Y_act & 0xff;
			TSA_Ack_RC_DATA[6] = (TA531_RC1.TA531_RC_Y_act >> 8) & 0xff;
			TSA_Ack_RC_DATA[7] = 0xff;

			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_RC_Header, TSA_Ack_RC_DATA);
		}

		if (TA531_Lock == 0)
		{
			if (HAL_GPIO_ReadPin(GPIOB,SW_UP_Pin) == 0)
			{
				OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 1, "Go X+");

				TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act +50;

				TA531_RC1_fg = 2;

//						TA531_RC1_x_ready = 0;
//						TA531_RC1_y_ready = 0;
			}
			else if (HAL_GPIO_ReadPin(GPIOB,SW_DOWN_Pin) == 0)
			{
				OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 1, "Go X-");

				TA531_RC1.TA531_RC_X_trg = TA531_RC1.TA531_RC_X_act -20;

				if(TA531_RC1.TA531_RC_X_trg < 0)
				{
					TA531_RC1.TA531_RC_X_trg = 0;
				}

				TA531_RC1_fg = 2;

			}
			else if (HAL_GPIO_ReadPin(GPIOB,SW_LEFT_Pin) == 0)
			{
				OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 1, "Go Y-");

				TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act -20;

				if(TA531_RC1.TA531_RC_Y_trg < 0)
				{
					TA531_RC1.TA531_RC_Y_trg = 0;
				}

				TA531_RC1_fg = 2;

			}
			else if (HAL_GPIO_ReadPin(GPIOF,SW_RIGHT_Pin) == 0)
			{
				OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 1, "Go Y+");

				TA531_RC1.TA531_RC_Y_trg = TA531_RC1.TA531_RC_Y_act +50;

				TA531_RC1_fg = 2;

			}
			else if (HAL_GPIO_ReadPin(GPIOF,SW_BUTTON_Pin) == 0)
			{
				OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 1, "Push!");

				TA531_RC1.TA531_RC_Z_code = 1;
				TA531_RC1.TA531_RC_Z_code2 = 0;

				TA531_RC1_fg = 2;

			}
		}




	    HAL_Delay(5);





///////test flash

//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, "DeviceID:");
//		uint16_t FlashID;
//		FlashID = SPI_Flash_GetID();
//		itoa(FlashID,str1,16);
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,10, 2, str1);

//		HAL_Delay(500);

////////test end


//		SPI_Flash_WriteSomeBytes(&Sys_CNT, Sys_CNT_Addr, sizeof(Sys_CNT));
//		SPI_Flash_WriteSomeBytes(&Sys_RunCNT, Sys_RunCNT_Addr, sizeof(Sys_RunCNT));
//
//		uint8_t str_CNT[32] = {0};
//		uint8_t str_RunCNT[32] = {0};
//		SPI_Flash_ReadBytes(str_CNT, Sys_CNT_Addr, sizeof(Sys_CNT));
//		SPI_Flash_ReadBytes(str_RunCNT, Sys_RunCNT_Addr, sizeof(Sys_RunCNT));
//
//		char strCNT;
//		itoa(Sys_CNT, &strCNT,10);
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 0, "Sys_CNT: ");
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, &strCNT);
//
//		char strRunCNT;
//		itoa(Sys_RunCNT, &strRunCNT,10);
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, "Sys_RunCNT: ");
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, &strRunCNT);


//		for (int i = 0; i < CANMsg_MNumb ; i++)
//		{
//
//			if(Ser2CAN_Msg[i].DataType == 0x01)	// CAN
//			{
//				CAN_TxHeader[i].Identifier = Ser2CAN_Msg[i].DataID;
//
//				CAN_TxHeader[i].FDFormat = FDCAN_CLASSIC_CAN;
//				CAN_TxHeader[i].BitRateSwitch = FDCAN_BRS_OFF;
//
//				CAN_TxHeader[i].DataLength = Ser2CAN_Msg[i].DataValueLength;
//			}
//			else if(Ser2CAN_Msg[i].DataType == 0x02)	// CAN FD
//			{
//				FDCAN_TxHeader[i].Identifier = Ser2CAN_Msg[i].DataID;
//
//				FDCAN_TxHeader[i].FDFormat = FDCAN_FD_CAN;
//				FDCAN_TxHeader[i].BitRateSwitch = FDCAN_BRS_ON;
//
//				FDCAN_TxHeader[i].DataLength = Ser2CAN_Msg[i].DataValueLength;
//			}
//
//		}

//		if (Sys_TIM_Flag == 1)
//		{
//
//			for (int i = 0; i < CANMsg_MNumb ; i++)
//			{
//
//				if ( (Sys_TIM_TICK > 0) & ((Sys_TIM_TICK % Ser2CAN_Msg[i].DataCycle) == 0 ) & (Ser2CAN_Msg[i].DataID > 0) & (Ser2CAN_Msg[i].TriggerType >0 )  )
//				{
//					if(Ser2CAN_Msg[i].DataChannel == 1)
//					{
//						HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeader[i], Ser2CAN_Msg[i].DataValue );
//					}
//					else if(Ser2CAN_Msg[i].DataChannel == 2)
//					{
//						HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN_TxHeader[i], Ser2CAN_Msg[i].FDDataValue );
//					}
//
//					char str1[3] = {0};
//					itoa(Ser2CAN_Msg[i].DataID ,str1,16);
//					OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 2, str1 );
//
//
//					if((Ser2CAN_Msg[i].TriggerType < 0x3F) & (Ser2CAN_Msg[i].TriggerType > 0))
//					{
//						Ser2CAN_Msg[i].TriggerType --;
//					}
//				}
//			}
//			Sys_TIM_Flag = 0;
//
//		}



//		if ( (SerLoCtrl_Msg.LocalRelay[0] != 3) & (SerLoCtrl_Msg.LocalRelay[1] != 3) & (SerLoCtrl_Msg.LocalRelay[2] != 3) & (SerLoCtrl_Msg.LocalRelay[3] != 3) )
//		{
//			if(SerLoCtrl_Msg.LocalRelay[0] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOE, KL15_RELAY_Pin, 1);	//F
//
//				char *str = "*KL15 Relay ON";
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[0] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOE, KL15_SW_Pin, 0);
//
//				char *str = "*KL15 Relay OFF";
//				OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
//			}
//
//			if(SerLoCtrl_Msg.LocalRelay[1] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_1_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[1] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_1_Pin, 0);
//			}
//
//			if(SerLoCtrl_Msg.LocalRelay[2] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_2_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[2] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_2_Pin, 0);
//			}
//
//			if(SerLoCtrl_Msg.LocalRelay[3] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_3_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[3] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_3_Pin, 0);
//			}
//		}

//		if ( (SerLoCtrl_Msg.LocalRelay[4] != 3) & (SerLoCtrl_Msg.LocalRelay[5] != 3) & (SerLoCtrl_Msg.LocalRelay[6] != 3) & (SerLoCtrl_Msg.LocalRelay[7] != 3) )
//		{
//			if(SerLoCtrl_Msg.LocalRelay[4] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_4_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[4] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_4_Pin, 0);
//			}
//
//			if(SerLoCtrl_Msg.LocalRelay[5] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_5_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[5] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_5_Pin, 0);
//			}
//
//			if(SerLoCtrl_Msg.LocalRelay[6] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_6_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[6] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_6_Pin, 0);
//			}
//
//			if(SerLoCtrl_Msg.LocalRelay[7] == 1)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_7_Pin, 1);
//			}
//			else if(SerLoCtrl_Msg.LocalRelay[7] == 2)
//			{
//				HAL_GPIO_WritePin(GPIOD, OUT_7_Pin, 0);
//			}
//		}



	}  //while

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 32773;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0xFFFF;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */
	//hcrc.Init.GeneratingPolynomial = 32773; //0x8005
  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 5;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 16;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 28;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  for(uint8_t i=0;i<28;i++)
  {
	  FDCAN_Filter1[i].IdType = FDCAN_STANDARD_ID;
	  FDCAN_Filter1[i].FilterIndex = i;
	  FDCAN_Filter1[i].FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	  FDCAN_Filter1[i].FilterType = FDCAN_FILTER_RANGE;
	  FDCAN_Filter1[i].FilterID1 = 0x0000;
	  FDCAN_Filter1[i].FilterID2 = 0x0000;
  }

  //24,25,26,27
  FDCAN_Filter1[1].FilterID1 = 0x0064;
  FDCAN_Filter1[1].FilterID2 = 0x0064;



  for(uint8_t i=0;i<28;i++)
  {
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter1[i]) != HAL_OK) //过滤器初始化
	{
		Error_Handler();
	}
  }

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE,
			DISABLE); //FDCAN_ACCEPT_IN_RX_FIFO0
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 64;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 5;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 16;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 5;
  hfdcan2.Init.DataTimeSeg2 = 2;
  hfdcan2.Init.StdFiltersNbr = 28;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  for(uint8_t i=0;i<28;i++)
  {
	  FDCAN_Filter2[i].IdType = FDCAN_STANDARD_ID;
	  FDCAN_Filter2[i].FilterIndex = i;
	  FDCAN_Filter2[i].FilterConfig = FDCAN_FILTER_TO_RXFIFO1;

	  FDCAN_Filter2[i].FilterType = FDCAN_FILTER_RANGE;
	  FDCAN_Filter2[i].FilterID1 = 0x0000;
	  FDCAN_Filter2[i].FilterID2 = 0x0000;
  }

  FDCAN_Filter2[1].FilterID1 = 0x0001;
  FDCAN_Filter2[1].FilterID2 = 0x0010;

  for(uint8_t i=0;i<28;i++)
  {
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_Filter2[i]) != HAL_OK) //滤波器初始化
	{
		Error_Handler();
	}
  }

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, DISABLE,
			DISABLE); //FDCAN_ACCEPT_IN_RX_FIFO0
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

	//HAL_UART_Transmit_IT(&huart1, " HAL_FDCAN2_Start! ", strlen(" HAL_FDCAN2_Start! "));
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10707DBC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 64000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 64000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 64000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 63;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 64000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 64-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */


  HAL_UART_Receive_IT(&huart1,u1RxData,Serial_Data_LENGTH);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DOOR_RELAY_HSD1_Pin|DOOR_RELAY_HSD2_Pin|FL_WINDORELAY_3_Pin|FL_WINDORELAY_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DOOR_RELAY_RR_Pin|DOOR_RELAY_FR_Pin|COM_RELAY_1_Pin|COM_RELAY_2_Pin
                          |COM_RELAY_3_Pin|COM_RELAY_4_Pin|SPI1_DC_Pin|SPI_Flash_NSS_Pin
                          |COM_RELAY_5_Pin|IO_SYS_LED_B_Pin|IO_SYS_LED_G_Pin|IO_SYS_LED_R_Pin
                          |DOOR_RELAY_FL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, USB_RELAY_1_Pin|USB_RELAY_2_Pin|Y_RELAY_6_Pin|KL15_RELAY_Pin
                          |FR_WINDORELAY_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Y_RELAY_1_Pin|Y_RELAY_2_Pin|Y_RELAY_3_Pin|Y_RELAY_4_Pin
                          |FL_WINDORELAY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Y_RELAY_5_Pin|COM_RELAY_6_Pin|COM_RELAY_7_Pin|FL_WINDORELAY_1_Pin
                          |DOOR_RELAY_RL_Pin|DOOR_RELAY_F_Pin|DOOR_RELAY_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RL_WINDORELAY_4_Pin|RL_WINDORELAY_3_Pin|RL_WINDORELAY_2_Pin|RL_WINDORELAY_1_Pin
                          |RR_WINDORELAY_1_Pin|RR_WINDORELAY_2_Pin|RR_WINDORELAY_3_Pin|RR_WINDORELAY_4_Pin
                          |FR_WINDORELAY_4_Pin|FR_WINDORELAY_3_Pin|FR_WINDORELAY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DOOR_RELAY_HSD1_Pin DOOR_RELAY_HSD2_Pin FL_WINDORELAY_3_Pin FL_WINDORELAY_4_Pin */
  GPIO_InitStruct.Pin = DOOR_RELAY_HSD1_Pin|DOOR_RELAY_HSD2_Pin|FL_WINDORELAY_3_Pin|FL_WINDORELAY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DOOR_RELAY_RR_Pin DOOR_RELAY_FR_Pin COM_RELAY_1_Pin COM_RELAY_2_Pin
                           COM_RELAY_3_Pin COM_RELAY_4_Pin SPI1_DC_Pin COM_RELAY_5_Pin
                           DOOR_RELAY_FL_Pin */
  GPIO_InitStruct.Pin = DOOR_RELAY_RR_Pin|DOOR_RELAY_FR_Pin|COM_RELAY_1_Pin|COM_RELAY_2_Pin
                          |COM_RELAY_3_Pin|COM_RELAY_4_Pin|SPI1_DC_Pin|COM_RELAY_5_Pin
                          |DOOR_RELAY_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_RELAY_1_Pin USB_RELAY_2_Pin Y_RELAY_6_Pin KL15_RELAY_Pin
                           FR_WINDORELAY_1_Pin */
  GPIO_InitStruct.Pin = USB_RELAY_1_Pin|USB_RELAY_2_Pin|Y_RELAY_6_Pin|KL15_RELAY_Pin
                          |FR_WINDORELAY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_RELAY_1_Pin Y_RELAY_2_Pin Y_RELAY_3_Pin Y_RELAY_4_Pin
                           FL_WINDORELAY_2_Pin */
  GPIO_InitStruct.Pin = Y_RELAY_1_Pin|Y_RELAY_2_Pin|Y_RELAY_3_Pin|Y_RELAY_4_Pin
                          |FL_WINDORELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Y_RELAY_5_Pin COM_RELAY_6_Pin COM_RELAY_7_Pin FL_WINDORELAY_1_Pin
                           DOOR_RELAY_RL_Pin DOOR_RELAY_F_Pin DOOR_RELAY_R_Pin */
  GPIO_InitStruct.Pin = Y_RELAY_5_Pin|COM_RELAY_6_Pin|COM_RELAY_7_Pin|FL_WINDORELAY_1_Pin
                          |DOOR_RELAY_RL_Pin|DOOR_RELAY_F_Pin|DOOR_RELAY_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_Flash_NSS_Pin */
  GPIO_InitStruct.Pin = SPI_Flash_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_Flash_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RL_WINDORELAY_4_Pin RL_WINDORELAY_3_Pin RL_WINDORELAY_2_Pin RL_WINDORELAY_1_Pin
                           RR_WINDORELAY_1_Pin RR_WINDORELAY_2_Pin RR_WINDORELAY_3_Pin RR_WINDORELAY_4_Pin
                           FR_WINDORELAY_4_Pin FR_WINDORELAY_3_Pin FR_WINDORELAY_2_Pin */
  GPIO_InitStruct.Pin = RL_WINDORELAY_4_Pin|RL_WINDORELAY_3_Pin|RL_WINDORELAY_2_Pin|RL_WINDORELAY_1_Pin
                          |RR_WINDORELAY_1_Pin|RR_WINDORELAY_2_Pin|RR_WINDORELAY_3_Pin|RR_WINDORELAY_4_Pin
                          |FR_WINDORELAY_4_Pin|FR_WINDORELAY_3_Pin|FR_WINDORELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_CFG_1_Pin */
  GPIO_InitStruct.Pin = IO_CFG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO_CFG_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IO_CFG_2_Pin IO_CFG_3_Pin IO_CFG_4_Pin */
  GPIO_InitStruct.Pin = IO_CFG_2_Pin|IO_CFG_3_Pin|IO_CFG_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_RIGHT_Pin SW_BUTTON_Pin */
  GPIO_InitStruct.Pin = SW_RIGHT_Pin|SW_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_UP_Pin SW_LEFT_Pin SW_DOWN_Pin */
  GPIO_InitStruct.Pin = SW_UP_Pin|SW_LEFT_Pin|SW_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IO_SYS_LED_B_Pin IO_SYS_LED_G_Pin IO_SYS_LED_R_Pin */
  GPIO_InitStruct.Pin = IO_SYS_LED_B_Pin|IO_SYS_LED_G_Pin|IO_SYS_LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB6);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB7);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB8);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (hfdcan == &hfdcan1) 	//CAN1 Classic CAN
		{
			uint8_t buf_rec[8];

			while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0))
			{
				HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN1_RxHeader,	buf_rec);
			}

			TA531_Lock = 1;

			if (FDCAN1_RxHeader.Identifier == 0x052)	//TSA_3
			{
				TA531SysEnv.TA531_env_KL15 = buf_rec[0]&0x03;
				TA531SysEnv.TA531_env_USB1 = buf_rec[1]&0x0F;
				TA531SysEnv.TA531_env_KeyLock = (buf_rec[2]>>0)&0x03;
				TA531SysEnv.TA531_env_KeyUnlock = (buf_rec[2]>>2)&0x03;
				TA531SysEnv.TA531_env_KeyRearDoor = (buf_rec[2]>>4)&0x03;
				TA531SysEnv.TA531_env_KeyBeep = (buf_rec[2]>>6)&0x03;

				TA531SysEnv.TA531_env_KeyWindow = (buf_rec[3]>>0)&0x03;
				TA531SysEnv.TA531_env_KeyLeftDoor = (buf_rec[3]>>2)&0x03;
				TA531SysEnv.TA531_env_KeyRightDoor = (buf_rec[3]>>4)&0x03;

				TA531SysEnv.TA531_env_Relay1 = (buf_rec[4]>>0)&0x03;
				TA531SysEnv.TA531_env_Relay2 = (buf_rec[4]>>2)&0x03;
				TA531SysEnv.TA531_env_Relay3 = (buf_rec[4]>>4)&0x03;
				TA531SysEnv.TA531_env_Relay4 = (buf_rec[4]>>6)&0x03;
				TA531SysEnv.TA531_env_Relay5 = (buf_rec[5]>>0)&0x03;
				TA531SysEnv.TA531_env_Relay6 = (buf_rec[5]>>2)&0x03;

				//TA531 Ack
				TSA_Ack_DATA[3] = 1;
				TSA3_0x52_Flag = 1;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_Header, TSA_Ack_DATA);
				TSA_Ack_DATA[3] = 0;
			}
			else if (FDCAN1_RxHeader.Identifier == 0x053)	//TSA_4
			{
				TA531SysEnv.TA531_env_WindowFL = (buf_rec[0]>>0)&0x0F;
				TA531SysEnv.TA531_env_WindowFR = (buf_rec[0]>>4)&0x0F;
				TA531SysEnv.TA531_env_WindowRL = (buf_rec[1]>>0)&0x0F;
				TA531SysEnv.TA531_env_WindowRR = (buf_rec[1]>>4)&0x0F;

				TA531SysEnv.TA531_env_DoorSwF = (buf_rec[4]>>0)&0x03;
				TA531SysEnv.TA531_env_DoorSwR = (buf_rec[4]>>2)&0x03;
				TA531SysEnv.TA531_env_DoorSwFL = (buf_rec[5]>>0)&0x03;
				TA531SysEnv.TA531_env_DoorSwFR = (buf_rec[5]>>2)&0x03;
				TA531SysEnv.TA531_env_DoorSwRL = (buf_rec[5]>>4)&0x03;
				TA531SysEnv.TA531_env_DoorSwRR = (buf_rec[5]>>6)&0x03;

				//TA531 Ack
				TSA_Ack_DATA[4] = 1;
				TSA4_0x53_Flag = 1;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_Header, TSA_Ack_DATA);
				TSA_Ack_DATA[4] = 0;
			}
			else if (FDCAN1_RxHeader.Identifier == 0x064)	//TSA_RC1
			{
				TA531_RC1.TA531_RC_X_trg = (int)((buf_rec[1]<< 8) + (buf_rec[0]<< 0));	//(int)
				if ((buf_rec[2]&0x80) == 0x80)	//<0
				{		TA531_RC1.TA531_RC_X_Mov = 0 - buf_rec[2];		}
				else
				{		TA531_RC1.TA531_RC_X_Mov = buf_rec[2];			}

				TA531_RC1.TA531_RC_Y_trg = (int)((buf_rec[4]<< 8) + (buf_rec[3]<< 0));
				if ((buf_rec[5]&0x80) == 0x80)	//<0
				{		TA531_RC1.TA531_RC_Y_Mov = 0 - buf_rec[5];		}
				else
				{		TA531_RC1.TA531_RC_Y_Mov = buf_rec[5];			}
				TA531_RC1.TA531_RC_Z = (int)(buf_rec[6]<< 0);
				TA531_RC1.TA531_RC_Z_code = buf_rec[7]&0x03;
				TA531_RC1.TA531_RC_Reset = (buf_rec[7]>> 6)&0x03;


				TA531_RC1_fg = 2;	//// 0-Error;	1-no task;	2-command received;	3-task accomplish;
//				TA531_RC1_x_ready = 0;
//				TA531_RC1_y_ready = 0;
//				TA531_RC1_fg = 2;
			}
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{

		if (hfdcan == &hfdcan2) 	//CAN1 Classic CAN
		{
			uint8_t buf_rec[8];

			while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1))
			{
				HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &FDCAN2_RxHeader,	buf_rec);
			}

			if (FDCAN2_RxHeader.Identifier == HostID)	//
			{


				uint16_t TeMP = (buf_rec[0]<<3)+((buf_rec[1]>>5)&0x07);
				if(TeMP == M1_ID)	//X
				{

					if((buf_rec[2] == 0x41)&(buf_rec[3] == 0)&(buf_rec[4] == 0)&(buf_rec[5] == 0)&(buf_rec[6] == 0)&(buf_rec[7] == 0) )	//init ok
					{
						MotorInit_M1 = 2;
						MotorCtrl_M1.M_Position = 0;
					}
					else
					{
						if ((MotorInit_M1 == 2)&(buf_rec[2] == 0x42))	//X_Position
						{
							MotorCtrl_M1.M_Position = (buf_rec[3] + (buf_rec[4]<< 8) + (buf_rec[5]<< 16) + (buf_rec[6]<< 24))/160 ;

							TA531_RC1.TA531_RC_X_act = MotorCtrl_M1.M_Position -10;

//							char str1[16] = {0};
//							itoa(MotorCtrl_M1.M_Position ,str1,10);
//							OLED_ShowString(OLED_I2C_ch ,OLED_type,12, 1, str1);

						}

						if ((buf_rec[7]>> 4 )>0)	//
						{
							MotorERR_M1 = 1;
						}
					}
				}
				else if(TeMP == M2_ID)	//x
				{
					if((buf_rec[2] == 0x41)&(buf_rec[3] == 0)&(buf_rec[4] == 0)&(buf_rec[5] == 0)&(buf_rec[6] == 0)&(buf_rec[7] == 0) )	//init ok
					{
						MotorInit_M2 = 2;
						MotorCtrl_M2.M_Position = 0;
					}
					else
					{
						if ((MotorInit_M2 == 2)&(buf_rec[2] == 0x42))	//X_Position
						{
							MotorCtrl_M2.M_Position = (buf_rec[3] + (buf_rec[4]<< 8) + (buf_rec[5]<< 16) + (buf_rec[6]<< 24))/160 ;

//							char str1[16] = {0};
//							itoa(MotorCtrl_M2.M_Position ,str1,10);
//							OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 3, str1);
						}

						if ((buf_rec[7]>> 4 )>0)	//
						{
							MotorERR_M2 = 1;
						}
					}
				}

				else if(TeMP == M3_ID)	//y
				{
					if((buf_rec[2] == 0x41)&(buf_rec[3] == 0)&(buf_rec[4] == 0)&(buf_rec[5] == 0)&(buf_rec[6] == 0)&(buf_rec[7] == 0) )	//init ok
					{
//						OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 0, "trigger" );
						MotorInit_M3 = 2;
						MotorCtrl_M3.M_Position = 0;
					}
					else
					{
						if ((MotorInit_M3 == 2)&(buf_rec[2] == 0x42))	//X_Position
						{
							MotorCtrl_M3.M_Position = (buf_rec[3] + (buf_rec[4]<< 8) + (buf_rec[5]<< 16) + (buf_rec[6]<< 24))/160 ;

							TA531_RC1.TA531_RC_Y_act = MotorCtrl_M3.M_Position -10;

//							char str1[16] = {0};
//							itoa(MotorCtrl_M3.M_Position ,str1,10);
//							OLED_ShowString(OLED_I2C_ch ,OLED_type,8, 3, str1);
						}

						if ((buf_rec[7]>> 4 )>0)	//
						{
							MotorERR_M3 = 1;
						}
					}
				}
			}
		}
	}
}


//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	char *str2;
//	sprintf(str2, "%c", I2CRxData[0]);
//	OLED_SSD1306_ShowString(3 ,0, 1, str2);
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6)	//50ms
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA1_ADC_Header, TSA1_ADC_DATA);
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA2_LS_Header, TSA2_LS_DATA);
	}
	if (htim == &htim7)	//100ms
	{
		if((Web_ConnectSts < 255)&(Web_ConnectSts > 0))
		{
			Web_ConnectSts--;
		}

		sys_state++;

		//int sys_state; //0 - 6; 	0 - none; 1- green	;2 - cyan(青色，天蓝）;3 - blue	;4 - yellow	;5 - purple	;6 - red
		sys_state = sys_state%7;
		switch(sys_state)
		{
		case 0:		//none
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 0);
			break;
		case 1:		//green
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 1);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 0);
			break;
		case 2:		//cyan
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 1);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 1);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 0);
			break;
		case 3:		//blue
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 1);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 0);
			break;
		case 4:		//yellow
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 1);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 1);
			break;
		case 5:		//purple
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 1);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 1);
			break;
		case 6:		//red
			HAL_GPIO_WritePin(GPIOB, IO_SYS_LED_B_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_G_Pin, 0);
			HAL_GPIO_WritePin(GPIOE, IO_SYS_LED_R_Pin, 1);
			break;
		}

	}
	if (htim == &htim14)	//1000ms
	{

		//TA531 Ack
		TSA_Ack_DATA[0] = 0x64;	//100
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TSA_Ack_Header, TSA_Ack_DATA);
		TSA_Ack_DATA[0] = 0;


		Sys_CNT++;

		uint32_t value_ADC1_0;
		value_ADC1_0 = mRead_ADC1_ch(1);
		if (value_ADC1_0 < 4500)
		{
			Bench_Info.Bench_PowerSts = 0;
		}
		else if (value_ADC1_0 < 9000)
		{
			Bench_Info.Bench_PowerSts = 1;
			Sys_RunCNT++;
		}
		else if (value_ADC1_0 < 13500)
		{
			Bench_Info.Bench_PowerSts = 2;
			Sys_RunCNT++;
		}
		else
		{
			Bench_Info.Bench_PowerSts = 3;
			Sys_RunCNT++;
		}


//		SPI_Flash_WriteSomeBytes(&Sys_CNT, Sys_CNT_Addr, sizeof(Sys_CNT));
//		SPI_Flash_WriteSomeBytes(&Sys_RunCNT, Sys_RunCNT_Addr, sizeof(Sys_RunCNT));

	}

	if (htim == &htim16)	//5ms
	{
		Sys_TIM_Flag = 1;	//for CAN


		Sys_TIM_TICK = Sys_TIM_TICK + 5;

		if(Sys_TIM_TICK > 0xFFFFFFF0)
		{
			Sys_TIM_TICK = 0;
		}
	}
	if (htim == &htim17)	//1ms
	{

	}

}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, "Uart_Callback!! ");

	if  (huart == &huart1)
	{
		for (int i = 0; i < Serial_Data_LENGTH ; i++)
		{
			RxData[i] = u1RxData[i];
		}

		UART_RESET(&huart1);
		HAL_UART_Receive_IT(&huart1,u1RxData, Serial_Data_LENGTH );
	}

//	Ser2CAN();
//	CAN2Ser_Config();


	////check heading and ending
//	if((RxData[0] == 0xFA) & (RxData[1] == 0xFB) & (RxData[2] == 0xF0) & (RxData[21] == 0xEA) & (RxData[22] == 0xEB) & (RxData[23] == 0xE0) )
//	{
//		Ser2CAN();
//	}

}


void UART_Init(UART_HandleTypeDef *handle, uint32_t data_length) {

	if (handle == &huart1)
	{
		HAL_UART_Receive_IT(&huart1,u1RxData, Serial_Data_LENGTH );	//
	}

}

void UART_RESET(UART_HandleTypeDef *handle) {

	HAL_UART_AbortReceive(handle);
	HAL_UART_AbortReceive_IT(handle);
	/* Clear RXNE interrupt flag *//* Discard the received data */
	__HAL_UART_SEND_REQ(handle, UART_RXDATA_FLUSH_REQUEST);

}
//void CAN2Ser_Config(void)
//{
//
//}

uint32_t mRead_ADC1_ch(uint8_t ch)
{
	//start ADC
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, 500);

	if (ch == 1)
	{

	}else
	if (ch == 2)	//```6
	{

	}




//    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))//读取ADC完成标志�?????????????????????????????????????
//    {
//        adc_value = HAL_ADC_GetValue(&hadc1);//读出ADC数�??
//
//        // 将ADC值转换为电压
//        mVoltage = adc_value  *1165 /165 * 3250/ 4095; // 12位ADC，最�?????????????????????????????????????4095
//    }



    // 停止ADC转换
    HAL_ADC_Stop(&hadc1);

    return mVoltage;
}


//void Ser2CAN(void)
//{
//	if((RxData[0] == 0xFA) & (RxData[1] == 0xFB) & (RxData[2] == 0xF0) & (RxData[21] == 0xEA) & (RxData[22] == 0xEB) & (RxData[23] == 0xE0) )	// ok Ser2CAN
//	{
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, "Hello ATP mini~");
//
//		char *str = "last TXmsg:";
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
//
//		Ser2CAN_Msg[RxData[3]].MsgNum = RxData[3];
//		Ser2CAN_Msg[RxData[3]].DataChannel = (RxData[4]>>0)& 0x0F;
//		Ser2CAN_Msg[RxData[3]].DataID = (((RxData[4]&0xF0) << 4) + RxData[5])&0x7FF;
//		Ser2CAN_Msg[RxData[3]].DataType = (RxData[6]>>0) & 0x03;
//		Ser2CAN_Msg[RxData[3]].TriggerType = (RxData[6]>>2) & 0x3F;		// max 0x3F
//		Ser2CAN_Msg[RxData[3]].DataCycle = RxData[7]*10;
//
//		if (((RxData[6]>>0) & 0x03) == 1) //CAN
//		{
//			Ser2CAN_Msg[RxData[3]].PDUID = 0;
//
//			uint8_t dLC = (RxData[8]>>4) & 0x0F;
//			if(dLC == 0)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_0;
//			}
//			else if(dLC == 1)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_1;
//			}
//			else if(dLC == 2)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_2;
//			}
//			else if(dLC == 3)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_3;
//			}
//			else if(dLC == 4)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_4;
//			}
//			else if(dLC == 5)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_5;
//			}
//			else if(dLC == 6)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_6;
//			}
//			else if(dLC == 7)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_7;
//			}
//			else if(dLC == 8)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_8;
//			}
//			else if(dLC == 9)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_12;
//			}
//			else if(dLC == 10)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_16;
//			}
//			else if(dLC == 11)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_20;
//			}
//			else if(dLC == 12)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_24;
//			}
//			else if(dLC == 13)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_32;
//			}
//			else if(dLC == 14)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_48;
//			}
//			else if(dLC == 15)
//			{
//				Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_64;
//			}
//
//			Ser2CAN_Msg[RxData[3]].DataValue[0] = RxData[13];
//			Ser2CAN_Msg[RxData[3]].DataValue[1] = RxData[14];
//			Ser2CAN_Msg[RxData[3]].DataValue[2] = RxData[15];
//			Ser2CAN_Msg[RxData[3]].DataValue[3] = RxData[16];
//			Ser2CAN_Msg[RxData[3]].DataValue[4] = RxData[17];
//			Ser2CAN_Msg[RxData[3]].DataValue[5] = RxData[18];
//			Ser2CAN_Msg[RxData[3]].DataValue[6] = RxData[19];
//			Ser2CAN_Msg[RxData[3]].DataValue[7] = RxData[20];
//		}
//		else if (((RxData[6]>>0) & 0x03) == 2) //CAN FD
//		{
//			Ser2CAN_Msg[RxData[3]].PDUID = (RxData[8]>>0) & 0x0F;
//
//			if (((RxData[8]>>0) & 0x0F) == 0) //frame ID
//			{
//
//				uint8_t dLC = (RxData[8]>>4) & 0x0F;
//				if(dLC == 0)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_0;
//				}
//				else if(dLC == 1)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_1;
//				}
//				else if(dLC == 2)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_2;
//				}
//				else if(dLC == 3)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_3;
//				}
//				else if(dLC == 4)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_4;
//				}
//				else if(dLC == 5)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_5;
//				}
//				else if(dLC == 6)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_6;
//				}
//				else if(dLC == 7)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_7;
//				}
//				else if(dLC == 8)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_8;
//				}
//				else if(dLC == 9)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_12;
//				}
//				else if(dLC == 10)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_16;
//				}
//				else if(dLC == 11)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_20;
//				}
//				else if(dLC == 12)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_24;
//				}
//				else if(dLC == 13)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_32;
//				}
//				else if(dLC == 14)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_48;
//				}
//				else if(dLC == 15)
//				{
//					Ser2CAN_Msg[RxData[3]].DataValueLength = FDCAN_DLC_BYTES_64;
//				}
//
//
//				for(int i=0;i<64;i++)
//				{
//					Ser2CAN_Msg[RxData[3]].FDDataValue[i] =  0x00;	//init
//				}
//			}
//			else if(((RxData[8]>>0) & 0x0F) == 1) //iPDU 1
//			{
//				//	header
//				Ser2CAN_Msg[RxData[3]].FDDataValue[0] =  RxData[9];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[1] =  RxData[10];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[2] =  RxData[11];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[3] =  RxData[12];
//
//				//	data
//				Ser2CAN_Msg[RxData[3]].FDDataValue[4] =  RxData[13];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[5] =  RxData[14];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[6] =  RxData[15];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[7] =  RxData[16];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[8] =  RxData[17];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[9] =  RxData[18];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[10] =  RxData[19];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[11] =  RxData[20];
//
//			}
//			else if(((RxData[8]>>0) & 0x0F) == 2) //iPDU 2
//			{
//				//	header
//				Ser2CAN_Msg[RxData[3]].FDDataValue[12] =  RxData[9];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[13] =  RxData[10];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[14] =  RxData[11];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[15] =  RxData[12];
//
//				//	data
//				Ser2CAN_Msg[RxData[3]].FDDataValue[16] =  RxData[13];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[17] =  RxData[14];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[18] =  RxData[15];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[19] =  RxData[16];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[20] =  RxData[17];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[21] =  RxData[18];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[22] =  RxData[19];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[23] =  RxData[20];
//			}
//			else if(((RxData[8]>>0) & 0x0F) == 3) //iPDU 3
//			{
//				//	header
//				Ser2CAN_Msg[RxData[3]].FDDataValue[24] =  RxData[9];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[25] =  RxData[10];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[26] =  RxData[11];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[27] =  RxData[12];
//
//				//	data
//				Ser2CAN_Msg[RxData[3]].FDDataValue[28] =  RxData[13];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[29] =  RxData[14];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[30] =  RxData[15];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[31] =  RxData[16];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[32] =  RxData[17];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[33] =  RxData[18];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[34] =  RxData[19];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[35] =  RxData[20];
//
//			}
//			else if(((RxData[8]>>0) & 0x0F) == 4) //iPDU 4
//			{
//				//	header
//				Ser2CAN_Msg[RxData[3]].FDDataValue[36] =  RxData[9];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[37] =  RxData[10];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[38] =  RxData[11];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[39] =  RxData[12];
//
//				//	data
//				Ser2CAN_Msg[RxData[3]].FDDataValue[40] =  RxData[13];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[41] =  RxData[14];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[42] =  RxData[15];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[43] =  RxData[16];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[44] =  RxData[17];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[45] =  RxData[18];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[46] =  RxData[19];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[47] =  RxData[20];
//
//			}
//			else if(((RxData[8]>>0) & 0x0F) == 5) //iPDU 5
//			{
//				//	header
//				Ser2CAN_Msg[RxData[3]].FDDataValue[48] =  RxData[9];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[49] =  RxData[10];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[50] =  RxData[11];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[51] =  RxData[12];
//
//				//	data
//				Ser2CAN_Msg[RxData[3]].FDDataValue[52] =  RxData[13];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[53] =  RxData[14];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[54] =  RxData[15];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[55] =  RxData[16];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[56] =  RxData[17];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[57] =  RxData[18];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[58] =  RxData[19];
//				Ser2CAN_Msg[RxData[3]].FDDataValue[59] =  RxData[20];
//
//			}
//		}
//	}
//
//	if((RxData[0] == 0xCA) & (RxData[1] == 0xCB) & (RxData[2] == 0xC0) & (RxData[21] == 0xEA) & (RxData[22] == 0xEB) & (RxData[23] == 0xE0) )	//CAN2Ser Setting
//	{
//		char *str = "CAN2Ser Setting";
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
//
//		if ((RxData[4]&0x0F)== 1 ) //CAN ch 1
//		{
//			if((RxData[4]&0xF0)>>4 == 0 )	//first time config
//			{
//				CAN1_2Ser_ID[0] = ((RxData[6]&0x0F)<< 8) + RxData[5];
//				CAN1_2Ser_ID[1] = ((RxData[6]&0xF0)<< 4) + RxData[7];
//
//				CAN1_2Ser_ID[2] = ((RxData[9]&0x0F)<< 8) + RxData[8];
//				CAN1_2Ser_ID[3] = ((RxData[9]&0xF0)<< 4) + RxData[10];
//
//				CAN1_2Ser_ID[4] = ((RxData[12]&0x0F)<< 8) + RxData[11];
//				CAN1_2Ser_ID[5] = ((RxData[12]&0xF0)<< 4) + RxData[13];
//
//				CAN1_2Ser_ID[6] = ((RxData[15]&0x0F)<< 8) + RxData[14];
//				CAN1_2Ser_ID[7] = ((RxData[15]&0xF0)<< 4) + RxData[16];
//
//				CAN1_2Ser_ID[8] = ((RxData[18]&0x0F)<< 8) + RxData[17];
//				CAN1_2Ser_ID[9] = ((RxData[18]&0xF0)<< 4) + RxData[19];
//			}
//			else if((RxData[4]&0xF0)>>4 == 1 )	//2x time config
//			{
//				CAN1_2Ser_ID[10] = ((RxData[6]&0x0F)<< 8) + RxData[5];
//				CAN1_2Ser_ID[11] = ((RxData[6]&0xF0)<< 4) + RxData[7];
//
//				CAN1_2Ser_ID[12] = ((RxData[9]&0x0F)<< 8) + RxData[8];
//				CAN1_2Ser_ID[13] = ((RxData[9]&0xF0)<< 4) + RxData[10];
//
//				CAN1_2Ser_ID[14] = ((RxData[12]&0x0F)<< 8) + RxData[11];
//				CAN1_2Ser_ID[15] = ((RxData[12]&0xF0)<< 4) + RxData[13];
//
//				CAN1_2Ser_ID[16] = ((RxData[15]&0x0F)<< 8) + RxData[14];
//				CAN1_2Ser_ID[17] = ((RxData[15]&0xF0)<< 4) + RxData[16];
//
//				CAN1_2Ser_ID[18] = ((RxData[18]&0x0F)<< 8) + RxData[17];
//				CAN1_2Ser_ID[19] = ((RxData[18]&0xF0)<< 4) + RxData[19];
//			}
//			else if((RxData[4]&0xF0)>>4 == 2 )	//3x time config	//max 0-23
//			{
//				CAN1_2Ser_ID[20] = ((RxData[6]&0x0F)<< 8) + RxData[5];
//				CAN1_2Ser_ID[21] = ((RxData[6]&0xF0)<< 4) + RxData[7];
//
//				CAN1_2Ser_ID[22] = ((RxData[9]&0x0F)<< 8) + RxData[8];
//				CAN1_2Ser_ID[23] = ((RxData[9]&0xF0)<< 4) + RxData[10];
//
////				CAN1_2Ser_ID[24] = ((RxData[12]&0x0F)<< 8) + RxData[11];
////				CAN1_2Ser_ID[25] = ((RxData[12]&0xF0)<< 4) + RxData[13];
////
////				CAN1_2Ser_ID[26] = ((RxData[15]&0x0F)<< 8) + RxData[14];
////				CAN1_2Ser_ID[27] = ((RxData[15]&0xF0)<< 4) + RxData[16];
//
//			}
//
//			for(uint8_t i=0;i<24;i++)
//			{
//				FDCAN_Filter1[i].FilterID1 = CAN1_2Ser_ID[i];
//				FDCAN_Filter1[i].FilterID2 = CAN1_2Ser_ID[i];
//
//				if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter1[i]) != HAL_OK) //滤波器初始化
//				{
//					Error_Handler();
//				}
//			}
//
//			char str1[4] = {0};
//			itoa(CAN1_2Ser_ID[0],str1,10);
//			OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, str1 );
//		}
//		else if ((RxData[4]&0x03)== 2 )//CH 2
//		{
//			if((RxData[4]&0xF0)>>4 == 0 )	//first time config
//			{
//				CAN2_2Ser_ID[0] = ((RxData[6]&0x0F)<< 8) + RxData[5];
//				CAN2_2Ser_ID[1] = ((RxData[6]&0xF0)<< 4) + RxData[7];
//
//				CAN2_2Ser_ID[2] = ((RxData[9]&0x0F)<< 8) + RxData[8];
//				CAN2_2Ser_ID[3] = ((RxData[9]&0xF0)<< 4) + RxData[10];
//
//				CAN2_2Ser_ID[4] = ((RxData[12]&0x0F)<< 8) + RxData[11];
//				CAN2_2Ser_ID[5] = ((RxData[12]&0xF0)<< 4) + RxData[13];
//
//				CAN2_2Ser_ID[6] = ((RxData[15]&0x0F)<< 8) + RxData[14];
//				CAN2_2Ser_ID[7] = ((RxData[15]&0xF0)<< 4) + RxData[16];
//
//				CAN2_2Ser_ID[8] = ((RxData[18]&0x0F)<< 8) + RxData[17];
//				CAN2_2Ser_ID[9] = ((RxData[18]&0xF0)<< 4) + RxData[19];
//			}
//			else if((RxData[4]&0xF0)>>4 == 1 )	//2x time config
//			{
//				CAN2_2Ser_ID[10] = ((RxData[6]&0x0F)<< 8) + RxData[5];
//				CAN2_2Ser_ID[11] = ((RxData[6]&0xF0)<< 4) + RxData[7];
//
//				CAN2_2Ser_ID[12] = ((RxData[9]&0x0F)<< 8) + RxData[8];
//				CAN2_2Ser_ID[13] = ((RxData[9]&0xF0)<< 4) + RxData[10];
//
//				CAN2_2Ser_ID[14] = ((RxData[12]&0x0F)<< 8) + RxData[11];
//				CAN2_2Ser_ID[15] = ((RxData[12]&0xF0)<< 4) + RxData[13];
//
//				CAN2_2Ser_ID[16] = ((RxData[15]&0x0F)<< 8) + RxData[14];
//				CAN2_2Ser_ID[17] = ((RxData[15]&0xF0)<< 4) + RxData[16];
//
//				CAN2_2Ser_ID[18] = ((RxData[18]&0x0F)<< 8) + RxData[17];
//				CAN2_2Ser_ID[19] = ((RxData[18]&0xF0)<< 4) + RxData[19];
//			}
//			else if((RxData[4]&0xF0)>>4 == 2 )	//3x time config
//			{
//				CAN2_2Ser_ID[20] = ((RxData[6]&0x0F)<< 8) + RxData[5];
//				CAN2_2Ser_ID[21] = ((RxData[6]&0xF0)<< 4) + RxData[7];
//
//				CAN2_2Ser_ID[22] = ((RxData[9]&0x0F)<< 8) + RxData[8];
//				CAN2_2Ser_ID[23] = ((RxData[9]&0xF0)<< 4) + RxData[10];
//
//				CAN2_2Ser_ID[24] = ((RxData[12]&0x0F)<< 8) + RxData[11];
//				CAN2_2Ser_ID[25] = ((RxData[12]&0xF0)<< 4) + RxData[13];
//
//				CAN2_2Ser_ID[26] = ((RxData[15]&0x0F)<< 8) + RxData[14];
//				CAN2_2Ser_ID[27] = ((RxData[15]&0xF0)<< 4) + RxData[16];
//
//			}
//
//			for(uint8_t i=0;i<24;i++)
//			{
//				FDCAN_Filter2[i].FilterID1 = CAN2_2Ser_ID[i];
//				FDCAN_Filter2[i].FilterID2 = CAN2_2Ser_ID[i];
//
//				if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter1[i]) != HAL_OK) //滤波器初始化
//				{
//					Error_Handler();
//				}
//			}
//
//		}
//
//	}
//
//
//	if((RxData[0] == 0x0A) & (RxData[1] == 0x0B) & (RxData[2] == 0x00) & (RxData[21] == 0xEA) & (RxData[22] == 0xEB) & (RxData[23] == 0xE0) )	//web ask
//	{
//		Web_ConnectSts = 150;	//15s
//
//		char *str = "Rec Web AskInfo";
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
//
//		Bench_Info_TxData[0] = 0x0A;
//		Bench_Info_TxData[1] = (Bench_Info.ATP_TD_ID)&0x00ff;
//		Bench_Info_TxData[2] = ((Bench_Info.ATP_TD_ID)&0xff00)>>8;
//
//		Bench_Info_TxData[3] = (Bench_Info.Bench_PowerSts&0x03) + 0 ;
//		Bench_Info_TxData[4] = (Bench_Info.ATP_TD_Model&0x0f) + 0 ;
//		Bench_Info_TxData[5] = (Bench_Info.Bench_PgLevel1&0x1f) + ((Bench_Info.Bench_PgLevel2&0x07)<<5 );
//		Bench_Info_TxData[6] = ((Bench_Info.Bench_PgLevel3&0x007)<<5) + (Bench_Info.Bench_PgLevel4&0x1F) ;
//		Bench_Info_TxData[7] = (Bench_Info.Bench_UTInfo&0x1f) + ((Bench_Info.Bench_TestType&0x07)<<5 );
//		Bench_Info_TxData[8] = Bench_Info.Bench_TestCase;
//		Bench_Info_TxData[9] = Bench_Info.Bench_TestProgress;
//		Bench_Info_TxData[10] = (Bench_Info.Bench_ErrSts&0x03) + ((Bench_Info.Bench_NeedEngineer&0x03)<<2 )+ ((Bench_Info.Bench_TestResult&0x0F)<<4);
//		Bench_Info_TxData[11] = 0xEA;
//
//		HAL_UART_Transmit_IT(Serial_Num, Bench_Info_TxData, 12 );
//	}
//
//
//
//
//
//	if((RxData[0] == 0xBA) & (RxData[1] == 0xBB) & (RxData[2] == 0xB0) & (RxData[21] == 0xEA) & (RxData[22] == 0xEB) & (RxData[23] == 0xE0) )	//relay
//	{
//		char *str = "Rec RelayCtrl";
//		OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str );
//
//		//SerLoCtrl_Msg
//		SerLoCtrl_Msg.LocalRelay[0] = (RxData[4]>>0)& 0x03;		//2 bits
//		SerLoCtrl_Msg.LocalRelay[1] = (RxData[4]>>2)& 0x03;
//		SerLoCtrl_Msg.LocalRelay[2] = (RxData[4]>>4)& 0x03;
//		SerLoCtrl_Msg.LocalRelay[3] = (RxData[4]>>6)& 0x03;
//
//		SerLoCtrl_Msg.LocalRelay[4] = (RxData[5]>>0)& 0x03;
//		SerLoCtrl_Msg.LocalRelay[5] = (RxData[5]>>2)& 0x03;
//		SerLoCtrl_Msg.LocalRelay[6] = (RxData[5]>>4)& 0x03;
//		SerLoCtrl_Msg.LocalRelay[7] = (RxData[5]>>6)& 0x03;
//
//
//		SerLoCtrl_Msg.Local_5VOUT[0] = (RxData[7]>>0)& 0x03;
//		SerLoCtrl_Msg.Local_5VOUT[1] = (RxData[7]>>2)& 0x03;
//		SerLoCtrl_Msg.Local_12VOUT[0] = (RxData[7]>>4)& 0x03;
//		SerLoCtrl_Msg.Local_12VOUT[1] = (RxData[7]>>6)& 0x03;
//
//
////		SerLoCtrl_Msg.LocalPSCtrl[0] = (RxData[11]<< 8) + RxData[12];
////		SerLoCtrl_Msg.LocalPSCtrl[1] = (RxData[13]<< 8) + RxData[14];
//
//	}
//
//
//
////	char *str2="";
////	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 0, RxData[0]);
////	sprintf(str2, "%d", RxData[0]);
////	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, str2);
////	sprintf(str2, "%c", RxData[0]);
////	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 2, str2);
////	sprintf(str2, "%x", RxData[0]);
////	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 3, str2);
//
//}


void MotoCtrl_PackSend12()
{
    MotrCtrl_1_DATA[0] = (MotorCtrl_M1.MotorCtrl_HostID >>3) & 0xff ;
    MotrCtrl_1_DATA[1] = (MotorCtrl_M1.MotorCtrl_HostID & 0x07) << 5 ;
    MotrCtrl_1_DATA[2] = ((MotorCtrl_M1.MotorCtrl_FuncType & 0x07)<< 5) + (MotorCtrl_M1.MotorCtrl_FuncCode & 0x1f) ;
    MotrCtrl_1_DATA[3] = MotorCtrl_M1.MotorCtrl_DataCode & 0xff ;
    MotrCtrl_1_DATA[4] = (MotorCtrl_M1.MotorCtrl_DataCode >>8) & 0xff ;
    MotrCtrl_1_DATA[5] = (MotorCtrl_M1.MotorCtrl_DataCode >>16) & 0xff ;
    MotrCtrl_1_DATA[6] = (MotorCtrl_M1.MotorCtrl_DataCode >>24) & 0xff ;
    MotrCtrl_1_DATA[7] = MotorCtrl_M1.MotorCtrl_ByteData;

	MotrCtrl_2_DATA[0] = (MotorCtrl_M2.MotorCtrl_HostID >>3) & 0xff ;
	MotrCtrl_2_DATA[1] = (MotorCtrl_M2.MotorCtrl_HostID & 0x07) << 5 ;
	MotrCtrl_2_DATA[2] = ((MotorCtrl_M2.MotorCtrl_FuncType & 0x07)<< 5) + (MotorCtrl_M2.MotorCtrl_FuncCode & 0x1f) ;
	MotrCtrl_2_DATA[3] = MotorCtrl_M2.MotorCtrl_DataCode & 0xff ;
	MotrCtrl_2_DATA[4] = (MotorCtrl_M2.MotorCtrl_DataCode >>8) & 0xff ;
	MotrCtrl_2_DATA[5] = (MotorCtrl_M2.MotorCtrl_DataCode >>16) & 0xff ;
	MotrCtrl_2_DATA[6] = (MotorCtrl_M2.MotorCtrl_DataCode >>24) & 0xff ;
	MotrCtrl_2_DATA[7] = MotorCtrl_M2.MotorCtrl_ByteData;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_2_TxHeader, MotrCtrl_2_DATA);
	HAL_Delay(10);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_1_TxHeader, MotrCtrl_1_DATA);
	HAL_Delay(20);
}

//void MotoCtrl_PackSend2()
//{
//    MotrCtrl_2_DATA[0] = (MotorCtrl_M2.MotorCtrl_HostID >>3) & 0xff ;
//    MotrCtrl_2_DATA[1] = (MotorCtrl_M2.MotorCtrl_HostID & 0x07) << 5 ;
//    MotrCtrl_2_DATA[2] = ((MotorCtrl_M2.MotorCtrl_FuncType & 0x07)<< 5) + (MotorCtrl_M2.MotorCtrl_FuncCode & 0x1f) ;
//    MotrCtrl_2_DATA[3] = MotorCtrl_M2.MotorCtrl_DataCode & 0xff ;
//    MotrCtrl_2_DATA[4] = (MotorCtrl_M2.MotorCtrl_DataCode >>8) & 0xff ;
//    MotrCtrl_2_DATA[5] = (MotorCtrl_M2.MotorCtrl_DataCode >>16) & 0xff ;
//    MotrCtrl_2_DATA[6] = (MotorCtrl_M2.MotorCtrl_DataCode >>24) & 0xff ;
//    MotrCtrl_2_DATA[7] = MotorCtrl_M2.MotorCtrl_ByteData;
//
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_2_TxHeader, MotrCtrl_2_DATA);
//}

void MotoCtrl_PackSend3()
{
    MotrCtrl_3_DATA[0] = (MotorCtrl_M3.MotorCtrl_HostID >>3) & 0xff ;
    MotrCtrl_3_DATA[1] = (MotorCtrl_M3.MotorCtrl_HostID & 0x07) << 5 ;
    MotrCtrl_3_DATA[2] = ((MotorCtrl_M3.MotorCtrl_FuncType & 0x07)<< 5) + (MotorCtrl_M3.MotorCtrl_FuncCode & 0x1f) ;
    MotrCtrl_3_DATA[3] = MotorCtrl_M3.MotorCtrl_DataCode & 0xff ;
    MotrCtrl_3_DATA[4] = (MotorCtrl_M3.MotorCtrl_DataCode >>8) & 0xff ;
    MotrCtrl_3_DATA[5] = (MotorCtrl_M3.MotorCtrl_DataCode >>16) & 0xff ;
    MotrCtrl_3_DATA[6] = (MotorCtrl_M3.MotorCtrl_DataCode >>24) & 0xff ;
    MotrCtrl_3_DATA[7] = MotorCtrl_M3.MotorCtrl_ByteData;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &MotrCtrl_3_TxHeader, MotrCtrl_3_DATA);
	HAL_Delay(20);
}

void MotoCtrl_PackSend4()
{

}


void MotoCtrl_PositionLoop(int PositionX_mm, int PositionY_mm)
{
	if (PositionX_mm < 0)	//10mm -10 = 0
	{
		PositionX_mm = 0;
	}
	else if (PositionX_mm > XmaxLimit)	//190mm - 10mm = 180
	{
		PositionX_mm = XmaxLimit;
	}

	if (PositionY_mm < 0)	//10mm
	{
		PositionY_mm = 0;
	}
	else if (PositionY_mm > YmaxLimit)	//190mm + 10mm
	{
		PositionY_mm = YmaxLimit;
	}

	MotorCtrl_M3.MotorCtrl_HostID = HostID;
	MotorCtrl_M3.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M3.MotorCtrl_FuncCode = 0x02;
	MotorCtrl_M3.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M3.MotorCtrl_DataCode = (PositionY_mm +10 ) * 160;

	MotoCtrl_PackSend3();


	MotorCtrl_M1.MotorCtrl_HostID = HostID;
	MotorCtrl_M1.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M1.MotorCtrl_FuncCode = 0x02;
	MotorCtrl_M1.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M1.MotorCtrl_DataCode = (PositionX_mm +10 ) * 160;

	MotorCtrl_M2.MotorCtrl_HostID = HostID;
	MotorCtrl_M2.MotorCtrl_FuncType = 0x01;
	MotorCtrl_M2.MotorCtrl_FuncCode = 0x02;
	MotorCtrl_M2.MotorCtrl_ByteData = 0x01;		//06/07
	MotorCtrl_M2.MotorCtrl_DataCode = (PositionX_mm +10 ) * 160;

	MotoCtrl_PackSend12();

}


uint8_t ByteEncryp(uint8_t byteData)
{
	return (EncrypKey ^ byteData);
}





//	MD5
#define ROTATELEFT(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

/**
 * @desc: convert message and mes_bkp string into integer array and store them in w
 */
//static void md5_process_part1(uint32_t *w, unsigned char *message,
//		uint32_t *pos, uint32_t mes_len, const unsigned char *mes_bkp) {
//	uint32_t i; // used in for loop
//
//	for (i = 0; i <= 15; i++) {
//		int32_t count = 0;
//		while (*pos < mes_len && count <= 24) {
//			w[i] += (((uint32_t) message[*pos]) << count);
//			(*pos)++;
//			count += 8;
//		}
//		while (count <= 24) {
//			w[i] += (((uint32_t) mes_bkp[*pos - mes_len]) << count);
//			(*pos)++;
//			count += 8;
//		}
//	}
//}

/**
 * @desc: start encryption based on w
 */
//static void md5_process_part2(uint32_t abcd[4], uint32_t *w,
//		const uint32_t k[64], const uint32_t s[64]) {
//	uint32_t i; // used in for loop
//
//	uint32_t a = abcd[0];
//	uint32_t b = abcd[1];
//	uint32_t c = abcd[2];
//	uint32_t d = abcd[3];
//	uint32_t f = 0;
//	uint32_t g = 0;
//
//	for (i = 0; i < 64; i++) {
//		if (i >= 0 && i <= 15) {
//			f = (b & c) | ((~b) & d);
//			g = i;
//		} else if (i >= 16 && i <= 31) {
//			f = (d & b) | ((~d) & c);
//			g = (5 * i + 1) % 16;
//		} else if (i >= 32 && i <= 47) {
//			f = b ^ c ^ d;
//			g = (3 * i + 5) % 16;
//		} else if (i >= 48 && i <= 63) {
//			f = c ^ (b | (~d));
//			g = (7 * i) % 16;
//		}
//		uint32_t temp = d;
//		d = c;
//		c = b;
//		b = ROTATELEFT((a + f + k[i] + w[g]), s[i]) + b;
//		a = temp;
//	}
//
//	abcd[0] += a;
//	abcd[1] += b;
//	abcd[2] += c;
//	abcd[3] += d;
//}
//
//static const uint32_t k_table[] = { 0xd76aa478, 0xe8c7b756, 0x242070db,
//		0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501, 0x698098d8,
//		0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e,
//		0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
//		0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87,
//		0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942,
//		0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60,
//		0xbebfbc70, 0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039,
//		0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244, 0x432aff97, 0xab9423a7,
//		0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1, 0x6fa87e4f,
//		0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
//		0xeb86d391 };
//
//static const uint32_t s_table[] = { 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
//		7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
//		4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15,
//		21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21 };
//
//int32_t cal_md5(unsigned char *result, unsigned char *data, int length) {
//	if (result == NULL) {
//		return 1;
//	}
//
//	uint32_t w[16];
//
//	uint32_t i; // used in for loop
//
//	uint32_t mes_len = length;
//	uint32_t looptimes = (mes_len + 8) / 64 + 1;
//	uint32_t abcd[] = { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476 };
//
//	uint32_t pos = 0; // position pointer for message
//	uint32_t bkp_len = 64 * looptimes - mes_len; // 经过计算发现不超�?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????72
//
////    unsigned char *bkp_mes = (unsigned char *)calloc(1, bkp_len);
//	unsigned char bkp_mes[80];
//	for (int i = 0; i < 80; i++) //初始�?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
//			{
//		bkp_mes[i] = 0;
//	}
//
//	bkp_mes[0] = (unsigned char) (0x80);
//	uint64_t mes_bit_len = ((uint64_t) mes_len) * 8;
//	for (i = 0; i < 8; i++) {
//		bkp_mes[bkp_len - i - 1] = (unsigned char) ((mes_bit_len
//				& (0x00000000000000FF << (8 * (7 - i)))) >> (8 * (7 - i)));
//	}
//
//	for (i = 0; i < looptimes; i++) {
//		for (int j = 0; j < 16; j++) //初始�?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
//				{
//			w[j] = 0x00000000;
//		}
//
//		md5_process_part1(w, data, &pos, mes_len, bkp_mes); // compute w
//
//		md5_process_part2(abcd, w, k_table, s_table); // calculate md5 and store the result in abcd
//	}
//
//	for (int i = 0; i < 16; i++) {
//		result[i] = ((unsigned char*) abcd)[i];
//	}
//
//	return 0;
//}

//void PCF8563_ReadDateTime(uint8_t *buffer)
//{
//HAL_I2C_Mem_Read(&hi2c1, PCF8563_ADDRESS, 2, I2C_MEMADD_SIZE_8BIT, buffer, 7, HAL_MAX_DELAY);
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	char *str1 = "SYS ERROR!";
	OLED_ShowString(OLED_I2C_ch ,OLED_type,0, 1, str1);


	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
