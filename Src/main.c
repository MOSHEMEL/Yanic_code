/* USER CODE BEGIN Header */
/**m*************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define OneWire0_Pin GPIO_PIN_5
#define OneWire0_GPIO_Port GPIOB

#define OneWire1_Pin GPIO_PIN_4
#define OneWire1_GPIO_Port GPIOC

#define OneWire2_Pin GPIO_PIN_5
#define OneWire2_GPIO_Port GPIOC

#define OneWire3_Pin GPIO_PIN_4
#define OneWire3_GPIO_Port GPIOB

#define PORT_RS_and_E GPIOB
#define PIN_RS GPIO_PIN_5
#define PIN_E  GPIO_PIN_5

// D0_PIN, D1_PIN, D2_PIN, D3_PIN;	// LSBs D0, D1, D2 and D3 pins


#define PORT_MSB	GPIOB									// MSBs D5, D6, D7 and D8 PORT
#define D4_PIN    GPIO_PIN_5
#define D5_PIN    GPIO_PIN_6
#define D6_PIN    GPIO_PIN_7
#define D7_PIN    GPIO_PIN_8


#define ADC_Presure 0
#define ADC_Battery 1

#define ADC_Presure_27_bar 700//  need to be 847 , we done factor  with adi
#define ADC_Pressure_20_bar 660

#define  sw_ver   "Armenta ver 8.0.3b 07/09/19"  // dd/mm/yy  // b - beta a-alpha

//#include "ds18b20.h"
//#include "STM_MY_LCD16X2.h"


#include <time.h>

//  for debug
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 


volatile float temp1;
volatile uint8_t humi;
volatile uint8_t read_data[2];
volatile uint16_t reg = 0;
int ErrorCounter;
char Tret = 0;
char PulseCouterDir = 0;      // Pulse counter direction [0=> from 0 to 400, 1=> 400 to 0
//	hdc1080_init(&hi2c1,Temperature_Resolution_14_bit,Humidity_Resolution_14_bit);
#define         HDC_1080_ADD                            0x40
#define         Configuration_register_add              0x02
#define         Temperature_register_add                0x00
#define         Humidity_register_add                   0x01
#define         START_BEEP 															100
#define         BEEP_COUNTER_200												110	
#define         BEEP_COUNTER_400												120	
#define         BEEP_COUNTER_0												  130	
#define         KEY_PRESS												        80	

#define         Beep_Every_X_Counting                   200 



#define MemReadHeadId 				50
#define MemUpdateHeadCounter  100
#define MemUpdateSytemCounter 110
#define MemCheck 							210
#define MemReset 							220
#define Pass									1
#define Faild			
 
#define TRANSMITTER_BOARD
#define RXBUFFERSIZE 256



//Controller 
#define CS1_LOW   			GPIOC,GPIO_PIN_12,GPIO_PIN_RESET
#define CS1_HIGH  			GPIOC,GPIO_PIN_12,GPIO_PIN_SET

// splitter ( rakezet )
#define CS2_LOW   			GPIOD,GPIO_PIN_2,GPIO_PIN_RESET
#define CS2_HIGH  			GPIOD,GPIO_PIN_2,GPIO_PIN_SET


// Head memory
#define CS3_LOW   			GPIOB,GPIO_PIN_8,GPIO_PIN_RESET
#define CS3_HIGH  			GPIOB,GPIO_PIN_8,GPIO_PIN_SET

#define HIGH  1
#define LOW   0


#define CS_Applicator 3
#define CS_Router     2
#define CS_Mcu        1

// PROGRAMER REMARK
//PulseCouter-> optocoloer motor cycles RPM
//piazo  check if there is hit
///
float BatteryPercent = 0, BatteryA, BatteryB;
int PiazoCouter = 0;
long MsCounter = 0;
int PulseCouter = 0;
int IsPulseFinshed = 0;
long temp_memory = 0;
char EnablePulseCouter = 0;
char aTxBuffer[100];
uint16_t Counter = 0;
uint16_t MaxCounter = 5000;  
uint8_t CountMethod = 0; 
uint8_t BtmPwm = 0; 

uint8_t PwmPulse = 4;     //hz
// check pulse :

uint8_t IsBeep400 = 0;
uint8_t PulseAplitudeCounter = 0;
uint8_t PulseAplitudeeThatWasRead = 0;
 
uint8_t RunMotor = 0;
uint16_t adc[16];
uint8_t IsSystemTestPaased = 0;

uint8_t SpiDataWrite[100];
uint8_t SpiDataRead[100];
uint8_t serial_number[6];
uint8_t Device_Id[6];


typedef enum
{
	Temperature_Resolution_14_bit = 0,
	Temperature_Resolution_11_bit = 1
}Temp_Reso;

typedef enum
{
	Humidity_Resolution_14_bit = 0,
	Humidity_Resolution_11_bit = 1,
	Humidity_Resolution_8_bit  = 2
}Humi_Reso;


// MOTOR PWM SETUP 16-10-18
#define MOTOR_1_PWM_ON_50 50
#define MOTOR_1_PWM_ON_75 75
#define MOTOR_1_PWM_ON 100
#define MOTOR_1_PWM_OFF 0

// 1 logical in software is on , and in hardware is off 
// 0 logical in software is off , and in hardware is on
//( please not sofware oposit from hardware)


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t pwm[5];


 
char CheckUartBuffer(void);      // check buufer and AT command 
float TestBatteryStatus(void);
char CheckIfCoverClosed(void);
void CheckCounterStatus(void);
 
 
char SystemTest(void);
void TestGasPresserSensor(void);
void BuzzerBip(uint8_t bip);
void MotorCommand(uint8_t commad);     // 0 stop else pwm value on
void Memory(uint8_t Command, uint16_t param);     // 0 stop else pwm value on
void PrintLcd(char *s); 
 
 
float Check_pulse_amplitude(void);
// memory read and write 
 char WriteToMemoryCounter(char CS, long  counter); 
char WriteToMemorySerial(char CS, uint8_t *serial_number); 
 
long  ReadFromMemoryCounter(char CS); 
char ReadFromMemorySerial(char CS); 
char ChipSelect(char CS, char status); 
char Compare(long p1, long p2);
 
void test_01(void);     // step from 100 to 0
void test_02(void);     // read potentiometer and and set pwm
void test_03(void);
 
// extern vars 
 char buf_rx_4[102];
char rx4_counter = 0;
char IsCommand;
long MemoryCounter[4];
// .........
 uint16_t adc[16];
uint16_t fan_counter, fan_counter2, current_fan2;
int16_t temp[3];

float PTuser, TempHeatSink2, TempHeatSink1, Tin, TempHeatSink3, TempHeatSink3, Ttarget, light, PTuser;     /// T tempature out and in .
uint8_t Menual = 0;
uint8_t Mode = 0;

int MIC_INPUT = 0;

double diff_t;
RTC_TimeTypeDef sTime1, sTime2;
RTC_DateTypeDef sDate1;
RTC_AlarmTypeDef sAlarm;
time_t start_t, end_t;
RTC_DateTypeDef sdatestructureget;
RTC_TimeTypeDef stimestructureget;
char key1, key2, key3, aShowTime[100];
char ChipSelectNumber = 3;     //CHIP SELECT 
	
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char ChipSelect(char CS, char status)
{
	if (CS == 1)
	{
		if (status == 1)
		{
			HAL_GPIO_WritePin(CS1_HIGH);
		}
		else
		{
			HAL_GPIO_WritePin(CS1_LOW);
		}									
	}		

			
			
	if (CS == 2)
	{
		if (status == 1)
		{
			HAL_GPIO_WritePin(CS2_HIGH);
		}
		else
		{
			HAL_GPIO_WritePin(CS2_LOW);
		}									
	}		
			
	if (CS == 3)
	{
		if (status == 1)
		{
			HAL_GPIO_WritePin(CS3_HIGH);
		}
		else
		{
			HAL_GPIO_WritePin(CS3_LOW);
		}									
	}
			
			
			
}

char Compare(long p1, long p2)
{
	if (p1 == p2)return (1);
	else return (0);
}				
 
char WriteToMemoryCounter(char CS, long counter)
{
	//	if(counter>1000000)return(-1); //limitter :   can not write never FFFFFF
					
		ChipSelectNumber = CS;
	ChipSelect(ChipSelectNumber, HIGH);
	HAL_Delay(10);

	/* ----------------------------------------------------- */
					
	// *****   Erease start Address :                  ******* //
	ChipSelect(ChipSelectNumber, LOW);		
	HAL_Delay(2);
	SpiDataWrite[0] = 0x06;     // write enable
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	ChipSelect(ChipSelectNumber, HIGH);
	HAL_Delay(20);


	ChipSelect(ChipSelectNumber, LOW);		
	HAL_Delay(2);
	SpiDataWrite[0] = 0x20;    // erase opcodes: 4Kbytes: 0x20, 32Kbytes: 0x52, 64Kbytes: 0xD8
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	//	HAL_GPIO_WritePin(CS1_HIGH);
							
// Address :
  SpiDataWrite[0] = 0x00  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF;
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	ChipSelect(ChipSelectNumber, HIGH);
			

	HAL_Delay(50);     // --

			if(CS == CS_Mcu)
	{
		sprintf(aTxBuffer, "CS[%d] [MCU] writing %ld \r\n", CS, counter);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000) != HAL_OK)
		{
			Error_Handler();   
		}
	}
									
	else if(CS == CS_Router)
	{
		sprintf(aTxBuffer, "CS[%d] [CS_Router] writing %ld \r\n", CS, counter);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000) != HAL_OK)
		{
			Error_Handler();   
		}
	}
	else if(CS == CS_Applicator)
	{
		sprintf(aTxBuffer, "CS[%d] [CS_Applicator] writing %ld \r\n", CS, counter);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000) != HAL_OK)
		{
			Error_Handler();   
		}
	}
	// Data Protection
									
									
// --------------------------------------------------------------
// write  start  : Counter 			
// counter value	
	  //		WriteToMemoryCounter(2, c1);
						
	  			// write enable
	  				ChipSelect(ChipSelectNumber, LOW);									
	//		HAL_GPIO_WritePin(CS2_LOW);
	HAL_Delay(2);
	SpiDataWrite[0] = 0x06;     // write enable
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	ChipSelect(ChipSelectNumber, HIGH);
	//	HAL_GPIO_WritePin(CS2_HIGH);
	HAL_Delay(20);
		
			

			
	ChipSelect(ChipSelectNumber, LOW);		
	//	HAL_GPIO_WritePin(CS2_LOW);
	HAL_Delay(2);
	SpiDataWrite[0] = 0x02;    // write Byte/Page Program (1 to 256 Bytes)
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	//	HAL_GPIO_WritePin(CS1_HIGH);
							
// Address start : where to write :
	 SpiDataWrite[0] = 0x00  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF;
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	// Address end :			

	  // after push address start to writ datas one by one ....
	  // 		until lenght 256 total per page
									
									
									
									
									
	  // datas start  from address  start 00-0F-FFh  00-00-00h 
	  			SpiDataWrite[0] = (unsigned char)(counter     & 0x0000FF);
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = (unsigned char)(counter >> 8  & 0x0000FF);
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = (unsigned char)(counter >> 16 & 0x0000FF);
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	// data end
	//ChipSelect(ChipSelectNumber,HIGH);
ChipSelect(ChipSelectNumber, HIGH);
	//HAL_GPIO_WritePin(CS2_HIGH);
	HAL_Delay(50);     // ---
					
				

// data end
ChipSelect(CS, HIGH);
	//HAL_GPIO_WritePin(CS1_HIGH);
			HAL_Delay(250);
	
	sprintf(aTxBuffer, "write data1  (%ld)  \r\n", counter);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000) != HAL_OK)
	{
		Error_Handler();   
	}
				
	return 0;
}

			
char WriteToMemorySerial(char CS, uint8_t *serial_number)
{
	ChipSelectNumber = CS;
	ChipSelect(ChipSelectNumber, HIGH);
	HAL_Delay(10);

	/* ----------------------------------------------------- */
					
	// *****   Erease start Address :                  ******* //
	ChipSelect(ChipSelectNumber, LOW);		
	HAL_Delay(2);
	SpiDataWrite[0] = 0x06;     // write enable
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	ChipSelect(ChipSelectNumber, HIGH);
	HAL_Delay(20);


	ChipSelect(ChipSelectNumber, LOW);		
	HAL_Delay(2);
	SpiDataWrite[0] = 0x20;    // erase opcodes: 4Kbytes: 0x20, 32Kbytes: 0x52, 64Kbytes: 0xD8
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	//	HAL_GPIO_WritePin(CS1_HIGH);
							
// Address :
  SpiDataWrite[0] = 0x00  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x10  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF;
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	ChipSelect(ChipSelectNumber, HIGH);
			

	HAL_Delay(50);     // --

	    sprintf(aTxBuffer, "Write   CS(%d)       [HEX] %02X-%02X-%02X-%02X-%02X-%02X\r\n", ChipSelectNumber, serial_number[0], serial_number[1], serial_number[2], serial_number[3], serial_number[4], serial_number[5]);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000) != HAL_OK)
	{
		Error_Handler();   
	}
			
			
	// Data Protection
									
									
				// --------------------------------------------------------------
				// write  start  : Counter 			
		  // counter value	
				//		WriteToMemoryCounter(2, c1);
						
							// write enable
								ChipSelect(ChipSelectNumber, LOW);									
	//		HAL_GPIO_WritePin(CS2_LOW);
	HAL_Delay(2);
	SpiDataWrite[0] = 0x06;     // write enable
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	ChipSelect(ChipSelectNumber, HIGH);
	//	HAL_GPIO_WritePin(CS2_HIGH);
	HAL_Delay(20);
		
			

			
	ChipSelect(ChipSelectNumber, LOW);		
	//	HAL_GPIO_WritePin(CS2_LOW);
	HAL_Delay(2);
	SpiDataWrite[0] = 0x02;    // write Byte/Page Program (1 to 256 Bytes)
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	//	HAL_GPIO_WritePin(CS1_HIGH);
							
// Address start : where to write :
	 SpiDataWrite[0] = 0x00  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x10  & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF;
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	// Address end :			

	  // after push address start to writ datas one by one ....
	  // 		until lenght 256 total per page
									
									
									
									
									
	  // datas start  from address  start 00-0F-FFh  00-00-00h 
	  															SpiDataWrite[0] = serial_number[0]  & 0xFF; 
	;
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = serial_number[1] & 0xFF;
	;
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = serial_number[2]  & 0xFF; 
	;  
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = serial_number[3]  & 0xFF;
	;  
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = serial_number[4]  & 0xFF;
	;   
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = serial_number[5]  & 0xFF;
	;  
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
												
	SpiDataWrite[0] = serial_number[6]  & 0xFF;
	;  
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	HAL_Delay(50);     // ---
					
				

// data end
ChipSelect(CS, HIGH);
	//HAL_GPIO_WritePin(CS1_HIGH);
			HAL_Delay(250);
	
	// ******   READ ****************
								//	ReadFromMemoryCounter(2);	
															
							
	
							
									
				
									//		HAL_Delay(100);
									//	}
				ChipSelect(ChipSelectNumber, HIGH);
	//	HAL_GPIO_WritePin(CS2_HIGH);


HAL_Delay(50);
	
	// read serial number 
							
ChipSelect(ChipSelectNumber, LOW);			
	//			HAL_GPIO_WritePin(CS2_LOW);
					SpiDataWrite[0] = 0x03;     //read 
					HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
						
	SpiDataWrite[0] = 0x00 & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
										
	SpiDataWrite[0] = 0x10  & 0xFF;     // page
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF;     // bytes index in page
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	//	for(int counter256=0;counter256<100;counter256++)
	//		{
			HAL_SPI_Receive(&hspi1, SpiDataRead, 6, 100);

	sprintf(aTxBuffer, "Read serial CS[%d]  %02X-%02X %02X-%02X-%02X-%02X \r\n\r\n", ChipSelectNumber, SpiDataRead[0], SpiDataRead[1], SpiDataRead[2], SpiDataRead[3], SpiDataRead[4], SpiDataRead[5]);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000) != HAL_OK)
	{
		Error_Handler();   
	}
							
									
				
	//		HAL_Delay(100);
	//	}
ChipSelect(ChipSelectNumber, HIGH);
	//	HAL_GPIO_WritePin(CS2_HIGH);

				
return 0;
						
}	
			
char ReadFromMemorySerial(char CS)
{
			
}	
			
			
long ReadFromMemoryCounter(char CS)
{
	long counter;
	ChipSelect(CS, HIGH);
	HAL_Delay(50);
	ChipSelect(CS, LOW);
	// read counter 
//HAL_GPIO_WritePin(CS1_LOW);
	SpiDataWrite[0] = 0x03;     //read 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
						
	SpiDataWrite[0] = 0x00 & 0xFF; 
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
										
	SpiDataWrite[0] = 0x00  & 0xFF;     // page
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	SpiDataWrite[0] = 0x00  & 0xFF;     // bytes index in page
	HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
	//	for(int counter256=0;counter256<100;counter256++)
	//		{
			HAL_SPI_Receive(&hspi1, SpiDataRead, 4, 100);

							
	ChipSelect(CS, HIGH);
	counter = SpiDataRead[2] << 16 | SpiDataRead[1] << 8 | SpiDataRead[0];	
	MemoryCounter[CS] = counter;		
	//	HAL_GPIO_WritePin(CS1_HIGH);
											
											
if(CS == CS_Router)
	{
		sprintf(aTxBuffer, "Read CS[%d]  [CS_Router]   val[%ld] - %02X-%02X %02X \r\n", CS, MemoryCounter[CS], SpiDataRead[2], SpiDataRead[1], SpiDataRead[0]);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
		{
			Error_Handler();   
		}
	}
	else if(CS == CS_Applicator)
	{
		sprintf(aTxBuffer, "Read CS[%d]  [CS_Applicator]   val[%ld] - %02X-%02X %02X \r\n", CS, MemoryCounter[CS], SpiDataRead[2], SpiDataRead[1], SpiDataRead[0]);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
		{
			Error_Handler();   
		}
	}	
	else if(CS == CS_Router)
	{
		sprintf(aTxBuffer, "Read CS[%d]  [CS_Router]   val[%ld] - %02X-%02X %02X \r\n", CS, MemoryCounter[CS], SpiDataRead[2], SpiDataRead[1], SpiDataRead[0]);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
		{
			Error_Handler();   
		}
	}	
	else if(CS == CS_Mcu)
	{
		sprintf(aTxBuffer, "Read CS[%d]  [CS_Mcu]   val[%ld] - %02X-%02X %02X \r\n", CS, MemoryCounter[CS], SpiDataRead[2], SpiDataRead[1], SpiDataRead[0]);
		if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
		{
			Error_Handler();   
		}
	}								
								

									
	return (counter);
						
}
			
float TestBatteryStatus(void)
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	// adcValue=HAL_ADC_GetValue(&hadc1);
  adc[0] = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
					
					
	sprintf(aTxBuffer, "battery status %dV\r\n", adc[0]);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
						
	return (adc[0]);
}	
				
			
		
				
char CheckIfCoverClosed(void)
{
	char isCheckIfCoverClosed = 0;
	//		  isCheckIfCoverClosed=HAL_GPIO_ReadPin(CoverClose_GPIO_Port, CoverClose_Pin); //S0
				sprintf(aTxBuffer, "Cover=%d\r\n", isCheckIfCoverClosed);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
									
	return (isCheckIfCoverClosed);
}

void CheckCounterStatus(void)
{
	sprintf(aTxBuffer, "Counter.Head = %d ,Counter.System  %d\r\n", 0, 0);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
}			




void TestGasPresserSensor(void)
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	// adcValue=HAL_ADC_GetValue(&hadc1);
  adc[0] = HAL_ADC_GetValue(&hadc);
									
									
	HAL_ADC_PollForConversion(&hadc, 100);
	// adcValue=HAL_ADC_GetValue(&hadc1);
  adc[1] = HAL_ADC_GetValue(&hadc);	
	HAL_ADC_Stop(&hadc);
									
									
	//sprintf(aTxBuffer,"Gas Pressure 1.0f\r\n",%adc[0] );
			  sprintf(aTxBuffer, "Gas Pressure %d ,%d \r\n", adc[0], adc[1]);
								
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
}	

void BuzzerBip(uint8_t bip)
{
	if (bip == 1)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
	}
	else
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
	}
									
	//	sprintf(aTxBuffer,"Buzzer beep\r\n" );
	//	if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500)!= HAL_OK)
	//	{
//			Error_Handler();   
//		}
									
									
	if(bip == KEY_PRESS)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
							
	}
									
									
	if (bip == START_BEEP)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(150);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
							
		HAL_Delay(150);
							
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(150);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
	}
									
									
	if (bip == BEEP_COUNTER_200)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
							
		HAL_Delay(50);
							
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
	}
								
	if (bip == BEEP_COUNTER_400)
	{
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
							
		HAL_Delay(50);
							
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
					
										HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
							
		HAL_Delay(50);
							
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);     //S0
		HAL_Delay(50);
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);     //S0
		
										
	}
									
}	

char SystemTest(void)
{
	sprintf(aTxBuffer, "system test\r\n");
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
									
	return (1);
										
}
								
float Check_pulse_amplitude(void)
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	// adcValue=HAL_ADC_GetValue(&hadc1);
	adc[0] = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
											
	sprintf(aTxBuffer, "pulse_amplitude %1.0f\r\n", adc[0]);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
									
	return (adc[0]);
									
}	
								
								
void MotorCommand(uint8_t commad)
{
									
	if (commad == 1) // motor on
		{

			//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 75); // motor
			//   HAL_Delay(50);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 101);     // motor
				 HAL_Delay(100);
			EnablePulseCouter = 1;
		}
									
	else if (commad == 0) // motor on
		{
			//		EnablePulseCouter=0;
			//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 60); // motor
				   HAL_Delay(250);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor
										
		}
									
	else if (commad == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 75);     // motor
	}
									
	else	if (commad == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50);     // motor
	}
									
	else	if (commad == 4)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 25);     // motor
	}
						
}								
								
void Memory(uint8_t Command, uint16_t param)
{
	sprintf(aTxBuffer, "test battery status\r\n");
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
}

void PrintLcd(char *s)
{
	sprintf(aTxBuffer, "%s\r\n", s);
	if (HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
	{
		Error_Handler();   
	}
}										






 
 
 
   
	 
	
	 
	 
	
 
void Humidity_init(I2C_HandleTypeDef* hi2c_x, Temp_Reso Temperature_Resolution_x_bit, Humi_Reso Humidity_Resolution_x_bit)
{
	/* Temperature and Humidity are acquired in sequence, Temperature first
	 * Default:   Temperature resolution = 14 bit,
	 *            Humidity resolution = 14 bit
	 */

	/* Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1 */
	uint16_t config_reg_value = 0x1000;
	uint8_t data_send[2];

	if (Temperature_Resolution_x_bit == Temperature_Resolution_11_bit)
	{
		config_reg_value |= (1 << 10);     //11 bit
	}

	switch (Humidity_Resolution_x_bit)
	{
	case Humidity_Resolution_11_bit:
		config_reg_value |= (1 << 8);
		break;
	case Humidity_Resolution_8_bit:
		config_reg_value |= (1 << 9);
		break;
	}

	data_send[0] = (config_reg_value >> 8);
	data_send[1] = (config_reg_value & 0x00ff);

	HAL_I2C_Mem_Write(hi2c_x, HDC_1080_ADD << 1, Configuration_register_add, I2C_MEMADD_SIZE_8BIT, data_send, 2, 1000);
}


uint8_t Read_Humidity(I2C_HandleTypeDef* hi2c_x, float* temperature, uint8_t* humidity)
{
	uint8_t receive_data[4];
	uint16_t temp_x, humi_x;
	uint8_t send_data = Temperature_register_add;

	HAL_I2C_Master_Transmit(hi2c_x, HDC_1080_ADD << 1, &send_data, 1, 1000);

	/* Delay here 20ms for conversion compelete.
	 * Note: datasheet say maximum is 7ms, but when delay=7ms, the read value is not correct
	 */
	HAL_Delay(20);

	/* Read temperature and humidity */
	HAL_I2C_Master_Receive(hi2c_x, HDC_1080_ADD << 1, receive_data, 4, 1000);

	temp_x = ((receive_data[0] << 8) | receive_data[1]);
	humi_x = ((receive_data[2] << 8) | receive_data[3]);

	*temperature = ((temp_x / 65536.0) * 165.0) - 40.0;
	*humidity = (uint8_t)((humi_x / 65536.0) * 100.0);

	return 0;

}

float ewma_bat(float new_batt)
{
	// Exponential window moving avverage
	// to battery lineariazation
	if(BatteryPercent > 0)
	{
		return (BatteryPercent * 0.99 + new_batt * 0.01);
	}
	else
	{
		float BatteryA = 0.23;
		float BatteryB = 24.86;
		return BatteryA*adc[ADC_Battery] - BatteryB ;
	}
}

char CheckUartBuffer(void)
{
	long temp_memory1;
	char str[250];
	//	sprintf(str,"\r\n### %d :<<%s>> ]\r\n ",rx4_counter,&buf_rx_4[0]);HAL_Delay(100);
	//		HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);

	if(IsCommand == 1)
	{	
		

		
		if (buf_rx_4[0] == '#')
		{
			
			if (strcmp(&buf_rx_4[0], "#T1\r\n") == 0)
			{
				//		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, MOTOR_1_PWM_ON); // start motor
					sprintf(str, "!Treatment Start#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						Tret = 1;
				PiazoCouter = 0;
				PulseCouter = 0;
				MotorCommand(1);
			} 
			else  if (strcmp(&buf_rx_4[0], "#M1=1\r\n") == 0)
			{
						


							
							
							
				MotorCommand(1);
				sprintf(str, "!Motor[1] Start#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						ErrorCounter++;
			} 
			else  if (strcmp(&buf_rx_4[0], "#M1=0\r\n") == 0)
			{
				MotorCommand(0);
							
				sprintf(str, "!Motor[1] Off#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						//ErrorCounter++;
							
			} 
			else  if (strcmp(&buf_rx_4[0], "#M1=2\r\n") == 0)
			{
				MotorCommand(2);
				sprintf(str, "!Motor[1] PWM 75#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						ErrorCounter++;
			} 
						
			else  if (strcmp(&buf_rx_4[0], "#M1=3\r\n") == 0)
			{
				MotorCommand(3);
				sprintf(str, "!Motor[1] PWM 50#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						ErrorCounter++;
			} 						
			else  if (strcmp(&buf_rx_4[0], "#M1=4\r\n") == 0)
			{
				MotorCommand(4);
				sprintf(str, "!Motor[1] PWM 25#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						ErrorCounter++;
			} 
			else  if (strcmp(&buf_rx_4[0], "#M1=-1\r\n") == 0)
			{
				sprintf(str, "!Motor[1] Reverse#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
				//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						ErrorCounter++;
			} 
					

			// check mem command 
				if(buf_rx_4[1] == 'Z')  //ZIKARON = MEMORY
			{
				if (buf_rx_4[2] == 'R')  //ZIKARON = MEMORY READ COMMAD 
					{			
						if (strcmp(&buf_rx_4[0], "#ZR=ALL\r\n") == 0)
						{
							ReadFromMemoryCounter(CS_Applicator);
							ReadFromMemoryCounter(CS_Router);
							ReadFromMemoryCounter(CS_Mcu);
							
							sprintf(str, "Read memory from cs %s\r\n ", &buf_rx_4[0]); HAL_Delay(100);
							HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
							//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						
						}
						else if (strcmp(&buf_rx_4[0], "#ZR=A\r\n") == 0)
						{
							ReadFromMemoryCounter(CS_Applicator);
							//	ReadFromMemoryCounter(CS_Router);
							//	ReadFromMemoryCounter(CS_Mcu);

						//		sprintf(str,"Read memory from cs %s\r\n ",&buf_rx_4[0]);HAL_Delay(100);
						//		HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
							  // HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						}
						else if (strcmp(&buf_rx_4[0], "#ZR=R\r\n") == 0)
						{
							//	ReadFromMemoryCounter(CS_Applicator);
								  ReadFromMemoryCounter(CS_Router);
							//	ReadFromMemoryCounter(CS_Mcu);

						//		sprintf(str,"Read memory from cs %s\r\n ",&buf_rx_4[0]);HAL_Delay(100);
						//		HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
							  // HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						}
						else if (strcmp(&buf_rx_4[0], "#ZR=M\r\n") == 0)
						{
							//	ReadFromMemoryCounter(CS_Applicator);
							//	  ReadFromMemoryCounter(CS_Router);
								    ReadFromMemoryCounter(CS_Mcu);

							//		sprintf(str,"Read memory from cs %s\r\n ",&buf_rx_4[0]);HAL_Delay(100);
							//		HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
								  // HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						}
						
						
					}
					
				if (buf_rx_4[2] == 'W')  //ZIKARON = MEMORY READ COMMAD 
					{	
						if (strncmp(&buf_rx_4[0], "#ZW=M,", 6) == 0)
						{
							//	ReadFromMemoryCounter(CS_Applicator);
							//	ReadFromMemoryCounter(CS_Router);
								  ReadFromMemoryCounter(CS_Mcu);
							
							temp_memory1 = atol(&buf_rx_4[6]);
							MemoryCounter[CS_Mcu] = temp_memory1;     //0x000000;
							WriteToMemoryCounter(CS_Mcu, MemoryCounter[CS_Mcu]);
							sprintf(str, "Write to  memory  cs %s\r\n ", &buf_rx_4[0]); HAL_Delay(100);
							HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
							
							ReadFromMemoryCounter(CS_Mcu);
							//		HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						}
						
						else if (strncmp(&buf_rx_4[0], "#ZW=R,", 6) == 0)
						{
							//	ReadFromMemoryCounter(CS_Applicator);
							  	ReadFromMemoryCounter(CS_Router);
							//	ReadFromMemoryCounter(CS_Mcu);
							
							temp_memory1 = atol(&buf_rx_4[6]);
							MemoryCounter[CS_Router] = temp_memory1;     //0x000000;
							WriteToMemoryCounter(CS_Router, MemoryCounter[CS_Router]);
							sprintf(str, "Write to  memory  cs %s\r\n ", &buf_rx_4[0]); HAL_Delay(100);
							HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
							
							ReadFromMemoryCounter(CS_Router);
							//		HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						}
						else if (strncmp(&buf_rx_4[0], "#ZW=A,", 6) == 0)
						{
							ReadFromMemoryCounter(CS_Applicator);
							//	ReadFromMemoryCounter(CS_Router);
							//	ReadFromMemoryCounter(CS_Mcu);
							
							temp_memory1 = atol(&buf_rx_4[6]);
							MemoryCounter[CS_Applicator] = temp_memory1;     //0x000000;
							WriteToMemoryCounter(CS_Applicator, MemoryCounter[CS_Applicator]);
							sprintf(str, "Write to  memory  cs %s\r\n ", &buf_rx_4[0]); HAL_Delay(100);
							HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
							
							ReadFromMemoryCounter(CS_Applicator);
							//		HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
						}
						
					}
					
			}			
						
						
		} // end if #  block
	   else  if(strstr(&buf_rx_4[0], "ERROR") > 0)
		{
			sprintf(str, "\r\n###ERR-FOUND-15  %d :<<%s>> ]\r\n ", ErrorCounter, &buf_rx_4[0]); HAL_Delay(100);
			HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
			//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
					ErrorCounter++;
		}
		
		else  if(strstr(&buf_rx_4[0], "SEND FAIL") > 0)
		{
			sprintf(str, "\r\n###SEND FAIL-FOUND-25  %d :<<%s>> ]\r\n ", ErrorCounter, &buf_rx_4[0]); HAL_Delay(100);
			HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
			//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
					ErrorCounter++;
		}
		else		if(strcmp(&buf_rx_4[0], "AT\r\n") == 0)
		{
			sprintf(str, "!o.k#\r\n ", &buf_rx_4[0]); HAL_Delay(100);
			HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);
			//	  HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100);
					ErrorCounter++;
		} 
		
		// end command
		   rx4_counter = 0;
		memset(&buf_rx_4[0], 0, 99);
		IsCommand = 0;
	}
}





char CheckApplicator(void)
{ 
	if (1)
	{
		// AT25SF081 ,ADESTO
	
	// for now check it by Device Id only
	char IsDeviceIdEqual = 0;
		ChipSelect(ChipSelectNumber, HIGH);
		HAL_Delay(150);        //worked also with 10ms but need to verifay
		ChipSelectNumber = CS_Applicator; 
		//			HAL_GPIO_WritePin(CS2_LOW);
					ChipSelect(ChipSelectNumber, LOW);
		HAL_Delay(2);                                //  ID 1F-85-01-00
		SpiDataWrite[0] = 0x9F;        //get device ID // adruino code id BF-25-8E
		
		HAL_SPI_Transmit(&hspi1, SpiDataWrite, 1, 500);
		HAL_SPI_Receive(&hspi1, SpiDataRead, 3, 1000);
		
		//HAL_GPIO_WritePin(CS2_HIGH);
				ChipSelect(ChipSelectNumber, HIGH);
 
		// set device id
				Device_Id[0] = 0x1F;
		Device_Id[1] = 0x85;
		Device_Id[2] = 0x01;
		// IsDeviceIdEqual=memcmp(&SpiDataRead[0],&Device_Id[0],2); // not working check later
		if((Device_Id[0] == SpiDataRead[0]) &&  (Device_Id[1] == SpiDataRead[1]) && (Device_Id[2] == SpiDataRead[2]))
		{
			IsDeviceIdEqual = 1;
		}

		/*
				sprintf(aTxBuffer,"\r\n\r\n IsDeviceIdEqual [%d]\r\n",IsDeviceIdEqual);
					 if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000)!= HAL_OK)
			{
				Error_Handler();   
			}
	
	
			sprintf(aTxBuffer,"\r\n\r\n IsDeviceIdEqual [%d] Device id %02X-%02X-%02X-%02X\r\n ",IsDeviceIdEqual , Device_Id[0] , Device_Id[1] ,Device_Id[2],Device_Id[3]);
				 if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000)!= HAL_OK)
			{
				Error_Handler();   
			}
	
			sprintf(aTxBuffer,"\r\n\r\nCS[%d] Device id %02X-%02X-%02X-%02X\r\n",CS_Applicator , SpiDataRead[0] , SpiDataRead[1] ,SpiDataRead[2],SpiDataRead[3]);
				 if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 5000)!= HAL_OK)
			{
				Error_Handler();   
			}
			*/
		ChipSelect(ChipSelectNumber, HIGH);
		// check Applicator - end
		return(IsDeviceIdEqual);
	}
	else
	{
		char str[20];
		char IsDeviceIdEqual = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
		sprintf(str, "MIC : %d\r\n", MIC_INPUT);	
		HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);
		return (IsDeviceIdEqual);		
	}
}
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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM15_Init();
	MX_USART1_UART_Init();
	MX_USART4_UART_Init();
	/* USER CODE BEGIN 2 */
	BuzzerBip(START_BEEP);
	int i = 0;
	char key1, key2, key3, aShowTime[100];
	int counter, counterbuzzer = 0;

	char program;
	fan_counter = 0;
	int counter256 = 0;
	uint8_t c1 = 0x01;
	int c;
	char str[80];
	float Humidity_Temperature;
	uint8_t Humidity1;
	char Btm_Counter;
	char Btm_Frequemcy;
	char Btm_SW;
	char isCoverClose;
	int CountCouter;
	int PulseCounter;

	char IsApplicatorConnected = 0;     // if head is connected
	long  ApplicatorMaximumCounter = 0;    	// set by manufacturer
	long  ApplicatorCounterUsed = 0;    	    // used by customer
	float     ApplicatorCounterPercent = 0;
	int Counter, BatteryValue, GasValue, adc3, temperature;
	char SystemCheck = 0;
	int faild = 0;
	PulseCouter = 0;
	int BtmFireCounterPressed = 0, IgnoreButtonDelay = 0;		
	char  Btm_Reset, MotorRun = 0;
	Counter = 0;

	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn); 

	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn); 

	// motor 
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);     // ina
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);      // inb
					 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor
	     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor
  

// HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(USART2_IRQn);
// set uart inerrupt
   /////////////////////////////////////////////////////__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
	// __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
 //	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);




  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);			

	//-------------------------------------

		 __HAL_SPI_ENABLE(&hspi1);

	float BatteryA, BatteryB;
	BatteryA = 4.327;
	BatteryB = 108.02;
	float adc_bat = adc[ADC_Battery];
	if (adc_bat > 0)
	{
		BatteryPercent = BatteryA*adc[ADC_Battery] - BatteryB;
	}

	
				
	//----------------------------------

		
		  		sprintf(str, "\r\n\r\n\r\n*** %s   ***\r\n\r\n", i, sw_ver);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
	// 	HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100); // lcd
		HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);      // terminal
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);     //STATUS
HAL_Delay(500);
	





	
	// motor 
	//     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	
	//	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); // motor


	i = 400;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
 	
	c1 = CheckUartBuffer();
	rx4_counter = 0;
	




	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	/* Display time Format : hh:mm:ss */
	

	sprintf((char*)aShowTime, "Time%02d:%02d:%02d# ", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	HAL_UART_Transmit(&huart4, (uint8_t *)aShowTime, strlen(aShowTime), 100);
	HAL_UART_Transmit(&huart4, (uint8_t *)aShowTime, strlen(aShowTime), 100);	




	//sprintf(buf,"BRIGHTNESS LEVEL:%i",i++);  
	//Config the HDC1080 to perform acquisition separately 
	HAL_Delay(15);

	//Humidity_init(&hi2c1,Temperature_Resolution_14_bit,Humidity_Resolution_14_bit);


	//--------------
	ApplicatorMaximumCounter = 70000;
	//--------------
	
	


	// **********************************************************
	// start : set motor driver configuration .
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);     // ina
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);      // inb
		
	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);      // sel0 active high
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor set =0 motor off , 101 =on
	    // start : set motor driver configuration .
	    // **********************************************************

	      MotorRun = 0;
	BuzzerBip(START_BEEP);

	
	// set vars 
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor pwm
	PulseCouter = 0;
	PulseCouterDir = 0;
	HAL_Delay(250);
	IsBeep400 = 0;
	c = 0;

	EnablePulseCouter = 0;


	fan_counter = 0;
	//#setMem
	// HACKY
	ReadFromMemoryCounter(CS_Applicator);	
	// motor 
	//     HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	
	//	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); // motor
  //		sprintf(str,"\r\n\r\n\r\n*** Armenta ver 6.1 30/6/19   ***\r\n\r\n",i);
	//			HAL_UART_Transmit(&huart1,(uint8_t *)str,strlen(str),100);
			// 	HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100); // lcd
	//			HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);  // terminal

	i = 400;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	
	c1 = CheckUartBuffer();
	rx4_counter = 0;
	
	//    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
			/* Get the RTC current Date */
	//		HAL_RTC_GetDate(&hrtc, &sdatestructureget,RTC_FORMAT_BIN);
			/* Display time Format : hh:mm:ss */

			//sprintf(buf,"BRIGHTNESS LEVEL:%i",i++);  
			//Config the HDC1080 to perform acquisition separately 
			HAL_Delay(15);

	//Humidity_init(&hi2c1,Temperature_Resolution_14_bit,Humidity_Resolution_14_bit);




	// check Applicator - start

	//while( CheckApplicator()==0) // need to change to while
	
		if(CheckApplicator() == 1) // need to change to while
	{
		sprintf(aTxBuffer, "Applicator check pass\r\n");     // write terminal
		if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
		{
			Error_Handler();   
		}
								
	}
	else
	{
		sprintf(str, "$zAM Disconnected#");    	// write to lcd  error
  HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);			
		HAL_Delay(500);
			
		sprintf(str, "$an#");
		HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);	
		HAL_Delay(500);
		sprintf(aTxBuffer, "Applicator check faild\r\n");     // write terminal
					if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500) != HAL_OK)
		{
			Error_Handler();   
		}
		
							
	}

	// end while applicator
   //c=0x2B55fC;
   //c=0x000001;
   MsCounter = 0;
	temp_memory = 0;
	//MemoryCounter[1]=c;
	// Serial number test	
	//						c=0x000001;
	//						MsCounter=0;
	//						temp_memory=300; // counter dec


	// memory test loop


	PulseCouter = 0;

	Counter = 0;

	// **********************************************************
	// start : set motor driver configuration .
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);     // ina
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);      // inb
		
	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);      // sel0 active high
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor set =0 motor off , 101 =on
	    // start : set motor driver configuration .
	    // **********************************************************

	    MotorRun = 0;
	BuzzerBip(START_BEEP);

	
	// set vars 
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);     // motor pwm
	PulseCouter = 0;
	PulseCouterDir = 0;
	HAL_Delay(250);
	IsBeep400 = 0;
	c = 0;

	EnablePulseCouter = 0;

 
	sprintf(str, "\r\n\r\n\r\n*** Armenta ver 6.9 28/10/19   ***\r\n\r\n", i);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
	// 	HAL_UART_Transmit(&huart2,(uint8_t *)str,strlen(str),100); // lcd
		HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);      // terminal
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);     //STATUS
HAL_Delay(2500);

	
	//	c=0;
	// ************************************************************************
	// start while / program
	// ************************************************************************
	
	 //  start  load datas from memory 
	/*
						      MemoryCounter[CS_Applicator]=0x000000;
									MemoryCounter[CS_Applicator]=60000;
	    					  MemoryCounter[CS_Router]=MemoryCounter[CS_Applicator];
									MemoryCounter[CS_Mcu]=MemoryCounter[CS_Applicator];
									
									WriteToMemoryCounter(CS_Applicator,MemoryCounter[CS_Applicator]);
									// write to memory flash
									WriteToMemoryCounter(CS_Router,MemoryCounter[CS_Router]);
									WriteToMemoryCounter(CS_Mcu,MemoryCounter[CS_Mcu]);
	*/
	
	
	ReadFromMemoryCounter(CS_Applicator);
	ReadFromMemoryCounter(CS_Router);
	ReadFromMemoryCounter(CS_Mcu);
						
	temp_memory = MemoryCounter[CS_Applicator];
	MsCounter = 0;
						
	///  MemoryCounter[CS_Applicator]=0x000000;
	///  MemoryCounter[CS_Applicator]=60000;
	if(MemoryCounter[CS_Applicator] <= 0)
	{		
		// stop system 
			   sprintf(str, "$F0#");    	//pulse remaining 0
			   HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);

		sprintf(str, "\r\n  *** ERROR F0 pulse remaining is 0 \r\n");    	//pulse remaining 0
		HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);												
		MotorRun = 0;
	}
	else 	if(MemoryCounter[CS_Applicator] == 0xFFFFFF)
	{		
		// stop system 
	}
							
	sprintf(str, "\r\n  *** start loop *** \r\n");    	//pulse remaining 0
	HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);												
	//--	MotorRun=0;					
	long temp_memory2;
	//  end   load datas from memory  
	
	counter = 0;

	// MotorCommand(1);
//***************************************************************************
	// initialization
//	MemoryCounter[CS_Applicator]=99;
	
		  				ReadFromMemoryCounter(CS_Applicator);

	ApplicatorCounterUsed = MemoryCounter[CS_Applicator];
	ApplicatorCounterPercent = (float)(ApplicatorCounterUsed) / ApplicatorMaximumCounter * 100;
	temp_memory = MemoryCounter[CS_Applicator];     // save last  total memory value

// initialization lcd
	

	//	sprintf(str,"$b%d#",adc[ADC_Battery]);
		sprintf(str, "$t%d#\r\n", (int)ApplicatorCounterPercent);	
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);	
	HAL_Delay(100);
			
	sprintf(str, "$t%d#\r\n", (int)ApplicatorCounterPercent);	
	HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);	
	HAL_Delay(100);
			
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	// adcValue=HAL_ADC_GetValue(&hadc1);
  adc[0] = HAL_ADC_GetValue(&hadc);
									
	HAL_ADC_PollForConversion(&hadc, 100);
	// adcValue=HAL_ADC_GetValue(&hadc1);
  adc[1] = HAL_ADC_GetValue(&hadc);			
									
	HAL_ADC_Stop(&hadc);
	

	
	if (adc[ADC_Presure] >= ADC_Presure_27_bar)  // presure to low
		{
			sprintf(str, "$pO.K#");     // o.k
		}
	else
	{
		sprintf(str, "$pLOW#");     // low
		if(adc[ADC_Presure] <= ADC_Pressure_20_bar)
		{
			MotorCommand(0); // Edi and sela wanted a check only on start of work
			MotorRun = 0;
		}					
	}
	//	sprintf(str,"\r\n$p%d#\r\n",adc[ADC_Presure]);
	//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
	HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 100);

	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);	
	
					
	HAL_Delay(100);
	BatteryA = 0.23;
	BatteryB = 24.86;
	//		sprintf(aTxBuffer,"\r\nADC Presure %d  Battery %d", adc[0], adc[1] );
	  BatteryPercent = ewma_bat(BatteryA*adc[ADC_Battery] - BatteryB);	
	sprintf(str, "$b%d#\r\n", (int)BatteryPercent);	
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);					

					
	MsCounter = 0;
	//***************************************************************************
// PROGARM WRITE 
// **************************************************************************	
	while(1)
	{
		// START LABEL
		
		// hacky stuff

		c++;
		CheckUartBuffer();
	
		
		
		if ((c % 20) == 0)  
		{
			
			/// CheckApplicator() start
					/*
					if( CheckApplicator()==0) 
					{			
						if( CheckApplicator()==0) 
						{
									if( CheckApplicator()==0) 
										{
										MotorCommand(0);	
										MotorRun=0;
										sprintf(str,"\r\n ***** CheckApplicator 3 times  faild ******");	
										HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),50);
									
				//		sprintf(str,"$E60#");	// write to lcd  error
				//		HAL_UART_Transmit(&huart1,(uint8_t *)str,strlen(str),50);			
				//		HAL_Delay(500);
						// send error code applicator error tp screen
						sprintf(str,"$an#");
						HAL_UART_Transmit(&huart1,(uint8_t *)str,strlen(str),50);	
									
						sprintf(str,"\r\n$an#");
						HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),50);		
						HAL_Delay(500);
	
									
									
						while(1)
						{
						}
									
								
					}	
	}
		}
		*/
		// CheckApplicator() end 
		//*** write  to lcd start ***
			sprintf(str, "$c%d#\r\n", PulseCouter);
			//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
				//HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
				HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);
						
			if ((c % 100) == 0)  
			{		

			
				HAL_ADC_Start(&hadc);
				HAL_ADC_PollForConversion(&hadc, 100);
				// adcValue=HAL_ADC_GetValue(&hadc1);
			  adc[0] = HAL_ADC_GetValue(&hadc);
									
				HAL_ADC_PollForConversion(&hadc, 100);
				// adcValue=HAL_ADC_GetValue(&hadc1);
			  adc[1] = HAL_ADC_GetValue(&hadc);			
									
				HAL_ADC_Stop(&hadc);
			 
			
				
				//		sprintf(aTxBuffer,"\r\nADC Presure %d  Battery %d", adc[0], adc[1] );
				  BatteryPercent = ewma_bat(BatteryA*adc[ADC_Battery] - BatteryB);
			
				// add factor percent - 09-10-2019 add by eddy
				  if(MotorRun == 1)
				{
					BatteryPercent = ewma_bat(BatteryA*adc[ADC_Battery] - BatteryB + 17);
				}
				else
				{
					BatteryPercent = ewma_bat(BatteryA*adc[ADC_Battery] - BatteryB);
				}


					
				//		sprintf(aTxBuffer,"\r\nPr[%d] ,Bat[%d]", adc[0], adc[1] );
					//	if(HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 100)!= HAL_OK)
					//		{
					//				Error_Handler();   
					//			}

			
						
							 if(adc[ADC_Presure] >= ADC_Presure_27_bar)  // presure to low
				{
					sprintf(str, "$pO.K#\r\n");       // o.k
				}
				else
				{
					sprintf(str, "$pLOW#\r\n");       // low
					if(adc[ADC_Presure] <= ADC_Pressure_20_bar)
					{
						MotorCommand(0);  // Edi and sela wanted a check only on start of work
						MotorRun = 0;
					}	
						
				}
				//	sprintf(str,"\r\n$p%d#\r\n",adc[ADC_Presure]);
				//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
				//	HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);

				   HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);			
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);					

				sprintf(str, "$b%d#\r\n", (int)BatteryPercent);	
				HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);
				HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);					

			}				
					
				
			ApplicatorCounterUsed = (temp_memory - MsCounter);
			
			if (ApplicatorCounterUsed >= ApplicatorMaximumCounter)
			{
				ApplicatorCounterPercent = 0;
			}
			else
			{
				ApplicatorCounterPercent = (float)(ApplicatorCounterUsed)  / ApplicatorMaximumCounter * 100;
			}		
		
			// applicator remainder sub 1000 
			if((temp_memory - MsCounter) <= 1000)
			{
			
				if (((temp_memory - MsCounter) % 200) == 0)
				{
					sprintf(str, "$F%d#\r\n", (temp_memory - MsCounter));      	//pulse remaining 0
					HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);
				}
			}
		
			sprintf(str, "$t%3.0f#\r\n", ApplicatorCounterPercent);	
			HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);			
			
			
			
			sprintf(str, "ADC[pressure] : %d\r\n", adc[ADC_Presure]);	
			HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);	
			if ((c % 1000) == 0)  
			{		
				//	sprintf(str,"$b%d#",adc[ADC_Battery]);
					sprintf(str, "$t%d#\r\n", (int)ApplicatorCounterPercent);	
				HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);			
			}			

	
			/** PiazoCouter lcd update ** */			
			if ((c % 115) == 0)  
			{		
				if (MotorRun == 1) 
				{
					//-	 sprintf(str,"\r\n");			
					//-  HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),50);
									if(PiazoCouter >= 1)
					{
						//	sprintf(str,"$b%d#",adc[ADC_Battery]);
							sprintf(str, "$ay#\r\n");
									
					}
					else
					{
						MotorCommand(0);
						MotorRun = 0;
						//sprintf(str, "$zno piezo sig#"); // This is the piezo error message
						//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
						//hal_delay(2000);
						//sprintf(str, "$R#");
						//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
						sprintf(str, "$an#\r\n");

					}
					PiazoCouter = 0;
					HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);	
					HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);
					
					//	sprintf(str,"\r\n******************************************");			
					//  HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),50);
				}
				else
				{
					PiazoCouter = 0;
					sprintf(str, "$an#\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);	
										
					//  HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),50);
				}	
			} // end if c%250
			
		
	

						// presure =0.0383*ADC[0]-4.981
			if(adc[ADC_Presure] >= ADC_Presure_27_bar)  // presure to low
			{
				//		sprintf(str,"\r\nPr %d [0.k] Battery %4.2f   Counter.Head %4.2f", adc[ADC_Presure], BatteryPercent,ApplicatorCounterPercent );
			}
			else
			{
				//		sprintf(str,"\r\nPr %d [low] Battery %4.2f   Counter.Head %4.2f ", adc[ADC_Presure], BatteryPercent,ApplicatorCounterPercent  );

			}
			//	HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
		//*** write  to lcd end ***

			// *******************************
			// *** write to memory   start ***
		  // *******************************
							// write to memories / flash
					//	MotorRun=1;
					//	MsCounter++;


					//if ( MotorRun==1) 
							if(0) 
			{ 
				//  sprintf(aTxBuffer,"\r\n  PulseCouter[%d] ",MsCounter );
		 // 	HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);

					temp_memory2 = MemoryCounter[CS_Applicator] - MsCounter;      // MsCounter is updted by external interrupt
				  MsCounter = 0;
				// set memory

				//			sprintf(aTxBuffer,"temp_memory2[%ld]\r\n ",temp_memory2 );
				// 	  HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
							   // need to be inprove 
						
					   if((temp_memory2) > 0) 
				{	
					MemoryCounter[CS_Applicator] = temp_memory2 = 0;
				}
				else 
				{
					MemoryCounter[CS_Applicator] = 0x0;
				}
			
					
				//			sprintf(aTxBuffer," [MemoryCounter=%ld] \r\n",MemoryCounter[CS_Applicator] );
		//  	  HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
		
	     
					
					
					
				if(((PulseCouter % 10) == 0) || MemoryCounter[CS_Applicator] == 0x0)
				{
					// set memory
					//MsCounter=0;
						
									
					  					MemoryCounter[CS_Router] = MemoryCounter[CS_Applicator];
					MemoryCounter[CS_Mcu] = MemoryCounter[CS_Applicator];
							
					if (MemoryCounter[CS_Applicator] > 0) 
					{	
						// write to memory flash
						WriteToMemoryCounter(CS_Applicator, MemoryCounter[CS_Applicator]);
						WriteToMemoryCounter(CS_Router, MemoryCounter[CS_Router]);
						WriteToMemoryCounter(CS_Mcu, MemoryCounter[CS_Mcu]);
						temp_memory = MemoryCounter[CS_Applicator];
					}	
					else
					{
						// stop motor and then write
						//--	MotorCommand(0);
			//	--	MotorRun=0;
								
							// add delay to motor coil ...back emp and spikes
							   MemoryCounter[CS_Applicator] = 0x0;
						MemoryCounter[CS_Router] = 0x0;
						MemoryCounter[CS_Mcu] = 0x0;
						// write to memory flash
						MemoryCounter[CS_Applicator] = MsCounter;
						WriteToMemoryCounter(CS_Applicator, MemoryCounter[CS_Applicator]);
						WriteToMemoryCounter(CS_Router, MemoryCounter[CS_Router]);
						WriteToMemoryCounter(CS_Mcu, MemoryCounter[CS_Mcu]);
						//		sprintf(aTxBuffer,"\r\n **** 0 HIT LEFTS \r\n" );
						//  	HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);

								
																	//write to lcd 
												sprintf(str, "$F0#\r\n");      	//pulse remaining 0
												HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);		
						//	MsCounter=0;
						//	--			MotorRun=0;	
									  temp_memory2 = 0;
					}
					//	temp_memory2=	908098;
				sprintf(aTxBuffer, "\r\n **** read from memories %ld ****\r\n", temp_memory2);
					HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
					
						
					WriteToMemoryCounter(CS_Applicator, MsCounter);
					WriteToMemoryCounter(CS_Router, MsCounter);
					WriteToMemoryCounter(CS_Mcu, MsCounter);
					MemoryCounter[CS_Applicator] = 0x0;
					MemoryCounter[CS_Router] = 0x0;
					MemoryCounter[CS_Mcu] = 0x0;
						
					ReadFromMemoryCounter(CS_Applicator);
					ReadFromMemoryCounter(CS_Router);
					ReadFromMemoryCounter(CS_Mcu);

				
					
					if (Compare(MemoryCounter[CS_Applicator], MsCounter) == 0)
					{
						//		sprintf(aTxBuffer,"\r\n ****Faild compare CS_Applicator ****\r\n" );
				//    HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
					
					}
						
								
					if (Compare(MemoryCounter[CS_Router], MsCounter) == 0)
					{
						//		sprintf(aTxBuffer,"\r\n ****Faild compare CS_Router ****\r\n" );
				//    HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
					
					}
								
						
					if (Compare(MemoryCounter[CS_Mcu], MsCounter) == 0)
					{
						//		sprintf(aTxBuffer,"\r\n ****Faild compare CS_Mcu ****\r\n" );
				//    HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
					
					}
						
						
				}
						
	
			}	
		
			if ((PulseCouter > 100) &&  (PulseCouter % 25) == 0) 
			{
				c1++;
			} // end PulseCouter
		// **************************************
			// *** End to write to memory   start ***
		  // **************************************
		} // if (  (c%20)==0)  

		
		// SET DIRECTION TO COUNT
		if(PulseCouterDir == 0) {sprintf(str, "Up"); }
		else if(PulseCouterDir == 1) { sprintf(str, "Down"); }
		;
	
		if ((c % 20) == 0)
		{
			sprintf(aTxBuffer, "\r\n%ld %d  %s Counter,PulseCouter=%d ,   PiezoCouter=%d  MotorRun=%d  $c%d# M[%d]\r\n", c, Counter, str, PulseCouter, PiazoCouter, MotorRun, PulseCouter, MsCounter);
			HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
			//	HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
			Counter++;
			if (counter > 10000)
			{
				//HAL_Delay(100);
				Counter = 0;
			}
		}
		
		// close motor if memory is equal to zero
					
		if((temp_memory - MsCounter) < 0)
		{
			MotorCommand(0);	
			MotorRun = 0;
			HAL_Delay(50);
			sprintf(aTxBuffer, "\r\n **** Applicator memory 0 ***\r\n");
			HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
			MemoryCounter[CS_Applicator] = 0;
			WriteToMemoryCounter(CS_Applicator, MemoryCounter[CS_Applicator]);
			WriteToMemoryCounter(CS_Router, MemoryCounter[CS_Applicator]);
			WriteToMemoryCounter(CS_Mcu, MemoryCounter[CS_Applicator]);
			temp_memory = MemoryCounter[CS_Applicator];
			ReadFromMemoryCounter(CS_Applicator);


			sprintf(str, "$F0#\r\n");      	//pulse remaining 0
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
			HAL_Delay(10000);
		}
	
			


		
		Btm_Reset = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		if (Btm_Reset)    //  Btm_Reset==1     
			{
				// SANITY TEST
				HAL_Delay(15);
				Btm_Reset = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
				if (Btm_Reset == 1)       // for debauncing
					{
						BuzzerBip(KEY_PRESS);
						if (PulseCouterDir == 0) {	IsBeep400 = 0; PulseCouterDir = 1; PulseCouter = 400; }
						else if (PulseCouterDir == 1) {IsBeep400 = 0; PulseCouterDir = 0; PulseCouter = 0; }
						

						
						sprintf(aTxBuffer, "\r\n **** Btn reset preesed ***\r\n");
						HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
						
						Btm_Reset = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
						if (Btm_Reset == 1)       // for debauncing
							{
								HAL_Delay(150);
							}
						Btm_Reset = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
						if (Btm_Reset == 1)       // for debauncing
							{
								HAL_Delay(550);
							}
							 
					}
			}
		
		// **** Start  button motor pressed	/ Btn Fire_Switch preesed		
		Btm_SW = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if (Btm_SW == 1)
		{

			HAL_Delay(15);
			Btm_SW = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			if (Btm_SW == 1)
			{	
				BuzzerBip(KEY_PRESS);
				
				sprintf(aTxBuffer, "\r\n **** Btn Fire_Switch preesed ***\r\n");
				HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
	
			
				//	BtmFireCounterPressed++;
			
		//		if (BtmFireCounterPressed >=5)
			
					//IgnoreButtonDelay++;
				if(MotorRun == 0)
				{
					MsCounter = 0;
					sprintf(str, "\r\nStart  motor write to %ld\r\n", MsCounter);
					HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);				
					if(CheckApplicator() == 1) 
					{	

						if (adc[ADC_Presure] >= ADC_Presure_27_bar)
						{
							MotorCommand(1);
							MotorRun = 1;
						}
					}
					else 
					{
						sprintf(str, "$zAM Disconnected#\r\n");
						HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 50);	
					}	
					//	}
						
			//	HAL_Delay(800);
				}
				else
				{
					
					// CLOSE MOTOR  OFF
					
					MotorCommand(0);	
					MotorRun = 0;
					HAL_Delay(50);
					sprintf(str, "\r\nclose  motor write to %ld\r\n", MsCounter);

					MemoryCounter[CS_Applicator] = MsCounter;
					MemoryCounter[CS_Router] = MsCounter;
					MemoryCounter[CS_Mcu] = MsCounter;
					
					HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);
	
					
					// need to check memory compare 
				   // check memory is not negativ
				   if((MemoryCounter[CS_Applicator] - MsCounter) < 0)
					{
						MemoryCounter[CS_Applicator] = 0;
					}
					
					ReadFromMemoryCounter(CS_Applicator);
					MemoryCounter[CS_Applicator] = MemoryCounter[CS_Applicator] - MsCounter;
					sprintf(str, "\r\write to  memory \r\n");
					HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);
					temp_memory2 = MemoryCounter[CS_Applicator];      // save memory before write for compare
					
					WriteToMemoryCounter(CS_Applicator, MemoryCounter[CS_Applicator]);
					WriteToMemoryCounter(CS_Router, MemoryCounter[CS_Router]);
					WriteToMemoryCounter(CS_Mcu, MemoryCounter[CS_Mcu]);
					sprintf(str, "\r\Read from  memory \r\n");
					HAL_UART_Transmit(&huart4, (uint8_t *)str, strlen(str), 50);
					
					ReadFromMemoryCounter(CS_Applicator);
					
					if (Compare(MemoryCounter[CS_Applicator], temp_memory2) == 0)
					{
						sprintf(aTxBuffer, "\r\n ****Faild compare CS_Applicator ****\r\n");
						HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 50);
						MemoryCounter[CS_Applicator] = temp_memory2;
						WriteToMemoryCounter(CS_Applicator, MemoryCounter[CS_Applicator]);
						WriteToMemoryCounter(CS_Router, MemoryCounter[CS_Router]);
						WriteToMemoryCounter(CS_Mcu, MemoryCounter[CS_Mcu]);
									
						ReadFromMemoryCounter(CS_Applicator);
					}
					
					temp_memory = MemoryCounter[CS_Applicator];      // save last  total memory value
					MsCounter = 0;
				}

			
				Btm_SW = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
				if (Btm_SW == 1)
				{
					HAL_Delay(250);
				}
			
			
				Btm_SW = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
				if (Btm_SW == 1)
				{
					HAL_Delay(250);
				}
			
			
				//  sprintf(aTxBuffer,"\r\n  Btm_SW \r\n");
					//	HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500);
			}
		}  // end button motor pressed
		else
		{
			//   sprintf(aTxBuffer,"\r\n not Btm_SW \r\n");
			  //	HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500);

		}
		CheckUartBuffer();
		
		if (PulseCouterDir == 0) // count up
			{
			
				if (PulseCouter > 0)
				{
					if ((PulseCouter % 200) == 0)
					{
						BuzzerBip(BEEP_COUNTER_200);
					}
				}		
			}
	
			
		if (PulseCouterDir == 1) // count down
		
			{
				if (PulseCouter < 400)
				{
					if ((PulseCouter != 0) &&  ((PulseCouter % 200) == 0))
					{
						BuzzerBip(BEEP_COUNTER_200);
					}
									
					else  if (PulseCouter <= 0)
					{
									
										
						if (MotorRun == 1)
						{
							// close motor
							MotorCommand(0);	
							MotorRun = 0;
							BuzzerBip(BEEP_COUNTER_200);	
							PulseCouterDir = 0;
						}
											
					}
			
									
				}		
			}
				
	


	
	
		if ((c % 10) == 0)
		{
			//	sprintf(str,"$%d#",PulseCouter);
		
		
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			//HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
			//HAL_UART_Transmit(&huart4,(uint8_t *)str,strlen(str),100);
		}
		HAL_Delay(10);	
	
		if (Tret == 1)
		{
			if (PulseCouter >= 10)
			{
				Tret = 0;
				sprintf(aTxBuffer, "\r\n --- Stop Cycle [motor stop] \r\n  ");
				HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, strlen((char*)aTxBuffer), 500);
				HAL_Delay(10);
				MotorCommand(0);
				//__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, MOTOR_1_PWM_OFF); // start motor

			}
		}



		sprintf(str, "HERE!");
		//HAL_UART_Transmit(&huart4, (uint8_t*)str, strlen((char*)str), 500);
	
		// ************************************************************************
		// End while / program
		// ************************************************************************

	}
	
	/* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_RTC;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
	*/
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_10B;
	hadc.Init.DataAlign = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	

	
	
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted. 
	*/

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted. 
	*/

	sConfig.Channel = ADC_CHANNEL_11;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only 
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 6;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 299;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 21;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 12;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 440;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */

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
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

	/* USER CODE BEGIN USART4_Init 0 */

	/* USER CODE END USART4_Init 0 */

	/* USER CODE BEGIN USART4_Init 1 */

	/* USER CODE END USART4_Init 1 */
	huart4.Instance = USART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART4_Init 2 */

	/* USER CODE END USART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
		INA_Pin|INB_Pin|GPIO_PIN_4|GPIO_PIN_5 
	                        |CS1_Pin,
		GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, STATUS_Pin | I03_Pin | IO1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Buzzer_Pin | CS3_Pin | CS4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Btm_Counter_Pin */
	GPIO_InitStruct.Pin = Btm_Counter_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Btm_Counter_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ADC_GAS_Pin ADC_BATTERY_Pin */
	GPIO_InitStruct.Pin = ADC_GAS_Pin | ADC_BATTERY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : INA_Pin INB_Pin PC4 PC5 
	                         CS1_Pin */
	GPIO_InitStruct.Pin = INA_Pin | INB_Pin | GPIO_PIN_4 | GPIO_PIN_5 
	                        | CS1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : Btm_SW_Pin */
	GPIO_InitStruct.Pin = Btm_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Btm_SW_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : STATUS_Pin I03_Pin IO1_Pin */
	GPIO_InitStruct.Pin = STATUS_Pin | I03_Pin | IO1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC_IN7_Pin */
	GPIO_InitStruct.Pin = ADC_IN7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_IN7_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : INT_F2_Pin INT_F1_Pin PiazoCouter_Pin IntPulse_Pin */
	GPIO_InitStruct.Pin = INT_F2_Pin | INT_F1_Pin | PiazoCouter_Pin | IntPulse_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Buzzer_Pin CS3_Pin CS4_Pin */
	GPIO_InitStruct.Pin = Buzzer_Pin | CS3_Pin | CS4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Btm_Frequemcy_Pin */
	GPIO_InitStruct.Pin = Btm_Frequemcy_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Btm_Frequemcy_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PulseCounter_Pin */
	GPIO_InitStruct.Pin = PulseCounter_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PulseCounter_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CS2_Pin */
	GPIO_InitStruct.Pin = CS2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

