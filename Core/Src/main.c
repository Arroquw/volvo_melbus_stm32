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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char byte;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MD_RESPONSE_ID 0xDE
#define MD_MASTER_ID 0xDF
#define MD_BASE_ID 0xD8
#define MD_ALT_ID 0xD9

#define CDC_RESPONSE_ID 0xEE
#define CDC_MASTER_ID 0xEF
#define CDC_BASE_ID 0xE8
#define CDC_ALT_ID 0xE9

#define MRB_1 {3, 0x00, 0x1C, 0xEC}            //Master Request Broadcast version 1
#define MRB_2 {3, 0x00, 0x1E, 0xEC}            //Master Request Broadcast version 2 (maybe this is second init seq?)
#define MI {3, 0x07, 0x1A, 0xEE}               //Main init sequence
#define SI {3, 0x00, 0x1E, 0xED}               //Secondary init sequence (turn off ignition, then on)
#define IGN_OFF {3, 0x00, 0x18, 0x12}           //this is the last message before HU goes to

#define MD_CIR {3, MD_BASE_ID, 0x1E, 0xEF}             //Cartridge info request. Respond with 6 bytes
#define MD_TIR {5, MD_ALT_ID, 0x1B, 0xE0, 0x01, 0x08}  //track info req. resp 9 bytes
#define MD_NXT {5, MD_BASE_ID, 0x1B, 0x2D, 0x40, 0x01} //next track.
#define MD_PRV {5, MD_BASE_ID, 0x1B, 0x2D, 0x00, 0x01} //prev track
#define MD_CHG {3, MD_BASE_ID, 0x1A, 0x50}             //change cd
#define MD_PUP {3, MD_BASE_ID, 0x19, 0x2F}             //power up. resp ack (0x00).
#define MD_PDN {3, MD_BASE_ID, 0x19, 0x22}             //power down. ack (0x00)
#define MD_FFW {3, MD_BASE_ID, 0x19, 0x29}             //FFW. ack
#define MD_FRW {3, MD_BASE_ID, 0x19, 0x26}             //FRW. ack
#define MD_SCN {3, MD_BASE_ID, 0x19, 0x2E}             //scan mode. ack
#define MD_RND {3, MD_BASE_ID, 0x19, 0x52}             //random mode. ack
#define MD_NU {3, MD_BASE_ID, 0x1A, 0x50}              //not used

#define CDC_CIR {3, CDC_BASE_ID, 0x1E, 0xEF}             //Cartridge info request. Respond with 6 bytes
#define CDC_TIR {5, CDC_ALT_ID, 0x1B, 0xE0, 0x01, 0x08}  //track info req. resp 9 bytes
#define CDC_NXT {5, CDC_BASE_ID, 0x1B, 0x2D, 0x40, 0x01} //next track.
#define CDC_PRV {5, CDC_BASE_ID, 0x1B, 0x2D, 0x00, 0x01} //prev track
#define CDC_CHG {3, CDC_BASE_ID, 0x1A, 0x50}             //change cd
#define CDC_PUP {3, CDC_BASE_ID, 0x19, 0x2F}             //power up. resp ack (0x00).
#define CDC_PDN {3, CDC_BASE_ID, 0x19, 0x22}             //power down. ack (0x00)
#define CDC_FFW {3, CDC_BASE_ID, 0x19, 0x29}             //FFW. ack
#define CDC_FRW {3, CDC_BASE_ID, 0x19, 0x26}             //FRW. ack
#define CDC_SCN {3, CDC_BASE_ID, 0x19, 0x2E}             //scan mode. ack
#define CDC_RND {3, CDC_BASE_ID, 0x19, 0x52}             //random mode. ack
#define CDC_NU {3, CDC_BASE_ID, 0x1A, 0x50}              //not used
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
uint32_t DWT_Delay_Init(void);
__STATIC_INLINE void DWT_Delay_ms(volatile uint32_t au32_milliseconds)
{
	uint32_t au32_initial_ticks = DWT->CYCCNT;
	uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
	au32_milliseconds *= au32_ticks;
	while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
	uint32_t au32_initial_ticks = DWT->CYCCNT;
	uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
	au32_microseconds *= au32_ticks;
	while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

static void melbusInitReq(void);
static void SetPinToInput(uint16_t);
static void SetPinToOutput(uint16_t);
static void SetClockToInt(void);
void SendByteToMelbus(void);
void SendText(void);
void SendTrackInfo(byte trackInfo[]);
void SendCartridgeInfo(byte trackInfo[]);
void changeCD(byte trackInfo[], byte *disk);
void fixTrack(byte *disk);
void nextTrack();
void prevTrack();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_InitTypeDef GPIO_InitStructInput = { .Mode = GPIO_MODE_INPUT, .Pull = GPIO_NOPULL };
GPIO_InitTypeDef GPIO_InitStructOutput = { .Mode = GPIO_MODE_OUTPUT_OD };
GPIO_InitTypeDef GPIO_InitStructIT = { .Pin = MELBUS_CLOCK_Pin, .Mode = GPIO_MODE_IT_RISING, .Pull = GPIO_NOPULL };
volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;
byte byteToSend = 0;
byte track = 1;
byte md = 1;
byte cd = 1;
byte textHeader[] = {0xFB, 0xD8, 0xFA, 0x00 };
byte textRow = 2;
byte customText[36] = "visualapproach";
byte mdTrackInfo[] = {0x00, 0x02, 0x00, 0x01, 0x80, 0x01, 0x0C, 0xCC, 0xCC};
byte mdCartridgeInfo[] = {0x80, 0x00, 0x0F, 0x04, 0x00, 0x0F};
byte cdcTrackInfo[] = {0x00, 0x02, 0x00, 0x01, 0x80, 0x01, 0xC7, 0x0A, 0x02};
byte cdcCartridgeInfo[] = {0x00, 0xFC, 0xFF, 0x4A, 0xFC, 0xFF};
byte startByte = 0x08; //on powerup - change cdcTrackInfo[1] & [8] to this
byte stopByte = 0x02; //same on powerdown

enum {
	E_MRB_1,  // 0 now we are master and can send stuff (like text) to the display!
	E_MI,     // 1 main init
	E_SI,     // 2 sec init (00 1E ED respond 0xC5 !!)
	E_MRB_2,  // 5 alternative master req bc
	E_IGN_OFF, // 21
	E_MD_CIR, // 22
	E_MD_TIR, // 23
	E_MD_NXT, // 24
	E_MD_PRV, // 25
	E_MD_CHG, // 26
	E_MD_PUP, // 27
	E_MD_PDN, // 28
	E_MD_FFW, // 29
	E_MD_FRW, // 30
	E_MD_SCN, // 31
	E_MD_RND, // 32
	E_MD_NU,   // 33
	E_CDC_CIR, // 34
	E_CDC_TIR, // 35
	E_CDC_NXT, // 36
	E_CDC_PRV, // 37
	E_CDC_CHG, // 38
	E_CDC_PUP, // 39
	E_CDC_PDN, // 40
	E_CDC_FFW, // 41
	E_CDC_FRW, // 42
	E_CDC_SCN, // 43
	E_CDC_RND, // 44
	E_CDC_NU,  // 45
	E_LIST_MAX
};
//This list can be quite long. We have approx 700 us between the received bytes.
const byte commands[E_LIST_MAX][7] = {
		[E_MRB_1] = MRB_1,  // 0 now we are master and can send stuff (like text) to the display!
		[E_MI] = MI,     // 1 main init
		[E_SI] = SI,     // 2 sec init (00 1E ED respond 0xC5 !!)
		[E_MRB_2] = MRB_2,  // 3 alternative master req bc
		[E_IGN_OFF] = IGN_OFF, // 4
		[E_MD_CIR] = MD_CIR, // 5
		[E_MD_TIR] = MD_TIR, // 6
		[E_MD_NXT] = MD_NXT, // 7
		[E_MD_PRV] = MD_PRV, // 8
		[E_MD_CHG] = MD_CHG, // 9
		[E_MD_PDN] = MD_PDN, // 28
		[E_MD_FFW] = MD_FFW, // 29
		[E_MD_FRW] = MD_FRW, // 30
		[E_MD_SCN] = MD_SCN, // 31
		[E_MD_RND] = MD_RND, // 32
		[E_MD_NU] = MD_NU,   // 33
		[E_CDC_CIR] = CDC_CIR, // 34
		[E_CDC_TIR] = CDC_TIR, // 35
		[E_CDC_NXT] = CDC_NXT, // 36
		[E_CDC_PRV] = CDC_PRV, // 37
		[E_CDC_CHG] = CDC_CHG, // 38
		[E_CDC_PUP] = CDC_PUP, // 39
		[E_CDC_PDN] = CDC_PDN, // 40
		[E_CDC_FFW] = CDC_FFW, // 41
		[E_CDC_FRW] = CDC_FRW, // 42
		[E_CDC_SCN] = CDC_SCN, // 43
		[E_CDC_RND] = CDC_RND, // 44
		[E_CDC_NU] = CDC_NU   // 45
};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static byte lastByte = 0;
	static bool powerOn = true;
	static long HWTicks = 0;      //age since last BUSY switch
	static long ComTicks = 0;     //age since last received byte
	static long ConnTicks = 0;    //age since last message to SIRIUS SAT
	static long timeout = 1000000; //should be around 10-20 seconds
	static byte matching[E_LIST_MAX];     //Keep track of every matching byte in the commands array
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	DWT_Delay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	melbusInitReq();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		bool busy = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
		byte byteCounter = 1;
		byte melbus_log[99] = {[0 ... 98] = 0xFF};
		HWTicks++;
		if (powerOn) {
			ComTicks++;
			ConnTicks++;
		} else {
			ComTicks = 1;
			ConnTicks = 1;
		}

		while (!busy) {
			HWTicks = 0;
			if (byteIsRead) {
				byteIsRead = false;
				lastByte = melbus_ReceivedByte;
				melbus_log[byteCounter - 1] = lastByte;
				ComTicks = 0; //reset age
				for (byte cmd = 0; cmd < E_LIST_MAX; cmd++) {
					if (lastByte == commands[cmd][byteCounter]) {
						matching[cmd]++;
						if ((matching[cmd] == commands[cmd][0]) && (byteCounter == commands[cmd][0])) {
							ConnTicks = 0;  //reset age
							switch (cmd) {
							case E_MRB_1:
								while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
									if (byteIsRead) {
										byteIsRead = false;
										if (melbus_ReceivedByte == MD_MASTER_ID) {
											byteToSend = MD_MASTER_ID;
											SendByteToMelbus();
											SendText();
											break;
										}
									}
								}
								break;
							case E_MI: /* Intentional fall-through */
							case E_SI:
								while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
									if(byteIsRead) {
										byteIsRead = false;
										if (melbus_ReceivedByte == MD_BASE_ID) {
											byteToSend = MD_RESPONSE_ID;
											SendByteToMelbus();
										}
										if (melbus_ReceivedByte == CDC_BASE_ID) {
											byteToSend = CDC_RESPONSE_ID;
											SendByteToMelbus();
										}
									}
								}
								break;
							case E_MRB_2:
								while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
									if (byteIsRead) {
										byteIsRead = false;
										if (melbus_ReceivedByte == MD_MASTER_ID) {
											byteToSend = MD_MASTER_ID;
											SendByteToMelbus();
											SendText();
											break;
										}
										//if (melbus_ReceivedByte == MASTER_ID) {
											//  byteToSend = MASTER_ID;
											//  SendByteToMelbus();
											//  SendText();
										//  SendTextI2c();
										//  break;
										//}
									}
								}
								//Serial.println("MRB 2");

								break;
							case E_IGN_OFF:
				                powerOn = false;
								break;
							case E_MD_CIR:
								SendCartridgeInfo(mdCartridgeInfo);
								break;
							case E_MD_TIR:
								SendTrackInfo(mdTrackInfo);
								break;
							case E_MD_NXT:
				                track++;
				                fixTrack(&md);
				                mdTrackInfo[5] = track;
				                nextTrack();
								break;
							case E_MD_PRV:
				                track--;
				                fixTrack(&md);
				                mdTrackInfo[5] = track;
				                prevTrack();
								break;
							case E_MD_CHG:
				                changeCD(mdTrackInfo, &md);
								break;
							case E_MD_PUP:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_MD_PDN:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_MD_FFW:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_MD_FRW:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_MD_SCN:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_MD_RND:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_MD_NU:
								break;
							case E_CDC_CIR:
								SendCartridgeInfo(cdcCartridgeInfo);
								break;
							case E_CDC_TIR:
								SendTrackInfo(cdcTrackInfo);
								break;
							case E_CDC_NXT:
				                track++;
				                fixTrack(&cd);
				                cdcTrackInfo[5] = track;
				                nextTrack();
								break;
							case E_CDC_PRV:
				                track--;
				                fixTrack(&cd);
				                cdcTrackInfo[5] = track;
				                prevTrack();
								break;
							case E_CDC_CHG:
								changeCD(cdcTrackInfo, &cd);
								break;
							case E_CDC_PUP:
								byteToSend = 0x00;
								SendByteToMelbus();
								cdcTrackInfo[1] = startByte;
								cdcTrackInfo[8] = startByte;
								break;
							case E_CDC_PDN:
				                byteToSend = 0x00;
				                SendByteToMelbus();
				                cdcTrackInfo[1] = stopByte;
				                cdcTrackInfo[8] = stopByte;
								break;
							case E_CDC_FFW:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_CDC_FRW:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_CDC_SCN:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_CDC_RND:
				                byteToSend = 0x00;
				                SendByteToMelbus();
								break;
							case E_CDC_NU:
								break;
							}
							break;
						}
					}
				}
				byteCounter++;
			}
			busy = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
		}
		if (!ComTicks && ConnTicks) {
			for (byte b = 0; b < byteCounter-1; b++) {
				int x = melbus_log[b];
				if (x == 0xFF) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				}
			}
		}
		if (ComTicks > timeout) {
			ComTicks = 0;
			melbusInitReq();
		}
		if (HWTicks > timeout) {
			HWTicks = 0;
		}
		melbus_Bitposition = 7;
		for (byte i = 0; i < E_LIST_MAX; i++) {
			matching[i] = 0;
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MELBUS_CLOCK_Pin */
  GPIO_InitStruct.Pin = MELBUS_CLOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MELBUS_CLOCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MELBUS_DATA_Pin MELBUS_BUSY_Pin */
  GPIO_InitStruct.Pin = MELBUS_DATA_Pin|MELBUS_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Notify HU that we want to trigger the first initiate procedure to add a new device
//(CD-CHGR/SAT etc) by pulling BUSY line low for 1s
static void melbusInitReq() {
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	// Wait until Busy-line goes high (not busy) before we pull BUSY low to request init
	while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {}
	DWT_Delay_us(20);

	SetPinToOutput(MELBUS_BUSY_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_BUSY_Pin, GPIO_PIN_RESET);

	DWT_Delay_ms(1000);

	HAL_GPIO_WritePin(GPIOA, MELBUS_BUSY_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_BUSY_Pin);

	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void SendByteToMelbus(void) {
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	SetPinToOutput(MELBUS_DATA_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_CLOCK_Pin);
	for (char i = 7; i >= 0; i--)
	{
		while (HAL_GPIO_ReadPin(GPIOA, MELBUS_CLOCK_Pin) == GPIO_PIN_SET) {} //wait for low clock
		if (byteToSend & (1 << i)) {
			HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_RESET);
		}
		if (i == 0) {
			break;
		}
		while (HAL_GPIO_ReadPin(GPIOA, MELBUS_CLOCK_Pin) == GPIO_PIN_RESET) {}  //wait for high clock

	}
	//Let the value be read by the HU
	DWT_Delay_us(20);

	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_DATA_Pin);
    SetClockToInt();
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

//This method generates our own clock. Used when in master mode.
void SendByteToMelbus2(void) {
	DWT_Delay_us(700);
	//For each bit in the byte
	//char, since it will go negative. byte 0..255, char -128..127
	//int takes more clockcycles to update on a 8-bit CPU.
	for (char i = 7; i >= 0; i--)
	{
		DWT_Delay_us(7);
		HAL_GPIO_WritePin(GPIOA, MELBUS_CLOCK_Pin, GPIO_PIN_RESET);
		if (byteToSend & (1 << i)) {
			HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_RESET);
		}
		//wait for output to settle
		DWT_Delay_us(5);
		HAL_GPIO_WritePin(GPIOA, MELBUS_CLOCK_Pin, GPIO_PIN_SET);
		//wait for HU to read the bit
	}
	DWT_Delay_us(20);
}

void SendText(void) {
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	//Convert datapin and clockpin to output
	//pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToOutput(MELBUS_DATA_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_CLOCK_Pin, GPIO_PIN_SET);
	SetPinToOutput(MELBUS_CLOCK_Pin);

	//send header
	for (byte b = 0; b < 4; b++) {
		byteToSend = textHeader[b];
		SendByteToMelbus2();
	}

	//send which row to show it on
	byteToSend = textRow;
	SendByteToMelbus2();

	//send text
	for (byte b = 0; b < 36; b++) {
		byteToSend = customText[b];
		SendByteToMelbus2();
	}

	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_DATA_Pin);

	SetClockToInt();
}

void SendTrackInfo(byte trackInfo[]) {
  for (byte i = 0; i < 9; i++) {
    byteToSend = trackInfo[i];
    SendByteToMelbus();
  }
}

void SendCartridgeInfo(byte cartridgeInfo[]) {
  for (byte i = 0; i < 6; i++) {
    byteToSend = cartridgeInfo[i];
    SendByteToMelbus();
  }
}

static void SetClockToInt(void) {
	HAL_GPIO_Init(MELBUS_CLOCK_GPIO_Port, &GPIO_InitStructIT);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

static void SetPinToInput(uint16_t pin) {
	GPIO_InitStructInput.Pin = pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructInput);
}

static void SetPinToOutput(uint16_t pin) {
	GPIO_InitStructOutput.Pin = pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructOutput);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin == GPIO_PIN_2) {
		//Read status of Datapin and set status of current bit in recv_byte
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
			melbus_ReceivedByte |= (1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "1"
		}
		else {
			melbus_ReceivedByte &= ~(1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "0"
		}

		//if all the bits in the byte are read:
		if (melbus_Bitposition == 0) {
			//set bool to true to evaluate the bytes in main loop
			byteIsRead = true;

			//Reset bitcount to first bit in byte
			melbus_Bitposition = 7;
		}
		else {
			//set bitnumber to address of next bit in byte
			melbus_Bitposition--;
		}
	}
}

void nextTrack() {
	DWT_Delay_ms(1);
}

void prevTrack() {
	DWT_Delay_ms(1);
}

void fixTrack(byte *disk) {
  //cut out A-F in each nibble, and skip "00"
  if (*disk > 4) {
    *disk = 1;
  } else if (*disk < 1) {
    *disk = 4;
  }
}

void changeCD(byte trackInfo[], byte *disk) {
  while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_SET) {
    if (byteIsRead) {
      byteIsRead = false;
      switch (melbus_ReceivedByte) {
        //0x81 to 0x86 corresponds to cd buttons 1 to 6 on the HU (650)
        case 0x41:  //next cd
          *disk++;
          track = 1;
          break;
        case 0x01:  //prev cd
          *disk--;
          track = 1;
          break;
        default:
          track = 1;
          break;
      }
    }
  }
  trackInfo[3] = *disk;
  trackInfo[5] = track;
}

uint32_t DWT_Delay_Init(void)
{
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	/* Enable  clock cycle counter */
	DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;

	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	/* Check if clock cycle counter has started */
	if(DWT->CYCCNT)
	{
		return 0; /*clock cycle counter started*/
	}
	else
	{
		return 1; /*clock cycle counter not started*/
	}
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
	__disable_irq();
	while (1)
	{
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
