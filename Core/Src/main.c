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
#include "master.h"
#include "pins.h"
#include "delay.h"
#include "slave.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile byte melbus_ReceivedByte = 0;
volatile byte melbus_Bitposition = 7;
volatile bool byteIsRead = false;

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
	E_MD_RTR,
	E_MD_RTR_2,
	E_MD_RTR_3,
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
const byte commands[E_LIST_MAX][8] = {
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
		[E_MD_PUP] = MD_PUP, // 10
		[E_MD_PDN] = MD_PDN, // 11
		[E_MD_FFW] = MD_FFW, // 12
		[E_MD_FRW] = MD_FRW, // 13
		[E_MD_SCN] = MD_SCN, // 14
		[E_MD_RND] = MD_RND, // 15
		[E_MD_NU] = MD_NU,   // 16
		[E_MD_RTR] = MD_RTR, // 17
		[E_MD_RTR_2] = MD_RTR_2, // 18
		[E_MD_RTR_3] = MD_RTR_3, // 19
		[E_CDC_CIR] = CDC_CIR, // 20
		[E_CDC_TIR] = CDC_TIR, // 21
		[E_CDC_NXT] = CDC_NXT, // 22
		[E_CDC_PRV] = CDC_PRV, // 23
		[E_CDC_CHG] = CDC_CHG, // 24
		[E_CDC_PUP] = CDC_PUP, // 25
		[E_CDC_PDN] = CDC_PDN, // 26
		[E_CDC_FFW] = CDC_FFW, // 27
		[E_CDC_FRW] = CDC_FRW, // 28
		[E_CDC_SCN] = CDC_SCN, // 29
		[E_CDC_RND] = CDC_RND, // 30
		[E_CDC_NU] = CDC_NU   // 31
};

int __io_putchar(int ch)
{
	// Write character to ITM ch.0
	ITM_SendChar(ch);
	return(ch);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	byte lastByte = 0;
	bool powerOn = true;
	long HWTicks = 0;      //age since last BUSY switch
	long ComTicks = 0;     //age since last received byte
	long ConnTicks = 0;    //age since last message to SIRIUS SAT
	long timeout = 1000000; //should be around 10-20 seconds
	long runOnce = 300000;     //counts down on every received message from HU. Triggers when it is passing 1.
	long runPeriodically = 100000; //same as runOnce but resets after each countdown.
	byte matching[E_LIST_MAX];     //Keep track of every matching byte in the commands array
	byte tircount = 0;
	byte track = 1;
	byte md = 0;
	byte cd = 1;
	byte mdTrackInfo[] = {0x00, 0x02, 0x00, 0x00, 0x80, 0x99, 0x0C, 0xCC, 0xCC};
	byte mdCartridgeInfo[] = {0x80, 0x00, 0x0F, 0x04, 0x00, 0x0F};
	byte cdcTrackInfo[] = {0x00, 0x02, 0x00, 0x01, 0x80, 0x01, 0xC7, 0x0A, 0x02};
	byte cdcCartridgeInfo[] = {0x00, 0xFC, 0xFF, 0x4A, 0xFC, 0xFF};
	bool textInit = false; // First MRB2 is to initialise sending text
	byte textHeader[] = {0xFB, 0xD8, 0xFA, 0x00, 0x01, 0x01, 0x03, 0x02, 0x00}; // Send this as prefix to text
	byte textInitHeader[] = {0xF9, 0xD8, 0xE1, 0x68, 0x00, 0x00, 0x40, 0x00, 0x0C, 0xCC, 0xCC }; // Send this as reply to first MRB2
	//byte textRow = 2; 						TODO: use for sending text
	//byte customText[36] = "visualapproach";	TODO: send text
	bool reqMasterFlag = false; //set this to request master mode (and sendtext) at a proper time.

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
		bool busy = HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin);
		byte byteCounter = 1;
		byte command = 0x00;
		byte melbus_log[99];
		HWTicks++;
		if (powerOn) {
			ComTicks++;
			ConnTicks++;
		} else {
			ComTicks = 1;
			ConnTicks = 1;
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, busy);
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
							case E_MI: /* Intentional fall-through */
							case E_SI:
								textInitHeader[3] = 0x68;
								textInitHeader[6] = 0x40;
								textInitHeader[7] = 0x0;
								reqMasterFlag = true;
								textInit = true;
								while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
									if(byteIsRead) {
										byteIsRead = false;
										if (melbus_ReceivedByte == CDC_BASE_ID) {
											SendByteToMelbus(CDC_RESPONSE_ID);
										}
										if (melbus_ReceivedByte == MD_BASE_ID) {
											SendByteToMelbus(MD_RESPONSE_ID);
										}
										/*  Adds TV (A9), DAB (B8). C0 is SAT, but HU803 does not support SAT.
										 * if (melbus_ReceivedByte == 0xA9) {
										 *   	byteToSend = 0xAE;
										 *	    SendByteToMelbus();
										 * }
										 * if (melbus_ReceivedByte == 0xB8) {
										 *	byteToSend = 0xBE;
										 *	SendByteToMelbus();
										 * }
										 * if (melbus_ReceivedByte == 0xC0) {
										 *	byteToSend = 0xC6;
										 *	SendByteToMelbus();
										 * }
										 *
										 */
									}
								}
								break;
							case E_MRB_1: /* Intentional fall-through */
							case E_MRB_2:
								bool state = HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin);
								while (state == GPIO_PIN_RESET) {
									if (byteIsRead) {
										byteIsRead = false;
										if (melbus_ReceivedByte == MD_MASTER_ID) {
											SendByteToMelbus(MD_MASTER_ID);
											if (textInit) {
												SendText(textInitHeader, sizeof(textInitHeader), true);
												textInit = false;
											} else {
												SendText(textHeader, sizeof(textHeader), false);
											}
											break;
										}
									}
									state = HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin);
								}
								break;
							case E_IGN_OFF:
								powerOn = false;
								break;
							case E_MD_CIR:
								SendCartridgeInfo(mdCartridgeInfo);
								break;
							case E_MD_TIR:
								tircount++;
								if (tircount > 5) {
									reqMasterFlag = true;
									tircount = 0;
								}
								SendTrackInfo(mdTrackInfo);
								break;
							case E_MD_NXT:
								fixTrack(++track);
								mdTrackInfo[5] = track;
								nextTrack();
								break;
							case E_MD_PRV:
								fixTrack(--track);
								mdTrackInfo[5] = track;
								prevTrack();
								break;
							case E_MD_CHG:
								while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
									if (byteIsRead) {
										byteIsRead = false;
										command = melbus_ReceivedByte;
									}
								}
								changeCD(&md, &track, command);
								if (md > 4) {
									md = 0;
								}
								mdTrackInfo[3] = md;
								mdTrackInfo[5] = track;
								break;
							case E_MD_NU:
								break;
							case E_MD_RTR_3:
							case E_MD_RTR_2:
							case E_MD_RTR:
								SendByteToMelbus(0x00);
								SendByteToMelbus(0x01);
								reqMasterFlag = true;
								break;
							case E_CDC_CIR:
								SendCartridgeInfo(cdcCartridgeInfo);
								break;
							case E_CDC_TIR:
								tircount++;
								if (tircount > 5) {
									reqMasterFlag = true;
									tircount = 0;
								}
								SendTrackInfo(cdcTrackInfo);
								break;
							case E_CDC_NXT:
								fixTrack(++track);
								cdcTrackInfo[5] = track;
								nextTrack();
								break;
							case E_CDC_PRV:
								fixTrack(--track);
								cdcTrackInfo[5] = track;
								prevTrack();
								break;
							case E_CDC_CHG:
								while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
									if (byteIsRead) {
										byteIsRead = false;
										command = melbus_ReceivedByte;
									}
								}
								changeCD(&cd, &track, command);
								if (cd > 10) {
									cd = 1;
								}
								if (cd < 1) {
									cd = 10;
								}
								cdcTrackInfo[3] = cd;
								cdcTrackInfo[5] = track;
								break;
							case E_CDC_PUP:
								cdcTrackInfo[1] = cdcTrackInfo[8] = TRACK_STARTBYTE;
								break;
							case E_CDC_PDN:
								if (cmd == E_CDC_PDN) {
									cdcTrackInfo[1] = cdcTrackInfo[8] = TRACK_STOPBYTE;
								}
							case E_MD_PUP: /* Intentional fall-through */
								mdTrackInfo[1] = TRACK_STARTBYTE;
								mdTrackInfo[6] = mdTrackInfo[7] = mdTrackInfo[8] = 0;
								reqMasterFlag = true;
								textInit = false;
							case E_MD_PDN: /* Intentional fall-through */
								if (cmd == E_MD_PDN) {
									mdTrackInfo[1] = TRACK_STOPBYTE;
									mdTrackInfo[6] = 0xC;
									mdTrackInfo[7] = mdTrackInfo[8] = 0xCC;
									textInitHeader[3] = TRACK_STOPBYTE;
									textInitHeader[6] = 0x80;
									textInitHeader[7] = 0x99;
									textInit = true;
									reqMasterFlag = true;
								}
							case E_MD_FFW: /* Intentional fall-through */
							case E_MD_FRW: /* Intentional fall-through */
							case E_MD_SCN: /* Intentional fall-through */
							case E_MD_RND: /* Intentional fall-through */
							case E_CDC_FFW: /* Intentional fall-through */
							case E_CDC_FRW: /* Intentional fall-through */
							case E_CDC_SCN: /* Intentional fall-through */
							case E_CDC_RND:
								SendByteToMelbus(0x00);
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
			busy = HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin);
		}
		if (ComTicks == 0 && ConnTicks != 0) {    //print unmatched messages (unknown)
			for (byte b = 0; b < byteCounter - 1; b++) {
				printf("%02X  ", melbus_log[b]);
			}
			printf("\r\n");
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
		if ((runOnce == 1) || reqMasterFlag) {
			reqMaster();
			reqMasterFlag = false;
		}

		if (runPeriodically == 0) {
			runPeriodically = 100000;
			//textRow = 2;
			reqMaster();
		}
	}
}
/* USER CODE END 3 */

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
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
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

void resetBitPosition(void) {
	melbus_Bitposition = 7;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin == MELBUS_CLOCK_Pin) {
		//Read status of Datapin and set status of current bit in recv_byte
		if (HAL_GPIO_ReadPin(GPIOA, MELBUS_DATA_Pin) == GPIO_PIN_SET) {
			melbus_ReceivedByte |= (1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "1"
		}
		else {
			melbus_ReceivedByte &= ~(1 << melbus_Bitposition); //set bit nr [melbus_Bitposition] to "0"
		}

		//if all the bits in the byte are read:
		if (melbus_Bitposition == 0) {
			//set bool to true to evaluate the bytes in main loop
			byteIsRead = true;

			//Reset bitcount to first bit in byte. ALSO do this when toggling clock from IT to input/output
			melbus_Bitposition = 7;
		}
		else {
			//set bitnumber to address of next bit in byte
			melbus_Bitposition--;
		}
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
