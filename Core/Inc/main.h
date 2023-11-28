/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef uint8_t byte;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void resetBitPosition(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MELBUS_CLOCK_Pin GPIO_PIN_2
#define MELBUS_CLOCK_GPIO_Port GPIOA
#define MELBUS_CLOCK_EXTI_IRQn EXTI2_IRQn
#define MELBUS_DATA_Pin GPIO_PIN_3
#define MELBUS_DATA_GPIO_Port GPIOA
#define MELBUS_BUSY_Pin GPIO_PIN_4
#define MELBUS_BUSY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define MD_RESPONSE_ID 0xDE
#define MD_MASTER_ID 0xDF
#define MD_BASE_ID 0xD8
#define MD_ALT_ID 0xD9

#define CDC_RESPONSE_ID 0xEE
#define CDC_MASTER_ID 0xEF
#define CDC_BASE_ID 0xE8
#define CDC_ALT_ID 0xE9

#define MRB_1 {3, 0x00, 0x1C, 0xEC}            //Master Request Broadcast version 1 - Never seen this in MD sniffs on HUx03, maybe HUx50/SAT specific
#define MRB_2 {3, 0x00, 0x1E, 0xEC}            //Master Request Broadcast version 2 (maybe this is second init seq?)
#define MI {3, 0x07, 0x1A, 0xEE}               //Main init sequence
#define SI {3, 0x00, 0x1E, 0xED}               //Secondary init sequence (turn off ignition, then on)
#define IGN_OFF {3, 0x00, 0x18, 0x12}          //this is the last message before HU goes to sleep

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
#define MD_RTR {7, MD_BASE_ID, 0x1E, 0xF9, 0x01, 0x01, 0x03, 0x01}			       //request text row respond with 2 bytes
#define MD_RTR_2 {7, MD_BASE_ID, 0x1E, 0xF9, 0x02, 0x02, 0x03, 0x01}			   //request text row
#define MD_RTR_3 {7, MD_BASE_ID, 0x1E, 0xF9, 0x03, 0x03, 0x03, 0x01}			   //request text row

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

//on powerup - change cdcTrackInfo[1] & [8] to this
#define TRACK_STARTBYTE 0x08
//same on powerdown
#define TRACK_STOPBYTE 0x02

/* MRB2 from IMIV:
 * 0 1E EC 87 FF DF DF FB D8 FA 0 2 1 3 2 0 80 99 54 68 65 20 52 6F 63 6B 61 66 65 6C 6C 65 72 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 1 1 3 2 0 80 99 50 72 61 69 73 65 20 59 6F 75 0 0 0 0 0 0
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 2 1 3 2 0 80 99 54 68 65 20 52 6F 63 6B 61 66 65 6C 6C 65 72 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 1 1 3 2 0 80 99 50 72 61 69 73 65 20 59 6F 75 0 0 0 0 0 0
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
#define MD_MRB2_REPLY {10, 0xFB, MD_BASE_ID, 0xFA, 0x00, row?, another row?, 0x03, yet another row?, 0x00, 0x80, track?}
possibly bit errors on sniffer:
#define MD_UNK_2 {3, MD_ALT_ID, 0x0D, 0xF0}
#define MD_UNK_3 {3, MD_ALT_ID, 0x19, 0xF0}
#define CDC_UNK {5, CDC_ALT_ID, 0x1B, 0xE0, 0x00, 0x84}
#define CDC_UNK_2 {5, CDC_BASE_ID, 0xD8, 0xF0, 0x00, 0x84}
 */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
