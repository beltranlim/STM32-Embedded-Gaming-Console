/******************************************************************************
* @file           : main.c
* @brief          : Main program body
* (c) EE2028 Teaching Team
******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/Components/lsm6dsl/lsm6dsl.h"
#include "../../Drivers/csrc/u8g2.h"

// Definitions
#define ACCEL_THRESHOLD 1.5f
#define GYRO_THRESHOLD 40.0f
#define MAG_THRESHOLD_LOW 50.0f
#define MAG_THRESHOLD_MED 80.0f
#define MAG_THRESHOLD_HIGH 100.0f
#define MAX_METEORS 5

TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;
static u8g2_t u8g2;

// Type definition
typedef enum{
	MAG_LEVEL_SAFE = 0,
	MAG_LEVEL_LOW = 1,
	MAG_LEVEL_MED = 2,
	MAG_LEVEL_HIGH = 3
}mag_level_t;

#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_PORT GPIOB

static unsigned char bitmap_troll[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x18, 0x00, 0x80, 0x1f, 0x00, 0xff, 0x0f, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x78, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x1c, 0x00, 0x18, 0xe0, 0x03, 0x3c, 0x00, 0x00, 0xe0, 0xe0, 0x03, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0e, 0x00, 0x83, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x07, 0x40, 0x10, 0x3c, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x03, 0x08, 0x43, 0x00, 0x00, 0x01, 0x00, 0x07, 0x60, 0x00, 0x70, 0x00, 0x00,
	0x00, 0x00, 0xc0, 0x01, 0x82, 0x18, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x60, 0x00, 0x00,
	0x00, 0x00, 0xc0, 0x01, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x04, 0x60, 0x00, 0x00,
	0x00, 0x00, 0xe0, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,
	0x00, 0x00, 0x70, 0x00, 0x00, 0xe0, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x38, 0x00, 0x00, 0xfc, 0xff, 0xf8, 0x01, 0x00, 0x80, 0xff, 0x00, 0xc0, 0x01, 0x00,
	0x00, 0x00, 0x0f, 0x00, 0x00, 0xce, 0x7f, 0xc0, 0x07, 0x00, 0xf8, 0xff, 0x03, 0x80, 0x07, 0x00,
	0x00, 0x80, 0xff, 0x1f, 0x1e, 0xff, 0xff, 0x07, 0x0f, 0x00, 0xfe, 0xff, 0x77, 0x3f, 0x1e, 0x00,
	0x00, 0xe0, 0x07, 0x00, 0x00, 0x7e, 0x80, 0x3f, 0x0f, 0xf0, 0xff, 0xff, 0x01, 0x40, 0x38, 0x00,
	0x00, 0x70, 0x81, 0xff, 0x01, 0x00, 0x1c, 0xf8, 0x07, 0xe0, 0x0f, 0x00, 0x00, 0x00, 0x73, 0x00,
	0x00, 0xbc, 0xe0, 0x81, 0x1f, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x03, 0x00, 0x00, 0x80, 0x60, 0x00,
	0x00, 0x5e, 0x70, 0x20, 0xfc, 0xff, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0xf8, 0x1f, 0x68, 0x00,
	0x00, 0x4e, 0x38, 0x70, 0x80, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x03, 0x70, 0x1e, 0x38, 0x70, 0x00,
	0x00, 0x46, 0x1c, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x0f, 0x00, 0x71, 0x00,
	0x00, 0x46, 0x0c, 0xf8, 0x07, 0x00, 0x00, 0xc4, 0x03, 0x00, 0x3e, 0x00, 0xc0, 0x00, 0x69, 0x00,
	0x00, 0x46, 0xcc, 0x1f, 0x7e, 0x00, 0x00, 0xfe, 0x03, 0x00, 0xf0, 0x01, 0xc0, 0x80, 0x68, 0x00,
	0x00, 0x4e, 0xdc, 0x3b, 0xf0, 0x07, 0x00, 0x38, 0x00, 0x00, 0xe0, 0x03, 0xc0, 0x63, 0x70, 0x00,
	0x00, 0x8e, 0x1c, 0x70, 0x00, 0x7f, 0x00, 0x38, 0xfe, 0x01, 0xf0, 0x39, 0xe0, 0x03, 0x3a, 0x00,
	0x00, 0x9c, 0x38, 0x70, 0x00, 0xff, 0x0f, 0x70, 0xc6, 0x00, 0x3c, 0xc0, 0xf0, 0x67, 0x38, 0x00,
	0x00, 0x38, 0x01, 0xf0, 0x07, 0x07, 0xff, 0x41, 0x00, 0x70, 0x0f, 0x00, 0xf8, 0x07, 0x1c, 0x00,
	0x00, 0x70, 0x18, 0xc0, 0x3f, 0x07, 0xc0, 0x7f, 0x00, 0xe0, 0x03, 0x80, 0xcf, 0x07, 0x0e, 0x00,
	0x00, 0xe0, 0x03, 0x80, 0xfb, 0x1f, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0xfc, 0xdf, 0x07, 0x07, 0x00,
	0x00, 0x80, 0x03, 0x00, 0x87, 0xff, 0x01, 0x06, 0xf8, 0xff, 0xff, 0x3f, 0x98, 0x07, 0x07, 0x00,
	0x00, 0x00, 0x0f, 0x00, 0x0e, 0xff, 0x1f, 0x07, 0x80, 0xf1, 0x7f, 0x30, 0x98, 0x07, 0x03, 0x00,
	0x00, 0x00, 0x1c, 0x00, 0x3c, 0xe6, 0xff, 0x03, 0x80, 0x01, 0x0e, 0x70, 0xfc, 0x07, 0x03, 0x00,
	0x00, 0x00, 0x38, 0x00, 0x78, 0x06, 0xfe, 0x7f, 0x80, 0x01, 0x0e, 0x60, 0xff, 0x07, 0x03, 0x00,
	0x00, 0x00, 0x70, 0x00, 0xe0, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x03, 0x00,
	0x00, 0x00, 0xe0, 0x00, 0x80, 0x03, 0x80, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x03, 0x00,
	0x00, 0x00, 0xe0, 0x00, 0x00, 0x0f, 0x80, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x03, 0x00,
	0x00, 0x00, 0xc0, 0x03, 0x00, 0x3c, 0xc0, 0x01, 0xe0, 0xff, 0xff, 0xff, 0x7f, 0x07, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x07, 0x00, 0xf0, 0xe1, 0x00, 0x60, 0x00, 0xff, 0x9f, 0xbb, 0x03, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x1e, 0x00, 0x80, 0x7f, 0x00, 0x60, 0x00, 0x03, 0xc7, 0xf9, 0x01, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x78, 0x06, 0x03, 0xfc, 0x00, 0x60, 0x00, 0x03, 0xe7, 0xf0, 0x00, 0x03, 0x00,
	0x00, 0x00, 0x00, 0xe0, 0x21, 0x18, 0xe0, 0x3f, 0x70, 0x80, 0x83, 0xe3, 0x3f, 0x00, 0x03, 0x00,
	0x00, 0x00, 0x00, 0xc0, 0x87, 0xc1, 0x00, 0xfe, 0xff, 0x80, 0xe3, 0xff, 0x0f, 0x00, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x3f, 0x0c, 0x06, 0x00, 0xfe, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xf8, 0xc0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0x0c, 0x0c, 0xc0, 0xff, 0xff, 0x01, 0x10, 0x08, 0x06, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xc0, 0x80, 0x01, 0x00, 0x00, 0x00, 0x06, 0x08, 0x06, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0xf8, 0xf0, 0xff, 0xff, 0x7f, 0x00, 0x06, 0x06, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0xf0, 0x0f, 0x00, 0x10, 0x00, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x3f, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0x00, 0x80, 0x1f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char bitmap_cat [] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0xe0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x79, 0x00, 0x00, 0x00, 0x3c, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0x03, 0x00, 0x00, 0x0f, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0xff, 0xff, 0xff, 0x01, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x58, 0xef, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x7c, 0x00, 0xe0, 0x01, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0xf0, 0x01, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0e, 0x3c, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x60, 0x00, 0x00, 0xfe, 0x07, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0x70, 0x00, 0x00, 0xa0, 0x00, 0x00, 0xe0, 0x00, 0xfc, 0x07, 0x3c, 0x00, 
	0x00, 0x00, 0x80, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xe0, 0x07, 0x00, 0x60, 0x00, 
	0x00, 0x00, 0xc0, 0x00, 0xe0, 0x00, 0x00, 0xf0, 0x7f, 0x00, 0x60, 0x3f, 0x00, 0x00, 0xc0, 0x00, 
	0x00, 0x00, 0xe0, 0x3c, 0xe0, 0x00, 0x00, 0x1c, 0xc0, 0x01, 0xf0, 0x01, 0x00, 0x00, 0xc0, 0x00, 
	0x00, 0x00, 0x70, 0xf8, 0xe1, 0x00, 0x00, 0x07, 0x80, 0x83, 0x1f, 0x00, 0x00, 0x00, 0xc0, 0x00, 
	0x00, 0x00, 0x38, 0x00, 0x60, 0x00, 0x80, 0x03, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 
	0x00, 0x00, 0x1c, 0x00, 0x70, 0x00, 0xe0, 0xe0, 0x03, 0x0f, 0x7e, 0x00, 0x00, 0x00, 0x1e, 0x00, 
	0x00, 0x00, 0x0e, 0x00, 0x30, 0x00, 0x70, 0x80, 0x87, 0x83, 0xc3, 0x03, 0x00, 0xe0, 0x07, 0x00, 
	0x00, 0x00, 0x07, 0x00, 0x18, 0x00, 0x38, 0x00, 0xc0, 0xe1, 0x00, 0x07, 0x00, 0x3e, 0x00, 0x00, 
	0x00, 0x80, 0x01, 0x00, 0x9c, 0xff, 0x0f, 0x00, 0xe0, 0x38, 0x00, 0x0e, 0xf0, 0x03, 0x00, 0x00, 
	0x00, 0xc0, 0x00, 0x00, 0x0e, 0x00, 0x07, 0x00, 0x70, 0x1c, 0x00, 0x86, 0x1f, 0x00, 0x00, 0x00, 
	0x00, 0x60, 0x00, 0x00, 0x07, 0x80, 0x03, 0x00, 0x38, 0x06, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x9c, 0x03, 0x1f, 0x07, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x80, 0x0f, 0x70, 0x00, 0x00, 0xc6, 0x01, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x1c, 0x00, 0xc0, 0xf9, 0x1f, 0x00, 0x00, 0xe3, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0xc0, 0x01, 0x0e, 0x00, 0xc0, 0x31, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0xc0, 0x01, 0x07, 0x00, 0xe0, 0x1c, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0xc0, 0x81, 0x01, 0x00, 0x70, 0x0e, 0x00, 0xf8, 0xf9, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0xc0, 0xe1, 0x00, 0x00, 0x18, 0x07, 0x00, 0x8c, 0x0f, 0x0f, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0xc0, 0x71, 0x00, 0x00, 0xcc, 0x01, 0x00, 0xe7, 0x00, 0x18, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x80, 0x19, 0x00, 0x00, 0xe7, 0x00, 0x80, 0x3b, 0x00, 0x30, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x80, 0x0d, 0x00, 0x80, 0x3b, 0x00, 0xc0, 0x0e, 0x3c, 0x38, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x80, 0x07, 0x00, 0xc0, 0x1d, 0x00, 0x70, 0x07, 0x60, 0x1c, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x80, 0x03, 0x00, 0x60, 0x06, 0x00, 0xf8, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0xc0, 0x01, 0x00, 0xb8, 0x03, 0x00, 0x7c, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x60, 0x00, 0x00, 0xdc, 0x01, 0x00, 0x3f, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x0f, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x00, 0xe0, 0x07, 0x80, 0x33, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0x80, 0x0d, 0x00, 0xf0, 0x01, 0xe0, 0x70, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0xc0, 0x07, 0x00, 0xf8, 0x00, 0x38, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0c, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x3e, 0x00, 0x0e, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xc0, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xf0, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x63, 0x7c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe3, 0x07, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xfc, 0x01, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0xf8, 0x03, 0xf8, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x00, 0xff, 0xcf, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0xfb, 0xbe, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x80, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xfe, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Function Initialization
static void UART1_Init(void);
static void MX_GPIO_PB_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
static void CatchAndRun(void);
static void RLGL(void);
static void MX_GPIO_LED_Init(void);
static bool is_accel_threshold(const float initial[3], const float current[3], float thresh);
static bool is_gyro_threshold(const float initial[3], const float current[3], float thresh);
static mag_level_t magneto_thresh(const float initial[3], const float current[3]);
static void PWM_BUZZER_Init(void);
static void TILT_Init(void);
static void TILT_DeInit(void);
static void I2C1_Init(void);
static void OLED_Init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_toggle(void);
void buzzer_beep(uint16_t freq,uint32_t duration_ms);
void buzzer_beep_special(void);
static void SpaceshipGame(void);
uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

// Variables Initialization
const uint32_t RLGL_interval_ms = 10000;
const uint32_t sensor_interval_ms = 2000;
static uint32_t ALT_previous_millis = 0;
static uint32_t sensor_previous_millis = 0;
static uint32_t G1_LED_previous_millis = 0;
const uint32_t G1_LED_interval_ms = 500;

const uint32_t PB_interval_ms = 3000;
static uint32_t PB_previous_millis = 0;
static uint32_t prox_millis = 0;

const uint32_t sensor_CNR_interval_ms = 1000;
static uint32_t sensor_CNR_previous_millis = 0;
static bool game_over_displayed = false;

static uint32_t buzzer_start_time = 0;
static uint32_t buzzer_duration = 0;
static bool buzzer_active = false;

static uint16_t beep_sequence[] = {1000, 800, 600, 400, 200};
static uint8_t beep_seq_length = 5;
static uint32_t beep_tone_duration = 300;
static uint32_t beep_pause_duration = 250;
static uint8_t beep_seq_index = 0;
static bool beep_seq_active = false;
static bool beep_seq_in_pause = false;
static uint32_t beep_seq_timer = 0;

UART_HandleTypeDef huart1;

//static uint32_t button_previous_millis = 0;
//const uint32_t button_interval_ms = 1000;
static int mode = 0;
static int button_count = 0;
static int prox_level = 0;
static volatile int spaceship_x = 60; // Spaceship X position, volatile for ISR access
static float initial_space_acc_x = 0;
static volatile bool baseline_captured = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin && mode == 2 && baseline_captured)
    {
		int16_t gyro_data_i16[3] = {0};
        const float TILT_DEVIATION_THRESHOLD = 1.0f; // Threshold to determine direction
        char message_print[50];

		sprintf(message_print, "Debug: Tilt Interrupt Triggered!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

        // Read the XYZ values at the moment of the interrupt
        // BSP_GYRO_GetXYZ(gyro_data_i16);
		BSP_ACCELERO_AccGetXYZ(gyro_data_i16);

		float current_space_gyro_x = (float)gyro_data_i16[0] * (9.8/1000.0f);

		// Debug message for current accel reading
        sprintf(message_print, "Debug: Current Acc X: %f\r\n", current_space_gyro_x);
        HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

		float deviation = current_space_gyro_x - initial_space_acc_x;

        // Check the Y-axis deviation from baseline
        if (deviation > TILT_DEVIATION_THRESHOLD) { // Tilted left
            spaceship_x -= 8;
            sprintf(message_print, "Tilt: LEFT\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
        }
        else if (deviation < -TILT_DEVIATION_THRESHOLD) { // Tilted right
            spaceship_x += 8;
            sprintf(message_print, "Tilt: RIGHT\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
        }

        // Boundary checks
        if (spaceship_x < 0) spaceship_x = 0;
        if (spaceship_x > 120) spaceship_x = 120; // Keep spaceship on screen

        return;
    }

	if(GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		uint32_t current_millis = HAL_GetTick();

		// "Tag" action in Catch and Run mode
        if (mode == 1 && prox_level != 0 && (current_millis - prox_millis <= PB_interval_ms)) {
            prox_level = 0;
            mode = 3; // Game Over
            char message_print[32];
            sprintf(message_print, "You got caught!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);	
            return; // Exit callback
        }

        // Double-press logic to switch modes
        if (button_count == 0) {
            // First press
            PB_previous_millis = current_millis;
            button_count = 1;
        } else if (current_millis - PB_previous_millis <= PB_interval_ms) {
            // Second press within interval (double-press)
            mode = (mode + 1) % 3; // Cycle through modes 0-2
            button_count = 0; // Reset for next double-press
			game_over_displayed = false; // Reset game over display flag
			
			// Reset game state variables
			prox_level = 0;
			buzzer_off();
			beep_seq_active = false;
			buzzer_active = false;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Turn off LED
        } else {
            // Second press was too late, treat as a new first press
            PB_previous_millis = current_millis;
            button_count = 1;
        }
	}
}

int main(void)
{
//        int seconds_count = 0;
    /* Reset of all peripherals, Initializes Systick etc. */
    HAL_Init();

	MX_GPIO_PB_Init();
	MX_GPIO_LED_Init();

	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();

	BSP_ACCELERO_Init();
	BSP_GYRO_Init();

	BSP_MAGNETO_Init();

	PWM_BUZZER_Init();
	I2C1_Init();
	OLED_Init();

	/* UART initialization  */
	UART1_Init();

	while(1){

		if (beep_seq_active) {
            uint32_t current_millis = HAL_GetTick();
            uint32_t elapsed = current_millis - beep_seq_timer;
            
            if (!beep_seq_in_pause) {
                // Currently playing a tone
                if (elapsed >= beep_tone_duration) {
                    buzzer_off();
                    beep_seq_in_pause = true;
                    beep_seq_timer = current_millis;
                    beep_seq_index++;
                    
                    // Check if sequence is complete
                    if (beep_seq_index >= beep_seq_length) {
                        beep_seq_active = false;
                    }
                }
            }
            else {
                // In pause between tones
                if (elapsed >= beep_pause_duration) {
                    // Play next tone
                    set_buzzer_freq(beep_sequence[beep_seq_index]);
                    buzzer_on();
                    beep_seq_in_pause = false;
                    beep_seq_timer = current_millis;
                }
            }
        }

		if (mode == 0){
			ALT_previous_millis = HAL_GetTick();
			u8g2_ClearBuffer(&u8g2);
			u8g2_SendBuffer(&u8g2);
			RLGL();
		}
		else if (mode == 1){
			ALT_previous_millis = HAL_GetTick();
			u8g2_ClearBuffer(&u8g2);
			u8g2_SendBuffer(&u8g2);
			CatchAndRun();
		}
		else if (mode == 2){
			ALT_previous_millis = HAL_GetTick();
			u8g2_ClearBuffer(&u8g2);
			u8g2_SendBuffer(&u8g2);
			SpaceshipGame();
		}
		else if (mode == 3 && !game_over_displayed){
			buzzer_beep_special();
			char message_print[32];
			sprintf(message_print, "Game Over!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			sprintf(message_print, "Press button to restart\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			game_over_displayed = true;

			// Game Over State
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			u8g2_ClearBuffer(&u8g2);

			u8g2_DrawXBM(&u8g2, 0, 0, 128, 64, bitmap_troll);

			u8g2_SendBuffer(&u8g2);
		}
	}
}

// Define a structure for meteors
typedef struct {
    int x;
    int y;
    int speed;
    bool active;
} Meteor;

// Define states for the spaceship game
typedef enum {
    SPACESHIP_STATE_INIT,
    SPACESHIP_STATE_WAIT_STABILIZE,
    SPACESHIP_STATE_CAPTURE_BASELINE,
    SPACESHIP_STATE_PLAYING
} SpaceshipGameState;

static void SpaceshipGame(void)
{
    char message_print[64];
    int16_t gyro_data_i16[3] = {0};
    
    Meteor meteors[MAX_METEORS];
    uint32_t last_meteor_spawn_time = 0;
    uint32_t meteor_spawn_interval = 5000; // Spawn a new meteor every 5 seconds

    uint32_t last_update_time = 0;
    const uint32_t update_interval = 30; // Game loop refresh interval

    SpaceshipGameState game_state = SPACESHIP_STATE_INIT;
    uint32_t state_timer = 0;
    int reading_count = 0;

    while(mode == 2)
    {
        uint32_t current_time = HAL_GetTick();

        // Non-blocking game loop
        if (current_time - last_update_time < update_interval) {
            continue; // Wait for the next frame
        }
        last_update_time = current_time;

        // --- Game State Machine ---
        switch(game_state)
		{
            case SPACESHIP_STATE_INIT:
                // Reset game variables
                spaceship_x = 60;
                baseline_captured = false;
                initial_space_acc_x = 0;
                reading_count = 0;

                // Initialize meteors
                for (int i = 0; i < MAX_METEORS; i++) {
                    meteors[i].active = false;
                }
                last_meteor_spawn_time = current_time;

                sprintf(message_print, "Entering Spaceship Game as Player\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
                
                // Transition to next state
                game_state = SPACESHIP_STATE_WAIT_STABILIZE;
                state_timer = current_time;
                break;

            case SPACESHIP_STATE_WAIT_STABILIZE:
                // Wait 200ms for sensor to stabilize
                if (current_time - state_timer >= 200) {
                    game_state = SPACESHIP_STATE_CAPTURE_BASELINE;
                }
                break;
			
			case SPACESHIP_STATE_CAPTURE_BASELINE:
                // Take the second reading as the baseline
                // BSP_GYRO_GetXYZ(gyro_data_i16);
				BSP_ACCELERO_AccGetXYZ(gyro_data_i16); // Using accelerometer for tilt
				
                reading_count++;

                if (reading_count == 2) {
					initial_space_acc_x = (float)gyro_data_i16[0] * (9.8/1000.0f);
                    baseline_captured = true;

					 // Debug message for baseline capture
                    sprintf(message_print, "Debug: Baseline Acc X: %f\r\n", initial_space_acc_x);
                    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

                    sprintf(message_print, "Baseline captured. Ready to play!\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
                    TILT_Init(); // Enable tilt interrupt AFTER baseline is captured
                    
                    // Transition to playing state
                    game_state = SPACESHIP_STATE_PLAYING;
                } else {
                    // If not the second reading, wait before the next one (implicitly handled by main loop delay)
                }
                break;

			case SPACESHIP_STATE_PLAYING:
                // --- Game Logic (Meteors) ---
                // Spawn new meteors
                if (current_time - last_meteor_spawn_time > meteor_spawn_interval) {
                    last_meteor_spawn_time = current_time;
                    for (int i = 0; i < MAX_METEORS; i++) {
                        if (!meteors[i].active) {
                            meteors[i].active = true;
                            meteors[i].x = rand() % 128; // Random X position
                            meteors[i].y = 0; // Start at the top
                            meteors[i].speed = (rand() % 3) + 1; // Random speed from 1 to 3
                            break; // Spawn one meteor at a time
                        }
                    }
                }
			
				for (int i = 0; i < MAX_METEORS; i++) {
                    if (meteors[i].active) {
                        meteors[i].y += meteors[i].speed;

                        // Deactivate if it goes off-screen
                        if (meteors[i].y > 64) {
                            meteors[i].active = false;
                        }

                        // Collision Detection
                        if (spaceship_x < meteors[i].x + 3 &&
                            spaceship_x + 8 > meteors[i].x &&
                            56 < meteors[i].y + 3 &&
                            64 > meteors[i].y)
                        {
                            sprintf(message_print, "GAME OVER\r\n");
                            HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
                            mode = 3; // Set game over mode
                            goto game_loop_exit; // Exit the while loop immediately
                        }
                    }
                }

				// --- Drawing ---
                u8g2_ClearBuffer(&u8g2);
                u8g2_DrawTriangle(&u8g2, spaceship_x, 64, spaceship_x + 4, 56, spaceship_x + 8, 64);

                // Draw meteors
                for (int i = 0; i < MAX_METEORS; i++) {
                    if (meteors[i].active) {
                        u8g2_DrawBox(&u8g2, meteors[i].x, meteors[i].y, 3, 3);
                    }
                }
                u8g2_SendBuffer(&u8g2);
                break;
		}
	}

game_loop_exit:
	TILT_DeInit(); // Disable tilt interrupt
}

static void RLGL(void)
{
	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };

	float gyro_data[3];
	float gyro_data_i16[3] = { 0 };

	float temp_data = 0;
	float pres_data = 0;
	float humi_data = 0;

	int state = 0;
	char message_print[64];

	int sensor_read_count = 0;
	float initial_accel[3];
	float initial_gyro[3];

	sprintf(message_print, "Entering Red Light, Green Light as Player\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

	sprintf(message_print, "Green Light!\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	// Initial Buzzer Beep to indicate start of Green Light
	buzzer_beep(200, 200);

	while(mode == 0){
		u8g2_ClearBuffer(&u8g2);
		uint32_t current_millis = HAL_GetTick();

		if (buzzer_active && (HAL_GetTick() - buzzer_start_time >= buzzer_duration)){
			buzzer_off();
			buzzer_active = false;
		}

		if (current_millis - ALT_previous_millis >= RLGL_interval_ms){
			ALT_previous_millis = current_millis;
			// State 0 stands for red light
			if (state == 0){
				state = 1;
				sensor_read_count = 0;
				sprintf(message_print, "Red Light!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				buzzer_beep(500, 200);
			}
			// State 1 stands for green light
			else{
				state = 0;
				sensor_read_count = 0;
				sprintf(message_print, "Green Light!\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				buzzer_beep(200, 200);
			}
		}

		// State = 0 means that its in Green Light Mode
		if (state == 0){

			if (current_millis - sensor_previous_millis >= sensor_interval_ms){
				sensor_previous_millis = current_millis;

				// Temperature sending
				temp_data = BSP_TSENSOR_ReadTemp();

				// Pressure sending
				pres_data = BSP_PSENSOR_ReadPressure();

				// Humidity sending
				humi_data = BSP_HSENSOR_ReadHumidity();

				sprintf(message_print, "Temperature: %f, Pressure: %f, Humidity: %f\r\n", temp_data, pres_data, humi_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}

		// State = 1 means that its in Red Light Mode
		else if (state == 1){
			// Set the first condition to determine a threshold

			// Blink the LED once every 0.5s
			if (current_millis - G1_LED_previous_millis >= G1_LED_interval_ms){
				G1_LED_previous_millis = current_millis;
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}
			// Send sensor readings every 2 seconds
			if (current_millis - sensor_previous_millis >= sensor_interval_ms){
				// Accelerometer Reading
				sensor_previous_millis = current_millis;
				++sensor_read_count;

				// Accelerometer Reading
				// Units in meters per second
				BSP_ACCELERO_AccGetXYZ(accel_data_i16);

				accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f); // m/s
				accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
				accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

				sprintf(message_print, "Accel X : %f; Accel Y : %f; Accel Z : %f\r\n", accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

				// Gyroscope Reading
				// Units in degree per second
				BSP_GYRO_GetXYZ(gyro_data_i16);

				gyro_data[0] = (float)gyro_data_i16[0] / 1000.0f; // dps
				gyro_data[1] = (float)gyro_data_i16[1] / 1000.0f; 
				gyro_data[2] = (float)gyro_data_i16[2] / 1000.0f;

				sprintf(message_print, "Gyro X : %f; Gyro Y : %f; Gyro Z : %f\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

				// Skip first reading so sensor doesn't read noisy info
				if (sensor_read_count == 1){
					sprintf(message_print, "Skipping first reading...\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					continue;
				}

				// Only start reading from reading 2 onwards
				if (sensor_read_count == 2){
					initial_accel[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
					initial_accel[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
					initial_accel[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

					initial_gyro[0] = (float)gyro_data_i16[0] / 1000.0f;
					initial_gyro[1] = (float)gyro_data_i16[1] / 1000.0f;
					initial_gyro[2] = (float)gyro_data_i16[2] / 1000.0f;

					sprintf(message_print, "Baseline captured: A[%f, %f, %f]\r\n", initial_accel[0], initial_accel[1], initial_accel[2]);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
					sprintf(message_print, "Baseline captured: G[%f, %f, %f]\r\n", initial_gyro[0], initial_gyro[1], initial_gyro[2]);
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
					continue;
				}

				// Check threshold only from second data reading onwards
				if (is_accel_threshold(initial_accel, accel_data, ACCEL_THRESHOLD) || is_gyro_threshold(initial_gyro, gyro_data,  GYRO_THRESHOLD)){
					sprintf(message_print, "GAME OVER\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					mode = 3;
					break;
				}
			}
		}
	}
}

static void CatchAndRun(void)
{
    float magneto_data[3];
    int16_t magneto_data_i16[3] = { 0 };
    float initial_magneto[3];

    uint32_t MAGNO_LED_previous_millis = 0;
    uint32_t MAGNO_previous_millis = 0;
    uint32_t led_blink_interval = 0;
    bool escaped = true;

    int temp_data = 0;
    int pres_data = 0;
    int humi_data = 0;

	int initial_temp = 0;
	int initial_pres = 0;
	int initial_humi = 0;

	const int TEMP_THRESHOLD = 5; // degrees Celsius
	const int PRES_THRESHOLD = 50; // hPa
	const int HUMI_THRESHOLD = 10; // %RH

    char message_print[80];
    int sensor_read_count = 0;
	int env_read_count = 0;
    sprintf(message_print, "Entering Catch And Run as Player\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

    while(mode == 1){
		if (buzzer_active && (HAL_GetTick() - buzzer_start_time >= buzzer_duration)){
			buzzer_off();
			buzzer_active = false;
		}

        u8g2_ClearBuffer(&u8g2);
        uint32_t current_millis = HAL_GetTick();

        if (led_blink_interval > 0 && (current_millis - MAGNO_LED_previous_millis >= led_blink_interval)){
            MAGNO_LED_previous_millis = current_millis;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        }

        if (current_millis - MAGNO_previous_millis >= 200){
            MAGNO_previous_millis = current_millis;
            ++sensor_read_count;

            BSP_MAGNETO_GetXYZ(magneto_data_i16);

            if (sensor_read_count == 1){
                sprintf(message_print, "Skipping first reading...\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
                continue;
            }
            if (sensor_read_count == 2){
				// Value from magnetometer returned as LSB per gauss, with a sensitivity of 6842 LSB/Gauss
				// Hence to get meaningful units, you have to divide by 6.842 to get milligauss
                initial_magneto[0] = (float)magneto_data_i16[0] / 6.842f;
                initial_magneto[1] = (float)magneto_data_i16[1] / 6.842f;
                initial_magneto[2] = (float)magneto_data_i16[2] / 6.842f;
                sprintf(message_print, "Baseline captured: M[%f, %f, %f]\r\n", initial_magneto[0], initial_magneto[1], initial_magneto[2]);
                HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
                continue;
            }

            magneto_data[0] = (float)magneto_data_i16[0] / 6.842f;
            magneto_data[1] = (float)magneto_data_i16[1] / 6.842f;
            magneto_data[2] = (float)magneto_data_i16[2] / 6.842f;

			// Send magneto data over UART
			sprintf(message_print, "Magneto X : %f; Magneto Y : %f; Magneto Z : %f\r\n", magneto_data[0], magneto_data[1], magneto_data[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

            mag_level_t level = magneto_thresh(initial_magneto, magneto_data);

            // Check if 3 seconds have passed at the SAME proximity level
            if (prox_level != 0 && !escaped){
                if (current_millis - prox_millis >= PB_interval_ms){
                    sprintf(message_print, "You escaped, good job!\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
                    escaped = true;
					buzzer_off();
                }
            }

            // Process proximity levels - always print when changing levels
            switch(level){
                case MAG_LEVEL_SAFE:
                    if (prox_level != 0){
                        // Transitioning to safe - can show escape message if timer expired
                        if (!escaped) {
                            sprintf(message_print, "You escaped, good job!\r\n");
                            HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
                        }
                        prox_level = 0;
                        escaped = true;
                        led_blink_interval = 0;
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
						buzzer_off();
                    }
                    break;
                case MAG_LEVEL_LOW:
                    if (prox_level != 1){
                        // Restart timer and update level
                        prox_millis = HAL_GetTick();
                        escaped = false;
                        prox_level = 1;
                        led_blink_interval = 1000;
                        
                        // Always print when entering a new level
                        sprintf(message_print, "Proximity Level: 1\r\n");
                        HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						buzzer_beep(700, 200);
					}
					
                    break;
                case MAG_LEVEL_MED:
                    if (prox_level != 2){
                        prox_millis = HAL_GetTick();
                        escaped = false;
                        prox_level = 2;
                        led_blink_interval = 500;
                        
                        sprintf(message_print, "Proximity Level: 2\r\n");
                        HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						buzzer_beep(1000, 200);
                    }
					
                    break;
                case MAG_LEVEL_HIGH:
                    if (prox_level != 3){
                        prox_millis = HAL_GetTick();
                        escaped = false;
                        prox_level = 3;
                        led_blink_interval = 200;
                        
                        sprintf(message_print, "Proximity Level: 3\r\n");
                        HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
						buzzer_beep(1000, 500);
                    }
					
                    break;
            }
        }

        // Temp, Pres and Humi sensor read every 1 second
        if (current_millis - sensor_CNR_previous_millis >= sensor_CNR_interval_ms){

            sensor_CNR_previous_millis = current_millis;

            temp_data = BSP_TSENSOR_ReadTemp(); // in degree Celsius
            pres_data = BSP_PSENSOR_ReadPressure(); // in hPa (hectopascal)
            humi_data = BSP_HSENSOR_ReadHumidity(); // in %RH (relative humidity)

			if (env_read_count == 0){
				initial_temp = temp_data;
				initial_pres = pres_data;
				initial_humi = humi_data;
				env_read_count = 1;
				sprintf(message_print, "Baseline captured: T: %dC, P: %dPa, H: %d%%\r\n", initial_temp, initial_pres, initial_humi);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				continue;
			}

			else{
				if (fabsf(temp_data - initial_temp) > TEMP_THRESHOLD){
                    sprintf(message_print, "Temperature spike detected! T: %dC (was %dC). Dangerous environment!\r\n", 
                            (int)temp_data, (int)initial_temp);
                    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
                }

                if (fabsf(pres_data - initial_pres) > PRES_THRESHOLD){
                    sprintf(message_print, "Pressure change detected! P: %d hPa (was %d hPa). Dangerous environment!\r\n", 
                            (int)pres_data, (int)initial_pres);
                    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
                }

                if (fabsf(humi_data - initial_humi) > HUMI_THRESHOLD){
                    sprintf(message_print, "Humidity change detected! H: %d%% (was %d%%). Dangerous environment!\r\n", 
                            (int)humi_data, (int)initial_humi);
                    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
                }
			}

			// Send all sensor data over UART
			sprintf(message_print, "Temp: %dC, Pres: %d hPa, Humi: %d%%\r\n", temp_data, pres_data, humi_data);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
        }
    }
}

static void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
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
		while(1);
	}
}

static void MX_GPIO_PB_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_GPIO_LED_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// No fundamental difference between ++i or i++ in this case, ++i is pre-increment, i++ is post-increment.
// Both will initialize i = 0, check if i < 3, then execute the body code after which performs ++i
// ++i generally more efficient since it only increments whereas i++ creates a copy then increments it

static bool is_accel_threshold(const float initial[3], const float current[3], float thresh){
    for (int i = 0; i < 3; ++i){
        float ini = initial[i];
        float cur = current[i];
        if (fabsf(cur - ini) > thresh){
            return true;
        }
    }
    return false;
}

static bool is_gyro_threshold(const float initial[3], const float current[3], float thresh){
    // Check if device is currently rotating (gyro measures rotation rate)
    for (int i = 0; i < 3; ++i){
        if (fabsf(current[i]) > thresh){
            return true;
        }
    }
    return false;
}

static mag_level_t magneto_thresh(const float initial[3], const float current[3]){
	float max_dev = 0.0f;
	for (int i = 0; i < 3; ++i){
		float deviation = fabsf(current[i] - initial[i]);
		if (deviation > max_dev){
			max_dev = deviation;
		}
	}

	if (max_dev > MAG_THRESHOLD_HIGH){
		return MAG_LEVEL_HIGH;
	}
	else if (max_dev > MAG_THRESHOLD_MED){
		return MAG_LEVEL_MED;
	}
	else if (max_dev > MAG_THRESHOLD_LOW){
		return MAG_LEVEL_LOW;
	}
	else{
		return MAG_LEVEL_SAFE;
	}
}

static void PWM_BUZZER_Init(void)
{
    // 1. Initialize GPIO for PWM output
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = BUZZER_PIN; // Defined above as PB4
	// Disconnects the pin from direct sw control and connects it to the timer
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
    GPIO_InitStruct.Pull = GPIO_NOPULL; // Disables internal pull-up/down resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // Sets speed to high to generate fast PWM signals
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3; // Alternate Function for TIM3
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

    // 2. Initialize TIM3 for PWM generation
    __HAL_RCC_TIM3_CLK_ENABLE(); // Enables clock for TIM3
    htim3.Instance = TIM3;
    // System Clock is 80MHz. Prescaler of 79 gives a 1MHz timer clock.
	// System Clock / (Prescaler + 1) = 80MHz / 80 = 1MHz
	// Meaning it counts up once every 1 microsecond
    htim3.Init.Prescaler = 79;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP; // Configs timer to count upwards from 0 to value stored in Period Register
    htim3.Init.Period = 0; // Will be set by set_buzzer_freq()
	// Frequency of PWM signal is Timer Clock / Period
	// No clock division, since we want full timer clock speed
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    // 3. Configure PWM Channel (Channel 1 for PB4)
    TIM_OC_InitTypeDef sConfigOC = {0};
	// PWM Mode 1: Output is high as long as the counter is less than the pulse value
    sConfigOC.OCMode = TIM_OCMODE_PWM1; // Sets timer to PWM mode 1
    sConfigOC.Pulse = 0; // Will be set later for 50% duty cycle
	// Sets output to active high
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	// Disable fast mode
	// Fast mode can force the output pin to inactive right after the trigger event
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}

void buzzer_on(void){
	// Commands Tim3 to start PWM on Channel 1 (PB4)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void buzzer_off(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void set_buzzer_freq(uint16_t freq)
{
	// If frequency is 0, stop the buzzer
    if (freq == 0) {
        // Sets the period to 0
        __HAL_TIM_SET_AUTORELOAD(&htim3, 0);
		// Set pulse to 0 to ensure no output
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        return;
    }
    // Calculate period for a 1MHz timer clock (How many microseconds per cycle?)
    uint32_t period = 1000000 / freq;
	// Set the auto-reload register to period - 1 (since counting starts from 0)
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);

    // Set duty cycle to 50%
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period / 2);
}

void buzzer_beep(uint16_t freq, uint32_t duration_ms)
{
    set_buzzer_freq(freq);
	// Records system time in ms when the beep starts
	buzzer_start_time = HAL_GetTick();
	// Tracks how long the buzzer should stay on
	buzzer_duration = duration_ms;
	buzzer_on();
	// Global flag to indicate buzzer is active
	buzzer_active = true;
}

void buzzer_beep_special(void)
{
	// Special init to enable non blocking beep sequence
	beep_seq_index = 0;
    beep_seq_active = true;
    beep_seq_in_pause = false;
    beep_seq_timer = HAL_GetTick();
    
    // Start first tone
    set_buzzer_freq(beep_sequence[0]);
    buzzer_on();
}

static void I2C1_Init(void)
{
    // 1. Configure I2C GPIO pins
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Bitwise OR to configure both SCL and SDA pins
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // PB8 for SCL, PB9 for SDA
	// In open-drain mode for I2C, the pins can pull the line low but need external pull-ups to go high
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Alternate Function Open Drain
	// Activate internal pull-up resistors for I2C lines
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	// Higher speed = sharper rise times, important for I2C
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	// From datasheet, AF4 is I2C1 for PB8 and PB9
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	// Apply all these settings to GPIOB
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 2. Configure I2C peripheral
    __HAL_RCC_I2C1_CLK_ENABLE();
	// Links the hi2c1 handle to the I2C1 peripheral
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00702991; // Standard Mode (100kHz) with 80MHz SysClk
	// Every device on the I2C bus must have a unique address, but as a master, we don't need one
    hi2c1.Init.OwnAddress1 = 0;
	// Standard 7 bit addressing mode
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	// We are not acting as a slave, so disable dual address mode
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
	// We are not listening to general call addresses
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	// We allow the slave to stretch the clock if needed
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	// Apply all settings and initialize the I2C peripheral
    HAL_I2C_Init(&hi2c1);
}

static void OLED_Init(void)
{
    // Setup u8g2 object
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8x8_byte_stm32_hw_i2c,
        u8x8_gpio_and_delay_stm32);

    u8g2_InitDisplay(&u8g2);      // send init sequence to the display
    u8g2_SetPowerSave(&u8g2, 0);  // wake up display
}

uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *)arg_ptr;
        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
        break;
    case U8X8_MSG_BYTE_INIT:
        break;
    case U8X8_MSG_BYTE_SET_DC:
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        if (HAL_I2C_Master_Transmit(&hi2c1, u8x8_GetI2CAddress(u8x8), buffer, buf_idx, HAL_MAX_DELAY) != HAL_OK)
            return 0; // Error
        break;
    default:
        return 0;
    }
    return 1;
}

uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        // Initialize hardware interfaces (done in main)
        break;
    case U8X8_MSG_DELAY_MILLI:
        {
			HAL_Delay(arg_int);
		}
        break;
    case U8X8_MSG_DELAY_I2C:
        // A small delay for I2C
        for (uint16_t n = 0; n < 400; n++)
        {
            __NOP();
        }
        break;
    // Other GPIO messages can be handled here if needed
    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}

static void TILT_Init(void)
{
	// 1. Configure Interrupt Pin PD11
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin; // Pin D11 for sensor interrupt
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	uint8_t ctrl;

	// 2. Turn on the accelerometer (Step 1 from datasheet)
	// Write 50h to CTRL1_XL (10h) to set ODR_XL = 208 Hz
	ctrl = 0x20;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, ctrl);

	// 3. Enable the tilt interrupt (Step 2 from datasheet)
	// Write 01h to TAP_CFG (58h) to set TILT_EN bit
	ctrl = 0x0C;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, ctrl);

	// 4. Route the tilt interrupt to the INT1 pin (Step 3 from datasheet)
	// Write 02h to MD1_CFG (5Eh) to set INT1_TILT bit
	ctrl = 0x02;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, ctrl);

	// 5. Enable the EXTI line for the interrupt pin
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void TILT_DeInit(void)
{
    uint8_t ctrl;

    // 1. Disable tilt interrupt routing to INT1 pin
    ctrl = 0x00;
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, ctrl);

    // 2. Disable the tilt feature
    ctrl = 0x00;
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, ctrl);
}
