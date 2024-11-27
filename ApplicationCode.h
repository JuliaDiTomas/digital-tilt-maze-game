/*
 * ApplicationCode.h
 *
 *  Created on: Dec 30, 2023
 *      Author: Xavion
 */

#include "LCD_Driver.h"
#include "stm32f4xx_hal.h"
#include "Gyro_Driver.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#ifndef INC_APPLICATIONCODE_H_
#define INC_APPLICATIONCODE_H_

void ApplicationInit(void);
void LCD_Visual_Demo(void);
void DrawMap(map);
void GenerateMap();
void DisplayGameEndScreen(bool game_win, uint8_t number_wps, float time);

// Game
#define GAME_TIME_FREQ 1 // Hz
#define GAME_TIME_MAX 120 // s
#define WP_REUSE 0
#define PIN_MAZE 1
#define BALL_THROWING_DIFFICULTY 30000
#define HARD_EDGED 1
#define CHECK_CORNERS 1

// Disruptor
#define DISRUPTOR_FREQ 20 // Hz
#define MAX_TIME 1 // s
#define MIN_ACTIVATION_ENERGY 6000 //J
#define DEPLETION_RATE 8000 //W
#define RECHARGE_RATE 600 //W
#define MAX_ENERGY 15000 //J
#define INITIAL_ENERGY 15000

// Physics
#define UPDATE_FREQUENCY 60 // Hz
#define SAMPLES_TO_AVERAGE 5
#define GRAVITY_FORCE 980 // kg*cm/s/s
#define BALL_MASS 10  // kg
#define ANGLE_GAIN 500

// Drawing the Map
#define BALL_RADIUS 8 // px
#define HOLE_RADIUS 8 // px
#define CONVERSION_CM_TO_PX 50
#define WP_RADIUS   5
#define NUM_WPS 4
#define NUM_HOLES 4
#define WALL_PROB 47
#define NUM_CELLS_Y 10
#define NUM_CELLS_X 10

//GPIO
#define PIN_LED_G GPIO_PIN_13
#define PIN_LED_R GPIO_PIN_14
#define PIN_BUTTON GPIO_PIN_0
#define PORT_BUTTON GPIOA
#define PORT_LED_G GPIOG
#define PORT_LED_R GPIOG
#define G_LED_FREQ 40 // Hz

// Gyro commands
#define GET_GYRO_X 0x28
#define GET_GYRO_Y 0x2A

// Event flags for position task's updates
#define VEL_UPDATE 1
#define GAME_WIN 2
#define GAME_LOSS 4

// Event flags for button's meanings
#define RESET_BUTTON 1
#define DISRUPTOR_ON 2
#define DISRUPTOR_OFF 4
#define DISRUPTOR_TIME 8
#define MAX_TIME_REACHED 16

typedef enum {
	NONE,
	RIGHT,
	BOTTOM,
	BOTH
} wall_on_cell;

typedef struct {
	int16_t holes_x[NUM_HOLES];
	int16_t holes_y[NUM_HOLES];
	int16_t wps_x[NUM_WPS];
	int16_t wps_y[NUM_WPS];
	wall_on_cell walls[NUM_CELLS_Y][NUM_CELLS_X];
	bool corners[NUM_CELLS_Y-1][NUM_CELLS_X-1];
} map;


#endif /* INC_APPLICATIONCODE_H_ */
