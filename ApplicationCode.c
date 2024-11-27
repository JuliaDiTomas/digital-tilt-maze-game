/*
 * ApplicationCode.c
 *
 *  Created on: April 27, 2024
 *      Author: Julia
 */

#include "ApplicationCode.h"
extern RNG_HandleTypeDef hrng;
static HAL_StatusTypeDef HAL_Status;
/* Static variables */


extern void initialise_monitor_handles(void); 


/* Static variables */

// Mutexes for global data
osMutexId_t ball_vel_mutex_id;
const osMutexAttr_t ball_vel_mutex_attr = {
  "Ball Velocity Mutex",
  0,    // attr_bits
  NULL,
  0U
};

osMutexId_t ball_pos_mutex_id;
const osMutexAttr_t ball_pos_mutex_attr = {
  "Ball Position Mutex",
  0,    // attr_bits
  NULL,
  0U
};

osMutexId_t map_mutex_id;
const osMutexAttr_t map_mutex_attr = {
  "Map Mutex",
  0,    // attr_bits
  NULL,
  0U
};

osMutexId_t obstacles_mutex_id;
const osMutexAttr_t obstacles_mutex_attr = {
  "Obstacles Enabled Mutex",
  0,    // attr_bits
  NULL,
  0U
};

osMutexId_t game_mutex_id;
const osMutexAttr_t game_mutex_attr = {
  "Game in Play Mutex",
  0,    // attr_bits
  NULL,
  0U
};

// ITC Semaphores
static osSemaphoreId_t velocity_sem_id;
static StaticSemaphore_t velocity_sem_cb; // control block
const osSemaphoreAttr_t velocity_sem_attr = {
		  .name = "Velocity Semaphore",
		  .attr_bits = 0,
		  .cb_mem = &velocity_sem_cb,
		  .cb_size = sizeof(velocity_sem_cb)
		};

static osSemaphoreId_t timer_sem_id;
static StaticSemaphore_t timer_sem_cb; // control block
const osSemaphoreAttr_t timer_sem_attr = {
		  .name = "Timer Semaphore",
		  .attr_bits = 0,
		  .cb_mem = &timer_sem_cb,
		  .cb_size = sizeof(timer_sem_cb)
		};

static osSemaphoreId_t button_sem_id;
static StaticSemaphore_t semaphore_cb; // control block
const osSemaphoreAttr_t semaphore_attr = {
		  .name = "Button Semaphore",
		  .attr_bits = 0,
		  .cb_mem = &semaphore_cb,
		  .cb_size = sizeof(semaphore_cb)
		};



// ITC Event Flags
static osEventFlagsId_t position_evt_id;
static StaticEventGroup_t position_evt_cb; // control block
const osEventFlagsAttr_t position_evt_attr = {
		  .name = "Position Event Group",
		  .attr_bits = 0,
		  .cb_mem = &position_evt_cb,
		  .cb_size = sizeof(position_evt_cb)
		};

static osEventFlagsId_t button_evt_id;
static StaticEventGroup_t button_evt_cb; // control block
const osEventFlagsAttr_t button_evt_attr = {
		  .name = "Button Event Group",
		  .attr_bits = 0,
		  .cb_mem = &button_evt_cb,
		  .cb_size = sizeof(button_evt_cb)
		};


// Vehicle Direction thread
static osThreadId_t velocity_task_id;
static StaticTask_t velocity_task_cb;
static uint32_t velocity_task_stack[256];
const osThreadAttr_t velocity_task_attr = {
		  .name = "Velocity Task",
		  .attr_bits = 0,
		  .cb_mem = &velocity_task_cb,
		  .cb_size = sizeof(velocity_task_cb),
		  .stack_mem = &velocity_task_stack,
		  .stack_size = sizeof(velocity_task_stack),
		  .priority = osPriorityNormal
		};
static void velocity_task_func (void *);



// Position thread
static osThreadId_t position_task_id;
static StaticTask_t position_task_cb;
static uint32_t position_task_stack[256];
const osThreadAttr_t position_task_attr = {
		  .name = "Position Task",
		  .attr_bits = 0,
		  .cb_mem = &position_task_cb,
		  .cb_size = sizeof(position_task_cb),
		  .stack_mem = &position_task_stack,
		  .stack_size = sizeof(position_task_stack),
		  .priority = osPriorityNormal
		};
static void position_task_func (void *);

// LCD Display thread
static osThreadId_t LCD_task_id;
static StaticTask_t LCD_task_cb;
static uint32_t LCD_task_stack[256];
const osThreadAttr_t LCD_task_attr = {
		  .name = "LCD Task",
		  .attr_bits = 0,
		  .cb_mem = &LCD_task_cb,
		  .cb_size = sizeof(LCD_task_cb),
		  .stack_mem = &LCD_task_stack,
		  .stack_size = sizeof(LCD_task_stack),
		  .priority = 23
		};
static void LCD_task_func (void *);

// LED Display thread
static osThreadId_t green_LED_task_id;
static StaticTask_t green_LED_task_cb;
static uint32_t green_LED_task_stack[256];
const osThreadAttr_t green_LED_task_attr = {
		  .name = "Green LED Task",
		  .attr_bits = 0,
		  .cb_mem = &green_LED_task_cb,
		  .cb_size = sizeof(green_LED_task_cb),
		  .stack_mem = &green_LED_task_stack,
		  .stack_size = sizeof(green_LED_task_stack),
		  .priority = 22
		};
static void green_LED_task_func (void *);

static osThreadId_t red_LED_task_id;
static StaticTask_t red_LED_task_cb;
static uint32_t red_LED_task_stack[256];
const osThreadAttr_t red_LED_task_attr = {
		  .name = "Red LED Task",
		  .attr_bits = 0,
		  .cb_mem = &red_LED_task_cb,
		  .cb_size = sizeof(red_LED_task_cb),
		  .stack_mem = &red_LED_task_stack,
		  .stack_size = sizeof(red_LED_task_stack),
		  .priority = 21
		};
static void red_LED_task_func (void *);

// Disruptor thread
static osThreadId_t disruptor_task_id;
static StaticTask_t disruptor_task_cb;
static uint32_t disruptor_task_stack[256];
const osThreadAttr_t disruptor_task_attr = {
		  .name = "Disruptor Task",
		  .attr_bits = 0,
		  .cb_mem = &disruptor_task_cb,
		  .cb_size = sizeof(disruptor_task_cb),
		  .stack_mem = &disruptor_task_stack,
		  .stack_size = sizeof(disruptor_task_stack),
		  .priority = osPriorityNormal
		};
static void disruptor_task_func (void *);


// Button thread
static osThreadId_t button_task_id;
static StaticTask_t button_task_cb;
static uint32_t button_task_stack[256];
const osThreadAttr_t button_task_attr = {
		  .name = "Button Task",
		  .attr_bits = 0,
		  .cb_mem = &button_task_cb,
		  .cb_size = sizeof(button_task_cb),
		  .stack_mem = &button_task_stack,
		  .stack_size = sizeof(button_task_stack),
		  .priority = osPriorityNormal
		};
static void button_task_func (void *);

// Reset thread
static osThreadId_t reset_task_id;
static StaticTask_t reset_task_cb;
static uint32_t reset_task_stack[256];
const osThreadAttr_t reset_task_attr = {
		  .name = "Reset Task",
		  .attr_bits = 0,
		  .cb_mem = &reset_task_cb,
		  .cb_size = sizeof(reset_task_cb),
		  .stack_mem = &reset_task_stack,
		  .stack_size = sizeof(reset_task_stack),
		  .priority = 25
		};
static void reset_task_func (void *);

// Timers
static osTimerId_t velocity_timer_id;
static StaticTimer_t velocity_timer_cb; // control block
const osTimerAttr_t velocity_timer_attr = {
		  .name = "Velocity Timer",
		  .attr_bits = 0,
		  .cb_mem = &velocity_timer_cb,
		  .cb_size = sizeof(velocity_timer_cb)
		};
static void velocity_timer_callback (void *);

static osTimerId_t game_timer_id;
static StaticTimer_t game_timer_cb; // control block
const osTimerAttr_t game_timer_attr = {
		  .name = "Game Timer",
		  .attr_bits = 0,
		  .cb_mem = &game_timer_cb,
		  .cb_size = sizeof(game_timer_cb)
		};
static void game_timer_callback (void *);

static osTimerId_t disruptor_timer_id;
static StaticTimer_t disruptor_timer_cb; // control block
const osTimerAttr_t disruptor_timer_attr = {
		  .name = "Disruptor Timer",
		  .attr_bits = 0,
		  .cb_mem = &disruptor_timer_cb,
		  .cb_size = sizeof(disruptor_timer_cb)
		};
static void disruptor_timer_callback (void *);

static osTimerId_t max_timer_id;
static StaticTimer_t max_timer_cb; // control block
const osTimerAttr_t max_timer_attr = {
		  .name = "Max Timer",
		  .attr_bits = 0,
		  .cb_mem = &max_timer_cb,
		  .cb_size = sizeof(max_timer_cb)
		};
static void max_timer_callback (void *);


// Global data
float x_vel, y_vel; // px/s
int x_pos, y_pos; // px
float x_angle, y_angle; // degrees from horizontal
map game_map;
bool game_on;
bool obstacles_enabled;
float game_time;
float energy;
int w;
int cell_size_y = 240/NUM_CELLS_Y;
int cell_size_x = 320/NUM_CELLS_X;

/**
* @brief Turn on disruptor and track energy
* @param default arg, not used in this app
*/
static void disruptor_task_func (void *){
	int flags_ret;
	osStatus_t status;
	while(1){
		// Pend disruptor button flag
		flags_ret = osEventFlagsWait(button_evt_id, (DISRUPTOR_ON|DISRUPTOR_OFF|DISRUPTOR_TIME|MAX_TIME_REACHED), osFlagsWaitAny, osWaitForever);
			if (flags_ret < 0)exit(61);


		status = osMutexAcquire(obstacles_mutex_id, osWaitForever); // Acquire access to position data
			if(status != osOK)exit(62);
			if ((flags_ret&MAX_TIME_REACHED)){ // turn off disruptor is time limit reached
							obstacles_enabled = 1;
			} else if ((flags_ret&DISRUPTOR_TIME)){
				// adjust energy level
				if (obstacles_enabled){
					energy = ((energy + RECHARGE_RATE/DISRUPTOR_FREQ) > MAX_ENERGY) ? MAX_ENERGY: energy + RECHARGE_RATE/DISRUPTOR_FREQ;
				} else {
					energy = (energy - DEPLETION_RATE/DISRUPTOR_FREQ);
					if (energy <= 0) {
						obstacles_enabled = 1;
						energy = 0;
					}
				}
			} // Toggle obstacles
			if ((flags_ret&DISRUPTOR_ON)&&(energy >= MIN_ACTIVATION_ENERGY)){
				obstacles_enabled = 0;
				// start timer if just turned on
				 status = osTimerStart(max_timer_id, (int)(1000*MAX_TIME));
													if (status != osOK) exit(179);

			} else if ((flags_ret&DISRUPTOR_OFF)){
				obstacles_enabled = 1;
				if (osTimerIsRunning(max_timer_id)){
					status = osTimerStop(max_timer_id);
					if(status != osOK)exit(178);
				}
			}
		status = osMutexRelease(obstacles_mutex_id); // Release access to map
			if(status != osOK)exit(63);



	}
}


/**
* @brief Reset game
* @details Generate new map, begin timer
* @param default arg, not used in this app
*/
static void reset_task_func (void *){
	int flags_ret;
	osStatus_t status;
	while(1){
		// Pend disruptor button flag
		flags_ret = osEventFlagsWait(button_evt_id, (RESET_BUTTON), osFlagsWaitAny, osWaitForever);
			if (flags_ret < 0)exit(134);

			status = osMutexAcquire(ball_pos_mutex_id, osWaitForever); // Acquire access to position data
								if(status != osOK)exit(130);
			status = osMutexAcquire(map_mutex_id, osWaitForever); // Acquire access to map
										if(status != osOK)exit(132);
			GenerateMap();
			status = osMutexRelease(map_mutex_id); // Release access to map
					if(status != osOK)exit(133);
			status = osMutexRelease(ball_pos_mutex_id); // Release access to velocity data
								if(status != osOK)exit(131);


			status = osMutexAcquire(ball_vel_mutex_id, osWaitForever); // Acquire access to velocity data
					if(status != osOK)exit(135);
			x_vel = 0;
			y_vel = 0;
			status = osMutexRelease(ball_vel_mutex_id); // Release access to velocity data
					if(status != osOK)exit(136);

			status = osMutexAcquire(obstacles_mutex_id, osWaitForever); // Acquire access to obstacles data
					if(status != osOK)exit(137);
			obstacles_enabled = 1;
			status = osMutexRelease(obstacles_mutex_id); // Release access to obstacles data
					if(status != osOK)exit(138);

			status = osMutexAcquire(game_mutex_id, osWaitForever); // Acquire access to game bool
					if(status != osOK)exit(139);
			game_on = 1;
			status = osMutexRelease(game_mutex_id); // Release access to game bool
					if(status != osOK)exit(140);

			game_time = 0;
			x_angle = 0;
			y_angle = 0;
			energy = INITIAL_ENERGY;
			w = 0;

			// start timers (game timer and velocity update)
		 status = osTimerStart(velocity_timer_id, (int)(1000/UPDATE_FREQUENCY/SAMPLES_TO_AVERAGE));
							if (status != osOK) exit(122);
		 status = osTimerStart(disruptor_timer_id, (int)(1000/DISRUPTOR_FREQ));
									if (status != osOK) exit(124);
			status = osTimerStart(game_timer_id, (int)(1000/GAME_TIME_FREQ));
							if (status != osOK) exit(123);


	}
}

/**
* @brief Send button to appropriate task
* @param default arg, not used in this app
*/
static void button_task_func (void *){
	osStatus_t status;
	GPIO_PinState button_state;
	int flags;
	while(1){
		// Pend semaphore from button IRQ handler
		status = osSemaphoreAcquire(button_sem_id, osWaitForever);
		if(status != osOK)exit(89);
		status = osMutexAcquire(game_mutex_id, osWaitForever); // Acquire access to velocity data
			if(status != osOK)exit(139);
		if(!game_on){
			flags = osEventFlagsSet(button_evt_id, (RESET_BUTTON));
			status = osMutexRelease(game_mutex_id); // Release access to velocity data
						if(status != osOK)exit(140);
			continue;
		}
	   status = osMutexRelease(game_mutex_id); // Release access to velocity data
			if(status != osOK)exit(140);

		// Read button state
		button_state = HAL_GPIO_ReadPin(PORT_BUTTON, PIN_BUTTON);
		if (button_state == 1){
			// Post to event flag group
			osEventFlagsSet(button_evt_id, DISRUPTOR_ON); // what does this function return? status?
		} else {
			osEventFlagsSet(button_evt_id, DISRUPTOR_OFF); // what does this function return? status?
		}
	}
}


/**
* @brief Display screen
* @param default arg, not used in this app
*/
static void LCD_task_func (void *){
	int flags_ret;
	osStatus_t status;
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);
	//uint8_t number_wps;
	while(1){

		// Read flags and determine what event happened
		flags_ret = osEventFlagsWait(position_evt_id, (VEL_UPDATE|GAME_WIN|GAME_LOSS), osFlagsWaitAny, osWaitForever);
				if (flags_ret < 0)exit(76);

		status = osMutexAcquire(game_mutex_id, osWaitForever); // Acquire access to position data
				if(status != osOK)exit(72);
		if (!game_on){
			status = osMutexRelease(game_mutex_id); // Release access to position data
					if(status != osOK)exit(73);
			continue;
		}

		// Game win
		if (flags_ret&GAME_WIN){
			game_on = 0;
			status = osTimerStop(velocity_timer_id);
							if (status != osOK) exit(119);
			status = osTimerStop(disruptor_timer_id);
							if (status != osOK) exit(127);
			status = osTimerStop(game_timer_id);
										if (status != osOK) exit(118);
			DisplayGameEndScreen(1, w, game_time);
		} else if (flags_ret&GAME_LOSS){
			// Game loss (unless wp_reuse mode, then might still be win)
			game_on = 0;
			status = osTimerStop(velocity_timer_id);
				if (status != osOK) exit(119);
			status = osTimerStop(disruptor_timer_id);
										if (status != osOK) exit(128);
			status = osTimerStop(game_timer_id);
							if (status != osOK) exit(117);


#if WP_REUSE
			if (w>1){
				DisplayGameEndScreen(1, w, game_time);
			} else {
				DisplayGameEndScreen(0, w, game_time);
			}
#else
			DisplayGameEndScreen(0, w, game_time);
#endif
		} else {
			//Clear screen
			LCD_Clear(LCD_COLOR_WHITE);
			// Get ball position and draw ball
			status = osMutexAcquire(ball_pos_mutex_id, osWaitForever); // Acquire access to position data
					if(status != osOK)exit(77);
			LCD_Draw_Circle_Fill(y_pos, x_pos, BALL_RADIUS, LCD_COLOR_BLUE);
			status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
					if(status != osOK)exit(78);
			// Get current map and draw on screen
			status = osMutexAcquire(map_mutex_id, osWaitForever); // Acquire access to map
					if(status != osOK)exit(79);
			DrawMap(game_map);
			status = osMutexRelease(map_mutex_id); // Release access to map
					if(status != osOK)exit(75);
		}
		status = osMutexRelease(game_mutex_id); // Release access to game bool
				if(status != osOK)exit(73);
	}
}


/**
* @brief Drive LEDs
* @param default arg, not used in this app
*/
static void green_LED_task_func (void *){
	float energy_local;
	while(1){
		energy_local = energy;
		HAL_GPIO_WritePin(PORT_LED_G, PIN_LED_G, GPIO_PIN_SET);
		osDelay((int)(1000/G_LED_FREQ*(energy_local/MAX_ENERGY)));
		HAL_GPIO_WritePin(PORT_LED_G, PIN_LED_G, GPIO_PIN_RESET);
		osDelay((int)(1000/G_LED_FREQ*((MAX_ENERGY-energy_local)/MAX_ENERGY)));
	}
}

/**
* @brief Drive LEDs
* @param default arg, not used in this app
*/
static void red_LED_task_func (void *){
	float energy_local;
	int half_period;
	while(1){
		energy_local = energy;
		half_period = (MIN_ACTIVATION_ENERGY-energy_local)/RECHARGE_RATE*100; // 1/10 the time it will take to be used again
		if (half_period <= 0){
			if ((energy_local-MIN_ACTIVATION_ENERGY)/DEPLETION_RATE*100 > 2){
				osDelay((energy_local-MIN_ACTIVATION_ENERGY)/DEPLETION_RATE*100);
			} else {
				osDelay(2);
			}
			continue;
		}
		if (half_period < 2){
			half_period = 2;
		}
		HAL_GPIO_WritePin(PORT_LED_R, PIN_LED_R, GPIO_PIN_SET);
		osDelay(half_period);
		HAL_GPIO_WritePin(PORT_LED_R, PIN_LED_R, GPIO_PIN_RESET);
		osDelay(half_period);
	}
}





/**
* @brief Update positions and check collisions
* @param default arg, not used in this app
*/
static void position_task_func (void *){
	int x_pos_local, y_pos_local;
	osStatus_t status;
	int i, j;
	wall_on_cell curr_cell, cell_above, cell_left;
	bool obstacles_en_local;
	bool bottom_left, top_left, top_right, bottom_right;
	float dtop_left, dtop_right, dbottom_left, dbottom_right; // Distances to cell corners
	bool corner_collision;
	float x_pos_in_cell, y_pos_in_cell;
		float theta;

	while(1){
		// Pend semaphore from velocity update task
		status = osSemaphoreAcquire(velocity_sem_id, osWaitForever);
				if(status != osOK)exit(99);

		// Find out if obstacles are on
		status = osMutexAcquire(obstacles_mutex_id, osWaitForever); // Acquire access to obstacles bool
				if(status != osOK)exit(98);
		obstacles_en_local = obstacles_enabled;
		status = osMutexRelease(obstacles_mutex_id); // Release access to obstacles bool
				if(status != osOK)exit(97);

		// Get current ball position
		status = osMutexAcquire(ball_pos_mutex_id, osWaitForever); // Acquire access to position data
				if(status != osOK)exit(102);
		y_pos_local = y_pos;
		x_pos_local = x_pos;
		status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
				if(status != osOK)exit(101);


		// Get current cell that ball lies in
		i = (y_pos_local)/cell_size_y;
		j = (x_pos_local)/cell_size_x;

		// Get walls/corners for that cell and check holes/waypoints
		status = osMutexAcquire(map_mutex_id, osWaitForever); // Acquire access to position data
				if(status != osOK)exit(92);

#if WP_REUSE
		if (pow(pow((game_map.wps_x[w%NUM_WPS]-x_pos_local),2)+pow((game_map.wps_y[w%NUM_WPS]-y_pos_local),2),0.5)<BALL_RADIUS){
			w++;
		}

#else
			if (pow(pow((game_map.wps_x[w]-x_pos_local),2)+pow((game_map.wps_y[w]-y_pos_local),2),0.5)<BALL_RADIUS){
				w++;
			}

		if (w==NUM_WPS){
			status = osMutexRelease(map_mutex_id); // Release access to position data
						if(status != osOK)exit(991);
			osEventFlagsSet(position_evt_id, GAME_WIN);
			continue;
		}
#endif

		if (obstacles_en_local){
			for (int k=0; k<NUM_HOLES; k++){
				if (pow(pow((game_map.holes_x[k]-x_pos_local),2)+pow((game_map.holes_y[k]-y_pos_local),2),0.5)<HOLE_RADIUS){
//					status = osMutexRelease(map_mutex_id); // Release access to position data
//								if(status != osOK)exit(991);
								osEventFlagsSet(position_evt_id, GAME_LOSS);
								continue;
				}
			}
		}
//		top_left = ((i>0)&&(j>0)) ? game_map.corners[i-1][j-1] : 1;
//		top_right = ((i>0)&&(j<8)) ? game_map.corners[i-1][j] : 1;
//		bottom_left = ((i<8)&&(j>0)) ? game_map.corners[i][j-1] : 1;
//		bottom_right = ((i<8)&&(j<8)) ? game_map.corners[i][j] : 1;

		curr_cell = game_map.walls[i][j];
		if (j>0){
			cell_above = game_map.walls[i][j-1];
		} else {
			cell_above = NONE;
		}
		if (i>0){
			cell_left = game_map.walls[i-1][j];
		} else {
			cell_left = NONE;
		}
		status = osMutexRelease(map_mutex_id); // Release access to position data
				if(status != osOK)exit(91);


		status = osMutexAcquire(ball_pos_mutex_id, osWaitForever); // Acquire access to position data
				if(status != osOK)exit(96);

		status = osMutexAcquire(ball_vel_mutex_id, osWaitForever); // Acquire access to position data
				if(status != osOK)exit(95);

		// Update position
		x_pos = x_pos + x_vel/UPDATE_FREQUENCY;
		y_pos = y_pos + y_vel/UPDATE_FREQUENCY;

		// Check if move caused collision

#if HARD_EDGED
		if (x_pos>(319-BALL_RADIUS)){
			x_pos = (319-BALL_RADIUS);
			x_vel = 0;
		} else if (x_pos<BALL_RADIUS){
			x_pos = BALL_RADIUS;
			x_vel = 0;
		} else if (obstacles_en_local){
			if (((cell_above==BOTH) || (cell_above==BOTTOM)) && (x_pos-j*cell_size_x<BALL_RADIUS)){
				x_pos = j*cell_size_x+1+BALL_RADIUS;
				x_vel = (x_vel < 0)? 0: x_vel;
			} else if (((curr_cell==BOTH) || (curr_cell==BOTTOM)) && (x_pos-j*cell_size_x>(cell_size_x-1-BALL_RADIUS))){
				x_pos = j*cell_size_x+cell_size_x-2-BALL_RADIUS;
				x_vel = (x_vel > 0)? 0: x_vel;
			}
		}

		if (y_pos>(239-BALL_RADIUS)){
			y_pos = (239-BALL_RADIUS);
			y_vel = 0;
		} else if (y_pos<BALL_RADIUS){
			y_pos = BALL_RADIUS;
			y_vel = 0;
		} else if (obstacles_en_local){
			if (((cell_left==BOTH) || (cell_left==RIGHT)) && (y_pos-i*cell_size_y<BALL_RADIUS)){
				y_pos = i*cell_size_y+1+BALL_RADIUS;
				y_vel = (y_vel < 0)? 0: y_vel;
			} else if (((curr_cell==BOTH) || (curr_cell==RIGHT)) && (y_pos-i*cell_size_y>(cell_size_y-1-BALL_RADIUS))){
				y_pos = i*cell_size_y+cell_size_y-2-BALL_RADIUS;
				y_vel = (y_vel > 0)? 0: y_vel;
			}
		}
#else
		if (x_pos>(319)){
			osEventFlagsSet(position_evt_id, GAME_LOSS);
			status = osMutexRelease(ball_vel_mutex_id); // Release access to position data
					if(status != osOK)exit(890);
			status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
						if(status != osOK)exit(891);
			continue;
		} else if (x_pos<0){
			osEventFlagsSet(position_evt_id, GAME_LOSS);
			status = osMutexRelease(ball_vel_mutex_id); // Release access to position data
					if(status != osOK)exit(890);
			status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
						if(status != osOK)exit(891);
			continue;
		} else if (obstacles_en_local){
			if (((cell_above==BOTH) || (cell_above==BOTTOM)) && (x_pos-j*cell_size_x<BALL_RADIUS)){
				x_pos = j*cell_size_x+1+BALL_RADIUS;
				x_vel = (x_vel < 0)? 0: x_vel;
			} else if (((curr_cell==BOTH) || (curr_cell==BOTTOM)) && (x_pos-j*cell_size_x>(cell_size_x-1-BALL_RADIUS))){
				x_pos = j*cell_size_x+cell_size_x-2-BALL_RADIUS;
				x_vel = (x_vel > 0)? 0: x_vel;
			}
		}
		if (y_pos>(239)){
			osEventFlagsSet(position_evt_id, GAME_LOSS);
			status = osMutexRelease(ball_vel_mutex_id); // Release access to position data
					if(status != osOK)exit(890);
			status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
						if(status != osOK)exit(891);
			continue;
		} else if (y_pos<0){
			osEventFlagsSet(position_evt_id, GAME_LOSS);
			status = osMutexRelease(ball_vel_mutex_id); // Release access to position data
					if(status != osOK)exit(890);
			status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
						if(status != osOK)exit(891);
			continue;
		} else if (obstacles_en_local){
			if (((cell_left==BOTH) || (cell_left==RIGHT)) && (y_pos-i*cell_size_y<BALL_RADIUS)){
				y_pos = i*cell_size_y+1+BALL_RADIUS;
				y_vel = (y_vel < 0)? 0: y_vel;
			} else if (((curr_cell==BOTH) || (curr_cell==RIGHT)) && (y_pos-i*cell_size_y>(cell_size_y-1-BALL_RADIUS))){
				y_pos = i*cell_size_y+cell_size_y-2-BALL_RADIUS;
				y_vel = (y_vel > 0)? 0: y_vel;
			}
		}
#endif
		if (obstacles_en_local&&(y_vel != 0)&&(x_vel != 0)){
			i = (y_pos)/cell_size_y;
			j = (x_pos)/cell_size_x;

			y_pos_in_cell = y_pos-i*24;
			x_pos_in_cell = x_pos-j*32;

			top_left = ((i>0)&&(j>0)) ? game_map.corners[i-1][j-1] : 1;
			top_right = ((i>0)&&(j<8)) ? game_map.corners[i-1][j] : 1;
			bottom_left = ((i<8)&&(j>0)) ? game_map.corners[i][j-1] : 1;
			bottom_right = ((i<8)&&(j<8)) ? game_map.corners[i][j] : 1;

			dtop_left = pow(pow(y_pos_in_cell,2)+pow(x_pos_in_cell,2),0.5);
			dtop_right = pow(pow(y_pos_in_cell-24,2)+pow(x_pos_in_cell,2),0.5);
			dbottom_left = pow(pow(y_pos_in_cell,2)+pow(x_pos_in_cell-32,2),0.5);
			dbottom_right = pow(pow(y_pos_in_cell-24,2)+pow(x_pos_in_cell-32,2),0.5);
			// Determine if there is a collision with a corner
			// if so, move backward in same direction
			if ((dtop_left<BALL_RADIUS-1)&&top_left){
				theta = atan(abs(x_vel/y_vel));
				if (y_pos>y_pos_local){
					y_pos = y_pos - dtop_left*cos(theta);
				} else {
					y_pos = y_pos + dtop_left*cos(theta);
				}
				if (x_pos>x_pos_local){
					x_pos = x_pos - dtop_left*sin(theta);
				} else {
					x_pos = x_pos + dtop_left*sin(theta);
				}
				x_vel = x_vel/2;
				y_vel = y_vel/2;
			} else if ((dtop_right<BALL_RADIUS-1)&&top_right){
				theta = atan(abs(x_vel/y_vel));
				if (y_pos>y_pos_local){
					y_pos = y_pos - dtop_right*cos(theta);
				} else {
					y_pos = y_pos + dtop_right*cos(theta);
				}
				if (x_pos>x_pos_local){
					x_pos = x_pos - dtop_right*sin(theta);
				} else {
					x_pos = x_pos + dtop_right*sin(theta);
				}
				x_vel = x_vel/2;
				y_vel = y_vel/2;
			} else if ((dbottom_right<BALL_RADIUS-1)&&bottom_right){
				theta = atan(abs(x_vel/y_vel));
				if (y_pos>y_pos_local){
					y_pos = y_pos - dbottom_right*cos(theta);
				} else {
					y_pos = y_pos + dbottom_right*cos(theta);
				}
				if (x_pos>x_pos_local){
					x_pos = x_pos - dbottom_right*sin(theta);
				} else {
					x_pos = x_pos + dbottom_right*sin(theta);
				}
				x_vel = x_vel/2;
				y_vel = y_vel/2;
			} else if ((dbottom_left<BALL_RADIUS-1)&&bottom_left){
				theta = atan(abs(x_vel/y_vel));
				if (y_pos>y_pos_local){
					y_pos = y_pos - dbottom_left*cos(theta);
				} else {
					y_pos = y_pos + dbottom_left*cos(theta);
				}
				if (x_pos>x_pos_local){
					x_pos = x_pos - dbottom_left*sin(theta);
				} else {
					x_pos = x_pos + dbottom_left*sin(theta);
				}
				x_vel = x_vel/2;
				y_vel = y_vel/2;
			}
		}

		status = osMutexRelease(ball_vel_mutex_id); // Release access to position data
				if(status != osOK)exit(94);

		status = osMutexRelease(ball_pos_mutex_id); // Release access to position data
					if(status != osOK)exit(93);
		// Post to event flag group
		osEventFlagsSet(position_evt_id, VEL_UPDATE); // what does this function return? status?

	}
}


/**
* @brief Read from gyro and update global variable
* @param default arg, not used in this app
*/
static void velocity_task_func (void *){
	int x_gyro = 0, y_gyro = 0;
	int old_x_gyro = 0, old_y_gyro = 0;
	float acceleration_of_gravity = GRAVITY_FORCE/BALL_MASS; // cm/s/s
	uint8_t sample_count = 0;
	uint16_t conversion_deg_per_sec_to_gyro_num = 32*1000/ANGLE_GAIN;
	osStatus_t status;
	while(1){
		status = osSemaphoreAcquire(timer_sem_id, osWaitForever);
		if (sample_count < SAMPLES_TO_AVERAGE){
			// Read from gyro for x and y axes
			x_gyro = x_gyro + Gyro_Get_Velocity(GET_GYRO_X);
			y_gyro = y_gyro + Gyro_Get_Velocity(GET_GYRO_Y);
			sample_count += 1;
		} else {
			x_gyro = (x_gyro + Gyro_Get_Velocity(GET_GYRO_X))/SAMPLES_TO_AVERAGE/conversion_deg_per_sec_to_gyro_num;
			y_gyro = (y_gyro + Gyro_Get_Velocity(GET_GYRO_Y))/SAMPLES_TO_AVERAGE/conversion_deg_per_sec_to_gyro_num;

			#if PIN_MAZE
				if (((old_y_gyro-y_gyro)*(120-y_pos)>BALL_THROWING_DIFFICULTY)||((old_x_gyro-x_gyro)*(160-x_pos)>1.75*BALL_THROWING_DIFFICULTY)){
					osEventFlagsSet(position_evt_id, GAME_LOSS);
													continue;
				}
			#endif

			old_x_gyro = x_gyro;
			old_y_gyro = y_gyro;

			x_angle = x_angle + (x_gyro)/UPDATE_FREQUENCY;
			y_angle = y_angle + (y_gyro)/UPDATE_FREQUENCY;
			if ((x_angle > 90)||(y_angle > 90)||(y_angle < -90)||(x_angle < -90)){
				osEventFlagsSet(position_evt_id, GAME_LOSS);
					continue;
			}


			// Update velocities
			status = osMutexAcquire(ball_vel_mutex_id, osWaitForever); // Acquire access to velocity data
					if(status != osOK)exit(1);
			x_vel = x_vel + (acceleration_of_gravity*sin(x_angle*3.141593/180))/UPDATE_FREQUENCY*CONVERSION_CM_TO_PX;
			y_vel = y_vel + (acceleration_of_gravity*sin(y_angle*3.141593/180))/UPDATE_FREQUENCY*CONVERSION_CM_TO_PX;
			status = osMutexRelease(ball_vel_mutex_id); // Release access to velocity data
					if(status != osOK)exit(2);
			// Post to semaphore to let position task know data has been updated
			osStatus_t status = osSemaphoreRelease(velocity_sem_id);
					if (status != osOK){
						exit(3);}
			sample_count = 0;
			x_gyro = 0;
			y_gyro = 0;
		}
	}
}

void ApplicationInit(void)
{
	initialise_monitor_handles(); // Allows printf functionality
    LTCD__Init();
    LTCD_Layer_Init(0);
    LCD_Clear(LCD_COLOR_WHITE);

    HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Enable button interrupt
	HAL_NVIC_SetPriority(EXTI0_IRQn, 13, 13); // button interrupt priority
	HAL_GPIO_WritePin(PORT_LED_R, PIN_LED_R, GPIO_PIN_RESET); // Turn off red led
	HAL_GPIO_WritePin(PORT_LED_G, PIN_LED_G, GPIO_PIN_RESET); // Turn off green led
	Gyro_Init();

	//srand(6542);
    HAL_Status = HAL_RNG_Init (&hrng);

	GenerateMap();

	x_vel = 0;
	y_vel = 0;
	game_on = 1;
	obstacles_enabled = 1;
	game_time = 0;
	y_angle = 0;
	y_angle = 0;
	energy = INITIAL_ENERGY;
	w = 0;

	ball_vel_mutex_id = osMutexNew(&ball_vel_mutex_attr);
	while (ball_vel_mutex_id==NULL){}
	ball_pos_mutex_id = osMutexNew(&ball_pos_mutex_attr);
	while (ball_pos_mutex_id==NULL){}
	map_mutex_id = osMutexNew(&map_mutex_attr);
	while (map_mutex_id==NULL){}
	obstacles_mutex_id = osMutexNew(&obstacles_mutex_attr);
	while (obstacles_mutex_id==NULL){}
	game_mutex_id = osMutexNew(&game_mutex_attr);
	while (game_mutex_id==NULL){}


	velocity_sem_id = osSemaphoreNew(1, 0, &velocity_sem_attr);
	while (velocity_sem_id==NULL){}
	timer_sem_id = osSemaphoreNew(1, 0, &timer_sem_attr);
	while (timer_sem_id==NULL){}
	button_sem_id = osSemaphoreNew(10, 0, &semaphore_attr);
	while (button_sem_id==NULL){}
    position_evt_id = osEventFlagsNew(&position_evt_attr);
    while (position_evt_id==NULL){}
    button_evt_id = osEventFlagsNew(&button_evt_attr);
    while (button_evt_id==NULL){}

    HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Enable button interrupt
    HAL_NVIC_SetPriority(EXTI0_IRQn, 13, 13); // button interrupt priority

	// Initialize threads
	velocity_task_id = osThreadNew(velocity_task_func, (void *)0, &velocity_task_attr);
	while (velocity_task_id==NULL){}
	position_task_id = osThreadNew(position_task_func, (void *)0, &position_task_attr);
	while (position_task_id==NULL){}
	disruptor_task_id = osThreadNew(disruptor_task_func, (void *)0, &disruptor_task_attr);
	while (disruptor_task_id==NULL){}
	button_task_id = osThreadNew(button_task_func, (void *)0, &button_task_attr);
	while (button_task_id==NULL){}
	reset_task_id = osThreadNew(reset_task_func, (void *)0, &reset_task_attr);
	while (reset_task_id==NULL){}
	LCD_task_id = osThreadNew(LCD_task_func, (void *)0, &LCD_task_attr);
	while (LCD_task_id==NULL){}
	green_LED_task_id = osThreadNew(green_LED_task_func, (void *)0, &green_LED_task_attr);
	while (green_LED_task_id==NULL){}
	red_LED_task_id = osThreadNew(red_LED_task_func, (void *)0, &red_LED_task_attr);
	while (red_LED_task_id==NULL){}

	velocity_timer_id = osTimerNew(velocity_timer_callback, osTimerPeriodic, (void *)0, &velocity_timer_attr);
	while (velocity_timer_id==NULL){}
	osStatus_t status = osTimerStart(velocity_timer_id, (int)(1000/UPDATE_FREQUENCY/SAMPLES_TO_AVERAGE));
	if (status != osOK) exit(120);
	max_timer_id = osTimerNew(max_timer_callback, osTimerOnce, (void *)0, &max_timer_attr);
	while (max_timer_id==NULL){}
	disruptor_timer_id = osTimerNew(disruptor_timer_callback, osTimerPeriodic, (void *)0, &disruptor_timer_attr);
	while (disruptor_timer_id==NULL){}
	status = osTimerStart(disruptor_timer_id, (int)(1000/DISRUPTOR_FREQ));
	if (status != osOK) exit(102);

	game_timer_id = osTimerNew(game_timer_callback, osTimerPeriodic, (void *)0, &game_timer_attr);
	while (game_timer_id==NULL){}
	status = osTimerStart(game_timer_id, (int)(1000/GAME_TIME_FREQ));
	if (status != osOK) exit(121);


}


void GenerateMap(){
	uint32_t r;
	//float r_p;

		for (int k=0; k<NUM_HOLES; k++){
			HAL_RNG_GenerateRandomNumber(&hrng, &r);
			game_map.holes_x[k] = r%(320-2*HOLE_RADIUS)+HOLE_RADIUS;
			HAL_RNG_GenerateRandomNumber(&hrng, &r);
			game_map.holes_y[k] = r%(240-2*HOLE_RADIUS)+HOLE_RADIUS;
		}
		for (int k=0; k<NUM_WPS; k++){
			HAL_RNG_GenerateRandomNumber(&hrng, &r);
					game_map.wps_x[k] = r%(320-2*WP_RADIUS)+WP_RADIUS;
					HAL_RNG_GenerateRandomNumber(&hrng, &r);
					game_map.wps_y[k] = r%(240-2*WP_RADIUS)+WP_RADIUS;
				}

		for (int i=0; i<NUM_CELLS_Y; i++){
			for (int j=0; j<NUM_CELLS_X; j++){
				HAL_RNG_GenerateRandomNumber(&hrng, &r);
				r=100*(r%100);
				//r_p = r/100;
				game_map.walls[i][j] = (r<pow(WALL_PROB, 2)) ? BOTH: (r<(pow(WALL_PROB, 2)+WALL_PROB*(100-WALL_PROB))) ? BOTTOM: (r<(pow(WALL_PROB, 2)+2*WALL_PROB*(100-WALL_PROB))) ? RIGHT: NONE ;
			}
		}

		for (int m=0; m<9; m++){
			for (int n=0; n<9; n++){
				if ((game_map.walls[m][n]!=NONE)||(game_map.walls[m+1][n]==BOTTOM)||(game_map.walls[m+1][n]==BOTH)||(game_map.walls[m][n+1]==RIGHT)||(game_map.walls[m][n+1]==BOTH)){
					game_map.corners[m][n] = 1;
				} else {
					game_map.corners[m][n] = 0;
				}
			}
		}

		// do not generate ball on a hole or in a wall
		bool ball_on_hole = 1;
		while (ball_on_hole==1){
			ball_on_hole = 0;
			HAL_RNG_GenerateRandomNumber(&hrng, &r);
			x_pos = cell_size_x*(r%10)+16;
			HAL_RNG_GenerateRandomNumber(&hrng, &r);
			y_pos = cell_size_y*(r%10)+12;

			for (uint8_t k=0; k<NUM_HOLES; k++){
				if (pow(pow((game_map.holes_x[k]-x_pos),2)+pow((game_map.holes_y[k]-x_pos),2),0.5)<HOLE_RADIUS){
					ball_on_hole = 1;
				}
			}
		}
}


void DrawMap(map){
	for (int i=0; i<NUM_HOLES; i++){
		LCD_Draw_Circle_Fill(game_map.holes_y[i], game_map.holes_x[i], HOLE_RADIUS, LCD_COLOR_BLACK);
	}
	// next waypoint in yellow, one after that in red, rest in magenta
#if WP_REUSE
	for (int i=0; i<NUM_WPS; i++){
				LCD_Draw_Circle_Fill(game_map.wps_y[i], game_map.wps_x[i], WP_RADIUS, LCD_COLOR_MAGENTA);
			}
	LCD_Draw_Circle_Fill(game_map.wps_y[w%NUM_WPS], game_map.wps_x[w%NUM_WPS], WP_RADIUS, LCD_COLOR_YELLOW);
	LCD_Draw_Circle_Fill(game_map.wps_y[(w+1)%NUM_WPS], game_map.wps_x[(w+1)%NUM_WPS], WP_RADIUS, LCD_COLOR_RED);

#else
	for (int i=w; i<NUM_WPS; i++){
			LCD_Draw_Circle_Fill(game_map.wps_y[i], game_map.wps_x[i], WP_RADIUS, LCD_COLOR_MAGENTA);
		}
	LCD_Draw_Circle_Fill(game_map.wps_y[w], game_map.wps_x[w], WP_RADIUS, LCD_COLOR_YELLOW);
	if (w<NUM_WPS-1){
		LCD_Draw_Circle_Fill(game_map.wps_y[w+1], game_map.wps_x[w+1], WP_RADIUS, LCD_COLOR_RED);
	}
#endif


	for (int k=0; k<NUM_CELLS_Y; k++){
		for (int j=0; j<NUM_CELLS_X; j++){
			if (game_map.walls[k][j]==BOTH){
				LCD_Draw_Horizontal_Line(cell_size_y*k, cell_size_x*j+cell_size_x-1, cell_size_y-1, LCD_COLOR_BLACK);
				LCD_Draw_Vertical_Line(cell_size_y*k+cell_size_y-1, cell_size_x*j, cell_size_x-1, LCD_COLOR_BLACK);
			} else if (game_map.walls[k][j]==BOTTOM){
				LCD_Draw_Horizontal_Line(cell_size_y*k, cell_size_x*j+cell_size_x-1, cell_size_y-1, LCD_COLOR_BLACK);
			}  else if (game_map.walls[k][j]==RIGHT){
				LCD_Draw_Vertical_Line(cell_size_y*k+cell_size_y-1, cell_size_x*j, cell_size_x-1, LCD_COLOR_BLACK);
			}
		}
	}
}


void DisplayGameEndScreen(bool game_win, uint8_t number_wps, float time){

	LCD_Clear(LCD_COLOR_WHITE);

	if (game_win){
		LCD_DisplayString(50,20,"You win!");
	} else {
		LCD_DisplayString(50,20,"You lose!");
	}

	LCD_DisplayString(40,50,"Waypoints:");
	LCD_DisplayNumber(190,50,number_wps);
	LCD_DisplayString(50,80,"Time (s):");
	LCD_DisplayNumber(185,80,time);
if (game_win){
#if WP_REUSE
	LCD_DisplayString(70,110,"Score:");
	LCD_DisplayNumber(185,110,(int)(number_wps/time));
#else
	LCD_DisplayString(70,110,"Score:");
	LCD_DisplayNumber(185,110,time);
#endif
}
	LCD_DisplayString(50,220,"Press user");
	LCD_DisplayString(50,250,"button to");
	LCD_DisplayString(50,280,"play again");
}



/**
* @brief User button interrupt IRQ handler
* @details Set button semaphore
*/
void EXTI0_IRQHandler(void){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn); // Disable to prevent interrupt while in this function
	osStatus_t status = osSemaphoreRelease(button_sem_id);
		if (status != osOK){
			exit(88);
		}
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0); // Clear interrupt
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); // Re-enable
}

/**
* @brief Record button input as a hold
* @param default arg, not used in this app
*/
void velocity_timer_callback (void * arg){
	(void) &arg;
	osStatus_t status = osSemaphoreRelease(timer_sem_id);
						if (status != osOK){
							exit(680);}
}

/**
* @brief Record button input as a hold
* @param default arg, not used in this app
*/
void disruptor_timer_callback (void * arg){
	(void) &arg;
	osStatus_t status = osEventFlagsSet(button_evt_id, DISRUPTOR_TIME);
						if (status != osOK){
							exit(688);}
}

/**
* @brief Record button input as a hold
* @param default arg, not used in this app
*/
void max_timer_callback (void * arg){
	(void) &arg;
	osStatus_t status = osEventFlagsSet(button_evt_id, MAX_TIME_REACHED);
						if (status != osOK){
							exit(689);}
}

/**
* @brief Record button input as a hold
* @param default arg, not used in this app
*/
void game_timer_callback (void * arg){
	(void) &arg;
	game_time += 1/GAME_TIME_FREQ;
#if WP_REUSE
	if (w<=1 && game_time>=GAME_TIME_MAX){
			osEventFlagsSet(position_evt_id, GAME_LOSS);
		}
#else
	if (game_time>=GAME_TIME_MAX){
		osEventFlagsSet(position_evt_id, GAME_LOSS);
	}
#endif
}


void LCD_Visual_Demo(void)
{
	visualDemo();
}
