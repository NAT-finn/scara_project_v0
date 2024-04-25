#ifndef stepmotor_h
#define stepmotor_h

/*
- Scara Robot (Basic) with 3 step motor for 3 Dof and 1 servo motor for 1 Dof
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define STEP_FULL_BASE          18000  // 1.8 Degrees*10000
#define ENDS_STOP_MAX           3

typedef struct {
    uint8_t dir_pin;    
    uint8_t step_pin;
    uint32_t full_step;   
} step_motor_t;

void init_motor(step_motor_t *motor, uint8_t step_pin, uint8_t dir_pin, uint8_t MS1, uint8_t MS2, uint8_t MS3);

void set_motor_high(step_motor_t motor);

void set_motor_low(step_motor_t motor);

void set_motor_dir(step_motor_t motor, uint8_t dir);

void run_motor_test(step_motor_t motor, uint16_t rev, uint16_t delay);

#endif