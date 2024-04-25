#ifndef servo_by_ledc_h
#define servo_by_ledc_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

#define SERVO_LEDC_INIT_BITS       LEDC_TIMER_10_BIT
#define FULL_DUTY                  ((1 << SERVO_LEDC_INIT_BITS) - 1)
#define SERVO_CHANNEL_MAX          8
#define MAX_SERVO_ANGLE            180
#define MIN_SERVO_ANGLE            0
#define MIN_WIDTH_US               500
#define MAX_WIDTH_US               2500
#define SERVO_FREQ                 50

typedef struct {
    gpio_num_t servo_pin[SERVO_CHANNEL_MAX];     
    ledc_channel_t ch[SERVO_CHANNEL_MAX];    
} servo_channel_t;

typedef struct {
    ledc_timer_t timer_number; 
    servo_channel_t channels;  
    uint8_t channel_num;  
} servo_config_t;

uint32_t calculate_duty(float angle);

esp_err_t servo_init(ledc_mode_t speed_mode, const servo_config_t *config);

esp_err_t servo_write(ledc_mode_t speed_mode, uint8_t channel, float angle);

#endif