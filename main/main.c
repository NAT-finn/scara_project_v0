#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "servo_by_ledc.h"
#include "hx711.h"
#include "stepmotor.h"
#include "scara_4_dof.h"
#include "sdkconfig.h"

#define DEBUG_TAG "T"

#define MOTOR_1_DIR_PIN     27
#define MOTOR_1_STEP_PIN    13

#define MOTOR_2_DIR_PIN     16
#define MOTOR_2_STEP_PIN    17

#define MOTOR_3_DIR_PIN     25
#define MOTOR_3_STEP_PIN    26

#define END_STOP_1          36
#define END_STOP_2          39
#define END_STOP_3          34

#define SERVO_1_PIN         32
#define SERVO_2_PIN         33

#define SERVO_1_ANG_HOME         32
#define SERVO_2_ANG_OPEN         33
#define SERVO_2_ANG_CLOSE        100

#define GPIO_HX711_DATA          13
#define GPIO_HX711_SCLK          27
#define AVG_SAMPLES              10

#define DETECT_OBJECT_PIN        4

#define LEVEL_WEIGHT_0           10000
#define LEVEL_WEIGHT_1           20000

unsigned long weight =0;

scara_motor_t scmotor1, scmotor2, scmotor3;
step_motor_t motor1, motor2, motor3;

typedef void (*functionPointers)(void);
const functionPointers func_point_motor3[] = {
    run_point_1_1,
    run_point_1_2,
    run_point_1_3
};

static uint8_t classification(unsigned long w){
    if(w > LEVEL_WEIGHT_1){
        return 3;
    }else if(w >= LEVEL_WEIGHT_0 && w <= LEVEL_WEIGHT_0){
        return 2;
    }else{
        return 1;
    }
}

static void robot_run_function(uint8_t class){
    int old_motor1_loc = 0, old_motor2_loc = 0;
    int new_motor1_loc = 0, new_motor2_loc = 0;
    run_point_0_0(&old_motor1_loc, &old_motor2_loc, &new_motor1_loc, &new_motor2_loc);
    ESP_LOGE(DEBUG_TAG, "update %d | %d", new_motor1_loc, new_motor2_loc);
    servo_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, SERVO_2_ANG_OPEN);
    vTaskDelay(500);
    run_point_0_1(&old_motor1_loc, &old_motor2_loc, &new_motor1_loc, &new_motor2_loc);
    ESP_LOGE(DEBUG_TAG, "update %d | %d", new_motor1_loc, new_motor2_loc);
    servo_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, SERVO_2_ANG_CLOSE);
    vTaskDelay(500);
    run_point_1(&old_motor1_loc, &old_motor2_loc, &new_motor1_loc, &new_motor2_loc);
    ESP_LOGE(DEBUG_TAG, "update %d | %d", new_motor1_loc, new_motor2_loc);
    func_point_motor3[class-1]();
    vTaskDelay(500);
    servo_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, SERVO_2_ANG_OPEN);
    vTaskDelay(500);
    servo_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, SERVO_2_ANG_CLOSE);
}

void app_main(void)
{
    init_motor(&motor1, MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN, 0, 0, 0);
    init_motor(&motor2, MOTOR_2_STEP_PIN, MOTOR_2_DIR_PIN, 1, 1, 0);
    init_motor(&motor3, MOTOR_3_STEP_PIN, MOTOR_3_DIR_PIN, 1, 1, 0);
    config_scara_motor(motor1, motor2, motor3, END_STOP_1, END_STOP_2, END_STOP_3);

    servo_config_t servo_cfg = {
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_1_PIN,
                SERVO_2_PIN,
                },
            .ch = {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
            },
        },
        .channel_num = 2,
    };
    servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

    gpio_config_t GPIO_detect_object = {};
    GPIO_detect_object.pin_bit_mask = ((uint64_t)1 << DETECT_OBJECT_PIN) ;
    GPIO_detect_object.mode = GPIO_MODE_INPUT;
    GPIO_detect_object.pull_up_en = 0;
    GPIO_detect_object.pull_down_en = 0;
    GPIO_detect_object.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_detect_object); 

    HX711_init(GPIO_HX711_DATA,GPIO_HX711_SCLK,eGAIN_128);
    HX711_tare();

    servo_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, SERVO_1_ANG_HOME);
    servo_write(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, SERVO_2_ANG_CLOSE);
    go_home();
    ESP_LOGE(DEBUG_TAG, "bug");

    uint8_t check_count = 0;
    while(1) {
        if(gpio_get_level(DETECT_OBJECT_PIN) == 0){
            check_count++;
            //ESP_LOGE(DEBUG_TAG, "check count: %d", check_count);
            if(check_count > 10){ 
                ESP_LOGE(DEBUG_TAG, "bug 2");
                check_count= 0;
                weight = 100;
                //weight = HX711_get_units(AVG_SAMPLES);
                uint8_t class = classification(weight);
                robot_run_function(class);
                go_home();
                ESP_LOGE(DEBUG_TAG, "end bug 2");
            }
        }else{
            check_count= 0;
        }
        vTaskDelay(50);
    }
}
