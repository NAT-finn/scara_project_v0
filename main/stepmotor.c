#include "stepmotor.h"

#define MOTOR_TAG "STEP_MOTOR"

void init_motor(step_motor_t *motor, uint8_t step_pin, uint8_t dir_pin, uint8_t MS1, uint8_t MS2, uint8_t MS3)
{
    motor->step_pin = step_pin;
    motor->dir_pin = dir_pin;
    gpio_config_t GPIO_config_motor = {};
    //ESP_LOGE(MOTOR_TAG, "pin: %d", motor->dir_pin);
    GPIO_config_motor.pin_bit_mask = ((uint64_t)1 << step_pin) | ((uint64_t)1 << dir_pin) ;
    GPIO_config_motor.mode = GPIO_MODE_OUTPUT;
    GPIO_config_motor.pull_up_en = 0;
    GPIO_config_motor.pull_down_en = 0;
    GPIO_config_motor.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_config_motor);

    if(MS1 == 0 && MS2 == 0 && MS3 == 0){
        motor->full_step = STEP_FULL_BASE;  // 1.8 Degrees
    }else if (MS1 == 1 && MS2 == 0 && MS3 == 0){
        motor->full_step = STEP_FULL_BASE / 2;  // 0.9 Degrees
    }else if (MS1 == 0 && MS2 == 1 && MS3 == 0){
        motor->full_step = STEP_FULL_BASE / 4;  // 0.45 Degrees
    }else if (MS1 == 1 && MS2 == 1 && MS3 == 0){
        motor->full_step = STEP_FULL_BASE / 8;  // 0.225 Degrees
    }else if (MS1 == 1 && MS2 == 1 && MS3 == 1){
        motor->full_step = STEP_FULL_BASE / 16;  // 0.1125 Degrees
    }
}


void set_motor_high(step_motor_t motor){
    gpio_set_level(motor.step_pin, 1);
}

void set_motor_low(step_motor_t motor){
    gpio_set_level(motor.step_pin, 0);
}

void set_motor_dir(step_motor_t motor, uint8_t dir){
    gpio_set_level(motor.dir_pin, dir);
}

void run_motor_test(step_motor_t motor, uint16_t rev, uint16_t delay){
    set_motor_dir(motor, 1);
    ESP_LOGE(MOTOR_TAG, "loop1");
    for(int i = 0; i < rev; i++){
        set_motor_high(motor);
        vTaskDelay(delay);
        set_motor_low(motor);
        vTaskDelay(delay);
    }
    ESP_LOGE(MOTOR_TAG, "loop2");
    vTaskDelay(1000);
    ESP_LOGE(MOTOR_TAG, "loop3");
    set_motor_dir(motor, 0);
    for(int i = 0; i < rev; i++){
        set_motor_high(motor);
        vTaskDelay(delay);
        set_motor_low(motor);
        vTaskDelay(delay);
    }
    ESP_LOGE(MOTOR_TAG, "loop4");
    vTaskDelay(1000);
}
