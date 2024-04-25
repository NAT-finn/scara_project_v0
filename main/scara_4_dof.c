#include "scara_4_dof.h"

#define SCARA_TAG "SCARA"

void config_scara_motor(step_motor_t motor1, step_motor_t motor2, step_motor_t motor3, uint8_t end1, uint8_t end2, uint8_t end3){
    scmotor1.motor = motor1;
    scmotor1.end_stop_pin = end1;

    scmotor2.motor = motor2;
    scmotor2.end_stop_pin = end2;   

    scmotor3.motor = motor3;
    scmotor3.end_stop_pin = end3;

    gpio_config_t GPIO_config_end_stop = {};
    GPIO_config_end_stop.pin_bit_mask = ((uint64_t)1 << end1) | ((uint64_t)1 << end2) | ((uint64_t)1 << end3);
    GPIO_config_end_stop.mode = GPIO_MODE_INPUT;
    GPIO_config_end_stop.pull_up_en = 0;
    GPIO_config_end_stop.pull_down_en = 0;
    GPIO_config_end_stop.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_config_end_stop); 

    // old_point.phi_1 = old_point.phi_2 = old_point.z = 0;
    // new_point.phi_1 = new_point.phi_2 = new_point.z = 0;
}

void go_home(){
    while (scmotor1.end_stop_pin == 1)
    {
        set_motor_dir(scmotor1.motor, BACK_MOTOR_1);
        set_motor_high(scmotor1.motor);
        vTaskDelay(1);
        set_motor_low(scmotor1.motor);
        vTaskDelay(1);
    }
    while (scmotor2.end_stop_pin == 1)
    {
        set_motor_dir(scmotor2.motor, BACK_MOTOR_2);
        set_motor_high(scmotor2.motor);
        vTaskDelay(1);
        set_motor_low(scmotor2.motor);
        vTaskDelay(1);
    }
    while (scmotor3.end_stop_pin == 1)
    {
        set_motor_dir(scmotor3.motor, BACK_MOTOR_3);
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor);
        vTaskDelay(1);
    }   
}

void motor3_go_home(){
        while (scmotor3.end_stop_pin == 1)
    {
        set_motor_dir(scmotor3.motor, BACK_MOTOR_3);
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor);
        vTaskDelay(1);
    } 
}

void run_point_0_0(int *old_motor1_loc, int *old_motor2_loc, int *new_motor1_loc, int *new_motor2_loc){
    *old_motor1_loc = *new_motor1_loc;
    *old_motor2_loc = *new_motor2_loc;
    *new_motor1_loc = POINT_0_0_MOR1;
    *new_motor2_loc = POINT_0_0_MOR2;
    int mor1 = *new_motor1_loc - *old_motor1_loc;
    int mor2 = *new_motor2_loc - *old_motor2_loc;
    set_motor_dir(scmotor1.motor, !BACK_MOTOR_1);
    set_motor_dir(scmotor2.motor, !BACK_MOTOR_2);
    if(mor1 < 0){
        set_motor_dir(scmotor1.motor, BACK_MOTOR_1);
        mor1 = -mor1;
    }
    if(mor2 < 0){
        set_motor_dir(scmotor2.motor, BACK_MOTOR_2);
        mor2 = -mor2;
    }
    run_motor(mor1, mor2);
    motor3_go_home();
}

void run_point_0_1(int *old_motor1_loc, int *old_motor2_loc, int *new_motor1_loc, int *new_motor2_loc){
    *old_motor1_loc = *new_motor1_loc;
    *old_motor2_loc = *new_motor2_loc;
    *new_motor1_loc = POINT_0_1_MOR1;
    *new_motor2_loc = POINT_0_1_MOR2;
    int mor1 = *new_motor1_loc - *old_motor1_loc;
    int mor2 = *new_motor2_loc - *old_motor2_loc;
    set_motor_dir(scmotor1.motor, !BACK_MOTOR_1);
    set_motor_dir(scmotor2.motor, !BACK_MOTOR_2);
    if(mor1 < 0){
        set_motor_dir(scmotor1.motor, BACK_MOTOR_1);
        mor1 = -mor1;
    }
    if(mor2 < 0){
        set_motor_dir(scmotor2.motor, BACK_MOTOR_2);
        mor2 = -mor2;
    }
    run_motor(mor1, mor2);
    motor3_go_home();
}

void run_point_1(int *old_motor1_loc, int *old_motor2_loc, int *new_motor1_loc, int *new_motor2_loc){
    *old_motor1_loc = *new_motor1_loc;
    *old_motor2_loc = *new_motor2_loc;
    *new_motor1_loc = POINT_1_MOR1;
    *new_motor2_loc = POINT_1_MOR2;
    int mor1 = *new_motor1_loc - *old_motor1_loc;
    int mor2 = *new_motor2_loc - *old_motor2_loc;
    set_motor_dir(scmotor1.motor, !BACK_MOTOR_1);
    set_motor_dir(scmotor2.motor, !BACK_MOTOR_2);
    if(mor1 < 0){
        set_motor_dir(scmotor1.motor, BACK_MOTOR_1);
        mor1 = -mor1;
    }
    if(mor2 < 0){
        set_motor_dir(scmotor2.motor, BACK_MOTOR_2);
        mor2 = -mor2;
    }
    run_motor(mor1, mor2);
    motor3_go_home();
}

void run_point_1_1(){
    set_motor_dir(scmotor3.motor, !BACK_MOTOR_3);
    for(int i = 0; i < POINT_1_1_MOR3; i++){
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor); 
        vTaskDelay(1);
    }
}

void run_point_1_2(){
    set_motor_dir(scmotor3.motor, !BACK_MOTOR_3);
    for(int i = 0; i < POINT_1_2_MOR3; i++){
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor); 
        vTaskDelay(1);
    }
}

void run_point_1_3(){
    set_motor_dir(scmotor3.motor, !BACK_MOTOR_3);
    for(int i = 0; i < POINT_1_3_MOR3; i++){
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor); 
        vTaskDelay(1);
    }
}

void run_motor(int mor1, int mor2){
    while(mor1 != 00 && mor2 != 0){
        if(mor1 !=0) { set_motor_high(scmotor1.motor); }
        if(mor2 !=0) { set_motor_high(scmotor2.motor); }
        vTaskDelay(1);
        if(mor1 !=0) { 
            set_motor_low(scmotor1.motor); 
            mor1--;
        }
        if(mor2 !=0) { 
            set_motor_low(scmotor2.motor); 
            mor2--;
        }
        vTaskDelay(1);
    }
}
    
