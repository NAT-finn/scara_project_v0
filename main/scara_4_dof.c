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

point_t run_locate(int x, int y, int z){
    int phi_1_d, phi_2_d;
    point_t point;
    //ESP_LOGE(SCARA_TAG, "bug");
    //double test = ((double)((x*x+y*y) - (L1*L1+L2*L2)))/(2*L1*L2);
    double phi_temp = acos(((double)((x*x+y*y) - (L1*L1+L2*L2)))/(2*L1*L2));
    //ESP_LOGE(SCARA_TAG, "phi_temp: %f", phi_temp);
    phi_2_d = (int)((phi_temp*1800000)/PI); 
    
    double min_value = 1000000;
    for(int i = -900000; i <= 900000; i +=  scmotor2.motor.full_step){
        //ESP_LOGE(SCARA_TAG, "bug1: %d", i);
        double ix = (((double)i)/1800000)*PI;
        double delta_1 = L1*cos(ix) + L2*cos(phi_temp + ix) - x;
        double delta_2 = L1*sin(ix) + L2*sin(phi_temp + ix) - y;
        if(fabs(delta_1) + fabs(delta_2) < min_value){
            min_value = fabs(delta_1) + fabs(delta_2);
            phi_1_d = i;
        }
    }
    ESP_LOGE(SCARA_TAG, "phi_1: %d | phi_2: %d | z: %d", phi_1_d, phi_2_d, z);
    point.phi_1 = phi_1_d;
    point.phi_2 = phi_2_d;
    point.z = z;

    return point;
}

void go_home(){
    while (gpio_get_level(scmotor2.end_stop_pin) == 1)
    {
        set_motor_dir(scmotor2.motor, BACK_MOTOR_2);
        set_motor_high(scmotor2.motor);
        vTaskDelay(1);
        set_motor_low(scmotor2.motor);
        vTaskDelay(1);
    }
    while (gpio_get_level(scmotor3.end_stop_pin) == 1)
    {
        set_motor_dir(scmotor3.motor, BACK_MOTOR_3);
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor);
        vTaskDelay(1);
    }   
    while (gpio_get_level(scmotor1.end_stop_pin) == 1)
    {
        set_motor_dir(scmotor1.motor, BACK_MOTOR_1);
        set_motor_high(scmotor1.motor);
        vTaskDelay(1);
        set_motor_low(scmotor1.motor);
        vTaskDelay(1);
    }
}

void motor3_go_home(){
        while (gpio_get_level(scmotor3.end_stop_pin) == 1)
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
    ESP_LOGE(SCARA_TAG, "run motor %d | %d", mor1, mor2);
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
        ESP_LOGE(SCARA_TAG, "switch");
        mor1 = -mor1;
    }
    if(mor2 < 0){
        set_motor_dir(scmotor2.motor, BACK_MOTOR_2);
        mor2 = -mor2;
    }
    ESP_LOGE(SCARA_TAG, "run motor %d | %d", mor1, mor2);
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
    ESP_LOGE(SCARA_TAG, "run motor %d | %d", mor1, mor2);
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
    ESP_LOGE(SCARA_TAG, "pick motor 3 %d", POINT_1_1_MOR3);
}

void run_point_1_2(){
    set_motor_dir(scmotor3.motor, !BACK_MOTOR_3);
    for(int i = 0; i < POINT_1_2_MOR3; i++){
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor); 
        vTaskDelay(1);
    }
    ESP_LOGE(SCARA_TAG, "pick motor 3 %d", POINT_1_2_MOR3);
}

void run_point_1_3(){
    set_motor_dir(scmotor3.motor, !BACK_MOTOR_3);
    for(int i = 0; i < POINT_1_3_MOR3; i++){
        set_motor_high(scmotor3.motor);
        vTaskDelay(1);
        set_motor_low(scmotor3.motor); 
        vTaskDelay(1);
    }
    ESP_LOGE(SCARA_TAG, "pick motor 3 %d", POINT_1_3_MOR3);
}

void run_motor(int mor1, int mor2){
    while(mor1 != 00 || mor2 != 0){
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
    
