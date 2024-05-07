#ifndef scara_4_dof_h
#define scara_4_dof_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

#include "servo_by_ledc.h"
#include "stepmotor.h"

#define L1         100
#define L2         120
#define PI         3.14159265358979323846

#define BACK_MOTOR_1   0
#define BACK_MOTOR_2   1
#define BACK_MOTOR_3   0

#define POINT_0_0_MOR1 1350
#define POINT_0_0_MOR2 1100

#define POINT_0_1_MOR1 500
#define POINT_0_1_MOR2 1100

#define POINT_1_MOR1 1200
#define POINT_1_MOR2 850

#define POINT_1_1_MOR3 300
#define POINT_1_2_MOR3 900
#define POINT_1_3_MOR3 1500


typedef struct {
    step_motor_t motor; 
    uint8_t end_stop_pin;  
} scara_motor_t;

typedef struct {
    int z;
    int phi_1;
    int phi_2;
} point_t;

extern scara_motor_t scmotor1, scmotor2, scmotor3;
// point_t old_point, new_point;

void config_scara_motor(step_motor_t motor1, step_motor_t motor2, step_motor_t motor3, uint8_t end1, uint8_t end2, uint8_t end3);

//point_t run_locate(int x, int y, int z);

void go_home();

void motor3_go_home();

void run_motor(int mor1, int mor2);

void run_point_0_0(int *old_motor1_loc, int *old_motor2_loc, int *new_motor1_loc, int *new_motor2_loc);

void run_point_0_1(int *old_motor1_loc, int *old_motor2_loc, int *new_motor1_loc, int *new_motor2_loc);

void run_point_1(int *old_motor1_loc, int *old_motor2_loc, int *new_motor1_loc, int *new_motor2_loc);

void run_point_1_1();
void run_point_1_2();
void run_point_1_3();



#endif