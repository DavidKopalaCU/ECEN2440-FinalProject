#include <stdint.h>

#include "msp.h"

#include "../MazeDrivers/LineSensors.h"
#include "../MazeDrivers/MotorEncoder.h"
#include "../MazeDrivers/LidarSensors.h"
#include "../MazeDrivers/BumpSwitch.h"
#include "../MazeDrivers/DriveSystem.h"

#include "../inc/SysTick.h"
#include "../inc/PWM.h"

extern volatile uint8_t line_sensor_count;

/**
 * main.c
 */

int values[MAX_LINE_SENSORS] = {0, };

int main_cnt = 0;
int bmp_cnt = 0;

void bump_interrupt(uint8_t);

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    SysTick_Init();

    line_sensor_t line_0, line_1, line_2, line_3, line_4, line_5, line_6, line_7;
    line_sensor_t **test = line_sensors_get();
    test[0] = &line_0;
    test[1] = &line_1;
    test[2] = &line_2;
    test[3] = &line_3;
    test[4] = &line_4;
    test[5] = &line_5;
    test[6] = &line_6;
    test[7] = &line_7;

//    line_sensors = {line_0, line_1, line_2, line_3, line_4, line_5, line_6, line_7};

    Init_LineSensors();
    line_sensors_add_port(P7);

    ir_led_t ir_led;
    line_sensor_config_ir_led(&ir_led, P5, BIT3);

    DriveSystem_Init(0xFFF);

    LidarSensors_Init();

    BumpSwitches_Init(bump_interrupt);

    drive_forward_cm(0, 0.5);

//    while (1) {
//        main_cnt += 1;
//
////        int i;
////        for (i = 0; i < line_sensor_get_count(); i++) {
////            values[i] = line_sensor_read(line_sensors_get()[i]);
////        }
//
//        if ((left_encoder_p->count - right_encoder_p->count) > 50) {
//            left_on -= 0.01;
//            right_on += 0.01;
//        } else if ((right_encoder_p->count - left_encoder_p->count) > 50) {
//            right_on -= 0.01;
//            left_on += 0.01;
//        }
//
//        if (left_on >= 0.5) {
//            left_on = 0.5;
//        }
//        if (left_on <= 0) {
//            left_on = 0;
//        }
//        if (right_on >= 0.5) {
//            right_on = 0.5;
//        }
//        if (right_on <= 0) {
//            right_on = 0;
//        }
//
//        PWM_Duty1(left_on * motor_period);
//        PWM_Duty2(right_on * motor_period);
//
//    }
}

void bump_interrupt(uint8_t read) {
    bmp_cnt++;
}
