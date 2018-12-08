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

uint32_t values[MAX_LINE_SENSORS] = {0, };
uint8_t line[MAX_LINE_SENSORS] = {0, };

int main_cnt = 0;
int bmp_cnt = 0;

void bump_interrupt(uint8_t);

uint32_t lidar_left_val = 0, lidar_forward_val = 0, lidar_right_val = 0;

void dead_reckoning(void) {
    drive_forward_cm(121.92, 0.5);
    drive_turn_right();
    drive_forward_cm(121.92, 0.5);
}

void maze(void) {
    while (1) {
            // Move forward until in front of wall
            while((lidar_forward_val = lidar_forward_read()) < 11000) {
    //            drive_forward_cm(1, 0.1);

                lidar_left_val = lidar_left_read();
                lidar_right_val = lidar_right_read();
                if (lidar_left_val > 14000) {
                    drive_left_speed(0.15);
                    drive_right_speed(0.01);

                    uint32_t i = 0;
                    for (i = 0; i < 30000; i++) {}
                } else if (lidar_right_val > 14000) {
                    drive_right_speed(0.15);
                    drive_left_speed(0.01);

                    uint32_t i = 0;
                    for (i = 0; i < 30000; i++) {}
                } else {
                    drive_left_speed(0.35);
                    drive_right_speed(0.35);
                }
            }
            if (lidar_left_val > lidar_right_val) {
                // Left wall is closer
                // Turn Right
                uint8_t fail_cnt = 0;
                uint32_t max_forward = lidar_forward_read();
                while (fail_cnt < 3) {
                    uint32_t read = lidar_forward_read();
                    if (read > max_forward) {
                        max_forward = read;
                    } else {
                        fail_cnt += 1;
                    }
                    drive_pivot_cw(5);
                }
    //            drive_turn_right();
            } else {
    //            drive_turn_left();
                uint8_t fail_cnt = 0;
                uint32_t max_forward = lidar_forward_read();
                while (fail_cnt < 5) {
                    uint32_t read = lidar_forward_read();
                    if (read > max_forward) {
                        max_forward = read;
                    } else {
                        fail_cnt += 1;
                    }
                    drive_pivot_ccw(5);
                }
            }
        }
}

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

    maze();

    while (1) {
        uint8_t i = 0;
        uint32_t sum = 0;
        for (i = 0; i < 8; i++) {
            uint32_t val = line_sensor_read(test[i]);
            values[i] = val;
            sum += val;
        }

        float periodL = 0, periodR = 0;

        int diff = values[1] + values[2] - values[6] - values[7];
        if (0) {
            if (diff >= -4000 && diff <= 4000) {
                periodL = 0.2f;
                periodR = 0.2f;
            } else if (diff > 0) { // LEFT IS OVER THE LINE
                periodL = 0.2f;
                periodR = 0.01f;
            } else {    // RIGHT IS OVER THE LINE
                periodL = 0.01f;
                periodR = 0.2f;
            }
        } else {
            if (diff >= -4000 && diff <= 4000) {
                periodL = 0.25f;
                periodR = 0.25f;
            } else if (diff < 0) { // LEFT IS OVER THE LINE
                periodL = 0.2f;
                periodR = 0.01f;
            } else {    // RIGHT IS OVER THE LINE
                periodL = 0.01f;
                periodR = 0.2f;
            }
        }

        drive_left_speed(periodL);
        drive_right_speed(periodR);
    }

    while (1) {
        uint8_t i = 0;
        uint32_t sum = 0;
        for (i = 0; i < 8; i++) {
            uint32_t val = line_sensor_read(test[i]);
            values[i] = val;
            sum += val;
        }

        uint32_t avg = sum / MAX_LINE_SENSORS;
        uint8_t over_cnt = 0;
        for (i = 0; i < MAX_LINE_SENSORS; i++) {
            line[i] = (values[i] > avg);
            over_cnt += (values[i] > avg);
        }

        for (i = 0; i < (MAX_LINE_SENSORS ); i++) {
            if (i == 0) {
                if (line[0] && !line[1]) {
                    line[0] = 0;
                }
            } else if (i == (MAX_LINE_SENSORS-1)) {
                if (line[i] && !line[i-1]) {
                    line[i] = 0;
                }
            } else if (line[i]) {
                if (!line[i-1] && !line[i+1]) {
                    line[i] = 0;
                }
            }
        }

        if (over_cnt > (MAX_LINE_SENSORS / 2)) {
            for (i = 0; i < MAX_LINE_SENSORS; i++) {
                line[i] = !line[i];
            }
        }

        uint8_t right_sum = line[0] + line[1] + line[2] + line[3];
        uint8_t left_sum = line[7] + line[6] + line[5] + line[4];

        float right_on = 0, left_on = 0;

        if (left_sum > right_sum) {
            right_on = 0.25;
            left_on = 0.1;
        } else if (right_sum > left_sum) {
            left_on = 0.25;
            right_on = 0.1;
        } else {
            left_on = 0.35;
            right_on = 0.35;
        }

        drive_left_speed(left_on);
        drive_right_speed(right_on);
    }
}

void bump_interrupt(uint8_t read) {
    bmp_cnt++;
}
