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

    float seg_length = 110;
    uint32_t abc = 0;

    drive_forward_cm(seg_length, 0.35);
//    drive_turn_right();
    for (abc = 0; abc < 100000; abc++) {}
    drive_pivot_cw(88);
    drive_forward_cm(seg_length, 0.35);

    for (abc = 0; abc < 100000; abc++) {}
    drive_pivot_ccw(180);

    drive_forward_cm(seg_length, 0.35);
//    drive_turn_left();
    for (abc = 0; abc < 100000; abc++) {}
    drive_pivot_ccw(90);
    drive_forward_cm(2 * seg_length, 0.35);

}

void maze(void) {
    while (1) {
            // Move forward until in front of wall
            while((lidar_forward_val = lidar_forward_read()) < 10000) {
    //            drive_forward_cm(1, 0.1);

                lidar_left_val = lidar_left_read();
                lidar_right_val = lidar_right_read();
                if (lidar_left_val > 14000) {
//                    drive_left_speed(0.15);
//                    drive_right_speed(0.06);
//
//                    uint32_t i = 0;
//                    for (i = 0; i < 30000; i++) {}
                    uint8_t left_fail_cnt = 0;
                    uint32_t left_max = 0;
                    while (left_fail_cnt < 1) {
                        uint32_t read = lidar_left_read();
                        if (read > left_max) {
                            left_max = read;
                        } else {
                            left_fail_cnt++;
                        }
                        drive_left_speed(0.25);
                        drive_right_speed(0.05);
                    }
                } else if (lidar_right_val > 14000) {
//                    drive_right_speed(0.15);
//                    drive_left_speed(0.06);
//
//                    uint32_t i = 0;
//                    for (i = 0; i < 30000; i++) {}
                    uint8_t right_fail_cnt = 0;
                    uint32_t right_max = 0;
                    while (right_fail_cnt < 1) {
                        uint32_t read = lidar_right_read();
                        if (read > right_max) {
                            right_max = read;
                        } else {
                            right_fail_cnt++;
                        }
                        drive_left_speed(0.05);
                        drive_right_speed(0.25);
                    }
                } else {
                    drive_left_speed(0.25);
                    drive_right_speed(0.25);
                }
            }
            if (lidar_left_val > lidar_right_val) {
                // Left wall is closer
                // Turn Right
                uint8_t fail_cnt = 0;
                uint32_t max_forward = lidar_forward_read();
                while (fail_cnt < 5) {
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

line_sensor_t **test = 0;

void avoid_object(void) {
//    uint32_t i = 0;
//    for(i = 0; i < 100000; i++) {}

    drive_backward_cm(5, 0.15);
    drive_turn_right();
    drive_forward_cm(30, 0.3);
    drive_turn_left();
    drive_forward_cm(55, 0.3);
    drive_turn_left();
//    drive_forward_cm(30, 0.3);
//    drive_turn_right();

    uint8_t i = 0;
    uint32_t line_status_cnt = 0;
    uint8_t found_line = 0;
    uint8_t line_status[8] = {0, };
    while (!found_line) {
        line_status_cnt = 0;
        drive_forward_cm(1, 0.3);
        for (i = 0; i < 8; i++) {
            uint32_t val = line_sensor_read(test[i]);
            if (val > 10000) {
                line_status[i] = 1;
            }
            line_status_cnt += line_status[i];
        }
        if (line_status_cnt >= 4) {
            found_line = 1;
        }
    }
    drive_turn_right();
}


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    SysTick_Init();

    line_sensor_t line_0, line_1, line_2, line_3, line_4, line_5, line_6, line_7;
    test = line_sensors_get();
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

    __enable_interrupt();

//    drive_forward_cm(1000, 0.5);
    uint32_t abc = 0;
    for (abc = 0; abc < 100000; abc++) {}
//    dead_reckoning();
    maze();
    while(1) {}

    uint8_t bmp_trig = 0;

    line_sensor_auto_cal();

    while (1) {
        if ((bmp_trig = bump_is_triggered())) {
            bmp_cnt++;
            avoid_object();
            bump_clear_trigger();
        }

        uint8_t i = 0;
        uint32_t sum = 0;
        for (i = 0; i < 8; i++) {
            uint32_t val = line_sensor_read(test[i]);
            values[i] = val;
            sum += val;
        }

        float periodL = 0, periodR = 0;

        int diff = values[1] + values[2] - values[6] - values[7];
        if (1) {
            if (diff >= -1000 && diff <= 1000) {
                periodL = 0.3f;
                periodR = 0.3f;

                if (lidar_forward_read() > 14000) {
                    avoid_object();
                }
            } else if (diff > 0) { // LEFT IS OVER THE LINE
                periodL = 0.25f;
                periodR = 0.02f;
            } else {    // RIGHT IS OVER THE LINE
                periodL = 0.02f;
                periodR = 0.25f;
            }
        } else {
            if (diff >= -8000 && diff <= 8000) {
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

    drive_backward_cm(5, 0.3);
    drive_turn_right();
    drive_forward_cm(40, 0.3);
    drive_turn_left();
    drive_forward_cm(5, 0.3);
    drive_turn_right();
}
