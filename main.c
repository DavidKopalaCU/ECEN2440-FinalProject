#include <stdint.h>

#include "msp.h"

#include "../MazeDrivers/LineSensors.h"
#include "../MazeDrivers/MotorEncoder.h"

#include "../inc/SysTick.h"

extern volatile uint8_t line_sensor_count;

/**
 * main.c
 */

int values[MAX_LINE_SENSORS] = {0, };

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

    MotorEncoder_Init();

    while (1) {
        int i;
        for (i = 0; i < line_sensor_get_count(); i++) {
            values[i] = line_sensor_read(line_sensors_get()[i]);
        }
    }
}
