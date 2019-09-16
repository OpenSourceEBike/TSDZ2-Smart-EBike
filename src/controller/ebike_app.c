/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho and EndlessCadence and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#include "ebike_app.h"
#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "main.h"
#include "interrupts.h"
#include "adc.h"
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "eeprom.h"
#include "lights.h"
#include "common.h"

volatile struct_configuration_variables m_configuration_variables;

// system
static uint8_t    ui8_riding_mode = OFF_MODE;
static uint8_t    ui8_riding_mode_parameter = 0;
static uint8_t    ui8_system_state = NO_ERROR;
static uint8_t    ui8_motor_enabled = 1;
static uint8_t    ui8_assist_without_pedal_rotation_threshold = 0;
static uint8_t    ui8_lights_configuration = 10;
static uint8_t    ui8_lights_state = 0;


// power control
static uint8_t    ui8_battery_current_max = DEFAULT_VALUE_BATTERY_CURRENT_MAX;
static uint16_t   ui16_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
static uint16_t   ui16_duty_cycle_ramp_up_inverse_step_default = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
static uint16_t   ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
static uint16_t   ui16_battery_voltage_filtered_x1000 = 0;
static uint8_t    ui8_battery_current_filtered_x10 = 0;
static uint8_t    ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
static uint8_t    ui8_adc_battery_current_target = 0;
static uint8_t    ui8_duty_cycle_target = 0;


// brakes
static uint8_t ui8_brakes_engaged = 0;


// cadence sensor
volatile uint8_t ui8_cadence_sensor_mode = STANDARD_MODE;
volatile uint16_t ui16_cadence_sensor_ticks_counter_min_speed_adjusted = CADENCE_SENSOR_TICKS_COUNTER_MIN;
static uint16_t ui16_cadence_sensor_pulse_high_percentage_x10 = CADENCE_SENSOR_PULSE_PERCENTAGE_X10_DEFAULT;
static uint8_t ui8_pedal_cadence_RPM = 0;


// torque sensor
volatile uint16_t ui16_adc_pedal_torque = 0;
static uint16_t   ui16_adc_pedal_torque_delta = 0;
static uint16_t   ui16_human_power_x10 = 0;
static uint16_t   ui16_pedal_torque_x100 = 0;


// wheel speed sensor
static uint16_t   ui16_wheel_speed_x10 = 0;


// throttle control
volatile uint8_t  ui8_adc_throttle = 0;


// motor temperature control
static uint16_t ui16_motor_temperature_filtered_x10 = 0;
static uint8_t ui8_motor_temperature_max_value_to_limit = 0;
static uint8_t ui8_motor_temperature_min_value_to_limit = 0;
static uint8_t ui8_temperature_current_limiting_value = 0;


// eMTB assist
#define eMTB_POWER_FUNCTION_ARRAY_SIZE      241

static const uint8_t ui8_eMTB_power_function_160[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 22, 23, 23, 23, 24, 24, 24, 24, 25, 25, 25, 26, 26, 26, 27, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 44, 44, 44, 45, 45, 45, 46, 46, 47, 47, 47, 48, 48, 48, 49, 49, 50, 50, 50, 51, 51, 52, 52, 52, 53, 53, 54, 54, 54, 55, 55, 56, 56, 56, 57, 57, 58, 58, 58, 59, 59, 60, 60, 61, 61, 61, 62, 62, 63, 63, 63, 64, 64 };
static const uint8_t ui8_eMTB_power_function_165[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 24, 25, 25, 25, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 30, 30, 30, 31, 31, 32, 32, 32, 33, 33, 34, 34, 34, 35, 35, 36, 36, 36, 37, 37, 38, 38, 39, 39, 39, 40, 40, 41, 41, 42, 42, 42, 43, 43, 44, 44, 45, 45, 46, 46, 47, 47, 47, 48, 48, 49, 49, 50, 50, 51, 51, 52, 52, 53, 53, 54, 54, 55, 55, 56, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 61, 62, 62, 63, 63, 64, 64, 65, 65, 66, 66, 67, 67, 68, 68, 69, 69, 70, 71, 71, 72, 72, 73, 73, 74, 74, 75, 75, 76, 77, 77, 78, 78, 79, 79, 80, 81, 81, 82, 82, 83, 83, 84, 85 };
static const uint8_t ui8_eMTB_power_function_170[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 21, 21, 21, 22, 22, 23, 23, 23, 24, 24, 25, 25, 26, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38, 39, 39, 40, 40, 41, 41, 42, 42, 43, 43, 44, 45, 45, 46, 46, 47, 47, 48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 53, 54, 55, 55, 56, 56, 57, 58, 58, 59, 59, 60, 61, 61, 62, 63, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 77, 77, 78, 79, 80, 80, 81, 82, 82, 83, 84, 84, 85, 86, 87, 87, 88, 89, 89, 90, 91, 92, 92, 93, 94, 94, 95, 96, 97, 97, 98, 99, 100, 100, 101, 102, 103, 103, 104, 105, 106, 107, 107, 108, 109, 110, 110, 111 };
static const uint8_t ui8_eMTB_power_function_175[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 36, 36, 37, 37, 38, 39, 39, 40, 40, 41, 42, 42, 43, 44, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 53, 54, 55, 56, 56, 57, 58, 58, 59, 60, 61, 61, 62, 63, 64, 64, 65, 66, 67, 67, 68, 69, 70, 70, 71, 72, 73, 74, 74, 75, 76, 77, 78, 78, 79, 80, 81, 82, 83, 83, 84, 85, 86, 87, 88, 88, 89, 90, 91, 92, 93, 94, 95, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146 };
static const uint8_t ui8_eMTB_power_function_180[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 25, 26, 27, 27, 28, 28, 29, 30, 30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 38, 38, 39, 40, 41, 41, 42, 43, 43, 44, 45, 46, 46, 47, 48, 49, 50, 50, 51, 52, 53, 54, 54, 55, 56, 57, 58, 59, 59, 60, 61, 62, 63, 64, 65, 66, 67, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 105, 106, 107, 108, 109, 110, 111, 112, 114, 115, 116, 117, 118, 119, 120, 122, 123, 124, 125, 126, 128, 129, 130, 131, 132, 134, 135, 136, 137, 139, 140, 141, 142, 144, 145, 146, 147, 149, 150, 151, 153, 154, 155, 157, 158, 159, 161, 162, 163, 165, 166, 167, 169, 170, 171, 173, 174, 176, 177, 178, 180, 181, 182, 184, 185, 187, 188, 190, 191, 192 };
static const uint8_t ui8_eMTB_power_function_185[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 74, 75, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 87, 89, 90, 91, 92, 93, 95, 96, 97, 98, 100, 101, 102, 104, 105, 106, 107, 109, 110, 111, 113, 114, 115, 117, 118, 120, 121, 122, 124, 125, 127, 128, 129, 131, 132, 134, 135, 137, 138, 140, 141, 143, 144, 146, 147, 149, 150, 152, 153, 155, 156, 158, 160, 161, 163, 164, 166, 168, 169, 171, 172, 174, 176, 177, 179, 181, 182, 184, 186, 187, 189, 191, 193, 194, 196, 198, 199, 201, 203, 205, 207, 208, 210, 212, 214, 216, 217, 219, 221, 223, 225, 227, 228, 230, 232, 234, 236, 238, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_190[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 16, 16, 17, 18, 18, 19, 20, 20, 21, 22, 22, 23, 24, 25, 25, 26, 27, 28, 29, 29, 30, 31, 32, 33, 34, 35, 36, 37, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 51, 52, 53, 54, 55, 56, 57, 58, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72, 73, 74, 76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 91, 92, 94, 95, 96, 98, 99, 101, 102, 104, 105, 107, 108, 110, 112, 113, 115, 116, 118, 120, 121, 123, 124, 126, 128, 130, 131, 133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 150, 152, 154, 156, 158, 160, 162, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187, 189, 191, 193, 195, 197, 199, 201, 203, 205, 207, 209, 211, 214, 216, 218, 220, 222, 224, 227, 229, 231, 233, 235, 238, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_195[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 15, 15, 16, 17, 17, 18, 19, 20, 21, 21, 22, 23, 24, 25, 26, 27, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 39, 40, 41, 42, 43, 44, 45, 47, 48, 49, 50, 51, 53, 54, 55, 57, 58, 59, 61, 62, 63, 65, 66, 68, 69, 70, 72, 73, 75, 76, 78, 79, 81, 83, 84, 86, 87, 89, 91, 92, 94, 96, 97, 99, 101, 103, 104, 106, 108, 110, 112, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 132, 134, 136, 139, 141, 143, 145, 147, 149, 151, 153, 155, 157, 160, 162, 164, 166, 168, 171, 173, 175, 177, 180, 182, 184, 187, 189, 191, 194, 196, 199, 201, 203, 206, 208, 211, 213, 216, 218, 221, 224, 226, 229, 231, 234, 237, 239, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_200[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 13, 14, 14, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 38, 40, 41, 42, 44, 45, 46, 48, 49, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 66, 67, 69, 71, 72, 74, 76, 77, 79, 81, 83, 85, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 117, 119, 121, 123, 125, 128, 130, 132, 135, 137, 139, 142, 144, 146, 149, 151, 154, 156, 159, 161, 164, 166, 169, 172, 174, 177, 180, 182, 185, 188, 190, 193, 196, 199, 202, 204, 207, 210, 213, 216, 219, 222, 225, 228, 231, 234, 237, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_205[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20, 21, 22, 23, 24, 26, 27, 28, 29, 30, 32, 33, 34, 36, 37, 38, 40, 41, 43, 44, 46, 47, 49, 50, 52, 54, 55, 57, 59, 61, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 95, 97, 99, 101, 104, 106, 108, 111, 113, 116, 118, 121, 123, 126, 128, 131, 134, 136, 139, 142, 145, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 189, 192, 196, 199, 202, 205, 209, 212, 216, 219, 222, 226, 229, 233, 236, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_210[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 12, 13, 14, 14, 15, 16, 17, 19, 20, 21, 22, 23, 24, 26, 27, 28, 30, 31, 32, 34, 35, 37, 39, 40, 42, 43, 45, 47, 49, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 71, 73, 75, 77, 80, 82, 84, 87, 89, 92, 94, 97, 99, 102, 104, 107, 110, 113, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 145, 149, 152, 155, 158, 162, 165, 169, 172, 176, 179, 183, 186, 190, 194, 197, 201, 205, 209, 213, 216, 220, 224, 228, 232, 237, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_215[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22, 24, 25, 26, 28, 29, 31, 33, 34, 36, 38, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 60, 62, 64, 67, 69, 71, 74, 76, 79, 82, 84, 87, 90, 93, 96, 98, 101, 104, 107, 111, 114, 117, 120, 123, 127, 130, 134, 137, 141, 144, 148, 152, 155, 159, 163, 167, 171, 175, 179, 183, 187, 191, 195, 200, 204, 208, 213, 217, 222, 226, 231, 235, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_220[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 22, 23, 25, 27, 28, 30, 32, 33, 35, 37, 39, 41, 43, 46, 48, 50, 52, 55, 57, 60, 62, 65, 67, 70, 73, 76, 79, 82, 85, 88, 91, 94, 97, 101, 104, 108, 111, 115, 118, 122, 126, 130, 133, 137, 141, 145, 150, 154, 158, 162, 167, 171, 176, 180, 185, 190, 194, 199, 204, 209, 214, 219, 224, 230, 235, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_225[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 12, 13, 14, 15, 17, 18, 20, 21, 23, 24, 26, 28, 30, 32, 34, 36, 38, 40, 43, 45, 47, 50, 52, 55, 58, 61, 64, 66, 70, 73, 76, 79, 82, 86, 89, 93, 96, 100, 104, 108, 112, 116, 120, 124, 128, 133, 137, 142, 146, 151, 156, 161, 166, 171, 176, 181, 186, 191, 197, 202, 208, 214, 219, 225, 231, 237, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_230[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 18, 20, 21, 23, 25, 27, 29, 31, 33, 36, 38, 40, 43, 46, 48, 51, 54, 57, 60, 63, 67, 70, 74, 77, 81, 85, 88, 92, 96, 101, 105, 109, 114, 118, 123, 128, 133, 138, 143, 148, 153, 158, 164, 170, 175, 181, 187, 193, 199, 205, 212, 218, 225, 231, 238, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_235[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 19, 21, 23, 25, 27, 30, 32, 34, 37, 40, 43, 45, 48, 52, 55, 58, 62, 65, 69, 73, 77, 81, 85, 89, 94, 98, 103, 108, 113, 118, 123, 128, 134, 139, 145, 151, 157, 163, 169, 176, 182, 189, 196, 202, 210, 217, 224, 232, 239, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_240[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 17, 19, 21, 23, 25, 27, 30, 32, 35, 38, 41, 44, 47, 51, 54, 58, 62, 66, 70, 74, 79, 83, 88, 93, 98, 103, 108, 114, 120, 125, 131, 137, 144, 150, 157, 164, 171, 178, 185, 193, 200, 208, 216, 224, 233, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_245[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 8, 9, 10, 12, 14, 15, 17, 19, 22, 24, 27, 29, 32, 35, 38, 42, 45, 49, 53, 57, 61, 65, 70, 74, 79, 84, 89, 95, 100, 106, 112, 119, 125, 132, 138, 145, 153, 160, 168, 176, 184, 192, 200, 209, 218, 227, 237, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_250[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 6, 7, 9, 10, 12, 14, 16, 18, 20, 23, 25, 28, 31, 34, 38, 41, 45, 49, 54, 58, 63, 67, 72, 78, 83, 89, 95, 101, 108, 114, 121, 128, 136, 144, 151, 160, 168, 177, 186, 195, 204, 214, 224, 235, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };
static const uint8_t ui8_eMTB_power_function_255[eMTB_POWER_FUNCTION_ARRAY_SIZE] = { 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 21, 24, 26, 30, 33, 37, 41, 45, 49, 54, 58, 64, 69, 75, 80, 87, 93, 100, 107, 114, 122, 130, 138, 146, 155, 164, 174, 184, 194, 204, 215, 226, 238, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240 };


// cruise
static uint8_t ui8_cruise_PID_initialize = 1;


// boost
uint8_t   ui8_startup_boost_enable = 0;
uint8_t   ui8_startup_boost_fade_enable = 0;
uint8_t   ui8_m_startup_boost_state_machine = 0;
uint8_t   ui8_startup_boost_no_torque = 0;
uint8_t   ui8_startup_boost_timer = 0;
uint8_t   ui8_startup_boost_fade_steps = 0;
uint16_t  ui16_startup_boost_fade_variable_x256;
uint16_t  ui16_startup_boost_fade_variable_step_amount_x256;
static void     boost_run_statemachine (void);
static uint8_t  boost(uint8_t ui8_max_current_boost_state);
static void     apply_boost_fade_out();
uint8_t ui8_boost_enabled_and_applied = 0;
static void apply_boost();


// UART
#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   7   // change this value depending on how many data bytes there are to receive ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_NUMBER_DATA_BYTES_TO_SEND      26  // change this value depending on how many data bytes there are to send ( Package = one start byte + data bytes + two bytes 16 bit CRC )

volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 3];
volatile uint8_t ui8_i;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
static uint16_t  ui16_crc_rx;
static uint16_t  ui16_crc_tx;
volatile uint8_t ui8_message_ID = 0;

static void communications_controller (void);
static void uart_receive_package (void);
static void uart_send_package (void);


// system functions
static void get_battery_voltage_filtered(void);
static void get_battery_current_filtered(void);
static void get_pedal_torque(void);
static void calc_wheel_speed(void);
static void calc_cadence(void);

static void ebike_control_lights(void);
static void ebike_control_motor(void);
static void check_system(void);
static void check_brakes(void);

static void apply_power_assist();
static void apply_torque_assist();
static void apply_cadence_assist();
static void apply_emtb_assist();
static void apply_walk_assist();
static void apply_cruise();
static void apply_cadence_sensor_calibration();
static void apply_throttle();
static void apply_temperature_limiting();
static void apply_speed_limit();



void ebike_app_controller (void)
{ 
  calc_wheel_speed();               // calculate the wheel speed
  calc_cadence();                   // calculate the cadence and set limits from wheel speed
  
  get_battery_voltage_filtered();   // get filtered voltage from FOC calculations
  get_battery_current_filtered();   // get filtered current from FOC calculations
  get_pedal_torque();               // get pedal torque
  
  check_system();                   // check if there are any errors for motor control 
  check_brakes();                   // check if brakes are enabled for motor control
  
  communications_controller();      // get data to use for motor control and also send new data
  ebike_control_lights();           // use received data and sensor input to control external lights
  ebike_control_motor();            // use received data and sensor input to control motor
  
  /*------------------------------------------------------------------------
  
    NOTE: regarding function call order
    
    Do not change order of functions if not absolutely sure it will 
    not cause any undesirable consequences.
    
  ------------------------------------------------------------------------*/
}



static void ebike_control_motor (void)
{
  // reset control variables (safety)
  ui16_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
  ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
  ui8_adc_battery_current_target = 0;
  ui8_duty_cycle_target = 0;

  // reset initialization of Cruise PID controller
  if (ui8_riding_mode != CRUISE_MODE) { ui8_cruise_PID_initialize = 1; }
  
  // select riding mode
  switch (ui8_riding_mode)
  {
    case POWER_ASSIST_MODE: apply_power_assist(); break;
    
    case TORQUE_ASSIST_MODE: apply_torque_assist(); break;
    
    case CADENCE_ASSIST_MODE: apply_cadence_assist(); break;
    
    case eMTB_ASSIST_MODE: apply_emtb_assist(); break;
    
    case WALK_ASSIST_MODE: apply_walk_assist(); break;
    
    case CRUISE_MODE: apply_cruise(); break;

    case CADENCE_SENSOR_CALIBRATION_MODE: apply_cadence_sensor_calibration(); break;
  }
  
  // select optional ADC function
  switch (m_configuration_variables.ui8_optional_ADC_function)
  {
    case THROTTLE_CONTROL: apply_throttle(); break;
    
    case TEMPERATURE_CONTROL: apply_temperature_limiting(); break;
  }
  
  // speed limit
  apply_speed_limit();
  
  // check if to enable the motor
  if ((!ui8_motor_enabled) &&
      (ui16_motor_get_motor_speed_erps() == 0) && // only enable motor if stopped, else something bad can happen due to high currents/regen or similar
      (ui8_adc_battery_current_target) &&
      (!ui8_brakes_engaged))
  {
    ui8_motor_enabled = 1;
    ui8_g_duty_cycle = 0;
    motor_enable_pwm();
  }

  // check if to disable the motor
  if ((ui8_motor_enabled) &&
      (ui16_motor_get_motor_speed_erps() == 0) &&
      (!ui8_adc_battery_current_target) &&
      (!ui8_g_duty_cycle))
  {
    ui8_motor_enabled = 0;
    motor_disable_pwm();
  }
  
  // reset control parameters if... (safety)
  if (ui8_brakes_engaged || ui8_system_state != NO_ERROR || !ui8_motor_enabled)
  {
    ui16_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
    ui16_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN;
    ui8_controller_adc_battery_current_target = 0;
    ui8_controller_duty_cycle_target = 0;
  }
  else
  {
    // limit max current if higher than configured hardware limit (safety)
    if (ui8_adc_battery_current_max > ADC_10_BIT_BATTERY_CURRENT_MAX) { ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX; }
    
    // limit target current if higher than max value (safety)
    if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
    
    // limit target duty cycle if higher than max value
    if (ui8_duty_cycle_target > PWM_DUTY_CYCLE_MAX) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    
    // limit target duty cycle ramp up inverse step if lower than min value (safety)
    if (ui16_duty_cycle_ramp_up_inverse_step < PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN) { ui16_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN; } 
    
    // limit target duty cycle ramp down inverse step if lower than min value (safety)
    if (ui16_duty_cycle_ramp_down_inverse_step < PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN) { ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN; } 
    
    // set duty cycle ramp up in controller
    ui16_controller_duty_cycle_ramp_up_inverse_step = ui16_duty_cycle_ramp_up_inverse_step;
    
    // set duty cycle ramp down in controller
    ui16_controller_duty_cycle_ramp_down_inverse_step = ui16_duty_cycle_ramp_down_inverse_step;
    
    // set target battery current in controller
    ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target;
    
    // set target duty cycle in controller
    ui8_controller_duty_cycle_target = ui8_duty_cycle_target;
  }
}



static void apply_power_assist()
{
  uint8_t ui8_power_assist_multiplier_x10 = ui8_riding_mode_parameter;
  
  // check for assist without pedal rotation threshold when there is no pedal rotation and standing still
  if (ui8_assist_without_pedal_rotation_threshold && !ui8_pedal_cadence_RPM && !ui16_wheel_speed_x10)
  {
    if (ui16_adc_pedal_torque_delta > (110 - ui8_assist_without_pedal_rotation_threshold)) { ui8_pedal_cadence_RPM = 4; }
  }
  
  // calculate power assist by multiplying human power with the power assist multiplier
  uint32_t ui32_power_assist_x100 = ((uint32_t) ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM * ui8_power_assist_multiplier_x10) / 96; // see note below
  
  /*------------------------------------------------------------------------

    NOTE: regarding the human power calculation
    
    (1) Formula: power = torque * rotations per second * 2 * pi
    (2) Formula: power = torque * rotations per minute * 2 * pi / 60
    (3) Formula: power = torque * rotations per minute * 0.1047
    (4) Formula: power = torque * 100 * rotations per minute * 0.001047
    (5) Formula: power = torque * 100 * rotations per minute / 955
    (6) Formula: power * 10  =  torque * 100 * rotations per minute / 96
    
  ------------------------------------------------------------------------*/
  
  // calculate target current
  uint16_t ui16_battery_current_target_x100 = (ui32_power_assist_x100 * 1000) / ui16_battery_voltage_filtered_x1000;
  
  // set battery current target in ADC steps
  uint16_t ui16_adc_battery_current_target = ui16_battery_current_target_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
  
  // set motor acceleration
  ui16_duty_cycle_ramp_up_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                             (uint32_t) 40, // 40 -> 4 kph
                                             (uint32_t) 200, // 200 -> 20 kph
                                             (uint32_t) ui16_duty_cycle_ramp_up_inverse_step_default,
                                             (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                             
  ui16_duty_cycle_ramp_down_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                               (uint32_t) 40, // 40 -> 4 kph
                                               (uint32_t) 200, // 200 -> 20 kph
                                               (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                                               (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
                                               
  // set battery current target
  if (ui16_adc_battery_current_target > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
  else { ui8_adc_battery_current_target = ui16_adc_battery_current_target; }
  
  // set duty cycle target
  if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
  else { ui8_duty_cycle_target = 0; }
}



static void apply_torque_assist()
{
  #define TORQUE_ASSIST_FACTOR_DENOMINATOR      110   // scale the torque assist target current
  
  // check for assist without pedal rotation threshold when there is no pedal rotation and standing still
  if (ui8_assist_without_pedal_rotation_threshold && !ui8_pedal_cadence_RPM && !ui16_wheel_speed_x10)
  {
    if (ui16_adc_pedal_torque_delta > (110 - ui8_assist_without_pedal_rotation_threshold)) { ui8_pedal_cadence_RPM = 1; }
  }
  
  // calculate torque assistance
  if (ui16_adc_pedal_torque_delta && ui8_pedal_cadence_RPM)
  {
    // get the torque assist factor
    uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;
    
    // calculate torque assist target current
    uint16_t ui16_adc_battery_current_target_torque_assist = ((uint16_t) ui16_adc_pedal_torque_delta * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
  
    // set motor acceleration
    ui16_duty_cycle_ramp_up_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                               (uint32_t) 40, // 40 -> 4 kph
                                               (uint32_t) 200, // 200 -> 20 kph
                                               (uint32_t) ui16_duty_cycle_ramp_up_inverse_step_default,
                                               (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                               
    ui16_duty_cycle_ramp_down_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                                 (uint32_t) 40, // 40 -> 4 kph
                                                 (uint32_t) 200, // 200 -> 20 kph
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
                                                 
    // set battery current target
    if (ui16_adc_battery_current_target_torque_assist > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
    else { ui8_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist; }

    // set duty cycle target
    if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    else { ui8_duty_cycle_target = 0; }
  }
}



static void apply_cadence_assist()
{
  #define CADENCE_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_OFFSET   50
  
  if (ui8_pedal_cadence_RPM)
  {
    // get the cadence assist duty cycle target
    uint8_t ui8_cadence_assist_duty_cycle_target = ui8_riding_mode_parameter;
    
    // limit cadence assist duty cycle target
    if (ui8_cadence_assist_duty_cycle_target > PWM_DUTY_CYCLE_MAX) { ui8_cadence_assist_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    
    // set motor acceleration
    ui16_duty_cycle_ramp_up_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                               (uint32_t) 40, // 40 -> 4 kph
                                               (uint32_t) 200, // 200 -> 20 kph
                                               (uint32_t) ui16_duty_cycle_ramp_up_inverse_step_default + CADENCE_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_OFFSET,
                                               (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                               
    ui16_duty_cycle_ramp_down_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                                 (uint32_t) 40, // 40 -> 4 kph
                                                 (uint32_t) 200, // 200 -> 20 kph
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
                                                 
    // set battery current target
    ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    
    // set duty cycle target
    ui8_duty_cycle_target = ui8_cadence_assist_duty_cycle_target;
  }
}



static void apply_emtb_assist()
{
  #define eMTB_ASSIST_ADC_TORQUE_OFFSET    10
  
  // check for assist without pedal rotation threshold when there is no pedal rotation and standing still
  if (ui8_assist_without_pedal_rotation_threshold && !ui8_pedal_cadence_RPM && !ui16_wheel_speed_x10)
  {
    if (ui16_adc_pedal_torque_delta > (110 - ui8_assist_without_pedal_rotation_threshold)) { ui8_pedal_cadence_RPM = 1; }
  }
  
  if ((ui16_adc_pedal_torque_delta > 0) && 
      (ui16_adc_pedal_torque_delta < (eMTB_POWER_FUNCTION_ARRAY_SIZE - eMTB_ASSIST_ADC_TORQUE_OFFSET)) &&
      (ui8_pedal_cadence_RPM))
  {
    // initialize eMTB assist target current
    uint8_t ui8_adc_battery_current_target_eMTB_assist = 0;
    
    // get the eMTB assist sensitivity
    uint8_t ui8_eMTB_assist_sensitivity = ui8_riding_mode_parameter;
    
    switch (ui8_eMTB_assist_sensitivity)
    {
      case 1: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_160[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 2: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_165[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 3: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_170[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 4: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_175[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 5: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_180[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 6: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_185[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 7: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_190[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 8: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_195[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 9: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_200[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 10: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_205[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 11: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_210[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 12: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_215[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 13: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_220[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 14: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_225[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 15: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_230[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 16: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_235[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 17: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_240[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 18: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_245[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 19: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_250[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
      case 20: ui8_adc_battery_current_target_eMTB_assist = ui8_eMTB_power_function_255[ui16_adc_pedal_torque_delta + eMTB_ASSIST_ADC_TORQUE_OFFSET]; break;
    }
    
    // set motor acceleration
    ui16_duty_cycle_ramp_up_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                               (uint32_t) 40, // 40 -> 4 kph
                                               (uint32_t) 200, // 200 -> 20 kph
                                               (uint32_t) ui16_duty_cycle_ramp_up_inverse_step_default,
                                               (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                               
    ui16_duty_cycle_ramp_down_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                                 (uint32_t) 40, // 40 -> 4 kph
                                                 (uint32_t) 200, // 200 -> 20 kph
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
                                                 
    // set battery current target
    if (ui8_adc_battery_current_target_eMTB_assist > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
    else { ui8_adc_battery_current_target = ui8_adc_battery_current_target_eMTB_assist; }

    // set duty cycle target
    if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    else { ui8_duty_cycle_target = 0; }
  }
}



static void apply_walk_assist()
{
  #define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP     200
  #define WALK_ASSIST_DUTY_CYCLE_MAX                      80
  #define WALK_ASSIST_ADC_BATTERY_CURRENT_MAX             80
  
  if (ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10)
  {
    // get the walk assist duty cycle target
    uint8_t ui8_walk_assist_duty_cycle_target = ui8_riding_mode_parameter;
    
    // check so that walk assist level factor is not too large (too powerful), if it is -> limit the value
    if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MAX) { ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MAX; }
    
    // set motor acceleration
    ui16_duty_cycle_ramp_up_inverse_step = WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
    ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
    
    // set battery current target
    ui8_adc_battery_current_target = ui8_min(WALK_ASSIST_ADC_BATTERY_CURRENT_MAX, ui8_adc_battery_current_max);
    
    // set duty cycle target
    ui8_duty_cycle_target = ui8_walk_assist_duty_cycle_target;
  }
}



static void apply_cruise()
{
  #define CRUISE_PID_KP                             12    // 48 volt motor: 12, 36 volt motor: 14
  #define CRUISE_PID_KI                             0.7   // 48 volt motor: 1, 36 volt motor: 0.7
  #define CRUISE_PID_INTEGRAL_LIMIT                 1000
  #define CRUISE_PID_KD                             0
  #define CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP    80
  
  if (ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10)
  {
    static int16_t i16_error;
    static int16_t i16_last_error;
    static int16_t i16_integral;
    static int16_t i16_derivative;
    static int16_t i16_control_output;
    static uint16_t ui16_wheel_speed_target_x10;
    
    // initialize cruise PID controller
    if (ui8_cruise_PID_initialize)
    {
      ui8_cruise_PID_initialize = 0;
      
      // reset PID variables
      i16_error = 0;
      i16_last_error = 0;
      i16_integral = 320; // initialize integral to a value so the motor does not start from zero
      i16_derivative = 0;
      i16_control_output = 0;
      
      // check what target wheel speed to use (received or current)
      uint16_t ui16_wheel_speed_target_received_x10 = (uint16_t) ui8_riding_mode_parameter * 10;
      
      if (ui16_wheel_speed_target_received_x10 > 0)
      {
        // set received target wheel speed to target wheel speed
        ui16_wheel_speed_target_x10 = ui16_wheel_speed_target_received_x10;
      }
      else
      {
        // set current wheel speed to maintain
        ui16_wheel_speed_target_x10 = ui16_wheel_speed_x10;
      }
    }
    
    // calculate error
    i16_error = (ui16_wheel_speed_target_x10 - ui16_wheel_speed_x10);
    
    // calculate integral
    i16_integral = i16_integral + i16_error;
    
    // limit integral
    if (i16_integral > CRUISE_PID_INTEGRAL_LIMIT)
    {
      i16_integral = CRUISE_PID_INTEGRAL_LIMIT; 
    }
    else if (i16_integral < 0)
    {
      i16_integral = 0;
    }
    
    // calculate derivative
    i16_derivative = i16_error - i16_last_error;

    // save error to last error
    i16_last_error = i16_error;
    
    // calculate control output ( output =  P I D )
    i16_control_output = (CRUISE_PID_KP * i16_error) + (CRUISE_PID_KI * i16_integral) + (CRUISE_PID_KD * i16_derivative);
    
    // limit control output to just positive values
    if (i16_control_output < 0) { i16_control_output = 0; }
    
    // limit control output to the maximum value
    if (i16_control_output > 1000) { i16_control_output = 1000; }
    
    // set motor acceleration
    ui16_duty_cycle_ramp_up_inverse_step = CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
    ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
    
    // set battery current target
    ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    
    // set duty cycle target  |  map the control output to an appropriate target PWM value
    ui8_duty_cycle_target = map((uint32_t) i16_control_output,
                                (uint32_t) 0,                     // minimum control output from PID
                                (uint32_t) 1000,                  // maximum control output from PID
                                (uint32_t) 0,                     // minimum duty cycle
                                (uint32_t) PWM_DUTY_CYCLE_MAX);   // maximum duty cycle
  }
}



static void apply_cadence_sensor_calibration()
{
  #define CADENCE_SENSOR_CALIBRATION_MODE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP     200
  #define CADENCE_SENSOR_CALIBRATION_MODE_ADC_BATTERY_CURRENT_TARGET          8   // 8 -> 8 * 0.2 = 1.6 A
  #define CADENCE_SENSOR_CALIBRATION_MODE_DUTY_CYCLE_TARGET                   24
  
  // get the ticks counter interrupt values for the different states
  uint32_t ui32_high_state = ui16_cadence_sensor_ticks_counter_min_high;
  uint32_t ui32_low_state = ui16_cadence_sensor_ticks_counter_min_low;
  
  // avoid zero division
  if ((ui32_high_state > 0) && (ui32_low_state > 0))
  {
    // calculate the cadence sensor pulse high percentage
    uint16_t ui16_cadence_sensor_pulse_high_percentage_x10_temp = (ui32_high_state * 1000) / (ui32_high_state + ui32_low_state);
    
    // limit the cadence sensor pulse high
    if (ui16_cadence_sensor_pulse_high_percentage_x10_temp > CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MAX) { ui16_cadence_sensor_pulse_high_percentage_x10_temp = CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MAX; }
    if (ui16_cadence_sensor_pulse_high_percentage_x10_temp < CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MIN) { ui16_cadence_sensor_pulse_high_percentage_x10_temp = CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MIN; }
    
    // filter the cadence sensor pulse high percentage
    ui16_cadence_sensor_pulse_high_percentage_x10 = filter(ui16_cadence_sensor_pulse_high_percentage_x10_temp, ui16_cadence_sensor_pulse_high_percentage_x10, 90);
  }
  
  // set motor acceleration
  ui16_duty_cycle_ramp_up_inverse_step = CADENCE_SENSOR_CALIBRATION_MODE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
  ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;

  // set battery current target
  ui8_adc_battery_current_target = CADENCE_SENSOR_CALIBRATION_MODE_ADC_BATTERY_CURRENT_TARGET;
  
  // set duty cycle target
  ui8_duty_cycle_target = CADENCE_SENSOR_CALIBRATION_MODE_DUTY_CYCLE_TARGET;
}



static void apply_throttle()
{
  #define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT    80
  #define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN        40
  
  // map value from 0 to 255
  ui8_adc_throttle = map((uint8_t) UI8_ADC_THROTTLE,
                         (uint8_t) ADC_THROTTLE_MIN_VALUE,
                         (uint8_t) ADC_THROTTLE_MAX_VALUE,
                         (uint8_t) 0,
                         (uint8_t) 255);
                         
  // map ADC throttle value from 0 to max battery current
  uint8_t ui8_adc_battery_current_target_throttle = map((uint8_t) ui8_adc_throttle,
                                                        (uint8_t) 0,
                                                        (uint8_t) 255,
                                                        (uint8_t) 0,
                                                        (uint8_t) ui8_adc_battery_current_max);
                                                        
  if (ui8_adc_battery_current_target_throttle > ui8_adc_battery_current_target)
  {
    // set motor acceleration
    ui16_duty_cycle_ramp_up_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                               (uint32_t) 40,
                                               (uint32_t) 400,
                                               (uint32_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                                               (uint32_t) THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                               
    ui16_duty_cycle_ramp_down_inverse_step = map((uint32_t) ui16_wheel_speed_x10,
                                                 (uint32_t) 40,
                                                 (uint32_t) 400,
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT,
                                                 (uint32_t) PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN);
                                                 
    // set battery current target
    ui8_adc_battery_current_target = ui8_adc_battery_current_target_throttle;
    
    // set duty cycle target
    ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
  }
}



static void apply_temperature_limiting()
{
  static uint16_t ui16_adc_motor_temperature_filtered;
  
  // get ADC measurement
  volatile uint16_t ui16_temp = UI16_ADC_10_BIT_THROTTLE;
  
  // filter ADC measurement to motor temperature variable
  ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 80);
  
  // convert ADC value
  ui16_motor_temperature_filtered_x10 = ((uint32_t) ui16_adc_motor_temperature_filtered * 10000) / 2048;
  
  // min temperature value can not be equal or higher than max temperature value
  if (ui8_motor_temperature_min_value_to_limit >= ui8_motor_temperature_max_value_to_limit)
  {
    ui8_adc_battery_current_target = 0;
    ui8_temperature_current_limiting_value = 0;
  }
  else
  {
    // adjust target current if motor over temperature limit
    ui8_adc_battery_current_target = map((uint32_t) ui16_motor_temperature_filtered_x10,
                                         (uint32_t) ui8_motor_temperature_min_value_to_limit * 10,
                                         (uint32_t) ui8_motor_temperature_max_value_to_limit * 10,
                                         (uint32_t) ui8_adc_battery_current_target,
                                         (uint32_t) 0);
                                         
    // get a value linear to the current limitation, just to show to user
    ui8_temperature_current_limiting_value = map((uint32_t) ui16_motor_temperature_filtered_x10,
                                                 (uint32_t) ui8_motor_temperature_min_value_to_limit * 10,
                                                 (uint32_t) ui8_motor_temperature_max_value_to_limit * 10,
                                                 (uint32_t) 255,
                                                 (uint32_t) 0);
  }
}



static void apply_speed_limit()
{
  if (m_configuration_variables.ui8_wheel_speed_max > 0)
  {
    // set battery current target 
    ui8_adc_battery_current_target = map((uint32_t) ui16_wheel_speed_x10,
                                         (uint32_t) ((m_configuration_variables.ui8_wheel_speed_max * 10) - 20),
                                         (uint32_t) ((m_configuration_variables.ui8_wheel_speed_max * 10) + 20),
                                         (uint32_t) ui8_adc_battery_current_target,
                                         (uint32_t) 0);
  }
}



static void calc_wheel_speed(void)
{ 
  // calc wheel speed in km/h
  if (ui16_wheel_speed_sensor_ticks)
  {
    float f_wheel_speed_x10 = (float) PWM_CYCLES_SECOND / ui16_wheel_speed_sensor_ticks; // rps
    ui16_wheel_speed_x10 = f_wheel_speed_x10 * m_configuration_variables.ui16_wheel_perimeter * 0.036; // rps * millimeters per second * ((3600 / (1000 * 1000)) * 10) kms per hour * 10
  }
  else
  {
    ui16_wheel_speed_x10 = 0;
  }
}



static void calc_cadence(void)
{
  #define CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED       800
  
  // get the cadence sensor ticks
  uint16_t ui16_cadence_sensor_ticks_temp = ui16_cadence_sensor_ticks;
  
  // get the cadence sensor pulse state
  uint8_t ui8_cadence_sensor_pulse_state_temp = ui8_cadence_sensor_pulse_state;
  
  // adjust cadence sensor ticks counter min depending on wheel speed
  ui16_cadence_sensor_ticks_counter_min_speed_adjusted = map((uint32_t) ui16_wheel_speed_x10,
                                                             (uint32_t) 40,
                                                             (uint32_t) 400,
                                                             (uint32_t) CADENCE_SENSOR_TICKS_COUNTER_MIN,
                                                             (uint32_t) CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED);
                                                             
  // select cadence sensor mode
  switch (ui8_cadence_sensor_mode)
  {
    case STANDARD_MODE:
    
      // calculate cadence in RPM and avoid zero division
      if (ui16_cadence_sensor_ticks_temp)
      {
        ui8_pedal_cadence_RPM = 46875 / ui16_cadence_sensor_ticks_temp;
      }
      else
      {
        ui8_pedal_cadence_RPM = 0;
      }
      
      /*-------------------------------------------------------------------------------------------------
      
        NOTE: regarding the cadence calculation
        
        Cadence in standard mode is calculated by counting how many ticks there are between two 
        transitions of LOW to HIGH.
        
        Formula for calculating the cadence in RPM:
        
        (1) Cadence in RPM = 60 / (ticks * CADENCE_SENSOR_NUMBER_MAGNETS * 0.000064)
        
        (2) Cadence in RPM = 60 / (ticks * 0.00128)
        
        (3) Cadence in RPM = 46875 / ticks
        
      -------------------------------------------------------------------------------------------------*/
    
    break;
    
    case ADVANCED_MODE:
    
      // set the pulse duty cycle in ticks
      ui16_cadence_sensor_ticks_counter_min_high = ((uint32_t) ui16_cadence_sensor_pulse_high_percentage_x10 * ui16_cadence_sensor_ticks_counter_min_speed_adjusted) / 1000;
      ui16_cadence_sensor_ticks_counter_min_low = ((uint32_t) (1000 - ui16_cadence_sensor_pulse_high_percentage_x10) * ui16_cadence_sensor_ticks_counter_min_speed_adjusted) / 1000;
      
      // calculate cadence in RPM and avoid zero division
      if (ui16_cadence_sensor_ticks_temp)
      {
        // adjust cadence calculation depending on pulse state
        if (ui8_cadence_sensor_pulse_state_temp)
        {
          ui8_pedal_cadence_RPM = ((uint32_t) (1000 - ui16_cadence_sensor_pulse_high_percentage_x10) * 46875) / ((uint32_t) ui16_cadence_sensor_ticks_temp * 1000);
        }
        else
        {
          ui8_pedal_cadence_RPM = ((uint32_t) ui16_cadence_sensor_pulse_high_percentage_x10 * 46875) / ((uint32_t) ui16_cadence_sensor_ticks_temp * 1000);
        }
      }
      else
      {
        ui8_pedal_cadence_RPM = 0;
      }
      
      /*-------------------------------------------------------------------------------------------------
      
        NOTE: regarding the cadence calculation
        
        Cadence in advanced mode is calculated by counting how many ticks there are between all 
        transitions of any kind. 
        
        By measuring all transitions it is possible to double the cadence 
        resolution or to half the response time. 
        
        When using the advanced mode it is important to adjust for the different spacings between 
        different kind of transitions. This is why there is a conversion factor.
        
        Formula for calculating the cadence in RPM using the advanced mode with 
        double the transitions:
        
        (1) Cadence in RPM = 6000 / (ticks * pulse_duty_cycle * CADENCE_SENSOR_NUMBER_MAGNETS * 0.000064)

        (2) Cadence in RPM = 6000 / (ticks * pulse_duty_cycle * 0.00128)
        
        (3) Cadence in RPM = 4687500 / (ticks * pulse_duty_cycle)


        (1) Cadence in RPM * 2 = 60 / (ticks * CADENCE_SENSOR_NUMBER_MAGNETS * 0.000064)
        
        (2) Cadence in RPM * 2 = 60 / (ticks * 0.00128)
        
        (3) Cadence in RPM * 2 = 4687500 / ticks

        
      -------------------------------------------------------------------------------------------------*/
  
    break;
    
    case CALIBRATION_MODE:
      
      // set the pedal cadence to zero because calibration is taking place
      ui8_pedal_cadence_RPM = 0;
    
    break;
  }
}



static void get_battery_voltage_filtered(void)
{
  ui16_battery_voltage_filtered_x1000 = ui16_adc_battery_voltage_filtered * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
}



static void get_battery_current_filtered(void)
{
  ui8_battery_current_filtered_x10 = ((uint16_t) ui8_adc_battery_current_filtered * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100) / 10;
}



static void get_pedal_torque(void)
{
  // get adc pedal torque
  ui16_adc_pedal_torque = UI16_ADC_10_BIT_TORQUE_SENSOR;
  
  // calculate the delta value of adc pedal torque and the adc pedal torque offset from calibration
  if (ui16_adc_pedal_torque > ui16_adc_pedal_torque_offset)
  {
    ui16_adc_pedal_torque_delta = ui16_adc_pedal_torque - ui16_adc_pedal_torque_offset;
  }
  else
  {
    ui16_adc_pedal_torque_delta = 0;
  }
  
  // calculate torque on pedals
  ui16_pedal_torque_x100 = ui16_adc_pedal_torque_delta * m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100;

  // calculate human power
  ui16_human_power_x10 = ((uint32_t) ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 96; // see note below
  
  /*------------------------------------------------------------------------

    NOTE: regarding the human power calculation
    
    (1) Formula: power = torque * rotations per second * 2 * pi
    (2) Formula: power = torque * rotations per minute * 2 * pi / 60
    (3) Formula: power = torque * rotations per minute * 0.1047
    (4) Formula: power = torque * 100 * rotations per minute * 0.001047
    (5) Formula: power = torque * 100 * rotations per minute / 955
    (6) Formula: power * 10  =  torque * 100 * rotations per minute / 96
    
  ------------------------------------------------------------------------*/
}



struct_configuration_variables* get_configuration_variables (void)
{
  return &m_configuration_variables;
}



static void check_brakes()
{
  ui8_brakes_engaged = ui8_brake_state;
}



static void check_system()
{
  #define MOTOR_BLOCKED_COUNTER_THRESHOLD               10    // 10  =>  1.0 second
  #define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10   50    // 50  =>  5.0 amps
  #define MOTOR_BLOCKED_ERPS_THRESHOLD                  10    // 10 ERPS
  #define MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD         100   // 100  =>  10 seconds
  
  static uint8_t ui8_motor_blocked_counter;
  static uint8_t ui8_motor_blocked_reset_counter;

  // if the motor blocked error is enabled start resetting it
  if (ui8_system_state == ERROR_MOTOR_BLOCKED)
  {
    // increment motor blocked reset counter with 100 milliseconds
    ui8_motor_blocked_reset_counter++;
    
    // check if the counter has counted to the set threshold for reset
    if (ui8_motor_blocked_reset_counter > MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD)
    {
      // reset motor blocked error code
      if (ui8_system_state == ERROR_MOTOR_BLOCKED) { ui8_system_state = NO_ERROR; }
      
      // reset the counter that clears the motor blocked error
      ui8_motor_blocked_reset_counter = 0;
    }
  }
  else
  {
    // if battery current is over the current threshold and the motor ERPS is below threshold start setting motor blocked error code
    if ((ui8_battery_current_filtered_x10 > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10) && (ui16_motor_get_motor_speed_erps() < MOTOR_BLOCKED_ERPS_THRESHOLD))
    {
      // increment motor blocked counter with 100 milliseconds
      ++ui8_motor_blocked_counter;
      
      // check if motor is blocked for more than some safe threshold
      if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD)
      {
        // set error code
        ui8_system_state = ERROR_MOTOR_BLOCKED;
        
        // reset motor blocked counter as the error code is set
        ui8_motor_blocked_counter = 0;
      }
    }
    else
    {
      // current is below the threshold and/or motor ERPS is above the threshold so reset the counter
      ui8_motor_blocked_counter = 0;
    }
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  // check torque sensor
  if (((ui16_adc_pedal_torque_offset > 300) || (ui16_adc_pedal_torque_offset < 10) || (ui16_adc_pedal_torque > 500)) &&
      ((ui8_riding_mode == POWER_ASSIST_MODE) || (ui8_riding_mode == TORQUE_ASSIST_MODE) || (ui8_riding_mode == eMTB_ASSIST_MODE)))
  {
    // set error code
    ui8_system_state = ERROR_TORQUE_SENSOR;
  }
  else if (ui8_system_state == ERROR_TORQUE_SENSOR)
  {
    // reset error code
    ui8_system_state = NO_ERROR;
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  // check cadence sensor calibration
  if ((ui8_cadence_sensor_mode == ADVANCED_MODE) &&
      ((ui16_cadence_sensor_pulse_high_percentage_x10 == CADENCE_SENSOR_PULSE_PERCENTAGE_X10_DEFAULT) ||
       (ui16_cadence_sensor_pulse_high_percentage_x10 > CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MAX) ||
       (ui16_cadence_sensor_pulse_high_percentage_x10 < CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MIN)))
  {
    // set error code
    ui8_system_state = ERROR_CADENCE_SENSOR_CALIBRATION;
  }
  else if (ui8_system_state == ERROR_CADENCE_SENSOR_CALIBRATION)
  {
    // reset error code
    ui8_system_state = NO_ERROR;
  }
}



void ebike_control_lights(void)
{
  #define DEFAULT_FLASH_ON_COUNTER_MAX      3
  #define DEFAULT_FLASH_OFF_COUNTER_MAX     1
  #define BRAKING_FLASH_ON_COUNTER_MAX      1
  #define BRAKING_FLASH_OFF_COUNTER_MAX     1
  
  static uint8_t ui8_default_flash_state;
  static uint8_t ui8_default_flash_state_counter; // increments every function call -> 100 ms
  static uint8_t ui8_braking_flash_state;
  static uint8_t ui8_braking_flash_state_counter; // increments every function call -> 100 ms
  
  
  /****************************************************************************/
  
  
  // increment flash counters
  ++ui8_default_flash_state_counter;
  ++ui8_braking_flash_state_counter;
  
  
  /****************************************************************************/
  
  
  // set default flash state
  if ((ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_ON_COUNTER_MAX))
  {
    // reset flash state counter
    ui8_default_flash_state_counter = 0;
    
    // toggle flash state
    ui8_default_flash_state = 0;
  }
  else if ((!ui8_default_flash_state) && (ui8_default_flash_state_counter > DEFAULT_FLASH_OFF_COUNTER_MAX))
  {
    // reset flash state counter
    ui8_default_flash_state_counter = 0;
    
    // toggle flash state
    ui8_default_flash_state = 1;
  }
  
  
  /****************************************************************************/
  
  
  // set braking flash state
  if ((ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_ON_COUNTER_MAX))
  {
    // reset flash state counter
    ui8_braking_flash_state_counter = 0;
    
    // toggle flash state
    ui8_braking_flash_state = 0;
  }
  else if ((!ui8_braking_flash_state) && (ui8_braking_flash_state_counter > BRAKING_FLASH_OFF_COUNTER_MAX))
  {
    // reset flash state counter
    ui8_braking_flash_state_counter = 0;
    
    // toggle flash state
    ui8_braking_flash_state = 1;
  }
  
  
  /****************************************************************************/
  
  
  // select lights configuration
  switch (ui8_lights_configuration)
  {
    case 0:
    
      // set lights
      lights_set_state(ui8_lights_state);
      
    break;

    case 1:
      
      // check lights state
      if (ui8_lights_state)
      {
        // set lights
        lights_set_state(ui8_default_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 2:
      
      // check light and brake state
      if (ui8_lights_state && ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_braking_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 3:
      
      // check light and brake state
      if (ui8_lights_state && ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_brakes_engaged);
      }
      else if (ui8_lights_state)
      {
        // set lights
        lights_set_state(ui8_default_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 4:
      
      // check light and brake state
      if (ui8_lights_state && ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_braking_flash_state);
      }
      else if (ui8_lights_state)
      {
        // set lights
        lights_set_state(ui8_default_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 5:
      
      // check brake state
      if (ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_brakes_engaged);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 6:
      
      // check brake state
      if (ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_braking_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 7:
      
      // check brake state
      if (ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_brakes_engaged);
      }
      else if (ui8_lights_state)
      {
        // set lights
        lights_set_state(ui8_default_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    case 8:
      
      // check brake state
      if (ui8_brakes_engaged)
      {
        // set lights
        lights_set_state(ui8_braking_flash_state);
      }
      else if (ui8_lights_state)
      {
        // set lights
        lights_set_state(ui8_default_flash_state);
      }
      else
      {
        // set lights
        lights_set_state(ui8_lights_state);
      }
      
    break;
    
    default:
    
      // set lights
      lights_set_state(ui8_lights_state);
      
    break;
  }
  
  /*------------------------------------------------------------------------------------------------------------------

    NOTE: regarding the various light modes
    
    (0) lights ON when enabled
    (1) lights FLASHING when enabled
    
    (2) lights ON when enabled and BRAKE-FLASHING when braking
    (3) lights FLASHING when enabled and ON when braking
    (4) lights FLASHING when enabled and BRAKE-FLASHING when braking
    
    (5) lights ON when enabled, but ON when braking regardless if lights are enabled
    (6) lights ON when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled
    
    (7) lights FLASHING when enabled, but ON when braking regardless if lights are enabled
    (8) lights FLASHING when enabled, but BRAKE-FLASHING when braking regardless if lights are enabled
    
  ------------------------------------------------------------------------------------------------------------------*/
}



// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    ui8_byte_received = UART2_ReceiveData8 ();

    switch (ui8_state_machine)
    {
      case 0:
      if (ui8_byte_received == 0x59) // see if we get start package byte
      {
        ui8_rx_buffer [ui8_rx_counter] = ui8_byte_received;
        ui8_rx_counter++;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      ui8_rx_buffer [ui8_rx_counter] = ui8_byte_received;
      
      // increment index for next byte
      ui8_rx_counter++;

      // reset if it is the last byte of the package and index is out of bounds
      if (ui8_rx_counter >= UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3)
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
        ui8_received_package_flag = 1; // signal that we have a full package to be processed
        UART2->CR2 &= ~(1 << 5); // disable UART2 receive interrupt
      }
      break;

      default:
      break;
    }
  }
}

static void communications_controller (void)
{
#ifndef DEBUG_UART

  // reset riding mode (safety)
  ui8_riding_mode = OFF_MODE;
  
  uart_receive_package ();

  uart_send_package ();

#endif
}

static void uart_receive_package(void)
{
  if (ui8_received_package_flag)
  {
    // validation of the package data
    ui16_crc_rx = 0xffff;
    
    for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_RECEIVE; ui8_i++)
    {
      crc16 (ui8_rx_buffer[ui8_i], &ui16_crc_rx);
    }

    // if CRC is correct read the package (16 bit value and therefore last two bytes)
    if (((((uint16_t) ui8_rx_buffer [UART_NUMBER_DATA_BYTES_TO_RECEIVE + 2]) << 8) + ((uint16_t) ui8_rx_buffer [UART_NUMBER_DATA_BYTES_TO_RECEIVE + 1])) == ui16_crc_rx)
    {
      // message ID
      ui8_message_ID = ui8_rx_buffer [1];
      
      // riding mode
      ui8_riding_mode = ui8_rx_buffer [2];
      
      // riding mode parameter
      ui8_riding_mode_parameter = ui8_rx_buffer [3];
      
      // lights state
      ui8_lights_state = ui8_rx_buffer [4];

      switch (ui8_message_ID)
      {
        case 0:
        
          // battery low voltage cut off x10
          m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer [6]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
          
          // set low voltage cutoff (8 bit)
          ui8_adc_battery_voltage_cut_off = ((uint32_t) m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 * 25) / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
          
          // wheel max speed
          m_configuration_variables.ui8_wheel_speed_max = ui8_rx_buffer [7];
          
        break;

        case 1:
        
          // wheel perimeter
          m_configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer [6]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
          
          // motor temperature limit function or throttle
          m_configuration_variables.ui8_optional_ADC_function = ui8_rx_buffer [7];

        break;

        case 2:
        
          // type of motor (36 volt, 48 volt or some experimental type)
          m_configuration_variables.ui8_motor_type = ui8_rx_buffer[5];
          
          // motor over temperature min value limit
          ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer[6];
          
          // motor over temperature max value limit
          ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer[7];

        break;

        case 3:
        
          // = ui8_rx_buffer[5];
          
          // = ui8_rx_buffer[6];
          
          // = ui8_rx_buffer[7];
          
        break;

        case 4:
          
          // lights configuration
          ui8_lights_configuration = ui8_rx_buffer[5];
          
          // assist without pedal rotation threshold
          ui8_assist_without_pedal_rotation_threshold = ui8_rx_buffer[6];
          
          // check if assist without pedal rotation threshold is valid (safety)
          if (ui8_assist_without_pedal_rotation_threshold > 100) { ui8_assist_without_pedal_rotation_threshold = 0; }
          
          // motor acceleration adjustment
          uint8_t ui8_motor_acceleration_adjustment = ui8_rx_buffer[7];
          
          // set duty cycle ramp up inverse step
          ui16_duty_cycle_ramp_up_inverse_step_default = map((uint32_t) ui8_motor_acceleration_adjustment,
                                                             (uint32_t) 0,
                                                             (uint32_t) 100,
                                                             (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                                                             (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                                             
        break;

        case 5:
        
          // pedal torque conversion
          m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_rx_buffer[5];
          
          // max battery current
          ui8_battery_current_max = ui8_rx_buffer[6];
          
          // battery power limit
          m_configuration_variables.ui8_target_battery_max_power_div25 = ui8_rx_buffer[7];
          
          // calculate max battery current in ADC steps from the received battery current limit
          uint8_t ui8_adc_battery_current_max_temp_1 = ((uint16_t) ui8_battery_current_max * 100) / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
          
          // calculate max battery current in ADC steps from the received power limit
          uint32_t ui32_battery_current_max_x100 = ((uint32_t) m_configuration_variables.ui8_target_battery_max_power_div25 * 2500000) / ui16_battery_voltage_filtered_x1000;
          uint8_t ui8_adc_battery_current_max_temp_2 = ui32_battery_current_max_x100 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100;
          
          // set max battery current
          ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max_temp_1, ui8_adc_battery_current_max_temp_2);
        
        break;
        
        case 6:
          
          // cadence sensor mode
          ui8_cadence_sensor_mode = ui8_rx_buffer[5];
          
          // cadence sensor pulse high percentage
          if (ui8_cadence_sensor_mode == ADVANCED_MODE)
          {
            ui16_cadence_sensor_pulse_high_percentage_x10 = (((uint16_t) ui8_rx_buffer[7]) << 8) + ((uint16_t) ui8_rx_buffer[6]);
          }
          
          /*-------------------------------------------------------------------------------------------------
          
            NOTE: regarding the cadence sensor mode and cadence sensor pulse high percentage
            
            These two variables need to be received at the same time. If they are not it might trigger the
            ERROR_CADENCE_SENSOR_CALIBRATION flag.
            
          -------------------------------------------------------------------------------------------------*/

        break;

        default:
          // nothing, should display error code
        break;
      }
    }
    
    // signal that we processed the full package
    ui8_received_package_flag = 0;

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }
}

static void uart_send_package(void)
{
  uint16_t ui16_temp;

  // start up byte
  ui8_tx_buffer[0] = 0x43;

  // battery voltage filtered x1000
  ui16_temp = ui16_battery_voltage_filtered_x1000;
  ui8_tx_buffer[1] = (uint8_t) (ui16_temp & 0xff);;
  ui8_tx_buffer[2] = (uint8_t) (ui16_temp >> 8);
  
  // battery current filtered x10
  ui8_tx_buffer[3] = ui8_battery_current_filtered_x10;

  // wheel speed x10
  ui8_tx_buffer[4] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
  ui8_tx_buffer[5] = (uint8_t) (ui16_wheel_speed_x10 >> 8);

  // brake state
  ui8_tx_buffer[6] = ui8_brakes_engaged;

  // optional ADC channel value
  ui8_tx_buffer[7] = UI8_ADC_THROTTLE;
  
  // throttle or temperature control
  switch (m_configuration_variables.ui8_optional_ADC_function)
  {
    case THROTTLE_CONTROL:
      
      // throttle value with offset applied and mapped from 0 to 255
      ui8_tx_buffer[8] = ui8_adc_throttle;
    
    break;
    
    case TEMPERATURE_CONTROL:
    
      // current limiting mapped from 0 to 255
      ui8_tx_buffer[8] = ui8_temperature_current_limiting_value;
    
    break;
  }

  // ADC torque sensor
  ui16_temp = ui16_adc_pedal_torque;
  ui8_tx_buffer[9] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[10] = (uint8_t) (ui16_temp >> 8);

  // pedal cadence
  ui8_tx_buffer[11] = ui8_pedal_cadence_RPM;

  // PWM duty_cycle
  ui8_tx_buffer[12] = ui8_g_duty_cycle;
  
  // motor speed in ERPS
  ui16_temp = ui16_motor_get_motor_speed_erps();
  ui8_tx_buffer[13] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[14] = (uint8_t) (ui16_temp >> 8);
  
  // FOC angle
  ui8_tx_buffer[15] = ui8_g_foc_angle;
  
  // system state
  ui8_tx_buffer[16] = ui8_system_state;
  
  // motor temperature
  ui8_tx_buffer[17] = ui16_motor_temperature_filtered_x10 / 10;
  
  // wheel_speed_sensor_tick_counter
  ui8_tx_buffer[18] = (uint8_t) (ui32_wheel_speed_sensor_ticks_total & 0xff);
  ui8_tx_buffer[19] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 8) & 0xff);
  ui8_tx_buffer[20] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 16) & 0xff);

  // pedal torque x100
  ui16_temp = ui16_pedal_torque_x100;
  ui8_tx_buffer[21] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[22] = (uint8_t) (ui16_temp >> 8);

  // human power x10
  ui8_tx_buffer[23] = (uint8_t) (ui16_human_power_x10 & 0xff);
  ui8_tx_buffer[24] = (uint8_t) (ui16_human_power_x10 >> 8);
  
  // cadence sensor pulse high percentage
  ui16_temp = ui16_cadence_sensor_pulse_high_percentage_x10;
  ui8_tx_buffer[25] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[26] = (uint8_t) (ui16_temp >> 8);

  // prepare crc of the package
  ui16_crc_tx = 0xffff;
  
  for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_SEND; ui8_i++)
  {
    crc16 (ui8_tx_buffer[ui8_i], &ui16_crc_tx);
  }
  
  ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 1] = (uint8_t) (ui16_crc_tx & 0xff);
  ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 2] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

  // send the full package to UART
  for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_SEND + 2; ui8_i++)
  {
    putchar (ui8_tx_buffer[ui8_i]);
  }
}





/* static void apply_boost()
{
  ui8_boost_enabled_and_applied = 0;
  uint8_t ui8_adc_max_battery_current_boost_state = 0;

  // 1.6 = 1 / 0.625 (each adc step for current)
  // 25 * 1.6 = 40
  // 40 * 4 = 160
  if(m_configuration_variables.ui8_startup_motor_power_boost_assist_level > 0)
  {
    uint32_t ui32_temp;
    ui32_temp = (uint32_t) ui16_pedal_torque_x100 * (uint32_t) m_configuration_variables.ui8_startup_motor_power_boost_assist_level;
    ui32_temp /= 100;

    // 1.6 = 1 / 0.625 (each adc step for current)
    // 1.6 * 8 = ~13
    ui32_temp = (ui32_temp * 13000) / ((uint32_t) ui16_battery_voltage_filtered_x1000);
    ui8_adc_max_battery_current_boost_state = ui32_temp >> 3;
    ui8_limit_max(&ui8_adc_max_battery_current_boost_state, 255);
  }
  
  // apply boost and boost fade out
  if(m_configuration_variables.ui8_startup_motor_power_boost_feature_enabled)
  {
    boost_run_statemachine();
    ui8_boost_enabled_and_applied = boost(ui8_adc_max_battery_current_boost_state);
    apply_boost_fade_out();
  }
}


static uint8_t boost(uint8_t ui8_max_current_boost_state)
{
  uint8_t ui8_boost_enable = ui8_startup_boost_enable && ui8_riding_mode_parameter && ui8_pedal_cadence_RPM > 0 ? 1 : 0;

  if (ui8_boost_enable)
  {
    ui8_adc_battery_current_target = ui8_max_current_boost_state;
  }

  return ui8_boost_enable;
}


static void apply_boost_fade_out()
{
  if (ui8_startup_boost_fade_enable)
  {
    // here we try to converge to the regular value, ramping down or up step by step
    uint16_t ui16_adc_battery_target_current_x256 = ((uint16_t) ui8_adc_battery_current_target) << 8;
    if (ui16_startup_boost_fade_variable_x256 > ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 -= ui16_startup_boost_fade_variable_step_amount_x256;
    }
    else if (ui16_startup_boost_fade_variable_x256 < ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 += ui16_startup_boost_fade_variable_step_amount_x256;
    }

    ui8_adc_battery_current_target = (uint8_t) (ui16_startup_boost_fade_variable_x256 >> 8);
  }
}

static void boost_run_statemachine(void)
{
  #define BOOST_STATE_BOOST_DISABLED        0
  #define BOOST_STATE_BOOST                 1
  #define BOOST_STATE_FADE                  2
  #define BOOST_STATE_BOOST_WAIT_TO_RESTART 3
  
  uint8_t ui8_torque_sensor = ui16_adc_pedal_torque_delta;

  if(m_configuration_variables.ui8_startup_motor_power_boost_time > 0)
  {
    switch(ui8_m_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
      
        if (ui8_torque_sensor > 12 && (ui8_brakes_engaged == 0))
        {
          ui8_startup_boost_enable = 1;
          ui8_startup_boost_timer = m_configuration_variables.ui8_startup_motor_power_boost_time;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST;
        }
        
      break;

      case BOOST_STATE_BOOST:
      
        // braking means reseting
        if(ui8_brakes_engaged)
        {
          ui8_startup_boost_enable = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        // end boost if
        if(ui8_torque_sensor < 12)
        {
          ui8_startup_boost_enable = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }

        // decrement timer
        if(ui8_startup_boost_timer > 0) { ui8_startup_boost_timer--; }

        // end boost and start fade if
        if(ui8_startup_boost_timer == 0)
        {
          ui8_m_startup_boost_state_machine = BOOST_STATE_FADE;
          ui8_startup_boost_enable = 0;

          // setup variables for fade
          ui8_startup_boost_fade_steps = m_configuration_variables.ui8_startup_motor_power_boost_fade_time;
          ui16_startup_boost_fade_variable_x256 = ((uint16_t) ui8_adc_battery_current_target << 8);
          ui16_startup_boost_fade_variable_step_amount_x256 = (ui16_startup_boost_fade_variable_x256 / ((uint16_t) ui8_startup_boost_fade_steps));
          ui8_startup_boost_fade_enable = 1;
        }
      break;

      case BOOST_STATE_FADE:
        // braking means reseting
        if(ui8_brakes_engaged)
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        if(ui8_startup_boost_fade_steps > 0) { ui8_startup_boost_fade_steps--; }

        // disable fade if
        if(ui8_torque_sensor < 12 ||
            ui8_startup_boost_fade_steps == 0)
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }
      break;

      // restart when user is not pressing the pedals AND/OR wheel speed = 0
      case BOOST_STATE_BOOST_WAIT_TO_RESTART:
        // wheel speed must be 0 as also torque sensor
        if((m_configuration_variables.ui8_startup_motor_power_boost_state & 1) == 0)
        {
          if(ui16_wheel_speed_x10 == 0 &&
              ui8_torque_sensor < 12)
          {
            ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        // torque sensor must be 0
        if((m_configuration_variables.ui8_startup_motor_power_boost_state & 1) > 0)
        {
          if(ui8_torque_sensor < 12 ||
              ui8_pedal_cadence_RPM == 0)
          {
            ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
      break;

      default:
      break;
    }
  }
} */