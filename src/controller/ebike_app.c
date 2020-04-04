/*
 * TongSheng TSDZ2 motor controller firmware
 *
 * Copyright (C) Casainho and EndlessCadence, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "ebike_app.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "main.h"
#include "interrupts.h"
#include "adc.h"
#include "utils.h"
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "config.h"
#include "utils.h"
#include "lights.h"

#define STATE_NO_PEDALLING                0
#define STATE_PEDALLING                   2

// BOOST state
#define BOOST_STATE_BOOST_DISABLED        0
#define BOOST_STATE_BOOST                 1
#define BOOST_STATE_FADE                  2
#define BOOST_STATE_BOOST_WAIT_TO_RESTART 3

uint8_t ui8_adc_battery_max_current       = ADC_BATTERY_CURRENT_MAX;
uint8_t ui8_target_battery_max_power_x10  = ADC_BATTERY_CURRENT_MAX;

volatile struct_config_vars m_config_vars;

// Error state
#define NO_ERROR                                0
#define ERROR_NOT_INIT                          (1 << 1)
#define ERROR_MOTOR_BLOCKED                     (1 << 2)
#define ERROR_TORQUE_APPLIED_DURING_POWER_ON    (1 << 3)
#define ERROR_BRAKE_APPLIED_DURING_POWER_ON     (1 << 4)
#define ERROR_THROTTLE_APPLIED_DURING_POWER_ON  (1 << 5)
#define ERROR_NO_SPEED_SENSOR_DETECTED          (1 << 6)
#define ERROR_FATAL                             (1 << 7)

// Motor init state
#define MOTOR_INIT_STATE_RESET                  0
#define MOTOR_INIT_STATE_NO_INIT                1
#define MOTOR_INIT_STATE_INIT_START_DELAY       2
#define MOTOR_INIT_STATE_INIT_WAIT_DELAY        3
#define MOTOR_INIT_OK                           4

// Motor init status
#define MOTOR_INIT_STATUS_RESET                 0
#define MOTOR_INIT_STATUS_GOT_CONFIG            1
#define MOTOR_INIT_STATUS_INIT_OK               2

// Communications package frame type
#define COMM_FRAME_TYPE_ALIVE                         0
#define COMM_FRAME_TYPE_STATUS                        1
#define COMM_FRAME_TYPE_PERIODIC                      2
#define COMM_FRAME_TYPE_CONFIGURATIONS                3
#define COMM_FRAME_TYPE_FIRMWARE_VERSION              4

// variables for various system functions
volatile uint8_t ui8_m_system_state = ERROR_NOT_INIT; // start with system error because configurations are empty at startup
volatile uint8_t ui8_m_motor_init_state = MOTOR_INIT_STATE_RESET;
volatile uint8_t ui8_m_motor_init_status = MOTOR_INIT_STATUS_RESET;
volatile uint16_t ui16_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t ui8_g_pedaling_direction = 0;
uint8_t   ui8_pas_cadence_rpm = 0;
uint16_t  ui16_m_pedal_torque_x10;
uint16_t  ui16_m_pedal_torque_x100;
uint16_t  ui16_m_pedal_power_x10;
uint16_t  ui16_m_pedal_power_max_x10;
uint8_t   ui8_m_pedal_human_power = 0;
uint8_t ui8_pas_pedal_position_right = 0;
uint16_t  ui16_m_adc_motor_temperatured_accumulated = 0;
uint16_t   ui16_m_adc_target_current;
uint8_t ui8_tstr_state_machine = STATE_NO_PEDALLING;
static volatile uint8_t ui8_m_motor_enabled = 0;
static uint8_t ui8_m_brake_is_set = 0;
volatile uint8_t  ui8_throttle = 0;
volatile uint16_t  ui16_m_torque_sensor_weight_x10 = 0;
volatile uint16_t  ui16_m_torque_sensor_weight_raw_x10 = 0;
volatile uint16_t  ui16_m_torque_sensor_weight_raw_with_offset_x10 = 0;
uint16_t  ui16_m_torque_sensor_weight_offset_x10 = 0;
uint8_t  ui8_m_first_time_torque_sensor_weight = 1;
volatile uint8_t  ui8_m_torque_sensor_weight_max = 0;
static volatile uint16_t ui16_m_torque_sensor_adc_steps = 0;
volatile uint16_t ui16_m_torque_sensor_raw = 0;
volatile uint16_t ui16_m_adc_torque_sensor_raw = 0;
volatile uint16_t ui16_g_adc_torque_sensor_min_value;
volatile uint8_t ui8_g_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
uint16_t ui16_m_adc_battery_current_max;
uint16_t ui16_m_adc_motor_current_max;
volatile uint16_t ui16_g_current_ramp_up_inverse_step;


// variables for walk assist
uint8_t ui8_m_walk_assist_target_duty_cycle = 0;


// variables for cruise function
static uint8_t    ui8_cruise_target_PWM = 0;
static uint8_t    ui8_initialize_cruise_PID = 1;
static uint16_t   ui16_received_target_wheel_speed_x10 = 0;
static uint16_t   ui16_target_wheel_speed_x10 = 0;


// variables for wheel speed
volatile uint16_t   ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
uint8_t             ui8_wheel_speed_max = 0;
float               f_wheel_speed_x10;
static uint16_t     ui16_wheel_speed_x10;
volatile uint32_t   ui32_wheel_speed_sensor_tick_counter = 0;


// UART
#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   85
#define UART_NUMBER_DATA_BYTES_TO_SEND      28

volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE];
volatile uint8_t ui8_rx_cnt = 0;
volatile uint8_t ui8_rx_len = 0;
volatile uint8_t ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND];
volatile uint8_t ui8_i;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
static uint16_t  ui16_crc_rx;
static uint16_t  ui16_crc_tx;
volatile uint8_t ui8_message_ID = 0;

static void communications_controller(void);
static void communications_process_packages(uint8_t ui8_frame_type);

// system functions
static void ebike_control_motor(void);
static void ebike_app_set_battery_max_current(uint8_t ui8_value);
static void ebike_app_set_motor_max_current(uint8_t ui8_value);
static void ebike_app_set_target_adc_battery_max_current(uint16_t ui16_value);
static void ebike_app_set_target_adc_motor_max_current(uint16_t ui16_value);

static void check_system(void);
static void throttle_read(void);
static void read_pas_cadence(void);
static void torque_sensor_read(void);
static void linearize_torque_sensor_to_kgs(uint16_t *ui16_p_torque_sensor_adc_steps, uint16_t *ui16_torque_sensor_weight, uint8_t *ui8_p_pas_pedal_right);
static void calc_pedal_force_and_torque(void);
static void calc_wheel_speed(void);
static void calc_motor_temperature(void);
static uint16_t calc_filtered_battery_voltage(void);

static void apply_speed_limit(uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint16_t *ui16_target_current);
static void apply_temperature_limiting(uint16_t *ui16_target_current);
static void apply_walk_assist(uint16_t *ui16_p_adc_target_current);
static void apply_cruise(uint16_t *ui16_target_current);
static void apply_throttle(uint8_t ui8_throttle_value, uint16_t *ui16_target_current);


// BOOST
uint8_t   ui8_startup_boost_enable = 0;
uint8_t   ui8_startup_boost_fade_enable = 0;
uint8_t   ui8_m_startup_boost_state_machine = 0;
uint8_t   ui8_startup_boost_no_torque = 0;
uint8_t   ui8_startup_boost_timer = 0;
uint8_t   ui8_startup_boost_fade_steps = 0;
uint16_t  ui16_startup_boost_fade_variable_x256;
uint16_t  ui16_startup_boost_fade_variable_step_amount_x256;
static void boost_run_statemachine(void);
static uint8_t apply_boost(uint8_t ui8_pas_cadence, uint16_t ui16_max_current_boost_state, uint16_t *ui16_target_current);
static void apply_boost_fade_out(uint16_t *ui16_adc_target_current);

#define TORQUE_SENSOR_LINEARIZE_NR_POINTS 8
uint16_t ui16_torque_sensor_linearize_right[TORQUE_SENSOR_LINEARIZE_NR_POINTS][2];
uint16_t ui16_torque_sensor_linearize_left[TORQUE_SENSOR_LINEARIZE_NR_POINTS][2];

static uint8_t m_ui8_got_configurations_timer = 0;

static uint8_t ui8_comm_error_counter = 0;

// Measured on 2020.01.02 by Casainho, the following function takes about 35ms to execute
void ebike_app_controller(void)
{
  throttle_read();
  torque_sensor_read();
  read_pas_cadence();
  calc_pedal_force_and_torque();
  calc_wheel_speed();
  calc_motor_temperature();
  ebike_control_motor();
  communications_controller();
  check_system();
}

static void ebike_control_motor(void)
{
  uint32_t ui32_temp = 0;
  uint32_t ui32_pedal_power_no_cadence_x10 = 0;
  uint32_t ui32_assist_level_factor_x1000;
  uint32_t ui32_current_amps_x10 = 0;
  uint8_t ui8_tmp_pas_cadence_rpm;
  uint16_t ui16_adc_current;
  uint16_t ui16_adc_max_battery_power_current = 0;
  uint8_t ui8_boost_enabled_and_applied = 0;
  uint16_t ui16_adc_max_current_boost_state = 0;
  uint16_t ui16_battery_voltage_filtered = calc_filtered_battery_voltage();

  // the ui8_m_brake_is_set is updated here only and used all over ebike_control_motor()
  ui8_m_brake_is_set = brake_is_set();

  // make sure this vars are reset to avoid repetion code on next elses
  ui16_m_adc_target_current = 0;

  // controller works with no less than 15V so calculate the target current only for higher voltages
  if (ui16_battery_voltage_filtered > 15)
  {
    if (m_config_vars.ui16_assist_level_factor_x1000 > 0)
    {
      ui32_assist_level_factor_x1000 = (uint32_t) m_config_vars.ui16_assist_level_factor_x1000;
      // force a min of 10 RPM cadence
      ui32_pedal_power_no_cadence_x10 = (((uint32_t) ui16_m_pedal_torque_x100 * 10) / (uint32_t) 96);

      if (m_config_vars.ui8_motor_assistance_startup_without_pedal_rotation == 0 ||
          ui8_pas_cadence_rpm)
      {
        ui32_current_amps_x10 = ((uint32_t) ui16_m_pedal_power_x10 * ui32_assist_level_factor_x1000) / 1000;
      }
      else
      {
        ui32_current_amps_x10 = (ui32_pedal_power_no_cadence_x10 * ui32_assist_level_factor_x1000) / 1000;
      }

      // 6.410 = 1 / 0.156 (each ADC step for current)
      // 6.410 * 8 = ~51
      ui16_adc_current = (uint16_t) ((ui32_current_amps_x10 * 51) / 80);
      ui16_limit_max(&ui16_adc_current, ui16_m_adc_motor_current_max);

      // if user is rotating the pedals, force use the min current value
      if (ui8_pas_cadence_rpm &&
          ui16_adc_current < m_config_vars.ui8_battery_current_min_adc)
      {
        ui16_adc_current = m_config_vars.ui8_battery_current_min_adc;
      }

      ui16_m_adc_target_current = ui16_adc_current;

      // now calculate the current for BOOST
      if (m_config_vars.ui16_startup_motor_power_boost_assist_level > 0)
      {
        ui32_current_amps_x10 = (ui32_pedal_power_no_cadence_x10 * (uint32_t) m_config_vars.ui16_startup_motor_power_boost_assist_level) / 100;

        // 6.410 = 1 / 0.156 (each ADC step for current)
        // 6.410 * 8 = ~51
        ui16_adc_current = (uint16_t) ((ui32_current_amps_x10 * 51) / 80);
        ui16_limit_max(&ui16_adc_current, ui16_m_adc_motor_current_max);

        // if user is rotating the pedals, force use the min current value
        if (ui8_pas_cadence_rpm &&
            ui16_adc_current < m_config_vars.ui8_battery_current_min_adc)
        {
          ui16_adc_current = m_config_vars.ui8_battery_current_min_adc;
        }

        ui16_adc_max_current_boost_state = ui16_adc_current;
      }
    }
    else
    {
      // nothing
    }

//    if(m_config_vars.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
//    {
//      // 160 is the factor to convert from AMPS_DIV25 to ADC steps
//      // 6.410 = 1 / 0.156 (each ADC step for current)
//      // 6.410 * 25 = 160
//      ui16_adc_max_battery_power_current = (((uint32_t) m_config_vars.ui8_target_battery_max_power_div25) * 160) / ((uint32_t) ui16_battery_voltage_filtered);
//    }
//    else
//    {
//      // nothing
//    }
  }
  else
  {
    // nothing
  }

  ui8_tmp_pas_cadence_rpm = ui8_pas_cadence_rpm;
  // let's cheat next value, only to cheat apply_boost()
  if (m_config_vars.ui8_motor_assistance_startup_without_pedal_rotation)
  {
    if (ui8_tmp_pas_cadence_rpm < 10) { ui8_tmp_pas_cadence_rpm = 10; }
  }

  // apply boost and boost fade out
  if(m_config_vars.ui8_startup_motor_power_boost_feature_enabled)
  {
    boost_run_statemachine();
    ui8_boost_enabled_and_applied = apply_boost(ui8_tmp_pas_cadence_rpm, ui16_adc_max_current_boost_state, &ui16_m_adc_target_current);
    apply_boost_fade_out(&ui16_m_adc_target_current);
  }

  // throttle
  apply_throttle(ui8_throttle, &ui16_m_adc_target_current);

  // walk assist or cruise
  if(m_config_vars.ui8_walk_assist)
  {
    // enable walk assist or cruise function depending on speed
    if(ui16_wheel_speed_x10 < WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10) // if current speed is less than threshold speed, enable walk assist
    {
      // enable walk assist
      apply_walk_assist(&ui16_m_adc_target_current);
    }
    else // if current speed is more or equal the threshold speed, enable cruise function
    {
      // enable cruise function
      apply_cruise(&ui16_m_adc_target_current);
    }
  }
  else
  {
    // set flag to initialize cruise PID if user later activates function
    ui8_initialize_cruise_PID = 1;
  }
  
  // speed limit
  apply_speed_limit(ui16_wheel_speed_x10, m_config_vars.ui8_wheel_max_speed, &ui16_m_adc_target_current);

//  // max power
//  if(m_config_vars.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
//  {
//    // limit the current to max value defined by user on LCD max power, if:
//    // - user defined to make that limitation
//    // - we are not on boost or fade state
//    if((m_config_vars.ui8_startup_motor_power_boost_limit_to_max_power == 1) ||
//        (!((ui8_boost_enabled_and_applied == 1) ||
//            (ui8_startup_boost_fade_enable == 1))))
//    {
//      if (ui16_adc_max_battery_power_current)
//        ui16_limit_max(&ui16_m_adc_target_current, ui16_adc_max_battery_power_current);
//      ui16_m_adc_battery_current_max
//    }
//    else
//    {
//      ui16_adc_max_battery_power_current = 0;
//    }
//  }

  // motor over temperature protection
  apply_temperature_limiting(&ui16_m_adc_target_current);

  // check if motor init delay has to be done
  switch (ui8_m_motor_init_state)
  {
    case MOTOR_INIT_STATE_INIT_START_DELAY:
      m_ui8_got_configurations_timer = 20;
      ui8_m_motor_init_state = MOTOR_INIT_STATE_INIT_WAIT_DELAY;
      // no break to execute next code

    case MOTOR_INIT_STATE_INIT_WAIT_DELAY:
      if (m_ui8_got_configurations_timer > 0) {
        m_ui8_got_configurations_timer--;
      }
      else
      {
        ui8_m_motor_init_state = MOTOR_INIT_OK;
        ui8_m_motor_init_status = MOTOR_INIT_STATUS_INIT_OK;
        ui8_m_system_state &= ~ERROR_NOT_INIT;
      }
      break;
  }

  // let's force our target current to 0 if brake is set or if there are errors
  if(ui8_m_brake_is_set || (ui8_m_system_state != NO_ERROR))
  {
    ui16_m_adc_target_current = 0;
  }

  // check to see if we should enable the motor
  if(ui8_m_motor_enabled == 0 &&
    (ui16_motor_get_motor_speed_erps() == 0) && // we can only enable if motor is stopped other way something bad can happen due to high currents/regen or something like that
     ui16_m_adc_target_current)
  {
    ui8_m_motor_enabled = 1;
    ui8_g_duty_cycle = 0;
    motor_enable_pwm();
  }

  // check to see if we should disable the motor
  if(ui8_m_system_state != NO_ERROR ||
      (ui8_m_motor_enabled &&
      ui16_motor_get_motor_speed_erps() == 0 &&
      ui16_m_adc_target_current == 0 &&
      ui8_g_duty_cycle == 0))
  {
    ui8_m_motor_enabled = 0;
    motor_disable_pwm();
  }

  // apply the target current if motor is enable and if not, reset the duty_cycle controller
  if(ui8_m_motor_enabled)
  {
    ebike_app_set_target_adc_motor_max_current(ui16_m_adc_target_current);
    ebike_app_set_target_adc_battery_max_current(ui16_m_adc_battery_current_max);
  }
  else
  {
    ebike_app_set_target_adc_motor_max_current(0);
    ebike_app_set_target_adc_battery_max_current(0);
    ui8_g_duty_cycle = 0;
  }

  // set motor PWM target
  if(m_config_vars.ui8_walk_assist && ui8_m_brake_is_set == 0 && ui8_m_motor_enabled)
  {
    if(ui16_wheel_speed_x10 < WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10)
    {
      motor_set_pwm_duty_cycle_target(ui8_m_walk_assist_target_duty_cycle);
    }
    else
    {
      motor_set_pwm_duty_cycle_target(ui8_cruise_target_PWM);
    }
  }
  else if(ui16_m_adc_target_current)
  {
    motor_set_pwm_duty_cycle_target(255);
  }
  else
  {
    motor_set_pwm_duty_cycle_target(0);
  }

  // let's reset this counter, meaning this code is called from main loop
  ui16_main_loop_wdt_cnt_1 = 0;
}

static void communications_controller(void)
{
#ifndef DEBUG_UART
  uint8_t ui8_frame_type_to_send = 0;
  uint8_t ui8_len;

  if (ui8_received_package_flag)
  {
    // just to make easy next calculations
    ui16_crc_rx = 0xffff;
    ui8_len = ui8_rx_buffer[1];
    for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
    {
      crc16(ui8_rx_buffer[ui8_i], &ui16_crc_rx);
    }

    // if CRC is correct read the package
    if (((((uint16_t) ui8_rx_buffer[ui8_len + 1]) << 8) +
          ((uint16_t) ui8_rx_buffer[ui8_len])) == ui16_crc_rx)
    {
      ui8_comm_error_counter = 0;

      if (ui8_m_motor_init_state == MOTOR_INIT_STATE_RESET)
        ui8_m_motor_init_state = MOTOR_INIT_STATE_NO_INIT;

      ui8_frame_type_to_send = ui8_rx_buffer[2];
      communications_process_packages(ui8_frame_type_to_send);
    }
    else
    {
      ui8_received_package_flag = 0;
      ui8_comm_error_counter++;
    }
  }
  else
  {
    ui8_comm_error_counter++;
  }

  // check for communications fail or display master fail
  // can't fail more then 800ms
  if (ui8_comm_error_counter > 7) {
    motor_disable_pwm();
    ui8_m_motor_enabled = 0;
    ui8_m_system_state |= ERROR_FATAL;
  }

  if (ui8_m_motor_init_state == MOTOR_INIT_STATE_RESET)
    communications_process_packages(COMM_FRAME_TYPE_ALIVE);
#endif
}

static void communications_process_packages(uint8_t ui8_frame_type)
{
  uint16_t ui16_temp;
  uint32_t ui32_temp;
  uint8_t ui8_len = 3; // 3 bytes: 1 type of frame + 2 CRC bytes
  uint8_t j;
  uint8_t i;

  // start up byte
  ui8_tx_buffer[0] = 0x43;
  ui8_tx_buffer[2] = ui8_frame_type;

  // prepare payload
  switch (ui8_frame_type) {
    // periodic data
    case COMM_FRAME_TYPE_PERIODIC:
      // display will send periodic command after motor init ok, now reset so the state machine will be ready for next time
      ui8_m_motor_init_status = MOTOR_INIT_STATUS_RESET;

      // assist level
      m_config_vars.ui16_assist_level_factor_x1000 = (((uint16_t) ui8_rx_buffer[4]) << 8) + ((uint16_t) ui8_rx_buffer[3]);

      // lights state
      m_config_vars.ui8_lights = (ui8_rx_buffer[5] & (1 << 0)) ? 1: 0;

      // set lights
      lights_set_state (m_config_vars.ui8_lights);

      // walk assist / cruise function
      m_config_vars.ui8_walk_assist = (ui8_rx_buffer[5] & (1 << 1)) ? 1: 0;

      // battery max power target
      m_config_vars.ui8_target_battery_max_power_div25 = ui8_rx_buffer[6];

      // startup motor power boost
      m_config_vars.ui16_startup_motor_power_boost_assist_level = (((uint16_t) ui8_rx_buffer[8]) << 8) + ((uint16_t) ui8_rx_buffer[7]);

      // now send data back
      // ADC 10 bits battery voltage
      ui16_temp = motor_get_adc_battery_voltage_filtered_10b();
      ui8_tx_buffer[3] = (ui16_temp & 0xff);
      ui8_tx_buffer[4] = ((uint8_t) (ui16_temp >> 4)) & 0x30;

      // battery current
      // ADC 10 bits each step current is 0.156
      // 0.156 * 5 = 0.78
      // send battery_current_x5
      ui8_tx_buffer[5] = (uint8_t) ((ui16_g_adc_battery_current_filtered * 78) / 100);

      // wheel speed
      ui8_tx_buffer[6] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
      ui8_tx_buffer[7] = (uint8_t) (ui16_wheel_speed_x10 >> 8);

      // brake state
      ui8_tx_buffer[8] = 0;
      if(motor_controller_state_is_set(MOTOR_CONTROLLER_STATE_BRAKE))
      {
        ui8_tx_buffer[8] |= 1;
      }
      // add the hall sensors state, that should be 3 bits only, value from 0 to 7
      ui8_tx_buffer[8] |= (ui8_g_hall_sensors_state << 1);
      // add pas pedal position
      ui8_tx_buffer[8] |= (ui8_pas_pedal_position_right << 4);

      // throttle value from ADC
      ui8_tx_buffer[9] = UI8_ADC_THROTTLE;

      // adjusted throttle value or temperature limit depending on user setup
      if(m_config_vars.ui8_temperature_limit_feature_enabled == 1)
      {
        // temperature value
        ui8_tx_buffer[10] = m_config_vars.ui8_motor_temperature;
      }
      else
      {
        // throttle value with offset removed and mapped to 255
        ui8_tx_buffer[10] = ui8_throttle;
      }

      // ADC torque_sensor
      ui8_tx_buffer[11] = (uint8_t) (ui16_m_adc_torque_sensor_raw & 0xff);
      // ADC torque_sensor (higher bits), this bits are shared with wheel speed bits
      ui8_tx_buffer[7] |= (uint8_t) ((ui16_m_adc_torque_sensor_raw & 0x300) >> 2); //xx00 0000

      // weight in kgs with offset
      ui8_tx_buffer[12] = (uint8_t) (ui16_m_torque_sensor_weight_raw_with_offset_x10 / 10);

      // weight in kgs
      ui8_tx_buffer[13] = (uint8_t) (ui16_m_torque_sensor_weight_raw_x10 / 10);

      // PAS cadence
      ui8_tx_buffer[14] = ui8_pas_cadence_rpm;

      // PWM duty_cycle
      ui8_tx_buffer[15] = ui8_g_duty_cycle;

      // motor speed in ERPS
      ui16_temp = ui16_motor_get_motor_speed_erps();
      ui8_tx_buffer[16] = (uint8_t) (ui16_temp & 0xff);
      ui8_tx_buffer[17] = (uint8_t) (ui16_temp >> 8);

      // FOC angle
      ui8_tx_buffer[18] = ui8_g_foc_angle;

      // system state
      ui8_tx_buffer[19] = ui8_m_system_state;

      // motor current
      // ADC 10 bits each step current is 0.156
      // 0.156 * 5 = 0.78
      // send battery_current_x5
      ui8_tx_buffer[20] = (uint8_t) ((ui16_g_adc_motor_current_filtered * 78) / 100);

      // wheel_speed_sensor_tick_counter
      ui8_tx_buffer[21] = (uint8_t) (ui32_wheel_speed_sensor_tick_counter & 0xff);
      ui8_tx_buffer[22] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 8) & 0xff);
      ui8_tx_buffer[23] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 16) & 0xff);

      // ui16_pedal_power_x10
      //  ui8_tx_buffer[24] = (uint8_t) (ui16_pedal_power_x10 & 0xff);
      //  ui8_tx_buffer[25] = (uint8_t) (ui16_pedal_power_x10 >> 8);
      ui8_tx_buffer[24] = (uint8_t) (ui16_m_pedal_power_max_x10 & 0xff);
      ui8_tx_buffer[25] = (uint8_t) (ui16_m_pedal_power_max_x10 >> 8);

      ui8_len += 23;
      break;

    // set configurations
    case COMM_FRAME_TYPE_CONFIGURATIONS:
      // disable the motor to avoid a quick of the motor while configurations are changed
      // disable the motor, lets hope this is safe to do here, in this way
      // the motor shold be enabled again on the ebike_control_motor()
      motor_disable_pwm();
      ui8_m_motor_enabled = 0;
      ui8_m_system_state |= ERROR_NOT_INIT;
      ui8_m_motor_init_state = MOTOR_INIT_STATE_INIT_START_DELAY;
      ui8_m_motor_init_status = MOTOR_INIT_STATUS_GOT_CONFIG;

      // battery low voltage cut-off
      m_config_vars.ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer[4]) << 8) + ((uint16_t) ui8_rx_buffer[3]);

      // calc the value in ADC steps and set it up
      ui32_temp = ((uint32_t) m_config_vars.ui16_battery_low_voltage_cut_off_x10 << 8) / ((uint32_t) ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256);
      ui32_temp /= 10;
      motor_set_adc_battery_voltage_cut_off ((uint8_t) ui32_temp);

      // wheel perimeter
      m_config_vars.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer[6]) << 8) + ((uint16_t) ui8_rx_buffer[5]);

      // wheel max speed
      m_config_vars.ui8_wheel_max_speed = ui8_rx_buffer[7];

      // battery max current
      ebike_app_set_battery_max_current(ui8_rx_buffer[8]);

      m_config_vars.ui8_startup_motor_power_boost_feature_enabled = ui8_rx_buffer[9] & 1;
      m_config_vars.ui8_startup_motor_power_boost_always = (ui8_rx_buffer[9] & 2) >> 1;
      m_config_vars.ui8_startup_motor_power_boost_limit_to_max_power = (ui8_rx_buffer[9] & 4) >> 2;
      m_config_vars.ui8_torque_sensor_calibration_feature_enabled = (ui8_rx_buffer[9] & 8) >> 3;
      m_config_vars.ui8_torque_sensor_calibration_pedal_ground = (ui8_rx_buffer[9] & 16) >> 4;
      m_config_vars.ui8_motor_assistance_startup_without_pedal_rotation = (ui8_rx_buffer[9] & 32) >> 5;
      m_config_vars.ui8_motor_type = (ui8_rx_buffer[9] >> 6) & 0x3;

      // motor max current
      ebike_app_set_motor_max_current(ui8_rx_buffer[10]);

      // startup motor power boost time
      m_config_vars.ui8_startup_motor_power_boost_time = ui8_rx_buffer[11];
      // startup motor power boost fade time
      m_config_vars.ui8_startup_motor_power_boost_fade_time = ui8_rx_buffer[12];

      // motor over temperature min value limit
      m_config_vars.ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer[13];
      // motor over temperature max value limit
      m_config_vars.ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer[14];
      // ramp up, amps per second
      m_config_vars.ui8_ramp_up_amps_per_second_x10 = ui8_rx_buffer[15];

      // check that value seems correct
      if (m_config_vars.ui8_ramp_up_amps_per_second_x10 < 4 || m_config_vars.ui8_ramp_up_amps_per_second_x10 > 100)
      {
       // value is not valid, set to default
       m_config_vars.ui8_ramp_up_amps_per_second_x10 = DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10;
      }

      // calculate current step for ramp up
      ui32_temp = ((uint32_t) 24375) / ((uint32_t) m_config_vars.ui8_ramp_up_amps_per_second_x10); // see note below
      ui16_g_current_ramp_up_inverse_step = (uint16_t) ui32_temp;

      /*---------------------------------------------------------
      NOTE: regarding ramp up

      Example of calculation:

      Target ramp up: 5 amps per second

      Every second has 15625 PWM cycles interrupts,
      one ADC battery current step --> 0.156 amps:

      5 / 0.156 = 32 (we need to do 32 steps ramp up per second)

      Therefore:

      15625 / 32 = 488

      15625 * 0.156 = 2437.5; 2437.5 * 10 = 24375
      ---------------------------------------------------------*/

      // received target speed for cruise
      ui16_received_target_wheel_speed_x10 = (uint16_t) (ui8_rx_buffer[16] * 10);

      // motor temperature limit function or throttle
      m_config_vars.ui8_temperature_limit_feature_enabled = ui8_rx_buffer[17];

      // torque sensor calibration tables
      j = 18;
      for (i = 0; i < 8; i++) {
        ui16_torque_sensor_linearize_left[i][0] = (uint16_t) ui8_rx_buffer[j++];
        ui16_torque_sensor_linearize_left[i][0] |= ((uint16_t) ui8_rx_buffer[j++]) << 8;
        ui16_torque_sensor_linearize_left[i][1] = (uint16_t) ui8_rx_buffer[j++];
        ui16_torque_sensor_linearize_left[i][1] |= ((uint16_t) ui8_rx_buffer[j++]) << 8;
      }

      for (i = 0; i < 8; i++) {
        ui16_torque_sensor_linearize_right[i][0] = (uint16_t) ui8_rx_buffer[j++];
        ui16_torque_sensor_linearize_right[i][0] |= ((uint16_t) ui8_rx_buffer[j++]) << 8;
        ui16_torque_sensor_linearize_right[i][1] = (uint16_t) ui8_rx_buffer[j++];
        ui16_torque_sensor_linearize_right[i][1] |= ((uint16_t) ui8_rx_buffer[j++]) << 8;
      }

      // battery current min ADC
      m_config_vars.ui8_battery_current_min_adc = ui8_rx_buffer[81];
      break;

    // firmware version
    case COMM_FRAME_TYPE_FIRMWARE_VERSION:
      ui8_tx_buffer[3] = ui8_m_system_state;
      ui8_tx_buffer[4] = 0;
      ui8_tx_buffer[5] = 56;
      ui8_tx_buffer[6] = 0;
      ui8_len += 4;
      break;

    case COMM_FRAME_TYPE_ALIVE:
      // nothing to add
      break;

    case COMM_FRAME_TYPE_STATUS:
      ui8_tx_buffer[3] = ui8_m_motor_init_status;
      ui8_len += 1;
      break;

    default:
      break;
  }

  ui8_tx_buffer[1] = ui8_len;

  // prepare crc of the package
  ui16_crc_tx = 0xffff;
  for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
  {
    crc16(ui8_tx_buffer[ui8_i], &ui16_crc_tx);
  }
  ui8_tx_buffer[ui8_len] = (uint8_t) (ui16_crc_tx & 0xff);
  ui8_tx_buffer[ui8_len + 1] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

  // send the full package to UART
  for (ui8_i = 0; ui8_i < (ui8_len + 2); ui8_i++)
  {
    putchar(ui8_tx_buffer[ui8_i]);
  }

  // get ready to get next package
  ui8_received_package_flag = 0;
}

// each 1 unit = 0.156 amps
static void ebike_app_set_target_adc_battery_max_current(uint16_t ui16_value)
{
  // limit max
  if (ui16_value > ui16_m_adc_battery_current_max)
  {
    ui16_value = ui16_m_adc_battery_current_max;
  }

  ui16_g_adc_target_battery_max_current = ui16_g_adc_current_offset + ui16_value;
}

static void ebike_app_set_target_adc_motor_max_current(uint16_t ui16_value)
{
  // limit max
  if (ui16_value > ui16_m_adc_motor_current_max)
  {
    ui16_value = ui16_m_adc_motor_current_max;
  }

  ui16_g_adc_target_motor_max_current = ui16_g_adc_current_offset + ui16_value;
}

// in amps
static void ebike_app_set_battery_max_current(uint8_t ui8_value)
{
  // each 1 unit = 0.156 amps (0.156 * 256 = 40)
  ui16_m_adc_battery_current_max = ((((uint16_t) ui8_value) << 8) / 40);

  if (ui16_m_adc_battery_current_max > ADC_BATTERY_CURRENT_MAX)
  {
    ui16_m_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX;
  }
}

// in amps
static void ebike_app_set_motor_max_current(uint8_t ui8_value)
{
  // each 1 unit = 0.156 amps (0.156 * 256 = 40)
  ui16_m_adc_motor_current_max = ((((uint16_t) ui8_value) << 8) / 40);

  if (ui16_m_adc_motor_current_max > ADC_MOTOR_CURRENT_MAX)
  {
    ui16_m_adc_motor_current_max = ADC_MOTOR_CURRENT_MAX;
  }
}

static void linearize_torque_sensor_to_kgs(uint16_t *ui16_adc_steps, uint16_t *ui16_weight_x10, uint8_t *ui8_pedal_right)
{
  uint16_t ui16_array_sum[TORQUE_SENSOR_LINEARIZE_NR_POINTS];
  uint8_t ui8_i;
  uint16_t ui16_adc_absolute_steps;
  uint16_t (*ui16_array_linear)[2];
  uint32_t ui32_temp = 0;

  memset(ui16_array_sum, 0, sizeof(ui16_array_sum));

  if(*ui8_pedal_right)
    ui16_array_linear = ui16_torque_sensor_linearize_right;
  else
    ui16_array_linear = ui16_torque_sensor_linearize_left;

  if(*ui16_adc_steps > 0)
  {
    ui16_adc_absolute_steps = *ui16_adc_steps + ui16_g_adc_torque_sensor_min_value;

    for(ui8_i = 0; ui8_i < (TORQUE_SENSOR_LINEARIZE_NR_POINTS - 1); ui8_i++)
    {
      // current value is under interval max value
      if(ui16_adc_absolute_steps < ui16_array_linear[ui8_i + 1][0])
      {
        // first interval
        if(ui8_i == 0)
        {
          ui16_array_sum[ui8_i] = *ui16_adc_steps;
        }
        else
        {
          ui16_array_sum[ui8_i] = ui16_adc_absolute_steps - ui16_array_linear[ui8_i][0];
        }

        // exit the for loop as this was the last interval
        break;
      }
      // current value is over current interval
      else
      {
        ui16_array_sum[ui8_i] = ui16_array_linear[ui8_i + 1][0] - ui16_array_linear[ui8_i][0];
      }
    }

    // count values under min value of array linear
    if (ui16_g_adc_torque_sensor_min_value < ui16_array_linear[0][0])
      ui16_array_sum[0] += (ui16_array_linear[0][0] - ui16_g_adc_torque_sensor_min_value);

    // count with the values over max value of array linear
    if (ui16_adc_absolute_steps > ui16_array_linear[7][0])
      ui16_array_sum[7] = ui16_adc_absolute_steps - ui16_array_linear[7][0];

    // sum the total parcels
    for(ui8_i = 0; ui8_i < TORQUE_SENSOR_LINEARIZE_NR_POINTS - 1; ui8_i++)
    {
      ui32_temp += ((uint32_t) ui16_array_sum[ui8_i] * (uint32_t) ui16_array_linear[ui8_i + 1][1]);
    }

    // sum the last parcel
    ui32_temp += ((uint32_t) ui16_array_sum[7] * (uint32_t) ui16_array_linear[7][1]);

    *ui16_weight_x10 = (uint16_t) (ui32_temp / 10);
  }
  // no torque_sensor_adc_steps
  else
  {
    *ui16_weight_x10 = 0;
  }
}

static void calc_pedal_force_and_torque(void)
{
  // calculate power on pedals
  // formula for angular velocity in degrees: power  =  force  *  rotations per second  *  2  *  pi
  // formula for angular velocity in degrees: power  =  (force  *  rotations per minute  *  2  *  pi) / 60

  // ui16_pedal_power_x10 = (ui16_pedal_torque_x100 * ui8_pas_cadence_rpm  *  2  *  pi) / (60 * 10)
  // (2 * pi) / (60 * 10) = 0.010466667
  // 1 / 0.010466667 = 96
  // ui16_pedal_power_x10 = (ui16_pedal_torque_x100 * ui8_pas_cadence_rpm) / 96

  // NOTE
  /*
  Users did report that pedal human power is about 2x more.

  @casainho had the idea to evaluate the torque sensor peak signal (measuring peak signal every pedal rotation) as being a sinewave and so the average would be:

  > [Average value = 0.637 × maximum or peak value, Vpk](https://www.electronics-tutorials.ws/accircuits/average-voltage.html)

  For a quick hack, we can just reduce actual value to 0.637.

  */

  // linearize and calculate weight on pedals
  // This here is needed to show the raw value to user, on the display. Otherwise, would be always zero while cadence is 0 / pedals not rotating
  linearize_torque_sensor_to_kgs(&ui16_m_torque_sensor_raw, &ui16_m_torque_sensor_weight_raw_x10, &ui8_pas_pedal_position_right);

  // let´s keep the the raw value with offset to send to display and show to user
  ui16_m_torque_sensor_weight_raw_with_offset_x10 = ui16_m_torque_sensor_weight_raw_x10;

  // let´s save the initial weight offset
  if (ui8_m_first_time_torque_sensor_weight &&
      (ui8_m_system_state == NO_ERROR) &&
      ui16_m_torque_sensor_raw) {
    ui8_m_first_time_torque_sensor_weight = 0;
    ui16_m_torque_sensor_weight_offset_x10 = ui16_m_torque_sensor_weight_raw_x10;
  }

  // linearize and calculate weight on pedals
  if (m_config_vars.ui8_torque_sensor_calibration_feature_enabled) {
    linearize_torque_sensor_to_kgs(&ui16_m_torque_sensor_adc_steps, &ui16_m_torque_sensor_weight_x10, &ui8_pas_pedal_position_right);

    // remove the weight offset
    if (ui8_m_first_time_torque_sensor_weight == 0) {
      if (ui16_m_torque_sensor_weight_x10 > ui16_m_torque_sensor_weight_offset_x10)
        ui16_m_torque_sensor_weight_x10 -= ui16_m_torque_sensor_weight_offset_x10;
      else
        ui16_m_torque_sensor_weight_x10 = 0;

      if (ui16_m_torque_sensor_weight_raw_x10 > ui16_m_torque_sensor_weight_offset_x10)
        ui16_m_torque_sensor_weight_raw_x10 -= ui16_m_torque_sensor_weight_offset_x10;
      else
        ui16_m_torque_sensor_weight_raw_x10 = 0;
    }

    ui16_m_pedal_torque_x100 = ui16_m_torque_sensor_weight_x10 * (uint16_t) TORQUE_SENSOR_WEIGHT_TO_FORCE_X10;
  } else {
    // calculate torque on pedals
    ui16_m_pedal_torque_x100 = (uint16_t) ui16_m_torque_sensor_adc_steps * (uint16_t) PEDAL_TORQUE_X100;
  }

  ui16_m_pedal_torque_x10 = ui16_m_pedal_torque_x100 / 10;
  ui16_m_pedal_power_x10 = (uint16_t) (((uint32_t) ui16_m_pedal_torque_x100 * (uint32_t) ui8_pas_cadence_rpm) / (uint32_t) 96);
  ui16_m_pedal_power_max_x10 = ui16_m_pedal_power_x10;
}


static void calc_wheel_speed(void)
{
  // calc wheel speed in km/h
  if (ui16_wheel_speed_sensor_pwm_cycles_ticks < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS)
  {
    f_wheel_speed_x10 = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
    f_wheel_speed_x10 *= m_config_vars.ui16_wheel_perimeter; // millimeters per second
    f_wheel_speed_x10 *= 0.036; // ((3600 / (1000 * 1000)) * 10) kms per hour * 10
    ui16_wheel_speed_x10 = (uint16_t) f_wheel_speed_x10;
  }
  else
  {
    ui16_wheel_speed_x10 = 0;
  }
}


static void calc_motor_temperature(void)
{
  uint16_t ui16_adc_motor_temperatured_filtered_10b;

  // low pass filter to avoid possible fast spikes/noise
  ui16_m_adc_motor_temperatured_accumulated -= ui16_m_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;
  ui16_m_adc_motor_temperatured_accumulated += UI16_ADC_10_BIT_THROTTLE;
  ui16_adc_motor_temperatured_filtered_10b = ui16_m_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;

  m_config_vars.ui16_motor_temperature_x2 = (uint16_t) ((float) ui16_adc_motor_temperatured_filtered_10b / 1.024);
  m_config_vars.ui8_motor_temperature = (uint8_t) (m_config_vars.ui16_motor_temperature_x2 >> 1);
}


static uint16_t calc_filtered_battery_voltage(void)
{
  uint16_t ui16_batt_voltage_filtered = (uint16_t) motor_get_adc_battery_voltage_filtered_10b() * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  return (ui16_batt_voltage_filtered >> 9);
}


static void apply_speed_limit(uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint16_t *ui16_target_current)
{
  *ui16_target_current = (uint16_t) (map((uint32_t) ui16_speed_x10,
                                        (uint32_t) ((ui8_max_speed * 10) - 20),
                                        (uint32_t) ((ui8_max_speed * 10) + 20),
                                        (uint32_t) *ui16_target_current,
                                        (uint32_t) 0));
}


static void apply_throttle(uint8_t ui8_throttle_value, uint16_t *ui16_target_current)
{
  // apply throttle if it is enabled and the motor temperature limit function is not enabled instead
  if (m_config_vars.ui8_temperature_limit_feature_enabled == 2)
  {
    // overide ui8_adc_battery_current_max with throttle value only when user is using throttle
    if (ui8_throttle_value)
    {
      uint16_t ui16_temp = (uint16_t) (map((uint32_t) ui8_throttle_value,
                                         (uint32_t) 0,
                                         (uint32_t) 255,
                                         (uint32_t) 0,
                                         (uint32_t) ui16_m_adc_motor_current_max));

      // set target current
      *ui16_target_current = ui16_temp;
    }
  }
}


static void apply_temperature_limiting(uint16_t *ui16_target_current)
{
  // apply motor temperature protection if it is enabled and the throttle function is not enabled instead
  if (m_config_vars.ui8_temperature_limit_feature_enabled == 1)
  {
    // min temperature value can't be equal or higher than max temperature value...
    if (m_config_vars.ui8_motor_temperature_min_value_to_limit >= m_config_vars.ui8_motor_temperature_max_value_to_limit)
    {
      *ui16_target_current = 0;
    }
    else
    {
      // reduce motor current if over temperature
      *ui16_target_current = 
        (uint16_t) (map((uint32_t) m_config_vars.ui16_motor_temperature_x2,
                        (uint32_t) (((uint16_t) m_config_vars.ui8_motor_temperature_min_value_to_limit) << 1),
                        (uint32_t) (((uint16_t) m_config_vars.ui8_motor_temperature_max_value_to_limit) << 1),
                        (uint32_t) *ui16_target_current,
                        (uint32_t) 0));
    }
  }
}

static void apply_walk_assist(uint16_t *ui16_p_adc_target_current)
{
  // use max current as the limit of current
  *ui16_p_adc_target_current = ui16_m_adc_motor_current_max;

  // check so that walk assist level factor is not too large (too powerful), if it is -> limit the value
  if(m_config_vars.ui16_assist_level_factor_x1000 > 100)
  {
    // limit and set walk assist PWM to some safe value, not too powerful 
    ui8_m_walk_assist_target_duty_cycle = 100;
  }
  else
  {
    // set walk assist PWM to the target duty cycle from user defined value on display (walk assist level factor)
    ui8_m_walk_assist_target_duty_cycle = (uint8_t) m_config_vars.ui16_assist_level_factor_x1000;
  }
}

static void apply_cruise(uint16_t *ui16_target_current)
{
  static int16_t i16_error;
  static int16_t i16_last_error;
  static int16_t i16_integral;
  static int16_t i16_derivative;
  static int16_t i16_control_output;
  
  // set target current to max current
  *ui16_target_current = ui16_m_adc_motor_current_max;
  
  // initialize cruise PID controller
  if (ui8_initialize_cruise_PID)
  {
    // reset flag to save current speed to maintain (for cruise function)
    ui8_initialize_cruise_PID = 0;
    
    // reset PID variables
    i16_error = 0;          // error should be 0 when cruise function starts
    i16_last_error = 0;     // last error should be 0 when cruise function starts 
    i16_integral = 250;     // integral can start at around 250 when cruise function starts ( 250 = around 64 target PWM = around 8 km/h depending on gear and bike )
    i16_derivative = 0;     // derivative should be 0 when cruise function starts 
    i16_control_output = 0; // control signal/output should be 0 when cruise function starts
    
    // check what target wheel speed to use (received or current)
    if (ui16_received_target_wheel_speed_x10 > 0)
    {
      // set received target wheel speed to target wheel speed
      ui16_target_wheel_speed_x10 = ui16_received_target_wheel_speed_x10;
    }
    else
    {
      // set current wheel speed to maintain
      ui16_target_wheel_speed_x10 = ui16_wheel_speed_x10;
    }
  }
  
  // calculate error
  i16_error = (ui16_target_wheel_speed_x10 - ui16_wheel_speed_x10);
  
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
  
  // map the control output to an appropriate target PWM value
  ui8_cruise_target_PWM = (uint8_t) (map((uint32_t) (i16_control_output),
                                          (uint32_t) 0,     // minimum control output from PID
                                          (uint32_t) 1000,  // maximum control output from PID
                                          (uint32_t) 0,     // minimum target PWM
                                          (uint32_t) 255)); // maximum target PWM
}


static void boost_run_statemachine(void)
{
// usually 10 units is between 2 and 4 kgs on the pedals, depending a lot from each torque sensor
#define TORQUE_SENSOR_ADC_STEPS_BOOST_THRESHOLD 10

  if(m_config_vars.ui8_startup_motor_power_boost_time > 0)
  {
    switch(ui8_m_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
        if(ui16_m_torque_sensor_adc_steps > TORQUE_SENSOR_ADC_STEPS_BOOST_THRESHOLD &&
            (ui8_m_brake_is_set == 0))
        {
          ui8_startup_boost_enable = 1;
          ui8_startup_boost_timer = m_config_vars.ui8_startup_motor_power_boost_time;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST;
        }
      break;

      case BOOST_STATE_BOOST:
        // braking means reseting
        if(ui8_m_brake_is_set)
        {
          ui8_startup_boost_enable = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        // end boost if
        if(ui16_m_torque_sensor_adc_steps < TORQUE_SENSOR_ADC_STEPS_BOOST_THRESHOLD)
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
          ui8_startup_boost_fade_steps = m_config_vars.ui8_startup_motor_power_boost_fade_time;
          ui16_startup_boost_fade_variable_x256 = ((uint16_t) ui16_m_adc_target_current << 8);
          ui16_startup_boost_fade_variable_step_amount_x256 = (ui16_startup_boost_fade_variable_x256 / ((uint16_t) ui8_startup_boost_fade_steps));
          ui8_startup_boost_fade_enable = 1;
        }
      break;

      case BOOST_STATE_FADE:
        // braking means reseting
        if(ui8_m_brake_is_set)
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
        }

        if(ui8_startup_boost_fade_steps > 0) { ui8_startup_boost_fade_steps--; }

        // disable fade if
        if(ui16_m_torque_sensor_adc_steps < TORQUE_SENSOR_ADC_STEPS_BOOST_THRESHOLD ||
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
        if((m_config_vars.ui8_startup_motor_power_boost_always & 1) == 0)
        {
          if(ui16_wheel_speed_x10 == 0 &&
              ui16_m_torque_sensor_adc_steps < TORQUE_SENSOR_ADC_STEPS_BOOST_THRESHOLD)
          {
            ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        // torque sensor must be 0
        if((m_config_vars.ui8_startup_motor_power_boost_always & 1) > 0)
        {
          if(ui16_m_torque_sensor_adc_steps < TORQUE_SENSOR_ADC_STEPS_BOOST_THRESHOLD ||
              ui8_pas_cadence_rpm == 0)
          {
            ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
      break;

      default:
      break;
    }
  }
}


static uint8_t apply_boost(uint8_t ui8_pas_cadence, uint16_t ui16_max_current_boost_state, uint16_t *ui16_target_current)
{
  uint8_t ui8_boost_enable = ui8_startup_boost_enable && // if we are currently on BOOST state
      m_config_vars.ui16_assist_level_factor_x1000 &&
      ui8_pas_cadence > 0 ? 1 : 0;

  if (ui8_boost_enable)
  {
    *ui16_target_current = ui16_max_current_boost_state;
  }

  return ui8_boost_enable;
}


static void apply_boost_fade_out(uint16_t *ui16_adc_target_current)
{
  if (ui8_startup_boost_fade_enable)
  {
    // here we try to converge to the regular value, ramping down or up step by step
    uint16_t ui16_adc_target_current_x256 = ((uint16_t) ui16_m_adc_target_current) << 8;
    if (ui16_startup_boost_fade_variable_x256 > ui16_adc_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 -= ui16_startup_boost_fade_variable_step_amount_x256;
    }
    else if (ui16_startup_boost_fade_variable_x256 < ui16_adc_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 += ui16_startup_boost_fade_variable_step_amount_x256;
    }

    *ui16_adc_target_current = ui16_startup_boost_fade_variable_x256 >> 8;
  }
}


static void read_pas_cadence(void)
{
  // cadence in RPM = 60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
  if((ui16_pas_pwm_cycles_ticks >= ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS)) ||
      (ui8_g_pedaling_direction != 1)) // if not rotating pedals forward
  { 
    ui8_pas_cadence_rpm = 0; 
  }
  else
  {
    ui8_pas_cadence_rpm = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) PAS_NUMBER_MAGNETS) * 0.000064));
  }

  if (m_config_vars.ui8_torque_sensor_calibration_pedal_ground)
    ui8_pas_pedal_position_right = ui8_g_pas_pedal_right ? 1: 0;
  else
    ui8_pas_pedal_position_right = ui8_g_pas_pedal_right ? 0: 1;
}


static void torque_sensor_read(void)
{
  uint16_t ui16_adc_torque_sensor = ui16_m_adc_torque_sensor_raw = UI16_ADC_10_BIT_TORQUE_SENSOR;

  // remove the offset
  // make sure readed value is higher than the offset
  if(ui16_adc_torque_sensor >= ui16_g_adc_torque_sensor_min_value)
  {
    ui16_m_torque_sensor_raw = ui16_adc_torque_sensor - ui16_g_adc_torque_sensor_min_value;
  }
  // offset is higher, something is wrong so just keep ui8_torque_sensor_raw at 0 value
  else
  {
    ui16_m_torque_sensor_raw = 0;
  }

  // next state machine is used to filter out the torque sensor signal
  // when user is resting on the pedals
  switch(ui8_tstr_state_machine)
  {
    // ebike is stopped
    case STATE_NO_PEDALLING:
    if(ui16_m_torque_sensor_raw > 0 && ui16_wheel_speed_x10)
    {
      ui8_tstr_state_machine = STATE_PEDALLING;
    }
    break;

    // wait on this state and reset when ebike stops
    case STATE_PEDALLING:
    if(ui16_wheel_speed_x10 == 0 && ui16_m_torque_sensor_raw == 0)
    {
      ui8_tstr_state_machine = STATE_NO_PEDALLING;
    }
    break;

    default:
    break;
  }

  // bike is moving but user doesn't pedal, disable torque sensor signal because user can be resting the feet on the pedals
  if(ui8_tstr_state_machine == STATE_PEDALLING && ui8_pas_cadence_rpm == 0)
  {
    ui16_m_torque_sensor_adc_steps = 0;
  }
  else
  {
    ui16_m_torque_sensor_adc_steps = ui16_m_torque_sensor_raw;
  }
}


static void throttle_read (void)
{
  // map value from 0 up to 255
  ui8_throttle =  (uint8_t) (map (UI8_ADC_THROTTLE,
                                 (uint8_t) ADC_THROTTLE_MIN_VALUE,
                                 (uint8_t) ADC_THROTTLE_MAX_VALUE,
                                 (uint8_t) 0,
                                 (uint8_t) 255));
}


// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    if (ui8_received_package_flag == 0) // only when package were previously processed
    {
      ui8_byte_received = UART2_ReceiveData8();

      switch (ui8_state_machine)
      {
        case 0:
        if (ui8_byte_received == 0x59) { // see if we get start package byte
          ui8_rx_buffer[0] = ui8_byte_received;
          ui8_state_machine = 1;
        }
        else
          ui8_state_machine = 0;
        break;

        case 1:
          ui8_rx_buffer[1] = ui8_byte_received;
          ui8_rx_len = ui8_byte_received;
          ui8_state_machine = 2;
        break;

        case 2:
        ui8_rx_buffer[ui8_rx_cnt + 2] = ui8_byte_received;
        ++ui8_rx_cnt;

        if (ui8_rx_cnt >= ui8_rx_len)
        {
          ui8_rx_cnt = 0;
          ui8_state_machine = 0;
          ui8_received_package_flag = 1; // signal that we have a full package to be processed
        }
        break;

        default:
        break;
      }
    }
  }
  else // if there was any error, restart our state machine
  {
    ui8_rx_cnt = 0;
    ui8_state_machine = 0;
  }
}


struct_config_vars* get_configuration_variables (void)
{
  return &m_config_vars;
}


void check_system()
{
  #define MOTOR_BLOCKED_COUNTER_THRESHOLD             30    // 30  =>  3 seconds
  #define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5  8     // 8  =>  (8 * 0.826) / 5 = 1.3216 ampere  =>  (X) units = ((X * 0.826) / 5) ampere
  #define MOTOR_BLOCKED_ERPS_THRESHOLD                10    // 10 ERPS
  #define MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD       100   // 100  =>  10 seconds
  
  static uint8_t ui8_motor_blocked_counter;
  static uint8_t ui8_motor_blocked_reset_counter;

  // if the motor blocked error is enabled start resetting it
  if (ui8_m_system_state & ERROR_MOTOR_BLOCKED)
  {
    // increment motor blocked reset counter with 100 milliseconds
    ui8_motor_blocked_reset_counter++;
    
    // check if the counter has counted to the set threshold for reset
    if (ui8_motor_blocked_reset_counter > MOTOR_BLOCKED_RESET_COUNTER_THRESHOLD)
    {
      // reset motor blocked error code
      ui8_m_system_state &= ~ERROR_MOTOR_BLOCKED;
      
      // reset the counter that clears the motor blocked error
      ui8_motor_blocked_reset_counter = 0;
    }
  }
  else
  {
    // if battery current (x5) is over the current threshold (x5) and the motor ERPS is below threshold start setting motor blocked error code
    if ((ui16_g_adc_battery_current_filtered > MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X5) && (ui16_motor_get_motor_speed_erps() < MOTOR_BLOCKED_ERPS_THRESHOLD))
    {
      // increment motor blocked counter with 100 milliseconds
      ui8_motor_blocked_counter++;
      
      // check if motor is blocked for more than some safe threshold
      if (ui8_motor_blocked_counter > MOTOR_BLOCKED_COUNTER_THRESHOLD)
      {
        // set motor blocked error code
        ui8_m_system_state |= ERROR_MOTOR_BLOCKED;
        
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
}
