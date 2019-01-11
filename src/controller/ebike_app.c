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
#include "eeprom.h"
#include "config.h"
#include "utils.h"
#include "lights.h"

#define STATE_NO_PEDALLING                0
#define STATE_STARTUP_PEDALLING           1
#define STATE_PEDALLING                   2

#define BOOST_STATE_BOOST_DISABLED        0
#define BOOST_STATE_START_BOOST           1
#define BOOST_STATE_BOOST                 2
#define BOOST_STATE_END_BOOST             3
#define BOOST_STATE_FADE                  4
#define BOOST_STATE_BOOST_WAIT_TO_RESTART 5

uint8_t ui8_adc_battery_max_current = ADC_BATTERY_CURRENT_MAX;
uint8_t ui8_target_battery_max_power_x10 = ADC_BATTERY_CURRENT_MAX;

volatile uint8_t ui8_throttle = 0;
volatile uint8_t ui8_torque_sensor_value1 = 0;
volatile uint8_t ui8_torque_sensor = 0;
volatile uint8_t ui8_torque_sensor_raw = 0;
volatile uint8_t ui8_adc_torque_sensor_min_value;
volatile uint8_t ui8_adc_torque_sensor_max_value;
volatile uint8_t ui8_adc_battery_current_offset;
volatile uint8_t ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
volatile uint8_t ui8_adc_target_battery_max_current;
uint8_t ui8_adc_battery_current_max;
uint8_t ui8_walk_assist_PWM;


volatile uint16_t ui16_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t ui8_pas_direction = 0;
uint8_t ui8_pas_cadence_rpm = 0;
uint16_t ui16_pedal_torque_x10;
uint16_t ui16_pedal_power_x10;
uint8_t ui8_pedal_human_power = 0;
uint8_t ui8_startup_boost_enable = 0;
uint8_t ui8_startup_boost_fade_enable = 0;
uint8_t ui8_startup_boost_state_machine = 0;
uint8_t ui8_startup_boost_no_torque = 0;
uint8_t ui8_startup_boost_timer = 0;
uint8_t ui8_startup_boost_fade_steps = 0;
uint16_t ui16_startup_boost_fade_variable_x256;
uint16_t ui16_startup_boost_fade_variable_step_amount_x256;


// wheel speed
volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
uint8_t ui8_wheel_speed_max = 0;
float f_wheel_speed_x10;
uint16_t ui16_wheel_speed_x10;
volatile uint32_t ui32_wheel_speed_sensor_tick_counter = 0;

volatile struct_configuration_variables configuration_variables;

// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[11];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[26];
volatile uint8_t ui8_tx_counter = 0;
volatile uint8_t ui8_i;
volatile uint8_t ui8_checksum;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
volatile uint8_t ui8_uart_received_first_package = 0;
static uint16_t ui16_crc_rx;
static uint16_t ui16_crc_tx;
static uint8_t ui8_master_comm_package_id = 0;
static uint8_t ui8_slave_comm_package_id = 0;

uint8_t ui8_tstr_state_machine = STATE_NO_PEDALLING;
uint8_t ui8_rtst_counter = 0;

uint16_t ui16_adc_motor_temperatured_accumulated = 0;

uint8_t ui8_adc_battery_target_current;

// safe tests
uint8_t safe_tests_state_machine = 0;
uint8_t safe_tests_state_machine_counter = 0;

static void ebike_control_motor (void);
static void ebike_app_set_battery_max_current (uint8_t ui8_value);
static void ebike_app_set_target_adc_battery_max_current (uint8_t ui8_value);

static void communications_controller (void);
static void uart_send_package (void);

static void throttle_read (void);
static void read_pas_cadence (void);
static void torque_sensor_read (void);
static void calc_pedal_force_and_torque (void);
static void calc_wheel_speed (void);
static void calc_motor_temperature (void);
static uint16_t calc_filtered_battery_voltage (void);

static void apply_offroad_mode (uint16_t ui16_battery_voltage, uint8_t *ui8_max_speed, uint8_t *ui8_target_current);
static void apply_speed_limit (uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint8_t *ui8_target_current);
static void apply_temperature_limiting (uint8_t *ui8_target_current);
static void apply_walk_assist (uint8_t *ui8_target_current);
//static void apply_cruise (uint8_t *ui8_motor_enable, uint8_t *ui8_target_current);

#if THROTTLE
  static void apply_throttle (uint8_t ui8_throttle_value, uint8_t *ui8_motor_enable, uint8_t *ui8_target_current);
#endif

static void boost_run_statemachine (void);
static uint8_t apply_boost (uint8_t ui8_pas_cadence, uint8_t ui8_max_current_boost_state, uint8_t *ui8_target_current);
static void apply_boost_fade_out (uint8_t *ui8_target_current);

static void safe_tests (void);

void ebike_app_init (void)
{
  // init variables with the stored value on EEPROM
  eeprom_init_variables ();
  ebike_app_set_battery_max_current (ADC_BATTERY_CURRENT_MAX);
}

void ebike_app_controller (void)
{
  throttle_read();
  torque_sensor_read();
  read_pas_cadence();
  calc_pedal_force_and_torque();
  calc_wheel_speed();
  calc_motor_temperature();
  ebike_control_motor();
  communications_controller();
}

static void ebike_control_motor (void)
{
  static uint32_t ui32_temp;
  uint8_t ui8_tmp_pas_cadence_rpm;
  uint32_t ui32_adc_max_battery_current_x4;
  uint8_t ui8_adc_max_battery_current = 0;
  uint8_t ui8_adc_max_battery_power_current = 0;
  uint8_t ui8_startup_enable;
  uint8_t ui8_boost_enabled_and_applied = 0;
  uint8_t ui8_tmp_max_speed;

  uint16_t ui16_battery_voltage_filtered = calc_filtered_battery_voltage ();


  // calc max battery current for boost state
  // calc max battery current for regular state
  // calc max battery current (defined by user on LCD3)
  uint8_t ui8_adc_max_battery_current_boost_state = 0;


  if (ui16_battery_voltage_filtered > 15)
  {
    // 1.6 = 1 / 0.625 (each adc step for current)
    // 25 * 1.6 = 40
    // 40 * 4 = 160
    if (configuration_variables.ui8_startup_motor_power_boost_assist_level > 0)
    {
      ui32_temp = (uint32_t) ui16_pedal_torque_x10 * (uint32_t) configuration_variables.ui8_startup_motor_power_boost_assist_level;
      ui32_temp /= 10;

      // 1.6 = 1 / 0.625 (each adc step for current)
      // 1.6 * 8 = ~13
      ui32_temp = (ui32_temp * 13) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_current_boost_state = ui32_temp >> 3;
      ui8_limit_max(&ui8_adc_max_battery_current_boost_state, 255);
    }

    if (configuration_variables.ui8_assist_level_factor_x10 > 0)
    {
      ui32_temp = (uint32_t) ui16_pedal_power_x10 * (uint32_t) configuration_variables.ui8_assist_level_factor_x10;
      ui32_temp /= 100;

      // 1.6 = 1 / 0.625 (each adc step for current)
      // 1.6 * 8 = ~13
      ui32_temp = (ui32_temp * 13) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_current = ui32_temp >> 3;
      ui8_limit_max(&ui8_adc_max_battery_current, 255);
    }

    if (configuration_variables.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
    {
      ui32_adc_max_battery_current_x4 = (((uint32_t) configuration_variables.ui8_target_battery_max_power_div25) * 160) / ((uint32_t) ui16_battery_voltage_filtered);
      ui8_adc_max_battery_power_current = ui32_adc_max_battery_current_x4 >> 2;
    }
  }


  // start when we press the pedals
  ui8_startup_enable = configuration_variables.ui8_assist_level_factor_x10 && ui8_torque_sensor ? 1 : 0;

  ui8_tmp_pas_cadence_rpm = ui8_pas_cadence_rpm;
  if (configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)
  {
    if (ui8_pas_cadence_rpm < 10) { ui8_tmp_pas_cadence_rpm = 10; }
  }
  else
  {
    if (ui8_pas_cadence_rpm < 10) { ui8_tmp_pas_cadence_rpm = 0; }
  }

  if (configuration_variables.ui8_startup_motor_power_boost_feature_enabled)
  {
    boost_run_statemachine ();  
    ui8_boost_enabled_and_applied = apply_boost (ui8_tmp_pas_cadence_rpm, ui8_adc_max_battery_current_boost_state, &ui8_adc_battery_target_current);
  }

  if (!ui8_boost_enabled_and_applied)
  {
    ui8_adc_battery_target_current = ui8_adc_max_battery_current;
  }


  /* Boost: make transition from boost to regular level */
  if (configuration_variables.ui8_startup_motor_power_boost_feature_enabled)
  {
    apply_boost_fade_out (&ui8_adc_battery_target_current);
  }


#if THROTTLE
  /* Throttle */
  apply_throttle (ui8_throttle, &ui8_startup_enable, &ui8_adc_battery_target_current);
#endif

  
  /* Walk assist / Cruise */
  if (configuration_variables.ui8_walk_assist)
  {
    // enable walk assist or cruise function depending on speed
    if (ui16_wheel_speed_x10 < 60) // if current speed is less than 6.0 km/h (60), enable walk assist
    {
      // enable walk assist
      apply_walk_assist(&ui8_adc_battery_target_current);
    }
    else // if current speed is more than 6.0 km/h (60), enable cruise function
    {
      // enable cruise function
      //apply_cruise(&ui8_startup_enable, &ui8_adc_battery_target_current);
    }
  }
  
  
  // get wheel speed and set in ui8_temp_max_speed
  ui8_tmp_max_speed = configuration_variables.ui8_wheel_max_speed;
  
  
  /* Offroad mode (limit speed if offroad mode is not active) */
  if (configuration_variables.ui8_offroad_feature_enabled) 
  {
    apply_offroad_mode (ui16_battery_voltage_filtered, &ui8_tmp_max_speed, &ui8_adc_battery_target_current);
  }


  /* Speed limit */
  apply_speed_limit (ui16_wheel_speed_x10, ui8_tmp_max_speed, &ui8_adc_battery_target_current);


  /* User configured max power on display */
  if (configuration_variables.ui8_target_battery_max_power_div25 > 0) //TODO: add real feature toggle for max power feature
  {
    // limit the current to max value defined by user on LCD max power, if:
    // - user defined to make that limitation
    // - we are not on boost or fade state
    if ((configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power == 1) || (!((ui8_boost_enabled_and_applied == 1) || (ui8_startup_boost_fade_enable == 1))))
    {
      // now let's limit the target battery current to battery max current (use min value of both)
      ui8_adc_battery_target_current = ui8_min (ui8_adc_battery_target_current, ui8_adc_max_battery_power_current);
    }
  }


  /* Limit current if motor temperature too high and this feature is enabled by the user */
  if (configuration_variables.ui8_temperature_limit_feature_enabled)
  {
    apply_temperature_limiting (&ui8_adc_battery_target_current);
  }
  else
  {
    // TODO: keep ui8_temperature_current_limiting_value = 255 because 255 means no current limiting happening
    // otherwise temperature symbol on display will be blinking
    configuration_variables.ui8_temperature_current_limiting_value = 255;
  }



  // finally set the target battery current to the battery current controller
  ebike_app_set_target_adc_battery_max_current (ui8_adc_battery_target_current);



  // execute some safe tests
  safe_tests ();

  
  /*************************************************************************************************************
  NOTE:
  
  If the battery_target_current == 0 AND configuration_variables.ui8_walk_assist == 1 AND brake is not set AND there are no 
  errors detected in function "safe_tests", set the target duty cycle to walk assist PWM.
  
  If the battery_target_current == 0 AND ui8_startup_enable == 0 AND brake is not set AND there are no 
  errors detected in function "safe_tests", set the target duty cycle to max (255) and the current will be 
  controlled by the battery current controller. Else set the target duty cycle to 0.
  
  *************************************************************************************************************/
  
  if (ui8_adc_battery_target_current && configuration_variables.ui8_walk_assist && (!brake_is_set()) && configuration_variables.ui8_error_states == ERROR_STATE_NO_ERRORS)
  {
    motor_set_pwm_duty_cycle_target (ui8_walk_assist_PWM);
  }
  else if (ui8_adc_battery_target_current && ui8_startup_enable && (!brake_is_set()) && configuration_variables.ui8_error_states == ERROR_STATE_NO_ERRORS)
  {
    motor_set_pwm_duty_cycle_target (255);
  }
  else
  {
    motor_set_pwm_duty_cycle_target (0);
  }
}


static void communications_controller (void)
{
  uint32_t ui32_temp;

#ifndef DEBUG_UART
  if (ui8_received_package_flag)
  {
    // verify crc of the package
    ui16_crc_rx = 0xffff;
    for (ui8_i = 0; ui8_i < 9; ui8_i++)
    {
      crc16 (ui8_rx_buffer[ui8_i], &ui16_crc_rx);
    }

    // see if CRC is ok...
    if (((((uint16_t) ui8_rx_buffer [10]) << 8) + ((uint16_t) ui8_rx_buffer [9])) == ui16_crc_rx)
    {
      ui8_master_comm_package_id = ui8_rx_buffer [1];

      // send a variable for each package sent but first verify if the last one was received otherwise, keep repeating
      // keep cycling so all variables are sent
#define VARIABLE_ID_MAX_NUMBER 5
      if ((ui8_rx_buffer [2]) == ui8_slave_comm_package_id) // last package data ID was receipt, so send the next one
      {
        ui8_slave_comm_package_id = (ui8_slave_comm_package_id + 1) % VARIABLE_ID_MAX_NUMBER;
      }

      // assist level
      configuration_variables.ui8_assist_level_factor_x10 = ui8_rx_buffer [3];
      
      // lights state
      configuration_variables.ui8_lights = (ui8_rx_buffer [4] & (1 << 0)) ? 1: 0;
      
      // set lights
      lights_set_state (configuration_variables.ui8_lights);
      
      // walk assist / cruise function 
      configuration_variables.ui8_walk_assist = (ui8_rx_buffer [4] & (1 << 1)) ? 1: 0;
      
      // offroad mode
      configuration_variables.ui8_offroad_mode = (ui8_rx_buffer [4]) & (1 << 2) ? 1: 0;

      // battery max current
      configuration_variables.ui8_battery_max_current = ui8_rx_buffer [5];
      
      // set max current from battery
      ebike_app_set_battery_max_current (configuration_variables.ui8_battery_max_current);
      
      // target battery max power
      configuration_variables.ui8_target_battery_max_power_div25 = ui8_rx_buffer [6];

      switch (ui8_master_comm_package_id)
      {
        case 0:
          // battery low voltage cut-off
          configuration_variables.ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer [8]) << 8) + ((uint16_t) ui8_rx_buffer [7]);
          
          // calc the value in ADC steps and set it up
          ui32_temp = ((uint32_t) configuration_variables.ui16_battery_low_voltage_cut_off_x10 << 8) / ((uint32_t) ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256);
          ui32_temp /= 10;
          motor_set_adc_battery_voltage_cut_off ((uint8_t) ui32_temp);
        break;

        case 1:
          // wheel perimeter
          configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer [8]) << 8) + ((uint16_t) ui8_rx_buffer [7]);
        break;

        case 2:
          // wheel max speed
          configuration_variables.ui8_wheel_max_speed = ui8_rx_buffer [7];
        break;

        case 3:
          // type of motor (36 volt, 48 volt or some experimental type)
          configuration_variables.ui8_motor_type = (ui8_rx_buffer [7] & 3);
          // motor assistance without pedal rotation enable/disable when startup 
          configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation = (ui8_rx_buffer [7] & 4) >> 2;
          // motor temperature limit function enable/disable
          configuration_variables.ui8_temperature_limit_feature_enabled = (ui8_rx_buffer [7] & 8) >> 3;
          // startup motor boost state
          configuration_variables.ui8_startup_motor_power_boost_state = (ui8_rx_buffer [8] & 1);
          // startup power boost max power limit
          configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power = (ui8_rx_buffer [8] & 2) >> 1;
        break;

        case 4:
          // startup motor power boost
          configuration_variables.ui8_startup_motor_power_boost_assist_level = ui8_rx_buffer [7];
          // startup motor power boost time
          configuration_variables.ui8_startup_motor_power_boost_time = ui8_rx_buffer [8];
        break;

        case 5:
          // startup motor power boost fade time
          configuration_variables.ui8_startup_motor_power_boost_fade_time = ui8_rx_buffer [7];
          configuration_variables.ui8_startup_motor_power_boost_feature_enabled = (ui8_rx_buffer [8] & 1);
        break;

        case 6:
          // motor temperature min and max values to limit
          configuration_variables.ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer [7];
          configuration_variables.ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer [8];
        break;

        case 7:
          // offroad mode configuration
          configuration_variables.ui8_offroad_feature_enabled = (ui8_rx_buffer [7] & 1);
          configuration_variables.ui8_offroad_enabled_on_startup = (ui8_rx_buffer [7] & 2) >> 1;
          // offroad mode speed limit
          configuration_variables.ui8_offroad_speed_limit = ui8_rx_buffer [8];
        break;

        case 8:
          // offroad mode power limit configuration
          configuration_variables.ui8_offroad_power_limit_enabled = (ui8_rx_buffer [7] & 1);
          configuration_variables.ui8_offroad_power_limit_div25 = ui8_rx_buffer [8];
        break;
        
        case 9:
          // current ramp up inverse step
          configuration_variables.ui16_ADC_battery_current_ramp_up_inverse_step = (((uint16_t) ui8_rx_buffer [8]) << 8) + ((uint16_t) ui8_rx_buffer [7]);
        break;

        default:
          // nothing
        break;
      }

      // verify if any configuration_variables did change and if so, save all of them in the EEPROM
      eeprom_write_if_values_changed ();

      // signal that we processed the full package
      ui8_received_package_flag = 0;

      ui8_uart_received_first_package = 1;
    }

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }

  uart_send_package ();
#endif
}

static void uart_send_package (void)
{
  uint16_t ui16_temp;

  // send the data to the LCD
  // start up byte
  ui8_tx_buffer[0] = 0x43;
  ui8_tx_buffer[1] = ui8_master_comm_package_id;
  ui8_tx_buffer[2] = ui8_slave_comm_package_id;

  ui16_temp = motor_get_adc_battery_voltage_filtered_10b ();
  // adc 10 bits battery voltage
  ui8_tx_buffer[3] = (ui16_temp & 0xff);
  ui8_tx_buffer[4] = ((uint8_t) (ui16_temp >> 4)) & 0x30;

  // battery current x5
  ui8_tx_buffer[5] = (uint8_t) ((float) motor_get_adc_battery_current_filtered_10b () * 0.826);

  // wheel speed
  ui8_tx_buffer[6] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
  ui8_tx_buffer[7] = (uint8_t) (ui16_wheel_speed_x10 >> 8);

  // brake state
  if (motor_controller_state_is_set (MOTOR_CONTROLLER_STATE_BRAKE))
  {
    ui8_tx_buffer[8] |= 1;
  }
  else
  {
    ui8_tx_buffer[8] &= ~1;
  }

  if (configuration_variables.ui8_temperature_limit_feature_enabled)
  {
    ui8_tx_buffer[9] = UI8_ADC_THROTTLE;
    ui8_tx_buffer[10] = configuration_variables.ui8_motor_temperature;
  }
  else
  {
    // ADC throttle
    ui8_tx_buffer[9] = UI8_ADC_THROTTLE;
    // throttle value with offset removed and mapped to 255
    ui8_tx_buffer[10] = ui8_throttle;
  }

  // ADC torque_sensor
  ui8_tx_buffer[11] = UI8_ADC_TORQUE_SENSOR;
  // torque sensor value with offset removed and mapped to 255
  ui8_tx_buffer[12] = ui8_torque_sensor;
  // PAS cadence
  ui8_tx_buffer[13] = ui8_pas_cadence_rpm;
  // pedal human power mapped to 255
  ui8_tx_buffer[14] = ui8_pedal_human_power;
  // PWM duty_cycle
  ui8_tx_buffer[15] = ui8_duty_cycle;
  // motor speed in ERPS
  ui16_temp = ui16_motor_get_motor_speed_erps ();
  ui8_tx_buffer[16] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[17] = (uint8_t) (ui16_temp >> 8);
  // FOC angle
  ui8_tx_buffer[18] = ui8_foc_angle;

  switch (ui8_slave_comm_package_id)
  {
    case 0:
      // error states
      ui8_tx_buffer[19] = configuration_variables.ui8_error_states;
    break;

    case 1:
      // temperature actual limiting value
      ui8_tx_buffer[19] = configuration_variables.ui8_temperature_current_limiting_value;
    break;

    case 2:
      // wheel_speed_sensor_tick_counter
      ui8_tx_buffer[19] = (uint8_t) (ui32_wheel_speed_sensor_tick_counter & 0xff);
    break;

    case 3:
      // wheel_speed_sensor_tick_counter
      ui8_tx_buffer[19] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 8) & 0xff);
    break;

    case 4:
      // wheel_speed_sensor_tick_counter
      ui8_tx_buffer[19] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 16) & 0xff);
    break;

    default:
      // keep at 0
      ui8_tx_buffer[19] = 0;
    break;
  }

  // ui16_pedal_torque_x10
  ui8_tx_buffer[20] = (uint8_t) (ui16_pedal_torque_x10 & 0xff);
  ui8_tx_buffer[21] = (uint8_t) (ui16_pedal_torque_x10 >> 8);

  // ui16_pedal_power_x10
  ui8_tx_buffer[22] = (uint8_t) (ui16_pedal_power_x10 & 0xff);
  ui8_tx_buffer[23] = (uint8_t) (ui16_pedal_power_x10 >> 8);

  // prepare crc of the package
  ui16_crc_tx = 0xffff;
  for (ui8_i = 0; ui8_i <= 23; ui8_i++)
  {
    crc16 (ui8_tx_buffer[ui8_i], &ui16_crc_tx);
  }
  ui8_tx_buffer[24] = (uint8_t) (ui16_crc_tx & 0xff);
  ui8_tx_buffer[25] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

  // send the full package to UART
  for (ui8_i = 0; ui8_i <= 25; ui8_i++)
  {
    putchar (ui8_tx_buffer[ui8_i]);
  }
}


// each 1 unit = 0.625 amps
static void ebike_app_set_target_adc_battery_max_current (uint8_t ui8_value)
{
  // limit max number of amps
  if (ui8_value > ui8_adc_battery_current_max)
    ui8_value = ui8_adc_battery_current_max;

  ui8_adc_target_battery_max_current = ui8_adc_battery_current_offset + ui8_value;
}


// in amps
static void ebike_app_set_battery_max_current (uint8_t ui8_value)
{
  // each 1 unit = 0.625 amps (0.625 * 256 = 160)
  ui8_adc_battery_current_max = ((((uint16_t) ui8_value) << 8) / 160);

  if (ui8_adc_battery_current_max > ADC_BATTERY_CURRENT_MAX)
    ui8_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX;
}


static void calc_pedal_force_and_torque(void)
{
  uint16_t ui16_temp;

  // calculate torque on pedals
  ui16_temp = (uint16_t) ui8_torque_sensor * (uint16_t) PEDAL_TORQUE_X100;
  ui16_pedal_torque_x10 = ui16_temp / 10;

  // calculate power on pedals
  // formula for angular velocity in degrees: power  =  force  *  rotations per second  *  2  *  pi
  // formula for angular velocity in degrees: power  =  force  *  rotations per minute  *  2  *  pi / 60
  // (100 * 2 * pi) / 60 = 628
  ui16_pedal_power_x10 = (uint16_t) ((((uint32_t) ui16_temp * (uint32_t) ui8_pas_cadence_rpm)) / 105);
}


static void calc_wheel_speed (void)
{
  // calc wheel speed in km/h
  if (ui16_wheel_speed_sensor_pwm_cycles_ticks < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS)
  {
    f_wheel_speed_x10 = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
    f_wheel_speed_x10 *= configuration_variables.ui16_wheel_perimeter; // millimeters per second
    f_wheel_speed_x10 *= 0.036; // ((3600 / (1000 * 1000)) * 10) kms per hour * 10
    ui16_wheel_speed_x10 = (uint16_t) f_wheel_speed_x10;
  }
  else
  {
    ui16_wheel_speed_x10 = 0;
  }
}


static void calc_motor_temperature (void)
{
  uint16_t ui16_adc_motor_temperatured_filtered_10b;

  // low pass filter to avoid possible fast spikes/noise
  ui16_adc_motor_temperatured_accumulated -= ui16_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;
  ui16_adc_motor_temperatured_accumulated += ui16_adc_read_throttle_10b ();
  ui16_adc_motor_temperatured_filtered_10b = ui16_adc_motor_temperatured_accumulated >> READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT;

  configuration_variables.ui16_motor_temperature_x2 = (uint16_t) ((float) ui16_adc_motor_temperatured_filtered_10b / 1.024);
  configuration_variables.ui8_motor_temperature = (uint8_t) (configuration_variables.ui16_motor_temperature_x2 >> 1);
}


static uint16_t calc_filtered_battery_voltage (void)
{
  uint16_t ui16_batt_voltage_filtered = (uint16_t) motor_get_adc_battery_voltage_filtered_10b () * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  return (ui16_batt_voltage_filtered >> 9);
}


static void apply_offroad_mode (uint16_t ui16_battery_voltage, uint8_t *ui8_max_speed, uint8_t *ui8_target_current)
{
  if (!configuration_variables.ui8_offroad_mode) 
  {
    // limit speed if offroad mode is not active
    *ui8_max_speed = configuration_variables.ui8_offroad_speed_limit;

    if (configuration_variables.ui8_offroad_power_limit_enabled && configuration_variables.ui8_offroad_power_limit_div25 > 0)
    {
      uint8_t ui8_offroad_mode_max_current = (uint8_t) (((((uint32_t) configuration_variables.ui8_offroad_power_limit_div25) * 160) / ((uint32_t) ui16_battery_voltage)) >> 2);
      *ui8_target_current = ui8_min (ui8_offroad_mode_max_current, *ui8_target_current);
    }
  }
}


static void apply_speed_limit (uint16_t ui16_speed_x10, uint8_t ui8_max_speed, uint8_t *ui8_target_current)
{
  *ui8_target_current = (uint8_t) (map ((uint32_t) ui16_speed_x10,
                                        (uint32_t) ((ui8_max_speed * 10) - 20),
                                        (uint32_t) ((ui8_max_speed * 10) + 20),
                                        (uint32_t) *ui8_target_current,
                                        (uint32_t) 0));
}


#if THROTTLE
  static void apply_throttle (uint8_t ui8_throttle_value, uint8_t *ui8_motor_enable, uint8_t *ui8_target_current)
  {
    uint8_t ui8_temp = (uint8_t) (map ((uint32_t) ui8_throttle_value,
                                       (uint32_t) 0,
                                       (uint32_t) 255,
                                       (uint32_t) 0,
                                       (uint32_t) ui8_adc_battery_current_max));
                                       
    // set target current
    *ui8_target_current = ui8_max (*ui8_target_current, ui8_temp);

    // enable motor assistance because user is using throttle
    if (*ui8_target_current) { *ui8_motor_enable = 1; }
  }
#endif


static void apply_temperature_limiting (uint8_t *ui8_target_current)
{
  // min temperature value can't be equal or higher than max temperature value...
  if (configuration_variables.ui8_motor_temperature_min_value_to_limit >= configuration_variables.ui8_motor_temperature_max_value_to_limit)
  {
    *ui8_target_current = 0;
    configuration_variables.ui8_temperature_current_limiting_value = 0;
  }
  else
  {
    // reduce motor current if over temperature
    *ui8_target_current = 
      (uint8_t) (map ((uint32_t) configuration_variables.ui16_motor_temperature_x2,
                      (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                      (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                      (uint32_t) *ui8_target_current,
                      (uint32_t) 0));

    // get a value linear to the current limitation, just to show to user
    configuration_variables.ui8_temperature_current_limiting_value = 
      (uint8_t) (map ((uint32_t) configuration_variables.ui16_motor_temperature_x2,
                      (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                      (uint32_t) (((uint16_t) configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                      (uint32_t) 255,
                      (uint32_t) 0));
  }
}


static void apply_walk_assist (uint8_t *ui8_target_current)
{
  // set target current to max current
  *ui8_target_current = ui8_adc_battery_current_max;
  
  // check so that walk assist level factor is not too large (too powerful), if it is -> limit walk assist PWM
  if (configuration_variables.ui8_assist_level_factor_x10 > 100)
  {
    // limit and set walk assist PWM to some value not too powerful 
    ui8_walk_assist_PWM = 100;
  }
  else
  {
    // set walk assist PWM from user defined value on display (walk assist level factor)
    ui8_walk_assist_PWM = configuration_variables.ui8_assist_level_factor_x10;
  }
}


/* static void apply_cruise (uint8_t *ui8_motor_enable, uint8_t *ui8_target_current)
{
  // save current speed to maintain
  
  // activate cruise flag
  
  
  uint8_t ui8_cruise_power_value = 0;
  
  // map the cruise power value to appropriate target current
  uint8_t ui8_temp = (uint8_t) (map ((uint32_t) ui8_cruise_power_value,
                                     (uint32_t) 0,
                                     (uint32_t) 255,
                                     (uint32_t) 0,
                                     (uint32_t) ui8_adc_battery_current_max));
                                     
  // set target current
  *ui8_target_current = ui8_max (*ui8_target_current, ui8_temp);

  // enable motor assistance because user requests cruise
  if (*ui8_target_current) { *ui8_motor_enable = 1; }
} */


static void boost_run_statemachine (void)
{
  if (configuration_variables.ui8_startup_motor_power_boost_time > 0)
  {
    switch (ui8_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
        if ((ui8_torque_sensor > 0) &&
            (!brake_is_set()))
        {
          ui8_startup_boost_state_machine = BOOST_STATE_START_BOOST;
        }
      break;

      case BOOST_STATE_START_BOOST:
        ui8_startup_boost_enable = 1;
        ui8_startup_boost_timer = configuration_variables.ui8_startup_motor_power_boost_time;
        ui8_startup_boost_state_machine = BOOST_STATE_BOOST;
      break;

      case BOOST_STATE_BOOST:
        // decrement timer
        if (ui8_startup_boost_timer > 0) { ui8_startup_boost_timer--; }

        // disable boost if
        if ((ui8_torque_sensor == 0) ||
            (ui8_startup_boost_timer == 0))
        {
          ui8_startup_boost_state_machine = BOOST_STATE_END_BOOST;
        }
      break;

      case BOOST_STATE_END_BOOST:
        ui8_startup_boost_enable = 0;

        // setup variables for fade
        ui8_startup_boost_fade_steps = configuration_variables.ui8_startup_motor_power_boost_fade_time;
        ui16_startup_boost_fade_variable_x256 = ((uint16_t) ui8_adc_battery_target_current << 8);
        ui16_startup_boost_fade_variable_step_amount_x256 = (ui16_startup_boost_fade_variable_x256 / ((uint16_t) ui8_startup_boost_fade_steps));
        ui8_startup_boost_fade_enable = 1;

        ui8_startup_boost_state_machine = BOOST_STATE_FADE;
      break;

      case BOOST_STATE_FADE:
        if (ui8_startup_boost_fade_steps > 0) { ui8_startup_boost_fade_steps--; }

        // disable fade if
        if ((ui8_torque_sensor_raw == 0) ||
            (ui8_pas_cadence_rpm == 0) ||
            (ui8_startup_boost_fade_steps == 0))
        {
          ui8_startup_boost_fade_enable = 0;
          ui8_startup_boost_fade_steps = 0;
          ui8_startup_boost_state_machine = BOOST_STATE_BOOST_WAIT_TO_RESTART;
        }
      break;

      // restart when user is not pressing the pedals AND/OR wheel speed = 0
      case BOOST_STATE_BOOST_WAIT_TO_RESTART:
        // wheel speed must be 0 as also torque sensor
        if ((configuration_variables.ui8_startup_motor_power_boost_state & 1) == 0)
        {
          if ((ui16_wheel_speed_x10 == 0) && (ui8_torque_sensor_raw == 0))
          {
            ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
        // torque sensor must be 0
        if ((configuration_variables.ui8_startup_motor_power_boost_state & 1) > 0)
        {
          if ((ui8_torque_sensor_raw == 0) || (ui8_pas_cadence_rpm == 0))
          {
            ui8_startup_boost_state_machine = BOOST_STATE_BOOST_DISABLED;
          }
        }
      break;

      default:
      break;
    }
  }
}


static uint8_t apply_boost (uint8_t ui8_pas_cadence, uint8_t ui8_max_current_boost_state, uint8_t *ui8_target_current)
{
  uint8_t ui8_boost_enable = ui8_startup_boost_enable && configuration_variables.ui8_assist_level_factor_x10 && ui8_pas_cadence > 0 ? 1 : 0;

  if (ui8_boost_enable)
  {
    *ui8_target_current = ui8_max_current_boost_state;
  }

  return ui8_boost_enable;
}


static void apply_boost_fade_out (uint8_t *ui8_target_current)
{
  if (ui8_startup_boost_fade_enable)
  {
    // here we try to converge to the regular value, ramping down or up step by step
    uint16_t ui16_adc_battery_target_current_x256 = ((uint16_t) ui8_adc_battery_target_current) << 8;
    if (ui16_startup_boost_fade_variable_x256 > ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 -= ui16_startup_boost_fade_variable_step_amount_x256;
    }
    else if (ui16_startup_boost_fade_variable_x256 < ui16_adc_battery_target_current_x256)
    {
      ui16_startup_boost_fade_variable_x256 += ui16_startup_boost_fade_variable_step_amount_x256;
    }

    *ui8_target_current = (uint8_t) (ui16_startup_boost_fade_variable_x256 >> 8);
  }
}


static void read_pas_cadence (void)
{
  // cadence in RPM =  60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
  if (ui16_pas_pwm_cycles_ticks >= ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS)) 
  { 
    ui8_pas_cadence_rpm = 0; 
  }
  else
  {
    ui8_pas_cadence_rpm = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) PAS_NUMBER_MAGNETS) * 0.000064));
  }
}


static void torque_sensor_read (void)
{
  // map value from 0 up to 255
  // map value from 0 up to 255
  ui8_torque_sensor_raw = (uint8_t) (map (
      UI8_ADC_TORQUE_SENSOR,
      (uint8_t) ui8_adc_torque_sensor_min_value,
      (uint8_t) ui8_adc_torque_sensor_max_value,
      (uint8_t) 0,
      (uint8_t) 255));

  switch (ui8_tstr_state_machine)
  {
    // ebike is stopped, wait for throttle signal
    case STATE_NO_PEDALLING:
    if ((ui8_torque_sensor_raw > 0) &&
        (!brake_is_set()))
    {
      ui8_tstr_state_machine = STATE_STARTUP_PEDALLING;
    }
    break;

    // now count 2 seconds
    case STATE_STARTUP_PEDALLING:
    if (ui8_rtst_counter++ > 20) // 2 seconds
    {
      ui8_rtst_counter = 0;
      ui8_tstr_state_machine = STATE_PEDALLING;
    }

    // ebike is not moving, let's return to begin
    if (ui16_wheel_speed_x10 == 0)
    {
      ui8_rtst_counter = 0;
      ui8_tstr_state_machine = 0;
    }
    break;

    // wait on this state and reset when ebike stops
    case STATE_PEDALLING:
    if ((ui16_wheel_speed_x10 == 0) && (ui8_torque_sensor_raw == 0))
    {
      ui8_tstr_state_machine = STATE_NO_PEDALLING;
    }
    break;

    default:
    break;
  }

  // bike is moving but user doesn't pedal, disable torque sensor signal because user can be resting the feet on the pedals
  if ((ui8_tstr_state_machine == STATE_PEDALLING) && (ui8_pas_cadence_rpm == 0))
  {
    ui8_torque_sensor = 0;
  }
  else
  {
    ui8_torque_sensor = ui8_torque_sensor_raw;
  }
}


static void throttle_read (void)
{
#if THROTTLE
  // map value from 0 up to 255
  ui8_throttle = (uint8_t) (map (
      UI8_ADC_THROTTLE,
      (uint8_t) ADC_THROTTLE_MIN_VALUE,
      (uint8_t) ADC_THROTTLE_MAX_VALUE,
      (uint8_t) 0,
      (uint8_t) 255));
#else
  ui8_throttle = 0;
#endif
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
      ui8_rx_counter++;

      // see if is the last byte of the package
      if (ui8_rx_counter > 12)
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


struct_configuration_variables* get_configuration_variables (void)
{
  return &configuration_variables;
}


static void safe_tests (void)
{
  // enabe only next state machine if user has startup without pedal rotation
  if (configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation)
  {
    // the state machine should restart if:
    if (brake_is_set() || // we hit brakes
        configuration_variables.ui8_assist_level_factor_x10 == 0) // we choose assist power assist level = 0
    {
      configuration_variables.ui8_error_states &= ~ERROR_STATE_EBIKE_WHEEL_BLOCKED; // disable error state in case it was enable
      safe_tests_state_machine = 0;
    }

    switch (safe_tests_state_machine)
    {
      // start when torque sensor or throttle or walk assist
      case 0:
      if (ui8_torque_sensor_raw || ui8_throttle || configuration_variables.ui8_walk_assist)
      {
        safe_tests_state_machine_counter = 0;
        safe_tests_state_machine = 1;
        break;
      }
      break;

      // wait during 3 seconds for bicyle wheel speed > 4km/h, if not we have an error
      case 1:
      safe_tests_state_machine_counter++;

      // timeout of 3 seconds, not less to be higher than value on torque_sensor_read ()
      // 3 seconds should be safe enough value, mosfets should not burn in 3 seconds if ebike wheel is blocked
      if (safe_tests_state_machine_counter > 30)
      {
        configuration_variables.ui8_error_states |= ERROR_STATE_EBIKE_WHEEL_BLOCKED;
        safe_tests_state_machine_counter = 0;
        safe_tests_state_machine = 2;
        break;
      }

      // bicycle wheel is rotating so we are safe
      if (ui16_wheel_speed_x10 > 40) // seems that 4 km/h may be the min value we can measure for the bicycle wheel speed
      {
        safe_tests_state_machine_counter = 0;
        safe_tests_state_machine = 3;
        break;
      }

      // if release of: torque sensor AND throttle AND walk assist -> restart
      if ((ui8_torque_sensor_raw == 0) && (ui8_throttle == 0) && (configuration_variables.ui8_walk_assist == 0))
      {
        safe_tests_state_machine = 0;
      }
      break;

      // wait 3 consecutive seconds for torque sensor and throttle and walk assist == 0, then restart
      case 2:
      if ((ui8_torque_sensor_raw == 0) && (ui8_throttle == 0) && (configuration_variables.ui8_walk_assist == 0))
      {
        safe_tests_state_machine_counter++;

        if (safe_tests_state_machine_counter > 30)
        {
          configuration_variables.ui8_error_states &= ~ERROR_STATE_EBIKE_WHEEL_BLOCKED;
          safe_tests_state_machine = 0;
          break;
        }
      }
      // keep reseting the counter so we keep on this state
      else
      {
        safe_tests_state_machine_counter = 0;
      }
      break;

      // wait for bicycle wheel to be stopped so we can start again our state machine
      case 3:
      if (ui16_wheel_speed_x10 == 0)
      {
        safe_tests_state_machine = 0;
        break;
      }
      break;

      default:
      safe_tests_state_machine = 0;
      break;
    }
  }
  else
  {
    // keep reseting state machine
    configuration_variables.ui8_error_states &= ~ERROR_STATE_EBIKE_WHEEL_BLOCKED; // disable error state in case it was enabled
    safe_tests_state_machine = 0;
  }
}
