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
#include "utils.h"
#include "lights.h"

volatile struct_configuration_variables m_configuration_variables;

// variables for various system functions
static uint8_t    ui8_riding_mode = POWER_ASSIST_MODE;
static uint8_t    ui8_riding_mode_parameter = 0;
static uint8_t    ui8_system_state = NO_ERROR;
static uint8_t    ui8_brakes_enabled = 0;


// variables for power control
static uint16_t   ui16_battery_voltage_filtered_x1000 = 0;
static uint8_t    ui8_battery_current_filtered_x10 = 0;
static uint8_t    ui8_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX;
static uint8_t    ui8_adc_battery_current_target = 0;
static uint8_t    ui8_duty_cycle_target = 0;
static uint8_t    ui8_m_motor_enabled = 1;


// variables for the cadence sensor
volatile uint16_t ui16_pas_pwm_cycles_ticks = PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t  ui8_g_pedaling_direction = 0;
uint8_t           ui8_pedal_cadence_RPM = 0;


// variables for the torque sensor
volatile uint8_t  ui8_adc_pedal_torque = 0;
uint16_t          ui16_pedal_power_x10 = 0;
uint16_t          ui16_pedal_torque_x100 = 0;


// variables for the throttle function
volatile uint8_t  ui8_adc_throttle = 0;


// variables for wheel speed calculation
volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks = WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
volatile uint32_t ui32_wheel_speed_sensor_tick_counter = 0;
static uint16_t   ui16_wheel_speed_x10 = 0;


// variables for boost
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


// UART
#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   7   // change this value depending on how many data bytes there is to receive ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_NUMBER_DATA_BYTES_TO_SEND      24  // change this value depending on how many data bytes there is to send ( Package = one start byte + data bytes + two bytes 16 bit CRC )

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
static void ebike_control_motor(void);
static void check_system(void);
static void check_brakes(void);

static void calc_cadence(void);
static void calc_crank_power(void);
static void calc_wheel_speed(void);
static void get_battery_voltage_filtered(void);
static void get_battery_current_filtered(void);

static void apply_boost();
static void apply_power_assist();
static void apply_torque_assist();
static void apply_emtb();
static void apply_cadence_assist();
static void apply_virtual_throttle();
static void apply_walk_assist();
static void apply_cruise();
static void apply_throttle();
static void apply_speed_limit();
static void apply_temperature_limiting();



void ebike_app_controller (void)
{
  get_battery_voltage_filtered();
  get_battery_current_filtered();
  
  calc_cadence();
  calc_crank_power();
  calc_wheel_speed();
  
  check_system();
  check_brakes();
  
  communications_controller();
  ebike_control_motor();
}



static void ebike_control_motor (void)
{
  // reset control variables
  ui8_adc_battery_current_target = 0;
  ui8_duty_cycle_target = 0;
  
  // boost
  //apply_boost();
  
  // power assist
  apply_power_assist();
  
  // torque assist
  apply_torque_assist();
  
  // eMTB
  apply_emtb();
  
  // walk assist
  apply_walk_assist();
  
  // cruise
  apply_cruise();
  
  // throttle
  apply_throttle();
  
  // speed limit
  apply_speed_limit();
  
  // motor temperature protection
  apply_temperature_limiting();

  // force target current to 0 if brakes are enabled or if there are errors
  if (ui8_brakes_enabled || ui8_system_state != NO_ERROR) { ui8_adc_battery_current_target = 0; }

  // check to see if we should enable the motor
  if (ui8_m_motor_enabled == 0 &&
     (ui16_motor_get_motor_speed_erps() == 0) && // only enable motor if stopped, other way something bad can happen due to high currents/regen or something like that
      ui8_adc_battery_current_target)
  {
    ui8_m_motor_enabled = 1;
    ui8_g_duty_cycle = 0;
    motor_enable_pwm();
  }

  // check to see if we should disable the motor
  if (ui8_m_motor_enabled &&
      ui16_motor_get_motor_speed_erps() == 0 &&
      ui8_adc_battery_current_target == 0 &&
      ui8_g_duty_cycle == 0)
  {
    ui8_m_motor_enabled = 0;
    motor_disable_pwm();
  }

  // set target current and duty cycle
  if (ui8_m_motor_enabled && !ui8_brakes_enabled)
  {
    // limit max current if larger than configured hardware limit
    if (ui8_adc_battery_current_max > ADC_BATTERY_CURRENT_MAX) { ui8_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX; }
    
    // limit target current if larger than max current
    if (ui8_adc_battery_current_target > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
    
    // limit target duty cycle if larger than max duty cycle
    if (ui8_duty_cycle_target > PWM_DUTY_CYCLE_MAX) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    
    // set controller target current with offset from ADC calibration
    ui8_controller_adc_battery_current_target = ui8_adc_battery_current_target + ui8_adc_battery_current_offset;
    
    // set controller target duty cycle
    ui8_controller_duty_cycle_target = ui8_duty_cycle_target;
  }
  else
  {
    ui8_controller_adc_battery_current_target = 0;
    ui8_controller_duty_cycle_target = 0;
    ui8_g_duty_cycle = 0;
  }
}



static void calc_crank_power(void)
{
  // get pedal torque
  ui8_adc_pedal_torque = UI8_ADC_TORQUE_SENSOR;
  
  // calculate torque on pedals
  if (ui8_adc_pedal_torque > ui8_adc_pedal_torque_offset)
  {
    ui16_pedal_torque_x100 = (uint16_t) (ui8_adc_pedal_torque - ui8_adc_pedal_torque_offset) * (uint16_t) PEDAL_TORQUE_PER_8_BIT_ADC_STEP_X100;
  }
  else
  {
    ui16_pedal_torque_x100 = 0;
  }

  // calculate power on crank
  ui16_pedal_power_x10 = (uint16_t) (((uint32_t) ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 105); // see note below

  /*---------------------------------------------------------

    NOTE: regarding the human power calculation
    
    Formula: power  =  force  *  rotations per second  *  2  *  pi
    Formula: power  =  force  *  rotations per minute  *  2  *  pi / 60
    
    (100 * 2 * pi) / 60 ≈ 1.047 -> 105
  ---------------------------------------------------------*/
}



static void apply_power_assist()
{
  if (ui8_riding_mode == POWER_ASSIST_MODE)
  {
    uint8_t ui8_power_assist_multiplier_x10 = ui8_riding_mode_parameter;
    
    // calculate power assist
    uint32_t ui32_power_assist_x100 = (uint32_t) ui16_pedal_power_x10 * ui8_power_assist_multiplier_x10;
    
    // calculate target current
    uint16_t ui16_battery_current_target_x10 = (ui32_power_assist_x100 * 100) / ui16_battery_voltage_filtered_x1000;
    
    // round up and set battery current target in ADC steps
    ui8_adc_battery_current_target = (ui16_battery_current_target_x10 + 5) / BATTERY_CURRENT_PER_8_BIT_ADC_STEP_X10;
    
    // set duty cycle target
    if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    else { ui8_duty_cycle_target = 0; }
  }
}



static void apply_torque_assist()
{
  if (ui8_riding_mode == TORQUE_ASSIST_MODE)
  {
    #define TORQUE_THRESHOLD                    1   // minimum torque to be applied for torque assist operation
    #define TORQUE_ASSIST_FACTOR_DENOMINATOR    50  // scale the torque assist factor
    
    uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;
    uint16_t ui16_adc_battery_current_target_torque_assist = 0;
    
    // calculate torque assist target current
    if ((ui8_adc_pedal_torque) > (ui8_adc_pedal_torque_offset + TORQUE_THRESHOLD))
    {
      ui16_adc_battery_current_target_torque_assist = ((uint16_t) (ui8_adc_pedal_torque - ui8_adc_pedal_torque_offset - TORQUE_THRESHOLD) * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
    }
    
    // set battery current target
    if (ui16_adc_battery_current_target_torque_assist > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
    else { ui8_adc_battery_current_target = ui16_adc_battery_current_target_torque_assist; }

    // set duty cycle target
    if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    else { ui8_duty_cycle_target = 0; }
  }
}



static void apply_emtb()
{
  if (ui8_riding_mode == eMTB_ASSIST_MODE) 
  {
  }
}



static void apply_walk_assist()
{
  if (ui8_riding_mode == WALK_ASSIST_MODE && ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10) 
  {
    #define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP   200
    #define WALK_ASSIST_DUTY_CYCLE_MAX                    80
    
    uint8_t ui8_walk_assist_duty_cycle_target = ui8_riding_mode_parameter;

    // limit acceleration
    ui16_duty_cycle_ramp_up_inverse_step = WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;

    // check so that walk assist level factor is not too large (too powerful), if it is -> limit the value
    if (ui8_walk_assist_duty_cycle_target > WALK_ASSIST_DUTY_CYCLE_MAX) { ui8_walk_assist_duty_cycle_target = WALK_ASSIST_DUTY_CYCLE_MAX; }
    
    // set battery current target
    ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    
    // set duty cycle target
    ui8_duty_cycle_target = ui8_walk_assist_duty_cycle_target;
  }
}



static void apply_cruise()
{
  static uint8_t ui8_cruise_PID_initialized;
  
  if (ui8_riding_mode == CRUISE_MODE && ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10) 
  {
    #define CRUISE_PID_KP                             12    // 48 volt motor: 12, 36 volt motor: 14
    #define CRUISE_PID_KI                             0.7   // 48 volt motor: 1, 36 volt motor: 0.7
    #define CRUISE_PID_INTEGRAL_LIMIT                 1000
    #define CRUISE_PID_KD                             0
    #define CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP    80
    
    static int16_t i16_error;
    static int16_t i16_last_error;
    static int16_t i16_integral;
    static int16_t i16_derivative;
    static int16_t i16_control_output;
    static uint16_t ui16_wheel_speed_target_x10;
    
    // limit acceleration
    ui16_duty_cycle_ramp_up_inverse_step = CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
    
    // initialize cruise PID controller
    if (!ui8_cruise_PID_initialized)
    {
      // reset flag to save current speed to maintain (for cruise function)
      ui8_cruise_PID_initialized = 1;
      
      // reset PID variables
      i16_error = 0;          // error should be 0 when cruise function starts
      i16_last_error = 0;     // last error should be 0 when cruise function starts 
      i16_integral = 250;     // integral can start at around 250 when cruise function starts ( 250 = around 64 target PWM = around 8 km/h depending on gear and bike )
      i16_derivative = 0;     // derivative should be 0 when cruise function starts 
      i16_control_output = 0; // control signal/output should be 0 when cruise function starts
      
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
    
    // set battery current target
    ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    
    // set duty cycle target  |  map the control output to an appropriate target PWM value
    ui8_duty_cycle_target = (uint8_t) (map ((uint32_t) i16_control_output,
                                            (uint32_t) 0,                    // minimum control output from PID
                                            (uint32_t) 1000,                 // maximum control output from PID
                                            (uint32_t) PWM_DUTY_CYCLE_MIN,   // minimum duty cycle
                                            (uint32_t) PWM_DUTY_CYCLE_MAX)); // maximum duty cycle
  }
  else
  {
    // set flag to initialize cruise PID if user later activates function
    ui8_cruise_PID_initialized = 0;
  }
}



static void apply_throttle()
{
  if ((m_configuration_variables.ui8_optional_ADC == THROTTLE_CONTROL) && (ui8_riding_mode != WALK_ASSIST_MODE) && (ui8_riding_mode != CRUISE_MODE))
  {
    uint8_t ui8_adc_battery_current_target_throttle = 0;
    
    // map value from 0 up to 255
    ui8_adc_battery_current_target_throttle = (uint8_t) (map ((uint8_t) UI8_ADC_THROTTLE,
                                                              (uint8_t) ADC_THROTTLE_MIN_VALUE,
                                                              (uint8_t) ADC_THROTTLE_MAX_VALUE,
                                                              (uint8_t) 0,
                                                              (uint8_t) ui8_adc_battery_current_max));
                                                             
    // set battery current target
    ui8_adc_battery_current_target = ui8_max(ui8_adc_battery_current_target, ui8_adc_battery_current_target_throttle);
    
    // set duty cycle target
    if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    else { ui8_duty_cycle_target = 0; }
  }
}



static void apply_speed_limit()
{
  if (m_configuration_variables.ui8_wheel_speed_max > 0)
  {
    // set battery current target 
    ui8_adc_battery_current_target = (uint8_t) (map ((uint32_t) ui16_wheel_speed_x10,
                                                     (uint32_t) ((m_configuration_variables.ui8_wheel_speed_max * 10) - 20),
                                                     (uint32_t) ((m_configuration_variables.ui8_wheel_speed_max * 10) + 20),
                                                     (uint32_t) ui8_adc_battery_current_target,
                                                     (uint32_t) 0));
  }
}



static void apply_temperature_limiting()
{
  if (m_configuration_variables.ui8_optional_ADC == TEMPERATURE_CONTROL)
  {
    static uint16_t ui16_adc_motor_temperature_filtered;
    
    // calculate motor temperature
    volatile uint16_t ui16_temp = UI16_ADC_10_BIT_THROTTLE;
    ui16_filter(&ui16_temp, &ui16_adc_motor_temperature_filtered, 5);
    m_configuration_variables.ui16_motor_temperature_x2 = (uint16_t) ui16_adc_motor_temperature_filtered / 1.024;
    m_configuration_variables.ui8_motor_temperature = (uint8_t) (m_configuration_variables.ui16_motor_temperature_x2 >> 1);
    
    // min temperature value can't be equal or higher than max temperature value...
    if (m_configuration_variables.ui8_motor_temperature_min_value_to_limit >= m_configuration_variables.ui8_motor_temperature_max_value_to_limit)
    {
      ui8_adc_battery_current_target = 0;
      m_configuration_variables.ui8_temperature_current_limiting_value = 0;
    }
    else
    {
      // reduce motor current if over temperature
      ui8_adc_battery_current_target = (uint8_t) (map ((uint32_t) m_configuration_variables.ui16_motor_temperature_x2,
                                                        (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                                                        (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                                                        (uint32_t) ui8_adc_battery_current_target,
                                                        (uint32_t) 0));
                                            
      // get a value linear to the current limitation, just to show to user
      m_configuration_variables.ui8_temperature_current_limiting_value = 
      (uint8_t) (map ((uint32_t) m_configuration_variables.ui16_motor_temperature_x2,
                      (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                      (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                      (uint32_t) 255,
                      (uint32_t) 0));
    }
  }
  else
  {
    // keep ui8_temperature_current_limiting_value = 255 because 255 means no current limiting happening, otherwise temperature symbol on display will be blinking
    m_configuration_variables.ui8_temperature_current_limiting_value = 255;
  }
}



static void calc_cadence(void)
{
  // if cadence is too low or direction of pedal rotation is not forward
  if ((ui16_pas_pwm_cycles_ticks > PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS) || (ui8_g_pedaling_direction != 1))
  {
    ui8_pedal_cadence_RPM = 0; 
  }
  else
  {
    ui8_pedal_cadence_RPM = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) PAS_NUMBER_MAGNETS) * 0.000064)); // cadence in RPM = 60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
  }
  
  if (m_configuration_variables.ui8_cadence_rpm_min > 10) { m_configuration_variables.ui8_cadence_rpm_min = 10; }
  
  if (ui8_pedal_cadence_RPM < m_configuration_variables.ui8_cadence_rpm_min) { ui8_pedal_cadence_RPM = m_configuration_variables.ui8_cadence_rpm_min; }
}



static void calc_wheel_speed(void)
{
  float f_wheel_speed_x10;
  
  // calc wheel speed in km/h
  if (ui16_wheel_speed_sensor_pwm_cycles_ticks < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS)
  {
    f_wheel_speed_x10 = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
    f_wheel_speed_x10 *= m_configuration_variables.ui16_wheel_perimeter; // millimeters per second
    f_wheel_speed_x10 *= 0.036; // ((3600 / (1000 * 1000)) * 10) kms per hour * 10
    ui16_wheel_speed_x10 = (uint16_t) f_wheel_speed_x10;
  }
  else
  {
    ui16_wheel_speed_x10 = 0;
  }
}



static void get_battery_voltage_filtered(void)
{
  ui16_battery_voltage_filtered_x1000 = ui16_adc_battery_voltage_filtered * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000;
}



static void get_battery_current_filtered(void)
{
  ui8_battery_current_filtered_x10 = ui8_adc_battery_current_filtered * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X10;
}



struct_configuration_variables* get_configuration_variables (void)
{
  return &m_configuration_variables;
}



static void check_brakes()
{
  // check if brakes are installed
  
  // set brake state
  ui8_brakes_enabled = brake_is_set();
}



static void check_system()
{
  #define MOTOR_BLOCKED_COUNTER_THRESHOLD               10    // 10  =>  1.0 second
  #define MOTOR_BLOCKED_BATTERY_CURRENT_THRESHOLD_X10   60    // 60  =>  6.0 amps
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
      ui8_system_state = NO_ERROR;
      
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
  
  
  // check if user applied force on the pedals during torque sensor calibration
  if (ui8_adc_pedal_torque_offset > 54)
  {
    // set error code
    ui8_system_state = ERROR_TORQUE_APPLIED_DURING_POWER_ON;
  }
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
      m_configuration_variables.ui8_lights = ui8_rx_buffer [4];
      
      // set lights
      lights_set_state(m_configuration_variables.ui8_lights);

      switch (ui8_message_ID)
      {
        case 0:
        
          // battery low voltage cut off x10
          m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = (((uint16_t) ui8_rx_buffer [6]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
          
          // set low voltage cut off
          ui8_adc_battery_voltage_cut_off = (uint8_t) (((uint32_t) m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 << 8) / (BATTERY_VOLTAGE_PER_8_BIT_ADC_STEP_X256 * 10));
          
          // wheel max speed
          m_configuration_variables.ui8_wheel_speed_max = ui8_rx_buffer [7];
          
        break;

        case 1:
        
          // wheel perimeter
          m_configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer [6]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
          
          // max battery current
          m_configuration_variables.ui8_battery_max_current = ui8_rx_buffer [7];
          
          // set max battery current
          ui8_adc_battery_current_max = (uint8_t) ((m_configuration_variables.ui8_battery_max_current * 10) / BATTERY_CURRENT_PER_8_BIT_ADC_STEP_X10);
          
        break;

        case 2:
        
          // type of motor (36 volt, 48 volt or some experimental type)
          m_configuration_variables.ui8_motor_type = ui8_rx_buffer [5];
          
          // motor over temperature min value limit
          m_configuration_variables.ui8_motor_temperature_min_value_to_limit = ui8_rx_buffer [6];
          
          // motor over temperature max value limit
          m_configuration_variables.ui8_motor_temperature_max_value_to_limit = ui8_rx_buffer [7];

        break;

        case 3:
        
          // boost assist level
          m_configuration_variables.ui8_startup_motor_power_boost_assist_level = ui8_rx_buffer [5];
          
          // boost state
          m_configuration_variables.ui8_startup_motor_power_boost_state = (ui8_rx_buffer [6] & 1);
          
          // boost max power limit enabled
          m_configuration_variables.ui8_startup_motor_power_boost_limit_to_max_power = (ui8_rx_buffer [6] & 2) >> 1;
          
          // boost runtime
          m_configuration_variables.ui8_startup_motor_power_boost_time = ui8_rx_buffer [7];
          
        break;

        case 4:

          // boost fade time
          m_configuration_variables.ui8_startup_motor_power_boost_fade_time = ui8_rx_buffer [5];
          
          // boost enabled
          m_configuration_variables.ui8_startup_motor_power_boost_feature_enabled = ui8_rx_buffer [6];
          
          // ramp up
          ui16_duty_cycle_ramp_up_inverse_step = ui8_rx_buffer [7];
        
        break;

        case 5:
        
          // motor temperature limit function or throttle
          m_configuration_variables.ui8_optional_ADC = ui8_rx_buffer [5];
          
          // motor assistance without pedal rotation
          m_configuration_variables.ui8_cadence_rpm_min = ui8_rx_buffer [6];
          
          // battery power limit
          m_configuration_variables.ui8_target_battery_max_power_div25 = ui8_rx_buffer [7];
          
          // set max battery current from power limit but do not set limit higher than max battery current
          if (m_configuration_variables.ui8_target_battery_max_power_div25 > 0)
          {
            uint32_t ui32_battery_current_max_x10 = ((uint32_t) m_configuration_variables.ui8_target_battery_max_power_div25 * 250000) / ui16_battery_voltage_filtered_x1000;
            uint8_t ui8_adc_battery_current_max_temp = ((ui32_battery_current_max_x10 + 5) / BATTERY_CURRENT_PER_8_BIT_ADC_STEP_X10);
            ui8_adc_battery_current_max = ui8_min(ui8_adc_battery_current_max, ui8_adc_battery_current_max_temp);
          }
          
        break;

        default:
          // nothing, should display error code
        break;
      }

      // verify if any configuration_variables did change and if so, save all of them in the EEPROM
      eeprom_write_if_values_changed();

      // signal that we processed the full package
      ui8_received_package_flag = 0;
    }

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
  ui8_tx_buffer[6] = ui8_brakes_enabled;

  // throttle value from ADC
  ui8_tx_buffer[7] = UI8_ADC_THROTTLE;
  
  // adjusted throttle value or temperature limit depending on user setup
  if (m_configuration_variables.ui8_optional_ADC == TEMPERATURE_CONTROL)
  {
    // temperature value
    ui8_tx_buffer[8] = m_configuration_variables.ui8_motor_temperature;
  }
  else
  {
    // throttle value with offset removed and mapped to 255
    ui8_tx_buffer[8] = ui8_adc_throttle;
  }

  // ADC torque sensor without torque offset
  ui8_tx_buffer[9] = ui8_adc_pedal_torque;
  
/*   // ADC torque_sensor with torque offset
  if (ui8_adc_pedal_torque > ui8_adc_pedal_torque_offset)
  {
    ui8_tx_buffer[10] = (ui8_adc_pedal_torque - ui8_adc_pedal_torque_offset);                                                            // test test test test test test remove remove remove
  }
  else
  {
    ui8_tx_buffer[10] = 0; 
  } */
  
  ui8_tx_buffer[10] = ui8_adc_pedal_torque_offset;

  // pedal cadence
  ui8_tx_buffer[11] = ui8_pedal_cadence_RPM;
  
  // PWM duty_cycle
  ui8_tx_buffer[12] = ui8_g_duty_cycle;
  
  // motor speed in ERPS
  ui16_temp = ui16_motor_get_motor_speed_erps();
  ui16_temp = UI16_ADC_10_BIT_BATTERY_CURRENT;                                                            // test test test test test test remove remove remove
  ui8_tx_buffer[13] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[14] = (uint8_t) (ui16_temp >> 8);
  
  // FOC angle
  ui8_tx_buffer[15] = ui8_g_foc_angle;
  
  // system state
  ui8_tx_buffer[16] = ui8_system_state;
  
  // temperature actual limiting value
  ui8_tx_buffer[17] = m_configuration_variables.ui8_temperature_current_limiting_value;
  
  // wheel_speed_sensor_tick_counter
  ui8_tx_buffer[18] = (uint8_t) (ui32_wheel_speed_sensor_tick_counter & 0xff);
  ui8_tx_buffer[19] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 8) & 0xff);
  ui8_tx_buffer[20] = (uint8_t) ((ui32_wheel_speed_sensor_tick_counter >> 16) & 0xff);

  // pedal torque x100
  ui8_tx_buffer[21] = (uint8_t) (ui16_pedal_torque_x100 & 0xff);
  ui8_tx_buffer[22] = (uint8_t) (ui16_pedal_torque_x100 >> 8);

  // ui16_pedal_power_x10
  ui8_tx_buffer[23] = (uint8_t) (ui16_pedal_power_x10 & 0xff);
  ui8_tx_buffer[24] = (uint8_t) (ui16_pedal_power_x10 >> 8);

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





static void apply_boost()
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
  
  uint8_t ui8_torque_sensor = 0;
  
  if (ui8_adc_pedal_torque > ui8_adc_pedal_torque_offset)
  {
    ui8_torque_sensor = ui8_adc_pedal_torque - ui8_adc_pedal_torque_offset;
  }

  if(m_configuration_variables.ui8_startup_motor_power_boost_time > 0)
  {
    switch(ui8_m_startup_boost_state_machine)
    {
      // ebike is stopped, wait for throttle signal to startup boost
      case BOOST_STATE_BOOST_DISABLED:
      
        if (ui8_torque_sensor > 12 && (ui8_brakes_enabled == 0))
        {
          ui8_startup_boost_enable = 1;
          ui8_startup_boost_timer = m_configuration_variables.ui8_startup_motor_power_boost_time;
          ui8_m_startup_boost_state_machine = BOOST_STATE_BOOST;
        }
        
      break;

      case BOOST_STATE_BOOST:
      
        // braking means reseting
        if(ui8_brakes_enabled)
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
        if(ui8_brakes_enabled)
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
}