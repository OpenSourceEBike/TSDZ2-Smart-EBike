/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "motor.h"
#include "interrupts.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "motor.h"
#include "ebike_app.h"
#include "pins.h"
#include "pwm.h"
#include "adc.h"
#include "utils.h"
#include "uart.h"
#include "adc.h"
#include "watchdog.h"
#include "math.h"

#define SVM_TABLE_LEN   256
#define SIN_TABLE_LEN   60

uint8_t ui8_svm_table [SVM_TABLE_LEN] =
{
    239 ,
    241 ,
    242 ,
    243 ,
    245 ,
    246 ,
    247 ,
    248 ,
    249 ,
    250 ,
    251 ,
    251 ,
    252 ,
    253 ,
    253 ,
    254 ,
    254 ,
    254 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    254 ,
    254 ,
    254 ,
    253 ,
    253 ,
    252 ,
    251 ,
    250 ,
    250 ,
    249 ,
    248 ,
    247 ,
    245 ,
    244 ,
    243 ,
    242 ,
    240 ,
    239 ,
    236 ,
    231 ,
    227 ,
    222 ,
    217 ,
    212 ,
    207 ,
    202 ,
    197 ,
    191 ,
    186 ,
    181 ,
    176 ,
    170 ,
    165 ,
    160 ,
    154 ,
    149 ,
    144 ,
    138 ,
    133 ,
    127 ,
    122 ,
    116 ,
    111 ,
    106 ,
    100 ,
    95  ,
    89  ,
    84  ,
    79  ,
    74  ,
    68  ,
    63  ,
    58  ,
    53  ,
    48  ,
    43  ,
    38  ,
    33  ,
    28  ,
    23  ,
    18  ,
    16  ,
    14  ,
    13  ,
    12  ,
    10  ,
    9 ,
    8 ,
    7 ,
    6 ,
    5 ,
    4 ,
    3 ,
    3 ,
    2 ,
    1 ,
    1 ,
    1 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    1 ,
    1 ,
    2 ,
    2 ,
    3 ,
    4 ,
    5 ,
    6 ,
    6 ,
    8 ,
    9 ,
    10  ,
    11  ,
    12  ,
    14  ,
    15  ,
    17  ,
    15  ,
    14  ,
    12  ,
    11  ,
    10  ,
    9 ,
    8 ,
    6 ,
    6 ,
    5 ,
    4 ,
    3 ,
    2 ,
    2 ,
    1 ,
    1 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    1 ,
    1 ,
    1 ,
    2 ,
    3 ,
    3 ,
    4 ,
    5 ,
    6 ,
    7 ,
    8 ,
    9 ,
    10  ,
    12  ,
    13  ,
    14  ,
    16  ,
    18  ,
    23  ,
    28  ,
    33  ,
    38  ,
    43  ,
    48  ,
    53  ,
    58  ,
    63  ,
    68  ,
    74  ,
    79  ,
    84  ,
    89  ,
    95  ,
    100 ,
    106 ,
    111 ,
    116 ,
    122 ,
    127 ,
    133 ,
    138 ,
    144 ,
    149 ,
    154 ,
    160 ,
    165 ,
    170 ,
    176 ,
    181 ,
    186 ,
    191 ,
    197 ,
    202 ,
    207 ,
    212 ,
    217 ,
    222 ,
    227 ,
    231 ,
    236 ,
    239 ,
    240 ,
    242 ,
    243 ,
    244 ,
    245 ,
    247 ,
    248 ,
    249 ,
    250 ,
    250 ,
    251 ,
    252 ,
    253 ,
    253 ,
    254 ,
    254 ,
    254 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    254 ,
    254 ,
    254 ,
    253 ,
    253 ,
    252 ,
    251 ,
    251 ,
    250 ,
    249 ,
    248 ,
    247 ,
    246 ,
    245 ,
    243 ,
    242 ,
    241 ,
    239 ,
    238 ,
};

uint8_t ui8_sin_table [SIN_TABLE_LEN] =
{
    0 ,
    3 ,
    6 ,
    9 ,
    12  ,
    16  ,
    19  ,
    22  ,
    25  ,
    28  ,
    31  ,
    34  ,
    37  ,
    40  ,
    43  ,
    46  ,
    49  ,
    52  ,
    54  ,
    57  ,
    60  ,
    63  ,
    66  ,
    68  ,
    71  ,
    73  ,
    76  ,
    78  ,
    81  ,
    83  ,
    86  ,
    88  ,
    90  ,
    92  ,
    95  ,
    97  ,
    99  ,
    101 ,
    102 ,
    104 ,
    106 ,
    108 ,
    109 ,
    111 ,
    113 ,
    114 ,
    115 ,
    117 ,
    118 ,
    119 ,
    120 ,
    121 ,
    122 ,
    123 ,
    124 ,
    125 ,
    125 ,
    126 ,
    126 ,
    127
};

uint16_t ui16_PWM_cycles_counter = 1;
uint16_t ui16_PWM_cycles_counter_6 = 1;
uint16_t ui16_PWM_cycles_counter_total = 0xffff;
uint16_t ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS;
static volatile uint16_t ui16_motor_speed_erps = 0;
uint8_t ui8_motor_rotor_absolute_angle;
uint8_t ui8_motor_rotor_angle;
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
uint8_t ui8_hall_sensors_state = 0;
uint8_t ui8_hall_sensors_state_last = 0;
uint8_t ui8_half_erps_flag = 0;


// power variables
volatile uint16_t ui16_adc_battery_current = 0;
volatile uint8_t ui8_controller_adc_battery_current = 0;
volatile uint16_t ui16_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
volatile uint16_t ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP;
volatile uint8_t ui8_g_duty_cycle = 0;
volatile uint16_t ui16_adc_battery_voltage_filtered = 0;
volatile uint8_t ui8_adc_battery_current_filtered = 0;
volatile uint8_t ui8_g_foc_angle = 0;
volatile uint8_t ui8_controller_adc_battery_current_target = 0;
volatile uint8_t ui8_controller_duty_cycle_target = 0;
volatile uint8_t ui8_adc_battery_voltage_cut_off = 0xff;


// cadence sensor
volatile uint16_t ui16_cadence_sensor_ticks = 0;
volatile uint16_t ui16_cadence_sensor_high_ticks_counter_min = CADENCE_SENSOR_TICKS_COUNTER_MIN;
volatile uint16_t ui16_cadence_sensor_low_ticks_counter_min = CADENCE_SENSOR_TICKS_COUNTER_MIN;
volatile uint16_t ui16_cadence_sensor_high_conversion_x100 = 100;
volatile uint16_t ui16_cadence_sensor_low_conversion_x100 = 100;
volatile uint16_t ui16_cadence_sensor_conversion_x100 = 100;


// wheel speed sensor
volatile uint16_t ui16_wheel_speed_sensor_ticks = 0;
volatile uint32_t ui32_wheel_speed_sensor_ticks_total = 0;


void read_battery_voltage(void);
void read_battery_current(void);
void calc_foc_angle(void);
uint8_t asin_table(uint8_t ui8_inverted_angle_x128);


void motor_controller(void)
{
  read_battery_voltage();
  read_battery_current();
  calc_foc_angle();
}


// Measures did with a 24V Q85 328 RPM motor, rotating motor backwards by hand:
// Hall sensor A positivie to negative transition | BEMF phase B at max value / top of sinewave
// Hall sensor B positivie to negative transition | BEMF phase A at max value / top of sinewave
// Hall sensor C positive to negative transition | BEMF phase C at max value / top of sinewave

// runs every 64us (PWM frequency)
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER)
{
  static uint8_t ui8_svm_table_index;
  static uint8_t ui8_adc_motor_phase_current;


  /****************************************************************************/
  
  
  // read battery current ADC value | should happen at middle of the PWM duty_cycle
  ADC1->CR2 &= (uint8_t)(~ADC1_CR2_SCAN);   // disable scan mode
  ADC1->CSR = 0x05;                         // clear EOC flag first (select channel 5)
  ADC1->CR1 |= ADC1_CR1_ADON;               // start ADC1 conversion
  while (!(ADC1->CSR & ADC1_FLAG_EOC));     // wait for end of conversion
  
  ui8_controller_adc_battery_current = ui16_adc_battery_current = UI16_ADC_10_BIT_BATTERY_CURRENT;

  // calculate motor phase current ADC value
  if (ui8_g_duty_cycle > 0)
  {
    ui8_adc_motor_phase_current = ((ui16_adc_battery_current << 6) / ((uint16_t) ui8_g_duty_cycle));
  }
  else
  {
    ui8_adc_motor_phase_current = 0;
  }


  /****************************************************************************/
  
  
  // trigger ADC conversion of all channels (scan conversion, buffered)
  ADC1->CR2 |= ADC1_CR2_SCAN;     // enable scan mode
  ADC1->CSR = 0x07;               // clear EOC flag first (select channel 7)
  ADC1->CR1 |= ADC1_CR1_ADON;     // start ADC1 conversion


  /****************************************************************************/
  
  
  // read hall sensor signals and:
  // - find the motor rotor absolute angle
  // - calc motor speed in erps (ui16_motor_speed_erps)

  // read hall sensors signal pins and mask other pins
  // hall sensors sequence with motor forward rotation: 4, 6, 2, 3, 1, 5
  ui8_hall_sensors_state = ((HALL_SENSOR_A__PORT->IDR & HALL_SENSOR_A__PIN) >> 5) |
                           ((HALL_SENSOR_B__PORT->IDR & HALL_SENSOR_B__PIN) >> 1) |
                           ((HALL_SENSOR_C__PORT->IDR & HALL_SENSOR_C__PIN) >> 3);
                           
  // make sure we run next code only when there is a change on the hall sensors signal
  if (ui8_hall_sensors_state != ui8_hall_sensors_state_last)
  {
    ui8_hall_sensors_state_last = ui8_hall_sensors_state;

    switch (ui8_hall_sensors_state)
    {
      case 3:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_150;
      break;

      case 1:
      if (ui8_half_erps_flag == 1)
      {
        ui8_half_erps_flag = 0;
        ui16_PWM_cycles_counter_total = ui16_PWM_cycles_counter;
        ui16_PWM_cycles_counter = 1;

        // this division takes 4.4us and without the cast (uint16_t) PWM_CYCLES_SECOND, would take 111us!! Verified on 2017.11.20
        // avoid division by 0
        if (ui16_PWM_cycles_counter_total > 0) 
        {
          ui16_motor_speed_erps = ((uint16_t) PWM_CYCLES_SECOND) / ui16_PWM_cycles_counter_total; 
        }
        else
        { 
          ui16_motor_speed_erps = ((uint16_t) PWM_CYCLES_SECOND); 
        }

        // update motor commutation state based on motor speed
        if (ui16_motor_speed_erps > MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES)
        {
          if (ui8_motor_commutation_type == BLOCK_COMMUTATION)
          {
            ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES;
          }
        }
        else
        {
          if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
          {
            ui8_motor_commutation_type = BLOCK_COMMUTATION;
            ui8_g_foc_angle = 0;
          }
        }
      }

      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_210;
      break;

      case 5:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_270;
      break;

      case 4:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_330;
      break;

      case 6:
      ui8_half_erps_flag = 1;

      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_30;
      break;

      // BEMF is always 90 degrees advanced over motor rotor position degree zero
      // and here (hall sensor C blue wire, signal transition from positive to negative),
      // phase B BEMF is at max value (measured on osciloscope by rotating the motor)
      case 2:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_90;
      break;

      default:
      return;
      break;
    }

    ui16_PWM_cycles_counter_6 = 1;
  }


  /****************************************************************************/
  
  
  // count number of fast loops / PWM cycles and reset some states when motor is near zero speed
  if (ui16_PWM_cycles_counter < ((uint16_t) PWM_CYCLES_COUNTER_MAX))
  {
    ui16_PWM_cycles_counter++;
    ui16_PWM_cycles_counter_6++;
  }
  else // happens when motor is stopped or near zero speed
  {
    ui16_PWM_cycles_counter = 1; // don't put to 0 to avoid 0 divisions
    ui16_PWM_cycles_counter_6 = 1;
    ui8_half_erps_flag = 0;
    ui16_motor_speed_erps = 0;
    ui16_PWM_cycles_counter_total = 0xffff;
    ui8_g_foc_angle = 0;
    ui8_motor_commutation_type = BLOCK_COMMUTATION;
    ui8_hall_sensors_state_last = 0; // this way we force execution of hall sensors code next time
  }


  /****************************************************************************/
  
  
  // - calc interpolation angle and sinewave table index
  
  static uint8_t ui8_interpolation_angle;
  
#define DO_INTERPOLATION 1 // may be useful to disable interpolation when debugging
#if DO_INTERPOLATION == 1
  // calculate the interpolation angle (and it doesn't work when motor starts and at very low speeds)
  if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
  {
    // division by 0: ui16_PWM_cycles_counter_total should never be 0
    // TODO: verifiy if (ui16_PWM_cycles_counter_6 << 8) do not overflow
    ui8_interpolation_angle = (ui16_PWM_cycles_counter_6 << 8) / ui16_PWM_cycles_counter_total; // this operations take 4.4us
    ui8_motor_rotor_angle = ui8_motor_rotor_absolute_angle + ui8_interpolation_angle;
    ui8_svm_table_index = ui8_motor_rotor_angle + ui8_g_foc_angle;
  }
  else
#endif
  {
    ui8_svm_table_index = ui8_motor_rotor_absolute_angle + ui8_g_foc_angle;
  }

  // we need to put phase voltage 90 degrees ahead of rotor position, to get current 90 degrees ahead and have max torque per amp
  ui8_svm_table_index -= 63;
  
  
  /****************************************************************************/


  // PWM duty_cycle controller:
  // - limit battery undervoltage
  // - limit battery max current
  // - limit motor max phase current
  // - limit motor max ERPS
  // - ramp up/down PWM duty_cycle value
  // do not execute all, otherwise duty cycle would be decremented more than onece on each PWM cycle

  static uint16_t ui16_counter_duty_cycle_ramp_up;
  static uint16_t ui16_counter_duty_cycle_ramp_down;

  if (UI8_ADC_BATTERY_VOLTAGE < ui8_adc_battery_voltage_cut_off) // battery voltage under min voltage, immediately reduce duty_cycle
  {
    if (ui8_g_duty_cycle > 0)
    {
      // decrement duty cycle
      ui8_g_duty_cycle--;
    }
  }
  else if (ui16_motor_speed_erps > ui16_max_motor_speed_erps) // motor speed over max motor ERPS, immediately reduce duty_cycle
  {
    if (ui8_g_duty_cycle > 0)
    {
      // decrement duty cycle
      ui8_g_duty_cycle--;
    }
  }
  else // adjust duty cycle to target duty cycle, include ramping
  {
    if ((ui8_controller_duty_cycle_target < ui8_g_duty_cycle) ||
        (ui8_controller_adc_battery_current > ui8_controller_adc_battery_current_target) ||
        (ui8_adc_motor_phase_current > ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX))
    {
      ui16_counter_duty_cycle_ramp_up = 0;
      
      // duty cycle ramp down
      if (++ui16_counter_duty_cycle_ramp_down > ui16_duty_cycle_ramp_down_inverse_step)
      {
        ui16_counter_duty_cycle_ramp_down = 0;
        
        // decrement duty cycle
        ui8_g_duty_cycle--;
      }
    }
    else if (ui8_controller_duty_cycle_target > ui8_g_duty_cycle)
    {
      ui16_counter_duty_cycle_ramp_down = 0;
      
      // limit duty cycle ramp up
      if (ui16_duty_cycle_ramp_up_inverse_step < 20 ) { ui16_duty_cycle_ramp_up_inverse_step = 20; }
      
      // duty cycle ramp up
      if (++ui16_counter_duty_cycle_ramp_up > ui16_duty_cycle_ramp_up_inverse_step)
      {
        ui16_counter_duty_cycle_ramp_up = 0;
        
        // increment duty cycle
        ui8_g_duty_cycle++;
      }
    }
    else
    {
      // duty cycle is where it needs to be so reset ramp counters
      ui16_counter_duty_cycle_ramp_up = 0;
      ui16_counter_duty_cycle_ramp_down = 0;
    }
  }



  /****************************************************************************/
  

  
  // calculate final PWM duty_cycle values to be applied to TIMER1
  
  uint8_t ui8_phase_a_voltage;
  uint8_t ui8_phase_b_voltage;
  uint8_t ui8_phase_c_voltage;
  uint8_t ui8_temp;
  uint16_t ui16_value;
  
  // scale and apply PWM duty_cycle for the 3 phases
  // phase A is advanced 240 degrees over phase B
  ui8_temp = ui8_svm_table [(uint8_t) (ui8_svm_table_index + 171 /* 240º */)];
  if (ui8_temp > MIDDLE_PWM_DUTY_CYCLE_MAX)
  {
    ui16_value = ((uint16_t) (ui8_temp - MIDDLE_PWM_DUTY_CYCLE_MAX)) * ui8_g_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_a_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + ui8_temp;
  }
  else
  {
    ui16_value = ((uint16_t) (MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp)) * ui8_g_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_a_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp;
  }

  // phase B as reference phase
  ui8_temp = ui8_svm_table [ui8_svm_table_index];
  if (ui8_temp > MIDDLE_PWM_DUTY_CYCLE_MAX)
  {
    ui16_value = ((uint16_t) (ui8_temp - MIDDLE_PWM_DUTY_CYCLE_MAX)) * ui8_g_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_b_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + ui8_temp;
  }
  else
  {
    ui16_value = ((uint16_t) (MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp)) * ui8_g_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_b_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp;
  }

  // phase C is advanced 120 degrees over phase B
  ui8_temp = ui8_svm_table [(uint8_t) (ui8_svm_table_index + 85 /* 120º */)];
  if (ui8_temp > MIDDLE_PWM_DUTY_CYCLE_MAX)
  {
    ui16_value = ((uint16_t) (ui8_temp - MIDDLE_PWM_DUTY_CYCLE_MAX)) * ui8_g_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_c_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + ui8_temp;
  }
  else
  {
    ui16_value = ((uint16_t) (MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp)) * ui8_g_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_c_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp;
  }

  // set final duty_cycle value
  // phase B
  TIM1->CCR3H = (uint8_t) (ui8_phase_b_voltage >> 7);
  TIM1->CCR3L = (uint8_t) (ui8_phase_b_voltage << 1);
  // phase C
  TIM1->CCR2H = (uint8_t) (ui8_phase_c_voltage >> 7);
  TIM1->CCR2L = (uint8_t) (ui8_phase_c_voltage << 1);
  // phase A
  TIM1->CCR1H = (uint8_t) (ui8_phase_a_voltage >> 7);
  TIM1->CCR1L = (uint8_t) (ui8_phase_a_voltage << 1);



  /****************************************************************************/
  
  
  #define CADENCE_SENSOR_SCHMITT_TRIGGER_THRESHOLD    500 // software based Schmitt trigger to stop motor jitter when at resolution limits
  
  static uint16_t ui16_cadence_sensor_ticks_counter;
  static uint16_t ui16_cadence_sensor_ticks_counter_min;
  static uint8_t ui8_cadence_sensor_ticks_counter_started;
  static uint8_t ui8_cadence_sensor_pin_state_old;
  
  // check cadence sensor pins state
  volatile uint8_t ui8_cadence_sensor_pin_1_state = PAS2__PORT->IDR & PAS2__PIN; // PAS2__PIN is leading on all controllers
  volatile uint8_t ui8_cadence_sensor_pin_2_state = PAS1__PORT->IDR & PAS1__PIN; // PAS1__PIN is following on all controllers
  
  // check if cadence sensor pin state has changed
  if (ui8_cadence_sensor_pin_1_state != ui8_cadence_sensor_pin_state_old)
  {
    // update old cadence sensor pin state
    ui8_cadence_sensor_pin_state_old = ui8_cadence_sensor_pin_1_state;
    
    // set the ticks counter limit and conversion depending on pin state
    if (ui8_cadence_sensor_pin_1_state)
    {
      // set counter limit depending on current pin state
      ui16_cadence_sensor_ticks_counter_min = ui16_cadence_sensor_high_ticks_counter_min;
      
      // set conversion depending on previous pin state
      ui16_cadence_sensor_conversion_x100 = ui16_cadence_sensor_low_conversion_x100;
    }
    else
    {
      // set counter limit depending on current pin state
      ui16_cadence_sensor_ticks_counter_min = ui16_cadence_sensor_low_ticks_counter_min;
      
      // set conversion depending on previous pin state
      ui16_cadence_sensor_conversion_x100 = ui16_cadence_sensor_high_conversion_x100;
    }
    
    // check if first or second transition
    if (!ui8_cadence_sensor_ticks_counter_started) 
    {
      // start cadence sensor ticks counter as this is the first transition
      ui8_cadence_sensor_ticks_counter_started = 1;
    }
    else // second transition
    {
      // check if cadence sensor ticks counter is out of bounds and also check direction of rotation
      if ((ui16_cadence_sensor_ticks_counter < CADENCE_SENSOR_TICKS_COUNTER_MAX) || 
          (ui8_cadence_sensor_pin_1_state == ui8_cadence_sensor_pin_2_state))
      {
        ui16_cadence_sensor_ticks = 0;
        ui16_cadence_sensor_ticks_counter = 0;
        ui8_cadence_sensor_ticks_counter_started = 0;
      }
      else
      {
        // set the cadence sensor ticks between two transitions
        ui16_cadence_sensor_ticks = ui16_cadence_sensor_ticks_counter;
        ui16_cadence_sensor_ticks_counter = 0;
        
        // software based Schmitt trigger to stop motor jitter when at resolution limits
        ui16_cadence_sensor_ticks_counter_min += CADENCE_SENSOR_SCHMITT_TRIGGER_THRESHOLD;
      }
    }
  }
  
  // increment and also limit the ticks counter
  if ((ui8_cadence_sensor_ticks_counter_started) && (ui16_cadence_sensor_ticks_counter < ui16_cadence_sensor_ticks_counter_min)) 
  {
    ++ui16_cadence_sensor_ticks_counter;
  }
  else
  {
    ui16_cadence_sensor_ticks = 0;
    ui16_cadence_sensor_ticks_counter = 0;
    ui8_cadence_sensor_ticks_counter_started = 0;
  }
  
  
  
  /****************************************************************************/
  
  
  
  static uint16_t ui16_wheel_speed_sensor_ticks_counter;
  static uint8_t ui8_wheel_speed_sensor_ticks_counter_started;
  static uint8_t ui8_wheel_speed_sensor_pin_state_old;
  
  // check wheel speed sensor pin state
  uint8_t ui8_wheel_speed_sensor_pin_state = WHEEL_SPEED_SENSOR__PORT->IDR & WHEEL_SPEED_SENSOR__PIN;
  
  // check if wheel speed sensor pin state has changed
  if (ui8_wheel_speed_sensor_pin_state != ui8_wheel_speed_sensor_pin_state_old)
  {
    // update old wheel speed sensor pin state
    ui8_wheel_speed_sensor_pin_state_old = ui8_wheel_speed_sensor_pin_state;
    
    // only consider the 0 -> 1 transition
    if (ui8_wheel_speed_sensor_pin_state)
    {
      // check if first or second transition
      if (!ui8_wheel_speed_sensor_ticks_counter_started) 
      {
        // start wheel speed sensor ticks counter as this is the first transition
        ui8_wheel_speed_sensor_ticks_counter_started = 1;
      }
      else // second transition
      {
        // check if wheel speed sensor ticks counter is out of bounds
        if (ui16_wheel_speed_sensor_ticks_counter < WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS)
        {
          ui16_wheel_speed_sensor_ticks = 0;
          ui16_wheel_speed_sensor_ticks_counter = 0;
          ui8_wheel_speed_sensor_ticks_counter_started = 0;
        }
        else
        {
          ui16_wheel_speed_sensor_ticks = ui16_wheel_speed_sensor_ticks_counter;
          ui16_wheel_speed_sensor_ticks_counter = 0;
          ++ui32_wheel_speed_sensor_ticks_total;
        }
      }
    }
  }
  
  // increment and also limit the ticks counter
  if ((ui8_wheel_speed_sensor_ticks_counter_started) && (ui16_wheel_speed_sensor_ticks_counter < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS))
  {
    ++ui16_wheel_speed_sensor_ticks_counter;
  }
  else
  {
    ui16_wheel_speed_sensor_ticks = 0;
    ui16_wheel_speed_sensor_ticks_counter = 0;
    ui8_wheel_speed_sensor_ticks_counter_started = 0;
  }



  /****************************************************************************/

//  static uint8_t ui8_first_time_run_flag;
//
//  // reload watchdog timer, every PWM cycle to avoid automatic reset of the microcontroller
//  if (!ui8_first_time_run_flag)
//  { // from the init of watchdog up to first reset on PWM cycle interrupt,
//    // it can take up to 250ms and so we need to init here inside the PWM cycle
//    ui8_first_time_run_flag = 1;
//    watchdog_init ();
//  }
//  else
//  {
//    IWDG->KR = IWDG_KEY_REFRESH; // reload watch dog timer counter
//  }


  /****************************************************************************/


  // clears the TIM1 interrupt TIM1_IT_UPDATE pending bit
  TIM1->SR1 = (uint8_t)(~(uint8_t)TIM1_IT_CC4);
}

void motor_disable_PWM(void)
{
  TIM1_CtrlPWMOutputs(DISABLE);
}

void motor_enable_PWM(void)
{
  TIM1_CtrlPWMOutputs(ENABLE);
}


void hall_sensor_init(void)
{
  GPIO_Init (HALL_SENSOR_A__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init (HALL_SENSOR_B__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init (HALL_SENSOR_C__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_NO_IT);
}


uint16_t ui16_motor_get_motor_speed_erps(void)
{
  return ui16_motor_speed_erps;
}


void read_battery_voltage(void)
{
  #define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT   2

  /*---------------------------------------------------------
    NOTE: regarding filter coefficients 

    Possible values: 0, 1, 2, 3, 4, 5, 6
    0 equals to no filtering and no delay, higher values 
    will increase filtering but will also add a bigger delay.
  ---------------------------------------------------------*/

  static uint16_t ui16_adc_battery_voltage_accumulated;
  
  // low pass filter the voltage readed value, to avoid possible fast spikes/noise
  ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui16_adc_battery_voltage_accumulated += ui16_adc_read_battery_voltage_10b();
  ui16_adc_battery_voltage_filtered = ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
}


void read_battery_current(void)
{
  #define READ_BATTERY_CURRENT_FILTER_COEFFICIENT   2

  /*---------------------------------------------------------
    NOTE: regarding filter coefficients 

    Possible values: 0, 1, 2, 3, 4, 5, 6
    0 equals to no filtering and no delay, higher values 
    will increase filtering but will also add a bigger delay.
  ---------------------------------------------------------*/

  static uint16_t ui16_adc_battery_current_accumulated;
  
  // low pass filter the positive battery readed value (no regen current), to avoid possible fast spikes/noise
  ui16_adc_battery_current_accumulated -= ui16_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
  ui16_adc_battery_current_accumulated += ui16_adc_battery_current;
  ui8_adc_battery_current_filtered = ui16_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
}


void calc_foc_angle(void)
{
  uint16_t ui16_temp;
  uint16_t ui16_e_phase_voltage;
  uint32_t ui32_i_phase_current_x2;
  uint32_t ui32_l_x1048576;
  uint32_t ui32_w_angular_velocity_x16;
  uint16_t ui16_iwl_128;

  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  // FOC implementation by calculating the angle between phase current and rotor magnetic flux (BEMF)
  // 1. phase voltage is calculate
  // 2. I*w*L is calculated, where I is the phase current. L was a measured value for 48V motor.
  // 3. inverse sin is calculated of (I*w*L) / phase voltage, were we obtain the angle
  // 4. previous calculated angle is applied to phase voltage vector angle and so the
  // angle between phase current and rotor magnetic flux (BEMF) is kept at 0 (max torque per amp)

  // calc E phase voltage
  ui16_e_phase_voltage = ((uint32_t) ui16_adc_battery_voltage_filtered * BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X512 * ui8_g_duty_cycle) >> 17;

  // calc I phase current
  if (ui8_g_duty_cycle > 10)
  {
    ui16_temp = ((uint16_t) ui8_adc_battery_current_filtered) * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X512;
    ui32_i_phase_current_x2 = ui16_temp / ui8_g_duty_cycle;
  }
  else
  {
    ui32_i_phase_current_x2 = 0;
  }

  // calc W angular velocity: erps * 6.3
  ui32_w_angular_velocity_x16 = ui16_motor_speed_erps * 101;

  // ---------------------------------------------------------------------------------------------------------------------
  // NOTE: EXPERIMENTAL and may not be good for the brushless motor inside TSDZ2
  // Original message from jbalat on 28.08.2018, about increasing the limits on 36 V motor -- please see that this seems to go over the recomended values
  // The ui32_l_x1048576 = 105 is working well so give that a try if you have a 36 V motor.
  // This is the minimum value that gives me 550 W of power when I have asked for 550 W at level 5 assist, > 36 km/hr
  //
  // Remember also to boost the max motor erps in main.h to get higher cadence
  // #define MOTOR_OVER_SPEED_ERPS 700 // motor max speed, protection max value | 30 points for the sinewave at max speed
  // ---------------------------------------------------------------------------------------------------------------------

  // ---------------------------------------------------------------------------------------------------------------------
  // 36 V motor: L = 76uH
  // 48 V motor: L = 135uH
  // ui32_l_x1048576 = 142; // 1048576 = 2^20 | 48V
  // ui32_l_x1048576 = 80; // 1048576 = 2^20 | 36V
  //
  // ui32_l_x1048576 = 142 <--- THIS VALUE WAS verified experimentaly on 2018.07 to be near the best value for a 48V motor
  // Test done with a fixed mechanical load, duty_cycle = 200 and 100 and measured battery current was 16 and 6 (10 and 4 amps)
  // ---------------------------------------------------------------------------------------------------------------------
  
  switch (p_configuration_variables->ui8_motor_type)
  {
    case 0:
      ui32_l_x1048576 = 142; // 48 V motor
      ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS;
    break;

    case 1:
      ui32_l_x1048576 = 80; // 36 V motor
      ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS;
    break;
    
    case 2: // experimental high cadence mode for 48 volt motor
      ui32_l_x1048576 = 199;
      ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS_EXPERIMENTAL;
    break;
    
    case 3: // experimental high cadence mode for 36 volt motor
      ui32_l_x1048576 = 115;
      ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS_EXPERIMENTAL;
    break;

    default:
      ui32_l_x1048576 = 142; // 48 V motor
      ui16_max_motor_speed_erps = (uint16_t) MOTOR_OVER_SPEED_ERPS;
    break;
  }

  // calc IwL
  ui16_iwl_128 = (ui32_i_phase_current_x2 * ui32_w_angular_velocity_x16 * ui32_l_x1048576) >> 18;

  // calc FOC angle
  ui8_g_foc_angle = asin_table(ui16_iwl_128 / ui16_e_phase_voltage);

  // low pass filter FOC angle
  static uint16_t ui16_foc_angle_accumulated;
  ui16_foc_angle_accumulated -= ui16_foc_angle_accumulated >> 4;
  ui16_foc_angle_accumulated += ui8_g_foc_angle;
  ui8_g_foc_angle = ui16_foc_angle_accumulated >> 4;
}

// calc asin also converts the final result to degrees
uint8_t asin_table (uint8_t ui8_inverted_angle_x128)
{
  uint8_t ui8_index = 0;

  while (ui8_index < SIN_TABLE_LEN)
  {
    if (ui8_inverted_angle_x128 < ui8_sin_table [ui8_index])
    {
      break;
    }

    ui8_index++;
  }

  // first value of table is 0 so ui8_index will always increment to at least 1 and return 0
  return ui8_index--;
}


void motor_enable_pwm(void)
{
  TIM1_OC1Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_ENABLE,
         TIM1_OUTPUTNSTATE_ENABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_ENABLE,
         TIM1_OUTPUTNSTATE_ENABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_ENABLE,
         TIM1_OUTPUTNSTATE_ENABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);
}

void motor_disable_pwm(void)
{
  TIM1_OC1Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
         TIM1_OUTPUTSTATE_DISABLE,
         TIM1_OUTPUTNSTATE_DISABLE,
         255, // initial duty_cycle value
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCPOLARITY_HIGH,
         TIM1_OCIDLESTATE_RESET,
         TIM1_OCIDLESTATE_SET);

}
