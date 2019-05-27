/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>

#include "stm8s.h"
#include "stm8s_uart2.h"
#include "main.h"
#include "lcd.h"
#include "utils.h"

#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   25  // change this value depending on how many data bytes there is to receive ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_NUMBER_DATA_BYTES_TO_SEND      6   // change this value depending on how many data bytes there is to send ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_MAX_NUMBER_MESSAGE_ID          7

volatile uint8_t  ui8_received_package_flag = 0;
volatile uint8_t  ui8_rx_buffer[UART_NUMBER_DATA_BYTES_TO_RECEIVE + 3];
volatile uint8_t  ui8_rx_counter = 0;
volatile uint8_t  ui8_tx_buffer[UART_NUMBER_DATA_BYTES_TO_SEND + 3];
volatile uint8_t  ui8_i;
volatile uint8_t  ui8_byte_received;
volatile uint8_t  ui8_state_machine = 0;
volatile uint8_t  ui8_uart_received_first_package = 0;
static uint16_t   ui16_crc_rx;
static uint16_t   ui16_crc_tx;
static uint8_t    ui8_message_ID = 0;


void uart2_init (void)
{
  UART2_DeInit();
  UART2_Init((uint32_t) 9600,
       UART2_WORDLENGTH_8D,
       UART2_STOPBITS_1,
       UART2_PARITY_NO,
       UART2_SYNCMODE_CLOCK_DISABLE,
       UART2_MODE_TXRX_ENABLE);

  UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
}

// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    ui8_byte_received = UART2_ReceiveData8 ();

    switch (ui8_state_machine)
    {
      case 0:
      if (ui8_byte_received == 67) // see if we get start package byte
      {
        ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
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
      ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
      
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


void uart_data_clock (void)
{
  static uint32_t ui32_wss_tick_temp;
  
  struct_motor_controller_data *p_motor_controller_data;
  struct_configuration_variables *p_configuration_variables;

  if (ui8_received_package_flag)
  {
    // validation of the package data
    ui16_crc_rx = 0xffff;
    
    for (ui8_i = 0; ui8_i <= UART_NUMBER_DATA_BYTES_TO_RECEIVE; ui8_i++)
    {
      crc16 (ui8_rx_buffer[ui8_i], &ui16_crc_rx);
    }
    
    // if CRC is ok read the package
    if (((((uint16_t) ui8_rx_buffer [UART_NUMBER_DATA_BYTES_TO_RECEIVE + 2]) << 8) + ((uint16_t) ui8_rx_buffer [UART_NUMBER_DATA_BYTES_TO_RECEIVE + 1])) == ui16_crc_rx)
    {
      p_motor_controller_data = lcd_get_motor_controller_data ();
      p_configuration_variables = get_configuration_variables ();
      
      // ADC 10 bits battery voltage
      p_motor_controller_data->ui16_adc_battery_voltage = ui8_rx_buffer[1];
      p_motor_controller_data->ui16_adc_battery_voltage |= ((uint16_t) (ui8_rx_buffer[2] & 0x30)) << 4;
      
      // battery current x5
      p_motor_controller_data->ui8_battery_current_x5 = ui8_rx_buffer[3];
      
      // wheel speed
      p_motor_controller_data->ui16_wheel_speed_x10 = (((uint16_t) ui8_rx_buffer [4]) << 8) + ((uint16_t) ui8_rx_buffer [5]);
      
      // brake state
      p_motor_controller_data->ui8_motor_controller_state_2 = ui8_rx_buffer[6];
      
      // set brake state
      p_motor_controller_data->ui8_braking = p_motor_controller_data->ui8_motor_controller_state_2 & 1;
      
      // throttle value from ADC
      p_motor_controller_data->ui8_adc_throttle = ui8_rx_buffer[7];
      
      // adjusted throttle value or temperature limit depending on user setup
      if (p_configuration_variables->ui8_temperature_limit_feature_enabled == 1)
      {
        // temperature value
        p_motor_controller_data->ui8_motor_temperature = ui8_rx_buffer[8];
      }
      else
      {
        // throttle value with offset removed and mapped to 255
        p_motor_controller_data->ui8_throttle = ui8_rx_buffer[8];
      }
      
      // ADC torque_sensor
      p_motor_controller_data->ui8_adc_pedal_torque_sensor = ui8_rx_buffer[9];
      
      // torque sensor value with offset removed and mapped to 255
      p_motor_controller_data->ui8_pedal_torque_sensor = ui8_rx_buffer[10];
      
      // PAS cadence
      p_motor_controller_data->ui8_pedal_cadence = ui8_rx_buffer[11];
      
      // pedal human power mapped to 255
      p_motor_controller_data->ui8_pedal_human_power = ui8_rx_buffer[12];
      
      // PWM duty_cycle
      p_motor_controller_data->ui8_duty_cycle = ui8_rx_buffer[13];
      
      // motor speed in ERPS
      p_motor_controller_data->ui16_motor_speed_erps = (((uint16_t) ui8_rx_buffer [15]) << 8) + ((uint16_t) ui8_rx_buffer [14]);
      
      // FOC angle
      p_motor_controller_data->ui8_foc_angle = ui8_rx_buffer[16];
      
      // controller system state
      p_motor_controller_data->ui8_error_states = ui8_rx_buffer[17];
      
      // temperature actual limiting value
      p_motor_controller_data->ui8_temperature_current_limiting_value = ui8_rx_buffer[18];
      
      // wheel_speed_sensor_tick_counter
      ui32_wss_tick_temp = ((uint32_t) ui8_rx_buffer[19]);
      ui32_wss_tick_temp |= (((uint32_t) ui8_rx_buffer[20]) << 8);
      ui32_wss_tick_temp |= (((uint32_t) ui8_rx_buffer[21]) << 16);
      p_motor_controller_data->ui32_wheel_speed_sensor_tick_counter = ui32_wss_tick_temp;

      // ui16_pedal_torque_x10
      p_motor_controller_data->ui16_pedal_torque_x10 = (((uint16_t) ui8_rx_buffer [23]) << 8) + ((uint16_t) ui8_rx_buffer [22]);
      
      // ui16_pedal_power_x10
      p_motor_controller_data->ui16_pedal_power_x10 = (((uint16_t) ui8_rx_buffer [25]) << 8) + ((uint16_t) ui8_rx_buffer [24]);

      // signal that we processed the full package
      ui8_received_package_flag = 0;


      // ----------------- now send the data to the motor controller ----------------- //

      
      // start up byte
      ui8_tx_buffer[0] = 0x59;
      
      // message ID
      ui8_tx_buffer[1] = ui8_message_ID;
      
      // assist level
      if (p_motor_controller_data->ui8_walk_assist_level) // if walk assist function is enabled, send walk assist level factor
      {
        ui8_tx_buffer[2] = p_configuration_variables->ui8_walk_assist_level_factor [(p_configuration_variables->ui8_assist_level)];
      }
      else if (p_configuration_variables->ui8_assist_level) // send assist level factor for normal operation 
      {
        ui8_tx_buffer[2] = p_configuration_variables->ui8_assist_level_factor [((p_configuration_variables->ui8_assist_level) - 1)];
      }
      else // send nothing 
      {
        ui8_tx_buffer[2] = 0;
      }

      // set lights state
      // walk assist level state
      ui8_tx_buffer[3] = ((p_motor_controller_data->ui8_lights & 1) |
                         ((p_motor_controller_data->ui8_walk_assist_level & 1) << 1));
      
      // battery power limit
      if (p_configuration_variables->ui8_offroad_feature_enabled && p_configuration_variables->ui8_offroad_power_limit_enabled)
      {
        ui8_tx_buffer[4] = p_configuration_variables->ui8_offroad_power_limit_div25;
      }
      else
      {
        ui8_tx_buffer[4] = p_configuration_variables->ui8_target_max_battery_power_div25;
      }

      switch (ui8_message_ID)
      {
        case 0:
          // battery low voltage cut-off
          ui8_tx_buffer[5] = (uint8_t) (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 0xff);
          ui8_tx_buffer[6] = (uint8_t) (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8);
        break;

        case 1:
          // wheel perimeter
          ui8_tx_buffer[5] = (uint8_t) (p_configuration_variables->ui16_wheel_perimeter & 0xff);
          ui8_tx_buffer[6] = (uint8_t) (p_configuration_variables->ui16_wheel_perimeter >> 8);
        break;

        case 2:
          // wheel max speed
          if (p_configuration_variables->ui8_offroad_feature_enabled)
          {
            ui8_tx_buffer[5] = p_configuration_variables->ui8_offroad_speed_limit;
          }
          else
          {
            ui8_tx_buffer[5] = p_configuration_variables->ui8_wheel_max_speed;
          }
          
          // battery max current in amps
          ui8_tx_buffer[6] = p_configuration_variables->ui8_battery_max_current;
        break;

        case 3:
          // set motor type
          // enable/disable motor assistance without pedal rotation
          // enable/disable motor temperature limit function
          ui8_tx_buffer[5] = ((p_configuration_variables->ui8_motor_type & 3) |
                             ((p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation & 1) << 2) |
                             ((p_configuration_variables->ui8_temperature_limit_feature_enabled & 3) << 3));
                             
          // motor power boost startup state
          ui8_tx_buffer[6] = p_configuration_variables->ui8_startup_motor_power_boost_state;
        break;

        case 4:
          // startup motor power boost
          ui8_tx_buffer[5] = p_configuration_variables->ui8_startup_motor_power_boost_factor [((p_configuration_variables->ui8_assist_level) - 1)];
          
          // startup motor power boost time
          ui8_tx_buffer[6] = p_configuration_variables->ui8_startup_motor_power_boost_time;
        break;

        case 5:
          // startup motor power boost fade time
          ui8_tx_buffer[5] = p_configuration_variables->ui8_startup_motor_power_boost_fade_time;
          
          // boost feature enabled
          ui8_tx_buffer[6] = (p_configuration_variables->ui8_startup_motor_power_boost_feature_enabled & 1) ? 1 : 0;
        break;

        case 6:
          // motor over temperature min value limit
          ui8_tx_buffer[5] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
          
          // motor over temperature max value limit
          ui8_tx_buffer[6] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;
        break;
        
        case 7:
          // ramp up, amps per second
          ui8_tx_buffer[5] = p_configuration_variables->ui8_ramp_up_amps_per_second_x10;
          
          // cruise target speed
          if (p_configuration_variables->ui8_cruise_function_set_target_speed_enabled)
          {
            ui8_tx_buffer[6] = p_configuration_variables->ui8_cruise_function_target_speed_kph;
          }
          else
          {
            ui8_tx_buffer[6] = 0;
          }
        break;
        
        default:
          // nothing
        break;
      }

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
      
      // increment message ID for next package
      if (++ui8_message_ID > UART_MAX_NUMBER_MESSAGE_ID) { ui8_message_ID = 0; }

      // let's wait for 10 packages, seems that first ADC battery voltage is an incorrect value
      if (++ui8_uart_received_first_package > 10) { ui8_uart_received_first_package = 10; }
    }

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }
}


uint8_t uart_received_first_package (void)
{
  return (ui8_uart_received_first_package == 10) ? 1: 0;
}


#if __SDCC_REVISION < 9624
void putchar(char c)
{
  //Write a character to the UART2
  UART2_SendData8(c);

  //Loop until the end of transmission
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET) ;
}
#else
int putchar(int c)
{
  //Write a character to the UART2
  UART2_SendData8(c);

  //Loop until the end of transmission
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);

  return((unsigned char)c);
}
#endif

#if __SDCC_REVISION < 9989
char getchar(void)
#else
int getchar(void)
#endif
{
  uint8_t c = 0;

  /* Loop until the Read data register flag is SET */
  while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET) ;

  c = UART2_ReceiveData8();

  return (c);
}
