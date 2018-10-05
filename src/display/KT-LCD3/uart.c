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

volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[22];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[11];
volatile uint8_t ui8_tx_counter = 0;
volatile uint8_t ui8_i;
volatile uint8_t ui8_checksum;
static uint16_t ui16_crc_rx;
static uint16_t ui16_crc_tx;
static uint8_t ui8_lcd_variable_id = 0;
static uint8_t ui8_master_comm_package_id = 0;
static uint8_t ui8_slave_comm_package_id = 0;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
volatile uint8_t ui8_uart_received_first_package = 0;

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
      ui8_rx_counter++;

      // see if is the last byte of the package
      if (ui8_rx_counter > 23)
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

void clock_uart_data (void)
{
  static uint32_t ui32_wss_tick_temp;
  struct_motor_controller_data *p_motor_controller_data;
  struct_configuration_variables *p_configuration_variables;

  if (ui8_received_package_flag)
  {
    // validation of the package data
    // last byte is the checksum
    ui16_crc_rx = 0xffff;
    for (ui8_i = 0; ui8_i <= 19; ui8_i++)
    {
      crc16 (ui8_rx_buffer[ui8_i], &ui16_crc_rx);
    }

    if (((((uint16_t) ui8_rx_buffer [21]) << 8) + ((uint16_t) ui8_rx_buffer [20])) == ui16_crc_rx)
    {
      p_motor_controller_data = lcd_get_motor_controller_data ();
      p_configuration_variables = get_configuration_variables ();

      // send a variable for each package sent but first verify if the last one was received otherwise, keep repeating
      // keep cycling so all variables are sent
#define VARIABLE_ID_MAX_NUMBER 9
      if ((ui8_rx_buffer [1]) == ui8_master_comm_package_id) // last package data ID was receipt, so send the next one
      {
        ui8_master_comm_package_id = (ui8_master_comm_package_id + 1) % VARIABLE_ID_MAX_NUMBER;
      }

      ui8_slave_comm_package_id = ui8_rx_buffer[2];

      p_motor_controller_data->ui16_adc_battery_voltage = ui8_rx_buffer[3];
      p_motor_controller_data->ui16_adc_battery_voltage |= ((uint16_t) (ui8_rx_buffer[4] & 0x30)) << 4;
      p_motor_controller_data->ui8_battery_current_x5 = ui8_rx_buffer[5];
      p_motor_controller_data->ui16_wheel_speed_x10 = (((uint16_t) ui8_rx_buffer [7]) << 8) + ((uint16_t) ui8_rx_buffer [6]);
      p_motor_controller_data->ui8_motor_controller_state_2 = ui8_rx_buffer[8];
      p_motor_controller_data->ui8_braking = p_motor_controller_data->ui8_motor_controller_state_2 & 1;

      if (p_configuration_variables->ui8_throttle_adc_measures_motor_temperature)
      {
        p_motor_controller_data->ui8_adc_throttle = ui8_rx_buffer[9];
        p_motor_controller_data->ui8_motor_temperature = ui8_rx_buffer[10];
      }
      else
      {
        p_motor_controller_data->ui8_adc_throttle = ui8_rx_buffer[9];
        p_motor_controller_data->ui8_throttle = ui8_rx_buffer[10];
      }

      p_motor_controller_data->ui8_adc_pedal_torque_sensor = ui8_rx_buffer[11];
      p_motor_controller_data->ui8_pedal_torque_sensor = ui8_rx_buffer[12];
      p_motor_controller_data->ui8_pedal_cadence = ui8_rx_buffer[13];
      p_motor_controller_data->ui8_pedal_human_power = ui8_rx_buffer[14];
      p_motor_controller_data->ui8_duty_cycle = ui8_rx_buffer[15];
      p_motor_controller_data->ui16_motor_speed_erps = (((uint16_t) ui8_rx_buffer [17]) << 8) + ((uint16_t) ui8_rx_buffer [16]);
      p_motor_controller_data->ui8_foc_angle = ui8_rx_buffer[18];

      switch (ui8_slave_comm_package_id)
      {
        case 0:
          // error states
          p_motor_controller_data->ui8_error_code = ui8_rx_buffer[19];
        break;

        case 1:
          // temperature actual limiting value
          p_motor_controller_data->ui8_temperature_current_limiting_value = ui8_rx_buffer[19];
        break;

        case 2:
          // wheel_speed_sensor_tick_counter
          ui32_wss_tick_temp = ((uint32_t) ui8_rx_buffer[19]);
        break;

        case 3:
          // wheel_speed_sensor_tick_counter
          ui32_wss_tick_temp |= (((uint32_t) ui8_rx_buffer[19]) << 8);
        break;

        case 4:
          // wheel_speed_sensor_tick_counter
          ui32_wss_tick_temp |= (((uint32_t) ui8_rx_buffer[19]) << 16);
          p_motor_controller_data->ui32_wheel_speed_sensor_tick_counter = ui32_wss_tick_temp;
        break;
      }

      // signal that we processed the full package
      ui8_received_package_flag = 0;

      // now send the data to the motor controller
      // start up byte
      ui8_tx_buffer[0] = 0x59;
      ui8_tx_buffer[1] = ui8_master_comm_package_id;
      ui8_tx_buffer[2] = ui8_slave_comm_package_id;

      // set assist level value
      if (p_configuration_variables->ui8_assist_level)
      {
        ui8_tx_buffer[3] = p_configuration_variables->ui8_assist_level_power [((p_configuration_variables->ui8_assist_level) - 1)];
      }
      else
      {
        ui8_tx_buffer[3] = 0;
      }

      // set lights state
      // walk assist level state
      // set offroad state
      ui8_tx_buffer[4] = (p_motor_controller_data->ui8_lights & 1) |
          ((p_motor_controller_data->ui8_walk_assist_level & 1) << 1) |
          ((p_motor_controller_data->ui8_offroad_mode & 1) << 2);

      // battery max current in amps
      ui8_tx_buffer[5] = p_configuration_variables->ui8_battery_max_current;

      // battery power
      ui8_tx_buffer[6] = p_configuration_variables->ui8_target_max_battery_power;

      switch (ui8_master_comm_package_id)
      {
        case 0:
          // battery low voltage cut-off
          ui8_tx_buffer[7] = (uint8_t) (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 0xff);
          ui8_tx_buffer[8] = (uint8_t) (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8);
        break;

        case 1:
          // wheel perimeter
          ui8_tx_buffer[7] = (uint8_t) (p_configuration_variables->ui16_wheel_perimeter & 0xff);
          ui8_tx_buffer[8] = (uint8_t) (p_configuration_variables->ui16_wheel_perimeter >> 8);
        break;

        case 2:
          // wheel max speed
          ui8_tx_buffer[7] = p_configuration_variables->ui8_wheel_max_speed;
          // PAS_MAX_CADENCE_RPM
          ui8_tx_buffer[8] = p_configuration_variables->ui8_pas_max_cadence;
        break;

        case 3:
          // bit 0: cruise control
          // bit 1: motor voltage type: 36V or 48V
          // bit 2: MOTOR_ASSISTANCE_CAN_START_WITHOUT_PEDAL_ROTATION
          ui8_tx_buffer[7] = ((p_configuration_variables->ui8_cruise_control & 1) |
                             ((p_configuration_variables->ui8_motor_voltage_type & 1) << 1) |
                              ((p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation & 1) << 2) |
                              ((p_configuration_variables->ui8_throttle_adc_measures_motor_temperature & 1) << 3));
          ui8_tx_buffer[8] = p_configuration_variables->ui8_startup_motor_power_boost_state;
        break;

        case 4:
          // startup motor power boost
          ui8_tx_buffer[7] = p_configuration_variables->ui8_startup_motor_power_boost [((p_configuration_variables->ui8_assist_level) - 1)];
          // startup motor power boost time
          ui8_tx_buffer[8] = p_configuration_variables->ui8_startup_motor_power_boost_time;
        break;

        case 5:
          // startup motor power boost fade time
          ui8_tx_buffer[7] = p_configuration_variables->ui8_startup_motor_power_boost_fade_time;
        break;

        case 6:
          // motor over temperature min and max values to limit
          ui8_tx_buffer[7] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
          ui8_tx_buffer[8] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;
        break;

        case 7:
          // offroad mode configuration
          ui8_tx_buffer[7] = ((p_configuration_variables->ui8_offroad_func_enabled & 1) |
                                ((p_configuration_variables->ui8_offroad_enabled_on_startup & 1) << 1)); 
          ui8_tx_buffer[8] = p_configuration_variables->ui8_offroad_speed_limit;
        break;

        case 8:
          // offroad mode power limit configuration
          ui8_tx_buffer[7] = p_configuration_variables->ui8_offroad_power_limit_enabled & 1;
          ui8_tx_buffer[8] = p_configuration_variables->ui8_offroad_power_limit_div25;
        break;

        default:
          ui8_lcd_variable_id = 0;
        break;
      }

      // prepare crc of the package
      ui16_crc_tx = 0xffff;
      for (ui8_i = 0; ui8_i <= 8; ui8_i++)
      {
        crc16 (ui8_tx_buffer[ui8_i], &ui16_crc_tx);
      }
      ui8_tx_buffer[9] = (uint8_t) (ui16_crc_tx & 0xff);
      ui8_tx_buffer[10] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

      // send the full package to UART
      for (ui8_i = 0; ui8_i <= 10; ui8_i++)
      {
        putchar (ui8_tx_buffer[ui8_i]);
      }

      // let's wait for 10 packages, seems that first ADC battery voltage is an incorrect value
      ui8_uart_received_first_package++;
      if (ui8_uart_received_first_package > 10)
        ui8_uart_received_first_package = 10;
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
