/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <string.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "stm8s_iwdg.h"
#include "gpio.h"
#include "timers.h"
#include "ht162.h"
#include "lcd.h"
#include "adc.h"
#include "main.h"
#include "config.h"
#include "button.h"
#include "eeprom.h"
#include "pins.h"
#include "uart.h"

#define LCD_MENU_CONFIG_SUBMENU_MAX_NUMBER 10

uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

uint8_t ui8_lcd_field_offset[] = {
    ASSIST_LEVEL_DIGIT_OFFSET,
    ODOMETER_DIGIT_OFFSET,
    TEMPERATURE_DIGIT_OFFSET,
    WHEEL_SPEED_OFFSET,
    BATTERY_POWER_DIGIT_OFFSET,
    0
};

uint8_t ui8_lcd_digit_mask[] = {
    NUMBER_0_MASK,
    NUMBER_1_MASK,
    NUMBER_2_MASK,
    NUMBER_3_MASK,
    NUMBER_4_MASK,
    NUMBER_5_MASK,
    NUMBER_6_MASK,
    NUMBER_7_MASK,
    NUMBER_8_MASK,
    NUMBER_9_MASK
};

uint8_t ui8_lcd_digit_mask_inverted[] = {
    NUMBER_0_MASK_INVERTED,
    NUMBER_1_MASK_INVERTED,
    NUMBER_2_MASK_INVERTED,
    NUMBER_3_MASK_INVERTED,
    NUMBER_4_MASK_INVERTED,
    NUMBER_5_MASK_INVERTED,
    NUMBER_6_MASK_INVERTED,
    NUMBER_7_MASK_INVERTED,
    NUMBER_8_MASK_INVERTED,
    NUMBER_9_MASK_INVERTED
};

static uint32_t ui32_battery_voltage_accumulated_x10000 = 0;
static uint16_t ui16_battery_voltage_filtered_x10;

static uint16_t ui16_battery_current_accumulated_x5 = 0;
static uint16_t ui16_battery_current_filtered_x5;

static uint16_t ui16_battery_power_accumulated = 0;
static uint16_t ui16_battery_power_filtered_x50;
static uint16_t ui16_battery_power_filtered;

static uint32_t ui32_wh_sum_x5 = 0;
static uint32_t ui32_wh_sum_counter = 0;
static uint32_t ui32_wh_x10 = 0;
static uint8_t ui8_config_wh_x10_offset;

static uint32_t ui32_pedal_torque_accumulated = 0;
static uint16_t ui16_pedal_torque_filtered;
static uint32_t ui32_pedal_power_accumulated = 0;
static uint16_t ui16_pedal_power_filtered;

static uint16_t ui16_pedal_cadence_accumulated = 0;
static uint8_t ui8_pedal_cadence_filtered;

static uint8_t ui8_motor_controller_init = 1;

static uint8_t ui8_lights_state = 0;
static uint8_t lcd_lights_symbol = 0;

static uint8_t ui8_lcd_menu = 0;
static uint8_t ui8_lcd_menu_config_submenu_state = 0;
static uint8_t ui8_lcd_menu_flash_counter = 0;
static uint16_t ui16_lcd_menu_flash_counter_temperature = 0;
static uint8_t ui8_lcd_menu_flash_state;
static uint8_t ui8_lcd_menu_flash_state_temperature;
static uint8_t ui8_lcd_menu_config_submenu_number = 0;
static uint8_t ui8_lcd_menu_config_submenu_active = 0;

static uint8_t ui8_lcd_menu_counter_100ms = 0;
static uint8_t ui8_lcd_menu_counter_100ms_state = 0;

static uint8_t ui8_lcd_menu_counter_500ms = 0;
static uint8_t ui8_lcd_menu_counter_500ms_state = 0;

static struct_motor_controller_data motor_controller_data;
static struct_configuration_variables configuration_variables;

static uint16_t ui16_battery_soc_watts_hour;

static uint8_t ui8_reset_to_defaults_counter;
static uint8_t ui8_state_temp_field;

uint8_t ui8_lcd_power_off_time_counter_minutes = 0;
static uint16_t ui16_lcd_power_off_time_counter = 0;

static uint8_t offroad_mode_assist_symbol_state = 0;
static uint8_t offroad_mode_assist_symbol_state_blink_counter = 0;

static uint16_t ui16_battery_voltage_soc_x10;

static volatile uint16_t ui16_timer3_counter = 0;

void low_pass_filter_battery_voltage_current_power (void);
void lcd_enable_motor_symbol (uint8_t ui8_state);
void lcd_enable_lights_symbol (uint8_t ui8_state);
void lcd_enable_walk_symbol (uint8_t ui8_state);
void lcd_enable_km_symbol (uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state);
void lcd_enable_kmh_symbol (uint8_t ui8_state);
void lcd_enable_mph_symbol (uint8_t ui8_state);
void lcd_enable_odo_symbol (uint8_t ui8_state);
void calc_wh (void);
void assist_level_state (void);
void brake (void);
void odometer (void);
void odometer_increase_field_state (void);
void wheel_speed (void);
void power (void);
void power_off_management (void);
uint8_t first_time_management (void);
void temperature (void);
void battery_soc (void);
void calc_battery_voltage_soc (void);
void low_pass_filter_pedal_torque_and_power (void);
static void low_pass_filter_pedal_cadence (void);
void lights_state (void);
void lcd_set_backlight_intensity (uint8_t ui8_intensity);
void walk_assist_state (void);
void offroad_mode (void);
void lcd_execute_main_screen (void);
void lcd_execute_menu_config (void);
void lcd_execute_menu_config_power (void);
void lcd_execute_menu_config_submenu_wheel_config (void);
void lcd_execute_menu_config_submenu_battery (void);
void lcd_execute_menu_config_submenu_battery_soc (void);
void lcd_execute_menu_config_submenu_assist_level (void);
void lcd_execute_menu_config_submenu_motor_startup_power_boost (void);
void lcd_execute_menu_config_submenu_motor_temperature (void);
void lcd_execute_menu_config_submenu_lcd ();
void lcd_execute_menu_config_submenu_offroad_mode (void);
void lcd_execute_menu_config_submenu_various (void);
void lcd_execute_menu_config_submenu_technical (void);
void update_menu_flashing_state (void);
void advance_on_submenu (uint8_t* ui8_p_state, uint8_t ui8_state_max_number);
void calc_battery_soc_watts_hour (void);
void calc_odometer (void);
static void automatic_power_off_management (void);
void lcd_power_off (uint8_t updateDistanceOdo);
void lcd_enable_vol_symbol (uint8_t ui8_state);
void lcd_enable_w_symbol (uint8_t ui8_state);
void lcd_enable_odometer_point_symbol (uint8_t ui8_state);
void lcd_enable_brake_symbol (uint8_t ui8_state);
void lcd_enable_assist_symbol (uint8_t ui8_state);
void lcd_enable_battery_power_1_symbol (uint8_t ui8_state);
void lcd_enable_temperature_1_symbol (uint8_t ui8_state);
void lcd_enable_kmh_symbol (uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state);
void lcd_enable_temperature_degrees_symbol (uint8_t ui8_state);
void lcd_enable_dst_symbol (uint8_t ui8_state);
void lcd_enable_tm_symbol (uint8_t ui8_state);
void lcd_update (void);
void lcd_clear (void);
void lcd_set_frame_buffer (void);
void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options);

// happens every 1ms
void TIM3_UPD_OVF_BRK_IRQHandler(void) __interrupt(TIM3_UPD_OVF_BRK_IRQHANDLER)
{
  static uint8_t ui8_100ms_timmer_counter;

  ui16_timer3_counter++;

  // calc wh every 100ms
  if (ui8_100ms_timmer_counter++ >= 100)
  {
    ui8_100ms_timmer_counter = 0;

    // must be called every 100ms
    calc_wh();
  }

  TIM3_ClearITPendingBit(TIM3_IT_UPDATE); // clear Interrupt Pending bit
}

uint16_t get_timer3_counter(void)
{
  return ui16_timer3_counter;
}

void clock_lcd (void)
{
  lcd_clear (); // start by clear LCD
  if (first_time_management ())
    return;

  update_menu_flashing_state ();

  // enter menu configurations: UP + DOWN click event
  if (get_button_up_down_click_event () &&
      ui8_lcd_menu != 1)
  {
    clear_button_up_down_click_event ();
    ui8_lcd_menu = 1;
  }

  // enter in menu set power: ONOFF + UP click event
  if (!configuration_variables.ui8_offroad_feature_enabled && 
      get_button_onoff_state () && get_button_up_state ())
  {
    button_clear_events ();
    ui8_lcd_menu = 2;
  }

  // change temperature field state: ONOFF + DOWN click event
  if (!configuration_variables.ui8_offroad_feature_enabled && 
      get_button_onoff_state () && get_button_down_state ())
  {
    button_clear_events ();

    if (ui8_state_temp_field == 0)
    {
      configuration_variables.ui8_temperature_field_config++;

      if (configuration_variables.ui8_temperature_limit_feature_enabled)
      {
        if (configuration_variables.ui8_temperature_field_config > 2) { configuration_variables.ui8_temperature_field_config = 0; }
      }
      else
      {
        if (configuration_variables.ui8_temperature_field_config > 1) { configuration_variables.ui8_temperature_field_config = 0; }
      }

      ui8_state_temp_field = 1;
    }
  }
  else
  {
    ui8_state_temp_field = 0;
  }

  calc_battery_soc_watts_hour ();

  switch (ui8_lcd_menu)
  {
    case 0:
      lcd_execute_main_screen ();
    break;

    case 1:
      lcd_execute_menu_config ();
    break;

    case 2:
      lcd_execute_menu_config_power ();
    break;
  }

  low_pass_filter_battery_voltage_current_power ();
  // filter using only each 500ms values
  if (ui8_lcd_menu_counter_500ms_state)
  {
    low_pass_filter_pedal_cadence ();
  }

  // filter using only each 100ms values
  if (ui8_lcd_menu_counter_100ms_state)
  {
    low_pass_filter_pedal_torque_and_power ();
  }

  calc_battery_voltage_soc ();
  calc_odometer ();
  automatic_power_off_management ();

  lcd_update ();

  // power off system: ONOFF long click event
  power_off_management ();
}

void lcd_execute_main_screen (void)
{
  temperature ();
  assist_level_state ();
  odometer ();
  wheel_speed ();
  walk_assist_state ();
  offroad_mode ();
  power ();
  battery_soc ();
  lights_state ();
  brake ();
}

void lcd_execute_menu_config (void)
{
  // button check when submenu is not active
  if (!ui8_lcd_menu_config_submenu_active)
  {
    // leave config menu with a button_onoff_long_click
    if (get_button_onoff_long_click_event ())
    {
      clear_button_onoff_long_click_event ();
      ui8_lcd_menu = 0;

      // save the updated variables on EEPROM
      eeprom_write_variables ();

      return;
    }

    // advance on submenu on button_onoff_click_event
    advance_on_submenu (&ui8_lcd_menu_config_submenu_number, LCD_MENU_CONFIG_SUBMENU_MAX_NUMBER);

    // check if we should enter a submenu
    if (get_button_up_click_event () || get_button_down_click_event ())
    {
      clear_button_up_click_event ();
      clear_button_down_click_event ();

      ui8_lcd_menu_config_submenu_active = 1;
      ui8_config_wh_x10_offset = 1;
    }

    // print submenu number only half of the time
    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (ui8_lcd_menu_config_submenu_number, WHEEL_SPEED_FIELD, 1);
    }
  }
  // ui8_lcd_menu_config_submenu_active == 1
  else
  {
    switch (ui8_lcd_menu_config_submenu_number)
    {
      case 0:
        lcd_execute_menu_config_submenu_wheel_config ();
      break;

      case 1:
        lcd_execute_menu_config_submenu_battery ();
      break;

      case 2:
        lcd_execute_menu_config_submenu_battery_soc ();
      break;

      case 3:
        lcd_execute_menu_config_submenu_assist_level ();
      break;

      case 4:
        lcd_execute_menu_config_submenu_motor_startup_power_boost ();
      break;

      case 5:
        lcd_execute_menu_config_submenu_motor_temperature ();        
      break;

      case 6:
        lcd_execute_menu_config_submenu_lcd ();
      break;

      case 7:
        lcd_execute_menu_config_submenu_offroad_mode ();
      break;

      case 8:
        lcd_execute_menu_config_submenu_various ();
      break;

      case 9:
        lcd_execute_menu_config_submenu_technical ();
      break;

      default:
        ui8_lcd_menu_config_submenu_number = 0;
      break;
    }

    // leave config menu with a button_onoff_long_click
    if (get_button_onoff_long_click_event ())
    {
      clear_button_onoff_long_click_event ();

      ui8_lcd_menu_config_submenu_active = 0;
      ui8_lcd_menu_config_submenu_state = 0;
    }
  }
}

void lcd_execute_menu_config_submenu_wheel_config (void)
{
  // advance on submenus on button_onoff_click_event
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 3);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // menu to choose max wheel speed
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();

        configuration_variables.ui8_wheel_max_speed++;
        if (configuration_variables.ui8_wheel_max_speed >= 99) { configuration_variables.ui8_wheel_max_speed = 99; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();

        configuration_variables.ui8_wheel_max_speed--;
        if (configuration_variables.ui8_wheel_max_speed < 1)  { configuration_variables.ui8_wheel_max_speed = 1; }

      }

      // print wheel speed only half of the time
      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (((uint16_t) configuration_variables.ui8_wheel_max_speed) * 10, WHEEL_SPEED_FIELD, 0);
      }

      lcd_enable_kmh_symbol (1);
    break;

    // menu to choose wheel perimeter
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();

        configuration_variables.ui16_wheel_perimeter += 10;
        // max value is 3000 mm
        if (configuration_variables.ui16_wheel_perimeter >= 3000) { configuration_variables.ui16_wheel_perimeter = 3000; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();

        configuration_variables.ui16_wheel_perimeter -= 10;
        // min value is 750 mm
        if (configuration_variables.ui16_wheel_perimeter < 750) { configuration_variables.ui16_wheel_perimeter = 750; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui16_wheel_perimeter, ODOMETER_FIELD, 1);
      }
    break;

    // menu to choose Km/h or MP/h
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();

        configuration_variables.ui8_units_type++;
        if (configuration_variables.ui8_units_type > 1) { configuration_variables.ui8_units_type = 1; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();

        configuration_variables.ui8_units_type--;
        if (configuration_variables.ui8_units_type != 0) { configuration_variables.ui8_units_type = 0; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        if (configuration_variables.ui8_units_type)
          lcd_enable_mph_symbol (1);
        else
          lcd_enable_kmh_symbol (1);
      }
    break;
  }
}

void lcd_execute_menu_config_submenu_battery (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 5);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // battery max current
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();

        configuration_variables.ui8_battery_max_current++;
        if (configuration_variables.ui8_battery_max_current > 100) { configuration_variables.ui8_battery_max_current = 100; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();

        if (configuration_variables.ui8_battery_max_current > 0)
          configuration_variables.ui8_battery_max_current--;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_battery_max_current, ODOMETER_FIELD, 1);
      }
    break;

    // battery low voltage cut-off
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui16_battery_low_voltage_cut_off_x10 < 630) { configuration_variables.ui16_battery_low_voltage_cut_off_x10++; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui16_battery_low_voltage_cut_off_x10 > 161) { configuration_variables.ui16_battery_low_voltage_cut_off_x10--; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui16_battery_low_voltage_cut_off_x10, ODOMETER_FIELD, 0);
      }
    break;

    // battery cells number
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui8_battery_cells_number < 15) { configuration_variables.ui8_battery_cells_number++; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui8_battery_cells_number > 7) { configuration_variables.ui8_battery_cells_number--; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_battery_cells_number, ODOMETER_FIELD, 1);
      }
    break;

    // battery pack resistance
    case 3:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui16_battery_pack_resistance_x1000 < 1001) { configuration_variables.ui16_battery_pack_resistance_x1000++; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui16_battery_pack_resistance_x1000 > 0) { configuration_variables.ui16_battery_pack_resistance_x1000--; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui16_battery_pack_resistance_x1000, ODOMETER_FIELD, 1);
      }
    break;

    // battery voltage SOC
    case 4:
      lcd_print (ui16_battery_voltage_soc_x10, ODOMETER_FIELD, 0);
    break;
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_battery_soc (void)
{
  uint8_t ui8_temp;

  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 5);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // menu to enable/disable show of numeric watts hour value
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_show_numeric_battery_soc |= 1;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_show_numeric_battery_soc &= ~1;
      }

      ui8_temp = ((configuration_variables.ui8_show_numeric_battery_soc & 1) ? 1 : 0);
      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (ui8_temp, ODOMETER_FIELD, 1);
      }
    break;

    // menu to enable/disable show of numeric watts hour value in incrementing or decementing percentage
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_show_numeric_battery_soc |= 2;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_show_numeric_battery_soc &= ~2;
      }

      ui8_temp = ((configuration_variables.ui8_show_numeric_battery_soc & 2) ? 1 : 0);
      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (ui8_temp, ODOMETER_FIELD, 1);
      }
    break;

    // battery_voltage_reset_wh_counter
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui16_battery_voltage_reset_wh_counter_x10 < 630) { configuration_variables.ui16_battery_voltage_reset_wh_counter_x10++; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui16_battery_voltage_reset_wh_counter_x10 > 161) { configuration_variables.ui16_battery_voltage_reset_wh_counter_x10--; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui16_battery_voltage_reset_wh_counter_x10, ODOMETER_FIELD, 0);
      }
    break;

    // menu to choose watts hour value to be equal to 100% of battery SOC
    case 3:
      if (get_button_up_click_event ())
      {
        button_clear_events ();

        // increment at 10 units
        configuration_variables.ui32_wh_x10_100_percent += 100;
        if (configuration_variables.ui32_wh_x10_100_percent > 99900) { configuration_variables.ui32_wh_x10_100_percent = 99900; }
      }

      if (get_button_down_click_event ())
      {
        button_clear_events ();

        configuration_variables.ui32_wh_x10_100_percent -= 100;
        if (configuration_variables.ui32_wh_x10_100_percent < 100) { configuration_variables.ui32_wh_x10_100_percent = 0; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui32_wh_x10_100_percent, ODOMETER_FIELD, 0);
      }
    break;

    // menu to set current watts hour value
    case 4:
      // on the very first time, use current value of ui32_wh_x10
      if (ui8_config_wh_x10_offset)
      {
        ui8_config_wh_x10_offset = 0;
        configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
      }
      // keep reseting this values
      ui32_wh_sum_x5 = 0;
      ui32_wh_sum_counter = 0;
      ui32_wh_x10 = 0;

      if (get_button_up_click_event ())
      {
        button_clear_events ();

        // increment at 10 units
        configuration_variables.ui32_wh_x10_offset += 100;
        if (configuration_variables.ui32_wh_x10_offset > 99900) { configuration_variables.ui32_wh_x10_offset = 99900; }
      }

      if (get_button_down_click_event ())
      {
        button_clear_events ();

        if (configuration_variables.ui32_wh_x10_offset > 100)
        {
          configuration_variables.ui32_wh_x10_offset -= 100;
        }
        else if (configuration_variables.ui32_wh_x10_offset > 0)
        {
          configuration_variables.ui32_wh_x10_offset = 0;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui32_wh_x10_offset, ODOMETER_FIELD, 0);
      }
    break;
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_assist_level (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, (configuration_variables.ui8_number_of_assist_levels + 1));

  // number of assist levels: 0 to 9
  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      if (configuration_variables.ui8_number_of_assist_levels < 9) { configuration_variables.ui8_number_of_assist_levels++; }
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      if (configuration_variables.ui8_number_of_assist_levels > 1) { configuration_variables.ui8_number_of_assist_levels--; }
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (configuration_variables.ui8_number_of_assist_levels, ODOMETER_FIELD, 1);
    }
  }
  // value of each assist level
  else
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      configuration_variables.ui8_assist_level_power [(ui8_lcd_menu_config_submenu_state - 1)]++;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_assist_level_power [(ui8_lcd_menu_config_submenu_state - 1)]--;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (configuration_variables.ui8_assist_level_power [ui8_lcd_menu_config_submenu_state - 1] * 25, ODOMETER_FIELD, 1);
    }
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_motor_startup_power_boost (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, (configuration_variables.ui8_number_of_assist_levels + 5));

  // feature enable or disable
  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_feature_enabled = 1;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_feature_enabled = 0;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (configuration_variables.ui8_startup_motor_power_boost_feature_enabled, ODOMETER_FIELD, 1);
    }
  }
  // enabled on startup when wheel speed is zero or always when cadence was zero
  else if (ui8_lcd_menu_config_submenu_state == 1)
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_state |= 1;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_state &= ~1;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print ((configuration_variables.ui8_startup_motor_power_boost_state & 1) ? 1: 0, ODOMETER_FIELD, 1);
    }
  }
  // limit to max power
  else if (ui8_lcd_menu_config_submenu_state == 2)
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_state |= 2;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_state &= ~2;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print ((configuration_variables.ui8_startup_motor_power_boost_state & 2) ? 1: 0, ODOMETER_FIELD, 1);
    }
  }
  // startup motor power boost time
  else if (ui8_lcd_menu_config_submenu_state == 3)
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_time++;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_time--;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (configuration_variables.ui8_startup_motor_power_boost_time, ODOMETER_FIELD, 0);
    }
  }
  // startup motor power boost fade time
  else if (ui8_lcd_menu_config_submenu_state == 4)
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_fade_time++;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_startup_motor_power_boost_fade_time--;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (configuration_variables.ui8_startup_motor_power_boost_fade_time, ODOMETER_FIELD, 0);
    }
  }
  // value of each assist level power boost
  else
  {
    if (get_button_up_click_event ())
    {
      clear_button_up_click_event ();
      // the BATTERY_POWER_FIELD can't show higher value
      configuration_variables.ui8_startup_motor_power_boost [(ui8_lcd_menu_config_submenu_state - 5)]++;
    }

    if (get_button_down_click_event ())
    {
      clear_button_down_click_event ();
      configuration_variables.ui8_startup_motor_power_boost [(ui8_lcd_menu_config_submenu_state - 5)]--;
    }

    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (configuration_variables.ui8_startup_motor_power_boost [ui8_lcd_menu_config_submenu_state - 5] * 25, ODOMETER_FIELD, 1);
    }
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_motor_temperature (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 3);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // motor temperature enable
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_temperature_limit_feature_enabled = 1;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_temperature_limit_feature_enabled = 0;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_temperature_limit_feature_enabled, ODOMETER_FIELD, 1);
      }
    break;

    // motor temperature limit min
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui8_motor_temperature_min_value_to_limit < 110)
        {
          configuration_variables.ui8_motor_temperature_min_value_to_limit++;
        }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui8_motor_temperature_min_value_to_limit > 0)
        {
          configuration_variables.ui8_motor_temperature_min_value_to_limit--;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_motor_temperature_min_value_to_limit, ODOMETER_FIELD, 1);
      }
    break;

    // motor temperature limit max
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui8_motor_temperature_max_value_to_limit < 110)
        {
          configuration_variables.ui8_motor_temperature_max_value_to_limit++;
        }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui8_motor_temperature_max_value_to_limit > 0)
        {
          configuration_variables.ui8_motor_temperature_max_value_to_limit--;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_motor_temperature_max_value_to_limit, ODOMETER_FIELD, 1);
      }
    break;
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_lcd (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 4);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // backlight off brightness
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui8_lcd_backlight_off_brightness < 20)
        {
          configuration_variables.ui8_lcd_backlight_off_brightness++;
        }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui8_lcd_backlight_off_brightness > 0)
        {
          configuration_variables.ui8_lcd_backlight_off_brightness--;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        // * 5 to show a value from 0 to 100% in steps of 5%
        lcd_print (configuration_variables.ui8_lcd_backlight_off_brightness * 5, ODOMETER_FIELD, 1);
      }
    break;

    // backlight on brightness
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        if (configuration_variables.ui8_lcd_backlight_on_brightness < 20)
        {
          configuration_variables.ui8_lcd_backlight_on_brightness++;
        }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (configuration_variables.ui8_lcd_backlight_on_brightness > 0)
        {
          configuration_variables.ui8_lcd_backlight_on_brightness--;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        // * 5 to show a value from 0 to 100% in steps of 5%
        lcd_print (configuration_variables.ui8_lcd_backlight_on_brightness * 5, ODOMETER_FIELD, 1);
      }
    break;

    // auto power off
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_lcd_power_off_time_minutes++;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_lcd_power_off_time_minutes--;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_lcd_power_off_time_minutes, ODOMETER_FIELD, 1);
      }
    break;

    // reset to defaults
    case 3:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();

        ui8_reset_to_defaults_counter++;
        if (ui8_reset_to_defaults_counter > 9)
        {
          eeprom_erase_key_value ();
          
          // Turn off LCD, when the user turns it on again it will rewrite the defaults
          lcd_power_off (0);
        }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (ui8_reset_to_defaults_counter > 0)
        {
          ui8_reset_to_defaults_counter--;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (ui8_reset_to_defaults_counter, ODOMETER_FIELD, 1);
      }
    break;
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_offroad_mode (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 5);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // enable/disable offroad functionality
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_offroad_feature_enabled |= 1;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_offroad_feature_enabled &= ~1;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print ((configuration_variables.ui8_offroad_feature_enabled & 1) ? 1: 0, ODOMETER_FIELD, 1);
      }

      lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
    break;

    // enable offroad mode on system startup
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_offroad_enabled_on_startup |= 1;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_offroad_enabled_on_startup &= ~1;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print ((configuration_variables.ui8_offroad_enabled_on_startup & 1) ? 1: 0, ODOMETER_FIELD, 1);
      }

      lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
    break;

    // offroad speed limit (when offroad mode is off)
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_offroad_speed_limit++;
        if (configuration_variables.ui8_offroad_speed_limit > 99)  { configuration_variables.ui8_offroad_speed_limit = 99; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_offroad_speed_limit--;
        if (configuration_variables.ui8_offroad_speed_limit < 1)  { configuration_variables.ui8_offroad_speed_limit = 1; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (((uint16_t) configuration_variables.ui8_offroad_speed_limit) * 10, WHEEL_SPEED_FIELD, 0);
      }

      lcd_enable_kmh_symbol (1);
    break;

    // enable/disable power limit
    case 3:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_offroad_power_limit_enabled |= 1;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_offroad_power_limit_enabled &= ~1;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print ((configuration_variables.ui8_offroad_power_limit_enabled & 1) ? 1: 0, ODOMETER_FIELD, 1);
      }

      lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
    break;

    // power limit (W)
    case 4:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_offroad_power_limit_div25++;
        if (configuration_variables.ui8_offroad_power_limit_div25 > 40)  { configuration_variables.ui8_offroad_power_limit_div25 = 40; }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_offroad_power_limit_div25--;
        if (configuration_variables.ui8_offroad_power_limit_div25 < 4)  { configuration_variables.ui8_offroad_power_limit_div25 = 4; }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_offroad_power_limit_div25 * 25, ODOMETER_FIELD, 1);
      }

      lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
    break;
  }
}

void lcd_execute_menu_config_submenu_various (void)
{
  uint8_t ui8_temp;

  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 4);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // motor voltage type
    case 0:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_motor_type++;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_motor_type--;
      }

      if (configuration_variables.ui8_motor_type > 2) { configuration_variables.ui8_motor_type = 2; }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_motor_type, ODOMETER_FIELD, 1);
      }
    break;

    // motor assistance startup without pedal rotation
    case 1:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation |= 1;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation &= ~1;
      }

      ui8_temp = ((configuration_variables.ui8_motor_assistance_startup_without_pedal_rotation & 1) ? 1 : 0);
      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (ui8_temp, ODOMETER_FIELD, 1);
      }
    break;

    // PAS max cadence
    case 2:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();
        configuration_variables.ui8_pas_max_cadence++;
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        configuration_variables.ui8_pas_max_cadence--;
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (configuration_variables.ui8_pas_max_cadence, ODOMETER_FIELD, 1);
      }
    break;

    // Reset trip distance and total trip distance
    case 3:
      if (get_button_up_click_event ())
      {
        clear_button_up_click_event ();

        ui8_reset_to_defaults_counter++;
        if (ui8_reset_to_defaults_counter > 9)
        {
          configuration_variables.ui32_odometer_x10 = 0;
          configuration_variables.ui16_odometer_distance_x10 = 0;
          ui8_reset_to_defaults_counter = 0;
        }
      }

      if (get_button_down_click_event ())
      {
        clear_button_down_click_event ();
        if (ui8_reset_to_defaults_counter > 0)
        {
          ui8_reset_to_defaults_counter--;
        }
      }

      if (ui8_lcd_menu_flash_state)
      {
        lcd_print (ui8_reset_to_defaults_counter, ODOMETER_FIELD, 1);
      }
    break;
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_submenu_technical (void)
{
  advance_on_submenu (&ui8_lcd_menu_config_submenu_state, 9);

  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
      lcd_print (motor_controller_data.ui8_adc_throttle, ODOMETER_FIELD, 1);
    break;

    case 1:
      lcd_print (motor_controller_data.ui8_throttle, ODOMETER_FIELD, 1);
    break;

    case 2:
      lcd_print (motor_controller_data.ui8_adc_pedal_torque_sensor, ODOMETER_FIELD, 1);
    break;

    case 3:
      lcd_print (motor_controller_data.ui8_pedal_torque_sensor, ODOMETER_FIELD, 1);
    break;

    case 4:
      lcd_print (motor_controller_data.ui8_pedal_cadence, ODOMETER_FIELD, 1);
    break;

    case 5:
      lcd_print (motor_controller_data.ui8_pedal_human_power, ODOMETER_FIELD, 1);
    break;

    case 6:
      lcd_print (motor_controller_data.ui8_duty_cycle, ODOMETER_FIELD, 1);
    break;

    case 7:
      lcd_print (motor_controller_data.ui16_motor_speed_erps, ODOMETER_FIELD, 1);
    break;

    case 8:
      lcd_print (motor_controller_data.ui8_foc_angle, ODOMETER_FIELD, 1);
    break;
  }

  lcd_print (ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 1);
}

void lcd_execute_menu_config_power (void)
{
  // because this click envent can happens and will block the detection of button_onoff_long_click_event
  clear_button_onoff_click_event ();

  // leave this menu with a button_onoff_long_click
  if (get_button_onoff_long_click_event ())
  {
    button_clear_events ();
    ui8_lcd_menu = 0;

    // save the updated variables on EEPROM
    eeprom_write_variables ();
  }

  if (get_button_up_click_event ())
  {
    button_clear_events ();

    if (configuration_variables.ui8_target_max_battery_power < 10)
    {
      configuration_variables.ui8_target_max_battery_power++;
    }
    else
    {
      configuration_variables.ui8_target_max_battery_power += 2;
    }

    // the BATTERY_POWER_FIELD can't show higher value
    if (configuration_variables.ui8_target_max_battery_power > 190) { configuration_variables.ui8_target_max_battery_power = 190; }
  }

  if (get_button_down_click_event ())
  {
    button_clear_events ();

    if (configuration_variables.ui8_target_max_battery_power == 0)
    {

    }
    else if (configuration_variables.ui8_target_max_battery_power <= 10)
    {
      configuration_variables.ui8_target_max_battery_power--;
    }
    else
    {
      configuration_variables.ui8_target_max_battery_power -= 2;
    }
  }

  if (ui8_lcd_menu_flash_state)
  {
    lcd_print (configuration_variables.ui8_target_max_battery_power * 25, BATTERY_POWER_FIELD, 0);
  }
}

uint8_t first_time_management (void)
{
  uint8_t ui8_status = 0;

  // don't update LCD up to we get first communication package from the motor controller
  if (ui8_motor_controller_init &&
      (uart_received_first_package () == 0))
  {
    ui8_status = 1;
  }
  // this will be executed only 1 time at startup
  else if (ui8_motor_controller_init)
  {
    ui8_motor_controller_init = 0;

    // reset Wh value if battery voltage is over ui16_battery_voltage_reset_wh_counter_x10 (value configured by user)
    if (((uint32_t) motor_controller_data.ui16_adc_battery_voltage *
        ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000) > ((uint32_t) configuration_variables.ui16_battery_voltage_reset_wh_counter_x10 * 1000))
    {
      configuration_variables.ui32_wh_x10_offset = 0;
    }

    if (configuration_variables.ui8_offroad_feature_enabled && 
      configuration_variables.ui8_offroad_enabled_on_startup)
    {
      motor_controller_data.ui8_offroad_mode = 1;
    }
  }  

  return ui8_status;
}

void power_off_management (void)
{
  // turn off
  if (get_button_onoff_long_click_event ()) { lcd_power_off (1); }
}

void temperature (void)
{
  // if motor current is being limited due to temperature, force showing temperature!!
  if (motor_controller_data.ui8_temperature_current_limiting_value != 255)
  {
    if (ui8_lcd_menu_flash_state_temperature)
    {
      lcd_print (motor_controller_data.ui8_motor_temperature, TEMPERATURE_FIELD, 0);
      lcd_enable_temperature_degrees_symbol (1);
    }
  }
  else
  {
    switch (configuration_variables.ui8_temperature_field_config)
    {
      // show nothing
      case 0:
      break;

      // show battery_soc_watts_hour
      case 1:
        lcd_print (ui16_battery_soc_watts_hour, TEMPERATURE_FIELD, 0);
      break;

      // show motor temperature
      case 2:
        lcd_print (motor_controller_data.ui8_motor_temperature, TEMPERATURE_FIELD, 0);
        lcd_enable_temperature_degrees_symbol (1);
      break;
    }
  }
}

void battery_soc (void)
{
  static uint8_t ui8_timmer_counter;
  static uint8_t ui8_battery_state_of_charge;
  uint8_t ui8_battery_cells_number_x10;

  // update battery level value only at every 100ms / 10 times per second and this helps to visual filter the fast changing values
  if (ui8_timmer_counter++ >= 10)
  {
    ui8_timmer_counter = 0;

    // to keep same scale as voltage of x10
    ui8_battery_cells_number_x10 = configuration_variables.ui8_battery_cells_number * 10;

    if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_83))) { ui8_battery_state_of_charge = 4; } // 4 bars | full
    else if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_50))) { ui8_battery_state_of_charge = 3; } // 3 bars
    else if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_17))) { ui8_battery_state_of_charge = 2; } // 2 bars
    else if (ui16_battery_voltage_soc_x10 > ((uint16_t) ((float) ui8_battery_cells_number_x10 * LI_ION_CELL_VOLTS_0))) { ui8_battery_state_of_charge = 1; } // 1 bar
    else { ui8_battery_state_of_charge = 0; } // flashing
  }

  /*
    ui8_lcd_frame_buffer[23] |= 16;  // empty
    ui8_lcd_frame_buffer[23] |= 128; // bar number 1
    ui8_lcd_frame_buffer[23] |= 1;   // bar number 2
    ui8_lcd_frame_buffer[23] |= 64;  // bar number 3
    ui8_lcd_frame_buffer[23] |= 32;  // bar number 4
    */

  // first clean battery symbols
  ui8_lcd_frame_buffer[23] &= ~241;

  switch (ui8_battery_state_of_charge)
  {
    case 0:
    // empty, so flash the empty battery symbol
    if (ui8_lcd_menu_flash_state)
    {
      ui8_lcd_frame_buffer[23] |= 16;
    }
    break;

    case 1:
      ui8_lcd_frame_buffer[23] |= 144;
    break;

    case 2:
      ui8_lcd_frame_buffer[23] |= 145;
    break;

    case 3:
      ui8_lcd_frame_buffer[23] |= 209;
    break;

    case 4:
      ui8_lcd_frame_buffer[23] |= 241;
    break;
  }
}

void calc_battery_voltage_soc (void)
{
  uint16_t ui16_fluctuate_battery_voltage_x10;

  // update battery level value only at every 100ms / 10 times per second and this helps to visual filter the fast changing values
  if (ui8_lcd_menu_counter_100ms_state)
  {
    // calculate flutuate voltage, that depends on the current and battery pack resistance
    ui16_fluctuate_battery_voltage_x10 = (uint16_t) ((((uint32_t) configuration_variables.ui16_battery_pack_resistance_x1000) * ((uint32_t) ui16_battery_current_filtered_x5)) / ((uint32_t) 500));
    // now add fluctuate voltage value
    ui16_battery_voltage_soc_x10 = ui16_battery_voltage_filtered_x10 + ui16_fluctuate_battery_voltage_x10;
  }
}

void power (void)
{
  lcd_print (ui16_battery_power_filtered, BATTERY_POWER_FIELD, 0);
  lcd_enable_motor_symbol (1);
  lcd_enable_w_symbol (1);
}

void assist_level_state (void)
{
  if (get_button_up_click_event ())
  {
    clear_button_up_click_event ();

    configuration_variables.ui8_assist_level++;
    if (configuration_variables.ui8_assist_level > configuration_variables.ui8_number_of_assist_levels)
      { configuration_variables.ui8_assist_level = configuration_variables.ui8_number_of_assist_levels; }
  }

  if (get_button_down_click_event ())
  {
    clear_button_down_click_event ();

    if (configuration_variables.ui8_assist_level > 0)
      configuration_variables.ui8_assist_level--;
  }

  lcd_print (configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD, 0);

  if (motor_controller_data.ui8_offroad_mode == 0)
  {
    lcd_enable_assist_symbol (1);
  }
}

void lights_state (void)
{
  if (get_button_up_long_click_event ())
  {
    clear_button_up_long_click_event ();

    if (ui8_lights_state == 0)
    {
      ui8_lights_state = 1;
      lcd_lights_symbol = 1;
      motor_controller_data.ui8_lights = 1;
    }
    else
    {
      ui8_lights_state = 0;
      lcd_lights_symbol = 0;
      motor_controller_data.ui8_lights = 0;
    }
  }

  if (ui8_lights_state == 0) { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_off_brightness); }
  else { lcd_set_backlight_intensity (configuration_variables.ui8_lcd_backlight_on_brightness); }
  lcd_enable_lights_symbol (lcd_lights_symbol);
}

void walk_assist_state (void)
{
  if (get_button_down_long_click_event ())
  {
    // user need to keep pressing the button to have walk assist
    if (get_button_down_state ())
    {
      motor_controller_data.ui8_walk_assist_level = 1;
      lcd_enable_walk_symbol (1);
    }
    else
    {
      motor_controller_data.ui8_walk_assist_level = 0;
      clear_button_down_long_click_event ();
    }
  }
}

void offroad_mode (void)
{
  if (configuration_variables.ui8_offroad_feature_enabled) 
  {
    if (get_button_onoff_state () && get_button_up_state ())
    {
      button_clear_events ();
      motor_controller_data.ui8_offroad_mode = 1;
    }

    if (get_button_onoff_state () && get_button_down_state ())
    {
      button_clear_events ();
      motor_controller_data.ui8_offroad_mode = 0;
    }

    if (motor_controller_data.ui8_offroad_mode == 1) 
    {
      if (offroad_mode_assist_symbol_state_blink_counter++ > 50)
      {
        offroad_mode_assist_symbol_state_blink_counter = 0;
        offroad_mode_assist_symbol_state = !offroad_mode_assist_symbol_state;
      }

      lcd_enable_assist_symbol (offroad_mode_assist_symbol_state);
    }
  }
}

void brake (void)
{
  if (motor_controller_data.ui8_braking) { lcd_enable_brake_symbol (1); }
  else { lcd_enable_brake_symbol (0); }
}

void odometer_increase_field_state (void)
{
  configuration_variables.ui8_odometer_field_state = (configuration_variables.ui8_odometer_field_state + 1) % 10;
}

void odometer (void)
{
  uint32_t uint32_temp;

  // odometer values
  if (get_button_onoff_click_event ())
  {
    clear_button_onoff_click_event ();
    odometer_increase_field_state ();
  }

  // if there are errors, show the error number on odometer field instead of any other information
  if (motor_controller_data.ui8_error_states != ERROR_STATE_NO_ERRORS)
  {
    if (ui8_lcd_menu_flash_state)
    {
      lcd_print (motor_controller_data.ui8_error_states, ODOMETER_FIELD, 1);
    }
  }
  else
  {
    switch (configuration_variables.ui8_odometer_field_state)
    {
      // DST Single Trip Distance OR
      case 0:
        lcd_print ((uint32_t) configuration_variables.ui16_odometer_distance_x10, ODOMETER_FIELD, 0);
  //      lcd_enable_dst_symbol (1); TODO: this fails, the symbol just work wehn we set it to 1 AND speed field number is equal or higher than 10.0. Seems the 3rd digit at left is needed.
        lcd_enable_km_symbol (1);
      break;

      // ODO Total Trip Distance
      case 1:
        uint32_temp = configuration_variables.ui32_odometer_x10 + ((uint32_t) configuration_variables.ui16_odometer_distance_x10);
        lcd_print (uint32_temp, ODOMETER_FIELD, 0);
        lcd_enable_odo_symbol (1);
        lcd_enable_km_symbol (1);
      break;

      // voltage value
      case 2:
        lcd_print (ui16_battery_voltage_filtered_x10, ODOMETER_FIELD, 0);
        lcd_enable_vol_symbol (1);
      break;

      // current value
      case 3:
        lcd_print (ui16_battery_current_filtered_x5 << 1, ODOMETER_FIELD, 0);
      break;

      // Wh value
      case 4:
        lcd_print (ui32_wh_x10, ODOMETER_FIELD, 0);
      break;

    // battery SOC in watts/hour
    case 5:
      if (configuration_variables.ui8_show_numeric_battery_soc & 1)
      {
        lcd_print (ui16_battery_soc_watts_hour, ODOMETER_FIELD, 1);
      }
      else
      {
        odometer_increase_field_state ();
      }
    break;

    // pedal cadence value
    case 6:
      lcd_print (ui8_pedal_cadence_filtered, ODOMETER_FIELD, 1);
    break;

    // pedal torque
    case 7:
      lcd_print (ui16_pedal_torque_filtered, ODOMETER_FIELD, 1);
    break;

    // pedal power
    case 8:
      lcd_print (ui16_pedal_power_filtered, ODOMETER_FIELD, 1);
    break;

    // motor temperature
    case 9:
      if (configuration_variables.ui8_temperature_limit_feature_enabled)
      {
        lcd_print (motor_controller_data.ui8_motor_temperature, ODOMETER_FIELD, 1);
      }
      else
      {
        odometer_increase_field_state ();
      }
    break;

    default:
    configuration_variables.ui8_odometer_field_state = 0;
    break;
  }
}

void wheel_speed (void)
{
  if (configuration_variables.ui8_units_type)
  {
    lcd_print (((float) motor_controller_data.ui16_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 0);
    lcd_enable_mph_symbol (1);
  }
  else
  {
    lcd_print (motor_controller_data.ui16_wheel_speed_x10, WHEEL_SPEED_FIELD, 0);
    lcd_enable_kmh_symbol (1);
  }
}

void lcd_clear (void)
{
  memset(ui8_lcd_frame_buffer, 0, LCD_FRAME_BUFFER_SIZE);
}

void lcd_set_frame_buffer (void)
{
  memset(ui8_lcd_frame_buffer, 255, LCD_FRAME_BUFFER_SIZE);
}

void lcd_update (void)
{
  ht1622_send_frame_buffer (ui8_lcd_frame_buffer);
}

void lcd_print (uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options)
{
  uint8_t ui8_digit;
  uint8_t ui8_counter;

  // let's multiply the number by 10 to not show decimal digit
  if (ui8_options == 1)
  {
    ui32_number *= 10;
  }

  // first delete the field
  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    if (ui8_lcd_field == ASSIST_LEVEL_FIELD ||
            ui8_lcd_field == ODOMETER_FIELD ||
            ui8_lcd_field == TEMPERATURE_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD ||
        ui8_lcd_field == BATTERY_POWER_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;
    }

    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TEMPERATURE_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
  }

  // enable only the "1" if power is >= 1000
  if (ui8_lcd_field == BATTERY_POWER_FIELD)
  {
    if (ui32_number >= 1000) { lcd_enable_battery_power_1_symbol (1); }
    else { lcd_enable_battery_power_1_symbol (0); }
  }

  // enable only the "1" if temperature is >= 100
  if (ui8_lcd_field == TEMPERATURE_FIELD)
  {
    if (ui32_number >= 100) { lcd_enable_temperature_1_symbol (1); }
    else { lcd_enable_temperature_1_symbol (0); }
  }

  // do not show the point symbol if number*10 is integer
  if (ui8_options == 1)
  {
    if (ui8_lcd_field == ODOMETER_FIELD) { lcd_enable_odometer_point_symbol (0); }
    else if (ui8_lcd_field == WHEEL_SPEED_FIELD) { lcd_enable_wheel_speed_point_symbol (0); }
  }
  else
  {
    if (ui8_lcd_field == ODOMETER_FIELD) { lcd_enable_odometer_point_symbol (1); }
    else if (ui8_lcd_field == WHEEL_SPEED_FIELD) { lcd_enable_wheel_speed_point_symbol (1); }
  }

  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    ui8_digit = ui32_number % 10;

    if (ui8_lcd_field == ASSIST_LEVEL_FIELD ||
            ui8_lcd_field == ODOMETER_FIELD ||
            ui8_lcd_field == TEMPERATURE_FIELD)
    {

      if ((ui8_options == 1) &&
          (ui8_counter == 0))
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      // print empty (NUMBERS_MASK) when ui32_number = 0
      else if ((ui8_counter > 1 && ui32_number == 0) ||
          // TEMPERATURE_FIELD: print 1 zero only when value is less than 10
          (ui8_lcd_field == TEMPERATURE_FIELD && ui8_counter > 0 && ui32_number == 0))
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      else
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
      }
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD ||
        ui8_lcd_field == BATTERY_POWER_FIELD)
    {
      if (ui8_lcd_field == WHEEL_SPEED_FIELD)
      {
        if ((ui8_options == 1) &&
            (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        // print only first 2 zeros
        else if (ui8_counter > 1 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
      }

      if (ui8_lcd_field == BATTERY_POWER_FIELD)
      {
        // print only first zero
        if (ui8_counter > 0 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
      }
    }

    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TEMPERATURE_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;

    ui32_number /= 10;
  }
}

void lcd_enable_w_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 128;
  else
    ui8_lcd_frame_buffer[9] &= ~128;
}

void lcd_enable_odometer_point_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[6] |= 8;
  else
    ui8_lcd_frame_buffer[6] &= ~8;
}

void lcd_enable_brake_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 4;
  else
    ui8_lcd_frame_buffer[23] &= ~4;
}

void lcd_enable_lights_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[23] |= 2;
  else
    ui8_lcd_frame_buffer[23] &= ~2;
}

void lcd_enable_cruise_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[0] |= 16;
  else
    ui8_lcd_frame_buffer[0] &= ~16;
}

void lcd_enable_assist_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[1] |= 8;
  else
    ui8_lcd_frame_buffer[1] &= ~8;
}

void lcd_enable_vol_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[2] |= 8;
  else
    ui8_lcd_frame_buffer[2] &= ~8;
}

void lcd_enable_odo_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[3] |= 8;
  else
    ui8_lcd_frame_buffer[3] &= ~8;
}

void lcd_enable_km_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[4] |= 8;
  else
    ui8_lcd_frame_buffer[4] &= ~8;
}

void lcd_enable_mil_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[5] |= 8;
  else
    ui8_lcd_frame_buffer[5] &= ~8;
}

void lcd_enable_temperature_1_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[7] |= 8;
  else
    ui8_lcd_frame_buffer[7] &= ~8;
}

void lcd_enable_battery_power_1_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[12] |= 8;
  else
    ui8_lcd_frame_buffer[12] &= ~8;
}

void lcd_enable_temperature_minus_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[8] |= 8;
  else
    ui8_lcd_frame_buffer[8] &= ~8;
}

void lcd_enable_temperature_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 16;
  else
    ui8_lcd_frame_buffer[9] &= ~16;
}

void lcd_enable_temperature_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 32;
  else
    ui8_lcd_frame_buffer[9] &= ~32;
}

void lcd_enable_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 1;
  else
    ui8_lcd_frame_buffer[9] &= ~1;
}

void lcd_enable_motor_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 2;
  else
    ui8_lcd_frame_buffer[9] &= ~2;
}

void lcd_enable_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[9] |= 64;
  else
    ui8_lcd_frame_buffer[9] &= ~64;
}

void lcd_enable_kmh_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 1;
  else
    ui8_lcd_frame_buffer[13] &= ~1;
}

void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 8;
  else
    ui8_lcd_frame_buffer[13] &= ~8;
}

void lcd_enable_avs_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 16;
  else
    ui8_lcd_frame_buffer[13] &= ~16;
}

void lcd_enable_mxs_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 32;
  else
    ui8_lcd_frame_buffer[13] &= ~32;
}

void lcd_enable_walk_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 64;
  else
    ui8_lcd_frame_buffer[13] &= ~64;
}

void lcd_enable_mph_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[13] |= 128;
  else
    ui8_lcd_frame_buffer[13] &= ~128;
}

void lcd_enable_dst_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[16] |= 8;
  else
    ui8_lcd_frame_buffer[16] &= ~8;
}

void lcd_enable_tm_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[17] |= 16;
  else
    ui8_lcd_frame_buffer[17] &= ~16;
}

void lcd_enable_ttm_symbol (uint8_t ui8_state)
{
  if (ui8_state)
    ui8_lcd_frame_buffer[17] |= 32;
  else
    ui8_lcd_frame_buffer[17] &= ~32;
}

void low_pass_filter_battery_voltage_current_power (void)
{
  // low pass filter battery voltage
  ui32_battery_voltage_accumulated_x10000 -= ui32_battery_voltage_accumulated_x10000 >> BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui32_battery_voltage_accumulated_x10000 += (uint32_t) motor_controller_data.ui16_adc_battery_voltage * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000;
  ui16_battery_voltage_filtered_x10 = ((uint32_t) (ui32_battery_voltage_accumulated_x10000 >> BATTERY_VOLTAGE_FILTER_COEFFICIENT)) / 1000;

  // low pass filter batery current
  ui16_battery_current_accumulated_x5 -= ui16_battery_current_accumulated_x5 >> BATTERY_CURRENT_FILTER_COEFFICIENT;
  ui16_battery_current_accumulated_x5 += (uint16_t) motor_controller_data.ui8_battery_current_x5;
  ui16_battery_current_filtered_x5 = ui16_battery_current_accumulated_x5 >> BATTERY_CURRENT_FILTER_COEFFICIENT;

  // battery power
  ui16_battery_power_filtered_x50 = ui16_battery_current_filtered_x5 * ui16_battery_voltage_filtered_x10;
  ui16_battery_power_filtered = ui16_battery_power_filtered_x50 / 50;

  // loose resolution under 200W
  if (ui16_battery_power_filtered < 200)
  {
    ui16_battery_power_filtered /= 10;
    ui16_battery_power_filtered *= 10;
  }
  // loose resolution under 400W
  else if (ui16_battery_power_filtered < 400)
  {
    ui16_battery_power_filtered /= 20;
    ui16_battery_power_filtered *= 20;
  }
  // loose resolution all other values
  else
  {
    ui16_battery_power_filtered /= 25;
    ui16_battery_power_filtered *= 25;
  }
}

void low_pass_filter_pedal_torque_and_power (void)
{
  // low pass filter
  ui32_pedal_torque_accumulated -= ui32_pedal_torque_accumulated >> PEDAL_TORQUE_FILTER_COEFFICIENT;
  ui32_pedal_torque_accumulated += (uint32_t) motor_controller_data.ui16_pedal_torque_x10 / 10;
  ui16_pedal_torque_filtered = ((uint32_t) (ui32_pedal_torque_accumulated >> PEDAL_TORQUE_FILTER_COEFFICIENT));

  // low pass filter
  ui32_pedal_power_accumulated -= ui32_pedal_power_accumulated >> PEDAL_POWER_FILTER_COEFFICIENT;
  ui32_pedal_power_accumulated += (uint32_t) motor_controller_data.ui16_pedal_power_x10 / 10;
  ui16_pedal_power_filtered = ((uint32_t) (ui32_pedal_power_accumulated >> PEDAL_POWER_FILTER_COEFFICIENT));

  if (ui16_pedal_torque_filtered > 200)
  {
    ui16_pedal_torque_filtered /= 20;
    ui16_pedal_torque_filtered *= 20;
  }
  else if (ui16_pedal_torque_filtered > 100)
  {
    ui16_pedal_torque_filtered /= 10;
    ui16_pedal_torque_filtered *= 10;
  }
  else
  {
    // do nothing to roginal values
  }

  if (ui16_pedal_power_filtered > 500)
  {
    ui16_pedal_power_filtered /= 25;
    ui16_pedal_power_filtered *= 25;
  }
  else if (ui16_pedal_power_filtered > 200)
  {
    ui16_pedal_power_filtered /= 20;
    ui16_pedal_power_filtered *= 20;
  }
  else if (ui16_pedal_power_filtered > 10)
  {
    ui16_pedal_power_filtered /= 10;
    ui16_pedal_power_filtered *= 10;
  }
  else
  {
    ui16_pedal_power_filtered = 0; // no point to show less than 10W
  }
}

static void low_pass_filter_pedal_cadence (void)
{
  // low pass filter
  ui16_pedal_cadence_accumulated -= (ui16_pedal_cadence_accumulated >> PEDAL_CADENCE_FILTER_COEFFICIENT);
  ui16_pedal_cadence_accumulated += (uint16_t) motor_controller_data.ui8_pedal_cadence;

  // consider the filtered value only for medium and high values of the unfiltered value
  if (motor_controller_data.ui8_pedal_cadence > 20)
  {
    ui8_pedal_cadence_filtered = (uint8_t) (ui16_pedal_cadence_accumulated >> PEDAL_CADENCE_FILTER_COEFFICIENT);
  }
  else
  {
    ui8_pedal_cadence_filtered = motor_controller_data.ui8_pedal_cadence;
  }
}

void calc_wh (void)
{
  static uint8_t ui8_1s_timmer_counter;
  uint32_t ui32_temp = 0;

  if (ui16_battery_power_filtered_x50 > 0)
  {
    ui32_wh_sum_x5 += ui16_battery_power_filtered_x50 / 10;
    ui32_wh_sum_counter++;
  }

  // calc at 1s rate
  if (ui8_1s_timmer_counter++ >= 10)
  {
    ui8_1s_timmer_counter = 0;

    // avoid  zero divisison
    if (ui32_wh_sum_counter != 0)
    {
      ui32_temp = ui32_wh_sum_counter / 36;
      ui32_temp = (ui32_temp * (ui32_wh_sum_x5 / ui32_wh_sum_counter)) / 500;
    }

    ui32_wh_x10 = configuration_variables.ui32_wh_x10_offset + ui32_temp;
  }
}

void calc_odometer (void)
{
  uint32_t uint32_temp;
  static uint8_t ui8_1s_timmer_counter;

  // calc at 1s rate
  if (ui8_1s_timmer_counter++ >= 100)
  {
    ui8_1s_timmer_counter = 0;

    uint32_temp = motor_controller_data.ui32_wheel_speed_sensor_tick_counter * ((uint32_t) configuration_variables.ui16_wheel_perimeter);
    // avoid division by 0
    if (uint32_temp > 100000) { uint32_temp /= 100000;}  // milimmeters to 0.1kms
    else { uint32_temp = 0; }

    // now store the value on the global variable
    configuration_variables.ui16_odometer_distance_x10 = (uint16_t) uint32_temp;
  }
}

static void automatic_power_off_management (void)
{
  if (configuration_variables.ui8_lcd_power_off_time_minutes != 0)
  {
    // see if we should reset the automatic power off minutes counter
    if ((motor_controller_data.ui16_wheel_speed_x10 > 0) ||   // wheel speed > 0
        (motor_controller_data.ui8_battery_current_x5 > 0) || // battery current > 0
        (motor_controller_data.ui8_braking) ||                // braking
        button_get_events ())                                 // any button active
    {
      ui16_lcd_power_off_time_counter = 0;
      ui8_lcd_power_off_time_counter_minutes = 0;
    }

    // increment the automatic power off minutes counter
    ui16_lcd_power_off_time_counter++;

    // check if we should power off the LCD
    if (ui16_lcd_power_off_time_counter >= (100 * 60)) // 1 minute passed
    {
      ui16_lcd_power_off_time_counter = 0;

      ui8_lcd_power_off_time_counter_minutes++;
      if (ui8_lcd_power_off_time_counter_minutes >= configuration_variables.ui8_lcd_power_off_time_minutes)
      {
        lcd_power_off (1);
      }
    }
  }
  else
  {
    ui16_lcd_power_off_time_counter = 0;
    ui8_lcd_power_off_time_counter_minutes = 0;
  }
}

struct_configuration_variables* get_configuration_variables (void)
{
  return &configuration_variables;
}

struct_motor_controller_data* lcd_get_motor_controller_data (void)
{
  return &motor_controller_data;
}

void lcd_init (void)
{
  ht1622_init ();
  lcd_set_frame_buffer ();
  lcd_update();

  // init variables with the stored value on EEPROM
  eeprom_init_variables ();
}

void lcd_set_backlight_intensity (uint8_t ui8_intensity)
{
  if (ui8_intensity == 0)
  {
    TIM1_CCxCmd (TIM1_CHANNEL_4, DISABLE);
  }
  else if (ui8_intensity <= 20)
  {
    TIM1_SetCompare4 ((uint16_t) ui8_intensity);
    TIM1_CCxCmd (TIM1_CHANNEL_4, ENABLE);
  }
}

void update_menu_flashing_state (void)
{
  // ***************************************************************************************************
  // For flashing on menus, 0.5 seconds flash
  if (ui8_lcd_menu_flash_counter++ > 50)
  {
    ui8_lcd_menu_flash_counter = 0;

    if (ui8_lcd_menu_flash_state)
      ui8_lcd_menu_flash_state = 0;
    else
      ui8_lcd_menu_flash_state = 1;
  }
  // ***************************************************************************************************

  // ***************************************************************************************************
  ui8_lcd_menu_counter_100ms_state = 0;
  if (ui8_lcd_menu_counter_100ms++ > 10)
  {
    ui8_lcd_menu_counter_100ms = 0;
    ui8_lcd_menu_counter_100ms_state = 1;
  }

  ui8_lcd_menu_counter_500ms_state = 0;
  if (ui8_lcd_menu_counter_500ms++ > 50)
  {
    ui8_lcd_menu_counter_500ms = 0;
    ui8_lcd_menu_counter_500ms_state = 1;
  }
  // ***************************************************************************************************

  // ***************************************************************************************************
  // For flashing the temperature field when the current is being limited due to motor over temperature
  // flash only if current is being limited: ui8_temperature_current_limiting_value != 255
  if (motor_controller_data.ui8_temperature_current_limiting_value != 255)
  {
    if (ui8_lcd_menu_flash_state_temperature == 0) // state 0: disabled
    {
      if (ui16_lcd_menu_flash_counter_temperature > 0)
      {
        ui16_lcd_menu_flash_counter_temperature--;
      }

      if (ui16_lcd_menu_flash_counter_temperature == 0)
      {
        // if motor_controller_data.ui8_temperature_current_limiting_value == 0, flash quicker meaning motor is shutoff
        if (motor_controller_data.ui8_temperature_current_limiting_value > 0)
        {
          ui16_lcd_menu_flash_counter_temperature = 50 + ((uint16_t) motor_controller_data.ui8_temperature_current_limiting_value);
        }
        else
        {
          ui16_lcd_menu_flash_counter_temperature = 25;
        }

        ui8_lcd_menu_flash_state_temperature = 1;
      }
    }

    if (ui8_lcd_menu_flash_state_temperature == 1) // state 1: enabled
    {
      if (ui16_lcd_menu_flash_counter_temperature > 0)
      {
        ui16_lcd_menu_flash_counter_temperature--;
      }

      if (ui16_lcd_menu_flash_counter_temperature == 0)
      {
        ui16_lcd_menu_flash_counter_temperature = 25; // 0.25 second
        ui8_lcd_menu_flash_state_temperature = 0;
      }
    }
  }
  else
  {
    ui8_lcd_menu_flash_state_temperature = 1;
  }
  // ***************************************************************************************************
}

void advance_on_submenu (uint8_t* ui8_p_state, uint8_t ui8_state_max_number)
{
  // advance on submenus on button_onoff_click_event
  if (get_button_onoff_click_event ())
  {
    clear_button_onoff_click_event ();

    *ui8_p_state = (*ui8_p_state + 1) % ui8_state_max_number;
  }
}

void calc_battery_soc_watts_hour (void)
{
  uint32_t ui32_temp;

  ui32_temp = ui32_wh_x10 * 100;
  if (configuration_variables.ui32_wh_x10_100_percent > 0)
  {
    ui32_temp /= configuration_variables.ui32_wh_x10_100_percent;
  }
  else
  {
    ui32_temp = 0;
  }

  // 100% - current SOC or just current SOC
  if (configuration_variables.ui8_show_numeric_battery_soc & 2)
  {
    if (ui32_temp > 100)
      ui32_temp = 100;

    ui16_battery_soc_watts_hour = 100 - ui32_temp;
  }
  else
  {
    ui16_battery_soc_watts_hour = ui32_temp;
  }
}

void lcd_power_off (uint8_t updateDistanceOdo)
{
  if (updateDistanceOdo)
  {
    configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
    configuration_variables.ui32_odometer_x10 += ((uint32_t) configuration_variables.ui16_odometer_distance_x10);
    eeprom_write_variables ();
  } 

  // clear LCD so it is clear to user what is happening
  lcd_clear ();
  lcd_update ();

  // now disable the power to all the system
  GPIO_WriteLow(LCD3_ONOFF_POWER__PORT, LCD3_ONOFF_POWER__PIN);

  // block here
  while (1) ;
}

