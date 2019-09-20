/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho and Leon, 2019.
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
#include "buttons.h"
#include "main.h"
#include "eeprom.h"
#include "pins.h"
#include "uart.h"
#include "common.h"


uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

uint8_t ui8_lcd_field_offset[] = {
    ASSIST_LEVEL_DIGIT_OFFSET,
    ODOMETER_DIGIT_OFFSET,
    TEMPERATURE_DIGIT_OFFSET,
    WHEEL_SPEED_OFFSET,
    BATTERY_POWER_DIGIT_OFFSET,
    SECOND_DIGIT_OFFSET,
    MINUTE_DIGIT_OFFSET,
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

typedef struct _var_number
{
  void *p_var_number;
  uint8_t ui8_size;
  uint8_t ui8_decimal_digit;
  uint32_t ui32_max_value;
  uint32_t ui32_min_value;
  uint32_t ui32_increment_step;
  uint8_t ui8_odometer_field;
} var_number_t;

static struct_motor_controller_data motor_controller_data;
static struct_configuration_variables configuration_variables;


// global system variables
static volatile uint16_t ui16_timer3_counter = 0;
static uint16_t   ui16_battery_voltage_filtered_x1000 = 32000;
static uint16_t   ui16_battery_current_filtered_x10 = 0;
static uint16_t   ui16_battery_power_filtered_x10 = 0;
static uint16_t   ui16_battery_power_step_filtered = 0;
static uint16_t   ui16_pedal_weight_filtered_x100 = 0;
static uint16_t   ui16_pedal_weight_x100 = 0;
static uint16_t   ui16_pedal_power_filtered_x10 = 0;
static uint16_t   ui16_pedal_power_step_filtered = 0;
static uint8_t    ui8_pedal_cadence_RPM_filtered = 0;
static volatile uint32_t ui32_wh_sum_x10 = 0;
static uint32_t   ui32_wh_x10 = 0;
static uint8_t    ui8_config_wh_x10_offset;
static uint16_t   ui16_battery_SOC_percentage;
static uint16_t   ui16_battery_SOC_voltage_x100;


// menu variables
static uint8_t    ui8_lcd_menu = MAIN_MENU;
static uint8_t    ui8_lcd_menu_config_submenu_state = 0;
static uint8_t    ui8_lcd_menu_flash_state;
static uint8_t    ui8_lcd_menu_flash_state_temperature;
static uint8_t    ui8_lcd_menu_config_submenu_number = 0;
static uint8_t    ui8_lcd_menu_config_submenu_active = 0;
static uint8_t    ui8_lcd_menu_config_submenu_change_variable_enabled = 0;
static uint8_t    ui8_odometer_sub_field_state;
uint8_t           ui8_start_odometer_show_field_number = 0;
uint8_t           ui8_odometer_show_field_number_counter = 1;
uint8_t           ui8_odometer_show_field_number = 0;


// time measurement variables
static volatile uint8_t ui8_second = 0;
static uint16_t   ui16_seconds_since_power_on = 0;
static uint8_t    ui8_second_TM = 0;
static uint16_t   ui16_minute_TM = 0;


// energy data variables
static uint16_t   ui16_average_energy_consumption_since_power_on_x10 = 0;
static uint32_t   ui32_wh_since_power_on_x10 = 0;
static uint16_t   ui16_estimated_range_since_power_on_x10 = 0;


// wheel measurement variables
static uint16_t ui16_average_measured_wheel_speed_x10 = 0;
static uint16_t ui16_max_measured_wheel_speed_x10 = 0;


// system functions
void temperature_field(void);
void odometer_field(void);
void wheel_speed_field(void);
void battery_symbol_field(void);
void power_field(void);
void time_measurement_field(void);
void assist_level_field(void);
void brake(void);
void calc_distance(void);
void filter_variables(void);
void lights(void);
void street_mode(void);
void energy(void);
void riding_mode_controller(void);


// menu functions
void lcd_execute_main_screen(void);
void lcd_execute_menu_config(void);
void lcd_execute_menu_config_power(void);
void lcd_execute_menu_config_submenu_basic_setup(void);
void lcd_execute_menu_config_submenu_battery(void);
void lcd_execute_menu_config_submenu_power_assist(void);
void lcd_execute_menu_config_submenu_torque_assist(void);
void lcd_execute_menu_config_submenu_cadence_assist(void);
void lcd_execute_menu_config_submenu_eMTB_assist(void);
void lcd_execute_menu_config_submenu_walk_assist(void);
void lcd_execute_menu_config_submenu_cruise(void);
void lcd_execute_menu_config_main_screen_setup(void);
void lcd_execute_menu_config_submenu_motor_startup_power_boost(void);
void lcd_execute_menu_config_submenu_advanced_setup(void);
void lcd_execute_menu_config_submenu_street_mode(void);
void lcd_execute_menu_config_submenu_technical(void);
void update_menu_flashing_state(void);
void submenu_state_controller(uint8_t ui8_state_max_number);
void advance_on_subfield(uint8_t* ui8_p_state, uint8_t ui8_state_max_number);
void odometer_increase_field_state(void);
uint8_t reset_variable_check(void);
//void change_variable(uint32_t *ui32_variable, uint32_t ui32_min_value, uint32_t ui32_max_value, uint32_t ui32_increment_step);


// LCD functions
void lcd_power_off(void);
void lcd_update(void);
void lcd_clear(void);
void lcd_set_frame_buffer(void);
void lcd_set_backlight_intensity(uint8_t ui8_intensity);
void lcd_print(uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options);
void lcd_configurations_print_number(var_number_t* p_lcd_var_number);
void power_off_timer(void);


// LCD symbol functions
void lcd_enable_w_symbol(uint8_t ui8_state);
void lcd_enable_A_symbol(uint8_t ui8_state);
void lcd_enable_C_symbol(uint8_t ui8_state);
void lcd_enable_E_symbol(uint8_t ui8_state);
void lcd_enable_P_symbol(uint8_t ui8_state);
void lcd_enable_vol_symbol(uint8_t ui8_state);
void lcd_enable_km_symbol(uint8_t ui8_state);
void lcd_enable_mil_symbol(uint8_t ui8_state);
void lcd_enable_kmh_symbol(uint8_t ui8_state);
void lcd_enable_mph_symbol(uint8_t ui8_state);
void lcd_enable_odo_symbol(uint8_t ui8_state);
void lcd_enable_avs_symbol(uint8_t ui8_state);
void lcd_enable_mxs_symbol(uint8_t ui8_state);
void lcd_enable_walk_symbol(uint8_t ui8_state);
void lcd_enable_kmh_symbol(uint8_t ui8_state);
void lcd_enable_dst_symbol(uint8_t ui8_state);
void lcd_enable_tm_symbol(uint8_t ui8_state);
void lcd_enable_ttm_symbol(uint8_t ui8_state);
void lcd_enable_colon_symbol(uint8_t ui8_state);
void lcd_enable_motor_symbol(uint8_t ui8_state);
void lcd_enable_brake_symbol(uint8_t ui8_state);
void lcd_enable_assist_symbol(uint8_t ui8_state);
void lcd_enable_lights_symbol(uint8_t ui8_state);
void lcd_enable_cruise_symbol(uint8_t ui8_state);
void lcd_enable_torque_symbol(uint8_t ui8_state);
void lcd_enable_Model_3_symbol(uint8_t ui8_state);
void lcd_enable_street_mode_symbol(uint8_t ui8_state);
void lcd_enable_temperature_1_symbol(uint8_t ui8_state);
void lcd_enable_odometer_point_symbol(uint8_t ui8_state);
void lcd_enable_battery_power_1_symbol(uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol(uint8_t ui8_state);
void lcd_enable_wheel_speed_point_symbol(uint8_t ui8_state);
void lcd_enable_temperature_degrees_symbol(uint8_t ui8_state);
void lcd_enable_temperature_farneight_symbol(uint8_t ui8_state);



// happens every 1 ms
void TIM3_UPD_OVF_BRK_IRQHandler(void) __interrupt(TIM3_UPD_OVF_BRK_IRQHANDLER)
{
  ui16_timer3_counter++;
  
  static uint8_t ui8_100ms_counter;
  static uint16_t ui16_second_counter;
  
  // calculate watt-hours every 100 ms
  if (++ui8_100ms_counter >= 100)
  {
    // reset counter
    ui8_100ms_counter = 0;
    
    // measure consumed watt-hours
    if (ui16_battery_power_filtered_x10 > 0)
    {
      ui32_wh_sum_x10 += ui16_battery_power_filtered_x10;
    }
  }
  
  // increment second for time measurement 
  if (++ui16_second_counter >= 1000)
  {
    // reset counter
    ui16_second_counter = 0;
    
    // increment second
    ui8_second++;
  }
  
  // clear Interrupt Pending bit
  TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
}



uint16_t get_timer3_counter(void)
{
  return ui16_timer3_counter;
}



void lcd_clock (void)
{
  // clear the screen
  lcd_clear();
  
  // return here until the first communication package is received from the motor controller
  if (!ui8_received_first_package) { return; }

  // LCD menus 
  switch (ui8_lcd_menu)
  {
    case MAIN_MENU: 
      lcd_execute_main_screen();
    break;

    case POWER_MENU:
      lcd_execute_menu_config_power();
    break;
    
    case CONFIGURATION_MENU:
      lcd_execute_menu_config();
    break;
    
    default:
      ui8_lcd_menu = MAIN_MENU;
    break;
  }
  
  filter_variables();
  update_menu_flashing_state();
  calc_distance();
  lcd_update();
  power_off_timer();
}



void lcd_execute_main_screen (void)
{
  riding_mode_controller();
  assist_level_field();
  temperature_field();
  odometer_field();
  wheel_speed_field();
  battery_symbol_field();  
  power_field();  
  time_measurement_field();
  street_mode();
  lights();
  brake();
  energy();
  
  // enter configuration menu if...
  if (UP_DOWN_LONG_CLICK)
  {
    ui8_lcd_menu = CONFIGURATION_MENU;
  }
  
  // enter power menu if...
  if (ONOFF_UP_LONG_CLICK && configuration_variables.ui8_main_screen_power_menu_enabled && !(configuration_variables.ui8_street_mode_enabled))
  {
    ui8_lcd_menu = POWER_MENU;
  }
  
  // power off if...
  if (ONOFF_LONG_CLICK) 
  {
    lcd_power_off();
  }
}



void lcd_execute_menu_config (void)
{
  #define MAX_NUMBER_OF_SUBMENUS    11
  
  if (ui8_lcd_menu_config_submenu_active)
  {
    switch (ui8_lcd_menu_config_submenu_number)
    {
      case 0:
        lcd_execute_menu_config_submenu_basic_setup();
      break;

      case 1:
        lcd_execute_menu_config_submenu_battery();
      break;

      case 2:
        lcd_execute_menu_config_submenu_power_assist();
      break;

      case 3:
        lcd_execute_menu_config_submenu_torque_assist();
      break;
      
      case 4:
        lcd_execute_menu_config_submenu_cadence_assist();
      break;
      
      case 5:
        lcd_execute_menu_config_submenu_eMTB_assist();
      break;
      
      case 6:
        lcd_execute_menu_config_submenu_walk_assist();
      break;
      
      case 7:
        lcd_execute_menu_config_submenu_cruise();
      break;
      
      case 8:
        lcd_execute_menu_config_main_screen_setup();
      break;

      case 9:
        lcd_execute_menu_config_submenu_street_mode();
      break;

      case 10:
        lcd_execute_menu_config_submenu_advanced_setup();
      break;

      case 11:
        lcd_execute_menu_config_submenu_technical();
      break;
      
      default:
        ui8_lcd_menu_config_submenu_active = 0;
      break;
    }
  }
  else
  {
    // advance on submenu if...
    if (UP_CLICK)
    {
      if (ui8_lcd_menu_config_submenu_number < MAX_NUMBER_OF_SUBMENUS) { ++ui8_lcd_menu_config_submenu_number; } 
      else { ui8_lcd_menu_config_submenu_number = 0; }
    }

    // recede on submenu if...
    if (DOWN_CLICK)
    {
      if (ui8_lcd_menu_config_submenu_number > 0) { --ui8_lcd_menu_config_submenu_number; } 
      else { ui8_lcd_menu_config_submenu_number = MAX_NUMBER_OF_SUBMENUS; }
    }
    
    // print submenu number and symbol only half of the time
    if (ui8_lcd_menu_flash_state)
    {
      lcd_print(ui8_lcd_menu_config_submenu_number, WHEEL_SPEED_FIELD, 0);
      
      switch (ui8_lcd_menu_config_submenu_number)
      {
        case 1:
          ui8_lcd_frame_buffer[23] |= 241;
        break;
        
        case 2:
          lcd_enable_P_symbol(1);
          lcd_enable_assist_symbol(1);
        break;
        
        case 3:
          lcd_enable_torque_symbol(1);
          lcd_enable_assist_symbol(1);
        break;
        
        case 4:
          lcd_enable_C_symbol(1);
          lcd_enable_assist_symbol(1);
        break;
        
        case 5:
          lcd_enable_E_symbol(1);
          lcd_enable_assist_symbol(1);
        break;

        case 6:
          lcd_enable_walk_symbol(1);
        break;

        case 7:
          lcd_enable_cruise_symbol(1);
        break;

        case 8:
          lcd_print(88, TIME_SECOND_FIELD, 0);
          lcd_print(888, TIME_MINUTE_FIELD, 0);
          lcd_print(88888, ODOMETER_FIELD, 1);
          lcd_print(888, TEMPERATURE_FIELD, 1);
        break;
        
        case 9:
          lcd_enable_street_mode_symbol(1);
        break;

        case 10:
          lcd_enable_A_symbol(1);
        break;
        
        case 11:
          lcd_enable_Model_3_symbol(1);
        break;
      }
    }
  
    // enter submenu if...
    if (ONOFF_CLICK)
    {
      ui8_lcd_menu_config_submenu_active = 1;
      
      ui8_config_wh_x10_offset = 1;
    }
    
    // leave config menu if...
    if (ONOFF_LONG_CLICK)
    {
      // save the updated variables to EEPROM
      EEPROM_controller(WRITE_TO_MEMORY);
      
      // switch to main menu
      ui8_lcd_menu = MAIN_MENU;
    }
  }
}



void lcd_execute_menu_config_submenu_basic_setup(void)
{
  var_number_t lcd_var_number;
  uint32_t ui32_temp;
  uint16_t ui16_temp;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
    
      // units
      lcd_var_number.p_var_number = &configuration_variables.ui8_units_type;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // clear previous number written on ODOMETER_FIELD
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ODOMETER_FIELD] - 1] &= NUMBERS_MASK;
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        if (configuration_variables.ui8_units_type == 1)
        {
          lcd_enable_mil_symbol(1);
          lcd_enable_mph_symbol(1);
        }
        else
        {
          lcd_enable_km_symbol(1);
          lcd_enable_kmh_symbol(1);
        }
      }

    break;
    
    case 1:
    
      // max wheel speed
      
      // display max wheel speed in either imperial or metric units
      if (configuration_variables.ui8_units_type)
      {
        // imperial
        lcd_var_number.p_var_number = &configuration_variables.ui8_wheel_max_speed_imperial;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 62; // needs to be 1.6 times smaller than metric max value
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_mph_symbol(1);
        
        // convert max wheel speed in imperial units to metric units and save to ui8_wheel_max_speed
        configuration_variables.ui8_wheel_max_speed = configuration_variables.ui8_wheel_max_speed_imperial * 1.6;
      }
      else
      {
        // metric  
        lcd_var_number.p_var_number = &configuration_variables.ui8_wheel_max_speed;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 99; // needs to be smaller than 100 or else value > digits on display
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;        
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_kmh_symbol(1);
      }
      
    break;
    
    case 2:
    
      // wheel perimeter in millimeters
      lcd_var_number.p_var_number = &configuration_variables.ui16_wheel_perimeter;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 3000;
      lcd_var_number.ui32_min_value = 750;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
/*       if (ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        change_variable(&configuration_variables.ui16_wheel_perimeter, 750, 3000, 1);
      }
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(configuration_variables.ui16_wheel_perimeter, ODOMETER_FIELD, 0);
      }
       */
    break;
    
    case 3:
    
      // motor voltage type
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_type;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 3;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
/*       if (ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        change_variable(&configuration_variables.ui8_motor_type, 0, 3, 1);
      }
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(configuration_variables.ui8_motor_type, ODOMETER_FIELD, 0);
      } */
      
    break;
    
    case 4:
    
      // motor power limit
      ui16_temp = configuration_variables.ui8_target_max_battery_power_div25 * 25;
      lcd_var_number.p_var_number = &ui16_temp;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 1; // needs to be for BATTERY_POWER_FIELD
      lcd_var_number.ui32_max_value = 1900;
      lcd_var_number.ui32_min_value = 0;

      if (configuration_variables.ui8_target_max_battery_power_div25 < 10)
      {
        lcd_var_number.ui32_increment_step = 25;
      }
      else
      {
        lcd_var_number.ui32_increment_step = 50;
      }

      lcd_var_number.ui8_odometer_field = BATTERY_POWER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      configuration_variables.ui8_target_max_battery_power_div25 = ui16_temp / 25;
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_enable_w_symbol (1);
        lcd_enable_motor_symbol (1);
      }

    break;
    
    case 5:
      
      // set odometer
      if (configuration_variables.ui8_units_type)
      {
        // imperial
        ui32_temp = configuration_variables.ui32_odometer_x10 / 1.6;

        lcd_var_number.p_var_number = &ui32_temp;
        lcd_var_number.ui8_size = 32;
        lcd_var_number.ui8_decimal_digit = 1;
        lcd_var_number.ui32_max_value = 4294967295; // needs to be 1.6 times smaller than metric max value
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 35;
        lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        // convert imperial distance back to metric and save
        configuration_variables.ui32_odometer_x10 = ui32_temp * 1.6;
        
        lcd_enable_odo_symbol(1);
        lcd_enable_mil_symbol(1);
      }
      else
      {
        // metric
        lcd_var_number.p_var_number = &configuration_variables.ui32_odometer_x10;
        lcd_var_number.ui8_size = 32;
        lcd_var_number.ui8_decimal_digit = 1;
        lcd_var_number.ui32_max_value = 4294967295;
        lcd_var_number.ui32_min_value = 0;
        lcd_var_number.ui32_increment_step = 35;
        lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
        lcd_configurations_print_number(&lcd_var_number);

        lcd_enable_odo_symbol(1);
        lcd_enable_km_symbol(1);
      }
      
    break;

    case 6:
    
      // light mode
      lcd_var_number.p_var_number = &configuration_variables.ui8_light_mode;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 2;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
/*       if (ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        change_variable(&configuration_variables.ui8_light_mode, 0, 2, 1);
      }
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(configuration_variables.ui8_light_mode, ODOMETER_FIELD, 0);
      } */
      
      lcd_enable_lights_symbol(1);
      
      // set backlight brightness after user has configured settings, looks nicer this way
      if (configuration_variables.ui8_lights_state) { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness); }
      else { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness); }
      
    break;
    
    case 7:
    
      // backlight day time brightness
      ui32_temp = configuration_variables.ui8_lcd_backlight_off_brightness * 5;
      
/*       if (ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        change_variable(&ui32_temp, 0, 100, 5);
      }
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(ui32_temp, ODOMETER_FIELD, 0);
      } */
      
      lcd_var_number.p_var_number = &ui32_temp;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 5;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // convert percentage value
      configuration_variables.ui8_lcd_backlight_off_brightness = ui32_temp / 5;
      
      // show user the chosen backlight brightness, looks nicer this way
      lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness);
      
    break;
    
    
    case 8:
    
      // backlight night time brightness
      ui32_temp = configuration_variables.ui8_lcd_backlight_on_brightness * 5;
      
      lcd_var_number.p_var_number = &ui32_temp;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 5;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // convert percentage value
      configuration_variables.ui8_lcd_backlight_on_brightness = ui32_temp / 5;
      
      // show user the chosen backlight brightness, looks nicer this way
      lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness);
      
    break;

    case 9:
    
      // auto power off
      lcd_var_number.p_var_number = &configuration_variables.ui8_lcd_power_off_time_minutes;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 255;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      // set backlight brightness after user has configured settings, looks nicer this way
      if (configuration_variables.ui8_lights_state) { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness); }
      else { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness); }
      
    break;

    case 10:
    
      if (ONOFF_LONG_CLICK && ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        EEPROM_controller(SET_TO_DEFAULT);
        EEPROM_controller(READ_FROM_MEMORY);
        
        // set backlight brightness after reset, looks nicer this way
        if (configuration_variables.ui8_lights_state) { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness); }
        else { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness); }
      }
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(42, ODOMETER_FIELD, 0); // just for show
      }
      
    break;
  }
  
  if ((ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled) && (ui8_lcd_menu_config_submenu_state > 1))
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(10);
}



void lcd_execute_menu_config_submenu_battery(void)
{
  var_number_t lcd_var_number;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    // max battery current
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_battery_max_current;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery low voltage cut-off
    case 1:
      lcd_var_number.p_var_number = &configuration_variables.ui16_battery_low_voltage_cut_off_x10;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 630;
      lcd_var_number.ui32_min_value = 160;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery number of cells in series
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_battery_cells_number;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 15;
      lcd_var_number.ui32_min_value = 7;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery internal resistance
    case 3:
      lcd_var_number.p_var_number = &configuration_variables.ui16_battery_pack_resistance_x1000;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1000;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // battery voltage SOC adjusted with internal resistance
    case 4:
      lcd_print(ui16_battery_SOC_voltage_x100 / 10, ODOMETER_FIELD, 1);
      lcd_enable_vol_symbol(1);
    break;
    
    // menu to enable/disable show of numeric watt-hour value and type of representation
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_battery_SOC_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 2;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // menu to set battery_voltage_reset_wh_counter
    case 6:
      lcd_var_number.p_var_number = &configuration_variables.ui16_battery_voltage_reset_wh_counter_x10;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 630;
      lcd_var_number.ui32_min_value = 160;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // menu to set battery capacity in watt-hours
    case 7:
      lcd_var_number.p_var_number = &configuration_variables.ui32_wh_x10_100_percent;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 100000;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 100;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // menu to set current watt hour value
    case 8:
      // on the very first time, use current value of ui32_wh_x10
      if (ui8_config_wh_x10_offset)
      {
        ui8_config_wh_x10_offset = 0;
        configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
      }
      
      // keep reseting these values
      ui32_wh_sum_x10 = 0;
      ui32_wh_x10 = 0;

      lcd_var_number.p_var_number = &configuration_variables.ui32_wh_x10_offset;
      lcd_var_number.ui8_size = 32;
      lcd_var_number.ui8_decimal_digit = 1;
      lcd_var_number.ui32_max_value = 100000;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 100;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
  }
  
  // battery symbol
  ui8_lcd_frame_buffer[23] |= 241;
  
  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(8); // 8 sub menus
}



void lcd_execute_menu_config_submenu_power_assist(void)
{
  var_number_t lcd_var_number;

  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    // enable/disable power assist function
    lcd_var_number.p_var_number = &configuration_variables.ui8_power_assist_function_enabled;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_P_symbol(1);
    lcd_enable_assist_symbol(1);

    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
    
    // if power assist is enabled...
    if (configuration_variables.ui8_power_assist_function_enabled)
    {
      // disable all other assist modes
      configuration_variables.ui8_torque_assist_function_enabled = 0;
      configuration_variables.ui8_cadence_assist_function_enabled = 0;
    }
  }
  else if (ui8_lcd_menu_config_submenu_state == 1)
  {
    // number of assist levels
    lcd_var_number.p_var_number = &configuration_variables.ui8_number_of_assist_levels;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 9;
    lcd_var_number.ui32_min_value = 1;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_P_symbol(1);
    lcd_enable_assist_symbol(1);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else
  {
    // value of each power assist level factor
    lcd_var_number.p_var_number = &configuration_variables.ui8_power_assist_level[ui8_lcd_menu_config_submenu_state - 2];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 255;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol(1);
      lcd_print(ui8_lcd_menu_config_submenu_state - 1, ASSIST_LEVEL_FIELD, 1);
    }
  }
  
  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels + 1);
}



void lcd_execute_menu_config_submenu_torque_assist(void)
{
  var_number_t lcd_var_number;

  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    // enable/disable torque assist function
    lcd_var_number.p_var_number = &configuration_variables.ui8_torque_assist_function_enabled;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_torque_symbol(1);
    lcd_enable_assist_symbol(1);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
    
    // if torque assist is enabled...
    if (configuration_variables.ui8_torque_assist_function_enabled)
    {
      // disable all other assist modes
      configuration_variables.ui8_power_assist_function_enabled = 0;
      configuration_variables.ui8_cadence_assist_function_enabled = 0;
    }
  }
  else if (ui8_lcd_menu_config_submenu_state == 1)
  {
    // number of assist levels
    lcd_var_number.p_var_number = &configuration_variables.ui8_number_of_assist_levels;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 9;
    lcd_var_number.ui32_min_value = 1;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_torque_symbol(1);
    lcd_enable_assist_symbol(1);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else
  {
    // value of each torque assist level factor
    lcd_var_number.p_var_number = &configuration_variables.ui8_torque_assist_level[ui8_lcd_menu_config_submenu_state - 2];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 1;
    lcd_var_number.ui32_max_value = 255;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol(1);
      lcd_print(ui8_lcd_menu_config_submenu_state - 1, ASSIST_LEVEL_FIELD, 1);
    }
  }
  
  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels + 1);
}



void lcd_execute_menu_config_submenu_cadence_assist(void)
{
  var_number_t lcd_var_number;

  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    // enable/disable cadence assist function
    lcd_var_number.p_var_number = &configuration_variables.ui8_cadence_assist_function_enabled;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_C_symbol(1);
    lcd_enable_assist_symbol(1);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
    
    // if cadence assist is enabled...
    if (configuration_variables.ui8_cadence_assist_function_enabled)
    {
      // disable all other assist modes
      configuration_variables.ui8_power_assist_function_enabled = 0;
      configuration_variables.ui8_torque_assist_function_enabled = 0;
    }
  }
  else if (ui8_lcd_menu_config_submenu_state == 1)
  {
    // number of assist levels
    lcd_var_number.p_var_number = &configuration_variables.ui8_number_of_assist_levels;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 9;
    lcd_var_number.ui32_min_value = 1;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    lcd_enable_C_symbol(1);
    lcd_enable_assist_symbol(1);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else
  {
    // value of each cadence assist level factor
    lcd_var_number.p_var_number = &configuration_variables.ui8_cadence_assist_level[ui8_lcd_menu_config_submenu_state - 2];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 254;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol(1);
      lcd_print(ui8_lcd_menu_config_submenu_state - 1, ASSIST_LEVEL_FIELD, 1);
    }
  }
  
  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels + 1);
}



void lcd_execute_menu_config_submenu_eMTB_assist(void)
{
  var_number_t lcd_var_number;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
    
      // enable eMTB mode
      lcd_var_number.p_var_number = &configuration_variables.ui8_eMTB_assist_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
    
    case 1:
      
      // set eMTB assist level
      lcd_var_number.p_var_number = &configuration_variables.ui8_eMTB_assist_sensitivity;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 20;
      lcd_var_number.ui32_min_value = 1;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
  }
  
  lcd_enable_E_symbol(1);
  lcd_enable_assist_symbol(1);
  
  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(1);
}



void lcd_execute_menu_config_submenu_walk_assist(void)
{
  var_number_t lcd_var_number;
  
  uint16_t ui16_temp;
  
  if (ui8_lcd_menu_config_submenu_state == 0)
  {
    // enable/disable walk assist function
    lcd_var_number.p_var_number = &configuration_variables.ui8_walk_assist_function_enabled;
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else if (ui8_lcd_menu_config_submenu_state == 1)
  {
    // set walk assist button bounce time
    ui16_temp = configuration_variables.ui8_walk_assist_button_bounce_time * 10;
    
    lcd_var_number.p_var_number = &ui16_temp;
    lcd_var_number.ui8_size = 16;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 1000;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 10;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);
    
    configuration_variables.ui8_walk_assist_button_bounce_time = ui16_temp / 10;
    
    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
    }
  }
  else
  {
    // value of each walk assist power value
    lcd_var_number.p_var_number = &configuration_variables.ui8_walk_assist_level[(ui8_lcd_menu_config_submenu_state - 2)];
    lcd_var_number.ui8_size = 8;
    lcd_var_number.ui8_decimal_digit = 0;
    lcd_var_number.ui32_max_value = 100;
    lcd_var_number.ui32_min_value = 0;
    lcd_var_number.ui32_increment_step = 1;
    lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
    lcd_configurations_print_number(&lcd_var_number);

    if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
    {
      lcd_enable_assist_symbol(1);
      lcd_print(ui8_lcd_menu_config_submenu_state - 1, ASSIST_LEVEL_FIELD, 1);
    }
  }
  
  lcd_enable_walk_symbol(1);
  
  submenu_state_controller(configuration_variables.ui8_number_of_assist_levels + 1);
}



void lcd_execute_menu_config_submenu_cruise(void)
{
  var_number_t lcd_var_number;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
      
      // cruise function enable/disable
      lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
    
    case 1:
      
      // enable/disable target speed for cruise 
      lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_set_target_speed_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
    
    case 2:
      
      // set target cruise speed in imperial or metric units
      if (configuration_variables.ui8_units_type)
      {
        // imperial
        lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_target_speed_mph;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 62; // needs to be 1.6 times smaller than metric max value
        lcd_var_number.ui32_min_value = CRUISE_THRESHOLD_SPEED_X10 / 16;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_mph_symbol(1);
        
        // convert imperial to metric and save 
        configuration_variables.ui8_cruise_function_target_speed_kph = configuration_variables.ui8_cruise_function_target_speed_mph * 1.6;
      }
      else
      {
        // metric
        lcd_var_number.p_var_number = &configuration_variables.ui8_cruise_function_target_speed_kph;
        lcd_var_number.ui8_size = 8;
        lcd_var_number.ui8_decimal_digit = 0;
        lcd_var_number.ui32_max_value = 99; // needs to be smaller than 100 or else value > digits on display
        lcd_var_number.ui32_min_value = CRUISE_THRESHOLD_SPEED_X10 / 10;
        lcd_var_number.ui32_increment_step = 1;
        lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
        lcd_configurations_print_number(&lcd_var_number);
        
        lcd_enable_kmh_symbol(1);
      }
      
    break;
  
    // show cruise function target speed enable/disable
    case 3:
    
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_cruise_function_set_target_speed;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
  }
  
  lcd_enable_cruise_symbol(1);
  
  if ((ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled) && (ui8_lcd_menu_config_submenu_state != 2))
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(3);
}



void lcd_execute_menu_config_main_screen_setup(void)
{
  var_number_t lcd_var_number;

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // enable/disable show of distance data in odometer field
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_distance_data_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break; 
    
    // enable/disable show of battery SOC data in odometer field
    case 1:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_battery_SOC_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of battery voltage or current in odometer field
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_battery_state_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of pedal data in odometer field
    case 3:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_pedal_data_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of energy data in odometer field
    case 4:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_energy_data_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of time measurement in odometer field
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_time_measurement_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable show of wheel speed in odometer field
    case 6:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_wheel_speed_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable/disable show of motor temperature in odometer field
    case 7:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_motor_temperature_odometer_field;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable/disable show of cruise function set target speed
    case 8:
      lcd_var_number.p_var_number = &configuration_variables.ui8_show_cruise_function_set_target_speed;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // enable/disable quick set power menu
    case 9:
      lcd_var_number.p_var_number = &configuration_variables.ui8_main_screen_power_menu_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // temperature field setup
    case 10:
      lcd_var_number.p_var_number = &configuration_variables.ui8_temperature_field_state;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 6;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
  }

  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(10);
}



void lcd_execute_menu_config_submenu_street_mode (void)
{
  var_number_t lcd_var_number;
  uint16_t ui16_temp;

  switch (ui8_lcd_menu_config_submenu_state)
  {
    // enable/disable and set street mode
    case 0:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_function_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 2;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // street mode speed limit
    case 1:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_speed_limit;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 99;
      lcd_var_number.ui32_min_value = 1;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = WHEEL_SPEED_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_kmh_symbol (1);
    break;

    // enable/disable street mode power limit
    case 2:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_power_limit_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;

    // street mode power limit
    case 3:
      ui16_temp = configuration_variables.ui8_street_mode_power_limit_div25 * 25;
      
      lcd_var_number.p_var_number = &ui16_temp;
      lcd_var_number.ui8_size = 16;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1900;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 25;
      lcd_var_number.ui8_odometer_field = BATTERY_POWER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      configuration_variables.ui8_street_mode_power_limit_div25 = ui16_temp / 25;
      
      
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_enable_w_symbol (1);
        lcd_enable_motor_symbol (1);
      }
      
    break;
    
    // enable/disable throttle in street mode
    case 4:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_throttle_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
    
    // enable/disable Cruise in street mode
    case 5:
      lcd_var_number.p_var_number = &configuration_variables.ui8_street_mode_cruise_enabled;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 1;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
    break;
  }
  
  if ((ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled) && (ui8_lcd_menu_config_submenu_state != 1))
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(5);
}



void lcd_execute_menu_config_submenu_advanced_setup(void)
{
  var_number_t lcd_var_number;
  
  static uint8_t ui8_hold_down_enabled;
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
    
      // motor acceleration adjustment    
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_acceleration;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
    
    case 1:
    
      // assist without pedal rotation threshold  
      lcd_var_number.p_var_number = &configuration_variables.ui8_assist_without_pedal_rotation_threshold;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 100;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
    
    case 2:
    
      // pedal torque conversion
      lcd_var_number.p_var_number = &configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 255;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;
    
    case 3:
      
      // weight on pedal from pedal torque conversion
      if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(ui16_pedal_weight_x100 / 10, ODOMETER_FIELD, 1);
      }
    
    break;
    
    case 4:
    
      // cadence sensor mode and automatic calibration
      if (ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        if (configuration_variables.ui8_cadence_sensor_mode > 2)
        {
          configuration_variables.ui8_cadence_sensor_mode = STANDARD_MODE;
        }
        
        if (UP_CLICK && configuration_variables.ui8_cadence_sensor_mode < 1)
        {
          ++configuration_variables.ui8_cadence_sensor_mode;
        }
        
        if (DOWN_CLICK && configuration_variables.ui8_cadence_sensor_mode > 0)
        {
          --configuration_variables.ui8_cadence_sensor_mode;
        }
        
        if (DOWN_LONG_CLICK && (configuration_variables.ui8_cadence_sensor_mode == ADVANCED_MODE))
        {
          ui8_hold_down_enabled = 1;
        }
        
        if (ui8_hold_down_enabled && buttons_get_down_state())
        {
          motor_controller_data.ui8_riding_mode = CADENCE_SENSOR_CALIBRATION_MODE;
          configuration_variables.ui8_cadence_sensor_mode = CALIBRATION_MODE;
          
          lcd_print(configuration_variables.ui16_cadence_sensor_pulse_high_percentage_x10, ODOMETER_FIELD, 1);
        }
        else
        {
          ui8_hold_down_enabled = 0;
          
          motor_controller_data.ui8_riding_mode = OFF_MODE;
          
          if (configuration_variables.ui8_cadence_sensor_mode == CALIBRATION_MODE)
          {
            configuration_variables.ui8_cadence_sensor_mode = ADVANCED_MODE;
          }
          
          if (ui8_lcd_menu_flash_state)
          {
            lcd_print(configuration_variables.ui8_cadence_sensor_mode, ODOMETER_FIELD, 0);
          }
        }
      }
      else if (ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
      {
        lcd_print(configuration_variables.ui8_cadence_sensor_mode, ODOMETER_FIELD, 0);
      }
      
    break;
    
    case 5:
      
      // lights configuration
      lcd_var_number.p_var_number = &configuration_variables.ui8_lights_configuration;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 8;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_lights_symbol(1);
      
    break;
    
    case 6:
      
      // motor temperature control or throttle control
      lcd_var_number.p_var_number = &configuration_variables.ui8_optional_ADC_function;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 2;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
    break;

    case 7:
    
      // motor temperature limit min
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_temperature_min_value_to_limit;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 110;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_temperature_degrees_symbol (1);

    break;

    case 8:
      
      // motor temperature limit max
      lcd_var_number.p_var_number = &configuration_variables.ui8_motor_temperature_max_value_to_limit;
      lcd_var_number.ui8_size = 8;
      lcd_var_number.ui8_decimal_digit = 0;
      lcd_var_number.ui32_max_value = 110;
      lcd_var_number.ui32_min_value = 0;
      lcd_var_number.ui32_increment_step = 1;
      lcd_var_number.ui8_odometer_field = ODOMETER_FIELD;
      lcd_configurations_print_number(&lcd_var_number);
      
      lcd_enable_temperature_degrees_symbol (1);
    
    break;
  }

  if (ui8_lcd_menu_flash_state || ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  }
  
  submenu_state_controller(8);
}



void lcd_execute_menu_config_submenu_technical (void)
{
  #define MAX_NUMBER_OF_SUBMENUS_TECHNICAL_DATA    7
  
  switch (ui8_lcd_menu_config_submenu_state)
  {
    case 0:
      lcd_print(motor_controller_data.ui8_adc_throttle, ODOMETER_FIELD, 0);
    break;

    case 1:
      lcd_print(motor_controller_data.ui8_throttle, ODOMETER_FIELD, 0);
    break;

    case 2:
      lcd_print(motor_controller_data.ui16_adc_pedal_torque_sensor, ODOMETER_FIELD, 0);
    break;

    case 3:
      lcd_print(motor_controller_data.ui8_pedal_cadence_RPM, ODOMETER_FIELD, 0);
    break;

    case 4:
      lcd_print(motor_controller_data.ui8_duty_cycle, ODOMETER_FIELD, 0);
    break;

    case 5:
      lcd_print(motor_controller_data.ui16_motor_speed_erps, ODOMETER_FIELD, 0);
    break;

    case 6:
      lcd_print(motor_controller_data.ui8_foc_angle, ODOMETER_FIELD, 0);
    break;
    
    case 7:
      lcd_print(configuration_variables.ui16_cadence_sensor_pulse_high_percentage_x10, ODOMETER_FIELD, 1);
    break;
  }
  
  lcd_print(ui8_lcd_menu_config_submenu_state, WHEEL_SPEED_FIELD, 0);
  
  // advance on sub menu
  if (UP_CLICK)
  {
    if (ui8_lcd_menu_config_submenu_state < MAX_NUMBER_OF_SUBMENUS_TECHNICAL_DATA) { ++ui8_lcd_menu_config_submenu_state; } 
    else { ui8_lcd_menu_config_submenu_state = 0; }
  }

  // recede on sub menu
  if (DOWN_CLICK)
  {
    if (ui8_lcd_menu_config_submenu_state > 0) { --ui8_lcd_menu_config_submenu_state; } 
    else { ui8_lcd_menu_config_submenu_state = MAX_NUMBER_OF_SUBMENUS_TECHNICAL_DATA; }
  }
  
  // leave sub menu 
  if (ONOFF_LONG_CLICK)
  {
    ui8_lcd_menu_config_submenu_active = 0;
    ui8_lcd_menu_config_submenu_state = 0;
  }
}



void lcd_execute_menu_config_power (void)
{
  var_number_t lcd_var_number;
  uint16_t ui16_temp;
  
  // enable change of variables
  ui8_lcd_menu_config_submenu_change_variable_enabled = 1;

  ui16_temp = configuration_variables.ui8_target_max_battery_power_div25 * 25;
  lcd_var_number.p_var_number = &ui16_temp;
  lcd_var_number.ui8_size = 16;
  lcd_var_number.ui8_decimal_digit = 1; // needs to be for BATTERY_POWER_FIELD
  lcd_var_number.ui32_max_value = 1900;
  lcd_var_number.ui32_min_value = 0;

  if (configuration_variables.ui8_target_max_battery_power_div25 < 10)
  {
    lcd_var_number.ui32_increment_step = 25;
  }
  else
  {
    lcd_var_number.ui32_increment_step = 50;
  }

  lcd_var_number.ui8_odometer_field = BATTERY_POWER_FIELD;
  lcd_configurations_print_number(&lcd_var_number);
  configuration_variables.ui8_target_max_battery_power_div25 = ui16_temp / 25;
  
  lcd_enable_w_symbol (1);
  lcd_enable_motor_symbol (1);
  
  // leave config power menu with a long onoff click
  if (ONOFF_LONG_CLICK)
  {
    // disable change of variables
    ui8_lcd_menu_config_submenu_change_variable_enabled = 0;

    // save the updated variables to EEPROM
    EEPROM_controller(WRITE_TO_MEMORY);
    
    // change to main menu
    ui8_lcd_menu = MAIN_MENU;
  }
}



void power_off_timer (void)
{
  static uint16_t ui16_seconds_since_power_on_offset;
  
  // check if automatic power off management is configured to any value
  if (configuration_variables.ui8_lcd_power_off_time_minutes != 0)
  {
    // check if there is system activity
    if ((motor_controller_data.ui16_wheel_speed_x10 > 0) ||       // wheel speed > 0
        (motor_controller_data.ui8_battery_current_x10 > 4) ||    // battery current > 0.4 A
        (motor_controller_data.ui8_braking) ||                    // braking
        (buttons_get_events()))                                   // any button active
    {
      // reset offset
      ui16_seconds_since_power_on_offset = ui16_seconds_since_power_on;
    }
    else 
    {
      // check if system has been inactive over or equal to the configured threshold time
      if (ui16_seconds_since_power_on - ui16_seconds_since_power_on_offset >= (configuration_variables.ui8_lcd_power_off_time_minutes * 60))
      {
        // power off system and save variables to EEPROM
        lcd_power_off();
      }
    }
  }
}



void temperature_field(void)
{
  // if motor current is being limited due to temperature, i.e. below 255, force showing motor temperature
  if ((configuration_variables.ui8_optional_ADC_function == TEMPERATURE_CONTROL) && (motor_controller_data.ui8_temperature_current_limiting_value < 255))
  {
    if (ui8_lcd_menu_flash_state_temperature)
    {
      lcd_print(motor_controller_data.ui8_motor_temperature, TEMPERATURE_FIELD, 1);
      lcd_enable_temperature_degrees_symbol (1);
    }
  }
  else
  {
    switch (configuration_variables.ui8_temperature_field_state)
    {
      // show motor temperature
      case 1:
        // if function is enabled -> display motor temperature
        if (configuration_variables.ui8_optional_ADC_function == TEMPERATURE_CONTROL)
        {
          lcd_print(motor_controller_data.ui8_motor_temperature, TEMPERATURE_FIELD, 1);
          lcd_enable_temperature_degrees_symbol(1);
        }
      break;
      
      // show battery state of charge watt-hours
      case 2:
        lcd_print(ui16_battery_SOC_percentage, TEMPERATURE_FIELD, 1);
      break;
      
      // battery voltage
      case 3:
        lcd_print(ui16_battery_voltage_filtered_x1000 / 1000, TEMPERATURE_FIELD, 1);
      break;
      
      // battery current
      case 4:
        lcd_print(ui16_battery_current_filtered_x10 / 10, TEMPERATURE_FIELD, 1);
      break;
      
      // pedal cadence
      case 5:
        lcd_print(ui8_pedal_cadence_RPM_filtered, TEMPERATURE_FIELD, 1);
      break;
      
      // average wheel speed since power on
      case 6:
        // check in what unit of measurement to display average wheel speed
        if (configuration_variables.ui8_units_type)
        {
          // imperial
          lcd_print (((float) ui16_average_measured_wheel_speed_x10/16), TEMPERATURE_FIELD, 1);
        }
        else
        {
          // metric
          lcd_print (ui16_average_measured_wheel_speed_x10/10, TEMPERATURE_FIELD, 1);
        }
      break;
      
      // show nothing
      default:
      break;
    }
  }
}



void time_measurement_field(void)
{
  // update time
  ui16_seconds_since_power_on += ui8_second;
  ui8_second_TM += ui8_second;
  configuration_variables.ui8_total_second_TTM += ui8_second;
  
  // reset elapsed seconds
  ui8_second = 0;
  
  // time measurement since power on
  
    // check if overflow
    if (ui8_second_TM >= 60)
    {
      // reset second
      ui8_second_TM = 0;
      
      // increment minute
      ui16_minute_TM++;
    }
  
  
  // total time measurement since last reset (TTM)
    
    // check if overflow
    if (configuration_variables.ui8_total_second_TTM >= 60)
    {
      // reset second
      configuration_variables.ui8_total_second_TTM = 0;
      
      // increment minute
      configuration_variables.ui8_total_minute_TTM++;
      
      // check if overflow
      if (configuration_variables.ui8_total_minute_TTM >= 60)
      {
        // reset minute
        configuration_variables.ui8_total_minute_TTM = 0;
        
        // increment hour
        configuration_variables.ui16_total_hour_TTM++;
      }
    }
    
  // display either TM or TTM
  if (configuration_variables.ui8_time_measurement_field_state)
  {
    lcd_enable_colon_symbol(1);
    lcd_enable_tm_symbol(1);
    lcd_print(ui8_second_TM, TIME_SECOND_FIELD, 0);
    lcd_print(ui16_minute_TM, TIME_MINUTE_FIELD, 0);
  }
  else
  {
    lcd_enable_colon_symbol(1);
    lcd_enable_ttm_symbol(1);
    lcd_print(configuration_variables.ui8_total_minute_TTM, TIME_SECOND_FIELD, 0);
    lcd_print(configuration_variables.ui16_total_hour_TTM, TIME_MINUTE_FIELD, 0);
  }
}



void energy(void)
{
  // reset watt-hour value if battery voltage is over threshold set from user, but only do this once for every power on
  
  static uint8_t ui8_wh_reset;
  
  if (((motor_controller_data.ui16_battery_voltage_x1000) > (configuration_variables.ui16_battery_voltage_reset_wh_counter_x10 * 100)) && (!ui8_wh_reset))
  {
    ui8_wh_reset = 1;
    configuration_variables.ui32_wh_x10_offset = 0;
  }
  

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // calculate watt-hours since power on
  ui32_wh_since_power_on_x10 = ui32_wh_sum_x10 / 36000;
  
  // calculate watt-hours since last full charge
  ui32_wh_x10 = configuration_variables.ui32_wh_x10_offset + ui32_wh_since_power_on_x10;
  
  // calculate average watt-hour consumption per distance traveled, since power on. Check to avoid zero division
  if (configuration_variables.ui16_distance_since_power_on_x10 == 0)
  {
    ui16_average_energy_consumption_since_power_on_x10 = 0;
  }
  else
  {
    // divide watt-hours with distance since power on and save in variable
    ui16_average_energy_consumption_since_power_on_x10 = (ui32_wh_since_power_on_x10 * 10) / configuration_variables.ui16_distance_since_power_on_x10; // multiply numerator with 10 to retain decimal 
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // calculate estimated range since power on by dividing watt-hours remaining with average consumption. Check to avoid zero division
  if (ui16_average_energy_consumption_since_power_on_x10 == 0)
  {
    ui16_estimated_range_since_power_on_x10 = 10000;
  }
  else if (ui32_wh_x10 > configuration_variables.ui32_wh_x10_100_percent)
  {
    ui16_estimated_range_since_power_on_x10 = 0;
  }
  else
  {
    ui16_estimated_range_since_power_on_x10 = ((configuration_variables.ui32_wh_x10_100_percent - ui32_wh_x10) * 10) / ui16_average_energy_consumption_since_power_on_x10; // multiply numerator with 10 to retain decimal 
  }
  
  // limit estimated range depending on unit of measurement, looks nicer this way
  if (configuration_variables.ui8_units_type)
  {
    // imperial units
    if (ui16_estimated_range_since_power_on_x10 > 9599) { ui16_estimated_range_since_power_on_x10 = 9599; }
  }
  else
  {
    // metric units
    if (ui16_estimated_range_since_power_on_x10 > 9999) { ui16_estimated_range_since_power_on_x10 = 9999; }
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // calculate battery SOC percentage
 
  uint32_t ui32_temp = ui32_wh_x10 * 100;
  
  if (configuration_variables.ui32_wh_x10_100_percent > 0)
  {
    ui32_temp /= configuration_variables.ui32_wh_x10_100_percent;
  }
  else
  {
    ui32_temp = 0;
  }

  if (configuration_variables.ui8_battery_SOC_function_enabled == 1) // SOC from 100 to 0 percent (remaining capacity in percent)
  {
    // limit percentage to 100
    if (ui32_temp > 100)
    {
      ui32_temp = 100;
    }
    
    // calculate and set remaining percentage
    ui16_battery_SOC_percentage = 100 - ui32_temp;
  }
  else if (configuration_variables.ui8_battery_SOC_function_enabled == 2) // SOC from 0 to 100 percent (consumed capacity in percent)
  {
    // set consumed percentage
    ui16_battery_SOC_percentage = ui32_temp;
  }
}



void battery_symbol_field(void)
{
  // This values were taken from a discharge graph of Samsung INR18650-25R cells, at almost no current discharge
  // This graph: https://endless-sphere.com/forums/download/file.php?id=183920&sid=b7fd7180ef87351cabe74a22f1d162d7
  
  #define LI_ION_CELL_VOLTS_83_x100    396
  #define LI_ION_CELL_VOLTS_50_x100    370
  #define LI_ION_CELL_VOLTS_17_x100    344
  #define LI_ION_CELL_VOLTS_0_x100     330
  
  // calculate battery voltage that takes internal battery pack resistance into consideration
  uint16_t ui16_battery_voltage_internal_resistance_adjusted_x100 = ((uint32_t) configuration_variables.ui16_battery_pack_resistance_x1000 * ui16_battery_current_filtered_x10) / 100;
  
  // add voltage value
  ui16_battery_SOC_voltage_x100 = (ui16_battery_voltage_filtered_x1000 / 10) + ui16_battery_voltage_internal_resistance_adjusted_x100;
  
  // first clean battery symbol
  ui8_lcd_frame_buffer[23] &= ~241;
  
  if (ui16_battery_SOC_voltage_x100 > (configuration_variables.ui8_battery_cells_number * LI_ION_CELL_VOLTS_83_x100)) // 4 bars | full
  {
    ui8_lcd_frame_buffer[23] |= 241;
  } 
  else if (ui16_battery_SOC_voltage_x100 > (configuration_variables.ui8_battery_cells_number * LI_ION_CELL_VOLTS_50_x100)) // 3 bars
  { 
    ui8_lcd_frame_buffer[23] |= 209;
  } 
  else if (ui16_battery_SOC_voltage_x100 > (configuration_variables.ui8_battery_cells_number * LI_ION_CELL_VOLTS_17_x100)) // 2 bars
  {
    ui8_lcd_frame_buffer[23] |= 145;
  }
  else if (ui16_battery_SOC_voltage_x100 > (configuration_variables.ui8_battery_cells_number * LI_ION_CELL_VOLTS_0_x100)) // 1 bar
  {
    ui8_lcd_frame_buffer[23] |= 144;
  }
  else // flashing
  {
    // empty, so flash the empty battery symbol
    if (ui8_lcd_menu_flash_state)
    {
      ui8_lcd_frame_buffer[23] |= 16;
    }
  }

  /*
  ui8_lcd_frame_buffer[23] |= 16;  // empty
  ui8_lcd_frame_buffer[23] |= 128; // bar number 1
  ui8_lcd_frame_buffer[23] |= 1;   // bar number 2
  ui8_lcd_frame_buffer[23] |= 64;  // bar number 3
  ui8_lcd_frame_buffer[23] |= 32;  // bar number 4
  */
}



void power_field(void)
{
  lcd_print(ui16_battery_power_step_filtered, BATTERY_POWER_FIELD, 0);
  lcd_enable_motor_symbol(1);
  lcd_enable_w_symbol(1);
}



void riding_mode_controller(void)
{
  static uint8_t ui8_long_hold_down_button;
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  // reset riding mode (safety)
  motor_controller_data.ui8_riding_mode = OFF_MODE;
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  // check button events and set assist level
  
  if (UP_CLICK)
  {
    // increment assist level
    if (++configuration_variables.ui8_assist_level > configuration_variables.ui8_number_of_assist_levels)
    {
      if (configuration_variables.ui8_eMTB_assist_function_enabled)
      {
        configuration_variables.ui8_assist_level = configuration_variables.ui8_number_of_assist_levels + 1;
      }
      else
      {
        configuration_variables.ui8_assist_level = configuration_variables.ui8_number_of_assist_levels;
      }
    }
  }
  
  if (DOWN_CLICK && !ui8_long_hold_down_button)
  {
    // decrement assist level
    if (configuration_variables.ui8_assist_level > 0)
    {
      --configuration_variables.ui8_assist_level;
    }
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  // set default riding mode

  if ((configuration_variables.ui8_assist_level > 0) && (configuration_variables.ui8_assist_level <= configuration_variables.ui8_number_of_assist_levels))
  {
    // set power assist riding mode
    if (configuration_variables.ui8_power_assist_function_enabled) { motor_controller_data.ui8_riding_mode = POWER_ASSIST_MODE; }
    
    // set torque assist riding mode
    if (configuration_variables.ui8_torque_assist_function_enabled) { motor_controller_data.ui8_riding_mode = TORQUE_ASSIST_MODE; }
    
    // set cadence assist riding mode
    if (configuration_variables.ui8_cadence_assist_function_enabled) { motor_controller_data.ui8_riding_mode = CADENCE_ASSIST_MODE; }
  }
  else if (configuration_variables.ui8_assist_level == configuration_variables.ui8_number_of_assist_levels + 1)
  {
    // set eMTB assist riding mode
    if (configuration_variables.ui8_eMTB_assist_function_enabled) { motor_controller_data.ui8_riding_mode = eMTB_ASSIST_MODE; }
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  // set walk assist or cruise
  
  #define BUTTON_DEBOUNCE_COUNTER_MAX                 100   // 100 -> 1.0 seconds, do not set over 255
  #define BUTTON_DEBOUNCE_COUNTER_MAX_CRUISE          20    // 20 -> 0.2 seconds, do not set over 255
  
  static uint8_t ui8_walk_assist_activated;
  static uint8_t ui8_cruise_activated;
  static uint8_t ui8_button_debounce_counter;
  
  if (DOWN_LONG_CLICK)
  {
    ui8_long_hold_down_button = 1;
  }
  
  if (ui8_long_hold_down_button)
  {
    // clear DOWN button events if button accidentally bounces when using Walk Assist or Cruise
    DOWN_CLICK = 0;
    DOWN_LONG_CLICK = 0;
    DOWN_CLICK_LONG_CLICK = 0;
    
    // if down button is pressed
    if (buttons_get_down_state())
    {
      // increment and limit button debounce counter to max
      if (ui8_button_debounce_counter < BUTTON_DEBOUNCE_COUNTER_MAX) { ++ui8_button_debounce_counter; };
    }
    else
    {
      // decrement and keep button debounce counter to zero
      if (ui8_button_debounce_counter > 0) { --ui8_button_debounce_counter; };
    }
    
    if (ui8_button_debounce_counter)
    {
      // enable walk assist or cruise if...
      if ((configuration_variables.ui8_walk_assist_function_enabled) &&
          (motor_controller_data.ui16_wheel_speed_x10 < WALK_ASSIST_THRESHOLD_SPEED_X10) &&
          !(ui8_cruise_activated) &&
          (configuration_variables.ui8_assist_level > 0) &&
          (configuration_variables.ui8_assist_level <= configuration_variables.ui8_number_of_assist_levels))
      {
        // enable walk assist
        lcd_enable_walk_symbol(1);
        motor_controller_data.ui8_riding_mode = WALK_ASSIST_MODE;
        
        // set flag indicating that walk assist was activated first during this button event
        ui8_walk_assist_activated = 1;
        
        // limit button debounce counter
        if (ui8_button_debounce_counter > configuration_variables.ui8_walk_assist_button_bounce_time) { ui8_button_debounce_counter = configuration_variables.ui8_walk_assist_button_bounce_time; };
      }
      else if ((configuration_variables.ui8_cruise_function_enabled) &&
               (motor_controller_data.ui16_wheel_speed_x10 > CRUISE_THRESHOLD_SPEED_X10) &&
               !(ui8_walk_assist_activated) &&
               !(configuration_variables.ui8_street_mode_function_enabled && configuration_variables.ui8_street_mode_cruise_enabled))
      {
        // enable cruise
        lcd_enable_cruise_symbol(1);
        motor_controller_data.ui8_riding_mode = CRUISE_MODE;
        
        // set flag indicating that cruise was activated first during this button event
        ui8_cruise_activated = 1;
        
        // limit button debounce counter
        if (ui8_button_debounce_counter > BUTTON_DEBOUNCE_COUNTER_MAX_CRUISE) { ui8_button_debounce_counter = BUTTON_DEBOUNCE_COUNTER_MAX_CRUISE; };
      }
      else
      {
        // reset button debounce counter
        ui8_button_debounce_counter = 0;
      }
    }
    else
    {
      // reset flags for walk assist and cruise activated
      ui8_walk_assist_activated = 0;
      ui8_cruise_activated = 0;
      
      // reset long button hold flag
      ui8_long_hold_down_button = 0;
    }
  }
}



void assist_level_field(void)
{
  static uint8_t ui8_street_mode_assist_symbol_state_counter;
  static uint8_t ui8_street_mode_assist_symbol_state;
  
  // display either assist level or eMTB symbol
  if (configuration_variables.ui8_assist_level > configuration_variables.ui8_number_of_assist_levels)
  {
    lcd_enable_E_symbol(1);
  }
  else
  {
    lcd_print(configuration_variables.ui8_assist_level, ASSIST_LEVEL_FIELD, 1);    
  }

  if (configuration_variables.ui8_street_mode_function_enabled && configuration_variables.ui8_street_mode_enabled)
  {
    lcd_enable_assist_symbol(1);
  }
  else if (configuration_variables.ui8_street_mode_function_enabled)
  {
    if (++ui8_street_mode_assist_symbol_state_counter > 45)
    {
      ui8_street_mode_assist_symbol_state_counter = 0;
      
      ui8_street_mode_assist_symbol_state = !ui8_street_mode_assist_symbol_state;
    }

    lcd_enable_assist_symbol(ui8_street_mode_assist_symbol_state);
  }
  else
  {
    lcd_enable_assist_symbol(1);
  }
}



void street_mode (void)
{
  static uint8_t ui8_executed_on_startup;
  
  if (configuration_variables.ui8_street_mode_function_enabled) 
  {
    // enable street mode if user has enabled street mode on startup
    if (!ui8_executed_on_startup)
    {
      ui8_executed_on_startup = 1;
      
      if (configuration_variables.ui8_street_mode_function_enabled > 1) 
      {
        configuration_variables.ui8_street_mode_enabled = 1;
      }
      else
      {
        configuration_variables.ui8_street_mode_enabled = 0;
      }
    }
    
    if (ONOFF_DOWN_LONG_CLICK)
    {
      configuration_variables.ui8_street_mode_enabled = !(configuration_variables.ui8_street_mode_enabled);
    }
  }
}



void brake (void)
{
  lcd_enable_brake_symbol(motor_controller_data.ui8_braking);
}



void lights(void)
{
  if (UP_LONG_CLICK)
  {
    // toggle lights state
    configuration_variables.ui8_lights_state = !configuration_variables.ui8_lights_state;
  }
  
  // set backlight brightness
  if (configuration_variables.ui8_lights_state) { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness); }
  else { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness); }
  
  // set light symbol on display
  lcd_enable_lights_symbol(configuration_variables.ui8_lights_state);
}



void odometer_increase_field_state(void)
{
  // increment odometer field state and check if out of bounds
  if (++configuration_variables.ui8_odometer_field_state > 8) // case 0 -> case 8, 9 odometer field states
  {
    // reset odometer field state
    configuration_variables.ui8_odometer_field_state = 0;
  }
}



void odometer_start_show_field_number(void)
{
  ui8_start_odometer_show_field_number = 1;
  ui8_odometer_show_field_number_counter = 0;
}



uint8_t reset_variable_check(void)
{
  static uint8_t ui8_odometer_reset_distance_counter_state;
  static uint16_t ui16_odometer_reset_distance_counter;
  
  if (DOWN_CLICK_LONG_CLICK)
  {
    // start counting to reset variable
    ui8_odometer_reset_distance_counter_state = 1;
    
    // reset counter for variable reset
    ui16_odometer_reset_distance_counter = 0;
  }
  
  if (ui8_odometer_reset_distance_counter_state)
  {
    if (buttons_get_down_state ())
    {
      // count time, after limit, reset everything
      if (++ui16_odometer_reset_distance_counter > 300)
      {
        // reset counter state
        ui8_odometer_reset_distance_counter_state = 0;
        
        // reset variable
        return 1;
      }
      
      // check if flash state for variable flashing
      if (ui8_lcd_menu_flash_state)
      {
        // do not display variable
        return 2;
      }
    }
    else // user is not pressing the down button anymore
    {
      ui8_odometer_reset_distance_counter_state = 0;
    }
  }
  
  // display variable as usual
  return 0;
}



void odometer_field(void)
{
  if (ONOFF_CLICK)
  {
    // increment odometer field state
    odometer_increase_field_state();
    
    // show field number
    odometer_start_show_field_number();
  }

  // if there are errors, show the error number on odometer field instead of any other information
  if (motor_controller_data.ui8_controller_system_state != NO_ERROR)
  {
    if (ui8_lcd_menu_flash_state)
    {
      lcd_print(motor_controller_data.ui8_controller_system_state, ODOMETER_FIELD, 0);
    }
  }
  else
  {
    switch (configuration_variables.ui8_odometer_field_state)
    {
      // trip distance 
      case 0:
        
        // check if user has disabled to show distance data in the odometer field
        if (!configuration_variables.ui8_show_distance_data_odometer_field)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_0, 2);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_0;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_0)
        {
          // trip distance
          case 0:
          
            switch (reset_variable_check ())
            {
              // display trip distance
              case 0:
                if (configuration_variables.ui8_units_type)
                {
                  // imperial units
                  lcd_print (((float) configuration_variables.ui32_trip_x10 / 1.6), ODOMETER_FIELD, 1);
                  lcd_enable_mil_symbol (1);
                }
                else
                {
                  // metric units
                  lcd_print (configuration_variables.ui32_trip_x10, ODOMETER_FIELD, 1);
                  lcd_enable_km_symbol (1);
                }
              break;
              
              // reset trip distance
              case 1:
                configuration_variables.ui32_trip_x10 = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;

          // distance since power on
          case 1:
          
            // display distance since power on in either imperial or metric units
            if (configuration_variables.ui8_units_type)
            {
              // imperial units
              lcd_print((float) configuration_variables.ui16_distance_since_power_on_x10 / 1.6, ODOMETER_FIELD, 1);
              // lcd_enable_dst_symbol (1); TODO: this fails, the symbol just work when we set it to 1 AND speed field number is equal or higher than 10.0. Seems the 3rd digit at left is needed.
              lcd_enable_mil_symbol (1);
            }
            else
            {
              // metric units
              lcd_print(configuration_variables.ui16_distance_since_power_on_x10, ODOMETER_FIELD, 1);
              // lcd_enable_dst_symbol (1); TODO: this fails, the symbol just work when we set it to 1 AND speed field number is equal or higher than 10.0. Seems the 3rd digit at left is needed.
              lcd_enable_km_symbol (1);
            }
            
          break;

          // odometer
          case 2:
          
            switch (reset_variable_check ())
            {
              // display odometer distance
              case 0:
                if (configuration_variables.ui8_units_type)
                {
                  // imperial units
                  lcd_print(((float) configuration_variables.ui32_odometer_x10 / 1.6), ODOMETER_FIELD, 1);
                  lcd_enable_odo_symbol (1);
                  lcd_enable_mil_symbol (1);
                }
                else
                {
                  // metric units
                  lcd_print(configuration_variables.ui32_odometer_x10, ODOMETER_FIELD, 1);
                  lcd_enable_odo_symbol (1);
                  lcd_enable_km_symbol (1);
                }
              break;
              
              // reset odometer distance
              case 1:
                configuration_variables.ui32_odometer_x10 = 0;
              break;
              
              // display nothing
              default:
              break;
            }
          
          break;
        }

      break; // end of distance

      // battery SOC
      case 1:
      
        // check if user has disabled to show battery state of charge in the odometer field
        if (!configuration_variables.ui8_show_battery_SOC_odometer_field || !configuration_variables.ui8_battery_SOC_function_enabled)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }

        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_1, 1);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_1;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_1)
        {
          // battery SOC in percentage
          case 0:
            lcd_print(ui16_battery_SOC_percentage, ODOMETER_FIELD, 0);
          break;

          // consumed watt-hours
          case 1:
            lcd_print(ui32_wh_x10, ODOMETER_FIELD, 1);
          break;
        }
      
      break; // end of battery SOC

      // battery state
      case 2:
      
        // check if user has disabled to show battery voltage or current in the odometer field
        if (!configuration_variables.ui8_show_battery_state_odometer_field)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
      
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_2, 1);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_2;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_2)
        {
          // voltage value
          case 0:
            lcd_print(ui16_battery_voltage_filtered_x1000 / 100, ODOMETER_FIELD, 1);
            lcd_enable_vol_symbol(1);
          break;

          // current value
          case 1:
            lcd_print(ui16_battery_current_filtered_x10, ODOMETER_FIELD, 1);
          break;
        }
      
      break; // end of battery state

      // pedal data
      case 3:
      
        // check if user has disabled to show pedal data in the odometer field
        if (!configuration_variables.ui8_show_pedal_data_odometer_field)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_3, 2);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_3;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_3)
        {
          case 0:
          
            // pedal power
            lcd_print(ui16_pedal_power_step_filtered, ODOMETER_FIELD, 0);
            
          break;

          case 1:
          
            // pedal cadence
            lcd_print(ui8_pedal_cadence_RPM_filtered, ODOMETER_FIELD, 0);
            
          break;
          
          case 2:
            
            // pedal weight
            lcd_print(ui16_pedal_weight_filtered_x100 / 10, ODOMETER_FIELD, 1);
            
          break;
        }
        
      break; // end of pedals

      // energy data
      case 4:
      
        // check if user has disabled to show energy data in the odometer field
        if (!configuration_variables.ui8_show_energy_data_odometer_field)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_4, 1);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_4;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_4)
        {
          case 0:
            
            // display energy consumption in either imperial or metric units
            if (configuration_variables.ui8_units_type)
            {
              // imperial units
              lcd_print((float) ui16_average_energy_consumption_since_power_on_x10 * 1.6, ODOMETER_FIELD, 1);
            }
            else
            {
              // metric units
              lcd_print(ui16_average_energy_consumption_since_power_on_x10, ODOMETER_FIELD, 1);
            }
          
          break;

          case 1:
            
            // check if user has disabled battery capacity function
            if (configuration_variables.ui8_battery_SOC_function_enabled == 0)
            {
              // function not enabled, go to next sub menu
              configuration_variables.ui8_odometer_sub_field_state_4 = 0;
              
              break;
            }
          
            // display estimated range since power on in either imperial or metric units
            if (configuration_variables.ui8_units_type)
            {
              // imperial units
              lcd_print((float) ui16_estimated_range_since_power_on_x10 / 1.6, ODOMETER_FIELD, 1);
              lcd_enable_mil_symbol (1);
            }
            else
            {
              // metric units
              lcd_print(ui16_estimated_range_since_power_on_x10, ODOMETER_FIELD, 1);
              lcd_enable_km_symbol (1);
            }
            
          break;
        }
        
      break; // end of energy data
      
      // time measurement
      case 5:
      
        // check if user has disabled to show time measurement in the odometer field
        if (!configuration_variables.ui8_show_time_measurement_odometer_field)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_5, 1);

        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_5;

        switch (configuration_variables.ui8_odometer_sub_field_state_5)
        {
          // time measurement since power on (TM)
          case 0:
            
            // display time measurement since power on (TM) in time measurement field
            configuration_variables.ui8_time_measurement_field_state = 1;
            
            switch (reset_variable_check ())
            {
              // display total minutes since power on (TM)
              case 0:
                lcd_print(ui16_minute_TM, ODOMETER_FIELD, 0);
              break;
              
              // reset time measurement since power on (TM)
              case 1:
                ui8_second_TM = 0;
                ui16_minute_TM = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;

          // time measurement since last reset (TTM)
          case 1:
            
            // display total time measurement since last reset (TTM) in time measurement field
            configuration_variables.ui8_time_measurement_field_state = 0;
            
            switch (reset_variable_check ())
            {
              // display total hours passed from TTM
              case 0:
                lcd_print(configuration_variables.ui16_total_hour_TTM, ODOMETER_FIELD, 0);
              break;
              
              // reset total time measurement since last reset (TTM)
              case 1:
                configuration_variables.ui8_total_second_TTM = 0;
                configuration_variables.ui8_total_minute_TTM = 0;
                configuration_variables.ui16_total_hour_TTM = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;
        }
        
      break; // end of time measurement
    
      case 6: // wheel speed
      
        // check if user has disabled to show wheel speed in the odometer field
        if (!configuration_variables.ui8_show_wheel_speed_odometer_field)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        // advance sub field on click-up-click-long-up button event
        advance_on_subfield (&configuration_variables.ui8_odometer_sub_field_state_6, 2);
        
        // set for flashing of sub field state number
        ui8_odometer_sub_field_state = configuration_variables.ui8_odometer_sub_field_state_6;
        
        switch (configuration_variables.ui8_odometer_sub_field_state_6)
        {
          // wheel speed
          case 0:
          
            // set wheel speed field state
            configuration_variables.ui8_wheel_speed_field_state = 0;
            
            // display wheel speed in either imperial or metric units in the odometer field
            if (configuration_variables.ui8_units_type)
            {
              // imperial
              lcd_print(((float) motor_controller_data.ui16_wheel_speed_x10 / 1.6), ODOMETER_FIELD, 1);
            }
            else
            {
              // metric
              lcd_print(motor_controller_data.ui16_wheel_speed_x10, ODOMETER_FIELD, 1);
            }
          break;
          
          // average wheel speed since power on
          case 1:
          
            // set wheel speed field state
            configuration_variables.ui8_wheel_speed_field_state = 1;
            
            // display average wheel speed in either imperial or metric units in the odometer field
            if (configuration_variables.ui8_units_type)
            {
              // imperial
              lcd_print(((float) ui16_average_measured_wheel_speed_x10 / 1.6), ODOMETER_FIELD, 1);
            }
            else
            {
              // metric
              lcd_print(ui16_average_measured_wheel_speed_x10, ODOMETER_FIELD, 1);
            }
          break;
          
          // maximum measured wheel speed since power on
          case 2:
          
            // set wheel speed field state
            configuration_variables.ui8_wheel_speed_field_state = 2;
            
            switch (reset_variable_check())
            {
              // display max measured wheel speed in either imperial or metric units in the odometer field
              case 0:
                if (configuration_variables.ui8_units_type)
                {
                  // imperial
                  lcd_print((float) ui16_max_measured_wheel_speed_x10 / 1.6, ODOMETER_FIELD, 1);
                }
                else
                {
                  // metric
                  lcd_print(ui16_max_measured_wheel_speed_x10, ODOMETER_FIELD, 1);
                }
              break;
              
              // reset maximum measured wheel speed since power on
              case 1:
                ui16_max_measured_wheel_speed_x10 = 0;
              break;
              
              // display nothing
              default:
              break;
            }
            
          break;
        }
        
      break; // end of wheel speed
      
      // motor temperature
      case 7:
      
        // check if user has enabled temperature limit function and enabled to show the value in the odometer field
        if (!configuration_variables.ui8_show_motor_temperature_odometer_field || (configuration_variables.ui8_optional_ADC_function != TEMPERATURE_CONTROL))
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        lcd_print(motor_controller_data.ui8_motor_temperature, ODOMETER_FIELD, 0);

      break; // end of motor temperature
      
      // cruise
      case 8:
        
        // check if user has enabled the set target speed feature and also enabled to show cruise set target speed in the odometer field
        if (!configuration_variables.ui8_show_cruise_function_set_target_speed || !configuration_variables.ui8_cruise_function_set_target_speed_enabled)
        {
          // increment odometer field state
          odometer_increase_field_state();
          
          break;
        }
        
        // display cruise target speed in either imperial or metric units
        if (configuration_variables.ui8_units_type)
        {
          // imperial
          lcd_print (configuration_variables.ui8_cruise_function_target_speed_mph * 10, ODOMETER_FIELD, 1);
        }
        else
        {
          // metric
          lcd_print (configuration_variables.ui8_cruise_function_target_speed_kph * 10, ODOMETER_FIELD, 1);
        }
        
      break; // end of cruise
    }

    if (ui8_start_odometer_show_field_number)
    {
      // limit field number flashing after a certain time
      if (++ui8_odometer_show_field_number_counter > 200) // 200 -> 2.0 seconds
      {
        // reset counter
        ui8_odometer_show_field_number_counter = 0;
        
        // disable the flashing
        ui8_start_odometer_show_field_number = 0;
      }

    
      if (ui8_lcd_menu_flash_state)
      {
        ui8_odometer_show_field_number = ((configuration_variables.ui8_odometer_field_state + 1) * 10); // add units for show (x10)
        ui8_odometer_show_field_number += ui8_odometer_sub_field_state;

        lcd_print (ui8_odometer_show_field_number, WHEEL_SPEED_FIELD, 1);
      }
    }
  }
}



void wheel_speed_field(void)
{
  // check if wheel speed is higher than maximum measured wheel speed
  if (motor_controller_data.ui16_wheel_speed_x10 > ui16_max_measured_wheel_speed_x10)
  {
    // wheel speed is higher than maximum measured wheel speed so update variable
    ui16_max_measured_wheel_speed_x10 = motor_controller_data.ui16_wheel_speed_x10;
  }
  
  // calculate average wheel speed since power on in km/s
  ui16_average_measured_wheel_speed_x10 = ((uint32_t) configuration_variables.ui16_distance_since_power_on_x10 * 3600) / ui16_seconds_since_power_on;
  
  // show wheel speed only when we should not show odometer field number
  if (ui8_start_odometer_show_field_number == 0)
  {
    switch (configuration_variables.ui8_wheel_speed_field_state)
    {
      // display wheel speed
      case 0:
      
        if (configuration_variables.ui8_units_type)
        {
          lcd_print(((float) motor_controller_data.ui16_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 1);
          lcd_enable_mph_symbol(1);
        }
        else
        {
          lcd_print(motor_controller_data.ui16_wheel_speed_x10, WHEEL_SPEED_FIELD, 1);
          lcd_enable_kmh_symbol(1);
        }
        
      break;
      
      // display average wheel speed since power on
      case 1:
      
        if (configuration_variables.ui8_units_type)
        {
          lcd_print(((float) ui16_average_measured_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 1);
          lcd_enable_mph_symbol(1);
        }
        else
        {
          lcd_print(ui16_average_measured_wheel_speed_x10, WHEEL_SPEED_FIELD, 1);
          lcd_enable_kmh_symbol(1);
        }
        
        lcd_enable_avs_symbol(1);
        
      break;
      
      // display maximum measured wheel speed since power on
      case 2:
      
        if (configuration_variables.ui8_units_type)
        {
          lcd_print(((float) ui16_max_measured_wheel_speed_x10 / 1.6), WHEEL_SPEED_FIELD, 1);
          lcd_enable_mph_symbol(1);
        }
        else
        {
          lcd_print(ui16_max_measured_wheel_speed_x10, WHEEL_SPEED_FIELD, 1);
          lcd_enable_kmh_symbol(1);
        }
        
        lcd_enable_mxs_symbol(1);
        
      break;
    }
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



void lcd_print(uint32_t ui32_number, uint8_t ui8_lcd_field, uint8_t ui8_options)
{
/*   #define DECIMAL   1

  switch (ui8_lcd_field)
  {
    case ASSIST_LEVEL_FIELD:
    
      // first delete the field
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field]] &= NUMBERS_MASK;
      
      // extract first digit
      uint8_t ui8_digit = ui32_number % 10;
      
      // print digit in field
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field]] &= ui8_lcd_digit_mask[NUMBERS_MASK];   

    break;
    
    case ODOMETER_FIELD:
    
      // first delete the old number in the field and then print the new number, digit by digit
      for (uint8_t ui8_counter = 0; ui8_counter < 5; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if ((ui8_options == DECIMAL) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else if (ui8_counter > 1 && ui32_number == 0) // print empty (NUMBERS_MASK) if digit is zero
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
        }
        
        // shift number so next digit is prepared to be printed
        ui32_number >> 1;
      }
      
      // enable decimal point
      if (ui8_options == DECIMAL) { lcd_enable_odometer_point_symbol(1); }
      else { lcd_enable_odometer_point_symbol(0); }
      
    break;
    
    case TEMPERATURE_FIELD:
    
      for (uint8_t ui8_counter = 0; ui8_counter < 2; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if ((ui8_options == DECIMAL) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else if ((ui8_counter > 0) && (ui32_number == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] |= ui8_lcd_digit_mask[ui8_digit];
        }

        ui32_number >> 1;
      }
  
      // enable only the "1 symbol" if temperature is > 99
      if (ui32_number > 99) { lcd_enable_temperature_1_symbol(1); }
      else { lcd_enable_temperature_1_symbol(0); }
    
    break;
    
    case WHEEL_SPEED_FIELD:
  
      for (uint8_t ui8_counter = 0; ui8_counter < 3; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if ((ui8_options == DECIMAL) && (ui8_counter == 0))
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else if (ui8_counter > 1 && ui32_number == 0) // print only first 2 zeros
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
        
        ui32_number >> 1;
      }
      
      // enable decimal point
      if (ui8_options == DECIMAL) { lcd_enable_wheel_speed_point_symbol(1); }
      else { lcd_enable_wheel_speed_point_symbol(0); }
      
    break;
    
    case BATTERY_POWER_FIELD:
    
      for (uint8_t ui8_counter = 0; ui8_counter < 3; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;

        if (ui8_counter > 0 && ui32_number == 0) // print only first zero
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }

        ui32_number >> 1;
      }
      
      // enable the "1 symbol" if power is > 999
      if (ui32_number >= 1000) { lcd_enable_battery_power_1_symbol(1); }
      else { lcd_enable_battery_power_1_symbol(0); }
    
    break;
    
    case TIME_SECOND_FIELD:
    
      for (uint8_t ui8_counter = 0; ui8_counter < 2; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;
        
        ui8_lcd_frame_buffer[SECOND_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];

        ui32_number >> 1;
      }
      
    break;
    
    case TIME_MINUTE_FIELD:
    
      // first delete the old number in the field and then print the new number, digit by digit
      for (uint8_t ui8_counter = 0; ui8_counter < 3; ui8_counter++)
      {
        ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;

        uint8_t ui8_digit = ui32_number % 10;
        
        if (ui8_counter > 0 && ui32_number == 0)
        {
          ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
        }
        else
        {
          ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
        }
        
        ui32_number >> 1;
      }
    
    break;
    
    default:
      // out of bounds
    break;
  } */
  
  uint8_t ui8_counter;
  uint8_t ui8_digit;
  
  // multiply the value by 10 to not show decimal digit if ...
  if( (ui8_options == 0) && (ui8_lcd_field != ASSIST_LEVEL_FIELD) && (ui8_lcd_field != BATTERY_POWER_FIELD) && (ui8_lcd_field != TIME_SECOND_FIELD) && (ui8_lcd_field != TIME_MINUTE_FIELD) )
  {
    ui32_number *= 10;
  }
  
  // first delete the field
  for (ui8_counter = 0; ui8_counter < 5; ui8_counter++)
  {
    if (ui8_lcd_field == ASSIST_LEVEL_FIELD || ui8_lcd_field == ODOMETER_FIELD || ui8_lcd_field == TEMPERATURE_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] - ui8_counter] &= NUMBERS_MASK;
    }

    // because the LCD mask/layout is different on some field, like numbers would be inverted
    if (ui8_lcd_field == WHEEL_SPEED_FIELD || ui8_lcd_field == BATTERY_POWER_FIELD || ui8_lcd_field == TIME_SECOND_FIELD || ui8_lcd_field == TIME_MINUTE_FIELD)
    {
      ui8_lcd_frame_buffer[ui8_lcd_field_offset[ui8_lcd_field] + ui8_counter] &= NUMBERS_MASK;
    }

    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TEMPERATURE_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TIME_SECOND_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == TIME_MINUTE_FIELD) break;
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
  if (ui8_options == 0)
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

    if (ui8_lcd_field == ASSIST_LEVEL_FIELD || ui8_lcd_field == ODOMETER_FIELD || ui8_lcd_field == TEMPERATURE_FIELD)
    {

      if ((ui8_options == 0) && (ui8_counter == 0))
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
    if (ui8_lcd_field == WHEEL_SPEED_FIELD || ui8_lcd_field == BATTERY_POWER_FIELD)
    {
      if (ui8_lcd_field == WHEEL_SPEED_FIELD)
      {
        if ((ui8_options == 0) && (ui8_counter == 0))
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
    
    if (ui8_lcd_field == TIME_SECOND_FIELD)
    {
      ui8_lcd_frame_buffer[SECOND_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
    }
    
    if (ui8_lcd_field == TIME_MINUTE_FIELD)
    {
      if (ui8_counter > 0 && ui32_number == 0)
      {
        ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] &= ui8_lcd_digit_mask[NUMBERS_MASK];
      }
      else
      {
        ui8_lcd_frame_buffer[MINUTE_DIGIT_OFFSET + ui8_counter] |= ui8_lcd_digit_mask_inverted[ui8_digit];
      }
    }
    
    // limit the number of printed digits for each field
    if (ui8_counter == 0 && ui8_lcd_field == ASSIST_LEVEL_FIELD) break;
    if (ui8_counter == 4 && ui8_lcd_field == ODOMETER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TEMPERATURE_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == WHEEL_SPEED_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == BATTERY_POWER_FIELD) break;
    if (ui8_counter == 1 && ui8_lcd_field == TIME_SECOND_FIELD) break;
    if (ui8_counter == 2 && ui8_lcd_field == TIME_MINUTE_FIELD) break;

    ui32_number /= 10;
  }
}



void lcd_enable_w_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[9] |= 128; }
}

void lcd_enable_odometer_point_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[6] |= 8; }
}

void lcd_enable_brake_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[23] |= 4; }
}

void lcd_enable_lights_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[23] |= 2; }
}

void lcd_enable_cruise_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[0] |= 16; }
}

void lcd_enable_assist_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[1] |= 8; }
}

void lcd_enable_vol_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[2] |= 8; }
}

void lcd_enable_odo_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[3] |= 8; }
}

void lcd_enable_km_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[4] |= 8; }
}

void lcd_enable_mil_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[5] |= 8; }
}

void lcd_enable_temperature_1_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[7] |= 8; }
}

void lcd_enable_battery_power_1_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[12] |= 8; }
}

void lcd_enable_temperature_minus_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[8] |= 8; }
}

void lcd_enable_temperature_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[9] |= 16; }
}

void lcd_enable_temperature_farneight_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[9] |= 32; }
}

void lcd_enable_fahrenheit_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[9] |= 1; }
}

void lcd_enable_motor_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[9] |= 2; }
}

void lcd_enable_degrees_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[9] |= 64; }
}

void lcd_enable_kmh_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[13] |= 1; }
}

void lcd_enable_wheel_speed_point_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[13] |= 8; }
}

void lcd_enable_avs_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[13] |= 16; }
}

void lcd_enable_mxs_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[13] |= 32; }
}

void lcd_enable_walk_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[13] |= 64; }
}

void lcd_enable_mph_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[13] |= 128; }
}

void lcd_enable_dst_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[16] |= 8; }
}

void lcd_enable_tm_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[17] |= 16; }
}

void lcd_enable_ttm_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[17] |= 32; }
}

void lcd_enable_colon_symbol (uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[23] |= 8; }
}

void lcd_enable_E_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 181; }
}

void lcd_enable_torque_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 162; }
}

void lcd_enable_Model_3_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 148; }
}

void lcd_enable_P_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 167; }
}

void lcd_enable_C_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 53; }
}

void lcd_enable_A_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 231; }
}

void lcd_enable_street_mode_symbol(uint8_t ui8_state)
{
  if (ui8_state) { ui8_lcd_frame_buffer[ui8_lcd_field_offset[ASSIST_LEVEL_FIELD]] |= 23; }
}



void filter_variables()
{
  #define FILTER_VARIABLES_COUNTER_MAX    10    // 10 -> filter every 100 ms, this matches the time it takes to receive new variables from the motor controller
  
  static uint8_t ui8_filter_variables_counter;
  
  // filter variables
  if (++ui8_filter_variables_counter > FILTER_VARIABLES_COUNTER_MAX)
  {
    // reset filter counter
    ui8_filter_variables_counter = 0;
    
    // battery voltage  
    ui16_battery_voltage_filtered_x1000 = filter(motor_controller_data.ui16_battery_voltage_x1000, ui16_battery_voltage_filtered_x1000, 60);
    
    // battery current
    ui16_battery_current_filtered_x10 = filter(motor_controller_data.ui8_battery_current_x10, ui16_battery_current_filtered_x10, 60);
    
    // battery power
    uint32_t ui32_battery_power_temp_x10 = ((uint32_t) motor_controller_data.ui16_battery_voltage_x1000 * motor_controller_data.ui8_battery_current_x10) / 1000;
    ui16_battery_power_filtered_x10 = filter(ui32_battery_power_temp_x10, ui16_battery_power_filtered_x10, 72);
    ui16_battery_power_step_filtered = ui16_battery_power_filtered_x10 / 100;
    ui16_battery_power_step_filtered = ui16_battery_power_step_filtered * 10;
    
    // pedal cadence
    ui8_pedal_cadence_RPM_filtered = filter(motor_controller_data.ui8_pedal_cadence_RPM, ui8_pedal_cadence_RPM_filtered, 52);
    
    // human power
    ui16_pedal_power_filtered_x10 = filter(motor_controller_data.ui16_pedal_power_x10, ui16_pedal_power_filtered_x10, 72);
    ui16_pedal_power_step_filtered = ui16_pedal_power_filtered_x10 / 100;
    ui16_pedal_power_step_filtered = ui16_pedal_power_step_filtered * 10;
    
    // pedal weight
    uint16_t ui16_pedal_weight_temp_x100 = ((uint32_t) motor_controller_data.ui16_pedal_torque_x100 * 100) / 167;
    ui16_pedal_weight_filtered_x100 = filter(ui16_pedal_weight_temp_x100, ui16_pedal_weight_filtered_x100, 52);
    ui16_pedal_weight_x100 = ui16_pedal_weight_temp_x100;
    
    /*-----------------------------------------------------------------
      
      NOTE: regarding the weight on pedal calculation
      
      Force (Nm) = Weight (Kg) * 9.81 * 0.17 (0.17 = arm cranks size)
      
      Weight (kg) = Force (Nm) / 9.81 * 0.17 (0.17 = arm cranks size)
    -----------------------------------------------------------------*/
  }
}



void calc_distance (void)
{
  // calculate how many revolutions since last reset and convert to distance traveled
  uint32_t ui32_temp = (motor_controller_data.ui32_wheel_speed_sensor_tick_counter - motor_controller_data.ui32_wheel_speed_sensor_tick_counter_offset) * configuration_variables.ui16_wheel_perimeter;
  
  // if traveled distance is more than 100 meters update all distance variables and reset
  if (ui32_temp >= 100000) // 100000 -> 100000 mm -> 0.1 km
  {
    // update all distance variables
    configuration_variables.ui16_distance_since_power_on_x10 += 1;
    configuration_variables.ui32_odometer_x10 += 1;
    configuration_variables.ui32_trip_x10 += 1;
   
    // reset the always incrementing value (up to motor controller power reset) by setting the offset to current value
    motor_controller_data.ui32_wheel_speed_sensor_tick_counter_offset = motor_controller_data.ui32_wheel_speed_sensor_tick_counter;
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
  ht1622_init();
  lcd_set_frame_buffer();
  lcd_update();
  
  // set lights state and backlight brightness
  if (configuration_variables.ui8_light_mode == 0) { configuration_variables.ui8_lights_state = 0; }
  else if (configuration_variables.ui8_light_mode == 1) { configuration_variables.ui8_lights_state = 1; }
  
  if (configuration_variables.ui8_lights_state) { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness); }
  else { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness); }
}



void lcd_set_backlight_intensity(uint8_t ui8_intensity)
{
  if (ui8_intensity == 0)
  {
    TIM1_CCxCmd (TIM1_CHANNEL_4, DISABLE);
  }
  else if (ui8_intensity <= 20)
  {
    TIM1_SetCompare4((uint16_t) ui8_intensity);
    TIM1_CCxCmd(TIM1_CHANNEL_4, ENABLE);
  }
}



void update_menu_flashing_state(void)
{
  static uint8_t ui8_lcd_menu_flash_counter;
  static uint16_t ui16_lcd_menu_flash_counter_temperature;
  
  #define LCD_MENU_FLASH_THRESHOLD              25
  #define LCD_TEMPERATURE_FLASH_OFF_THRESHOLD   10  // 10 -> 100 ms 

  // flash on menus
  if (++ui8_lcd_menu_flash_counter > LCD_MENU_FLASH_THRESHOLD)
  {
    ui8_lcd_menu_flash_counter = 0;
    
    ui8_lcd_menu_flash_state = !ui8_lcd_menu_flash_state;
  }

  // flash the temperature field when the current is being limited due to motor over temperature
  if ((configuration_variables.ui8_optional_ADC_function == TEMPERATURE_CONTROL) && (motor_controller_data.ui8_temperature_current_limiting_value < 255)) // flash only if current is being limited, i.e. below 255
  {
    if (ui8_lcd_menu_flash_state_temperature == 0)
    {
      if (++ui16_lcd_menu_flash_counter_temperature > LCD_TEMPERATURE_FLASH_OFF_THRESHOLD)
      {
        ui16_lcd_menu_flash_counter_temperature = 0;
        
        ui8_lcd_menu_flash_state_temperature = 1;
      }
    }
    else if (++ui16_lcd_menu_flash_counter_temperature > motor_controller_data.ui8_temperature_current_limiting_value + LCD_TEMPERATURE_FLASH_OFF_THRESHOLD)
    {
      ui16_lcd_menu_flash_counter_temperature = 0;
      
      ui8_lcd_menu_flash_state_temperature = 0;
    }
  }
  else
  {
    ui8_lcd_menu_flash_state_temperature = 1;
  }
}



void submenu_state_controller(uint8_t ui8_state_max_number)
{
  if (ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    // stop changing variables
    if (ONOFF_CLICK || ONOFF_LONG_CLICK)
    {
      ui8_lcd_menu_config_submenu_change_variable_enabled = 0;
    }
  }
  else
  {
    // advance on submenus
    if (UP_CLICK)
    {
      if (ui8_lcd_menu_config_submenu_state < ui8_state_max_number) { ++ui8_lcd_menu_config_submenu_state; } 
      else { ui8_lcd_menu_config_submenu_state = 0; }
    }

    // recede on submenus
    if (DOWN_CLICK)
    {
      if (ui8_lcd_menu_config_submenu_state > 0) { --ui8_lcd_menu_config_submenu_state; } 
      else { ui8_lcd_menu_config_submenu_state = ui8_state_max_number; }
    }
    
    // change variables
    if (ONOFF_CLICK)
    {
      ui8_lcd_menu_config_submenu_change_variable_enabled = 1;
    }
    
    // leave config menu
    if (ONOFF_LONG_CLICK)
    {
      ui8_lcd_menu_config_submenu_active = 0;
      ui8_lcd_menu_config_submenu_state = 0;
      
      // set backlight brightness
      if (configuration_variables.ui8_lights_state) { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_on_brightness); }
      else { lcd_set_backlight_intensity(configuration_variables.ui8_lcd_backlight_off_brightness); }
    }
  }
}



void advance_on_subfield (uint8_t* ui8_p_state, uint8_t ui8_state_max_number)
{
  if (UP_CLICK_LONG_CLICK)
  {
    if (*ui8_p_state < ui8_state_max_number) { ++*ui8_p_state; } 
    else { *ui8_p_state = 0; }
    
    odometer_start_show_field_number();
  }
}



void lcd_power_off()
{
  // add consumed watt-hours to watt-hours variable 
  configuration_variables.ui32_wh_x10_offset = ui32_wh_x10;
  
  // save variables to EEPROM
  EEPROM_controller(WRITE_TO_MEMORY);
  
  // clear LCD so it is clear to user what is happening
  lcd_clear();
  lcd_update();

  // now disable the power to the system
  GPIO_WriteLow(LCD3_ONOFF_POWER__PORT, LCD3_ONOFF_POWER__PIN);

  // block here
  while (1);
}



void change_variable(uint32_t *ui32_variable, uint32_t ui32_min_value, uint32_t ui32_max_value, uint32_t ui32_increment_step)
{
  static uint8_t ui8_long_click_started;
  static uint8_t ui8_long_click_counter;
  uint8_t ui8_long_click_trigger = 0;
  
  // if LONG CLICK, keep track of long click so variable is increased automatically 10x every second
  if (UP_LONG_CLICK  || DOWN_LONG_CLICK)
  {
    ui8_long_click_started = 1;
  }

  // trigger at every X ms if UP/DOWN LONG CLICK
  if ((ui8_long_click_started == 1) && (buttons_get_up_state() || buttons_get_down_state()))
  {
    if (++ui8_long_click_counter > 9)
    {
      ui8_long_click_counter = 0;
      ui8_long_click_trigger = 1;
    }
  }
  else
  {
    ui8_long_click_started = 0;
    ui8_long_click_counter = 0;
  }
  
  // decrease
  if (DOWN_CLICK || (buttons_get_down_state() && ui8_long_click_trigger))
  {
    if ((*ui32_variable) >= (ui32_min_value + ui32_increment_step)) { *ui32_variable -= ui32_increment_step; }
    else { *ui32_variable = ui32_min_value; }
  }
  
  // increase
  if (UP_CLICK || (buttons_get_up_state() && ui8_long_click_trigger))
  {
    if ((*ui32_variable) <= (ui32_max_value - ui32_increment_step)) { *ui32_variable += ui32_increment_step; }
    else { *ui32_variable = ui32_max_value; }
  }
}



void lcd_configurations_print_number(var_number_t* p_lcd_var_number)
{
  static uint8_t ui8_long_click_started;
  static uint8_t ui8_long_click_counter;
  
  uint8_t *ui8_p_var = 0;
  uint16_t *ui16_p_var = 0;
  uint32_t *ui32_p_var = 0;
  uint32_t ui32_value = 0;
  uint8_t ui8_long_click_trigger = 0;

  if(p_lcd_var_number->ui8_size == 8)
  {
    ui8_p_var = ((uint8_t *) p_lcd_var_number->p_var_number);
  }
  else if(p_lcd_var_number->ui8_size == 16)
  {
    ui16_p_var = ((uint16_t *) p_lcd_var_number->p_var_number);
  }
  else if(p_lcd_var_number->ui8_size == 32)
  {
    ui32_p_var = ((uint32_t *) p_lcd_var_number->p_var_number);
  }
  
  if (ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    // if LONG CLICK, keep track of long click so variable is increased automatically 10x every second
    if (UP_LONG_CLICK  || DOWN_LONG_CLICK)
    {
      ui8_long_click_started = 1;
    }

    // trigger at every 100 ms if UP/DOWN LONG CLICK
    if((ui8_long_click_started == 1) && (buttons_get_up_state() || buttons_get_down_state()))
    {
      if(++ui8_long_click_counter >= 10)
      {
        ui8_long_click_counter = 0;
        ui8_long_click_trigger = 1;
      }
    }
    else
    {
      ui8_long_click_started = 0;
      ui8_long_click_counter = 0;
    }

    // increase
    if (UP_CLICK || (buttons_get_up_state() && ui8_long_click_trigger))
    {
      if(p_lcd_var_number->ui8_size == 8)
      {
        if((*ui8_p_var) <= (p_lcd_var_number->ui32_max_value - p_lcd_var_number->ui32_increment_step)) { (*ui8_p_var) += p_lcd_var_number->ui32_increment_step; }
        else { (*ui8_p_var) = (uint8_t) p_lcd_var_number->ui32_max_value; }
      }
      else if(p_lcd_var_number->ui8_size == 16)
      {
        if((*ui16_p_var) <= (p_lcd_var_number->ui32_max_value - p_lcd_var_number->ui32_increment_step)) { (*ui16_p_var) += p_lcd_var_number->ui32_increment_step; }
        else { (*ui16_p_var) = (uint16_t) p_lcd_var_number->ui32_max_value; }
      }
      else if(p_lcd_var_number->ui8_size == 32)
      {
        if((*ui32_p_var) <= (p_lcd_var_number->ui32_max_value - p_lcd_var_number->ui32_increment_step)) { (*ui32_p_var) += p_lcd_var_number->ui32_increment_step; }
        else { (*ui32_p_var) = p_lcd_var_number->ui32_max_value; }
      }
    }

    // decrease
    if (DOWN_CLICK || (buttons_get_down_state() && ui8_long_click_trigger))
    {
      if(p_lcd_var_number->ui8_size == 8)
      {
        if((*ui8_p_var) >= (p_lcd_var_number->ui32_min_value + p_lcd_var_number->ui32_increment_step)) { (*ui8_p_var) -= p_lcd_var_number->ui32_increment_step; }
        else { (*ui8_p_var) = (uint8_t) p_lcd_var_number->ui32_min_value; }
      }
      else if(p_lcd_var_number->ui8_size == 16)
      {
        if((*ui16_p_var) >= (p_lcd_var_number->ui32_min_value + p_lcd_var_number->ui32_increment_step)) { (*ui16_p_var) -= p_lcd_var_number->ui32_increment_step; }
        else { (*ui16_p_var) = (uint16_t) p_lcd_var_number->ui32_min_value; }
      }
      else if(p_lcd_var_number->ui8_size == 32)
      {
        if((*ui32_p_var) >= (p_lcd_var_number->ui32_min_value + p_lcd_var_number->ui32_increment_step)) { (*ui32_p_var) -= p_lcd_var_number->ui32_increment_step; }
        else { (*ui32_p_var) = p_lcd_var_number->ui32_min_value; }
      }
    }
  }
  
  if(p_lcd_var_number->ui8_size == 8)
  {
    ui32_value = (uint32_t) (*ui8_p_var);
  }
  else if(p_lcd_var_number->ui8_size == 16)
  {
    ui32_value = (uint32_t) (*ui16_p_var);
  }
  else if(p_lcd_var_number->ui8_size == 32)
  {
    ui32_value = (*ui32_p_var);
  }

  // draw only at every ui8_lcd_menu_flash_state -- will flash the number on the LCD
  if(ui8_lcd_menu_flash_state || !ui8_lcd_menu_config_submenu_change_variable_enabled)
  {
    lcd_print(ui32_value, p_lcd_var_number->ui8_odometer_field, p_lcd_var_number->ui8_decimal_digit);
  }
}