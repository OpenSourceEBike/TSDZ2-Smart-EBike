/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// This values were taken from a discharge graph of Samsung INR18650-25R cells, at almost no current discharge
// This graph: https://endless-sphere.com/forums/download/file.php?id=183920&sid=b7fd7180ef87351cabe74a22f1d162d7
#define LI_ION_CELL_VOLTS_83    3.96
#define LI_ION_CELL_VOLTS_50    3.70
#define LI_ION_CELL_VOLTS_17    3.44
#define LI_ION_CELL_VOLTS_0     3.30

// Battery voltage (readed on motor controller):
#define ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000 866

// Battery voltage (readed on LCD3):
// 30.0V --> 447 | 0.0671 volts per each ADC unit
// 40.0V --> 595 | 0.0672 volts per each ADC unit

// Possible values: 0, 1, 2, 3, 4, 5, 6
// 0 equal to no filtering and no delay, higher values will increase filtering but will also add bigger delay
#define BATTERY_VOLTAGE_FILTER_COEFFICIENT 6
#define BATTERY_CURRENT_FILTER_COEFFICIENT 5
#define TORQUE_FILTER_COEFFICIENT          5
#define PEDAL_CADENCE_FILTER_COEFFICIENT   2

#endif /* CONFIG_H_ */
