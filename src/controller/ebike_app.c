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
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "eeprom.h"
#include "lights.h"
#include "common.h"

volatile struct_configuration_variables m_configuration_variables;

// system variables
static uint8_t    ui8_riding_mode = OFF_MODE;
static uint8_t    ui8_riding_mode_parameter = 0;
static uint8_t    ui8_system_state = NO_ERROR;
static uint8_t    ui8_brakes_enabled = 0;
static uint8_t    ui8_motor_enabled = 0;


// power control variables
static uint16_t   ui16_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
static uint16_t   ui16_duty_cycle_ramp_up_inverse_step_default = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
static uint16_t   ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
static uint16_t   ui16_battery_voltage_filtered_x1000 = 0;
static uint8_t    ui8_battery_current_filtered_x10 = 0;
static uint8_t    ui8_adc_battery_current_max = ADC_10_BIT_BATTERY_CURRENT_MAX;
static uint8_t    ui8_adc_battery_current_target = 0;
static uint8_t    ui8_duty_cycle_target = 0;


// cadence sensor variables
volatile uint8_t ui8_cadence_sensor_mode = STANDARD_MODE;
volatile uint16_t ui16_cadence_sensor_ticks_counter_min_speed_adjusted = CADENCE_SENSOR_STANDARD_MODE_TICKS_COUNTER_MIN;
static uint16_t ui16_cadence_sensor_pulse_high_percentage_x10 = CADENCE_SENSOR_PULSE_PERCENTAGE_X10_DEFAULT;
static uint8_t ui8_pedal_cadence_RPM = 0;


// torque sensor variables
volatile uint16_t ui16_adc_pedal_torque = 0;
static uint16_t   ui16_adc_pedal_torque_delta = 0;
static uint16_t   ui16_pedal_power_x10 = 0;
static uint16_t   ui16_pedal_torque_x100 = 0;


// wheel speed sensor variables
static uint16_t   ui16_wheel_speed_x10 = 0;


// throttle function variables
volatile uint8_t  ui8_adc_throttle = 0;


// eMTB assist variables
static const uint16_t eMTB_power_1_3[256] = {0, 1, 2, 4, 6, 8, 10, 13, 15, 17, 20, 23, 25, 28, 31, 34, 37, 40, 43, 46, 49, 52, 56, 59, 62, 66, 69, 73, 76, 80, 83, 87, 91, 94, 98, 102, 105, 109, 113, 117, 121, 125, 129, 133, 137, 141, 145, 149, 153, 157, 162, 166, 170, 174, 179, 183, 187, 192, 196, 200, 205, 209, 214, 218, 223, 227, 232, 237, 241, 246, 250, 255, 260, 264, 269, 274, 279, 283, 288, 293, 298, 303, 308, 312, 317, 322, 327, 332, 337, 342, 347, 352, 357, 362, 367, 372, 378, 383, 388, 393, 398, 403, 408, 414, 419, 424, 429, 435, 440, 445, 451, 456, 461, 467, 472, 477, 483, 488, 494, 499, 505, 510, 516, 521, 527, 532, 538, 543, 549, 554, 560, 566, 571, 577, 582, 588, 594, 599, 605, 611, 617, 622, 628, 634, 640, 645, 651, 657, 663, 669, 674, 680, 686, 692, 698, 704, 710, 716, 722, 727, 733, 739, 745, 751, 757, 763, 769, 775, 781, 788, 794, 800, 806, 812, 818, 824, 830, 836, 842, 849, 855, 861, 867, 873, 880, 886, 892, 898, 904, 911, 917, 923, 930, 936, 942, 949, 955, 961, 968, 974, 980, 987, 993, 999, 1006, 1012, 1019, 1025, 1032, 1038, 1044, 1051, 1057, 1064, 1070, 1077, 1083, 1090, 1096, 1103, 1110, 1116, 1123, 1129, 1136, 1142, 1149, 1156, 1162, 1169, 1176, 1182, 1189, 1196, 1202, 1209, 1216, 1222, 1229, 1236, 1242, 1249, 1256, 1263, 1269, 1276, 1283, 1290, 1297, 1303, 1310, 1317, 1324, 1331, 1337, 1344};
static const uint16_t eMTB_power_1_4[256] = {0, 1, 3, 5, 7, 10, 12, 15, 18, 22, 25, 29, 32, 36, 40, 44, 49, 53, 57, 62, 66, 71, 76, 81, 86, 91, 96, 101, 106, 112, 117, 122, 128, 134, 139, 145, 151, 157, 163, 169, 175, 181, 187, 194, 200, 206, 213, 219, 226, 232, 239, 246, 253, 259, 266, 273, 280, 287, 294, 301, 309, 316, 323, 330, 338, 345, 353, 360, 368, 375, 383, 391, 398, 406, 414, 422, 430, 438, 446, 454, 462, 470, 478, 486, 494, 503, 511, 519, 528, 536, 544, 553, 561, 570, 579, 587, 596, 605, 613, 622, 631, 640, 649, 658, 667, 676, 685, 694, 703, 712, 721, 730, 739, 749, 758, 767, 777, 786, 795, 805, 814, 824, 833, 843, 853, 862, 872, 882, 891, 901, 911, 921, 931, 941, 950, 960, 970, 980, 990, 1001, 1011, 1021, 1031, 1041, 1051, 1061, 1072, 1082, 1092, 1103, 1113, 1123, 1134, 1144, 1155, 1165, 1176, 1186, 1197, 1208, 1218, 1229, 1240, 1250, 1261, 1272, 1283, 1294, 1304, 1315, 1326, 1337, 1348, 1359, 1370, 1381, 1392, 1403, 1414, 1426, 1437, 1448, 1459, 1470, 1482, 1493, 1504, 1516, 1527, 1538, 1550, 1561, 1573, 1584, 1596, 1607, 1619, 1630, 1642, 1653, 1665, 1677, 1688, 1700, 1712, 1724, 1735, 1747, 1759, 1771, 1783, 1795, 1807, 1819, 1831, 1843, 1855, 1867, 1879, 1891, 1903, 1915, 1927, 1939, 1951, 1964, 1976, 1988, 2000, 2013, 2025, 2037, 2050, 2062, 2074, 2087, 2099, 2112, 2124, 2137, 2149, 2162, 2174, 2187, 2200, 2212, 2225, 2238, 2250, 2263, 2276, 2288, 2301, 2314, 2327, 2340};
static const uint16_t eMTB_power_1_5[256] = {0, 1, 3, 5, 8, 11, 15, 19, 23, 27, 32, 36, 42, 47, 52, 58, 64, 70, 76, 83, 89, 96, 103, 110, 118, 125, 133, 140, 148, 156, 164, 173, 181, 190, 198, 207, 216, 225, 234, 244, 253, 263, 272, 282, 292, 302, 312, 322, 333, 343, 354, 364, 375, 386, 397, 408, 419, 430, 442, 453, 465, 476, 488, 500, 512, 524, 536, 548, 561, 573, 586, 598, 611, 624, 637, 650, 663, 676, 689, 702, 716, 729, 743, 756, 770, 784, 798, 811, 826, 840, 854, 868, 882, 897, 911, 926, 941, 955, 970, 985, 1000, 1015, 1030, 1045, 1061, 1076, 1091, 1107, 1122, 1138, 1154, 1169, 1185, 1201, 1217, 1233, 1249, 1266, 1282, 1298, 1315, 1331, 1348, 1364, 1381, 1398, 1414, 1431, 1448, 1465, 1482, 1499, 1517, 1534, 1551, 1569, 1586, 1604, 1621, 1639, 1657, 1674, 1692, 1710, 1728, 1746, 1764, 1782, 1800, 1819, 1837, 1856, 1874, 1893, 1911, 1930, 1948, 1967, 1986, 2005, 2024, 2043, 2062, 2081, 2100, 2119, 2139, 2158, 2178, 2197, 2217, 2236, 2256, 2275, 2295, 2315, 2335, 2355, 2375, 2395, 2415, 2435, 2455, 2476, 2496, 2516, 2537, 2557, 2578, 2598, 2619, 2640, 2660, 2681, 2702, 2723, 2744, 2765, 2786, 2807, 2828, 2850, 2871, 2892, 2914, 2935, 2957, 2978, 3000, 3021, 3043, 3065, 3087, 3109, 3131, 3153, 3175, 3197, 3219, 3241, 3263, 3285, 3308, 3330, 3353, 3375, 3398, 3420, 3443, 3465, 3488, 3511, 3534, 3557, 3580, 3602, 3626, 3649, 3672, 3695, 3718, 3741, 3765, 3788, 3811, 3835, 3858, 3882, 3906, 3929, 3953, 3977, 4000, 4024, 4048, 4072};
static const uint16_t eMTB_power_1_6[256] = {0, 1, 3, 6, 9, 13, 18, 22, 28, 34, 40, 46, 53, 61, 68, 76, 84, 93, 102, 111, 121, 130, 141, 151, 162, 172, 184, 195, 207, 219, 231, 243, 256, 269, 282, 295, 309, 323, 337, 351, 366, 381, 396, 411, 426, 442, 458, 474, 490, 506, 523, 540, 557, 574, 591, 609, 627, 645, 663, 681, 700, 719, 738, 757, 776, 796, 815, 835, 855, 875, 896, 916, 937, 958, 979, 1000, 1022, 1043, 1065, 1087, 1109, 1131, 1154, 1176, 1199, 1222, 1245, 1268, 1292, 1315, 1339, 1363, 1387, 1411, 1436, 1460, 1485, 1510, 1534, 1560, 1585, 1610, 1636, 1662, 1688, 1714, 1740, 1766, 1793, 1819, 1846, 1873, 1900, 1927, 1955, 1982, 2010, 2037, 2065, 2094, 2122, 2150, 2179, 2207, 2236, 2265, 2294, 2323, 2353, 2382, 2412, 2441, 2471, 2501, 2531, 2562, 2592, 2623, 2653, 2684, 2715, 2746, 2778, 2809, 2840, 2872, 2904, 2936, 2968, 3000, 3032, 3065, 3097, 3130, 3163, 3195, 3228, 3262, 3295, 3328, 3362, 3396, 3429, 3463, 3497, 3532, 3566, 3600, 3635, 3670, 3704, 3739, 3774, 3810, 3845, 3880, 3916, 3951, 3987, 4023, 4059, 4095, 4132, 4168, 4204, 4241, 4278, 4315, 4352, 4389, 4426, 4463, 4501, 4538, 4576, 4614, 4652, 4690, 4728, 4766, 4804, 4843, 4882, 4920, 4959, 4998, 5037, 5076, 5116, 5155, 5195, 5234, 5274, 5314, 5354, 5394, 5434, 5474, 5515, 5555, 5596, 5637, 5678, 5719, 5760, 5801, 5842, 5884, 5925, 5967, 6008, 6050, 6092, 6134, 6177, 6219, 6261, 6304, 6346, 6389, 6432, 6475, 6518, 6561, 6604, 6648, 6691, 6735, 6778, 6822, 6866, 6910, 6954, 6998, 7043, 7087};
static const uint16_t eMTB_power_1_7[256] = {0, 1, 3, 6, 11, 15, 21, 27, 34, 42, 50, 59, 68, 78, 89, 100, 111, 124, 136, 149, 163, 177, 191, 207, 222, 238, 254, 271, 289, 306, 324, 343, 362, 381, 401, 422, 442, 463, 485, 507, 529, 552, 575, 598, 622, 646, 671, 696, 721, 747, 773, 800, 826, 854, 881, 909, 937, 966, 995, 1024, 1054, 1084, 1114, 1145, 1176, 1208, 1239, 1272, 1304, 1337, 1370, 1403, 1437, 1471, 1506, 1540, 1575, 1611, 1646, 1683, 1719, 1756, 1793, 1830, 1868, 1906, 1944, 1982, 2021, 2060, 2100, 2140, 2180, 2220, 2261, 2302, 2343, 2385, 2427, 2469, 2512, 2555, 2598, 2641, 2685, 2729, 2773, 2818, 2863, 2908, 2954, 3000, 3046, 3092, 3139, 3186, 3233, 3280, 3328, 3376, 3425, 3473, 3522, 3571, 3621, 3671, 3721, 3771, 3822, 3873, 3924, 3975, 4027, 4079, 4131, 4184, 4237, 4290, 4343, 4397, 4451, 4505, 4559, 4614, 4669, 4724, 4780, 4835, 4892, 4948, 5004, 5061, 5118, 5176, 5233, 5291, 5349, 5408, 5467, 5526, 5585, 5644, 5704, 5764, 5824, 5885, 5945, 6006, 6068, 6129, 6191, 6253, 6315, 6378, 6441, 6504, 6567, 6631, 6694, 6758, 6823, 6887, 6952, 7017, 7083, 7148, 7214, 7280, 7346, 7413, 7480, 7547, 7614, 7682, 7749, 7817, 7886, 7954, 8023, 8092, 8161, 8231, 8300, 8370, 8441, 8511, 8582, 8653, 8724, 8795, 8867, 8939, 9011, 9083, 9156, 9229, 9302, 9375, 9449, 9523, 9597, 9671, 9745, 9820, 9895, 9970, 10046, 10121, 10197, 10274, 10350, 10427, 10503, 10580, 10658, 10735, 10813, 10891, 10969, 11048, 11127, 11205, 11285, 11364, 11444, 11523, 11604, 11684, 11764, 11845, 11926, 12007, 12089, 12170, 12252, 12334};
static const uint16_t eMTB_power_1_8[256] = {0, 1, 3, 7, 12, 18, 25, 33, 42, 52, 63, 75, 88, 101, 116, 131, 147, 164, 182, 200, 220, 240, 261, 283, 305, 328, 352, 377, 403, 429, 456, 484, 512, 541, 571, 602, 633, 665, 698, 731, 765, 800, 835, 871, 908, 946, 984, 1023, 1062, 1102, 1143, 1185, 1227, 1270, 1313, 1357, 1402, 1447, 1493, 1540, 1587, 1635, 1684, 1733, 1783, 1833, 1884, 1936, 1988, 2041, 2095, 2149, 2204, 2259, 2315, 2372, 2429, 2487, 2545, 2605, 2664, 2724, 2785, 2847, 2909, 2971, 3035, 3098, 3163, 3228, 3293, 3359, 3426, 3494, 3561, 3630, 3699, 3769, 3839, 3910, 3981, 4053, 4126, 4199, 4272, 4347, 4421, 4497, 4573, 4649, 4726, 4804, 4882, 4961, 5040, 5120, 5200, 5281, 5363, 5445, 5527, 5611, 5694, 5779, 5864, 5949, 6035, 6121, 6208, 6296, 6384, 6473, 6562, 6652, 6742, 6833, 6924, 7016, 7109, 7202, 7295, 7389, 7484, 7579, 7675, 7771, 7867, 7965, 8063, 8161, 8260, 8359, 8459, 8559, 8660, 8762, 8864, 8966, 9069, 9173, 9277, 9382, 9487, 9593, 9699, 9806, 9913, 10021, 10129, 10238, 10347, 10457, 10567, 10678, 10789, 10901, 11013, 11126, 11240, 11354, 11468, 11583, 11698, 11814, 11931, 12048, 12165, 12283, 12402, 12521, 12640, 12760, 12881, 13002, 13123, 13245, 13368, 13491, 13614, 13738, 13863, 13988, 14113, 14239, 14366, 14493, 14620, 14748, 14877, 15006, 15135, 15265, 15396, 15527, 15658, 15790, 15923, 16056, 16189, 16323, 16457, 16592, 16728, 16864, 17000, 17137, 17274, 17412, 17550, 17689, 17828, 17968, 18108, 18249, 18390, 18532, 18674, 18817, 18960, 19104, 19248, 19392, 19537, 19683, 19829, 19976, 20123, 20270, 20418, 20566, 20715, 20865, 21015, 21165, 21316, 21467};
static const uint16_t eMTB_power_1_9[256] = {0, 1, 4, 8, 14, 21, 30, 40, 52, 65, 79, 95, 112, 131, 151, 172, 194, 218, 243, 269, 296, 325, 355, 387, 419, 453, 488, 524, 562, 601, 641, 682, 724, 768, 812, 858, 906, 954, 1004, 1054, 1106, 1160, 1214, 1269, 1326, 1384, 1443, 1503, 1564, 1627, 1691, 1755, 1821, 1889, 1957, 2026, 2097, 2169, 2241, 2315, 2390, 2467, 2544, 2623, 2702, 2783, 2865, 2948, 3032, 3118, 3204, 3291, 3380, 3470, 3561, 3653, 3746, 3840, 3935, 4032, 4129, 4228, 4328, 4428, 4530, 4633, 4737, 4843, 4949, 5056, 5165, 5274, 5385, 5497, 5610, 5724, 5839, 5955, 6072, 6190, 6310, 6430, 6551, 6674, 6798, 6922, 7048, 7175, 7303, 7432, 7562, 7693, 7826, 7959, 8093, 8229, 8365, 8503, 8641, 8781, 8922, 9063, 9206, 9350, 9495, 9641, 9788, 9936, 10086, 10236, 10387, 10539, 10693, 10847, 11003, 11159, 11317, 11475, 11635, 11796, 11958, 12120, 12284, 12449, 12615, 12782, 12950, 13119, 13289, 13460, 13632, 13806, 13980, 14155, 14331, 14509, 14687, 14867, 15047, 15228, 15411, 15594, 15779, 15965, 16151, 16339, 16527, 16717, 16908, 17100, 17292, 17486, 17681, 17877, 18074, 18271, 18470, 18670, 18871, 19073, 19276, 19480, 19685, 19891, 20098, 20306, 20515, 20725, 20936, 21148, 21362, 21576, 21791, 22007, 22224, 22442, 22661, 22882, 23103, 23325, 23548, 23772, 23998, 24224, 24451, 24679, 24909, 25139, 25370, 25602, 25835, 26070, 26305, 26541, 26778, 27017, 27256, 27496, 27738, 27980, 28223, 28467, 28712, 28959, 29206, 29454, 29703, 29954, 30205, 30457, 30710, 30964, 31220, 31476, 31733, 31991, 32250, 32510, 32771, 33034, 33297, 33561, 33826, 34092, 34359, 34627, 34896, 35166, 35437, 35709, 35982, 36256, 36531, 36807, 37084, 37362};
static const uint16_t eMTB_power_2_0[256] = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 169, 196, 225, 256, 289, 324, 361, 400, 441, 484, 529, 576, 625, 676, 729, 784, 841, 900, 961, 1024, 1089, 1156, 1225, 1296, 1369, 1444, 1521, 1600, 1681, 1764, 1849, 1936, 2025, 2116, 2209, 2304, 2401, 2500, 2601, 2704, 2809, 2916, 3025, 3136, 3249, 3364, 3481, 3600, 3721, 3844, 3969, 4096, 4225, 4356, 4489, 4624, 4761, 4900, 5041, 5184, 5329, 5476, 5625, 5776, 5929, 6084, 6241, 6400, 6561, 6724, 6889, 7056, 7225, 7396, 7569, 7744, 7921, 8100, 8281, 8464, 8649, 8836, 9025, 9216, 9409, 9604, 9801, 10000, 10201, 10404, 10609, 10816, 11025, 11236, 11449, 11664, 11881, 12100, 12321, 12544, 12769, 12996, 13225, 13456, 13689, 13924, 14161, 14400, 14641, 14884, 15129, 15376, 15625, 15876, 16129, 16384, 16641, 16900, 17161, 17424, 17689, 17956, 18225, 18496, 18769, 19044, 19321, 19600, 19881, 20164, 20449, 20736, 21025, 21316, 21609, 21904, 22201, 22500, 22801, 23104, 23409, 23716, 24025, 24336, 24649, 24964, 25281, 25600, 25921, 26244, 26569, 26896, 27225, 27556, 27889, 28224, 28561, 28900, 29241, 29584, 29929, 30276, 30625, 30976, 31329, 31684, 32041, 32400, 32761, 33124, 33489, 33856, 34225, 34596, 34969, 35344, 35721, 36100, 36481, 36864, 37249, 37636, 38025, 38416, 38809, 39204, 39601, 40000, 40401, 40804, 41209, 41616, 42025, 42436, 42849, 43264, 43681, 44100, 44521, 44944, 45369, 45796, 46225, 46656, 47089, 47524, 47961, 48400, 48841, 49284, 49729, 50176, 50625, 51076, 51529, 51984, 52441, 52900, 53361, 53824, 54289, 54756, 55225, 55696, 56169, 56644, 57121, 57600, 58081, 58564, 59049, 59536, 60025, 60516, 61009, 61504, 62001, 62500, 63001, 63504, 64009, 64516, 65025};


// cruise variables
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
#define UART_NUMBER_DATA_BYTES_TO_RECEIVE   7   // change this value depending on how many data bytes there is to receive ( Package = one start byte + data bytes + two bytes 16 bit CRC )
#define UART_NUMBER_DATA_BYTES_TO_SEND      26  // change this value depending on how many data bytes there is to send ( Package = one start byte + data bytes + two bytes 16 bit CRC )

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

static void get_battery_voltage_filtered(void);
static void get_battery_current_filtered(void);
static void get_pedal_torque(void);
static void calc_wheel_speed(void);
static void calc_cadence(void);


static void apply_power_assist();
static void apply_torque_assist();
static void apply_cadence_assist();
static void apply_emtb_assist();
static void apply_virtual_throttle();
static void apply_walk_assist();
static void apply_cruise();
static void apply_cadence_sensor_calibration();
static void apply_throttle();
static void apply_speed_limit();
static void apply_temperature_limiting();


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
  ebike_control_motor();            // use received data and sensor input to control motor 
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

  // force target current to 0 if brakes are enabled or if there are errors
  if (ui8_brakes_enabled || ui8_system_state != NO_ERROR) { ui8_adc_battery_current_target = 0; }

  // check if to enable the motor
  if (!ui8_motor_enabled &&
      ui16_motor_get_motor_speed_erps() == 0 && // only enable motor if stopped, other way something bad can happen due to high currents/regen or similar
      ui8_adc_battery_current_target)
  {
    ui8_motor_enabled = 1;
    ui8_g_duty_cycle = 0;
    motor_enable_pwm();
  }

  // check if to disable the motor
  if (ui8_motor_enabled &&
      ui16_motor_get_motor_speed_erps() == 0 &&
      !ui8_adc_battery_current_target &&
      !ui8_g_duty_cycle)
  {
    ui8_motor_enabled = 0;
    motor_disable_pwm();
  }

  // set control parameters
  if (ui8_motor_enabled && !ui8_brakes_enabled)
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
  else
  {
    // reset motor control variables (safety)
    ui16_controller_duty_cycle_ramp_up_inverse_step = PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT;
    ui16_controller_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
    ui8_controller_adc_battery_current_target = 0;
    ui8_controller_duty_cycle_target = 0;
    ui8_g_duty_cycle = 0;
  }
}



static void apply_power_assist()
{
  uint8_t ui8_power_assist_multiplier_x10 = ui8_riding_mode_parameter;
  
  // calculate power assist
  uint32_t ui32_power_assist_x100 = (uint32_t) ui16_pedal_power_x10 * ui8_power_assist_multiplier_x10;
  
  // calculate target current
  uint16_t ui16_battery_current_target_x10 = (ui32_power_assist_x100 * 100) / ui16_battery_voltage_filtered_x1000;
  
  // set battery current target in ADC steps
  uint16_t ui16_adc_battery_current_target = ui16_battery_current_target_x10 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X10;
  
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
  #define ADC_PEDAL_TORQUE_THRESHOLD            6     // minimum ADC torque to enable torque assist
  #define TORQUE_ASSIST_FACTOR_DENOMINATOR      110   // scale the torque assist target current
  
  if ((ui16_adc_pedal_torque_delta > ADC_PEDAL_TORQUE_THRESHOLD) && ui8_pedal_cadence_RPM)
  {
    // get the torque assist factor
    uint8_t ui8_torque_assist_factor = ui8_riding_mode_parameter;
    
    // calculate torque assist target current
    uint16_t ui16_adc_battery_current_target_torque_assist = ((uint16_t) (ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD) * ui8_torque_assist_factor) / TORQUE_ASSIST_FACTOR_DENOMINATOR;
  
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
  #define ADC_PEDAL_TORQUE_THRESHOLD                  6     // minimum ADC torque to enable eMTB assist
  #define eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR      20    // scale the eMTB assist target current
  
  if ((ui16_adc_pedal_torque_delta > ADC_PEDAL_TORQUE_THRESHOLD) && ui8_pedal_cadence_RPM)
  {
    // initialize eMTB assist target current
    uint16_t ui16_adc_battery_current_target_eMTB_assist = 0;
    
    // get the eMTB assist sensitivity
    uint8_t ui8_eMTB_assist_sensitivity = ui8_riding_mode_parameter;
    
    switch (ui8_eMTB_assist_sensitivity)
    {
      case 1:
      
        // ADC torque value to the power of 1.3
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_3[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
      
      case 2:
      
        // ADC torque value to the power of 1.4
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_4[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
      
      case 3:
      
        // ADC torque value to the power of 1.5
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_5[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
      
      case 4:
      
        // ADC torque value to the power of 1.6
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_6[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
      
      case 5:
      
        // ADC torque value to the power of 1.7
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_7[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
      
      case 6:
      
        // ADC torque value to the power of 1.8
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_8[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;

      case 7:
      
        // ADC torque value to the power of 1.9
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_1_9[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
      
      case 8:
      
        // ADC torque value to the power of 2
        ui16_adc_battery_current_target_eMTB_assist = (uint16_t) eMTB_power_2_0[ui16_adc_pedal_torque_delta - ADC_PEDAL_TORQUE_THRESHOLD] / eMTB_ASSIST_TARGET_CURRENT_DENOMINATOR;
        
      break;
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
    if (ui16_adc_battery_current_target_eMTB_assist > ui8_adc_battery_current_max) { ui8_adc_battery_current_target = ui8_adc_battery_current_max; }
    else { ui8_adc_battery_current_target = ui16_adc_battery_current_target_eMTB_assist; }

    // set duty cycle target
    if (ui8_adc_battery_current_target) { ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX; }
    else { ui8_duty_cycle_target = 0; }
  }
}



static void apply_walk_assist()
{
  #define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP     200
  #define WALK_ASSIST_DUTY_CYCLE_MAX                      80
  
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
    ui8_adc_battery_current_target = ui8_adc_battery_current_max;
    
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
    ui16_cadence_sensor_pulse_high_percentage_x10 = filter(ui16_cadence_sensor_pulse_high_percentage_x10_temp, ui16_cadence_sensor_pulse_high_percentage_x10, 95);
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
  #define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP    80
  
  if ((ui8_riding_mode != WALK_ASSIST_MODE) && (ui8_riding_mode != CRUISE_MODE))
  {
    // map value from 0 up to duty cycle max
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
      ui16_duty_cycle_ramp_up_inverse_step = THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP;
      ui16_duty_cycle_ramp_down_inverse_step = PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT;
      
      // set battery current target
      ui8_adc_battery_current_target = ui8_adc_battery_current_target_throttle;
      
      // set duty cycle target
      ui8_duty_cycle_target = PWM_DUTY_CYCLE_MAX;
    }
  }
}



static void apply_temperature_limiting()
{
  static uint16_t ui16_adc_motor_temperature_filtered;
  
  // calculate motor temperature
  volatile uint16_t ui16_temp = UI16_ADC_10_BIT_THROTTLE;
  
  // filter motor temperature value
  ui16_adc_motor_temperature_filtered = filter(ui16_temp, ui16_adc_motor_temperature_filtered, 80);
  
  m_configuration_variables.ui16_motor_temperature_x2 = (float) ui16_adc_motor_temperature_filtered / 1.024;
  m_configuration_variables.ui8_motor_temperature = m_configuration_variables.ui16_motor_temperature_x2 >> 1;
  
  // min temperature value can't be equal or higher than max temperature value...
  if (m_configuration_variables.ui8_motor_temperature_min_value_to_limit >= m_configuration_variables.ui8_motor_temperature_max_value_to_limit)
  {
    ui8_adc_battery_current_target = 0;
    m_configuration_variables.ui8_temperature_current_limiting_value = 0;
  }
  else
  {
    // reduce motor current if over temperature
    ui8_adc_battery_current_target = (map((uint32_t) m_configuration_variables.ui16_motor_temperature_x2,
                                          (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                                          (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                                          (uint32_t) ui8_adc_battery_current_target,
                                          (uint32_t) 0));
                                           
    // get a value linear to the current limitation, just to show to user
    m_configuration_variables.ui8_temperature_current_limiting_value = (map((uint32_t) m_configuration_variables.ui16_motor_temperature_x2,
                                                                            (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_min_value_to_limit) << 1),
                                                                            (uint32_t) (((uint16_t) m_configuration_variables.ui8_motor_temperature_max_value_to_limit) << 1),
                                                                            (uint32_t) 255,
                                                                            (uint32_t) 0));
  }
}



static void apply_speed_limit()
{
  if (m_configuration_variables.ui8_wheel_speed_max > 0)
  {
    // set battery current target 
    ui8_adc_battery_current_target = (uint8_t) (map((uint32_t) ui16_wheel_speed_x10,
                                                    (uint32_t) ((m_configuration_variables.ui8_wheel_speed_max * 10) - 20),
                                                    (uint32_t) ((m_configuration_variables.ui8_wheel_speed_max * 10) + 20),
                                                    (uint32_t) ui8_adc_battery_current_target,
                                                    (uint32_t) 0));
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
  // get the cadence sensor ticks
  uint16_t ui16_cadence_sensor_ticks_temp = ui16_cadence_sensor_ticks;
  
  // get the cadence sensor pulse state
  uint8_t ui8_cadence_sensor_pulse_state_temp = ui8_cadence_sensor_pulse_state;
  
  // select cadence sensor mode
  switch (ui8_cadence_sensor_mode)
  {
    case STANDARD_MODE:
    
      #define CADENCE_SENSOR_STANDARD_MODE_TICKS_COUNTER_MIN_AT_SPEED    2000
      
      // adjust cadence sensor ticks counter min depending on wheel speed
      ui16_cadence_sensor_ticks_counter_min_speed_adjusted = map((uint32_t) ui16_wheel_speed_x10,
                                                                 (uint32_t) 40,
                                                                 (uint32_t) 200,
                                                                 (uint32_t) CADENCE_SENSOR_STANDARD_MODE_TICKS_COUNTER_MIN,
                                                                 (uint32_t) CADENCE_SENSOR_STANDARD_MODE_TICKS_COUNTER_MIN_AT_SPEED);
                                                                 
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
    
      #define CADENCE_SENSOR_ADVANCED_MODE_TICKS_COUNTER_MIN_AT_SPEED    1000
      
      // adjust cadence sensor ticks counter min depending on wheel speed
      ui16_cadence_sensor_ticks_counter_min_speed_adjusted = map((uint32_t) ui16_wheel_speed_x10,
                                                                 (uint32_t) 40,
                                                                 (uint32_t) 200,
                                                                 (uint32_t) CADENCE_SENSOR_ADVANCED_MODE_TICKS_COUNTER_MIN,
                                                                 (uint32_t) CADENCE_SENSOR_ADVANCED_MODE_TICKS_COUNTER_MIN_AT_SPEED);
                                                                 
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
  ui8_battery_current_filtered_x10 = ui8_adc_battery_current_filtered * BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X10;
}



static void get_pedal_torque(void)
{
  #define ADC_TORQUE_MEASUREMENT_THRESHOLD      90 // 50 pedal cadence RPM
  
  // get the adc torque sensor value depending on cadence RPM
  if (ui8_pedal_cadence_RPM > ADC_TORQUE_MEASUREMENT_THRESHOLD)
  {
    // get the max adc pedal torque from the motor PWM control loop
    ui16_adc_pedal_torque = ui16_adc_pedal_torque_max;
    
    // approximate the max adc value to a sinewave and calculate average torque
    ui16_adc_pedal_torque = ((uint32_t) ui16_adc_pedal_torque * 637) / 1000;
  }
  else
  {
    // get the adc pedal torque
    ui16_adc_pedal_torque = UI16_ADC_10_BIT_TORQUE_SENSOR;
  }

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

  // calculate crank power
  ui16_pedal_power_x10 = ((uint32_t) ui16_pedal_torque_x100 * ui8_pedal_cadence_RPM) / 105; // see note below

  /*---------------------------------------------------------

    NOTE: regarding the human power calculation
    
    Formula: power  =  force  *  rotations per second  *  2  *  pi
    Formula: power  =  force  *  rotations per minute  *  2  *  pi / 60
    
    (100 * 2 * pi) / 60 ≈ 1.047 -> 105
  ---------------------------------------------------------*/
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
  if (((ui16_adc_pedal_torque_offset > 300) || (ui16_adc_pedal_torque_offset < 5)) &&
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
      uint8_t ui8_lights = ui8_rx_buffer [4];
      
      // set lights
      lights_set_state(ui8_lights);

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
          
          // motor temperature limit function or throttle
          m_configuration_variables.ui8_optional_ADC_function = ui8_rx_buffer [7];

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
          
          // motor acceleration adjustment
          uint8_t ui8_motor_acceleration_adjustment = ui8_rx_buffer [7];
          
          // set duty cycle ramp up inverse step
          ui16_duty_cycle_ramp_up_inverse_step_default = map((uint32_t) ui8_motor_acceleration_adjustment,
                                                             (uint32_t) 0,
                                                             (uint32_t) 100,
                                                             (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT,
                                                             (uint32_t) PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN);
                                                             
        break;

        case 5:
        
          // pedal torque conversion
          m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_rx_buffer [5];
          
          // max battery current
          m_configuration_variables.ui8_battery_max_current = ui8_rx_buffer [6];
          
          // battery power limit
          m_configuration_variables.ui8_target_battery_max_power_div25 = ui8_rx_buffer [7];
          
          // calculate max battery current in ADC steps from the received battery current limit
          uint8_t ui8_adc_battery_current_max_temp_1 = ((m_configuration_variables.ui8_battery_max_current * 10) / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X10);
          
          // calculate max battery current in ADC steps from the received power limit
          uint32_t ui32_battery_current_max_x10 = ((uint32_t) m_configuration_variables.ui8_target_battery_max_power_div25 * 250000) / ui16_battery_voltage_filtered_x1000;
          uint8_t ui8_adc_battery_current_max_temp_2 = ui32_battery_current_max_x10 / BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X10;
          
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

        break;

        default:
          // nothing, should display error code
        break;
      }

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
      ui8_tx_buffer[8] = m_configuration_variables.ui8_temperature_current_limiting_value;
    
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
  ui16_temp = ui16_adc_pedal_torque_offset; //ui16_motor_get_motor_speed_erps();                          // CHANGE CHANGE CHANGE
  ui8_tx_buffer[13] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[14] = (uint8_t) (ui16_temp >> 8);
  
  // FOC angle
  ui8_tx_buffer[15] = ui8_g_foc_angle;
  
  // system state
  ui8_tx_buffer[16] = ui8_system_state;
  
  // motor temperature
  ui8_tx_buffer[17] = m_configuration_variables.ui8_motor_temperature;
  
  // wheel_speed_sensor_tick_counter
  ui8_tx_buffer[18] = (uint8_t) (ui32_wheel_speed_sensor_ticks_total & 0xff);
  ui8_tx_buffer[19] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 8) & 0xff);
  ui8_tx_buffer[20] = (uint8_t) ((ui32_wheel_speed_sensor_ticks_total >> 16) & 0xff);

  // pedal torque x100
  ui16_temp = ui16_pedal_torque_x100;
  ui8_tx_buffer[21] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[22] = (uint8_t) (ui16_temp >> 8);

  // pedal power x10
  ui8_tx_buffer[23] = (uint8_t) (ui16_pedal_power_x10 & 0xff);
  ui8_tx_buffer[24] = (uint8_t) (ui16_pedal_power_x10 >> 8);
  
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
} */