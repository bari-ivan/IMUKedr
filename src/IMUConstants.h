/*
 * Файл - хранилище констант для работы 
 * инерциально-измерительного модуля
 * 
 * Версия: 1.0.1.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

 
#ifndef __KDR_IMU_IMUCONSTANTS_ARDUINO
#define __KDR_IMU_IMUCONSTANTS_ARDUINO

// Фундаментальные константы
#define KDR_CONST_PI                                  3.1415926535

#define KDR_CONST_G                                   9.80665

#define KDR_CONST_AIR_A                               0.003665

// Преобразование величин
#define KDR_CONST_DEG_TO_RAD                          (KDR_CONST_PI / 180)
#define KDR_CONST_RAD_TO_DEG                          (180 / KDR_CONST_PI)
#define KDR_CONST_CELSIUS_TO_KELVIN                   273.15
#define KDR_CONST_CELSIUS_TO_FARENGHEIT_MULT          1.8
#define KDR_CONST_CELSIUS_TO_FARENGHEIT_INC           32
#define KDR_CONST_MILLIBARS_TO_PASCALS                100
#define KDR_CONST_MILLIBARS_TO_MMHG                   0.75



#endif
