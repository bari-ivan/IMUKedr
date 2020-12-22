/*
 * Модуль управления гироскопом L3G4200D 
 * с использованием протокола SPI ARDUINO.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_L3G4200D_ARDUINO
#define __KDR_IMU_L3G4200D_ARDUINO

#include "AGMCommon.h"

/*
 * Основные константы, относящиеся к микросхеме
 */

// Содержимое по умолчанию регистра "КТО Я" микросхемы
#define L3G4200D_WHO_AM_I_REPLY 0xD3


// Константы осей
#define KDR_L3G4200D_AXIS_NONE 0b00000000
#define KDR_L3G4200D_AXIS_X    0b00000100
#define KDR_L3G4200D_AXIS_Y    0b00000010
#define KDR_L3G4200D_AXIS_Z    0b00000001
#define KDR_L3G4200D_AXIS_XY   0b00000110
#define KDR_L3G4200D_AXIS_XZ   0b00000101
#define KDR_L3G4200D_AXIS_YZ   0b00000011
#define KDR_L3G4200D_AXIS_XYZ  0b00000111

/*
 * Константы, определяющие чувствительность датчика
 */

// Константы для преобразования значения и задания режима в библиотеке
#define KDR_FS_250DPS           250
#define KDR_FS_500DPS           500
#define KDR_FS_2000DPS          2000

// Значения четвёртого контрольного регистра для перевода микросхемы в нужный режим
#define CTRL4_REG_250DPS        0x00
#define CTRL4_REG_500DPS        0x10
#define CTRL4_REG_2000DPS       0x20

// Значения коэффициента преобразования данных для каждого из режимов
#define KDR_G_MULT_250DPS       0.00875
#define KDR_G_MULT_500DPS       0.0175
#define KDR_G_MULT_2000DPS      0.07

/*
 * Константы, определяющие режим работы микросхемы
 */

// Формат константы: KDR_G_DR_%Частота обновления выходных данных%_CO_%Частота среза%

#define KDR_G_DR_SLEEP                0

#define KDR_G_DR_NORMAL               1
#define KDR_G_DR_100Hz_CO_25Hz        2

#define KDR_G_DR_200Hz_CO_12_5Hz      3
#define KDR_G_DR_200Hz_CO_25Hz        4
#define KDR_G_DR_200Hz_CO_50Hz        5
#define KDR_G_DR_200Hz_CO_70Hz        6

#define KDR_G_DR_400Hz_CO_20Hz        7
#define KDR_G_DR_400Hz_CO_25Hz        8
#define KDR_G_DR_400Hz_CO_50Hz        9
#define KDR_G_DR_400Hz_CO_110Hz       10

#define KDR_G_DR_800Hz_CO_30Hz        11
#define KDR_G_DR_800Hz_CO_35Hz        12
#define KDR_G_DR_800Hz_CO_50Hz        13
#define KDR_G_DR_800Hz_CO_110Hz       14

class L3G4200D : public AGMCommon {
  
  public:
    // Конструктор класса
    L3G4200D();
    // Конструктор класса
    L3G4200D(uint8_t CS);
    // Деструктор класса
    ~L3G4200D();
    // Проверка правильности подключения гироскопа и настроек SPI
    uint8_t testConnection();
    // Установка активных осей
    void setAxis(uint8_t aM);
    // Установка чувствительности
    void setFullScale(uint16_t fS);
    // Установка режима работы микросхемы в зависимости от частоты выходных данных и частоты среза фильтра нижних частот
    void setMode(int16_t dataRateCutOff);
    // Получение угловой скорости по оси X в градусах в секунду
    float readDegX();
    // Получение угловой скорости по оси Y в градусах в секунду
    float readDegY();
    // Получение угловой скорости по оси Z в градусах в секунду
    float readDegZ();
    // Получение угловой скорости по оси X в радианах в секунду
    float readRadX();
    // Получение угловой скорости по оси Y в радианах в секунду
    float readRadY();
    // Получение угловой скорости по оси Z в радианах в секунду
    float readRadZ();
    // Получение угловой скорости по всем трём осям в градусах в секунду
    void readDegXYZ(float *gx, float *gy, float *gz);
    // Получение угловой скорости по всем трём осям в радианах в секунду
    void readRadXYZ(float *gx, float *gy, float *gz);

};


#endif
