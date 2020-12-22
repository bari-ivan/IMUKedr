/*
 * Родительский класс для реализации модулей работы с
 * барометрами STMicroelectronics. 
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */


#ifndef __KDR_IMU_BCOMMON_ARDUINO
#define __KDR_IMU_BCOMMON_ARDUINO

#include "IMUCommon.h"

/*
 * Регистры барометров
 */

// Регистры получения данных. Указаны первые регистры групп
#define OUT_PRESSURE        0x28
#define OUT_TEMPERATURE     0x2B


class BCommon : public IMUCommon {
  
  public:
    // Конструктор класса
    BCommon(uint8_t CS);
    // Деструктор класса
    ~BCommon();

    // Чтение данных из регистров давления
    uint32_t readPressureRaw();
    // Чтение данных из регистров температуры
    int16_t readTemperatureRaw();
    // Чтение данных из регистров давления и температуры
    void readPT(int16_t *p, int16_t *t);

  protected:
    float _multiplicatorT;
};

#endif
