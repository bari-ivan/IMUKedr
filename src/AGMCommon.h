/*
 * Родительские классы для реализации модулей работы с
 * инерциальными сенсорами STMicroelectronics. 
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */


#ifndef __KDR_IMU_AGMCOMMON_ARDUINO
#define __KDR_IMU_AGMCOMMON_ARDUINO

#include "IMUCommon.h"

/*
 * Регистры инерциальных датчиков
 */

// Группа контрольных регистров для управленя функциями устройств
#define CTRL_REG5       0x24

// Регистры получения данных по осям. Указаны первые регистры пары
#define OUT_X           0x28
#define OUT_Y           0x2A
#define OUT_Z           0x2C

class AGMCommon : public IMUCommon {
  
  public:
    // Конструктор класса
    AGMCommon(uint8_t CS);
    // Деструктор класса
    ~AGMCommon();

    // Запись в пятый контрольный регистр
    void writeCTRL5();
    // Чтение данных по оси X
    int16_t readX();
    // Чтение данных по оси Y
    int16_t readY();
    // Чтение данных по оси Z
    int16_t readZ();
    // Чтение данных по осям
    void readXYZ(int16_t *x, int16_t *y, int16_t *z);

  protected:
    // Значение пятого контрольного регистра
    uint8_t _valCTL5;
};

#endif
