
/*
 * Родительский класс для реализации модулей работы с
 * MEMS-сенсорами STMicroelectronics. 
 * 
 * Поддерживаемые платы: КДР-ИНС-Л5В.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */


#ifndef __KDR_IMU_IMUCOMMON_ARDUINO
#define __KDR_IMU_IMUCOMMON_ARDUINO


#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

#include "IMUConstants.h"

/*
 * Настройки SPI
 */

// Максимальная скорость обмена для используемых микросхем
#define KDR_SPI_MAXSPEED 10000000

// Порядок отправки-чтения битов
#define KDR_SPI_FIRSTBIT MSBFIRST

// Режим работы SPI
#define KDR_SPI_MODE SPI_MODE3

/*
 * Параметры чтения/записи по SPI для используемых микросхем
 */

// Настройка для чтения 1 байта
#define IMU_SIMPLE_READ 0b10000000
// Настройка для чтения группы байт
#define IMU_MSB_READ 0b11000000
// Настройка для записи 1 байта
#define IMU_SIMPLE_WRITE 0b00000000

/*
 * Общие регистры
 */

// Регистр "КТО Я" для проверки корректности подключения к устройству
#define WHO_AM_I       0x0F

// Группа контрольных регистров для управленя функциями устройств
#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define CTRL_REG4       0x23

class IMUCommon {
  
  public:
    // Конструктор класса
    IMUCommon(uint8_t CS);
    // Деструктор класса
    ~IMUCommon();

    // Чтение регистра
    uint8_t readRegister(uint8_t addr);
    // Чтение группы последовательных регистров
    void readMSBRegister(uint8_t addr, uint8_t count, uint8_t * out);
    // Запись в регистр
    void writeRegister(uint8_t addr, uint8_t value);

    // Чтение регистра "КТО Я"
    uint8_t whoAmI();
    // Запись в первый контрольный регистр
    void writeCTRL1();
    // Запись в второй контрольный регистр
    void writeCTRL2();
    // Запись в третий контрольный регистр
    void writeCTRL3();
    // Запись в четвёртый контрольный регистр
    void writeCTRL4();

  protected:
    // Значение первого контрольного регистра
    uint8_t _valCTL1;
    // Значение второго контрольного регистра
    uint8_t _valCTL2;
    // Значение третьего контрольного регистра
    uint8_t _valCTL3;
    // Значение четвёртого контрольного регистра
    uint8_t _valCTL4;
    // Коеффициент для преобразования данных, полученных с датчика
    float _multiplicator;

  private:
    // Порт выбора микросхемы
    uint8_t chipSelectPin;
};



#endif
