/*
 * Модуль управления барометром LPS25HB 
 * с использованием протокола SPI ARDUINO.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_LPS25HB_ARDUINO
#define __KDR_IMU_LPS25HB_ARDUINO

#include "BCommon.h"

/*
 * Основные константы, относящиеся к микросхеме
 */

// Содержимое по умолчанию регистра "КТО Я" микросхемы
#define LPS25HB_WHO_AM_I_REPLY 0xBD

/*
 * Константы, определяющие частоту обновления выходных данных
 */

#define KDR_B_DR_SLEEP        0
#define KDR_B_DR_ONE_SHOT     -1
#define KDR_B_DR_1_Hz         1
#define KDR_B_DR_NORMAL       7
#define KDR_B_DR_12_5_Hz      12
#define KDR_B_DR_25_Hz        25

// Значения коэффициента преобразования данных для давления и температуры
#define KDR_B_MULT_PRESSURE       4096
#define KDR_B_MULT_TEMPERATURE    480
#define KDR_B_RAW_TO_CELSIUS      42.5


class LPS25HB : public BCommon {
  
  public:
    // Конструктор класса
    LPS25HB();
    // Конструктор класса
    LPS25HB(uint8_t CS);
    // Деструктор класса
    ~LPS25HB();
    // Проверка правильности подключения акселерометра и настроек SPI
    uint8_t testConnection();
    // Установка частоты выходных данных
    void setMode(int16_t dataRate);
    // Получение давления в Паскалях
    float readPressurePascals();
    // Получение давления в миллибарах
    float readPressureMillibars();
    // Получение давления в миллиметрах ртутного столба
    float readPressureMillimetersHg();
    // Получение температуры в градусах по Цельсию
    float readTemperatureC();
    // Получение температуры в градусах по Кельвину
    float readTemperatureK();
    // Получение температуры в градусах по Фаренгейту
    float readTemperatureF();
};

#endif
