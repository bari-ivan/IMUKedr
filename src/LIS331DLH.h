/*
 * Модуль управления акселерометром LIS331DLH 
 * с использованием протокола SPI ARDUINO.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_LIS331DLH_ARDUINO
#define __KDR_IMU_LIS331DLH_ARDUINO

#include "AGMCommon.h"

/*
 * Основные константы, относящиеся к микросхеме
 */

// Содержимое по умолчанию регистра "КТО Я" микросхемы
#define LIS331DLH_WHO_AM_I_REPLY 0x32

// Константы осей
#define KDR_LIS331DLH_AXIS_NONE 0b00000000
#define KDR_LIS331DLH_AXIS_X    0b00000100
#define KDR_LIS331DLH_AXIS_Y    0b00000010
#define KDR_LIS331DLH_AXIS_Z    0b00000001
#define KDR_LIS331DLH_AXIS_XY   0b00000110
#define KDR_LIS331DLH_AXIS_XZ   0b00000101
#define KDR_LIS331DLH_AXIS_YZ   0b00000011
#define KDR_LIS331DLH_AXIS_XYZ  0b00000111

/*
 * Константы, определяющие чувствительность датчика
 */

// Константы для преобразования значения и задания режима в библиотеке
#define KDR_FS_2G 2
#define KDR_FS_4G 4
#define KDR_FS_8G 8

// Значения четвёртого контрольного регистра для перевода микросхемы в нужный режим
#define CTRL4_REG_2G 0x00
#define CTRL4_REG_4G 0x10
#define CTRL4_REG_8G 0x30

/*
 * Константы, определяющие частоту обновления выходных данных
 */

#define KDR_DR_SLEEP    0
#define KDR_DR_HALF_Hz  -1
#define KDR_DR_1_Hz     1
#define KDR_DR_2_Hz     2
#define KDR_DR_5_Hz     5
#define KDR_DR_10_Hz    10
#define KDR_DR_NORMAL   50
#define KDR_DR_100_Hz   100
#define KDR_DR_400_Hz   400
#define KDR_DR_1000_Hz  1000


class LIS331DLH : public AGMCommon {
  
  public:
    // Конструктор класса
    LIS331DLH();
    // Конструктор класса
    LIS331DLH(uint8_t CS);
    // Деструктор класса
    ~LIS331DLH();
    // Проверка правильности подключения акселерометра и настроек SPI
    uint8_t testConnection();
    // Установка активных осей
    void setAxis(uint8_t aM);
    // Установка чувствительности
    void setFullScale(uint8_t fS);
    // Установка режима работы микросхемы в зависимости от частоты выходных данных
    void setMode(int16_t dataRate);
    // Получение ускорения по оси X в числах G
    float readGX();
    // Получение ускорения по оси Y в числах G
    float readGY();
    // Получение ускорения по оси Z в числах G
    float readGZ();
    // Получение ускорения по оси X в м/с^2
    float readAX();
    // Получение ускорения по оси Y в м/с^2
    float readAY();
    // Получение ускорения по оси Z в м/с^2
    float readAZ();
    // Получение ускорения по всем трём осям в числах G
    void readGXYZ(float *ax, float *ay, float *az);
    // Получение ускорения по всем трём осям в м/с^2
    void readAXYZ(float *gx, float *gy, float *gz);

};

#endif
