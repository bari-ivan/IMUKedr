/*
 * Родительский класс для реализации пространственной
 * ориентации.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_IMUORIENTATION_ARDUINO
#define __KDR_IMU_IMUORIENTATION_ARDUINO

#include "IMUCommon.h"

// Точка переполнения функций millis и micros аппаратной платформы Arduino
#define KDR_ULONG_MAXVALUE 4294967295

class IMUOrientation
{
  public:
    // Конструктор класса
    IMUOrientation();
    // Деструктор класса
    ~IMUOrientation();

  protected:
    // Вычисление периода с момента предыдущего измерения в секундах
    float getPeriod();

    // Отсчёт времени предыдущего вычисления
    uint32_t _previousCalculateMicros;
};


#endif
