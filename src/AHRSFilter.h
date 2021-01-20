/*
 * Родительский класс для реализации работы
 * модуля в режиме курсовертикали.
 * 
 * Версия: 1.0.1.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_AHRSFILTER_ARDUINO
#define __KDR_IMU_AHRSFILTER_ARDUINO

#include "IMUOrientation.h"


class AHRSFilter : public IMUOrientation
{

  public:
    // Сброс данных и инициализация фильтра
    virtual void init() = 0;
    // Обновление данных фильтра
    virtual void update(float ax, float ay, float az, float gx, float gy, float gz, 
                float mx, float my, float mz) = 0;
    // Получение угла тангажа в радианах
    virtual float getPitchRad() = 0;
    // Получение угла крена в радианах
    virtual float getRollRad() = 0;
    // Получение угла рыскания в радианах
    virtual float getYawRad() = 0;
    // Получение угла тангажа в градусах
    virtual float getPitchDeg() = 0;
    // Получение угла крена в градусах
    virtual float getRollDeg() = 0;
    // Получение угла рыскания в градусах
    virtual float getYawDeg() = 0;
    // Получение положения в виде кватерниона
    virtual void getQuaternion(float * qW, float * qX, float * qY, float * qZ) = 0;

  protected:

    // Включение коррекции по магнитрометру
    uint8_t _magnetrometerCorrect;
};


#endif
