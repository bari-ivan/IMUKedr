/*
 * Родительский класс для реализации работы
 * модуля в режиме курсовертикали.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_AHRSFILTER_ARDUINO
#define __KDR_IMU_AHRSFILTER_ARDUINO

#include "IMUOrientation.h"


class AHRSFilter : public IMUOrientation
{

  public:
    // Конструктор класса
    AHRSFilter();
    // Деструктор класса
    ~AHRSFilter();
    // Сброс данных и инициализация фильтра
    virtual void init();
    // Обновление данных фильтра
    virtual void update(float ax, float ay, float az, float gx, float gy, float gz, 
                float mx, float my, float mz);
    // Получение угла тангажа в радианах
    virtual float getPitchRad();
    // Получение угла крена в радианах
    virtual float getRollRad();
    // Получение угла рыскания в радианах
    virtual float getYawRad();
    // Получение угла тангажа в градусах
    virtual float getPitchDeg();
    // Получение угла крена в градусах
    virtual float getRollDeg();
    // Получение угла рыскания в градусах
    virtual float getYawDeg();
    // Получение положения в виде кватерниона
    virtual void getQuaternion(float * qW, float * qX, float * qY, float * qZ);

  protected:

    // Включение коррекции по магнитрометру
    uint8_t _magnetrometerCorrect;
};


#endif
