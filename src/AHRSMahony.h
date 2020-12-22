/*
 * Класс для реализации фильтра Махони.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_AHRSMAHONY_ARDUINO
#define __KDR_IMU_AHRSMAHONY_ARDUINO

#include "AHRSFilter.h"

// Коэффициент обратной связи фильтра Махони
#define KDR_COEFF_Kp 2.0f * 5.0f
// Коэффициент накопления ошибки интегрирования
#define KDR_COEFF_Ki 0.0f

class AHRSMahony : public AHRSFilter
{

  public:
    // Конструктор класса
    AHRSMahony(uint8_t magnetrometerCorrect = 0);
    // Деструктор класса
    ~AHRSMahony();
    // Сброс данных и инициализация фильтра
    void init();
    // Обновление данных фильтра
    void update(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz);
    // Получение угла тангажа в радианах
    float getPitchRad();
    // Получение угла крена в радианах
    float getRollRad();
    // Получение угла рыскания в радианах
    float getYawRad();
    // Получение угла тангажа в градусах
    float getPitchDeg();
    // Получение угла крена в градусах
    float getRollDeg();
    // Получение угла рыскания в градусах
    float getYawDeg();
    // Получение положения в виде кватерниона
    void getQuaternion(float * qW, float * qX, float * qY, float * qZ);

  private:
    // Кватернион 0
    volatile float _q0 = 1.0f;
    // Кватернион 1
    volatile float _q1 = 0.0f;
    // Кватернион 2
    volatile float _q2 = 0.0f;
    // Кватернион 3
    volatile float _q3 = 0.0f;
    // Ошибка интегрирования по 3 осям
    float  eInt[3];
};


#endif
