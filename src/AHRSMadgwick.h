/*
 * Класс для реализации фильтра Маджвика.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_AHRSMADGWICK_ARDUINO
#define __KDR_IMU_AHRSMADGWICK_ARDUINO

#include "AHRSFilter.h"

// Эмпирически вычисленная ошибка измерений гироскопа
#define KDR_F_GYRO_MEAS_ERROR       KDR_CONST_PI * (40.0f / 180.0f)
// Эмпирически вычисленный дрифт нуля гироскопа
#define KDR_F_GYRO_MEAS_DRIFT       KDR_CONST_PI * (0.0f / 180.0f)
// Множитель для вычисления корректирующих коэффициентов
#define KDR_F_COEFF_MULT            sqrt(3.0f / 4.0f)

class AHRSMadgwick : public AHRSFilter
{

  public:
    // Конструктор класса
    AHRSMadgwick(uint8_t magnetrometerCorrect = 0, float betaErr = KDR_F_GYRO_MEAS_ERROR, float zetaErr = KDR_F_GYRO_MEAS_DRIFT);
    // Деструктор класса
    ~AHRSMadgwick();
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
    // Вычисление обратного квадратного корня
    float invSqrt(float x);
    
    // Кватернион 0
    volatile float _q0 = 1.0f;
    // Кватернион 1
    volatile float _q1 = 0.0f;
    // Кватернион 2
    volatile float _q2 = 0.0f;
    // Кватернион 3
    volatile float _q3 = 0.0f;
    // Корректирующий коэффициент бета
    float _beta;
    // Корректирующий коэффициент зета
    float _zeta;
};


#endif
