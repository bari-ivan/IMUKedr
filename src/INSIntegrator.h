/*
 * Родительский класс для реализации работы
 * модуля в режиме инерциальной навигационной системы.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_INSINTEGRATOR_ARDUINO
#define __KDR_IMU_INSINTEGRATOR_ARDUINO

#include "IMUOrientation.h"

// Коэффициент комплиментарного фильтра по умолчанию
#define KDR_INS_COMLIMENTARY_COEFF_DEFAULT          0.01

class INSIntegrator : public IMUOrientation
{

  public:
    // Конструктор класса
    INSIntegrator();
    // Конструктор класса
    INSIntegrator(float comlimentaryCoeff);
    // Деструктор класса
    ~INSIntegrator();
    // Сброс данных и инициализация системы
    void init(float P0, float T0);
    // Обновление данных системы
    void update(float ax, float ay, float az, float bp, float bt, float qW, float qX, float qY, float qZ);
    // Вычисление высоты и вертикальной скорости с использованием комплиментарного фильтра
    void altitudeComplimentary(float * H, float * Vz, float baroH, float Az, float period);
    // Вычитание компонентов ускорения свободного падения из вектора ускорения модуля
    void earthAccelerationCorrect(float *ax, float *ay, float *az, float qW, float qX, float qY, float qZ);
    // Перевод собственных ускорений модуля в систему координат относительно Земли
    void accelerationToGeo(float *ax, float *ay, float *az, float qW, float qX, float qY, float qZ);
    // Вычисление разницы высот по показателям барометра
    float baroHDelta(float baroP);
    // Вычисление разницы высот по показателям барометра с учётом изменений температуры
    float baroHDelta(float baroP, float baroT);
    // Получение перемещений модуля по оси Z
    float getAltitude();

  protected:
    // Перемещение по геопространственной оси Z
    float _altitude;
    // Вектор скорости, разложенный по осям
    float _verticalVelocity;
    // Величина ускорения свободного падения с учетом калибровок акселерометра
    float accG;
    // Атмосферное давление в момент инициализации датчика
    float initP0;
    // Температура в момент инициализации датчика
    float initT0;
    // Коэффициент комплиментарного фильтра
    float _comlimentaryCoeff;
    // P - составляющая PID-регулятора
    float _deltaP;
    // I - составляющая PID-регулятора
    float _deltaI;
    // D - составляющая PID-регулятора
    float _deltaD;
    // P - коэффициент PID-регулятора
    float _P;
    // I - коэффициент PID-регулятора
    float _I;
    // D - коэффициент PID-регулятора
    float _D;

};


#endif
