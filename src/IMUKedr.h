/*
 * Основной класс для управления 
 * инерциально-измерительным модулем.
 * 
 * Поддерживаемые платы: КДР-ИНС-Л5В.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_IMUKEDR_ARDUINO
#define __KDR_IMU_IMUKEDR_ARDUINO

#include "IMUCommon.h"

#include "LIS331DLH.h"
#include "L3G4200D.h"
#include "LIS3MDL.h"
#include "LPS25HB.h"

#include "AHRSFilter.h"
#include "AHRSMadgwick.h"

#ifndef KDR_LIGHT
#include "AHRSMahony.h"
#endif

#include "INSIntegrator.h"

/*
 * Режимы работы модуля
 * 
 * Определяет режим работы модуля 
 */
#ifndef KDR_LIGHT
  #define KDR_IMU_MAHONY_MAG      0
  #define KDR_IMU_MAHONY          1
#endif

#define KDR_IMU_MADGWICK_MAG    2
#define KDR_IMU_MADGWICK        3

/*
 * Скорость работы модуля
 * 
 * Определяет скорость работы модуля 
 */

#define KDR_IMU_LOW_ENERGY      0
#define KDR_IMU_NORMAL          1
#define KDR_IMU_FAST            2
#define KDR_IMU_MAXFAST         3


class IMUKedr {
  
  public:
    // Конструктор класса
    IMUKedr(uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t CS4);
    // Деструктор класса
    ~IMUKedr();

    // Инициализация модуля
    void IMUInit(uint8_t mode = KDR_IMU_MADGWICK_MAG, uint8_t dspeed = KDR_IMU_NORMAL);
    // Инициализация курсовертикали
    void AHRSInit();
    // Инициализация инерциальной навигационной системы
    void INSInit();
    // Ожидание стабилизации курсовертикали в течение заданного времени
    void AHRSStabilise(uint8_t seconds);
    // Обновление данных модуля
    void IMUUpdate();

    // Установка матрицы калибровок магнетрометра
    void setMagnetrometerCalibration(const double calibrationMatrix[3][3], const double bias[3]);
    // Установка калибровок гироскопа
    void setGyroscopeCalibration(float biassgx, float biassgy, float biassgz);

    // Получение высоты модуля относительно точки последней инициализации
    float getAltitude();

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
    // Получение собственных ускорений модуля без учета притяжения Земли
    void getClearAcceleration(float * cax, float * cay, float * caz);
    // Получение собственных геопространственных ускорений модуля без учета притяжения Земли
    void getClearGeoAcceleration(float * cgax, float * cgay, float * cgaz);

    LIS331DLH accelerometer;
    L3G4200D  gyroscope;
    LIS3MDL   magnetrometer;
    LPS25HB   barometer;

  private:
    AHRSFilter * ahrs;
    INSIntegrator ins;

    float _biassgx;
    float _biassgy;
    float _biassgz;

    uint8_t _activeAHRS;
    uint8_t _activeINS;
    uint8_t _calibrateCompass;
};

#endif
