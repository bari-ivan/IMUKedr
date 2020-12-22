/*
 * Модуль управления магнетрометром LIS3MDL 
 * с использованием протокола SPI ARDUINO.
 * 
 * Версия: 1.0.0.
 * Автор: Ivan Barinov <bari_ivan@mail.ru>
 */

#ifndef __KDR_IMU_LIS3MDL_ARDUINO
#define __KDR_IMU_LIS3MDL_ARDUINO

#include "AGMCommon.h"

// Содержимое по умолчанию регистра "КТО Я" микросхемы
#define LIS3MDL_WHO_AM_I_REPLY 0x3D

/*
 * Калибровочные данные по умолчанию
 */

// Смещения по умолчанию
#define KDR_M_DEF_BIAS {0, 0, 0}

// Матрица калибровок по умолчанию
#define KDR_M_DEF_CALIBRATION_MATRIX {{0.3333333, 0.3333333, 0.3333333}, {0.3333333, 0.3333333, 0.3333333}, {0.3333333, 0.3333333, 0.3333333}}

/*
 * Константы, определяющие чувствительность датчика
 */

// Константы для преобразования значения и задания режима в библиотеке
#define KDR_FS_4_GS               4
#define KDR_FS_8_GS               8
#define KDR_FS_12_GS              12
#define KDR_FS_16_GS              16

// Значения четвёртого контрольного регистра для перевода микросхемы в нужный режим
#define CTRL2_REG_4_GS        0x00
#define CTRL2_REG_8_GS        0x20
#define CTRL2_REG_12_GS       0x40
#define CTRL2_REG_16_GS       0x60

// Значения коэффициента преобразования данных для каждого из режимов
#define KDR_M_MULT_4_GS       6842
#define KDR_M_MULT_8_GS       3421
#define KDR_M_MULT_12_GS      2281
#define KDR_M_MULT_16_GS      1711

/*
 * Константы, определяющие плоскость, в которой вычисляется азимут
 */

// Модуль параллельно земле микросхемами вверх - направляющая ось - X
#define KDR_M_AZIMUT_XY       0
// Модуль вертикально контактами вверх - направляющая ось - Z
#define KDR_M_AZIMUT_YZ       1
// Модуль на боку выходом GND вверх - направляющая ось - Z
#define KDR_M_AZIMUT_XZ       2

/*
 * Константы, определяющие частоту обновления выходных данных и их качество
 */
 
// *****
// Формат константы KDR_M_DR_%частота%_%качество% - где качество задаётся обозначениями:
//
// UHP - наивысшее качество
// HP  - высокое качество
// MP  - среднее качество
// LP  - энергосберегающий режим (низкое качество)
//
// *****

#define KDR_M_DR_SLEEP            0

#define KDR_M_DR_0_625_UHP        1
#define KDR_M_DR_0_625_HP         2
#define KDR_M_DR_0_625_MP         3
#define KDR_M_DR_0_625_LP         4

#define KDR_M_DR_1_25_UHP         5
#define KDR_M_DR_1_25_HP          6
#define KDR_M_DR_1_25_MP          7
#define KDR_M_DR_1_25_LP          8

#define KDR_M_DR_2_5_UHP          9
#define KDR_M_DR_2_5_HP           10
#define KDR_M_DR_2_5_MP           11
#define KDR_M_DR_2_5_LP           12

#define KDR_M_DR_5_UHP            13
#define KDR_M_DR_5_HP             14
#define KDR_M_DR_5_MP             15
#define KDR_M_DR_5_LP             16

#define KDR_M_DR_NORMAL           17
#define KDR_M_DR_10_HP            18
#define KDR_M_DR_10_MP            19
#define KDR_M_DR_10_LP            20

#define KDR_M_DR_20_UHP           21
#define KDR_M_DR_20_HP            22
#define KDR_M_DR_20_MP            23
#define KDR_M_DR_20_LP            24

#define KDR_M_DR_40_UHP           25
#define KDR_M_DR_40_HP            26
#define KDR_M_DR_40_MP            27
#define KDR_M_DR_40_LP            28

#define KDR_M_DR_80_UHP           29
#define KDR_M_DR_80_HP            30
#define KDR_M_DR_80_MP            31
#define KDR_M_DR_80_LP            32

#define KDR_M_DR_155_UHP          33
#define KDR_M_DR_300_HP           34
#define KDR_M_DR_560_MP           35
#define KDR_M_DR_1000_LP          36


/*
 * Константы, определяющие режим преобразования встроенного АЦП
 */

#define KDR_M_CM_SINGLE           0
#define KDR_M_CM_CONTINIOUS       1

class LIS3MDL : public AGMCommon {
  
  public:
    // Конструктор класса
    LIS3MDL();
    // Конструктор класса
    LIS3MDL(uint8_t CS);
    // Конструктор класса с заданием матрицы калибровок
    LIS3MDL(const double calibrationMatrix[3][3], const double bias[3]);
    // Конструктор класса с заданием матрицы калибровок
    LIS3MDL(uint8_t CS, const double calibrationMatrix[3][3], const double bias[3]);
    // Деструктор класса
    ~LIS3MDL();
    // Проверка правильности подключения акселерометра и настроек SPI
    uint8_t testConnection();
    // Установка активных осей
    void setAxis(uint8_t aM);
    // Установка чувствительности
    void setFullScale(uint8_t fS);
    // Установка режима преобразования встроенного АЦП
    void setConversionMode(uint8_t cM);
    // Установка режима работы микросхемы в зависимости от частоты выходных данных
    void setMode(int16_t dataRate);
    // Установка матрицы калибровок
    void setCalibrationMatrix(const double calibrationMatrix[3][3], const double bias[3]);
    // Получение индукции магнитного поля по оси X в Гауссах без калибровки
    float readGaussX();
    // Получение индукции магнитного поля по оси Y в Гауссах без калибровки
    float readGaussY();
    // Получение индукции магнитного поля по оси Z в Гауссах без калибровки
    float readGaussZ();
    // Получение индукции магнитного поля по оси X в Гауссах с наложением калибровок
    float readCalibrateGaussX();
    // Получение индукции магнитного поля по оси Y в Гауссах с наложением калибровок
    float readCalibrateGaussY();
    // Получение индукции магнитного поля по оси Z в Гауссах с наложением калибровок
    float readCalibrateGaussZ();
    // Получение индукции магнитного поля по всем трём осям в Гауссах без калибровки
    void readGaussXYZ(float *gx, float *gy, float *gz);
    // Получение индукции магнитного поля по всем трём осям в Гауссах с наложением калибровок
    void readCalibrateGaussXYZ(float *gcx, float *gcy, float *gcz);
    // Получение текущего азимута
    float readAzimut(uint8_t direction);
    // Получение текущего азимута без учёта калибровок
    float readUncalibratedAzimut(uint8_t direction);

  private:
    // Включает магнетрометр с настройками по умолчанию
    void beginDefault();
    // Получение исходных данных магнетрометра по всем трём осям с наложением калибровок
    void readCalibrateXYZ(float *cx, float *cy, float *cz);
    // Матрица калибровок для магнетрометра
    double _calibrationMatrix[3][3];
    // Биасы для магнетрометра
    double _bias[3];
    // Режим преобразования встроенного АЦП
    uint8_t conversionMode;
};

#endif
