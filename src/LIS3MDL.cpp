#include "LIS3MDL.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса модуля управления 
* магнетрометром LIS3MDL с использованием протокола SPI ARDUINO.
*/
LIS3MDL::LIS3MDL() : AGMCommon(SS) {

  // Установка калибровочных данных по умолчанию
  const double calibrationMatrix[3][3] = KDR_M_DEF_CALIBRATION_MATRIX;
  const double bias[3] = KDR_M_DEF_BIAS;
  
  setCalibrationMatrix(calibrationMatrix, bias);

  // Включение магнетрометра с настройками по умолчанию
  beginDefault();
}

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса модуля управления 
* магнетрометром LIS3MDL с использованием протокола SPI ARDUINO.
* 
* @param CS - номер пина выбора микросхемы
*/
LIS3MDL::LIS3MDL(uint8_t CS) : AGMCommon(CS) {

  // Установка калибровочных данных по умолчанию
  const double calibrationMatrix[3][3] = KDR_M_DEF_CALIBRATION_MATRIX;
  const double bias[3] = KDR_M_DEF_BIAS;
  
  setCalibrationMatrix(calibrationMatrix, bias);

  // Включение магнетрометра с настройками по умолчанию
  beginDefault();
}

/**
* Конструктор класса с заданием матрицы калибровок
* 
* Инициализирует экземпляр класса модуля управления 
* магнетрометром LIS3MDL с использованием протокола SPI ARDUINO и
* устанавливает параметры калибровки выходных данных.
* 
* @param calibrationMatrix - матрица масштаба и ортогонализации
* @param bias - смещения по осям
*/
LIS3MDL::LIS3MDL(const double calibrationMatrix[3][3], const double bias[3]) : AGMCommon(SS) {
  // Установка пользовательских калибровочных данных
  setCalibrationMatrix(calibrationMatrix, bias);

  // Включение магнетрометра с настройками по умолчанию
  beginDefault();
}

/**
* Конструктор класса с заданием матрицы калибровок
* 
* Инициализирует экземпляр класса модуля управления 
* магнетрометром LIS3MDL с использованием протокола SPI ARDUINO и
* устанавливает параметры калибровки выходных данных.
* 
* @param CS - номер пина выбора микросхемы
* @param calibrationMatrix - матрица масштаба и ортогонализации
* @param bias - смещения по осям
*/
LIS3MDL::LIS3MDL(uint8_t CS, const double calibrationMatrix[3][3], const double bias[3]) : AGMCommon(CS) {
  // Установка пользовательских калибровочных данных
  setCalibrationMatrix(calibrationMatrix, bias);

  // Включение магнетрометра с настройками по умолчанию
  beginDefault();
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса модуля управления 
* магнетрометром LIS3MDL с использованием протокола SPI ARDUINO
* и очищает память.
*/
LIS3MDL::~LIS3MDL() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Проверка правильности подключения магнетрометра и настроек SPI
* 
* Проверяет корректность подключения акселерометра и настроек шины SPI
* с помощью тестового опроса регистра "КТО Я" микросхемы.
* 
* @return - 1 при корректном подключении, 0 при ошибке
*/
uint8_t LIS3MDL::testConnection() {
  return whoAmI() == LIS3MDL_WHO_AM_I_REPLY;
}

/**
* Включает магнетрометр с настройками по умолчанию
* 
* Задаёт значения управляющих регистров магнетрометра по умолчанию
* и подготавливает микросхему к работе.
*/
void LIS3MDL::beginDefault() {
  // Включение режима непрерывного преобразования
  setConversionMode(KDR_M_CM_CONTINIOUS);

  // Задание режима работы по умолчанию
  setMode(KDR_M_DR_NORMAL);

  // Установка чувствительности
  setFullScale(KDR_FS_4_GS);
}

/**
* Установка чувствительности
* 
* Устанавливает максимальную измеряемую индукцию для магнетрометра.
* Доступны варианты 4, 8, 12 или 16 Гс. Чем ниже максимальная измеряемая
* индукция, тем выше точность измерений.
* 
* @param fS - величина максимальной измеряемой индукции магнитного поля
*/
void LIS3MDL::setFullScale(uint8_t fS) {

  switch (fS) {
    case KDR_FS_4_GS: 
      _valCTL2 = CTRL2_REG_4_GS;
      _multiplicator = KDR_M_MULT_4_GS;
      break;
    case KDR_FS_8_GS: 
      _valCTL2 = CTRL2_REG_8_GS;
      _multiplicator = KDR_M_MULT_8_GS;
      break;
    case KDR_FS_12_GS: 
      _valCTL2 = CTRL2_REG_12_GS;
      _multiplicator = KDR_M_MULT_12_GS;
      break;
    case KDR_FS_16_GS: 
      _valCTL2 = CTRL2_REG_16_GS;
      _multiplicator = KDR_M_MULT_16_GS;
      break;
    default:
      _valCTL2 = CTRL2_REG_4_GS;
      _multiplicator = KDR_M_MULT_4_GS;
      break;
  }

  writeCTRL2();
}

/**
* Установка режима преобразования встроенного АЦП
* 
* Устанавливает режим преобразования встроенного АЦП. При непрерывном преобразовании
* данные будут обновляться непрерывно. При одиночном - следующее измерение начнётся
* не раньше считывания предыдущих данных.
* 
* @param cM - Константа, определяющая режим преобразования встроенного АЦП
*/
void LIS3MDL::setConversionMode(uint8_t cM) {
  // Сохранение устанавливаемого значения
  conversionMode = cM;

  // Очистка старших 2-х бит третьего контрольного регистра
  _valCTL3 &= 0b11111100;

  switch (cM)
  {
  case KDR_M_CM_SINGLE:
    _valCTL1 |= (1 << 0);
    break;

  case KDR_M_CM_CONTINIOUS:
    // --
    break;
  }

}

/**
* Установка режима работы микросхемы в зависимости от частоты выходных данных
* 
* Устанавливает режим работы микросхемы в зависимости от необходимой частоты обновления
* выходных данных. Варианты заданы константами. Стандартная частота по умолчанию - 10 Гц.
* Может изменяться как в меньшую сторону, вплоть до 0, при котором микросхема засыпает, 
* так и в большую сторону вплоть до 1000 Гц. Для каждой частоты доступно задание качества
* получаемых данных. Чем выше качество, тем больше энергии потребляет микросхема. Учитывайте
* это, если есть жесткие требования к энергопотреблению вашего устройства.
* 
* Необходимо учитывать, что при повышении частоты получения данных выше 155 Гц микросхема 
* безальтернативно начинает снижать качество выходных данных ради обеспечения высокой частоты 
* измерений.
* 
* @param dataRate - частота обновления данных микросхемы с заданием качества получаемых данных
*/
void LIS3MDL::setMode(int16_t dataRate) {
  // Очистка первого контрольного регистра
  _valCTL1 = 0x00;

  // Очистка четвёртого контрольного регистра
  _valCTL4 = 0x00;
  
  // Переустановка режима преобразования встроенного АЦП
  setConversionMode(conversionMode);

  switch(dataRate) {
    case KDR_M_DR_SLEEP:
      _valCTL3 |= 0x03;
      break;
    case KDR_M_DR_0_625_UHP:
      // Установка частоты
      // --
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_0_625_HP:
      // Установка частоты
      // --
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_0_625_MP:
      // Установка частоты
      // --
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_0_625_LP:
      // Установка частоты
      // --
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_1_25_UHP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_1_25_HP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_1_25_MP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_1_25_LP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_2_5_UHP:
      // Установка частоты
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_2_5_HP:
      // Установка частоты
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_2_5_MP:
      // Установка частоты
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_2_5_LP:
      // Установка частоты
      _valCTL1 |= (1 << 3);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_5_UHP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_5_HP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_5_MP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_5_LP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_NORMAL:
      // Установка частоты
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_10_HP:
      // Установка частоты
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_10_MP:
      // Установка частоты
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_10_LP:
      // Установка частоты
      _valCTL1 |= (1 << 4);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_20_UHP:
      // Установка частоты
      // --
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_20_HP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_20_MP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_20_LP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 4);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_40_UHP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_40_HP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_40_MP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_40_LP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_80_UHP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_80_HP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_80_MP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_80_LP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;
    case KDR_M_DR_155_UHP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Включение FastMode
      _valCTL1 |= (1 << 1);
      // Установка качества
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_300_HP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Включение FastMode
      _valCTL1 |= (1 << 1);
      // Установка качества
      _valCTL1 |= (1 << 6);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 3);
      break;
    case KDR_M_DR_560_MP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Включение FastMode
      _valCTL1 |= (1 << 1);
      // Установка качества
      _valCTL1 |= (1 << 5);
      // Установка качества для оси Z
      _valCTL4 |= (1 << 2);
      break;
    case KDR_M_DR_1000_LP:
      // Установка частоты
      _valCTL1 |= (1 << 2);
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      // Включение FastMode
      _valCTL1 |= (1 << 1);
      // Установка качества
      // --
      // Установка качества для оси Z
      // --
      break;

  }
  
  // Запись настроек в регистры
  writeCTRL1();
  writeCTRL3();
  writeCTRL4();
}

/**
* Установка матрицы калибровок
* 
* Устанавливает параметры калибровки выходных данных.
* 
* @param calibrationMatrix - матрица масштаба и ортогонализации
* @param bias - смещения по осям
*/
void LIS3MDL::setCalibrationMatrix(const double calibrationMatrix[3][3], const double bias[3]) {
  memcpy (_bias, bias, 3 * sizeof (double));
  memcpy (_calibrationMatrix, calibrationMatrix, 3 * 3 * sizeof (double));
}

/**
* Получение индукции магнитного поля по оси X в Гауссах без калибровки
* 
* Получает последние данные по оси X из магнетрометра
* и представлет их в Гауссах без учёта калибровочных данных
* 
* @return - величина индукции магнитного поля по оси X в Гауссах без калибровки
*/
float LIS3MDL::readGaussX() {
  return readX() / _multiplicator;
}

/**
* Получение индукции магнитного поля по оси Y в Гауссах без калибровки
* 
* Получает последние данные по оси Y из магнетрометра
* и представлет их в Гауссах без учёта калибровочных данных
* 
* @return - величина индукции магнитного поля по оси Y в Гауссах без калибровки
*/
float LIS3MDL::readGaussY() {
  return readY() / _multiplicator;
}

/**
* Получение индукции магнитного поля по оси Z в Гауссах без калибровки
* 
* Получает последние данные по оси Z из магнетрометра
* и представлет их в Гауссах без учёта калибровочных данных
* 
* @return - величина индукции магнитного поля по оси Z в Гауссах без калибровки
*/
float LIS3MDL::readGaussZ() {
  return readZ() / _multiplicator;
}

/**
* Получение индукции магнитного поля по оси X в Гауссах с наложением калибровок
* 
* Получает последние данные по оси X из магнетрометра
* и представлет их в Гауссах с учётом калибровочных данных
* 
* @return - величина индукции магнитного поля по оси X в Гауссах с наложением калибровок
*/
float LIS3MDL::readCalibrateGaussX() {
  float x, y, z;

  readCalibrateXYZ(&x, &y, &z);

  // Формирование выходного значения
  return x / _multiplicator;
}

/**
* Получение индукции магнитного поля по оси Y в Гауссах с наложением калибровок
* 
* Получает последние данные по оси Y из магнетрометра
* и представлет их в Гауссах с учётом калибровочных данных
* 
* @return - величина индукции магнитного поля по оси Y в Гауссах с наложением калибровок
*/
float LIS3MDL::readCalibrateGaussY() {
  float x, y, z;

  readCalibrateXYZ(&x, &y, &z);

  // Формирование выходного значения
  return y / _multiplicator;
}

/**
* Получение индукции магнитного поля по оси Z в Гауссах с наложением калибровок
* 
* Получает последние данные по оси Z из магнетрометра
* и представлет их в Гауссах с учётом калибровочных данных
* 
* @return - величина индукции магнитного поля по оси Z в Гауссах с наложением калибровок
*/
float LIS3MDL::readCalibrateGaussZ() {
  float x, y, z;

  readCalibrateXYZ(&x, &y, &z);

  // Формирование выходного значения
  return z / _multiplicator;
}

/**
* Получение индукции магнитного поля по всем трём осям в Гауссах без калибровки
* 
* Получает последние данные по всем трём осям из магнетрометра
* и представлет их в Гауссах без учёта калибровочных данных
* 
* @return - величина индукции магнитного поля по всем трём осям в Гауссах без калибровки
*/
void LIS3MDL::readGaussXYZ(float *gx, float *gy, float *gz) {
  int16_t x, y, z;
  readXYZ(&x, &y, &z);
  
  *gx = x / _multiplicator;
  *gy = y / _multiplicator;
  *gz = z / _multiplicator;
}

// 
/**
* Получение исходных данных магнетрометра по всем трём осям с наложением калибровок
* 
* Получает последние данные по всем трём осям из магнетрометра
* и представлет их с учётом калибровочных данных
* 
* @return - исходные данных магнетрометра по всем трём осям с наложением калибровок
*/
void LIS3MDL::readCalibrateXYZ(float *cx, float *cy, float *cz) {
  float result[3] = {0, 0, 0};
  float uncalibratedValues[3];
  int16_t x, y, z;

  // Чтение исходных значений по осям X, Y, Z
  readXYZ(&x, &y, &z);

  // Задание смещений
  uncalibratedValues[0] = x - _bias[0];
  uncalibratedValues[1] = y - _bias[1];
  uncalibratedValues[2] = z - _bias[2];

  // Наложение матрицы калибровок
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i] += _calibrationMatrix[i][j] * uncalibratedValues[j];
    }
  }

  // Формирование выходных значений
  *cx = result[0];
  *cy = result[1];
  *cz = result[2];
}

/**
* Получение индукции магнитного поля по всем трём осям в Гауссах с наложением калибровок
* 
* Получает последние данные по всем трём осям из магнетрометра
* и представлет их в Гауссах с учётом калибровочных данных
* 
* @return - величина индукции магнитного поля по всем трём осям в Гауссах с наложением калибровок
*/
void LIS3MDL::readCalibrateGaussXYZ(float *gcx, float *gcy, float *gcz) {
  float x, y, z;

  // Чтение исходных значений по осям X, Y, Z с учётом калибровок
  readCalibrateXYZ(&x, &y, &z);

  // Формирование выходных значений
  *gcx = x / _multiplicator;
  *gcy = y / _multiplicator;
  *gcz = z / _multiplicator;
}

/**
* Получение текущего азимута без учёта калибровок
* 
* Вычисляет текущий азимут в градусах.
* 
* @param direction - константа, задающая плоскость, в которой вычисляется азимут
* @return - текущий азимут в градусах
*/
float LIS3MDL::readUncalibratedAzimut(uint8_t direction) {
  int16_t cX, cY, cZ;
  float heading;
  float headingDegrees;

  // Чтение исходных значений по осям X, Y, Z с учётом калибровок
  readXYZ(&cX, &cY, &cZ);

  // Выбор плоскости для вычислений и вычисление азимута в радианах
  switch (direction)
  {
  case KDR_M_AZIMUT_XY:
    heading = atan2(cY, cX);
    break;

  case KDR_M_AZIMUT_YZ:
    heading = atan2(cZ, cY);
    break;

  case KDR_M_AZIMUT_XZ:
    heading = atan2(cZ, cX);
    break;
  
  default:
    heading = atan2(cY, cX);
    break;
  }

  // Приведение значения азимута в диапазон 0 - 2*Pi
  if(heading < 0) {
    heading += 2 * KDR_CONST_PI;
  } else if(heading > 2 * KDR_CONST_PI) {
    heading -= 2 * KDR_CONST_PI;
  }

  // Перевод значения азимута в градусы
  headingDegrees = heading * KDR_CONST_RAD_TO_DEG;

  return headingDegrees;
}

/**
* Получение текущего азимута
* 
* Вычисляет текущий азимут в градусах.
* 
* @param direction - константа, задающая плоскость, в которой вычисляется азимут
* @return - текущий азимут в градусах
*/
float LIS3MDL::readAzimut(uint8_t direction) {
  float cX, cY, cZ;
  float heading;
  float headingDegrees;

  // Чтение исходных значений по осям X, Y, Z с учётом калибровок
  readCalibrateXYZ(&cX, &cY, &cZ);

  // Выбор плоскости для вычислений и вычисление азимута в радианах
  switch (direction)
  {
  case KDR_M_AZIMUT_XY:
    heading = atan2(cY, cX);
    break;

  case KDR_M_AZIMUT_YZ:
    heading = atan2(cZ, cY);
    break;

  case KDR_M_AZIMUT_XZ:
    heading = atan2(cZ, cX);
    break;
  
  default:
    heading = atan2(cY, cX);
    break;
  }

  // Приведение значения азимута в диапазон 0 - 2*Pi
  if(heading < 0) {
    heading += 2 * KDR_CONST_PI;
  } else if(heading > 2 * KDR_CONST_PI) {
    heading -= 2 * KDR_CONST_PI;
  }

  // Перевод значения азимута в градусы
  headingDegrees = heading * KDR_CONST_RAD_TO_DEG;

  return headingDegrees;
}
