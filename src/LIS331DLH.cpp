#include "LIS331DLH.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса модуля управления 
* акселерометром LIS331DLH с использованием протокола SPI ARDUINO.
*/
LIS331DLH::LIS331DLH() : AGMCommon(SS) {
  // Включение осей
  setAxis(KDR_LIS331DLH_AXIS_XYZ);
  
  // Задание режима работы по умолчанию
  setMode(KDR_DR_NORMAL);
   
  // Установка максимального измеряемого ускорения в G
  setFullScale(KDR_FS_2G);
}

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса модуля управления 
* акселерометром LIS331DLH с использованием протокола SPI ARDUINO.
* 
* @param CS - номер пина выбора микросхемы
*/
LIS331DLH::LIS331DLH(uint8_t CS) : AGMCommon(CS) {
  // Включение осей
  setAxis(KDR_LIS331DLH_AXIS_XYZ);
  
  // Задание режима работы по умолчанию
  setMode(KDR_DR_NORMAL);
   
  // Установка максимального измеряемого ускорения в G
  setFullScale(KDR_FS_2G);
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса модуля управления 
* акселерометром LIS331DLH с использованием протокола SPI ARDUINO
* и очищает память.
*/
LIS331DLH::~LIS331DLH() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Проверка правильности подключения акселерометра и настроек SPI
* 
* Проверяет корректность подключения акселерометра и настроек шины SPI
* с помощью тестового опроса регистра "КТО Я" микросхемы.
* 
* @return - 1 при корректном подключении, 0 при ошибке
*/
uint8_t LIS331DLH::testConnection() {
  return whoAmI() == LIS331DLH_WHO_AM_I_REPLY;
}

/**
* Установка активных осей
* 
* Устанавливает оси, по которым проводятся измерения.
* 
* @param aM - константа, определяющая включаемые оси
*/
void LIS331DLH::setAxis(uint8_t aM) {
  // Очищаем старшие 3 бита первого контрольного регистра
  _valCTL1 &= 0b11111000;

  // Устанавливаем биты осей
  _valCTL1 |= aM;

  // Записываем обновлённые данные первого контрольного регистра
  writeCTRL1();
}

/**
* Установка чувствительности
* 
* Устанавливает максимальное измеряемое ускорение для акселерометра.
* Доступны варианты 2G, 4G или 8G. Чем ниже максимальное измеряемое
* ускорение, тем выше точность измерений.
* 
* @param fS - величина максимального измеряемого ускорения
*/
void LIS331DLH::setFullScale(uint8_t fS) {
  switch (fS) {
    case KDR_FS_2G:
      _valCTL4 = CTRL4_REG_2G;
      _multiplicator = KDR_FS_2G / 32767.0;
      break;
    case KDR_FS_4G:
      _valCTL4 = CTRL4_REG_4G;
      _multiplicator = KDR_FS_4G / 32767.0;
      break;
    case KDR_FS_8G: 
      _valCTL4 = CTRL4_REG_8G;
      _multiplicator = KDR_FS_8G / 32767.0;
      break;
    default:
      _multiplicator = KDR_FS_2G / 32767.0;    
      break;
    }
    
    writeCTRL4();
}

/**
* Установка режима работы микросхемы в зависимости от частоты выходных данных
* 
* Устанавливает режим работы микросхемы в зависимости от необходимой частоты обновления
* выходных данных. Варианты заданы константами. Стандартная частота по умолчанию - 50 Гц.
* Может изменяться как в меньшую сторону с переходом в энергосберегающий режим,
* вплоть до 0, при котором микросхема засыпает, так и в большую сторону вплоть до 1000 Гц.
* 
* Необходимо учитывать, что при повышении частоты получения данных выше 50 Гц увеличивается 
* частота среза фильтра нижних частот. Микросхема начинает пропускать больше высокочастотных 
* помех, что может отрицательно сказаться на точности получаемой информации в ваших условиях
* 
* @param dataRate - частота обновления данных микросхемы
*/
void LIS331DLH::setMode(int16_t dataRate) {

  // Очищаем младшие 5 бит первого контрольного регистра
  _valCTL1 &= 0x07;
  
  switch (dataRate) {
    case KDR_DR_HALF_Hz:
      _valCTL1 |= (1 << 6);
      break;
    case KDR_DR_1_Hz:
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      break;
    case KDR_DR_2_Hz:
      _valCTL1 |= (1 << 7);
      break;
    case KDR_DR_5_Hz:
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_DR_10_Hz:
      _valCTL1 |= (1 << 6);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_DR_NORMAL:
      _valCTL1 |= (1 << 5);
      break;
    case KDR_DR_100_Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 5);
      break;
    case KDR_DR_400_Hz:
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 5);
      break;
    case KDR_DR_1000_Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 5);
      break;
  }

  // Записываем обновлённые данные первого контрольного регистра
  writeCTRL1();
}

/**
* Получение ускорения по оси X в числах G
* 
* Получает последние данные по оси X из акселерометра
* и представлет их в виде, кратном числу G
* 
* @return - величина ускорения по оси X в числах G
*/
float LIS331DLH::readGX() {
  return readX() * _multiplicator;
}

/**
* Получение ускорения по оси Y в числах G
* 
* Получает последние данные по оси Y из акселерометра
* и представлет их в виде, кратном числу G
* 
* @return - величина ускорения по оси Y в числах G
*/
float LIS331DLH::readGY() {
  return readY() * _multiplicator;
}

/**
* Получение ускорения по оси Z в числах G
* 
* Получает последние данные по оси Z из акселерометра
* и представлет их в виде, кратном числу G
* 
* @return - величина ускорения по оси Z в числах G
*/
float LIS331DLH::readGZ() {
  return readZ() * _multiplicator;
}

/**
* Получение ускорения по оси X в м/с^2
* 
* Получает последние данные по оси X из акселерометра
* и представлет их в м/с^2
* 
* @return - величина ускорения по оси X в м/с^2
*/
float LIS331DLH::readAX() {
  return readGX() * KDR_CONST_G;
}

/**
* Получение ускорения по оси Y в м/с^2
* 
* Получает последние данные по оси Y из акселерометра
* и представлет их в м/с^2
* 
* @return - величина ускорения по оси Y в м/с^2
*/
float LIS331DLH::readAY() {
  return readGY() * KDR_CONST_G;
}

/**
* Получение ускорения по оси Z в м/с^2
* 
* Получает последние данные по оси Z из акселерометра
* и представлет их в м/с^2
* 
* @return - величина ускорения по оси Z в м/с^2
*/
float LIS331DLH::readAZ() {
  return readGZ() * KDR_CONST_G;
}

/**
* Получение ускорения по всем трём осям в числах G
* 
* Получает последние данные по всем трём осям из акселерометра
* и представлет их виде, кратном числу G
* 
* @return - величина ускорения по всем трём осям в числах G
*/
void LIS331DLH::readGXYZ(float *gx, float *gy, float *gz) {
  int16_t x, y, z;
  readXYZ(&x, &y, &z);
   
  *gx = x * _multiplicator;
  *gy = y * _multiplicator;
  *gz = z * _multiplicator;
}

/**
* Получение ускорения по всем трём осям в м/с^2
* 
* Получает последние данные по всем трём осям из акселерометра
* и представлет их в м/с^2
* 
* @return - величина ускорения по всем трём осям в м/с^2
*/
void LIS331DLH::readAXYZ(float *ax, float *ay, float *az) {
  readGXYZ(ax, ay, az);
  
  (*ax) *= KDR_CONST_G;
  (*ay) *= KDR_CONST_G;
  (*az) *= KDR_CONST_G;
}