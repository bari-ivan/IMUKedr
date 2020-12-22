#include "L3G4200D.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса модуля управления 
* гироскопом L3G4200D с использованием протокола SPI ARDUINO.
*/
L3G4200D::L3G4200D() : AGMCommon(SS) {
  // Включение осей
  setAxis(KDR_L3G4200D_AXIS_XYZ);
  
  // Задание режима работы по умолчанию
  setMode(KDR_G_DR_NORMAL);
  
  // Установка максимального измеряемого ускорения в градусах в секунду
  setFullScale(KDR_FS_250DPS);
}

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса модуля управления 
* гироскопом L3G4200D с использованием протокола SPI ARDUINO.
* 
* @param CS - номер пина выбора микросхемы
*/
L3G4200D::L3G4200D(uint8_t CS) : AGMCommon(CS) {
  // Включение осей
  setAxis(KDR_L3G4200D_AXIS_XYZ);
  
  // Задание режима работы по умолчанию
  setMode(KDR_G_DR_NORMAL);
  
  // Установка максимального измеряемого ускорения в градусах в секунду
  setFullScale(KDR_FS_250DPS);
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса модуля управления 
* гироскопом L3G4200D с использованием протокола SPI ARDUINO
* и очищает память.
*/
L3G4200D::~L3G4200D() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Проверка правильности подключения гироскопа и настроек SPI
* 
* Проверяет корректность подключения гироскопа и настроек шины SPI
* с помощью тестового опроса регистра "КТО Я" микросхемы.
* 
* @return - 1 при корректном подключении, 0 при ошибке
*/
uint8_t L3G4200D::testConnection() {
  return whoAmI() == L3G4200D_WHO_AM_I_REPLY;
}

/**
* Установка активных осей
* 
* Устанавливает оси, по которым проводятся измерения.
* 
* @param aM - константа, определяющая включаемые оси
*/
void L3G4200D::setAxis(uint8_t aM) {
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
* Устанавливает максимальную измеряемую угловую скорость для гироскопа.
* Доступны варианты 250, 500 или 2000 градусов в секунду. Чем ниже 
* максимальная измеряемая угловая скорость, тем выше точность измерений.
* 
* @param fS - величина максимальной измеряемой угловой скорости
*/
void L3G4200D::setFullScale(uint16_t fS) {
    switch (fS) {
    case KDR_FS_250DPS:
      _valCTL4 = CTRL4_REG_250DPS;
      _multiplicator = KDR_G_MULT_250DPS;
      break;
    case KDR_FS_500DPS:
      _valCTL4 = CTRL4_REG_500DPS;
      _multiplicator = KDR_G_MULT_500DPS;
      break;
    case KDR_FS_2000DPS: 
      _valCTL4 = CTRL4_REG_2000DPS;
      _multiplicator = KDR_G_MULT_2000DPS;
      break;
    default:
      _multiplicator = KDR_G_MULT_250DPS;    
      break;
    }
    
    writeCTRL4();
}
    
/**
* Установка режима работы микросхемы в зависимости от частоты выходных 
* данных и частоты среза фильтра нижних частот
* 
* Устанавливает режим работы микросхемы в зависимости от необходимой частоты обновления
* выходных данных и частоты среза фильтра нижних частот. Варианты заданы константами. 
* Стандартная частота по умолчанию - 100 Гц.
* Может изменяться в большую сторону вплоть до 800 Гц. При задании 0 Гц микросхема выключается.
* Для каждой частоты обновления выходных данных можно настроить частоту среза фильтра нижних частот,
* выбрав один из предложенных вариантов. По умолчанию для частоты обновления в 100 Гц это 12,5 Гц.
* 
* Необходимо учитывать, что при повышении частоты среза фильтра нижних частот микросхема начинает 
* пропускать больше высокочастотных помех, что может отрицательно сказаться на точности получаемой 
* информации в ваших условиях.
* 
* @param dataRateCutOff - константа, определяющая режим работы микросхемы
*/
void L3G4200D::setMode(int16_t dataRateCutOff) {
  // Очищаем младшие 5 бит первого контрольного регистра
  _valCTL1 &= 0x07;

  switch (dataRateCutOff) {
    case KDR_G_DR_SLEEP:
      _valCTL1 &= 0x00;
      _valCTL1 |= (1 << 3);
      break;
    case KDR_G_DR_NORMAL:
      _valCTL1 |= (1 << 3);
      break;
    case KDR_G_DR_100Hz_CO_25Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      break;
    case KDR_G_DR_200Hz_CO_12_5Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 6);
      break;
    case KDR_G_DR_200Hz_CO_25Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 6);
      break;
    case KDR_G_DR_200Hz_CO_50Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      break;
    case KDR_G_DR_200Hz_CO_70Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      break;
    case KDR_G_DR_400Hz_CO_20Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_400Hz_CO_25Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_400Hz_CO_50Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_400Hz_CO_110Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_800Hz_CO_30Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 6);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_800Hz_CO_35Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 6);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_800Hz_CO_50Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      _valCTL1 |= (1 << 7);
      break;
    case KDR_G_DR_800Hz_CO_110Hz:
      _valCTL1 |= (1 << 3);
      _valCTL1 |= (1 << 4);
      _valCTL1 |= (1 << 5);
      _valCTL1 |= (1 << 6);
      _valCTL1 |= (1 << 7);
      break;
  }

  // Записываем обновлённые данные первого контрольного регистра
  writeCTRL1();
}

/**
* Получение угловой скорости по оси X в градусах в секунду
* 
* Получает последние данные по оси X из гироскопа
* и представлет их в градусах в секунду
* 
* @return - величина ускорения по оси X в градусах в секунду
*/
float L3G4200D::readDegX() {
  return readX() * _multiplicator;
}

/**
* Получение угловой скорости по оси Y в градусах в секунду
* 
* Получает последние данные по оси Y из гироскопа
* и представлет их в градусах в секунду
* 
* @return - величина ускорения по оси Y в градусах в секунду
*/
float L3G4200D::readDegY() {
  return readY() * _multiplicator;
}

/**
* Получение угловой скорости по оси Z в градусах в секунду
* 
* Получает последние данные по оси Z из гироскопа
* и представлет их в градусах в секунду
* 
* @return - величина ускорения по оси Z в градусах в секунду
*/
float L3G4200D::readDegZ() {
  return readZ() * _multiplicator;
}

/**
* Получение угловой скорости по оси X в радианах в секунду
* 
* Получает последние данные по оси X из гироскопа
* и представлет их в радианах в секунду
* 
* @return - величина ускорения по оси X в радианах в секунду
*/
float L3G4200D::readRadX() {
  return readDegX() * KDR_CONST_DEG_TO_RAD;
}

/**
* Получение угловой скорости по оси Y в радианах в секунду
* 
* Получает последние данные по оси Y из гироскопа
* и представлет их в радианах в секунду
* 
* @return - величина ускорения по оси Y в радианах в секунду
*/
float L3G4200D::readRadY() {
  return readDegY() * KDR_CONST_DEG_TO_RAD;
}

/**
* Получение угловой скорости по оси Z в радианах в секунду
* 
* Получает последние данные по оси Z из гироскопа
* и представлет их в радианах в секунду
* 
* @return - величина ускорения по оси Z в радианах в секунду
*/
float L3G4200D::readRadZ() {
  return readDegZ() * KDR_CONST_DEG_TO_RAD;
}

/**
* Получение угловой скорости по всем трём осям в градусах в секунду
* 
* Получает последние данные по всем трём осям из гироскопа
* и представлет их в градусах в секунду
* 
* @return - величина ускорения по всем трём осям в градусах в секунду
*/
void L3G4200D::readDegXYZ(float *gx, float *gy, float *gz) {
  int16_t x, y, z;
  readXYZ(&x, &y, &z);
  
  *gx = x * _multiplicator;
  *gy = y * _multiplicator;
  *gz = z * _multiplicator;
}

/**
* Получение угловой скорости по всем трём осям в радианах в секунду
* 
* Получает последние данные по всем трём осям из гироскопа
* и представлет их в радианах в секунду
* 
* @return - величина ускорения по всем трём осям в радианах в секунду
*/
void L3G4200D::readRadXYZ(float *gx, float *gy, float *gz) {
    readDegXYZ(gx, gy, gz);
    
    (*gx) *= KDR_CONST_DEG_TO_RAD;
    (*gy) *= KDR_CONST_DEG_TO_RAD;
    (*gz) *= KDR_CONST_DEG_TO_RAD;
}
