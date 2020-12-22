#include "IMUCommon.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса для реализации 
* модулей работы с MEMS-сенсорами STMicroelectronics
* 
* @param CS - номер пина выбора микросхемы
*/
IMUCommon::IMUCommon(uint8_t CS) {
  chipSelectPin = CS;
  // Инициализируем пин выбора микросхемы как GPIO-выход
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);

  _valCTL1 = 0;
  _valCTL2 = 0;
  _valCTL3 = 0;
  _valCTL4 = 0;
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса для реализации 
* модулей работы с MEMS-сенсорами STMicroelectronics
* и очищает память
*/
IMUCommon::~IMUCommon() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Чтение регистра
* 
* Читает значение регистра микросхемы с заданным 
* адресом
* 
* @param addr - адрес регистра, значение от 0x00 до 0xFF
* @return - прочитанный из регистра микросхемы байт данных
*/
uint8_t IMUCommon::readRegister(uint8_t addr) {
  uint8_t masterReceive;
  uint8_t masterSend;

  // Добавляем настройки для чтения регистра
  masterSend = addr | IMU_SIMPLE_READ;
  
  // Начало использования шины SPI
  SPI.beginTransaction(SPISettings(KDR_SPI_MAXSPEED, KDR_SPI_FIRSTBIT, KDR_SPI_MODE));
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(masterSend);
  masterReceive = SPI.transfer(0x00);

  digitalWrite(chipSelectPin, HIGH);
  // Конец использования шины SPI
  SPI.endTransaction();

  return masterReceive;
}

// Чтение группы последовательных регистров
/**
* Чтение регистра
* 
* Читает значение регистра микросхемы с заданным 
* адресом, а затем заданное количество последующих
* регистров, инкрементируя их адреса. 
* 
* @param addr - адрес регистра, значение от 0x00 до 0xFF
* @param count - количество последовательных регистров для чтения
* @param out - буфер, в который записываются результаты чтения
*/
void IMUCommon::readMSBRegister(uint8_t addr, uint8_t count, uint8_t * out) {
  uint8_t masterSend;

  // Добавляем настройки для чтения регистра
  masterSend = addr | IMU_MSB_READ;
  
  // Начало использования шины SPI
  SPI.beginTransaction(SPISettings(KDR_SPI_MAXSPEED, KDR_SPI_FIRSTBIT, KDR_SPI_MODE));
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(masterSend);
  for(int i = 0; i < count; i++) {
    out[i] = SPI.transfer(0x00);
  }

  digitalWrite(chipSelectPin, HIGH);
  // Конец использования шины SPI
  SPI.endTransaction();
}

/**
* Запись в регистр
* 
* Записывает значение в регистр микросхемы с заданным 
* адресом
* 
* @param addr - адрес регистра, значение от 0x00 до 0xFF
* @param value - байт данных, который будет записан в регистр
*/   
void IMUCommon::writeRegister(uint8_t addr, uint8_t value) {
  uint8_t masterSend;

  // Добавляем настройки для чтения регистра
  masterSend = addr | IMU_SIMPLE_WRITE;
  // Начало использования шины SPI
  SPI.beginTransaction(SPISettings(KDR_SPI_MAXSPEED, KDR_SPI_FIRSTBIT, KDR_SPI_MODE));
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(masterSend);
  SPI.transfer(value);

  digitalWrite(chipSelectPin, HIGH);
  // Конец использования шины SPI
  SPI.endTransaction();
}

/**
* Чтение регистра "КТО Я"
* 
* Читает значение регистра WHO_AM_I. Используется для проверки
* корректности подключения к микросхеме.
* 
* @return - индивидуальное для микросхемы значение регистра WHO_AM_I
*/
uint8_t IMUCommon::whoAmI() {
  return readRegister(WHO_AM_I);
}

/**
* Запись в первый контрольный регистр
* 
* Записывает значение в первый контрольный регистр микросхемы.
* Ипользуется для первоначальной настройки и/или смены параметров
* работы микросхемы
*/   
void IMUCommon::writeCTRL1() {
  writeRegister(CTRL_REG1, _valCTL1);
}

/**
* Запись во второй контрольный регистр
* 
* Записывает значение во второй контрольный регистр микросхемы.
* Ипользуется для первоначальной настройки и/или смены параметров
* работы микросхемы
*/
void IMUCommon::writeCTRL2() {
  writeRegister(CTRL_REG2, _valCTL2);
}

/**
* Запись в третий контрольный регистр
* 
* Записывает значение в третий контрольный регистр микросхемы.
* Ипользуется для первоначальной настройки и/или смены параметров
* работы микросхемы
*/
void IMUCommon::writeCTRL3() {
  writeRegister(CTRL_REG3, _valCTL3);
}

/**
* Запись в четвёртый контрольный регистр
* 
* Записывает значение в четвёртый контрольный регистр микросхемы.
* Ипользуется для первоначальной настройки и/или смены параметров
* работы микросхемы
*/
void IMUCommon::writeCTRL4() {
  writeRegister(CTRL_REG4, _valCTL4);
}
