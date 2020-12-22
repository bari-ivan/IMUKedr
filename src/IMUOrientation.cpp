#include "IMUOrientation.h"


/**
* Конструктор класса
* 
* Инициализирует экземпляр класса для реализации пространственной
* ориентации
*/
IMUOrientation::IMUOrientation() {
  _previousCalculateMicros = 0;
}

/**
* Деструктор класса 
* 
* Уничтожает экземпляр класса для реализации пространственной
* ориентации 
*/
IMUOrientation::~IMUOrientation() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Вычисление периода с момента предыдущего измерения в секундах
*
* Вычисляет период, прошедший с предыдущего вызова этой функции и представляет
*  его в секундах.
*
* @return - период, прошедший с предыдущего вызова функции в секундах
*/
float IMUOrientation::getPeriod() {
  uint32_t now;
  uint32_t pc;

  // Поучение текущего значения врмени
  now = micros();
  pc = _previousCalculateMicros;
  _previousCalculateMicros = now;
  

  // Проверка на переполнение
  if (now >= pc) {
    // Возврат значения без переполнения
    return (now - pc) / 1000000.0;
  } else {
    // Возврат значения с переполнением
    return (KDR_ULONG_MAXVALUE - pc + now) / 1000000.0;
  }
}