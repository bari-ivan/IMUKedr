// Подключение библиотеки IMUKedr
#include <IMUKedr.h>

// Обозначение пинов, к которым подключены 
// контакты выбора устройства
#define CS1 9
#define CS2 8
#define CS3 7
#define CS4 6

// Создание экземпляра класса управления модулем
IMUKedr kedr(CS1, CS2, CS3, CS4);

void setup() {
  // Инициализация последовательного порта на скорости 115200
  Serial.begin(115200);

  // Инициализация модуля
  kedr.IMUInit();
}

void loop() {
  // Вывод данных акселерометра в м/с^2 относительно оси X, Y и Z
  Serial.print(kedr.accelerometer.readGX(), 4);
  Serial.print("\t");
  Serial.print(kedr.accelerometer.readGY(), 4);
  Serial.print("\t");
  Serial.print(kedr.accelerometer.readGZ(), 4);
  Serial.print("\t");

  // Вывод данных гироскопа в градусах в секунду относительно оси X, Y и Z
  Serial.print(kedr.gyroscope.readDegX(), 4);
  Serial.print("\t");
  Serial.print(kedr.gyroscope.readDegY(), 4);
  Serial.print("\t");
  Serial.print(kedr.gyroscope.readDegZ(), 4);
  Serial.print("\t");

  // Вывод данных магнетрометра в Гауссах без калибровки относительно оси X, Y и Z
  Serial.print(kedr.magnetrometer.readGaussX(), 4);
  Serial.print("\t");
  Serial.print(kedr.magnetrometer.readGaussY(), 4);
  Serial.print("\t");
  Serial.print(kedr.magnetrometer.readGaussZ(), 4);
  Serial.print("\t");

  // Вывод данных барометра в миллиметрах ртутного столба и температуру в градусах по Цельсию 
  Serial.print(kedr.barometer.readPressureMillimetersHg(), 4);
  Serial.print("\t");
  Serial.print(kedr.barometer.readTemperatureC(), 4);
  Serial.print("\t");

  // Задержка 100 мс
  delay(100);
}
