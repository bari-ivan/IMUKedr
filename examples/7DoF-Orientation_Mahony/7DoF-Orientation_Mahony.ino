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

// Переменная для отсчётов времени
uint32_t mil;

void setup() {
  // Инициализация последовательного порта на скорости 115200
  Serial.begin(115200);

  // Инициализация модуля
  kedr.IMUInit(KDR_IMU_MAHONY, KDR_IMU_FAST);
  // Инициализация курсовертикали
  kedr.AHRSInit();
  // Стабилизация курсовертикали в течение 2 секунд (рекомендуется)
  kedr.AHRSStabilise(2);
  // Инициализация инерциальной навигации
  kedr.INSInit();

  // Начало временных отсчётов
  mil = millis();
}

void loop() {
  // Непрерывное обновление данных
  kedr.IMUUpdate();

  // Каждые 100 мс выводим данные
  if (millis() - mil >= 100) {

    // Вывод углов крена, тангажа и рыскания в градусах
    Serial.print(kedr.getPitchDeg(), 4);
    Serial.print("\t");
    Serial.print(kedr.getRollDeg(), 4);
    Serial.print("\t");
    Serial.print(kedr.getYawDeg(), 4);
    Serial.print("\t");

    // Вывод углов крена, тангажа и рыскания в радианах
    Serial.print(kedr.getPitchRad(), 4);
    Serial.print("\t");
    Serial.print(kedr.getRollRad(), 4);
    Serial.print("\t");
    Serial.print(kedr.getYawRad(), 4);

    // Вывод высоты относительно точки инициализации в метрах
    Serial.print(kedr.getAltitude(), 4);
    Serial.print("\t");

    // Получение положения в виде кватерниона
    float qW, qX, qY, qZ;
    kedr.getQuaternion(&qW, &qX, &qY, &qZ);
    
    Serial.print(qW, 4);
    Serial.print("\t");
    Serial.print(qX, 4);
    Serial.print("\t");
    Serial.print(qY, 4);
    Serial.print("\t");
    Serial.print(qZ, 4);
    Serial.print("\t");

    // Получение собственных ускорений модуля без учета притяжения Земли
    float cax, cay, caz;
    kedr.getClearAcceleration(&cax, &cay, &caz);

    Serial.print(cax, 4);
    Serial.print("\t");
    Serial.print(cay, 4);
    Serial.print("\t");
    Serial.print(caz, 4);
    Serial.print("\t");

    // Получение собственных геопространственных ускорений модуля без учета притяжения Земли
    float cgax, cgay, cgaz;
    kedr.getClearGeoAcceleration(&cgax, &cgay, &cgaz);

    Serial.print(cgax, 4);
    Serial.print("\t");
    Serial.print(cgay, 4);
    Serial.print("\t");
    Serial.print(cgaz, 4);
    Serial.print("\t");

    Serial.println();

    mil = millis();
    
  } 

}
