#include "IMUKedr.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр основного класса для управления 
* инерциально-измерительным модулем
*/
IMUKedr::IMUKedr(uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t CS4) {
  SPI.begin();

  _biassgx = 0.0;
  _biassgy = 0.0;
  _biassgz = 0.0;

  _activeAHRS = 0;
  _activeINS = 0;
  _calibrateCompass = 0;
  
  accelerometer = LIS331DLH(CS1);
  gyroscope = L3G4200D(CS2);
  magnetrometer = LIS3MDL(CS3);
  barometer = LPS25HB(CS4);
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр основного класса для управления 
* инерциально-измерительным модулем и очищает память
*/
IMUKedr::~IMUKedr() {
  delete(ahrs);
}

/**
* Инициализация пользовательских параметров работы модуля
* 
* Инициализирует инерциально-измерительный модуль и подготавливает
* его к работе с заданными параметрами.
*
* ВНИМАНИЕ! Увеличение скорости может отрицательно влиять на защищенность
* получаемых данных от высокочастотных помех.
*
* @param mode - определяет используемый при работе математический аппарат
* @param speed - определяет скорость обновления данных микросхемы
*/
void IMUKedr::IMUInit(uint8_t mode = KDR_IMU_MADGWICK_MAG, uint8_t dspeed = KDR_IMU_NORMAL) {
  // Установка типа фильтра
  switch (mode)
  {
#ifndef KDR_LIGHT
  case KDR_IMU_MAHONY_MAG:
    ahrs = new AHRSMahony(HIGH);
    break;

  case KDR_IMU_MAHONY:
    ahrs = new AHRSMahony(LOW);
    break;
#endif

  case KDR_IMU_MADGWICK_MAG:
    ahrs = new AHRSMadgwick(HIGH);
    break;

  case KDR_IMU_MADGWICK:
    ahrs = new AHRSMadgwick(LOW);
    break;
  
  default:
    ahrs = new AHRSMadgwick(HIGH);
    break;
  }

  // Установка параметров ИНС
  ins = INSIntegrator();

  // Установка скорости работы датчиков
  switch (dspeed)
  {
  case KDR_IMU_LOW_ENERGY:
    accelerometer.setMode(KDR_DR_5_Hz);
    gyroscope.setMode(KDR_G_DR_NORMAL);
    magnetrometer.setMode(KDR_M_DR_2_5_UHP);
    barometer.setMode(KDR_B_DR_1_Hz);
    break;

  case KDR_IMU_NORMAL:
    accelerometer.setMode(KDR_DR_NORMAL);
    gyroscope.setMode(KDR_G_DR_200Hz_CO_12_5Hz);
    magnetrometer.setMode(KDR_M_DR_NORMAL);
    barometer.setMode(KDR_B_DR_NORMAL);
    break;

  case KDR_IMU_FAST:
    accelerometer.setMode(KDR_DR_400_Hz);
    gyroscope.setMode(KDR_G_DR_400Hz_CO_20Hz);
    magnetrometer.setMode(KDR_M_DR_155_UHP);
    barometer.setMode(KDR_B_DR_12_5_Hz);
    break;

  case KDR_IMU_MAXFAST:
    accelerometer.setMode(KDR_DR_1000_Hz);
    gyroscope.setMode(KDR_G_DR_800Hz_CO_30Hz);
    magnetrometer.setMode(KDR_M_DR_1000_LP);
    barometer.setMode(KDR_B_DR_25_Hz);
    break;
  
  default:
    accelerometer.setMode(KDR_DR_NORMAL);
    gyroscope.setMode(KDR_G_DR_NORMAL);
    magnetrometer.setMode(KDR_M_DR_NORMAL);
    barometer.setMode(KDR_B_DR_NORMAL);
    break;
  }
}

/**
* Инициализация курсовертикали
* 
* Включает модуль в режим курсовертикали и
* обнуляет начальные значения ориентации
*/
void IMUKedr::AHRSInit() {
  ahrs->init();
  _activeAHRS = 1;
}

/**
* Инициализация инерциальной навигационной системы
* 
* Включает модуль в режим обработки инерциальных данных и
* обнуляет начальные значения положения в пространстве
*
* ВНИМАНИЕ! Для корректной работы данного режима в модуль
* должен быть переведён в режим курсовертикали.
*/
void IMUKedr::INSInit() {
  ins.init(barometer.readPressurePascals(), barometer.readTemperatureK());
  _activeINS = 1;
}

/**
* Ожидание стабилизации курсовертикали в течение заданного времени
* 
* Непрерывно обновляет данные курсовертикали в течение заданного промежутка времени
* для коррекции начальных скачков показателей датчиков.
*
* ВНИМАНИЕ! Для корректной работы данного режима в модуль
* должен быть переведён в режим курсовертикали. 
*
* @param seconds - время проведения стабилизации в секундах. Рекомендуемое - 2 секунды.
*/
void IMUKedr::AHRSStabilise(uint8_t seconds) {
  uint16_t start;

  // Сброс параметров инерциальной навигационной системы
  INSInit();

  // Фиксация времени начала операции
  start = millis();

  // Непрерывное обновление данных
  while (millis() < (start + seconds * 1000))
  {
    IMUUpdate();
  }

  // Сброс параметров инерциальной навигационной системы
  INSInit();
}

/**
* Установка матрицы калибровок магнетрометра
* 
* Устанавливает параметры калибровки выходных данных магнетрометра.
* 
* @param calibrationMatrix - матрица масштаба и ортогонализации
* @param bias - смещения по осям
*/
void IMUKedr::setMagnetrometerCalibration(const double calibrationMatrix[3][3], const double bias[3]) {
  magnetrometer.setCalibrationMatrix(calibrationMatrix, bias);
  _calibrateCompass = 1;
}

/**
* Установка калибровок гироскопа
* 
* Устанавливает параметры смещения выходных данных гироскопа.
* Позволяет немного скорректировать дрейф нуля.
* 
* @param biassgx - смещение по оси X
* @param biassgy - смещение по оси Y
* @param biassgz - смещение по оси Z
*/
void IMUKedr::setGyroscopeCalibration(float biassgx, float biassgy, float biassgz) {
  _biassgx = biassgx;
  _biassgy = biassgy;
  _biassgz = biassgz;
}

/**
* Обновление данных модуля
* 
* Выполняет одиночную итерацию обновления данных модуля. Как правило,
* чем чаще происходит выхов этого метода, тем точнее данные будут на 
* выходе у модуля.
*/
void IMUKedr::IMUUpdate() {
  float q0, q1, q2, q3;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float bp, bt;

  // Получение данных
  accelerometer.readGXYZ(&ax, &ay, &az);
  gyroscope.readRadXYZ(&gx, &gy, &gz);

  if (_calibrateCompass) {
    magnetrometer.readGaussXYZ(&mx, &my, &mz);
  } else {
    magnetrometer.readCalibrateGaussXYZ(&mx, &my, &mz);
  }

  bp = barometer.readPressurePascals();
  bt = barometer.readTemperatureK();

  // Обработка данных
  ahrs->update(ax, ay, az, gx, gy, gz, mx, my, mz);
  ahrs->getQuaternion(&q0, &q1, &q2, &q3);
  ins.update(ax, ay, az, bp, bt, q0, q1, q2, q3);
}


/**
* Получение высоты модуля относительно точки последней инициализации
* 
* Получает разницу высот между точкой последней 
* инициализации и текущим положением модуля
*
* @return - высота, выраженная в метрах
*/
float IMUKedr::getAltitude() {
  return ins.getAltitude();
}

/**
* Получение угла тангажа в радианах
*
* Получает угол тангажа и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол тангажа, выраженный в радианах
*/
float IMUKedr::getPitchRad() {
  return ahrs->getPitchRad();
}

/**
* Получение угла крена в радианах
*
* Получает угол крена и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол крена, выраженный в радианах
*/
float IMUKedr::getRollRad() {
  return ahrs->getRollRad();
}

/**
* Получение угла рыскания в радианах
*
* Получает угол рыскания и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол рыскания, выраженный в радианах
*/
float IMUKedr::getYawRad() {
  return ahrs->getYawRad();
}

/**
* Получение угла тангажа в градусах
*
* Получает угол тангажа и представляет его в градусах.
* Направляющая ось - X.
*
* @return - угол тангажа, выраженный в градусах
*/
float IMUKedr::getPitchDeg() {
  return ahrs->getPitchDeg();
}

/**
* Получение угла крена в градусах
*
* Получает угол крена и представляет его в градусах.
* Направляющая ось - X.
*
* @return - угол крена, выраженный в градусах
*/
float IMUKedr::getRollDeg() {
  return ahrs->getRollDeg();
}

/**
* Получение угла рыскания в градусах
*
* Получает угол рыскания и представляет его в градусах.
* Направляющая ось - X.
*
* @return - угол рыскания, выраженный в градусах
*/
float IMUKedr::getYawDeg() {
  return ahrs->getYawDeg();
}

/**
* Получение положения в виде кватерниона
* 
* Возвращает положение модуля в виде кватерниона.
* 
* @param qW - первая составляющая кватерниона
* @param qX - вторая составляющая кватерниона
* @param qY - третья составляющая кватерниона
* @param qZ - четвертая составляющая кватерниона
*/
void IMUKedr::getQuaternion(float * qW, float * qX, float * qY, float * qZ) {
  ahrs->getQuaternion(qW, qX, qY, qZ);
}

/**
* Получение собственных ускорений модуля без учета притяжения Земли
* 
* На основании данных о пространственной ориентации вычитает вектор ускорения
* свободного падения из вектора ускорения модуля и присваивает ему значения
* собственного ускорения
*
* @param cax - собственное ускорение модуля по оси X
* @param cay - собственное ускорение модуля по оси Y
* @param caz - собственное ускорение модуля по оси Z
*/
void IMUKedr::getClearAcceleration(float * cax, float * cay, float * caz) {
  float q0, q1, q2, q3;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float bp, bt;

  // Получение данных
  accelerometer.readGXYZ(&ax, &ay, &az);
  gyroscope.readRadXYZ(&gx, &gy, &gz);

  if (_calibrateCompass) {
    magnetrometer.readGaussXYZ(&mx, &my, &mz);
  } else {
    magnetrometer.readCalibrateGaussXYZ(&mx, &my, &mz);
  }

  bp = barometer.readPressurePascals();
  bt = barometer.readTemperatureK();

  // Обработка данных
  ahrs->update(ax, ay, az, gx, gy, gz, mx, my, mz);
  ahrs->getQuaternion(&q0, &q1, &q2, &q3);
  ins.earthAccelerationCorrect(&ax, &ay, &az, q0, q1, q2, q3);

  *cax = ax;
  *cay = ay;
  *caz = az;
}

/**
* Получение собственных геопространственных ускорений модуля без учета притяжения Земли
* 
* На основании данных о пространственной ориентации вычитает вектор ускорения
* свободного падения из вектора ускорения модуля и присваивает ему значения
* собственного ускорения, после чего пересчитывает вектор ускорения
* из системы координат относительно модуля в систему координат относительно 
* поверхности Земли
*
* @param cgax - ускорение модуля по оси X относительно поверхности Земли
* @param cgay - ускорение модуля по оси Y относительно поверхности Земли
* @param cgaz - ускорение модуля по оси Z относительно поверхности Земли
*/
void IMUKedr::getClearGeoAcceleration(float * cgax, float * cgay, float * cgaz) {
  float q0, q1, q2, q3;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float bp, bt;

  // Получение данных
  accelerometer.readGXYZ(&ax, &ay, &az);
  gyroscope.readRadXYZ(&gx, &gy, &gz);

  if (_calibrateCompass) {
    magnetrometer.readGaussXYZ(&mx, &my, &mz);
  } else {
    magnetrometer.readCalibrateGaussXYZ(&mx, &my, &mz);
  }

  bp = barometer.readPressurePascals();
  bt = barometer.readTemperatureK();

  // Обработка данных
  ahrs->update(ax, ay, az, gx, gy, gz, mx, my, mz);
  ahrs->getQuaternion(&q0, &q1, &q2, &q3);
  ins.earthAccelerationCorrect(&ax, &ay, &az, q0, q1, q2, q3);
  ins.accelerationToGeo(&ax, &ay, &az, q0, q1, q2, q3);

  *cgax = ax;
  *cgay = ay;
  *cgaz = az;
}
