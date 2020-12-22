#include "INSIntegrator.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса для реализации работы модуля
* в режиме ИНС
*/
INSIntegrator::INSIntegrator() {
  accG = 1;
  _comlimentaryCoeff = KDR_INS_COMLIMENTARY_COEFF_DEFAULT;
}

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса для реализации работы модуля
* в режиме ИНС
*
* @param comlimentaryCoeff - коэффициент комплиментарного фильтра
*/
INSIntegrator::INSIntegrator(float comlimentaryCoeff) {
  accG = 1;
  _comlimentaryCoeff = comlimentaryCoeff;
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса для реализации работы модуля
* в режиме ИНС и очищает память
*/
INSIntegrator::~INSIntegrator() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Сброс данных и инициализация интегратора
* 
* Сбрасывает информацию о скорости и местоположении и
* начинает отсчёт заново
*/
void INSIntegrator::init(float P0, float T0) {
  initP0 = P0;
  initT0 = T0;
  _verticalVelocity = 0;
  _altitude = 0;
  _deltaP = 0;
  _deltaI = 0;
  _deltaD = 0;
}

/**
* Обновление данных системы
* 
* Выполняет одиночную итерацию обновления данных системы. Как правило,
* чем чаще происходит выхов этого метода, тем точнее данные будут на 
* выходе у системы.
*
* @param ax - показания акселерометра по оси X
* @param ay - показания акселерометра по оси Y
* @param az - показания акселерометра по оси Z
* @param bp - атмосферное давление в Паскалях
* @param bt - температура в Кельвинах
* @param qW - кватернион 0
* @param qX - кватернион 1
* @param qY - кватернион 2
* @param qZ - кватернион 3
*/
void INSIntegrator::update(float ax, float ay, float az, float bp, float bt, float qW, float qX, float qY, float qZ) {
  // Получение периода измерений
  float _period = getPeriod();

  // Вычисление собственных ускорений модуля
  earthAccelerationCorrect(&ax, &ay, &az, qW, qX, qY, qZ);

  // Перевод в геопространственную систему координат
  accelerationToGeo(&ax, &ay, &az, qW, qX, qY, qZ); 

  // Вычисление высоты и вертикальной скорости
  altitudeComplimentary(&_altitude, &_verticalVelocity, baroHDelta(bp, bt), az, _period);
}

/**
* Вычитание компонентов ускорения свободного падения из вектора ускорения модуля
* 
* На основании данных о пространственной ориентации вычитает вектор ускорения
* свободного падения из вектора ускорения модуля и присваивает ему значения
* собственного ускорения
*
* @param ax - показания акселерометра по оси X
* @param ay - показания акселерометра по оси Y
* @param az - показания акселерометра по оси Z
* @param qW - кватернион 0
* @param qX - кватернион 1
* @param qY - кватернион 2
* @param qZ - кватернион 3
*/
void INSIntegrator::earthAccelerationCorrect(float *ax, float *ay, float *az, 
                                             float qW, float qX, float qY, float qZ) {
  float vqW, vqX, vqY, vqZ;
  float GqW, GqX, GqY, GqZ;
  float gravityX, gravityY, gravityZ;

  // Инверсия кватерниона
  vqW = qW;
  vqX = - qX;
  vqY = - qY;
  vqZ = - qZ;

  // Определение начального вектора силы тяжести
  gravityX = 0;
  gravityY = 0;
  gravityZ = accG;

  // Поворот вектора обратным кватернионом
  GqW = -vqX * gravityX - vqY * gravityY - vqZ * gravityZ;
  GqX = vqW * gravityX + vqY * gravityZ - vqZ * gravityY;
  GqY = vqW * gravityY - vqX * gravityZ + vqZ * gravityX;
  GqZ = vqW * gravityZ + vqX * gravityY - vqY * gravityX;

  gravityX = GqW * qX + GqX * qW + GqY * qZ - GqZ * qY;
  gravityY = GqW * qY - GqX * qZ + GqY * qW + GqZ * qX;
  gravityZ = GqW * qZ + GqX * qY - GqY * qX + GqZ * qW;

  // Вычитание составляющих ускорения свободного падения из показателей акселерометра
  *ax -= gravityX;
  *ay -= gravityY;
  *az -= gravityZ;
   
}

/**
* Перевод собственных ускорений модуля в систему координат относительно Земли
* 
* На основании данных о пространственной ориентации пересчитывает вектор ускорения
* из системы координат относительно модуля в систему координат относительно поверхности
* Земли
*
* @param ax - показания акселерометра по оси X
* @param ay - показания акселерометра по оси Y
* @param az - показания акселерометра по оси Z
* @param qW - кватернион 0
* @param qX - кватернион 1
* @param qY - кватернион 2
* @param qZ - кватернион 3
*/
void INSIntegrator::accelerationToGeo(float *ax, float *ay, float *az, 
                                      float qW, float qX, float qY, float qZ) {
  float vqW, vqX, vqY, vqZ;
  float AqW, AqX, AqY, AqZ;
  float accelerationX, accelerationY, accelerationZ;

  // Инверсия кватерниона
  vqW = qW;
  vqX = - qX;
  vqY = - qY;
  vqZ = - qZ;

  // Определение начального вектора ускорения модуля
  accelerationX = *ax;
  accelerationY = *ay;
  accelerationZ = *az;

  // Поворот вектора кватернионом
  AqW = -qX * accelerationX - qY * accelerationY - qZ * accelerationZ;
  AqX = qW * accelerationX + qY * accelerationZ - qZ * accelerationY;
  AqY = qW * accelerationY - qX * accelerationZ + qZ * accelerationX;
  AqZ = qW * accelerationZ + qX * accelerationY - qY * accelerationX;

  accelerationX = AqW * vqX + AqX * vqW + AqY * vqZ - AqZ * vqY;
  accelerationY = AqW * vqY - AqX * vqZ + AqY * vqW + AqZ * vqX;
  accelerationZ = AqW * vqZ + AqX * vqY - AqY * vqX + AqZ * vqW;

  // Возврат итоговых значений
  *ax = accelerationX;
  *ay = accelerationY;
  *az = accelerationZ;
}

/**
* Вычисление высоты и вертикальной скорости с использованием комплиментарного фильтра
* 
* Вычисляет высоту и вертикальную скорость, используя данные о высоте на основании
* изменения атмосферного давления, сочетая их с данными, полученными посредством
* интегрирования вертикального ускорения посредством комплиментарного фильтра
*
* @param H - ссылка на переменную, в которой хранятся данные о высоте
* @param Vz - ссылка на переменную, в которой хранятся данные о вертикальной скорости
* @param az - показания акселерометра по оси Z
* @param baroH - данные о высоте, полученные на основании показаний барометра
* @param Az - вертикальная составляющая ускорения модуля
* @param period - период, прошедший с предыдущего вычисления
*/
void INSIntegrator::altitudeComplimentary(float * H, float * Vz, float baroH, float Az, float period) {

  // Вычисление результирующей высоты
  *H = (1 - _comlimentaryCoeff) * (*H + *Vz * period) + _comlimentaryCoeff * baroH;

}

/**
* Вычисление разницы высот по показателям барометра
* 
* Вычисляет изменение высоты относительно точки инициализации ИНС на основании
* изменения атмосферного давления, измеренного барометром.
*
* @param baroP - текущее значение атмосферного давления в Паскалях
*/
float INSIntegrator::baroHDelta(float baroP) {
  float delta;

  // Вычисление высоты из барометрической формулы
  delta = 18400 * (1 + KDR_CONST_AIR_A * initT0) * log(initP0 / baroP);

  return delta / 5;
}
    
/**
* Вычисление разницы высот по показателям барометра с учётом изменений температуры
* 
* Вычисляет изменение высоты относительно точки инициализации ИНС на основании
* изменения атмосферного давления, измеренного барометром. При этом учитывается
* разница температур между двумя точками.
*
* Данный метод не рекомендуется использовать при высокой вероятности нагрева модуля
* во время работы из-за искажений показаний температуры.
*
* @param baroP - текущее значение атмосферного давления в Паскалях
* @param baroT - текущее значение температуры в градусах по Кельвину
*/
float INSIntegrator::baroHDelta(float baroP, float baroT) {
  float delta;
  float T = (baroT + initT0) / 2;

  // Вычисление высоты из барометрической формулы
  delta = 18400 * (1 + KDR_CONST_AIR_A * initT0) * log(initP0 / baroP);

  return delta / 5;
}

// Получение высоты относительно точки инициализации
float INSIntegrator::getAltitude() {
  return _altitude;
}
