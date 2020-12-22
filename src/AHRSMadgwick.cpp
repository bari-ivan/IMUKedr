#include "AHRSMadgwick.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса для реализации фильтра Маджвика 
* 
* @param magnetrometerCorrect - включение коррекции с использованием магнетрометра.
*                               При включённой коррекции угол рыскания считается относительно
*                               направления на Север. 0 - коррекция выключена, 1 - включена.
* @param betaErr - регулярная ошибка показаний угловой скорости гироскопа в радианах.
* @param zetaErr - регулярный дрейф нуля гироскопа в радианах.
*/
AHRSMadgwick::AHRSMadgwick(uint8_t magnetrometerCorrect = 0, float betaErr = KDR_F_GYRO_MEAS_ERROR, float zetaErr = KDR_F_GYRO_MEAS_DRIFT) {
  _magnetrometerCorrect = magnetrometerCorrect;
  _beta = betaErr * KDR_F_COEFF_MULT;
  _zeta = zetaErr * KDR_F_COEFF_MULT;
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса для реализации фильтра Маджвика 
* и очищает память
*/
AHRSMadgwick::~AHRSMadgwick() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Сброс данных и инициализация фильтра
* 
* Сбрасывает имеющиеся данные и 
* реинициализирует фильтр
*/
void AHRSMadgwick::init() {
    _q0 = 1.0;
    _q1 = 0;
    _q2 = 0;
    _q3 = 0;
    _previousCalculateMicros = 0;
}

/**
* Обновление данных фильтра
* 
* Выполняет одиночную итерацию обновления данных фильтра. Как правило,
* чем чаще происходит выхов этого метода, тем точнее данные будут на 
* выходе у фильтра.
* 
* @param ax - данные измерения акселерометра по оси X
* @param ay - данные измерения акселерометра по оси Y
* @param az - данные измерения акселерометра по оси Z
* @param gx - данные измерения гироскопа по оси X
* @param gy - данные измерения гироскопа по оси Y
* @param gz - данные измерения гироскопа по оси Z
* @param mx - данные измерения магнетрометра по оси X
* @param my - данные измерения магнетрометра по оси Y
* @param mz - данные измерения магнетрометра по оси Z
*/
void AHRSMadgwick::update(float ax, float ay, float az, float gx, float gy, float gz, 
                float mx, float my, float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot0, qDot1, qDot2, qDot3;
  float hx, hy;
  float _period;

  // Вычисление периода с предыдущего измерения
  _period = getPeriod();

  // Вычисление изменений кватерниона по данным гироскопа
  qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
  qDot1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
  qDot2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
  qDot3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

  // Проверка на валидность данных акселерометра
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Нормализация данных акселерометра
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    recipNorm = 1 / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // В зависимости от необходимости коррекции по магнетрометру
    if (_magnetrometerCorrect) {
      // Объявление вспомогательных переменных
      float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
      _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
      q2q2, q2q3, q3q3;

      // Нормализация данных магнетрометра
      recipNorm = sqrtf(mx * mx + my * my + mz * mz);
      recipNorm = 1 / recipNorm;
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Вычисление вспомогательных переменных
      _2q0mx = 2.0f * _q0 * mx;
      _2q0my = 2.0f * _q0 * my;
      _2q0mz = 2.0f * _q0 * mz;
      _2q1mx = 2.0f * _q1 * mx;
      _2q0 = 2.0f * _q0;
      _2q1 = 2.0f * _q1;
      _2q2 = 2.0f * _q2;
      _2q3 = 2.0f * _q3;
      _2q0q2 = 2.0f * _q0 * _q2;
      _2q2q3 = 2.0f * _q2 * _q3;
      q0q0 = _q0 * _q0;
      q0q1 = _q0 * _q1;
      q0q2 = _q0 * _q2;
      q0q3 = _q0 * _q3;
      q1q1 = _q1 * _q1;
      q1q2 = _q1 * _q2;
      q1q3 = _q1 * _q3;
      q2q2 = _q2 * _q2;
      q2q3 = _q2 * _q3;
      q3q3 = _q3 * _q3;

      // Контроль направления магнитного поля Земли
      hx = mx * q0q0 - _2q0my * _q3 + _2q0mz * _q2 + mx * q1q1
           + _2q1 * my * _q2 + _2q1 * mz * _q3 - mx * q2q2 - mx * q3q3;
      hy = _2q0mx * _q3 + my * q0q0 - _2q0mz * _q1 + _2q1mx * _q2 - my * q1q1
           + my * q2q2 + _2q2 * mz * _q3 - my * q3q3;
      _2bx = sqrt(hx * hx + hy * hy);
      _2bz = -_2q0mx * _q2 + _2q0my * _q1 + mz * q0q0 + _2q1mx * _q3
             - mz * q1q1 + _2q2 * my * _q3 - mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;

      // Итерация алгоритма градиентного спуска
      s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
           - _2bz * _q2
                * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * _q3 + _2bz * _q1)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * _q2
                * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
           - 4.0f * _q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
           + _2bz * _q3
                * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (_2bx * _q2 + _2bz * _q0)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + (_2bx * _q3 - _4bz * _q1)
                * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
           - 4.0f * _q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
           + (-_4bx * _q2 - _2bz * _q0)
                * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (_2bx * _q1 + _2bz * _q3)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + (_2bx * _q0 - _4bz * _q2)
                * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
           + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
           + (-_4bx * _q3 + _2bz * _q1)
                * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * _q0 + _2bz * _q2)
                * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * _q1
                * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    } else {
      // Объявление вспомогательных переменных
      float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1,
      q2q2, q3q3;

      // Вычисление вспомогательных переменных
      _2q0 = 2.0f * _q0;
      _2q1 = 2.0f * _q1;
      _2q2 = 2.0f * _q2;
      _2q3 = 2.0f * _q3;
      _4q0 = 4.0f * _q0;
      _4q1 = 4.0f * _q1;
      _4q2 = 4.0f * _q2;
      _8q1 = 8.0f * _q1;
      _8q2 = 8.0f * _q2;
      q0q0 = _q0 * _q0;
      q1q1 = _q1 * _q1;
      q2q2 = _q2 * _q2;
      q3q3 = _q3 * _q3;

      // Итерация алгоритма градиентного спуска
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1
           + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
           + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
    }

    // Нормализация магнитуды итерации
    recipNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    recipNorm = 1 / recipNorm;
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Применение коэффициента постоянной ошибки
    qDot0 -= _beta * s0;
    qDot1 -= _beta * s1;
    qDot2 -= _beta * s2;
    qDot3 -= _beta * s3;
  }

  // Интегрирования показаний кватерниона
  _q0 += qDot0 * _period;
  _q1 += qDot1 * _period;
  _q2 += qDot2 * _period;
  _q3 += qDot3 * _period;

  // Нормализация кватерниона
  recipNorm = sqrtf(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
  recipNorm = 1 / recipNorm;
  _q0 *= recipNorm;
  _q1 *= recipNorm;
  _q2 *= recipNorm;
  _q3 *= recipNorm;
}

/**
* Получение угла тангажа в радианах
*
* Получает угол тангажа и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол тангажа, выраженный в радианах
*/
float AHRSMadgwick::getPitchRad() {
  return -1 * atan2(2.0f * (_q0 * _q2 - _q1 * _q3),
                    1.0f - 2.0f * (_q2 * _q2 + _q1 * _q1));
}

/**
* Получение угла крена в радианах
*
* Получает угол крена и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол крена, выраженный в радианах
*/
float AHRSMadgwick::getRollRad() {
  return atan2(2 * _q2 * _q3 - 2 * _q0 * _q1,
                 2 * _q0 * _q0 + 2 * _q3 * _q3 - 1);
}

/**
* Получение угла рыскания в радианах
*
* Получает угол рыскания и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол рыскания, выраженный в радианах
*/
float AHRSMadgwick::getYawRad() {
  return atan2(2 * _q1 * _q2 - 2 * _q0 * _q3,
                 2 * _q0 * _q0 + 2 * _q1 * _q1 - 1);
}

/**
* Получение угла тангажа в градусах
*
* Получает угол тангажа и представляет его в градусах.
* Направляющая ось - X.
*
* @return - угол тангажа, выраженный в градусах
*/
float AHRSMadgwick::getPitchDeg() {
  return getPitchRad() * KDR_CONST_RAD_TO_DEG;
}

/**
* Получение угла крена в градусах
*
* Получает угол крена и представляет его в градусах.
* Направляющая ось - X.
*
* @return - угол крена, выраженный в градусах
*/
float AHRSMadgwick::getRollDeg() {
  return getRollRad() * KDR_CONST_RAD_TO_DEG;
}

/**
* Получение угла рыскания в градусах
*
* Получает угол рыскания и представляет его в градусах.
* Направляющая ось - X.
*
* @return - угол рыскания, выраженный в градусах
*/
float AHRSMadgwick::getYawDeg() {
  return getYawRad() * KDR_CONST_RAD_TO_DEG;
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
void AHRSMadgwick::getQuaternion(float * qW, float * qX, float * qY, float * qZ) {
	*qW = _q0;
	*qX = _q1;
	*qY = _q2;
	*qZ = _q3;
}
