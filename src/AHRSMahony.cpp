#include "AHRSMahony.h"

/**
* Конструктор класса
* 
* Инициализирует экземпляр класса для реализации фильтра Махони 
* 
* @param magnetrometerCorrect - включение коррекции с использованием магнетрометра.
*                               При включённой коррекции угол рыскания считается относительно
*                               направления на Север. 0 - коррекция выключена, 1 - включена.
*/
AHRSMahony::AHRSMahony(uint8_t magnetrometerCorrect = 0) {
  _magnetrometerCorrect = magnetrometerCorrect;
}

/**
* Деструктор класса
* 
* Уничтожает экземпляр класса для реализации фильтра Махони 
* и очищает память
*/
AHRSMahony::~AHRSMahony() {
  // Объекты с динамическим выделением памяти отсутствуют
}

/**
* Сброс данных и инициализация фильтра
* 
* Сбрасывает имеющиеся данные и 
* реинициализирует фильтр
*/
void AHRSMahony::init() {
    _q0 = 1.0;
    _q1 = 0;
    _q2 = 0;
    _q3 = 0;
    
    for (uint8_t i = 0; i < sizeof(eInt); i++) {
      eInt[i] = 0.0f;
    }
    
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
void AHRSMahony::update(float ax, float ay, float az, float gx, float gy, float gz, 
                float mx, float my, float mz) {
  float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;
  float qDot0, qDot1, qDot2, qDot3;
  float _period;

  // Вычисление периода с предыдущего измерения
  _period = getPeriod();


	// Вычисление вспомогательных переменных
	float q0q0 = _q0 * _q0;
	float q0q1 = _q0 * _q1;
	float q0q2 = _q0 * _q2;
	float q0q3 = _q0 * _q3;
	float q1q1 = _q1 * _q1;
	float q1q2 = _q1 * _q2;
	float q1q3 = _q1 * _q3;
	float q2q2 = _q2 * _q2;
	float q2q3 = _q2 * _q3;
	float q3q3 = _q3 * _q3;   

  // Проверка на валидность данных акселерометра и магнетрометра
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
	  // Нормализация данных акселерометра
	  norm = sqrtf(ax * ax + ay * ay + az * az);
		norm = 1.0f / norm;
		ax *= norm;
		ay *= norm;
		az *= norm;

		// Нормализация данных магнетрометра
		norm = sqrtf(mx * mx + my * my + mz * mz);

    if (_magnetrometerCorrect) {
		  norm = 1.0f / norm;
    } else {
      norm = 0;
    }

	  mx *= norm;
		my *= norm;
		mz *= norm;
    

		// Сверка направления магнитного поля Земли
		hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
		hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
		bx = sqrtf((hx * hx) + (hy * hy));
		bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

		// Расчет направлений силы тяжести и магнитного поля
		vx = 2.0f * (q1q3 - q0q2);
		vy = 2.0f * (q0q1 + q2q3);
		vz = q0q0 - q1q1 - q2q2 + q3q3;
		wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
		wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
		wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  

		// Вычисление ошибки - произведения между предполагаемым и измеренным направлениями силы тяжести
		ex = (ay * vz - az * vy) + (my * wz - mz * wy);
		ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
		ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

		if (KDR_COEFF_Ki > 0.0f) {
      // Накопление ошибки интегрирования
			eInt[0] += ex;
			eInt[1] += ey;
			eInt[2] += ez;
		} else {
      // Сброс ошибки интегрирования
			eInt[0] = 0.0f;
			eInt[1] = 0.0f;
			eInt[2] = 0.0f;
		}

		// Применение обратной связи
		gx = gx + KDR_COEFF_Kp * ex + KDR_COEFF_Ki * eInt[0];
		gy = gy + KDR_COEFF_Kp * ey + KDR_COEFF_Ki * eInt[1];
		gz = gz + KDR_COEFF_Kp * ez + KDR_COEFF_Ki * eInt[2];

		// Интегрирование скорости изменения кватерниона
		pa = _q1;
		pb = _q2;
		pc = _q3;

    // Применение изменений к кватерниону
		_q0 = _q0 + (-_q1 * gx - _q2 * gy - _q3 * gz) * (0.5f * _period);
		_q1 = pa + (_q0 * gx + pb * gz - pc * gy) * (0.5f * _period);
		_q2 = pb + (_q0 * gy - pa * gz + pc * gx) * (0.5f * _period);
		_q3 = pc + (_q0 * gz + pa * gy - pb * gx) * (0.5f * _period);
   
  } else {
    // Вычисление изменений кватерниона по данным гироскопа
    qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    qDot1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    qDot2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    qDot3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    // Интегрирования показаний кватерниона
    _q0 += qDot0 * _period;
    _q1 += qDot1 * _period;
    _q2 += qDot2 * _period;
    _q3 += qDot3 * _period;
  }

  // Нормализация кватерниона
  norm = sqrtf(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
  norm = 1 / norm;
  _q0 *= norm;
  _q1 *= norm;
  _q2 *= norm;
  _q3 *= norm;
}

/**
* Получение угла тангажа в радианах
*
* Получает угол тангажа и представляет его в радианах.
* Направляющая ось - X.
*
* @return - угол тангажа, выраженный в радианах
*/
float AHRSMahony::getPitchRad() {
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
float AHRSMahony::getRollRad() {
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
float AHRSMahony::getYawRad() {
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
float AHRSMahony::getPitchDeg() {
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
float AHRSMahony::getRollDeg() {
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
float AHRSMahony::getYawDeg() {
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
void AHRSMahony::getQuaternion(float * qW, float * qX, float * qY, float * qZ) {
	*qW = _q0;
	*qX = _q1;
	*qY = _q2;
	*qZ = _q3;
}
