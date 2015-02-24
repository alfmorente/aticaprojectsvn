
/** 
 * @file  Timer.cpp
 * @brief Implementación de la clase "Timer"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "Timer.h"

#include <unistd.h>
#include <sys/time.h>

namespace Time {

  void Wait(double t) {
    if (t >= 0.0)
      usleep((__useconds_t) (t * 1000000));
  }

  void Wait(float t) {
    if (t >= 0.f)
      usleep((__useconds_t) (t * 1000000));
  }

  double Timed() {
    timeval t = {0, 0};
    gettimeofday(&t, NULL);
    return t.tv_sec + (t.tv_usec / 1000000.0);
  }

  float Time() {
    return (float) Timed();
  }
}

/**
 * Constructor de la clase
 */
Timer::Timer() : startTime(0.0) {
  this->Reset();
  this->state = 0;
}

/**
 * Método público que efectúa un reinicio de la cuenta del temportizador
 */
void Timer::Reset() {
  this->startTime = Time::Timed();
}

/**
 * Método público que obtiene el los segundos desde la inicialización del 
 * temporizador
 * @return Número de segundos desde la inicialización del 
 * temporizador
 */
double Timer::GetTimed() {
  if(state==0) return 0;
  return Time::Timed() - this->startTime;
}

/**
 * Método público que habilita el temporizador e inicia la cuenta
 */
void Timer::Enable() {
  this->Reset();
  this->state = 1; // Activo
}

/**
 * Método público que deshabilita el temporizador e inicia la cuenta
 */
void Timer::Disable() {
  this->Reset();
  this->state = 0; // Inactivo
}


