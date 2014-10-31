
/** 
 * @file  Timer.h
 * @brief Declara el tipo de la clase "Timer"
 * - La clase implementa la gestión de un temporizador
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup CommunicationsRosNode
 * @{
 */

#ifndef _TIMER_H
#define	_TIMER_H

namespace Time {
  void Wait(double t);
  void Wait(float t);
  double Timed();
  float Time();
}

/**
 * \class Timer
 * \brief Clase que representa un temporizador utilizado para contear la 
 * frecuencia de de publicación del nodo ROS
 */
class Timer {
private:
  double startTime;
  int state;
public:
  Timer();
  void Reset();
  double GetTimed();
  void Enable();
  void Disable();
};

#endif	/* _TIMER_H */
