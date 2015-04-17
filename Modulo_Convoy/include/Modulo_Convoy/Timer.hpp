/** 
 * @file  Timer.hpp
 * @brief Declara el tipo de la clase "Timer"
 * - La clase implementa la gestión de un temporizador
 * @author Sergio Doctor
 * @date 05/02/2014
 */

#ifndef _TIMER_HPP
#define	_TIMER_HPP

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
    double startTime;
public:
    Timer();

    int state;      // ON/OFF

    void Reset();
    double GetTimed();
    float GetTime();
    int GetState();
    void Enable();
    void Disable();

    void WaitUntil(double t);
    void WaitUntil(float t);
};


#endif	/* _TIMER_HPP */

