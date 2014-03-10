/* 
 * File:   Timer.hpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de febrero de 2014
 */

#ifndef _TIMER_HPP
#define	_TIMER_HPP

namespace Time {

    void Wait(double t);
    void Wait(float t);
    double Timed();
    float Time();

}

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

