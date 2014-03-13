/* 
 * File:   Timer.hpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de febrero de 2014
 */

#include "../include/Modulo_Conduccion/Timer.hpp"
#include <unistd.h>
#include <sys/time.h>

namespace Time {

    void Wait(double t) {
        if(t >= 0.0)
            usleep((__useconds_t)(t*1000000));
    }
    void Wait(float t) {
        if(t >= 0.f)
            usleep((__useconds_t)(t*1000000));
    }
    double Timed() {
        timeval t = {0,0};
        gettimeofday(&t,NULL);
        return t.tv_sec + (t.tv_usec/1000000.0);
    }
    float Time() {
        return (float)Timed();
    }
}

Timer::Timer() : startTime(0.0) {
    this->Reset();
    this->state = 0;
}

void Timer::Reset() {
    this->startTime = Time::Timed();
}

double Timer::GetTimed() {
    return Time::Timed()-this->startTime;
}

float Timer::GetTime() {
    return (float)this->GetTimed();
}

int Timer::GetState() {
    return (int)this->state;
}

void Timer::WaitUntil(double t) {
    Time::Wait(t-this->startTime);
}

void Timer::WaitUntil(float t) {
    Time::Wait(t-(float)this->startTime);
}

void Timer::Enable() {
    this->Reset();
    this->state = 1; // Activo
}

void Timer::Disable() {
    this->Reset();
    this->state = 0; // Inactivo
}



