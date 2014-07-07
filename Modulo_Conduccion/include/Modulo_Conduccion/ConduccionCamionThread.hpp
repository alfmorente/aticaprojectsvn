/* 
 * File:   ConduccionCamionThread.hpp
 * Author: Sergio Doctor López
 *
 * Created on 2 de Julio de 2014
 */

#ifndef CONDUCCIONCAMIONTHREAD_HPP
#define	CONDUCCIONCAMIONTHREAD_HPP

#include "Thread.hpp"
#include "CANCommunication.hpp"
#include "Timer.hpp"
#include "operaciones.h"
#include "ksm.h"
#include "built_in.h"
#include <queue>

class ConduccionCamionThread : public Thread {
    
public:
    ConduccionCamionThread(CANCommunication * canCOND);
    virtual ~ConduccionCamionThread();
    virtual void DoWork();
    
    // Semáforos de acceso a datos comunes
    pthread_mutex_t ConduccionCamionThread_mutex;
    
    //Timer time1;
    
    bool CONDUCCION_CAMION_ACTIVE; // Flag de estado
    
    void print_message(TPCANRdMsg m);
    
    struct ksmData ksm;
    
private:
    
    CANCommunication * CAN_CAMION_CONDUCCION;
   

    
};

#endif	/* CONDUCCIONTHREAD_HPP */

