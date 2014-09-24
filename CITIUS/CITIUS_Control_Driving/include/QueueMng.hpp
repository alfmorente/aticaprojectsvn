/* 
 * File:   QueueMng.hpp
 * Author: Carlos Amores
 *
 * Created on 24 de septiembre de 2014
 */

#ifndef QUEUEMNG_HPP
#define	QUEUEMNG_HPP

#include "Thread.hpp"
#include "constant.h"
#include <queue>
#include <cstdlib>

class QueueMng : public Thread {
    
public:
    QueueMng();
    virtual ~QueueMng();
    virtual void DoWork();
    
    // Atributo cola de datos
    queue<FrameDriving> queueMsgdata;
    
    // Informe llegada de ACK/NACK
    FrameDriving *informResponse(bool,short);
    
private:
   
};

#endif	/* TEACHTHREAD_HPP */

