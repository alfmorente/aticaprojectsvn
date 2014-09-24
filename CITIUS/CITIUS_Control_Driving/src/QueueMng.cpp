/* 
 * File:   QueueMng.cpp
 * Author: Carlos Amores
 *
 * Created on 24 de septiembre de 2014
 */

#include "Modulo_GPS/QueueMng.hpp"
#include <queue>
#include <iostream>

using namespace std;

QueueMng::QueueMng(){
    this->mode_active=true;
}

QueueMng::~QueueMng() {
    
}

void QueueMng::DoWork(){
    
}

FrameDriving *QueueMng::informResponse(bool ack, short id_instruction){
    
    if(ack){ // ACK
        
        if(id_instruction == queueMsgdata.front().id_instruction){ // Primer elemento y requerido coinciden
            queueMsgdata.pop();
        }else if(id_instruction > queueMsgdata.front().id_instruction){ // Confirmacion de varios elementos
            while(id_instruction >= queueMsgdata.front().id_instruction){
                queueMsgdata.pop();
            }
        }
        
        return NULL;
        
    } else { // NACK
    
        
    }
}
