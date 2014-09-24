/* 
 * File:   QueueMng.cpp
 * Author: Carlos Amores
 *
 * Created on 24 de septiembre de 2014
 */

#include "QueueMng.hpp"
#include <queue>
#include <iostream>

using namespace std;

QueueMng::QueueMng(){

}

QueueMng::~QueueMng() {
    
}

void QueueMng::DoWork(){
    
}

FrameDriving *QueueMng::informResponse(bool ack, short id_instruction){
    
    if(ack){ // ACK
        
        if(id_instruction == queueMsgdata.front().id_instruccion){ // Primer elemento y requerido coinciden
            queueMsgdata.pop();
        }else if(id_instruction > queueMsgdata.front().id_instruccion){ // Confirmacion de varios elementos
            while(id_instruction >= queueMsgdata.front().id_instruccion){
                queueMsgdata.pop();
            }
        }
        
        return NULL;
        
    } else { // NACK
    
        
    }
    return NULL;
}
