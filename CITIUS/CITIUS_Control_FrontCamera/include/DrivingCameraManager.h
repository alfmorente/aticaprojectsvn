/* 
 * File:   DrivingCameraManager.h
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 16:10
 */

#ifndef DRIVINGCAMERAMANAGER_H
#define	DRIVINGCAMERAMANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* DRIVINGCAMERAMANAGER_H */

#include "ros/ros.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

class DrivingCameraManager{
private:

public:
    // Constructor
    DrivingCameraManager();
    // Gestion de la camara
    bool connect();
    void setParam(short idParam, float value);
    float getParam(short idParam);
    bool disconnect();

};

