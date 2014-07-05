
/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

#include "TraxAHRSDriver.h"


TraxAHRSDriver::TraxAHRSDriver(){}

bool TraxAHRSDriver::connectToDevice(){
    return true;
}

void TraxAHRSDriver::disconnectDevice(){

}

void TraxAHRSDriver::configureDevice(){

}

MagnetometerInfo TraxAHRSDriver::getData(){
    MagnetometerInfo info;
    info.orientationStatus = 2;
    return info;
}

