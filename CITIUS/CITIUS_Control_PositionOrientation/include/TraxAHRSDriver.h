/* 
 * File:   TraxAHRSDriver.h
 * Author: atica
 *
 * Created on 4 de julio de 2014, 19:26
 */

#ifndef TRAXAHRSDRIVER_H
#define	TRAXAHRSDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* TRAXAHRSDRIVER_H */

typedef struct{
  short positionStatus;
  short orientationStatus;
}MagnetometerInfo;

class TraxAHRSDriver{
private:
  
public:
  TraxAHRSDriver();
  bool connectToDevice();
  void disconnectDevice();
  void configureDevice();
  MagnetometerInfo getData();
};


