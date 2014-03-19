/* 
 * File:   gps.h
 * Author: atica
 *
 * Created on 19 de marzo de 2014, 13:00
 */

#ifndef GPS_H
#define	GPS_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* GPS_H */

// Datos tramas
int hora, minutos, segundos, decsegundos, gpsquality, numsatellites;
float latitude, longitude;
char dirlatitude, dirlongitude, unit, checksum[2];
float precision, altitude;
float track_true, track_mag, knot_speed, km_speed;
char t_indicator, m_indicator, k_indicator, km_indicator;
// Estructura para almacenar los datos del gps
typedef struct{
    float latitude;
    float longitude;
    float altitude;
    float roll;
    float pitch;
    float yaw;
}insData;

// Variable de continuacion de modulo
bool exitModule, readyToPublish, teachActive, launchTeach;
// Variable donde guardar datos de INS
insData insdata;
// Hilo que controla el teach
TeachThread teachThread;
// Estructura para cola de datos en el hilo de teach
TeachData teachData;
// Frecuencia de muestre del GPS
int freq;

char serial[] = "/dev/ttyUSB0";

ros::Publisher pub_gps;
ros::Publisher pub_errores;
ros::Publisher pub_stream;
int res;


