/**
 * @file   Seguimiento.h
 * @brief  Fichero de cabecera para la gestion del seguimiento
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */

#ifndef SEGUIMIENTO_H
#define	SEGUIMIENTO_H


//Mensajes
//#include "Modulo_Navegacion/ll-utm_constants.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_gps.h"
#include "Common_files/msg_laser.h"
#include "Common_files/msg_mode.h"
#include "Common_files/msg_waypoint.h"
#include "Common_files/msg_stream.h"
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//ROS y demas librerias
#include "ros/ros.h"
#include "../../../../Common_files/include/Common_files/constant.h"
#include <queue>
#include <iostream>
#include <vector>
#include <pthread.h>
#include "AnalisisFichero.h"

using namespace std;

#define NO_GOAL 0
#define GOAL_ACTIVE 1
#define GOAL_REACHED 2
#define GOAL_CANCELED 3

#define X_OFFSET 0
#define Y_OFFSET 0
#define Z_OFFSET 0
#define YAW_OFFSET 0
#define HEADER_FRAME "map"
#define CHILD_FRAME "/robot_0/odom"

#define RADIO 3
#define PI 3.14159265359


/**
 * \struct SensorGPS
 * \brief  Estructura con los datos recibidos por el GPS
 */
//Estructura que contiene los datos de los sensores
struct SensorGPS{
    double lat;
    double lon;
    float alt;
    float pitch;
    float yaw;
    float roll;
};

/**
 * \struct Transform
 * \brief  Estructura con los datos del vector de transformacion
 */
//Estructura que contiene los datos de los sensores
struct Transform{
    string head;
    string child;  
    float x_offset;
    float y_offset;
    float z_offset;
    float yaw_offset;
};

/**
 * \struct geograficPose
 * \brief  Estructura con los datos de la posicion geografica
 */
//Estructura que contiene los datos de los sensores
struct geograficPose{
    double wpLat;
    double wpLon;
};

/**
 * \struct mapPose
 * \brief  Estructura con de la posicion en el mapa
 */
//Estructura que contiene los datos de los sensores
struct mapPose{
    double x;
    double y;
};

/**
 * \class Seguimiento
 * \brief Clase que engloba todo lo necesario para la realizacion del seguimiento
 */
class Seguimiento
{
	public:
            Transform tfData;
            SensorGPS sensorData;
            int currentStatus;
            int currentType;
            geograficPose wpGoal;
            geograficPose posInit;
            queue<geograficPose> wpGoalPlan; 
            int statusGoal;
            bool wpReceived;
            bool planReceived;
            int hopePage;
            PathPlan newPlan;

        public:
            Seguimiento();
            ~Seguimiento(); 
            void updateSensorGPS(double lat,double lon,float alt,float roll,float pitch,float yaw);
            void updateGoal(double wplat,double wplon);
            void Follow_Me();
            void Come_To_Me();
            void Plan();  
            int getCurrentStatus();
            void setCurrentStatus(int);
            int getCurrentType();
            void setCurrentType(int);            
        private:
            // Funciones propias
            //calcular TF
            Transform calculateTF(SensorGPS);
            //Publica datos al move_base
            void sendGoalToMoveBase();
            void sendTFtoMoveBase();
            void sendSensorsToMoveBase();
            void sendStopToMoveBase();
            mapPose calculatePosition(SensorGPS,geograficPose);
            mapPose calculateGoal(SensorGPS,geograficPose);

		
};


#endif //SEGUIMIENTO_H 