/* 
 * File:   gest_navegacion.h
 * Author: carlosamores
 *
 * Created on 13 de septiembre de 2013, 12:27
 */

#ifndef GEST_NAVEGACION_H
#define	GEST_NAVEGACION_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* GEST_NAVEGACION_H */

//Mensajes
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "Modulo_Navegacion/ll-utm_constants.h"
#include "Modulo_Navegacion/AnalisisFichero.h"
#include "../../../../Common_files/include/Common_files/constant.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_gps.h"
#include "Common_files/msg_laser.h"
#include "Common_files/msg_mode.h"
#include "Common_files/msg_waypoint.h"
#include "Common_files/msg_stream.h"


//ROS y demas librerias
#include "ros/ros.h"
#include "constant.h"
#include <queue>
#include <iostream>
#include <vector>

using namespace std;

#define NO_GOAL 0
#define GOAL_ACTIVE 1
#define GOAL_REACHED 2
#define GOAL_CANCELED 3

//definimos el radio de la tierra
#define RADIO 6372.795477598 
#define PI 3.14159


//Estructura que contiene los datos de los sensores
typedef struct{
    double lat;
    double lon;
    float alt;
    float pitch;
    float yaw;
    float roll;
   // std::vector<float> angles;
   // std::vector<float> distances;
}Sensors;

//Estructura que contiene los datos de los sensores
typedef struct{
    float tf_x;
    float tf_y;
    float tf_rot;
}Transform;

//Estructura que contiene los datos de los sensores
typedef struct{
    double wpLat;
    double wpLon;
}Pose;

//Estructura que contiene los datos de los sensores
typedef struct{
    double x;
    double y;
}mapPose;

//Estructura que contiene los datos de los sensores
typedef struct
{
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped tf;
    geometry_msgs::PoseStamped goal;
    //sensor_msgs::LaserScan laser_data;
    actionlib_msgs::GoalID cancel_data;  
    
}DataMoveBase;

typedef struct{

    short submodule;
    short status;

}mode_nav;


DataMoveBase dmb;
mode_nav modeNavValue;
Transform tfData;
Sensors sensorData;
Pose posInit;

//Sensors sensorData;
int statusGoal;

// Variable de salida del modulo
int statusModule;
bool exitModule;

// Cola de waypoints
queue<float> queueWPsLatitude; 
queue<float> queueWPsLongitude;

// Publicadores globales
ros::Publisher pub_mode;
ros::Publisher pub_error;
ros::Publisher pub_odom;
ros::Publisher pub_waypoint;
ros::Publisher pub_stop;

//Función de subscripcion al estado del módulo
void fcn_sub_module_enable(const Common_files::msg_module_enable);


//Funciones de suscripcion de datos
void fcn_sub_gps(const Common_files::msg_gps);
//void fcn_sub_laser(const Common_files::msg_laser);
void fcn_sub_waypoint(const Common_files::msg_waypoint);
void fcn_sub_status_nav(const actionlib_msgs::GoalStatusArray);
void fcn_sub_plan(const Common_files::msg_stream);
void fcn_sub_vel(const geometry_msgs::Twist);


// Funciones propias
// Comprueba que la lista de waypoints esta vacia
bool checkEndListWaypoints();

//Se encarga de la navegacion del vehiculo en los tres submodos posibles
void fcn_mng_CTME();
void fcn_mng_PLAN();
void fcn_mng_FLME();

//calcular TF
Transform calculateTF(Sensors);

//Publica datos al move_base
void sendGoalToMoveBase(Pose goal);
void sendTFtoMoveBase(Transform,tf::TransformBroadcaster);
void sendSensorsToMoveBase(Sensors);
void sendStopToMoveBase();
void inizialiteTF();
//Pose convertPositionToUTM(Sensors);
mapPose calculatePosition(Sensors,Pose);
mapPose calculateGoal(Sensors,Pose);


//Come To Me
bool wpComeReceived;

//Plan recibido
int currentPage;
bool planReceived;
PathPlan newPlan;