/**
 * @file   NodeROSNavigation.h
 * @brief  Fichero de cabecera del nodo ROS para gestionar la navegación
 * @author David Jiménez 
 * @date   2013, 2014, 2015
 * @addtogroup Navigation
 * @{
 */

#ifndef NODEROSNAVIGATION_H
#define	NODEROSNAVIGATION_H

#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_gps.h"
#include "Common_files/msg_laser.h"
#include "Common_files/msg_mode.h"
#include "Common_files/msg_waypoint.h"
#include "Common_files/msg_stream.h"
#include "../../../../Common_files/include/Common_files/constant.h"
#include "Seguimiento.h"



//Estructura que contiene los datos de los sensores
/*typedef struct{

    short submodule;
    short status;

}ModeNav;*/


/*Clase que implementa las funciones del nodo ROS de navegación*/
class NodeROSNavigation
{
       private:
            //ModeNav   enableModule;
            bool exitModule;      
            static ros::Publisher pub_mode;
            static ros::Publisher pub_error;
            static ros::Publisher pub_odom;
            static ros::Publisher pub_waypoint;
            static ros::Publisher pub_stop; 
            ros::Subscriber sub_gps; 
            ros::Subscriber sub_module_enable; 
            ros::Subscriber sub_waypoints; 
            ros::Subscriber sub_plan; 
            ros::Subscriber sub_move_base_status;             
            ros::NodeHandle* n;
        public: 
            Seguimiento nav;
        public:
            NodeROSNavigation(int argc,char** argv,char* name);
            void createPublishers();
            void createSubscribers();
            //void createServers();
            static void publishMode(int mode);
            static void publishError(int idError,int typeError);
            static void publishOdom(double x,double y,double yaw);
            static void publishWaypoint(double longitud,double latitud,float yawRel);
            static void publishStop();
            static void publishTF(double,double,double,double,string,string);       
            //ModeNav getEnableModule();
            bool getExitModule();       
            int getStateModule();
            void setStateModule(int);  
        private:
            void fcn_sub_module_enable(Common_files::msg_module_enable);            
            void fcn_sub_gps(Common_files::msg_gps);
            void fcn_sub_waypoint(Common_files::msg_waypoint);
            void fcn_sub_status_nav(actionlib_msgs::GoalStatusArray);
            void fcn_sub_plan(Common_files::msg_stream);
            void fcn_sub_vel(const geometry_msgs::Twist msg); 
};
#endif

/**
 *@}
 */