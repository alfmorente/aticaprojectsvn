/**
 * @file   NodeROSCommunication.h
 * @brief  Fichero de cabecera para gestion del Nodo ROS de comunicaciones
 * @author David Jiménez 
 * @date   2013, 2014, 2015
 */



#ifndef NODEROSCOMMUNICATION_H
#define	NODEROSCOMMUNICATION_H

#include <ros/ros.h>
#include <Modulo_Comunicaciones/constantCommunication.h>
#include "../../../Common_files/include/Common_files/constant.h"

/**
 * \class NodeROSCommunication
 * \brief Clase que gestiona el nodo ROS del Módulo de comunicaciones
 */
class NodeROSCommunication {

    public:
    //Clientes
    ros::ServiceClient clientMode;
    
    //Servidores
    ros::ServiceServer server_alive;

    // Publicadores
    // comentario
    ros::Publisher pub_mode;
    ros::Publisher pub_comteleop_unclean;
    ros::Publisher pub_comteleop_clean;
    ros::Publisher pub_waypoint;
    ros::Publisher pub_error;
    ros::Publisher pub_fcn_aux;
    ros::Publisher pub_plan;
    ros::Publisher pub_ctrl_camera;  
  
    ros::Subscriber sub_gps; 
    ros::Subscriber sub_error;
    ros::Subscriber sub_camera;
    ros::Subscriber sub_modeSC;
    ros::Subscriber sub_modeNC;
    ros::Subscriber sub_backup;
    ros::Subscriber sub_file_teach;  
    ros::Subscriber sub_available; 
    ros::Subscriber sub_info_stop ; 
    ros::Subscriber sub_fcn_aux_ack; 

    ros::NodeHandle* n;
    
    
    
    private:
    static NodeROSCommunication* nodeCom;
    static bool instanceROSCreate;
    NodeROSCommunication();
    ~NodeROSCommunication();
    
    public:
    static NodeROSCommunication* getInstance();
    static void init(int argc,char** argv,char* name);
    void createSubscribers();
    void createPublishers();
    void createClients();
    void createServers();
    
    // Funciones de suscripcion
    static void fcn_sub_gps(const Common_files::msg_gpsPtr&);
    static void fcn_sub_error(const Common_files::msg_errorPtr&);
    static void fcn_sub_camera(const Common_files::msg_cameraPtr&);
    static void fcn_sub_mode(const Common_files::msg_modePtr&);
    static void fcn_sub_backup(const Common_files::msg_backupPtr&);
    static void fcn_sub_available(const Common_files::msg_availablePtr&);
    static void fcn_sub_teach_file(const Common_files::msg_streamPtr&);
    static void fcn_sub_info_stop(const Common_files::msg_info_stopPtr&);
    static void fcn_sub_fcn_aux(const Common_files::msg_fcn_auxPtr&);
    static bool fcn_server_alive(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);

    int getStateModule();
    void setStateModule(int);
    string getDebugConfiguration();
    bool setDebugConfiguration(string file);
    bool requestMode(int* mode);
    bool requestAvailable();
    bool requestErrors();
};

#endif	/* NODEROSCOMMUNICATION_H */

