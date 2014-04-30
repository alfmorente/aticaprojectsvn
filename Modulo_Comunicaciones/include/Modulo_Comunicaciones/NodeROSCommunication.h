/* 
 * File:   NodeROSCommunication.h
 * Author: atica
 *
 * Created on 28 de abril de 2014, 12:33
 */

#ifndef NODEROSCOMMUNICATION_H
#define	NODEROSCOMMUNICATION_H
#include<ros/ros.h>
#include <Modulo_Comunicaciones/ConverterTypes.h>



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
    
    
    
    public:
    static NodeROSCommunication* nodeROS;
    static bool instanceROSCreate;
    NodeROSCommunication();
    ~NodeROSCommunication();
    
    static NodeROSCommunication* getInstance();
    static void init(int argc,char** argv,char* name);
    void createSubscribers();
    void createPublishers();
    void createClients();
    void createServers();
    
    // Funciones de suscripcion
    static void fcn_sub_gps(const Common_files::msg_gps);
    static void fcn_sub_error(const Common_files::msg_error);
    static void fcn_sub_camera(const Common_files::msg_camera);
    static void fcn_sub_mode(const Common_files::msg_mode);
    static void fcn_sub_backup(const Common_files::msg_backup);
    static void fcn_sub_available(const Common_files::msg_available);
    static void fcn_sub_teach_file(const Common_files::msg_stream);
    static void fcn_sub_info_stop(const Common_files::msg_info_stop);
    static void fcn_sub_fcn_aux(const Common_files::msg_fcn_aux);
    static bool fcn_server_alive(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);

    int getStateModule();
    void setStateModule(int);
    string getDebugConfiguration();
    bool setDebugConfiguration(string file);
};

#endif	/* NODEROSCOMMUNICATION_H */

