/**
 * @file   NodeROSNavigation.cpp
 * @brief  Fichero fuente para gestionar el nodo ROS de navegación 
 * @author David Jiménez 
 * @date   2013, 2014, 2015
 */
#include <Modulo_Navegacion/NodeROSNavigation.h>

ros::Publisher NodeROSNavigation::pub_mode;
ros::Publisher NodeROSNavigation::pub_error;
ros::Publisher NodeROSNavigation::pub_odom;
ros::Publisher NodeROSNavigation::pub_waypoint;
ros::Publisher NodeROSNavigation:: pub_stop; 

/**
 * Constructor de la clase
 * @param[in] argc Número de argumentos de entrada
 * @param[io] argv Valores de los argumentos de entrada
 * @param[in] name Nombre del nodo ROS
 */
NodeROSNavigation::NodeROSNavigation(int argc, char** argv, char* name) 
{
    ros::init(argc,argv,name);
    n=new ros::NodeHandle();
}

/**
 * Método que crea los publicadores
 */
void NodeROSNavigation::createPublishers()
{
    //Generación de publicadores
    pub_mode = n->advertise<Common_files::msg_mode>("mode", 1000);
    pub_error = n->advertise<Common_files::msg_error>("error",1000);
    pub_odom = n->advertise<nav_msgs::Odometry>("mi_odom",1000); 
    pub_waypoint=n->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    pub_stop=n->advertise<actionlib_msgs::GoalID>("move_base/cancel",1000);  
}

/**
 * Método que crea los subscriptores
 */
void NodeROSNavigation::createSubscribers()
{
    // Generacion de suscriptores
    sub_module_enable=n->subscribe("/module_enable", 1000,&NodeROSNavigation::fcn_sub_module_enable,this);
    sub_gps=n->subscribe("/gps",1000,&NodeROSNavigation::fcn_sub_gps,this);
    sub_waypoints=n->subscribe("/waypoints",1000,&NodeROSNavigation::fcn_sub_waypoint,this);
    sub_move_base_status=n->subscribe("/move_base/status",1000,&NodeROSNavigation::fcn_sub_status_nav,this);
    sub_plan=n->subscribe("/plan",1000,&NodeROSNavigation::fcn_sub_plan,this);    
    
}

/**
 * Callback subscriptor para la recepcio
 * @param msg
 */
/**void NodeROSNavigation::fcn_sub_vel(const geometry_msgs::Twist msg)
{
    double v_lineal=msg.linear.x;
    double v_angular=msg.angular.z;
    double longitud_ejes=1.93; //distancia entre ejes
    double angulo;
    angulo=atan(v_angular*longitud_ejes/v_lineal)*180/M_PI;
    ROS_INFO("Angulo de la rueda puesto a: %lf",angulo);
    
}**/

/**
 * Callback subscriptor ROS de datos de GPS
 */
void NodeROSNavigation::fcn_sub_gps(Common_files::msg_gps msg)
{
    nav.updateSensorGPS(msg.latitude,msg.longitude,msg.altitude,msg.roll,msg.pitch,msg.yaw);
}

/**
 * Callback subscriptor ROS del estado del seguimiento
 * @param[in] msg Mensaje ROS con el estado del seguimiento
 */
void NodeROSNavigation::fcn_sub_status_nav(actionlib_msgs::GoalStatusArray msg)
{
    //Compruebo el estado de la navegacion
    //ROS_INFO("RECIBO ESTADO DE LA NAVEGACION");  
    int numWP=msg.status_list.size();
    if(numWP > 0)
    {
        int status=msg.status_list[numWP-1].status;
        switch(status)
        {
            case 1:
                nav.statusGoal=GOAL_ACTIVE;            
                break;
            case 3:               
                nav.statusGoal=GOAL_REACHED;                
                break;
            default:
                nav.statusGoal=GOAL_CANCELED;
                break;

        }  
    }    
    
}

/**
 * Callback subscriptor ROS de habilitación/deshabilitación del módulo
 * @param[in] msg Mensaje ROS para habilitar o deshabilitar el módulo
 */
void NodeROSNavigation::fcn_sub_module_enable(Common_files::msg_module_enable msg)
{
    cout <<"llega module enable"<<endl;
    // Suscriptor de gestion de sistema
    if(msg.id_module==ID_MOD_NAVIGATION)
    {
            //enableModule.submodule=msg.submode;
            //enableModule.status=msg.status;
            
            if(nav.getCurrentStatus()!=MOD_OFF && nav.getCurrentType()==msg.submode)
            {
                nav.setCurrentType(msg.submode);
                nav.setCurrentStatus(msg.status);                
            }
            else if(nav.getCurrentStatus()==MOD_OFF && msg.status==MOD_ON)
            {
                nav.setCurrentType(msg.submode);
                nav.setCurrentStatus(MOD_ON);                
            }            
            
    }
}

/**
 * Callback subscriptor ROS de los datos del plan
 * @param[in] msg Mensaje ROS con una página del plan
 */
void NodeROSNavigation::fcn_sub_plan(Common_files::msg_stream msg)
{
    if(nav.getCurrentStatus()==MOD_ON && nav.getCurrentType()==SUBMODE_NAV_PLAN)
    {
        if(msg.id_file==TOF_PLAN && !nav.planReceived)
        {
            if(nav.hopePage==1)
                cleanPathPlan(&nav.newPlan);

            if(analizePage(msg.stream,nav.hopePage,&nav.newPlan))
            {

                if(nav.newPlan.numPages==nav.hopePage)
                {
                    ROS_INFO("Plan Completo recibido");
                    while(!nav.wpGoalPlan.empty())
                         nav.wpGoalPlan.pop();     
                    //lleno las listas con los waypoints
                    for(int i=0;i<nav.newPlan.numPages;i++)
                    {
                        ROS_INFO("Numero de pagina: %d",nav.newPlan.vecDP.at(i).currentPage);
                        for(int j=0;j<nav.newPlan.vecDP.at(i).numWP;j++)
                        {
                            ROS_INFO("Numero de waypoints: %d",nav.newPlan.vecDP.at(i).numWP);
                            nav.wpGoal.wpLat=nav.newPlan.vecDP.at(i).wpLat.at(j);
                            nav.wpGoal.wpLon=nav.newPlan.vecDP.at(i).wpLon.at(j);
                            nav.wpGoalPlan.push(nav.wpGoal);    
                        }
                            
                    }
                    cleanPathPlan(&nav.newPlan); 
                    nav.planReceived=true;
                }
                else
                    nav.hopePage++;
            }  
            else
            {
                nav.hopePage=1;                
                ROS_INFO("Plan mal recibido....Esperando a recibir un nuevo plan");
                //Envio error de pagina mal estructurada
            }
        }
    }    
    
}

/**
 * Callback subscriptor ROS de los waypoints para modos COME TO ME y FOLLOW ME 
 * @param[in] msg Mensaje ROS con el waypoint a alcanzar
 */
void NodeROSNavigation::fcn_sub_waypoint(Common_files::msg_waypoint msg)
{
    if(nav.getCurrentStatus()==MOD_ON)    
    {
        ROS_INFO("RECIBO WAYPOINT");
        ROS_INFO("Latitud: %f",msg.wp_latitude);
        ROS_INFO("Longitude: %f",msg.wp_longitude);
        nav.updateGoal(msg.wp_latitude,msg.wp_longitude);
    }
    
}

/*
ModeNav NodeROSNavigation::getEnableModule()
{
    return enableModule;
}*/

/**
 * Metodo que devuelve si el módulo ha finalizado
 * @return Booleano indicando la finalización del módulo
 */
bool NodeROSNavigation::getExitModule()
{
    return exitModule;
}

/**
 * Método que devuelve el estado del módulo
 * @return Entero con el estado actual del módulo
 */
int NodeROSNavigation::getStateModule()
{
    int state_module;
    n->getParam("state_module_navigation",state_module);
    return state_module;
}

/**
 * Método que para poner el estado del módulo en un estado determinado
 * @param[in] state_module Estado al que se quiere poner
 */
void NodeROSNavigation::setStateModule(int state_module)
{
    n->setParam("state_module_navigation",state_module);
}



/**
 * Método que publica un error ocurrido en el módulo
 * @param[in] id Identificador del error
 * @param[in] type Tipo de error
 */
void NodeROSNavigation::publishError(int id,int type)
{
    Common_files::msg_error msg;
    msg.id_subsystem=SUBS_NAVIGATION;
    msg.id_error=id;
    msg.type_error=type;
    pub_error.publish(msg);
}

/**
 * Método para publicar que la ruta ha finalizado
 * @param[in] mode Modo de seguimiento en activo 
 */
void NodeROSNavigation::publishMode(int mode)
{
    Common_files::msg_mode msg;
    msg.mode=mode;
    msg.status=MODE_FINISH;
    msg.type_msg=SET;
    pub_mode.publish(msg);
}

/**
 * Método para publicar la odometría del vehículo
 * @param[in] x Posición en el eje x
 * @param[in] y Posición en el eje y
 * @param[in] yaw Orientación
 */
void NodeROSNavigation::publishOdom(double x, double y, double yaw)
{
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x=x;
    msg.pose.pose.position.y=y;
    msg.pose.pose.orientation=tf::createQuaternionMsgFromYaw(yaw);
    pub_odom.publish(msg);
}

/**
 * Método para publicar parada del seguimiento
 */
void NodeROSNavigation::publishStop()
{
    actionlib_msgs::GoalID msg;
    pub_stop.publish(msg);
}

/**
 * Método para publicar el goal del seguimiento
 * @param[in] xRel Posición relativa x del goal
 * @param[in] yRel Posición relativa y del goal
 * @param[in] yawRel Orientación relativa del goal
 */
void NodeROSNavigation::publishWaypoint(double xRel,double yRel,float yawRel)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id="/robot_0/base_link";
    msg.pose.position.x=xRel;
    msg.pose.position.y=yRel;
    msg.pose.orientation=tf::createQuaternionMsgFromYaw(yawRel);     
    pub_waypoint.publish(msg);
}

/**
 * Método que publica el vector de transformación 
 * @param x Offset en el eje x
 * @param y Offset en el eje y
 * @param z Offset en el eje z
 * @param yaw Offset en la orientación
 * @param header Header del vector TF
 * @param child Child del vestor TF
 */
void NodeROSNavigation::publishTF(double x,double y,double z,double yaw,string header,string child)
{
    tf::TransformBroadcaster tfBroad;   
    geometry_msgs::TransformStamped tfStamped;  

    tfStamped.header.stamp = ros::Time::now();
    tfStamped.header.frame_id=header.c_str();
    tfStamped.child_frame_id=child.c_str(); //robot_0 es para el caso de dos moviles
    tfStamped.transform.translation.x=x;
    tfStamped.transform.translation.y=y;
    tfStamped.transform.translation.z=z;    
    tfStamped.transform.rotation=tf::createQuaternionMsgFromYaw(yaw);        
            
    tfBroad.sendTransform(tfStamped);    
      
}
