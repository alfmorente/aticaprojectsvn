#include <vector>

#include  "Modulo_Navegacion/navegacion.h"

using namespace std;



int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "navegacion");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_navegacion",estado_actual);
  }
  cout << "Atica Navegacion :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_mode = n.advertise<Common_files::msg_mode>("modeNC", 1000);
  pub_error = n.advertise<Common_files::msg_error>("error",1000);
  pub_odom = n.advertise<nav_msgs::Odometry>("mi_odom",1000); 
  pub_waypoint=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
  pub_stop=n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1000);  

  // Generacion de suscriptores
  //ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
  ros::Subscriber sub_module_enable = n.subscribe("/module_enable", 1000, fcn_sub_module_enable);
  ros::Subscriber sub_gps = n.subscribe("/gps",1000,fcn_sub_gps);
  ros::Subscriber sub_waypoints = n.subscribe("/waypoints",1000,fcn_sub_waypoint);
  ros::Subscriber sub_status_nav = n.subscribe("/move_base/status",1000,fcn_sub_status_nav);
  ros::Subscriber sub_plan = n.subscribe("/plan",1000,fcn_sub_plan);
  ros::Subscriber sub_vel = n.subscribe("/robot_0/cmd_vel",1000,fcn_sub_vel);
  

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_navegacion",STATE_OK);
  cout << "Atica Laser2D :: Configurado y funcionando" << endl;

  statusGoal=NO_GOAL;
  inizialiteTF();   

  while (ros::ok() && !exitModule){
      n.getParam("state_module_navigation",estado_actual);
      if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
           exitModule=true;
      }
      else if(modeNavValue.status==MOD_ON)
      {
          //tfData=calculateTF(sensorData); 
          //posInit=convertPositionToUTM(sensorData);  
          //cout << "Posicion Inicial: x="<< posInit.wpLon<<"  y="<<posInit.wpLat <<endl;
          posInit.wpLon=sensorData.lon;  
          posInit.wpLat=sensorData.lat;  
          cout << "Posicion Inicial: Lon="<< posInit.wpLon<<"  Lat="<<posInit.wpLat <<endl;
          switch(modeNavValue.submodule)
          {
              case SUBMODE_NAV_COME_TO_ME:
                  ROS_INFO("INICIO COME TO ME");
                  fcn_mng_CTME();
                  ROS_INFO("FINALIZO COME TO ME");                  
                  break;
              case SUBMODE_NAV_PLAN:
                  ROS_INFO("INICIO PLAN");                  
                  fcn_mng_PLAN();
                  ROS_INFO("FINALIZO PLAN");
                  break;
              case SUBMODE_NAV_FOLLOW_ME:
                  ROS_INFO("INICIO FOLLOW ME");                  
                  fcn_mng_FLME();
                  ROS_INFO("FINALIZO FOLLOW ME");                  
                  break;
              default:
                  break;
          }

      }
      ros::spinOnce();
  }

  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/
void fcn_sub_vel(const geometry_msgs::Twist msg)
{
    double v_lineal=msg.linear.x;
    double v_angular=msg.angular.z;
    double longitud_ejes=1.93; //distancia entre ejes
    double angulo;
    angulo=atan(v_angular*longitud_ejes/v_lineal)*180/M_PI;
    ROS_INFO("Angulo de la rueda puesto a: %lf",angulo);
    
}

void fcn_sub_plan(const Common_files::msg_stream msg)
{
    double latitude;
    double longitude;
    int numWaypoints;
    if(modeNavValue.status==MOD_ON && modeNavValue.submodule==SUBMODE_NAV_PLAN)
    {
        if(msg.id_file==TOF_PLAN && !planReceived)
        {
            if(currentPage==1)
                cleanPathPlan(&newPlan);

            if(analizePage(msg.stream,currentPage,&newPlan))
            {

                if(newPlan.numPages==currentPage)
                {
                    planReceived=true;
                    ROS_INFO("Plan Completo recibido");
                    while(!queueWPsLongitude.empty())
                            queueWPsLongitude.pop();
                    while(!queueWPsLatitude.empty())
                            queueWPsLatitude.pop();
                    
                    //lleno las listas con los waypoints
                    for(int i=0;i<newPlan.numPages;i++)
                    {
                        ROS_INFO("Numero de pagina: %d",newPlan.vecDP.at(i).currentPage);
                        for(int j=0;j<newPlan.vecDP.at(i).numWP;j++)
                        {
                            ROS_INFO("Numero de waypoints: %d",newPlan.vecDP.at(i).numWP);
                            queueWPsLatitude.push(newPlan.vecDP.at(i).wpLat.at(j));
                            queueWPsLongitude.push(newPlan.vecDP.at(i).wpLon.at(j));
                            
                        }
                            
                    }
                    cleanPathPlan(&newPlan);           
                }
                else
                    currentPage++;
            }  
            else
            {
                currentPage=1;                
                ROS_INFO("Plan mal recibido....Esperando a recibir un nuevo plan");
                //Envio error de pagina mal estructurada
            }
        }
    }
       
}



// Suscriptor de gestion de sistema
void fcn_sub_module_enable(const Common_files::msg_module_enable msg)
{
    if(msg.id_module==ID_MOD_NAVIGATION)
    {
        ROS_INFO("Recibo activacion de modulo de navegacion");
        modeNavValue.submodule=msg.submode;
        modeNavValue.status=msg.status;
    }
}

// Suscriptor de errores
/**void fcn_sub_laser(const Common_files::msg_laser msg)
{
    sensorsValues.angles=msg.angle;
    sensorsValues.distances=msg.distance;
}**/

// Suscriptor de gps
void fcn_sub_gps(const Common_files::msg_gps msg)
{
    
    sensorData.alt=msg.altitude;
    sensorData.lon=msg.longitude;
    sensorData.lat=msg.latitude;
    sensorData.pitch=msg.pitch; //rad (dicho por carlos))
    sensorData.yaw=msg.yaw;     //rad (dicho por carlos))
    sensorData.roll=msg.roll;   //rad (dicho por carlos))
    if(modeNavValue.status==MOD_ON)    
            sendSensorsToMoveBase(sensorData);            
}

// Suscriptor de un solo waypoint
void fcn_sub_waypoint(const Common_files::msg_waypoint msg)
{
    if(modeNavValue.status==MOD_ON)    
    {
        ROS_INFO("RECIBO WAYPOINT");
        wpComeReceived=true;
        queueWPsLatitude.push(msg.wp_latitude);
        queueWPsLongitude.push(msg.wp_longitude);
        ROS_INFO("Latitud: %f",msg.wp_latitude);
        ROS_INFO("Longitude: %f",msg.wp_longitude);
    }
}

// Suscriptor de un solo waypoint
void fcn_sub_status_nav(const actionlib_msgs::GoalStatusArray msg)
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
                statusGoal=GOAL_ACTIVE;            
                break;
            case 3:               
                statusGoal=GOAL_REACHED;                
                break;
            default:
                statusGoal=GOAL_CANCELED;
                break;

        }  
    }
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

bool checkEndListWaypoints()
{
    if(queueWPsLatitude.empty() || queueWPsLongitude.empty())
    {
        //ROS_INFO("COLA VACIA");
        return true;
    }
    else
    {
        //ROS_INFO("COLA CON DATOS");        
        return false;
    }
}

void fcn_mng_CTME()
{
    // Se activa el modo COME TO ME
    bool endCTME=false;
    bool navigationPaused=false;
    tf::TransformBroadcaster tfBroad;     
    Pose wpGoal;

    wpComeReceived=false;
    //Espero waypoint
 
    while(!wpComeReceived && modeNavValue.status!=MOD_OFF)
    {
        sendTFtoMoveBase(tfData,tfBroad);
        usleep(100000);
        ros::spinOnce();
    }      
    if(modeNavValue.status==MOD_OFF)
        endCTME=true;
    else
    {
        ROS_INFO("RECIBO WAYPOINT DE COME TO ME");        
        wpGoal.wpLat=queueWPsLatitude.front();
        queueWPsLatitude.pop();

        wpGoal.wpLon=queueWPsLongitude.front();
        queueWPsLongitude.pop();

        ROS_INFO("ENVIO WAYPOINT A MOVE BASE");
        sendGoalToMoveBase(wpGoal);
        /**while(statusGoal!=GOAL_ACTIVE)
            ros::spinOnce();**/                            
    }

    while(!endCTME)
    {

        sendTFtoMoveBase(tfData,tfBroad);          
        switch(modeNavValue.status)
        {
            case MOD_OFF:
                //Mandar stop a move base
                endCTME=true;
                sendStopToMoveBase();
                break;
            case MOD_PAUSE:
                if(!navigationPaused)
                {                    
                    ROS_INFO("NAVEGACION PAUSADA");
                    //Se manda pause a move base
                    sendStopToMoveBase();
                    navigationPaused=true;
                }
                break;
            case MOD_ON:
                // Obtencion del siguiente WP
                if(navigationPaused)
                {
                    ROS_INFO("NAVEGACION RESUME");                    
                    sendGoalToMoveBase(wpGoal);
                    navigationPaused=false;
                }
                else
                {

                    if(statusGoal==GOAL_REACHED)
                    {
                          ROS_INFO("HE LLEGADO A MI DESTINO");
                          Common_files::msg_mode mode;
                          mode.mode=MODE_COME_TO_ME;
                          mode.status=MODE_FINISH;
                          pub_mode.publish(mode);

                          while(modeNavValue.status!=MOD_OFF)
                          {
                               ros::spinOnce();
                               usleep(100000);
                          }
                          endCTME=true;

                    }
                }
                break;
            default:
                break;

        }            
        ros::spinOnce();
        usleep(100000);
    }
 
}


void fcn_mng_PLAN()
{
    // Se activa el modo PLAN
    bool navigationPaused=false;
    bool endPLAN=false;
    tf::TransformBroadcaster tfBroad;    
    Pose wpGoal;
    
   //Espero waypoint
    planReceived=false;
    while(!endPLAN)
    {

        if(!planReceived)
        {
            currentPage=1;   
            ROS_INFO("Esperando PLAN....");
            while(!planReceived && modeNavValue.status!=MOD_OFF)
                    ros::spinOnce();      
            if(modeNavValue.status==MOD_OFF)
                endPLAN=true;
            
            else
            {
                ROS_INFO("Comienzo PLAN....");        
                wpGoal.wpLat=queueWPsLatitude.front();
                queueWPsLatitude.pop();

                wpGoal.wpLon=queueWPsLongitude.front();
                queueWPsLongitude.pop();

                sendGoalToMoveBase(wpGoal);
                //while(statusGoal!=GOAL_ACTIVE)
                 //    ros::spinOnce();            
                 sendTFtoMoveBase(tfData,tfBroad);
            }
        }
        else
        {
            sendTFtoMoveBase(tfData,tfBroad);            
            switch(modeNavValue.status)
            {
                case MOD_OFF:
                    //Mandar stop a move base
                    sendStopToMoveBase();
                    endPLAN=true;
                    break;
                case MOD_PAUSE:
                    if(!navigationPaused)
                    {               
                        ROS_INFO("NAVEGACION PAUSADA");
                        //Se manda pause a move base
                        sendStopToMoveBase();
                        navigationPaused=true;
                    }
                    break;    
                case MOD_ON:
                    // Obtencion del siguiente WP
                    if(navigationPaused)
                    {
                        ROS_INFO("NAVEGACION RESUME");     
                        sendGoalToMoveBase(wpGoal);
                        navigationPaused=false;
                    }
                    else
                    {
                        if(statusGoal==GOAL_REACHED)
                        {
                            ROS_INFO("HE LLEGADO AL WAYPOINT");
                            if(checkEndListWaypoints())
                            {
                                    ROS_INFO("HE LLEGADO A MI DESTINO");
                                    Common_files::msg_mode mode;
                                    mode.mode=MODE_PLAN;
                                    mode.status=MODE_FINISH;
                                    pub_mode.publish(mode);
                                    planReceived=false;
                            }
                            else
                            {
                                    wpGoal.wpLat=queueWPsLatitude.front();
                                    queueWPsLatitude.pop();

                                    wpGoal.wpLon=queueWPsLongitude.front();
                                    queueWPsLongitude.pop();

                                    sendGoalToMoveBase(wpGoal);
                            }
                        }

                    }
                    break;
                default:
                    break;
            }
        }
        usleep(100000);
        ros::spinOnce();
    }

}

void fcn_mng_FLME()
{
    // Se activa el modo PLAN 
    bool endFLME=false;  
    bool navigationPaused=false;
    tf::TransformBroadcaster tfBroad;
    Pose wpGoal;
    
    while(!endFLME)
    {
        sendTFtoMoveBase(tfData,tfBroad);
        switch(modeNavValue.status)
        {
            case MOD_OFF:
                //Mandar stop a move base
                endFLME=true;
                sendStopToMoveBase();
                break;
            case MOD_PAUSE:
                //Se manda pause a move base
                if(!navigationPaused)
                {
                    ROS_INFO("NAVEGACION PAUSADA");
                    sendStopToMoveBase();
                    navigationPaused=true;
                }
                break;
            case MOD_ON:
                if(navigationPaused)
                {
                    ROS_INFO("NAVEGACION RESUME");                    
                    // Se borra la lista de WPs
                    while(!queueWPsLongitude.empty())
                        queueWPsLongitude.pop();
                    while(!queueWPsLatitude.empty())
                        queueWPsLatitude.pop();                    
                    navigationPaused=false;
                }                
                else
                {
                    
                    // Obtencion del siguiente WP
                    if(!checkEndListWaypoints())
                    {
                        wpGoal.wpLat=queueWPsLatitude.front();
                        queueWPsLatitude.pop();

                        wpGoal.wpLon=queueWPsLongitude.front();
                        queueWPsLongitude.pop();

                        ROS_INFO("ENVIO WAYPOINT A MOVE BASE");
                        sendGoalToMoveBase(wpGoal);                  
                    }
                }
                break;
            default:
                break;

        }
        usleep(100000);
        ros::spinOnce();
    }

}


void sendTFtoMoveBase(Transform tfData,tf::TransformBroadcaster tfBroad)
{   
    //Envio la transformacion de /odom a /map
    dmb.tf.header.stamp = ros::Time::now();
    dmb.tf.header.frame_id="map";
    dmb.tf.child_frame_id="/robot_0/odom"; //robot_0 es para el caso de dos moviles
    dmb.tf.transform.translation.x=tfData.tf_x;
    dmb.tf.transform.translation.y=tfData.tf_y;
    dmb.tf.transform.translation.z=0;    
    dmb.tf.transform.rotation=tf::createQuaternionMsgFromYaw(tfData.tf_rot);
             
            
    tfBroad.sendTransform(dmb.tf);
   
}
void sendGoalToMoveBase(Pose Goal)
{
    char UTMZone[5];
    double utm_x;
    double utm_y;
 
    mapPose positionGoal;
    mapPose positionCurrent;
    //LLtoUTM(23,Goal.wpLat,Goal.wpLon,utm_y,utm_x,UTMZone);
    positionGoal=calculateGoal(sensorData,Goal);
    positionCurrent=calculatePosition(sensorData,posInit);

    //Envio el siguiente waypoint 
    dmb.goal.header.frame_id="/robot_0/base_link";
    //dmb.goal.pose.position.x=utm_x-posInit.wpLon;    
    //dmb.goal.pose.position.y=utm_y-posInit.wpLat;
    dmb.goal.pose.position.x=positionGoal.x*cos(sensorData.yaw)+positionGoal.y*sin(sensorData.yaw); 
    dmb.goal.pose.position.y=positionGoal.y*cos(sensorData.yaw)-positionGoal.x*sin(sensorData.yaw); 
    dmb.goal.pose.position.z=0;


    //Calculo la orientacion optima
    double ang_auxiliar=atan2(positionGoal.y,positionGoal.x);
    double orientacion_optima=ang_auxiliar-sensorData.yaw;
    dmb.goal.pose.orientation=tf::createQuaternionMsgFromYaw(orientacion_optima);

    printf("Posicion actual x: %f\n",positionCurrent.x);
    printf("Posicion actual y: %f\n",positionCurrent.y);
    printf("Posicion Goal x: %f\n",positionGoal.x);
    printf("Posicion Goal y: %f\n",positionGoal.y);
    printf("ang_auxiliar: %f\n",ang_auxiliar);
    printf("Yaw: %f\n",sensorData.yaw);
    printf("Orientacion optima: %f\n",orientacion_optima);
    printf("Lat1: %f\n",Goal.wpLat);
    printf("Lat2: %f\n",sensorData.lat);
    printf("Lon1: %f\n",Goal.wpLon);
    printf("Lon2: %f\n",sensorData.lon);
    printf("Posicion final x: %f\n",positionGoal.x+positionCurrent.x);
    printf("Posicion final y: %f\n\n",positionGoal.y+positionCurrent.y);
    
    //Publico el waypoint
    pub_waypoint.publish(dmb.goal);
}
void sendSensorsToMoveBase(Sensors sensorData)
{   
    mapPose position;
    //position=convertPositionToUTM(sensorData);
    //dmb.odom.pose.pose.position.x=position.wpLon-posInit.wpLon;
    //dmb.odom.pose.pose.position.y=position.wpLat-posInit.wpLat;
    position=calculatePosition(sensorData,posInit);
    dmb.odom.pose.pose.position.x=position.x;
    dmb.odom.pose.pose.position.y=position.y; 
    dmb.odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(sensorData.yaw);   
    
    //cout<< "X:"<< dmb.odom.pose.pose.position.x << "  Y:"<< dmb.odom.pose.pose.position.y<<endl;
    //Publico datos de sensores  
    pub_odom.publish(dmb.odom);
}

/**Pose convertPositionToUTM(Sensors sensorData)
{
    Pose UTMpos;
    char UTMZone[5];
    LLtoUTM(23,sensorData.lat,sensorData.lon,UTMpos.wpLat,UTMpos.wpLon,UTMZone);
    return UTMpos;
}**/

mapPose calculatePosition(Sensors sensorData, Pose posInit)
{

    //printf("Lat1: %f\n",posInit.wpLat);
    //printf("Lat2: %f\n",sensorData.lat);
    //printf("Lon1: %f\n",posInit.wpLon);
    //printf("Lon2: %f\n",sensorData.lon);

    double lat2=sensorData.lat*PI/180;
    double lat1=posInit.wpLat*PI/180;
    double lon2=sensorData.lon*PI/180;
    double lon1=posInit.wpLon*PI/180;

    mapPose position;
    position.x=RADIO*acos(sin(lat1)*sin(lat1)+cos(lat1)*cos(lat1)*cos(lon1-lon2))*1000;         //Diferencia de latitudes a 0
    position.y=RADIO*acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(0))*1000;         //Diferencia de longitudes a 0

    if(lat2 <lat1)
	position.y=-position.y;
    if(lon2 <lon1)
	position.x=-position.x;


    //ROS_INFO("X: %f\n",position.x);
    //ROS_INFO("y: %f\n\n",position.y);
    return position;
}
mapPose calculateGoal(Sensors sensorData, Pose poseGoal)
{

    double lat2=poseGoal.wpLat*PI/180;
    double lat1=sensorData.lat*PI/180;
    double lon2=poseGoal.wpLon*PI/180;
    double lon1=sensorData.lon*PI/180;

    mapPose position;
    position.x=RADIO*acos(sin(lat1)*sin(lat1)+cos(lat1)*cos(lat1)*cos(lon1-lon2))*1000;         //Diferencia de latitudes a 0
    position.y=RADIO*acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(0))*1000;         //Diferencia de longitudes a 0

    if(lat2 <lat1)
	position.y=-position.y;
    if(lon2 <lon1)
	position.x=-position.x;

    return position;
}
void sendStopToMoveBase()
{
    //publica cancel data
    pub_stop.publish(dmb.cancel_data);
}

void inizialiteTF()
{
    Transform tfData;
    tfData.tf_x=0;
    tfData.tf_y=0;
    tfData.tf_rot=0;//sensorValues.yaw;
}

