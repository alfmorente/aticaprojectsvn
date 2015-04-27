/**
 * @file   Seguimiento.cpp
 * @brief  Fichero fuente para la gestion del seguimiento
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */
#include "../include/Modulo_Navegacion/NodeROSNavigation.h"

/**
 * Constructor de la clase
 */
Seguimiento::Seguimiento() 
{
    tfData={HEADER_FRAME,CHILD_FRAME,X_OFFSET,Y_OFFSET,Z_OFFSET,YAW_OFFSET};
    wpGoal={0.0,0.0};
    posInit={0.0,0.0};
    sensorData={0.0,0.0,0.0,0.0,0.0,0.0,};
    wpReceived=false;
    currentStatus=MOD_OFF;
    statusGoal=NO_GOAL;
}

/**
 * Destructor de la clase
 */
Seguimiento::~Seguimiento()
{
    
}

/**
 * Método que envia un goal al move_base
 */
void Seguimiento::sendGoalToMoveBase()
{
    mapPose positionGoal;
    mapPose positionCurrent;
    positionGoal=calculateGoal(sensorData,wpGoal);
    positionCurrent=calculatePosition(sensorData,posInit);

    double xRel=positionGoal.x*cos(sensorData.yaw)+positionGoal.y*sin(sensorData.yaw);
    double yRel=positionGoal.y*cos(sensorData.yaw)-positionGoal.x*sin(sensorData.yaw);
    double ang_auxiliar=atan2(positionGoal.y,positionGoal.x);
    float yaw_opt=ang_auxiliar-sensorData.yaw;    
    
    NodeROSNavigation::publishWaypoint(xRel,yRel,yaw_opt); 
}

/**
 * Método que envia la odometría al move_base
 */
void Seguimiento::sendSensorsToMoveBase()
{
    mapPose position;
    position=calculatePosition(sensorData,posInit);
    NodeROSNavigation::publishOdom(position.x,position.y,sensorData.yaw);  
}

/**
 * Método que envia una parada de seguimiento
 */
void Seguimiento::sendStopToMoveBase()
{
    NodeROSNavigation::publishStop();    
}

/**
 * Método que enviar el TF al move_base
 */
void Seguimiento::sendTFtoMoveBase()
{    
    NodeROSNavigation::publishTF(X_OFFSET,Y_OFFSET,Z_OFFSET,YAW_OFFSET,HEADER_FRAME,CHILD_FRAME);
}

/**
 * Método que obtiene el estado del seguimiento
 * @return Entero que indica el estado del seguimiento
 */
int Seguimiento::getCurrentStatus()
{
    return currentStatus;
}

/**
 * Método para poner un estado determinado al seguimiento
 * @param[in] status Estado del seguimiento  
 */
void Seguimiento::setCurrentStatus(int status)
{
    this->currentStatus=status;
}

/**
 * Método para obtener el tipo de seguimiento actual
 * @return Entero que indica el tipo de seguimiento actual
 */
int Seguimiento::getCurrentType()
{
    return currentType;
}

/**
 * Método para poner un tipo de seguimiento determinado
 * @param[in] type Tipo de seguimiento
 */
void Seguimiento::setCurrentType(int type)
{
    this->currentType=type;
}

/**
 * Método que actualiza la estructura con los datos del GPS
 * @param lat[in] Latitud actual
 * @param lon[in] Longitud actual
 * @param alt[in] Altitud actual
 * @param roll[in] Roll actual
 * @param pitch[in] Pitch actual
 * @param yaw[in] Yaw actual
 */
void Seguimiento::updateSensorGPS(double lat,double lon,float alt,float roll,float pitch,float yaw)
{
    sensorData.lat=lat;
    sensorData.lon=lon;
    sensorData.alt=alt;
    sensorData.roll=roll;
    sensorData.pitch=pitch;
    sensorData.yaw=yaw;
}

/**
 * Método que actualiza el nuevo goal
 * @param wpLat[in] Latitud del goal
 * @param wpLon[in] Longitud del goal
 */
void Seguimiento::updateGoal(double wpLat,double wpLon)
{
    if(!wpReceived || this->currentType==SUBMODE_NAV_FOLLOW_ME)
    {
        wpReceived=true;
        wpGoal.wpLat=wpLat;
        wpGoal.wpLon=wpLon;  
    }
}

/**
 * Método que ejecuta el modo Follow Me
 */
void Seguimiento::Follow_Me()
{ 
    // FOLLOW ME    
    bool endFLME=false;  
    bool navigationPaused=false;
    
    while(!endFLME)
    {
        sendTFtoMoveBase();
        switch(currentStatus)
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
                    sendGoalToMoveBase();       
                    navigationPaused=false;
                }                
                else
                {
                    if(wpReceived)
                    {
                        ROS_INFO("ENVIO WAYPOINT A MOVE BASE");
                        sendGoalToMoveBase();       
                        wpReceived=false;
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

/**
 * Método que ejecuta el modo Come To Me
 */
void Seguimiento::Come_To_Me()
{
    // COME TO ME
    bool endCTME=false;
    bool navigationPaused=false;  
    wpReceived=false;
    
    //Espero waypoint
    while(!wpReceived && currentStatus!=MOD_OFF)
    {
        sendTFtoMoveBase();
        usleep(100000);
        ros::spinOnce();        
    }      
    if(currentStatus==MOD_OFF)
        endCTME=true;
    else
    {
        ROS_INFO("RECIBO WAYPOINT DE COME TO ME");        
        sendGoalToMoveBase();
        ROS_INFO("ENVIO WAYPOINT A MOVE BASE");                     
    }

    while(!endCTME)
    {

        sendTFtoMoveBase();       
        switch(currentStatus)
        {
            case MOD_OFF:
                //Mandar stop a move base
                endCTME=true;
                sendStopToMoveBase();
                break;
            case MOD_PAUSE:
                if(!navigationPaused)
                { 
                    ROS_INFO("NAVEGACION PAUSE");  
                    sendStopToMoveBase();
                    navigationPaused=true;
                }
                break;
            case MOD_ON:
                // Obtencion del siguiente WP
                if(navigationPaused)
                {
                    ROS_INFO("NAVEGACION RESUME");                      
                    sendGoalToMoveBase();
                    navigationPaused=false;
                }
                else
                {

                    if(statusGoal==GOAL_REACHED)
                    {
                          ROS_INFO("HE LLEGADO A MI DESTINO");
                          NodeROSNavigation::publishMode(MODE_COME_TO_ME);

                          while(currentStatus!=MOD_OFF)
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

/**
 * Método que ejecuta el modo Plan
 */
void Seguimiento::Plan()
{
    // Se activa el modo PLAN
    bool navigationPaused=false;
    bool endPLAN=false;
    tf::TransformBroadcaster tfBroad;    
    
   //Espero waypoint
    planReceived=false;
    while(!endPLAN)
    {

        if(!planReceived)
        {
            hopePage=1;   
            ROS_INFO("Esperando PLAN....");
            while(!planReceived && currentStatus!=MOD_OFF)
            {
                sendTFtoMoveBase();
                usleep(100000);
                ros::spinOnce();                   
            }      
            if(currentStatus==MOD_OFF)
                endPLAN=true;
            
            else
            {
                ROS_INFO("Comienzo PLAN....");        
                wpGoal=wpGoalPlan.front();
                wpGoalPlan.pop();

                sendGoalToMoveBase();
                //while(statusGoal!=GOAL_ACTIVE)
                 //    ros::spinOnce();            
            }
        }
        else
        {
            sendTFtoMoveBase();            
            switch(currentStatus)
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
                        sendGoalToMoveBase();
                        navigationPaused=false;
                    }
                    else
                    {
                        if(statusGoal==GOAL_REACHED)
                        {
                            ROS_INFO("HE LLEGADO AL WAYPOINT");
                            if(wpGoalPlan.empty())
                            {
                                    ROS_INFO("HE LLEGADO A MI DESTINO");
                                    Common_files::msg_mode mode;
                                    mode.mode=MODE_PLAN;
                                    mode.status=MODE_FINISH;
                                    NodeROSNavigation::publishMode(MODE_FOLLOW_ME);
                                    planReceived=false;
                            }
                            else
                            {
                                    wpGoal=wpGoalPlan.front();
                                    wpGoalPlan.pop();
                                    sendGoalToMoveBase();
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

/**
 * Método que calcula la posicion actual respecto al inicio del seguimiento
 */
mapPose Seguimiento::calculatePosition(SensorGPS sensorData, geograficPose posInit)
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

/**
 * Método que calcula la posición del goal respecto a la posicion en el inicio 
 * del seguimiento 
 */
mapPose Seguimiento::calculateGoal(SensorGPS sensorData, geograficPose poseGoal)
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
