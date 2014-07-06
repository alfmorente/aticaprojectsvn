#include <Modulo_Camaras/camaras.h>
#include <Modulo_Camaras/Files.h>

using namespace std;

int main(int argc, char **argv)
{

    // Inicio de ROS
    ros::init(argc, argv, "Camaras");

    // Manejador ROS
    ros::NodeHandle n;


    // Espera activa de inicio de modulo
    int state_module=STATE_OFF;
    do{
            n.getParam("state_module_camera",state_module);
            usleep(50000);
    }while(state_module!=STATE_CONF);
    ROS_INFO("Atica CAMARAS :: Init configuration...");

    // Generación de publicadores
    sub_cam = n.subscribe<Common_files::msg_ctrl_camera>("ctrlCamera", 1000,fcn_sub_ctrl_camera);
    pub_error = n.advertise<Common_files::msg_error>("error", 1000);
  
    // Inicializacion de variable global de fin de modulo
    exitModule=false;
    Files files;
    CameraConfig configModule;

    int error=files.openFiles();
    if(error!=NO_ERROR)
    {
         //Publico el error
        Common_files::msg_error errorCAM;
        errorCAM.id_subsystem=SUBS_CAMERA;
        errorCAM.type_error=TOE_UNDEFINED;
        errorCAM.id_error=error; //por definir
        pub_error.publish(errorCAM); 
        /**if(error==COMM_LOG_FILE_ERROR)
                Files::writeErrorInLOG(error,"Fichero LOG data: ");
        else if(error==COMM_CONFIG_FILE_ERROR)
                Files::writeErrorInLOG(error,"Fichero de configuracion: ");**/
        exit(1);
    }
    
    ROS_INFO("Leyendo fichero de configuracion...");
    error =files.readConfig(&configModule);
    if(error!=NO_ERROR)
    {
         //Publico el error
        Common_files::msg_error errorCOM;
        errorCOM.id_subsystem=SUBS_CAMERA;
        errorCOM.type_error=TOE_UNDEFINED;
        errorCOM.id_error=error; 
        pub_error.publish(errorCOM);    
        Files::writeErrorInLOG(error,"Fichero de configuracion: ");
        exit(1);
    } 

    ROS_INFO("Conectando con el dispositivo...%s",configModule.portName);
    // Conexion con dispositivo
    camara=new ProtPelcoD(configModule.idCamera,configModule.velPAN,configModule.velTILT);
    if(!camara->connect(configModule.portName,configModule.portVelocity))
    {
        ROS_INFO("Error al conectar con la camara");
        Common_files::msg_error error;
        error.id_subsystem = SUBS_CAMERA;
        error.id_error=0; // TODO por definir
        error.type_error=TOE_UNDEFINED;
        pub_error.publish(error);
        return 1;
    }

    // Configuracion del dispositivo

    // Todo esta correcto, lo especificamos con el correspondiente parametro
    n.setParam("state_module_camera",STATE_OK);
    cameraAlive=true;
    ROS_INFO("Atica CAMARAS :: Configurado"); 

    // Espera activa de inicio general de los módulos
    int state_system;
    do
    {
      n.getParam("state_system",state_system);
      n.getParam("state_module_camera",state_module);
      if(state_module==STATE_OFF)
      {
            ROS_INFO("ATICA COMMUNICATION VEHICLE:: Module finish for error in other module");
            exit(1);
      }   
      usleep(500000);
    }while(state_system!=STATE_SYSTEM_ON);
    
    ROS_INFO("Atica CAMARAS :: funcionando");    
    while (ros::ok() && !exitModule)
    {
        if(cameraAlive)
        {
           n.getParam("state_module_camera",state_module);
           if(state_module==STATE_OFF)
                  exitModule=true;
        }
        else
        {
            Common_files::msg_error error;
            error.id_subsystem = SUBS_CAMERA;
            error.id_error=0; // TODO por definir
            error.type_error=TOE_UNDEFINED;
            pub_error.publish(error);
            exitModule=true;
        }
        usleep(25000);
        ros::spinOnce();
    }
    camara->disconnect();
    cout << "Atica CAMARAS :: Módulo finalizado" << endl;
    return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

void fcn_sub_ctrl_camera(Common_files::msg_ctrl_camera msg)
{
    if(msg.id_control==CAMERA_PAN)
        camara->commandPAN(msg.value);
    else if(msg.id_control==CAMERA_TILT)
        camara->commandTILT(msg.value);
    else if(msg.id_control==CAMERA_ZOOM)
        camara->commandZOOM(msg.value);    
}

