/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 25 de junio de 2014, 9:34
 */

#include <cstdlib>
#include <jaus.h>
#include <openJaus.h>
#include <iostream>
// inclusion de funciones de recepcion
#include "recieveExp.h"
// inclusion de funciones para envio
#include "sendExp.h"


using namespace std;

/*
 * 
 */
int state; //estado del componente
int cont = 0; //contador de mensajes recibidos

//Función para estado ready
void fcn_state_ready(OjCmpt gpos);

//Función para estado shutdown
void fcn_state_shutdown(OjCmpt gpos);

//Funcion que recibe un report
void fcn_receive_report(OjCmpt comp, JausMessage msg);

//Funcion que recibe un mensaje cualquiera JAUS
void fcn_receive_any(OjCmpt comp, JausMessage msg);

int main(int argc, char** argv) {
    cout << "********************************" << endl;
    cout << "*         Client START         *" << endl;
    cout << "********************************" << endl;
    //Pasos a seguir:

    //1. Creacion del componente
    OjCmpt clientCmpt = ojCmptCreate((char *)"Primitive Driver", JAUS_PRIMITIVE_DRIVER, 1);
    if (clientCmpt == NULL) {
        cout << "(Client) Fail to create component for Experimental testing" << endl;
        exit(0);
    }
    cout << "(Client) Component created" << endl;

    //2. Servicios del componente
    ojCmptAddService(clientCmpt, JAUS_GLOBAL_POSE_SENSOR);
    // Mensajes que envia    
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_USV_REMOTE_1, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_USV_REMOTE_CONTROL_2, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_USV_INFO_3, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_ADITIONAL_GPSINS_INFO_4, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_ANEMOMETER_INFO_5, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SCPM_INFO_6, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_USV_OBSERVATIONS_CONFIG_7, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_USV_OBSERVATIONS_CONFIG_8, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_TELEMETER_INFO_10, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_SCIENTIFICS_OPERATIONS_11, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_UGV_INFO_12, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_SCIENTIFIC_OPERATIONS_13, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_LIST_OF_WAYPOINTS_14, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_LIST_OF_WAYPOINTS_15, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_HEARTBEAT_CHANNEL_STATE_16, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_SIGNALING_ELEMENTS_18, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_POSITIONER_19, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_POSITIONER_20, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DAY_TIME_CAMERA_21, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DAY_TIME_CAMERA_22, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_NIGHT_TIME_CAMERA_23, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_NIGHT_TIME_CAMERA_24, 0xFF);
    ojCmptAddServiceOutputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_SIGNALING_ELEMENTS_25, 0xFF);
    // Mensajes que recibe    
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_USV_REMOTE_1, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_USV_REMOTE_CONTROL_2, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_USV_INFO_3, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_ADITIONAL_GPSINS_INFO_4, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_ANEMOMETER_INFO_5, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SCPM_INFO_6, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_USV_OBSERVATIONS_CONFIG_7, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_USV_OBSERVATIONS_CONFIG_8, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_TELEMETER_INFO_10, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_SCIENTIFICS_OPERATIONS_11, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_UGV_INFO_12, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_SCIENTIFIC_OPERATIONS_13, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_LIST_OF_WAYPOINTS_14, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_LIST_OF_WAYPOINTS_15, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_HEARTBEAT_CHANNEL_STATE_16, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_SIGNALING_ELEMENTS_18, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_POSITIONER_19, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_POSITIONER_20, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DAY_TIME_CAMERA_21, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DAY_TIME_CAMERA_22, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_SET_NIGHT_TIME_CAMERA_23, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_NIGHT_TIME_CAMERA_24, 0xFF);
    ojCmptAddServiceInputMessage(clientCmpt, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_SIGNALING_ELEMENTS_25, 0xFF);
    
    cout << "(Client) Component: Services added" << endl;

    //3. Configurar funciones que implementan las actividades del componente

    //3.1 Funcion asociada a la ejecución de la maquina de estado según estado del componente
    ojCmptSetStateCallback(clientCmpt, JAUS_READY_STATE, fcn_state_ready);
    ojCmptSetStateCallback(clientCmpt, JAUS_SHUTDOWN_STATE, fcn_state_shutdown);


    //3.2 Función asociada a la recepción de mensajes
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_USV_REMOTE_1, fcn_receive_exp1);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_USV_REMOTE_CONTROL_2, fcn_receive_exp2);
    ojCmptSetMessageCallback(clientCmpt, JAUS_USV_INFO_3, fcn_receive_exp3);
    ojCmptSetMessageCallback(clientCmpt, JAUS_ADITIONAL_GPSINS_INFO_4, fcn_receive_exp4);
    ojCmptSetMessageCallback(clientCmpt, JAUS_ANEMOMETER_INFO_5, fcn_receive_exp5);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SCPM_INFO_6, fcn_receive_exp6);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_USV_OBSERVATIONS_CONFIG_7, fcn_receive_exp7);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_USV_OBSERVATIONS_CONFIG_8, fcn_receive_exp8);
    ojCmptSetMessageCallback(clientCmpt, JAUS_TELEMETER_INFO_10, fcn_receive_exp10);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_SCIENTIFICS_OPERATIONS_11, fcn_receive_exp11);
    ojCmptSetMessageCallback(clientCmpt, JAUS_UGV_INFO_12, fcn_receive_exp12);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_SCIENTIFIC_OPERATIONS_13, fcn_receive_exp13);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_LIST_OF_WAYPOINTS_14, fcn_receive_exp14);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_LIST_OF_WAYPOINTS_15, fcn_receive_exp15);
    ojCmptSetMessageCallback(clientCmpt, JAUS_HEARTBEAT_CHANNEL_STATE_16, fcn_receive_exp16);
    ojCmptSetMessageCallback(clientCmpt, JAUS_HEARTBEAT_POSITION_INFO_17, fcn_receive_exp17);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_SIGNALING_ELEMENTS_18, fcn_receive_exp18);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_POSITIONER_19, fcn_receive_exp19);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_POSITIONER_20, fcn_receive_exp20);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_DAY_TIME_CAMERA_21, fcn_receive_exp21);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_DAY_TIME_CAMERA_22, fcn_receive_exp22);
    ojCmptSetMessageCallback(clientCmpt, JAUS_SET_NIGHT_TIME_CAMERA_23, fcn_receive_exp23);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_NIGHT_TIME_CAMERA_24, fcn_receive_exp24);
    ojCmptSetMessageCallback(clientCmpt, JAUS_REPORT_SIGNALING_ELEMENTS_25, fcn_receive_exp25);
    //ojCmptSetMessageProcessorCallback(clientCmpt,fcn_receive_any); 
    //cout << "(Client) Component Global Pose: Configurated" << endl;
    
    //4  Cambiar estado del componente
    ojCmptSetState(clientCmpt, JAUS_READY_STATE);
    cout << "(Client) Component: state READY" << endl;

    //5  Poner en ejecución el componente
    if (ojCmptRun(clientCmpt) != 0) {
        cout << "(Client) Failed to run Component Global Pose" << endl;
        exit(0);
    }
    cout << "(Client) Component Global Pose: RUN" << endl;


    cout << "Set Intro to shutdown Component" << endl;
    int c = getchar();
    ojCmptSetState(clientCmpt, JAUS_SHUTDOWN_STATE);
    while (state != JAUS_SHUTDOWN_STATE)
        usleep(100000);

    //se destruye el componente
    ojCmptDestroy(clientCmpt);
    cout << "(Client) Component: Destroyed" << endl;
    cout << "*********************************" << endl;
    cout << "*         Client FINISH         *" << endl;
    cout << "*********************************" << endl;
    return 0;
}

void fcn_state_ready(OjCmpt comp) {
    if (state != JAUS_READY_STATE)
        state = JAUS_READY_STATE;
    
/*******************************************************************************
 EXP 1. MONTAJE DIRECCION
 ******************************************************************************/ 
    //Se crea la direccion destino del Mensaje a enviar
    JausAddress destino;
    destino = jausAddressCreate();
    destino->subsystem = 1; //Subsistema
    destino->node = 1; //Nodo
    destino->component = JAUS_GLOBAL_POSE_SENSOR;
    destino->instance = 1; //Instancia
    
/*******************************************************************************
 EXP 1. SET USV REMOTE CONTROL
 ******************************************************************************/    
    send_msg_exp1(comp, destino);
    send_msg_exp1_pv(comp, destino);        
            
    
    
    // Liberacion de memoria
    jausAddressDestroy(destino);
    
}

//Función para componente en Standby

void fcn_state_shutdown(OjCmpt gpos) {
    if (state != JAUS_SHUTDOWN_STATE) {
        state = JAUS_SHUTDOWN_STATE;
        cout << "(Client) Component: state SHUTDOWN" << endl;

    }
}

