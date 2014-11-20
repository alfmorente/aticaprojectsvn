
#include "JausController.h"

// Declaracion para patron Singleton
bool JausController::instanceCreated = false;
JausController *JausController::instance = NULL;

using namespace std;

/*******************************************************************************
 *******************************************************************************
 *                            PATRON SINGLETON                                 *
 *******************************************************************************
 ******************************************************************************/

JausController *JausController::getInstance(){
    if(!instanceCreated){ 
        instance = new JausController();
        instanceCreated = true;
    }
    return instance;
}


JausController::JausController() {
    subsystemController = 3; // UGV
    nodeController = 1; // Control
}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS JAUS                       *
 *******************************************************************************
 ******************************************************************************/

void JausController::initJAUS() {
    
    // Inicializacion de JAUS
    configData = new FileLoader("nodeManager.conf");
    handler = new JausHandler();
    
    try {
        
        configData = new FileLoader("nodeManager.conf");
        handler = new JausHandler();
        nm = new NodeManager(this->configData, this->handler);
        
    }catch(...){
        
        cout << "No se ha podido inicializar JAUS" << endl;
        
    }
     // DECOMENTAR CUANDO TENGA NODEMANAGER PROPIO
    /*
     * Creacion de componentes
     * 
     */
    
    // Mission Spooler
    missionSpoolerComponent = ojCmptCreate((char *) "Mission Spooler", JAUS_MISSION_SPOOLER, 1);
    if (missionSpoolerComponent == NULL) {
        cout << "No se ha podido crear el componente MISSION SPOOLER" << endl;
        exit(0);
    }else{
                
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_RUN_MISSION, 0xFF);
        
    }

    
    // Run de componentes
    ojCmptRun(missionSpoolerComponent);

}

/*******************************************************************************
 *******************************************************************************
 *                       FINALIZACION DE ARTEFACTOS JAUS                       *
 *******************************************************************************
 ******************************************************************************/

void JausController::endJAUS(){
    ojCmptDestroy(missionSpoolerComponent);
}

void JausController::sendMessage(int identifier){
    switch(identifier){
        case 1: // Run Mission
            sendRunMissionMessage();
            break;
      default:break;
    }
}

// Run Mission
void JausController::sendRunMissionMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_MISSION_SPOOLER;
    jAdd->instance = 1;
    
    RunMissionMessage spfMsg = runMissionMessageCreate();
    
    // Campo Mission ID
    int missionID;
    cout << "Selecciona el MissionID a enviar:" << endl;
    cin >> missionID;
    spfMsg->missionId = missionID;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = runMissionMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    runMissionMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje RUN MISSION" << endl;
}
