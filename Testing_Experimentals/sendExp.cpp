#include "sendExp.h"

/*******************************************************************************
 EXP 1. SET USV REMOTE CONTROL
 ******************************************************************************/

void send_msg_exp1(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetUSVRemote1Message msgExp = SetUSVRemote1Message();
    msgExp->rpm_order = 2000;
    msgExp->rudder_angle = -30;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    ojCmptSendMessage(comp, setUSVRemote1MessageToJausMessage(msgExp));
    // Liberación de memoria
    setUSVRemote1MessageDestroy(msgExp);
}

void send_msg_exp1_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetUSVRemote1Message msgExp = SetUSVRemote1Message();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->rpm_order = 2000;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    ojCmptSendMessage(comp, setUSVRemote1MessageToJausMessage(msgExp));
    
    // Segundo parametro
    msgExp = SetUSVRemote1Message();
    msgExp->presenceVector = 0x02;
    msgExp->rudder_angle = -30;
    // Envio el mensaje JAUS
    ojCmptSendMessage(comp, setUSVRemote1MessageToJausMessage(msgExp));
    
    // Liberación de memoria
    setUSVRemote1MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 2. REPORT USV REMOTE CONTROL
 ******************************************************************************/

void send_msg_exp2(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp3(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp4(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp5(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp6(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp7(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp8(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp10(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp11(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp12(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp13(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp14(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp15(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp16(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp17(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp18(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp19(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp20(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp21(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp22(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp23(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp24(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp25(OjCmpt comp, JausAddress jAdd){}
// Envio test PV

void send_msg_exp2_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp3_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp4_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp5_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp6_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp7_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp8_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp10_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp11_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp12_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp13_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp14_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp15_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp16_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp17_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp18_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp19_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp20_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp21_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp22_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp23_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp24_pv(OjCmpt comp, JausAddress jAdd){}
void send_msg_exp25_pv(OjCmpt comp, JausAddress jAdd){}
