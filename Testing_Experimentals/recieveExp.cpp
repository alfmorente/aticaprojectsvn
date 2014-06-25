#include "recieveExp.h"

/*******************************************************************************
 EXP 1. SET USV REMOTE CONTROL
 ******************************************************************************/
void fcn_receive_exp1(OjCmpt comp, JausMessage msg){
    printf("*****************************");
    printf("Recibido mensaje: SET USV REMOTE CONTROL (EXP #1)");
    SetUSVRemote1Message msgExp = setUSVRemote1MessageFromJausMessage(msg);
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_1_PV_RPM_ORDER_BIT)){
        printf("Recibido parámetro: ORDER_RPM con valor: %f",msgExp->rpm_order);
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_1_PV_RUDDER_ANGLE_BIT)){
        printf("Recibido parámetro: RUDDER_ANGLE con valor: %f",msgExp->rudder_angle);
    }
}
void fcn_receive_exp2(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp3(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp4(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp5(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp6(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp7(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp8(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp10(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp11(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp12(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp13(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp14(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp15(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp16(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp17(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp18(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp19(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp20(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp21(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp22(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp23(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp24(OjCmpt comp, JausMessage msg){}
void fcn_receive_exp25(OjCmpt comp, JausMessage msg){}
