/*
 * rebuspid.c
 *
 *  Created on: Apr 23, 2018
 *      Author: Ben
 */




void PIDVelupdate(){

    currVel1 = QEIVelocityGet(QEI1_BASE);
    currVel0 = QEIVelocityGet(QEI0_BASE);

    errorv0 = (desiredVel0 - currVel0); // get current error
    eintv0 = eintv0 + errorv0; // integrate up the error
    edivv0 = errorv0-eprevv0; // no need to divide by time since you are multiplying by a constant.
    controlsigv0 = Kp0*errorv0+Kd0*edivv0+Ki0*eintv0;

    if (eintv0>200){
        eintv0 = 200;
    }
    else if(eintv0<-200){
        eintv0 = -200;
    }

    if (controlsigv0>100) {
        pwmoutv0 = 100;
        dirv0 = 0; // forward full speed
    }
    else if(controlsigv0<-100) {
        dirv0 = 1;
        pwmoutv0 = 100; // backwards full speed
    }
    else if((controlsigv0<0) && (controlsigv0>-100)){
        pwmoutv0 = -1*controlsigv0; // backwards at controlsig
        dirv0 = 1;
    }
    else if ((controlsigv0>0)&&(controlsigv0<100)){
        pwmoutv0 = controlsigv0; // forwards at controlsig
        dirv0 = 0;
    }

    errorv1 = (desiredVel1 - currVel1); // get current error
    eintv1 = eintv1 + errorv1; // integrate up the error
    edivv1 = errorv1-eprevv1; // no need to divide by time since you are multiplying by a constant.
    controlsigv1 = Kp1*errorv1+Kd1*edivv1+Ki1*eintv1;

    if (eintv1>200){
        eintv1 = 200;
    }
    else if(eintv1<-200){
        eintv1 = -200;
    }

    if (controlsigv1>100) {
        pwmoutv1 = 100;
        dirv1 = 0; // forward full speed
    }
    else if(controlsigv1<-100) {
        dirv1 = 1;
        pwmoutv1 = 100; // backwards full speed
    }
    else if((controlsigv1<0) && (controlsigv1>-100)){
        pwmoutv1 = -1*controlsigv1; // backwards at controlsig
        dirv1 = 1;
    }
    else if ((controlsigv1>0)&&(controlsigv1<100)){
        pwmoutv1 = controlsigv1; // forwards at controlsig
        dirv1 = 0;
    }
    eprev0 = errorv0; // update previous error
    eprev1 = errorv1; // update previous error

}

void PIDPosupdate(){

    currPosition1 = QEI1posGet(ENCODER_COUNT_1-1);
    currPosition0 = QEI0posGet(ENCODER_COUNT_1-1);

    error0 = (desiredPos0 - currPosition0); // get current error
    eint0 = eint0 + error0; // integrate up the error
    ediv0 = error0-eprev0; // no need to divide by time since you are multiplying by a constant.
    controlsig0 = Kp0*error0+Kd0*ediv0+Ki0*eint0;

    if (eint0>200){
        eint0 = 200;
    }
    else if(eint0<-200){
        eint0 = -200;
    }

    if (controlsig0>100) {
        pwmout0 = 100;
        dir0 = 0; // forward full speed
    }
    else if(controlsig0<-100) {
        dir0 = 1;
        pwmout0 = 100; // backwards full speed
    }
    else if((controlsig0<0) && (controlsig0>-100)){
        pwmout0 = -1*controlsig0; // backwards at controlsig
        dir0 = 1;
    }
    else if ((controlsig0>0)&&(controlsig0<100)){
        pwmout0 = controlsig0; // forwards at controlsig
        dir0 = 0;
    }

    error1 = (desiredPos1 - currPosition1); // get current error
    eint1 = eint1 + error1; // integrate up the error
    ediv1 = error1-eprev1; // no need to divide by time since you are multiplying by a constant.
    controlsig1 = Kp1*error1+Kd1*ediv1+Ki1*eint1;

    if (eint1>200){
        eint1 = 200;
    }
    else if(eint1<-200){
        eint1 = -200;
    }

    if (controlsig1>100) {
        pwmout1 = 100;
        dir1 = 0; // forward full speed
    }
    else if(controlsig1<-100) {
        dir1 = 1;
        pwmout1 = 100; // backwards full speed
    }
    else if((controlsig1<0) && (controlsig1>-100)){
        pwmout1 = -1*controlsig1; // backwards at controlsig
        dir1 = 1;
    }
    else if ((controlsig1>0)&&(controlsig1<100)){
        pwmout1 = controlsig1; // forwards at controlsig
        dir1 = 0;
    }
    eprev0 = error0; // update previous error
    eprev1 = error1; // update previous error

}
