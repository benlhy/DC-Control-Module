//*****************************************************************************
// This project aims to build a simple motor controller around the TM4C123GXL
//
//
//
// PID controller
// Combined PWM and QEI
// PC5 - A, PC6 - B, Module 1
// PD6 - A, PD7 - B, Module 0
// Give it the limit to count to before rolling over
// current motor is 200:1 ratio with 7 pulses per revolution
// 100 * 7 * 4 = 700 * 4 = 2800 counts/ revolution
// Multiply it by 5 so that you get 5 revolutions of counting = 2800 * 5 = 14000
// Position counter is reset on value reset or seeing an index pulse
// clocking on edges is 4x decoding mode.
//
//
// Change stack size: Arm Linker -> Basic options
//
// PWM control
// PWM5: PF1 motor 0
// PWM6: PF2 motor 1
//
// Direction pins
// PE0, PE1 - motor 0
// PE2, PE3 - motor 1
//
// Encoder pins
// PD6 - A, PD7 - B, Module 0
// PC5 - A, PC6 - B, Module 1
//
// CAN pins
// CAN reference: https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/p/657164/2416622#pi320098=3
// WARNING, CURRENT CAN DOES NOT WORK, fix here: https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/551204#pi320098=2
// PB5 TX
// PB4 RX
//
// Messages to send over can..
// We need to know how many controllers are hooked up
// Transmit on all UARTs of each controller
// B - Controller 1 - A
//          |
// B - Controller 2 - A
// Need to set:
// 1. Velocity/position control
// 2. Velocity/position to go to
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/can.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

// Definitions
#define ENCODER_COUNT_1 14000 // set up to count 5 revolutions
#define ENCODER_COUNT_2 14000
#define COUNT_PER_REV 2800
#define TIME_TO_COUNT 0.5 // for velocity prediction
// Prototypes

// Convert/set velocity
int angleToPosition(int angle);
int positionToAngle(int position);
float counttoRPM(int count);
void setSpeed(int pwm, int state, int motor); // 0 = brake, 1 = CCW, 2 = CW, 3 = STOP, motor 0 and 1

// Config
void GPIOconfig(void);
void PWMconfig(int period);
void QEIconfig(int limit1, int limit2);
void TIMERconfig(void);
void CANconfig(void);
void QEIvelocityConfig(void);
void InitConsole(void);

// CAN
void CANsend();
void CANget();
void CANTXmsgconfig(uint8_t *pui8MsgData);
void CANRXmsgconfig();

// Controllers
void PIDPosupdate(void);
void PIDVelupdate(void);

int QEI0posGet(int max_count);
int QEI0angleGet(int max_count);

int QEI1posGet(int max_count);
int QEI1angleGet(int max_count);


// Variables
volatile int pwmSpeed=1;
volatile int desiredPos0=0;
volatile int desiredPos1=0;
volatile int motornum = 1;

volatile int desiredVel0=0;
volatile int desiredVel1=0;

// Begin

volatile int currPosition0;
int dir0 = 1;
int pwmout0 = 0;
volatile int currPosition1;
int dir1 = 1;
int pwmout1 = 0;

// PID position gains
float Kp0 = 1;
float Kd0 = 0.1;
float Ki0 = 0.15;

float Kp1 = 1;
float Kd1 = 0.1;
float Ki1 = 0.15;

// Variables for 0
float error0 = 0;
int eprev0 = 0;
float ediv0 = 0;
float eint0 = 0;
int controlsig0 = 0;

// Variables for 1
float error1 = 0;
int eprev1 = 0;
float ediv1 = 0;
float eint1 = 0;
int controlsig1 = 0;

// Velocity

volatile int currVel0;
int dirv0 = 1;
int pwmoutv0 = 0;
volatile int currVel1;
int dirv1 = 1;
int pwmoutv1 = 0;

// PID velocity gains
float Kpv0 = 1;
float Kdv0 = 0.1;
float Kiv0 = 0.15;

float Kpv1 = 1;
float Kdv1 = 0.1;
float Kiv1 = 0.15;

// Variables for 0
float errorv0 = 0;
int eprevv0 = 0;
float edivv0 = 0;
float eintv0 = 0;
int controlsigv0 = 0;

// Variables for 1
float errorv1 = 0;
int eprevv1 = 0;
float edivv1 = 0;
float eintv1 = 0;
int controlsigv1 = 0;

uint32_t ledFlag = 1;

uint8_t stateTable[4] = {
                         GPIO_PIN_0, // CW
                         GPIO_PIN_1, // CCW
                         GPIO_PIN_0+GPIO_PIN_1,//brake 1
                         0, // STOP
};

// CAN
tCANMsgObject sCANMessageTX;
uint32_t ui32MsgDataTX;
uint8_t *pui8MsgDataTX;

tCANMsgObject sCANMessageRX;
uint32_t ui32MsgDataRX;
uint8_t *pui8MsgDataRX;


// Added to github

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;


//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        UARTprintf("Status: 0x%08X",ui32Status);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //

    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32MsgCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    else if(ui32Status == 2)
    {
        //UARTprintf("Interrupt reached!");
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32MsgCount++;

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
        //UARTprintf("Okay done.\n");
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        UARTprintf("Errortime\n");
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    ledFlag ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ledFlag<<3);

}



void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char charBuf[15];

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    // We need to clear the flag otherwise it will not interrupt again
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    int count = 0;
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //


        char newchar = UARTCharGet(UART0_BASE);
        charBuf[count] = newchar;
        count++;

    }

    sscanf(charBuf,"%d %d %d %d", &desiredPos0,&desiredPos1,&desiredVel0,&desiredVel1);
    desiredPos0 = angleToPosition(desiredPos0);
    desiredPos1 = angleToPosition(desiredPos1);
    //UARTprintf("%d\n",pwmSpeed); // send it back.

}


int main(void) {
    pui8MsgDataTX=(uint8_t *)&ui32MsgDataTX; // point it to ui32MsgData because CAN only deals in 8 bytes
    pui8MsgDataRX=(uint8_t *)&ui32MsgDataRX; // point it to ui32MsgData because CAN only deals in 8 bytes
    //ui32MsgDataTX = 0;

    // Set the clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    FPULazyStackingEnable();
    FPUEnable();

    IntMasterEnable();
    PWMconfig(320);
    QEIconfig(ENCODER_COUNT_1-1,ENCODER_COUNT_1-1); // zero based
    QEIvelocityConfig();
    InitConsole();
    GPIOconfig();
    TIMERconfig();
    CANconfig();
    CANTXmsgconfig(pui8MsgDataTX); // set pointer to message data.
    CANRXmsgconfig();
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);



    //setSpeed(pwmout0,dir0,motornum);
    char floatBuf[20];


    while (1)
    {
        CANsend();
        CANget();


        //PIDPosupdate();
        //PIDVelupdate();

        //currVel1 = counttoRPM(QEIVelocityGet(QEI1_BASE));
        //currVel0 = counttoRPM(QEIVelocityGet(QEI0_BASE));
        //UARTprintf("Current speed is:");
        //int mycount = QEIVelocityGet(QEI1_BASE);
        //float count1=counttoRPM(mycount);
        //float count2=counttoRPM(QEIVelocityGet(QEI0_BASE));
        //sprintf(floatBuf,"%d %f\n",mycount,counttoRPM(mycount));
        //UARTprintf("%d %s",mycount,floatBuf);


        //UARTprintf("%d %d\n",(QEIVelocityGet(QEI0_BASE)),QEIVelocityGet(QEI1_BASE));



        //UARTprintf("curr_angle0:%d pwmSig0:%d error0:%d curr_angle1:%d pwmSig1:%d error1:%d curr_vel0:%d curr_vel1:%d \n",positionToAngle(currPosition0),pwmout0,(int)error0,positionToAngle(currPosition1),pwmout1,(int)error1,currVel0,currVel1);

        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dir);
        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,pwmout);
        //setSpeed(pwmout1,dir1,1);
        //setSpeed(pwmout0,dir0,0);


        //SysCtlDelay (1000);
    }
}
void CANget(){

    unsigned int uIdx;

    //
    // If the flag is set, that means that the RX interrupt occurred and
    // there is a message ready to be read from the CAN
    //
    if(g_bRXFlag)
    {
        // UARTprintf("Function reached\n");
        //
        // Reuse the same message object that was used earlier to configure
        // the CAN for receiving messages.  A buffer for storing the
        // received data must also be provided, so set the buffer pointer
        // within the message object.
        //
        sCANMessageRX.pui8MsgData = pui8MsgDataRX;

        //
        // Read the message from the CAN.  Message object number 2 is used
        // (which is not the same thing as CAN ID).  The interrupt clearing
        // flag is not set because this interrupt was already cleared in
        // the interrupt handler.
        //
        CANMessageGet(CAN0_BASE, 2, &sCANMessageRX, 0);

        //
        // Clear the pending message flag so that the interrupt handler can
        // set it again when the next message arrives.
        //
        g_bRXFlag = 0;


        //
        // Check to see if there is an indication that some messages were
        // lost.
        //
        if(sCANMessageRX.ui32Flags & MSG_OBJ_DATA_LOST)
        {
            UARTprintf("CAN message loss detected\n");
        }

        //
        // Print out the contents of the message that was received.
        //
        UARTprintf("Msg ID=0x%08X len=%u data=0x",
                   sCANMessageRX.ui32MsgID, sCANMessageRX.ui32MsgLen);
        for(uIdx = 0; uIdx < sCANMessageRX.ui32MsgLen; uIdx++)
        {
            UARTprintf("%02X ", pui8MsgDataRX[uIdx]);
        }
        UARTprintf(" total count=%u\n", g_ui32MsgCount);
    }
    //UARTprintf("Oops, flag not set!\n");

}


void CANsend() {
    //
    // Data structure:
    // Address (to/from)
    // Command
    // Data
    //
    //UARTprintf("Sending msg: 0x%02X %02X %02X %02X",
    //           pui8MsgDataTX[0], pui8MsgDataTX[1], pui8MsgDataTX[2],
    //           pui8MsgDataTX[3]);
    //UARTprintf("data is %d",ui32MsgDataTX);
    //
    // Send the CAN message using object number 1 (not the same thing as
    // CAN ID, which is also 1 in this example).  This function will cause
    // the message to be transmitted right away.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessageTX, MSG_OBJ_TYPE_TX);
    //
    // Check the error flag to see if errors occurred
    //
    if(g_bErrFlag)
    {
        UARTprintf(" error - cable connected?\n");
    }
    else
    {
        //
        // If no errors then print the count of message sent
        //
        //UARTprintf(" total count = %u\n", g_ui32MsgCount);
    }

    //
    // Increment the value in the message data.
    //
    ui32MsgDataTX++;
}

void CANRXmsgconfig(){
    sCANMessageRX.ui32MsgID = 0;
    sCANMessageRX.ui32MsgIDMask = 0;
    sCANMessageRX.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANMessageRX.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, 2, &sCANMessageRX, MSG_OBJ_TYPE_RX);

}

void CANTXmsgconfig(uint8_t *pui8MsgData){
    sCANMessageTX.ui32MsgID = 1;
    sCANMessageTX.ui32MsgIDMask = 0;
    sCANMessageTX.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessageTX.ui32MsgLen = sizeof(pui8MsgDataTX);
    sCANMessageTX.pui8MsgData = pui8MsgDataTX;
    CANMessageSet(CAN0_BASE, 1, &sCANMessageTX, MSG_OBJ_TYPE_TX);

}

void CANconfig(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_6); // standby pin
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0); // HIGH = standby, LOW = active

    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    CANInit(CAN0_BASE);
    // In this example, the CAN bus is set to 500 kHz.
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 5000);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);



}

float counttoRPM(int count){
     return (float)count/(COUNT_PER_REV*TIME_TO_COUNT);
}

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

void TIMERconfig(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/4); // activate every 1/2 of a second 120/120/2 = 0.5s
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); // LED for the timer

}

void setSpeed(int pwm, int state, int motor){ // 0 to CW, // 1 to CCW // 2 to brake // 4 to stop
    if (motor==0){ // motor 1
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,pwm);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1,stateTable[state]);
    }
    else if (motor == 1){
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,pwm);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_3,stateTable[state]<<2);
    }
}

int angleToPosition(int angle){
    return (int)((float)angle/360.0*COUNT_PER_REV);
}

int positionToAngle(int position){
    return (int)((float)position/(float)COUNT_PER_REV*360);
}

int QEI0angleGet(int max_count){
    return (int)((float)QEI0posGet(max_count)/(float)max_count*360.0*5);
}

int QEI0posGet(int max_count){
   int curr_pos = QEIPositionGet(QEI0_BASE);
   curr_pos = curr_pos - max_count/2;
   return curr_pos;
}

void QEI0zero(int max_count){
    QEIPositionSet(QEI0_BASE,max_count/2);
}


int QEI1angleGet(int max_count){
    int some_val = (int)(((float)QEI1posGet(max_count)/(float)max_count)*360.0*5);
    return some_val;

}

int QEI1posGet(int max_count){
   int curr_pos = QEIPositionGet(QEI1_BASE);
   curr_pos = curr_pos - max_count/2;
   return curr_pos;
}

void QEI1zero(int max_count){
    QEIPositionSet(QEI1_BASE,max_count/2);
}

void QEIconfig(int limit1, int limit2){
    // QEI Config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // We are using PD7 for PhB0, QEI module 0 phase B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // PC5 - A,PC6 - B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0); // QEI module 0 phase A
    GPIOPinConfigure(GPIO_PD7_PHB0); // QEI module 0 phase B

    //Set Pins to be PHA1 and PHB1
    GPIOPinConfigure(GPIO_PC5_PHA1); // QEI module 1 phase A
    GPIOPinConfigure(GPIO_PC6_PHB1); // QEI module 1 phase B

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);

    //Disable peripheral and int before configuration
    QEIDisable(QEI0_BASE);

    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), limit1);
    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, limit1/2);

    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), limit2);
    // Enable the quadrature encoder.
    QEIEnable(QEI1_BASE);
    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI1_BASE, limit2/2);
}

void QEIvelocityConfig(){
    QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,SysCtlClockGet()*TIME_TO_COUNT); // base, predivider, clock ticks to count for
    QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,SysCtlClockGet()*TIME_TO_COUNT);
    QEIVelocityEnable(QEI0_BASE);
    QEIVelocityEnable(QEI1_BASE);
    // QEIVelocityGet(QEI0_BASE); // returns the number of pulses captured in the time period
}

void PWMconfig(int period){

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);


    GPIOPinConfigure(GPIO_PF1_M1PWM5); //PF1
    GPIOPinConfigure(GPIO_PF2_M1PWM6); //PF2

    // Set pin types
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2); // yep
    // PWM configuration
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,100);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT , true);
}




void GPIOconfig(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); // AN1
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2); // AN1
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3); // AN1
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // AN1


}

void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
    UARTprintf("Console init complete.");
}
