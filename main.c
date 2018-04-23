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
// Uses timer 0 and timer 1
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
//
//
/*
 * PULSE
 * D1 - in
 * D2 - out
 *
 *
 * CAN message format:
 * Negotiating, on boot
 *
 * Bits[0:3]
 *
 * 0 - No number
 * 1 ... 7 - assigned number
 *
 * For one that is connected to UART, set flag, it is number 2
 *
 * HOW DO WE DETECT NEIGHBOUR NODES?
 * - Add 1 wire. Network node that is connected to UART will send out a pulse. Next node will subtract one from pulse.
 * - What if not connected to UART?
 *
 *
 * MODE:
 * 0. PID tuning
 * 1. PID control
 * 2. Lead-lag tuning
 * 3. Lead-lag control
 */
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
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

#define MASTER 2 // rank of the master NODE

#define WIRE_GPIO_INT_PIN GPIO_INT_PIN_1
#define WIRE_GPIO_PIN GPIO_PIN_1
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
void WIREconfig(void);
void PULSEconfig(void);

// Master-Slave communication over CAN
void tellSlaveAngle(int device,int mode,int angle1,int angle2);
void tellMasterAngle(int device,int mode,int angle1,int angle2);

// Pulse
void PULSEsend(void);
void PULSEorderconfig(void);

// Wire
void readWire(void);

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

char first_node_flag = 0;

uint8_t stateTable[4] = {
                         GPIO_PIN_0, // CW
                         GPIO_PIN_1, // CCW
                         GPIO_PIN_0+GPIO_PIN_1,//brake 1
                         0, // STOP
};

// CAN
tCANMsgObject sCANMessageTX;
uint32_t ui32MsgDataTX[2];
uint8_t *pui8MsgDataTX;

uint32_t txID=0;

tCANMsgObject sCANMessageRX;
uint32_t ui32MsgDataRX;
uint8_t *pui8MsgDataRX;

// GPIO
volatile int interrupt_counter=0;
volatile int config_flag = 1;


// Reference
// https://sites.google.com/site/luiselectronicprojects/tutorials/tiva-tutorials/tiva-gpio/digital-input-with-interrupt
void PortDIntHandler(void){
    uint32_t status=0;
    status = GPIOIntStatus(GPIO_PORTD_BASE,true);
    GPIOIntClear(GPIO_PORTD_BASE, WIRE_GPIO_INT_PIN);
    if( (status & WIRE_GPIO_INT_PIN) == WIRE_GPIO_INT_PIN){
      //Then there was a pin4 interrupt
        interrupt_counter = interrupt_counter+1;
    }

    if( (status & GPIO_INT_PIN_5) == GPIO_INT_PIN_5){
      //Then there was a pin5 interrupt
    }

}



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
        //UARTprintf("Status: 0x%08X",ui32Status);

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
        //g_ui32MsgCount++;

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
Timer2IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    ledFlag ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ledFlag<<0);

}



void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char charBuf[15];
    if (config_flag == 1){
        // we are in configuration mode
        interrupt_counter = 1;
    }


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
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    GPIOconfig();
    TIMERconfig();
    PULSEconfig();
    tellSlaveAngle(3,2,180,160);

    PULSEorderconfig(); // infinite loop until signal received
    CANconfig();
    CANRXmsgconfig();

    // Need a wait for ready pulse here? Maybe not, loss in messages has to deal with which board is reset first. WHY?





    CANTXmsgconfig(pui8MsgDataTX); // set pointer to message data.





    //setSpeed(pwmout0,dir0,motornum);
    //char floatBuf[20];
    SysCtlDelay(SysCtlClockGet());
    while (1)
    {

        // pulse complete
        // okay we received one, wait for time!

        CANget();
        CANsend();
        SysCtlDelay(SysCtlClockGet()/3);




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












void GPIOconfig(void){
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_3);


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
