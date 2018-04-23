/*
 * rebuspwm.c
 *
 *  Created on: Apr 23, 2018
 *      Author: Ben
 */
#include "rebuspwm.h"

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


