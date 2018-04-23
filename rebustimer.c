/*
 * rebustimer.c
 *
 *  Created on: Apr 23, 2018
 *      Author: Ben
 */





void TIMERconfig(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/4); // activate every 1/2 of a second 120/120/2 = 0.5s
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER2_BASE, TIMER_A);

}
