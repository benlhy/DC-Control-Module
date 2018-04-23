/*
 * rebuscan.c
 *
 *  Created on: Apr 23, 2018
 *      Author: Ben
 */

/*
 * This function sets the CAN TX message for the master to tell slave devices the angle to turn to
 */

void tellSlaveAngle(int device, int mode, int angle1, int angle2){
    ui32MsgDataTX = 0;
    ui32MsgDataTX = device<<4; // 8 bits
    ui32MsgDataTX |= mode; // 4 bits
    ui32MsgDataTX =  ui32MsgDataTX << 16; // counts
    ui32MsgDataTX |= angle1;
    ui32MsgDataTX =  ui32MsgDataTX << 16; // counts
    ui32MsgDataTX |= angle2;
    UARTprintf("Argh: %d\n",ui32MsgDataTX);
    UARTprintf("Size of: %d\n", sizeof(pui8MsgDataTX));
    int i;
    for(i = 0; i<sizeof(pui8MsgDataTX);i++){
        UARTprintf("%d\n",pui8MsgDataTX[i]);
    }

}

/*
 * This function tells the Master what the hell is going on.
 * Total number of bits: 4+4+16+16 = 40
 */
void tellMasterAngle(int device,int mode,int angle1,int angle2) {
    ui32MsgDataTX = 0;
    ui32MsgDataTX = device << 4; // 4 bits
    ui32MsgDataTX |= mode; // 4 bits
    ui32MsgDataTX =  ui32MsgDataTX << 16; // counts
    ui32MsgDataTX |= angle1;
    ui32MsgDataTX =  ui32MsgDataTX << 16; // counts
    ui32MsgDataTX |= angle2;
}


/*
 * This function sets up the order in which the controllers are ordered.
 */
void PULSEorderconfig(){
    while(interrupt_counter==0){
        UARTprintf("Waiting for command\n");
        // do nothing. It will only change if the input GPIO is interrupted OR UART is triggered, then it will be 1.
    }
    UARTprintf("Okay, counting!\n");
    SysCtlDelay(100000);
    interrupt_counter = interrupt_counter+1; // increment our counter, this is our ID
    txID = interrupt_counter;
    UARTprintf("ID is: %d\n", interrupt_counter);
    PULSEsend();
    UARTprintf("Sending complete.\n");

}


void PULSEconfig(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, WIRE_GPIO_PIN);  // make F4 an input
    GPIOPadConfigSet(GPIO_PORTD_BASE,WIRE_GPIO_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);   // enable F4's pullup, the drive strength won't affect the input
    GPIOIntTypeSet(GPIO_PORTD_BASE,WIRE_GPIO_PIN,GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTD_BASE,PortDIntHandler);
    GPIOIntEnable(GPIO_PORTD_BASE, WIRE_GPIO_INT_PIN);
}

void PULSEsend(){
    // config output here
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    int i;
    uint32_t state=0;
    for(i=0;i<interrupt_counter;i++){
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2, state);
        SysCtlDelay(1000);
        state^=GPIO_PIN_2;
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2, state);
        SysCtlDelay(1000);
    }
    GPIOIntDisable(GPIO_PORTD_BASE,WIRE_GPIO_INT_PIN); // We no longer need the interrupt
    config_flag=0; // no longer configuring
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
        uint8_t device = pui8MsgDataRX[0]&0b11110000; // first 4 bits is address
        uint8_t mode = pui8MsgDataRX[0]&0b00001111;  // next 4 bits is the mode
        uint16_t angle1;
        uint16_t angle2;
        if (mode == 1){
            // we are tracking.
            angle1 = (pui8MsgDataRX[1]<<8)|pui8MsgDataRX[2];
            angle2 = (pui8MsgDataRX[3]<<8)|pui8MsgDataRX[4];
        }
        UARTprintf("Device: %d, Mode: %d, Angle 1:%d, Angle 2:%d \n",device,mode,angle1,angle2);

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

    //ui32MsgDataTX++;
    if (txID==MASTER){
        // tell slaves what to do here
        tellSlaveAngle(3,1,180,0); // slave 3, mode 1, angle 1 = 180, angle 2 = 0
    }
    else {
        // report back to master what we are doing
        tellMasterAngle(2,1,currPosition0,currPosition1);
    }
    //SysCtlDelay(500);
}

void CANRXmsgconfig(){
    if (txID==2){ // if I am the controller
        sCANMessageRX.ui32MsgID = 0;  // accept all incoming data
    }
    else { // if I am not controller
        sCANMessageRX.ui32MsgID = 2;  // accept only commands from 2
    }
    sCANMessageRX.ui32MsgIDMask = 0;
    sCANMessageRX.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANMessageRX.ui32MsgLen = 5;
    CANMessageSet(CAN0_BASE, 2, &sCANMessageRX, MSG_OBJ_TYPE_RX);

}

void CANTXmsgconfig(uint8_t *pui8MsgData){
    sCANMessageTX.ui32MsgID = txID;
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
    //CANRetrySet(CAN0_BASE,1);
    // In this example, the CAN bus is set to 500 kHz.
    if(CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000)==0){
        while(1){
            UARTprintf("Bit rate set error!");
        }
    }
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);



}
