/***************************************************************************************
 * FileName:    main.c 
 * Project:     Dimmer USB PIC 32MX220 - MPLABX C languageproject
 *              For PIC 32MX220F032B - 28 pin DIP
 *  
 *              Controls four 120 VAC lamp dimmers 
 *              using zero crossing detection and timers to get duty cycle.  
 *              The zero cross detector circuit is connected to RB0
 *              and an interrupt on change occurs 120 times per second.
 *              Each of four dimmers is controlled by one TRIAC_OUT output.
 *              Duty cycle is adjusted by turning on triac 120 times per second.
 *              Turn-on is delayed using PIC timer to control lamp intensity.
 * 
 *              COMPILED WITH Microchip XC32 Compiler V1.43 and MPLABX V5.15.
 *              XC32-gcc optimization level set to 1. To set compiler optimization level in MPLABX, go to:
 *              MPLABX Project Properties / XC32 (Global Options) / XC32-gcc / Option categories: Optiomization  
 * 
 * 04-04-20     Warwick: Got four timers working independently with one AD
 *              and USB but polling zero crossing instead of using interrupt on change.
 *              Got interrupt on change working. Use DMA to transmit U2TX instead of printf
 * 
 * 04-05-20     Command string may be sent on either UART2 or USB virtual serial port.
 *              Example: 1000, 500, 200, 800, sets dimmers #1, #2, #3, #4 to 1000, 500, 200, and 800 respectively.
 *              Max brightness is 1023, min is 0. Also one pot is read continuously and 
 *              when pot reading changes by more than 32 counts, dimmer #1 is set to that value.
 ****************************************************************************************/
#define USE_USB
#define USE_28PIN_DIP
#define NUM_AD_INPUTS 1
#define LED_OUT LATAbits.LATA4
#define TRIAC_OUT1 LATBbits.LATB1 // LATBbits.LATB3
#define TRIAC_OUT2 LATBbits.LATB4
#define TRIAC_OUT3 LATBbits.LATB7
#define TRIAC_OUT4 LATBbits.LATB13
#define SYS_FREQ 60000000
#define GetPeripheralClock() SYS_FREQ 

#include "USB/usb.h"
#include "USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "Delay.h"
#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <ctype.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

#define false FALSE
#define true TRUE


/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"

DmaChannel	DmaUARTChannel = DMA_CHANNEL1;	// DMA channel to use for our example
						// NOTE: the ISR setting has to match the channel number

union convertType {
    unsigned char byte[2];
    short integer;
} convert;

short intCounter = 0;
unsigned char USBRxBuffer[MAXBUFFER + 1] = "500, 500, 500, 500, \r";
unsigned char USBTxBuffer[MAXBUFFER + 1];
unsigned char HOSTTxBuffer[MAXBUFFER + 1] = "I can tell you what you needs to do.";
unsigned char HOSTRxBuffer[MAXBUFFER + 1];
unsigned char HOSTRxBufferFull = FALSE;
unsigned char HOSTTxBufferFull = FALSE;
unsigned char USBBufferFull = false;

#define UART_TX_REG U2TXREG
#define UART_TX_IRQ _UART2_TX_IRQ  

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void UserInit(void);
void ConfigAd(void);
void putch(unsigned char ch);

unsigned short ADresult[NUM_AD_INPUTS];
unsigned short PORTBreg = 0;

unsigned short Timer1Reload, Timer2Reload, Timer3Reload, Timer4Reload, Timer5Reload;

#define true TRUE
#define false FALSE

int main(void) 
{
float area;
float theta;
long currentPotValue = 0, previousPotValue = 0;
short i, j, k, dimVal, dimmerValues[8] = {0,0,0,0};
unsigned char ch;
unsigned char numString[MAXBUFFER + 1];

    Timer1Reload = Timer2Reload = Timer3Reload = Timer4Reload = Timer5Reload = 15625;
    InitializeSystem();            
    
    while(1)
    {
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = FALSE;
            strcpy(HOSTTxBuffer, HOSTRxBuffer);
            HOSTTxBufferFull = TRUE;
        }
        if (USBBufferFull)
        {
            USBBufferFull = FALSE;
            strcpy(HOSTTxBuffer, USBRxBuffer);
            HOSTTxBufferFull = TRUE;            
        }
        if (HOSTTxBufferFull)
        {
            HOSTTxBufferFull = false;
            i = 0; k = 0; j = 0;
            do
            {                
                ch = HOSTTxBuffer[i++];
                if (ch == '>')
                {
                    ch = HOSTTxBuffer[i++];                    
                    k = ch - '0' - 1;
                    if (k < 0) k = 0;
                    if (k > 3) k = 3;
                }
                else if (isdigit(ch)) numString[j++] = ch;
                else if (j)
                {
                    numString[j++] = '\0';
                    dimVal = atoi(numString);
                    if (dimVal > 1023) dimVal = 1023;
                    if (dimVal < 0) dimVal = 0;
                    dimmerValues[k++] = dimVal;
                    j = 0;
                }
            } while(ch && i < MAXBUFFER && k < 8);

            area = (float) dimmerValues[0] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer2Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            area = (float) dimmerValues[1] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer3Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            area = (float) dimmerValues[2] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer4Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            area = (float) dimmerValues[3] / 1024;
            theta = acos ((2 * area)-1);                   
            Timer5Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            // printf(HOSTTxBuffer, "Dimmers #1: %d, #2: %d, #3: %d, #4: %d\r", dimmerValues[0], dimmerValues[1], dimmerValues[2], dimmerValues[3]);
            DmaChnSetTxfer(DmaUARTChannel, HOSTTxBuffer, (void*)&UART_TX_REG, MAXBUFFER, 1, 1);        
            DmaChnStartTxfer(DmaUARTChannel, DMA_WAIT_NOT, 0);	// force the DMA transfer: the UART TX flag it's already been active             
        }        
        currentPotValue = ADresult[0];
        if ( abs(currentPotValue-previousPotValue) > 8 )
        {                    
            previousPotValue = currentPotValue;            
            area = (float) currentPotValue / 1024;
            theta = acos ((2 * area)-1);                   
            Timer2Reload = (unsigned short) (theta * 15625 / M_PI);                 
            
            // sprintf(HOSTTxBuffer, "Pot: %d = > %d\r", currentPotValue, Timer2Reload);                    
            //DmaChnSetTxfer(DmaUARTChannel, HOSTTxBuffer, (void*)&UART_TX_REG, MAXBUFFER, 1, 1);                    
            //DmaChnStartTxfer(DmaUARTChannel, DMA_WAIT_NOT, 0);	// force the DMA transfer: the UART TX flag it's already been active                                     
        }        
        
        mAD1IntEnable(INT_ENABLED);            
    
#ifdef USE_USB           
#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) {
            USBDeviceAttach();
        }
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); 
#endif				  
        ProcessIO();
#endif
    } // end while(1)

  
} // end main()

void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 *
 *****************************************************************************/


void UserInit(void) {
    unsigned char ch;
    unsigned short dummyRead;

    mJTAGPortEnable(false); 
 
    ConfigAd();
    
    // DIGITAL OUTPUTS: 
    PORTSetPinsDigitalOut(IOPORT_A, BIT_4); 
    PORTSetPinsDigitalOut(IOPORT_B, BIT_1 | BIT_4 | BIT_7 | BIT_13);      
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0);

    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
        
    ANSELBbits.ANSB15 = 1;
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;        
    ANSELBbits.ANSB13 = 0;
    ANSELBbits.ANSB14 = 0;  

    // Set up HOST UART #2
    PPSOutput(4, RPB9, U2TX); 
    PPSInput(2, U2RX, RPB8); 
    // PPSOutput(4, RPC2, U2TX); For PIC32MX220F032D
    // PPSInput(2, U2RX, RPA8);    
    
    // Configure UART #2 (HOST UART))
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    // UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 19200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    
    
    
   	// now the TX part
	// reconfigure the channel
	DmaChnOpen(DmaUARTChannel, DMA_CHN_PRI2, DMA_OPEN_MATCH);
    DmaChnSetMatchPattern(DmaUARTChannel, '\r');	// set \r as ending character    
    
	// set the events: now the start event is the UART tx being empty
	// we maintain the pattern match mode
	DmaChnSetEventControl(DmaUARTChannel, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(UART_TX_IRQ));
	// set the transfer source and dest addresses, source and dest size and cell size     
	DmaChnSetTxfer(DmaUARTChannel, HOSTTxBuffer, (void*)&UART_TX_REG, MAXBUFFER, 1, 1);
	DmaChnSetEvEnableFlags(DmaUARTChannel, DMA_EV_BLOCK_DONE);		// enable the transfer done interrupt: pattern match or all the characters transferred
    
	INTSetVectorPriority(INT_VECTOR_DMA(DmaUARTChannel), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(DmaUARTChannel), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority
	INTEnable(INT_SOURCE_DMA(DmaUARTChannel), INT_ENABLED);		// enable the DmaUARTChannel interrupt in the INT controller    
	
	// DmaChnStartTxfer(DmaUARTChannel, DMA_WAIT_NOT, 0);	// force the DMA transfer: the UART2 tx flag it's already been active	
	// DMA Echo is complete
	INTEnable(INT_SOURCE_DMA(DmaUARTChannel), INT_DISABLED);		// disable further interrupts from the DMA controller        
     

    do {
        ch = UARTGetDataByte(HOSTuart);
    } while (ch);        

    LED_OUT = 0;   
       
    
    // Set up interrupt on change for the PORT B input pin RB0 for zero cross detection.
    CNCONBbits.ON = 1; // CN is enabled
    CNCONBbits.SIDL = 0; // CPU Idle does not affect CN operation
    CNENBbits.CNIEB0 = 1; // Enable RB0 change notice    
    // CNENBbits.CNIEB1 = 1; // Enable RB1 change notice

    // Read port B to clear mismatch condition
    dummyRead = PORTB;
    
    
    mCNSetIntPriority(2);
    mCNSetIntSubPriority(2);
    mCNBClearIntFlag();
    // mCNBIntEnable(BIT_0 | BIT_1);
    mCNBIntEnable(BIT_0);
        
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();


}//end UserInit

// handler for the DMA channel 1 interrupt used for transmitting on UART 2
void __ISR(_DMA1_VECTOR, IPL5SOFT) DmaHandler1(void)
{
	int	evFlags;				// event flags when getting the interrupt
	INTClearFlag(INT_SOURCE_DMA(DmaUARTChannel));	// release the interrupt in the INT controller, we're servicing int
	evFlags = DmaChnGetEvFlags(DmaUARTChannel);	// get the event flags
    if(evFlags & DMA_EV_BLOCK_DONE)
    { 
        // just a sanity check. we enabled just the DMA_EV_BLOCK_DONE transfer done interrupt
        DmaChnClrEvFlags(DmaUARTChannel, DMA_EV_BLOCK_DONE);
        // RS485TxBufferFull = false;
        HOSTTxBuffer[0] = '\0';
    }
}


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
#ifdef USE_USB    
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif
#endif

#ifdef USE_USB
    USBDeviceInit();
#endif
    
    UserInit();
}//end InitializeSystem

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    ;
}

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}

void USBCBStdSetDscHandler(void) {
    ;
}

void USBCBInitEP(void) {
    CDCInitEP();
}

/*
void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again, 
            //until a new suspend condition is detected.
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}
 */

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}


void ConfigAd(void) {
    
    // ---- configure and enable the ADC ----
    mPORTBSetPinsAnalogIn(BIT_15);

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

#define PARAM5  SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
                SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
                SKIP_SCAN_AN8 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 |\
                SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
#define PARAM4  ENABLE_AN9_ANA

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, IPL2AUTO) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < NUM_AD_INPUTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}

#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
unsigned char RXchar, TXchar;
static unsigned short RxIndex = 0;
static unsigned short TxIndex = 1;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                RXchar = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            RXchar = UARTGetDataByte(HOSTuart);            
            if (RXchar != 0 && RXchar != '\n')             
            {            
                /*
                if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                */
                {
                    HOSTRxBuffer[RxIndex] = RXchar; // toupper(ch);
                    RxIndex++;
                }            
                if ('\r' == RXchar) 
                {
                    HOSTRxBufferFull = TRUE;
                    HOSTRxBuffer[RxIndex] = '\0';                    
                    RxIndex = 0;
                }
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
    {           
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
#ifdef USE_USB

#define MAXUSBBUFFER 64

// USB IO is handled here:
void ProcessIO(void) 
{
    static unsigned char ch;
    static unsigned short USBRxIndex = 0;
    unsigned short i, length;
    BYTE numBytesRead;
    static unsigned short USBcounter = 0;
    unsigned char USBData[MAXUSBBUFFER];

    // If USB is not active, turn off indicator LED
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) 
    {        
        LED_OUT = 0;
        return;
    }
    
    // Flash LED to indicate USB is active
    USBcounter++;
    if (USBcounter > 4000)
    {
        USBcounter = 0;
        if (LED_OUT) LED_OUT = 0;
        else LED_OUT = 1;
    }

    numBytesRead = getsUSBUSART(USBData, MAXUSBBUFFER);
    if (numBytesRead) {
        for (i = 0; i < numBytesRead; i++) {
            ch = USBData[i];
            if (USBRxIndex < MAXBUFFER - 2) USBRxBuffer[USBRxIndex++] = ch;
            if (ch == '\r') 
            {         
                USBRxBuffer[USBRxIndex++] = '\0';
                USBRxIndex = 0;                
                if (USBUSARTIsTxTrfReady()) 
                {                    
                    length = strlen(USBRxBuffer);
                    // length = strlen(HOSTTxBuffer);
                    if (length < MAXUSBBUFFER) 
                    {
                       putUSBUSART(USBRxBuffer, length);
                       USBBufferFull = true;
                    }
                       //  putUSBUSART(HOSTTxBuffer, length);
                    else 
                    {
                        length = (USBRxBuffer, "Error: USB data %d bytes", length);
                        // length = (HOSTTxBuffer, "Error: USB data %d bytes", length);                                                
                        putUSBUSART(USBRxBuffer, length);
                    }
                }                    
            }
        }
        numBytesRead = 0;
    }
    CDCTxService();
} //end ProcessIO
#endif

void __ISR(_TIMER_1_VECTOR, IPL2AUTO) Timer1Handler(void) 
{   
    mT1ClearIntFlag(); // clear the interrupt flag      
}

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) 
{   
    mT2ClearIntFlag(); // clear the interrupt flag              
    TRIAC_OUT1 = 1;
    CloseTimer2();
}

void __ISR(_TIMER_3_VECTOR, IPL2AUTO) Timer3Handler(void) 
{   
    mT3ClearIntFlag(); // clear the interrupt flag          
    TRIAC_OUT2 = 1;
    CloseTimer3();
}

void __ISR(_TIMER_4_VECTOR, IPL2AUTO) Timer4Handler(void) 
{   
    mT4ClearIntFlag(); // clear the interrupt flag      
    TRIAC_OUT3 = 1;
    CloseTimer4();    
}

void __ISR(_TIMER_5_VECTOR, IPL2AUTO) Timer5Handler(void) 
{
    mT5ClearIntFlag(); // Clear interrupt flag     
    TRIAC_OUT4 = 1;
    CloseTimer5();    
}

// AC ZERO CROSSING INTERRUPTS HANDLED HERE
// This executes 120 times per second for 60 Hz AC.
// A zero cross detection circuit must be connected to RB0 on PORT B.
// All for triacs are shut off here each time AC goes to zero.
void __ISR(_CHANGE_NOTICE_VECTOR, IPL2AUTO) ChangeNotice_Handler(void) {

    // Step #1 - always clear the mismatch condition first
    PORTBreg = PORTB & 0x0003;

            
            TRIAC_OUT1 = 0;
            TRIAC_OUT2 = 0;
            TRIAC_OUT3 = 0;            
            TRIAC_OUT4 = 0;                        
            
            TMR2 = 0x0000;
            OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_32, Timer2Reload);
            ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    
            TMR3 = 0x0000;        
            OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_32, Timer3Reload);
            ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);           
    
            TMR4 = 0x0000;        
            OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_32, Timer4Reload);
            ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);           
    
            TMR5 = 0x0000;        
            OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_32, Timer5Reload);
            ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2);                                            
    
    // Step #2 - then clear the interrupt flag
    mCNBClearIntFlag();

}