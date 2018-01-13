/*
 *  @Module_Name: UARTDriver
 *  @Module_Description: This module handles all the UART TX and RX using Interrupts
 * 
 *  @Configuration:
 * 
 *  In order to configure this driver, following constants have to be changed in this source file
 * 
 *      Interrupt Priority Level: P_UART_IPL_LEVEL
 *      Baud Rate: BAUD_RATE
 *      RX Pin: U1RXR
 *      TX Pin: RPF0R
 *      Buffer Size: UART_BUFF_SIZE
 */

//==================== Preprocessor Directives ==================//

#include "UARTDriver.h"
#include "System.h"


//=========================== CONSTANTS =========================//


#define     P_UART_IPL_LEVEL    7                                             //Priotiry Level For UART ISR
#define     BAUD_RATE           115200                                        //Target Baud Rate. This Baud rate is specified based on Bluetooth Chip.

#define     UBRG_BAUD_RATE (((PB_FREQ + (2 * BAUD_RATE))/(4 * BAUD_RATE))-1); //Micro to generate Baud Rate as a fuction of PBCLK Frequency
                                                                              //This UBRG number is being generated based on formula in the PIC32 Family Reference Manual
                                                                              //This formula can be found on chapter 21 (UART) - section 3 (UART BAUD RATE GENERATOR) - Page 12
       
#define     UP_TX_PIN           BIT(0)                                        //This is simply Binary number 1. It is used to Clear TX pin before configuration.
#define     UP_RX_PIN           BIT(0)                                        //This is simply Binary number 1. It is used to Clear RX pin before configuration.

#define     MAP_RX_PIN()        U1RXR = 4                                     //RX on RC14/RPC14(4)  This pin is specified on PCB schematics
#define     MAP_TX_PIN()        RPF0R = 3                                     //TX on RC13/RPC13(3)  This pin is specified on PCB schematics


//Set TX and RX Pins in safe state 

#define     SET_PINS_TO_SAFE_STATE(){           \
                LATFCLR  = UP_TX_PIN;           \
                TRISFCLR = UP_TX_PIN;           \
                TRISFSET = UP_RX_PIN;           \
            }(void)0 

//Set the software buffer size. Both TX and RX buffers have this size

#define     UART_BUFF_SIZE 128           


//==================== Data Structures =========================//


uint8_t Rx_Buffer[UART_BUFF_SIZE];      // RX FIFO Buffer
unsigned int Rx_Out_Pos;                // When we want to get data from buffer, we use this index.
volatile unsigned int Rx_In_Pos;        // When interrupt gets new incoming byte, assigns it in RX buffer using this index
volatile unsigned int Rx_Num_Bytes;     // RX Number of bytes currently in the buffer

uint8_t Tx_Buffer[UART_BUFF_SIZE];      // TX FIFO Buffer
unsigned int Tx_In_Pos;                 // When there is data to be transferred, it gets stored into TX buffer using this index.
volatile unsigned int Tx_Out_Pos;       // When TX interrup handler want to take data from buffer and transmit, it uses this index.


/***************************************************************************
 * Function :  UART1InitializeDriver
 * Input    :  none
 * Output   :  none
 * Comment  :  Initialize the UART
 *             We want to set port pins in safe state first Set Pin Direction (this 
 *             may seem Redundant, but we do this to keep the pins in a safe state
 ***************************************************************************/

void UART1InitializeDriver(void){
    
    SET_PINS_TO_SAFE_STATE();                           //Set pins to safe state first
    MAP_RX_PIN();                                       //Then assign registers to RX and TX pins
    MAP_TX_PIN();                                       

    U1MODE = 0;                                         //Fresh U1MODE - we want to have a clean slate register. U1MODE handles the UART enablement and configuration
    U1BRG  = UBRG_BAUD_RATE;                            //Configure the Baud Rate

    U1MODE = _U1MODE_BRGH_MASK | _U1MODE_ON_MASK;       //Set U1MODE Register for High Speed and enable U1MODE. We are using masks with bitwise OR operations.
    
    U1STA  = _U1STA_UTXEN_MASK | _U1STA_URXEN_MASK |    //Enable TX and RX by using masks with bitwise OR operations
                    (0x2 << _U1STA_UTXISEL_POSITION);   //Sets the transmit Interrupt mode to 10. 
                                                        //In this mode, interrupt is generated when the transmit buffer becomes empty

    
    Rx_Num_Bytes = 0;                                   //Set to number of bytes available to read to 0 
    Rx_In_Pos = Rx_Out_Pos = 0;                         //Initialize RX Buffer Variables 
    Tx_In_Pos = Tx_Out_Pos = 0;  
    
    memset(Rx_Buffer, 0 ,sizeof(Rx_Buffer));          

    IPC7bits.U1IP = P_UART_IPL_LEVEL;                   //Set Interrupt Priority To 7
    IFS1CLR = _IFS1_U1RXIF_MASK | _IFS1_U1TXIF_MASK;    //clear Interrupt TX and RX flags
    IEC1SET = _IEC1_U1RXIE_MASK | _IEC1_U1TXIE_MASK;    //Enable the TX & RX Interrupts
    
}

/***************************************************************************
 * Function :  repushTx
 * Input    :  none
 * Output   :  none
 * Comment  :  Restart the sending of characters (helper routine)
 ***************************************************************************/

inline
void repushTx(void){
    
    //While buffer is not full, transmit data from buffer
    //U1TXBF checks to see if U1TXREG buffer is full 
    while((U1STAbits.UTXBF == false) && (Tx_In_Pos != Tx_Out_Pos)){
        
        U1TXREG = Tx_Buffer[Tx_Out_Pos];                     //Transmit next byte from buffer
        Tx_Out_Pos = (Tx_Out_Pos + 1) % UART_BUFF_SIZE;      //circulate the buffer by incrementing the index so next byte can be popped from.             
    }
    
    IFS1CLR = _IFS1_U1TXIF_MASK;                            //Clear the TX interrupt flag after buffer is full
    
    if (Tx_In_Pos != Tx_Out_Pos)
        IEC1SET = _IEC1_U1TXIE_MASK;                        //Kill once buffer depleted
    else IEC1CLR = _IEC1_U1TXIE_MASK;
    
}

/***************************************************************************
 * Function :  UART1IRQ (both RX and TX)
 * Input    :  _UART_1_VECTOR
 * Output   :  none
 * Comment  :  Setup UART Interrupt Service Request Routine 
 ***************************************************************************/

__ISRFAST(_UART_1_VECTOR) void Uart1IRQ(void){
    
    uint8_t next;
    
    //Check if bytes available in buffer & push into buffer
    while ((U1STAbits.URXDA == true) && (UART_BUFF_SIZE != Rx_Num_Bytes)){
        
        ++ Rx_Num_Bytes;                             
        Rx_Buffer[Rx_In_Pos] = U1RXREG;                 //Assign byte from Interrupt register to the RX buffer
        Rx_In_Pos = (Rx_In_Pos + 1) % UART_BUFF_SIZE;    //Increment the index for next incoming buffer
        
    }
    
    IFS1CLR = _IFS1_U1RXIF_MASK;                        //Clear RX Interrupt if buffer is empty
    
    //Transmit Interrupt - Transmit data as long as buffer is not full
    
    if (IEC1bits.U1TXIE == true){
        repushTx();
    }
    
}

/***************************************************************************
 * Function :  U1ReadSingleByte
 * Input    :  none
 * Output   :  char
 * Comment  :  Read Individual Character from buffer
 ***************************************************************************/

uint8_t U1ReadSingleByte(void){
    
    uint8_t  rxByte;
    
    if (Rx_Num_Bytes != 0){             //If bytes available to read             
        --Rx_Num_Bytes;                 //Then pop a byte from the buffer
        rxByte = Rx_Buffer[Rx_Out_Pos];
        Rx_Out_Pos = (Rx_Out_Pos + 1) % UART_BUFF_SIZE;
    }
    
    return rxByte;
    
}

/***************************************************************************
 * Function :  U1GetReadCount
 * Input    :  none
 * Output   :  Uint 16 - Number of bytes available to read
 * Comment  :  Returns number of available bytes in RX Buffer to read
 ***************************************************************************/

uint16_t U1GetReadCount(void){
    
    return Rx_Num_Bytes;

}

/***************************************************************************
 * Function :  U1WriteSingleByte
 * Input    :  input byte - one single byte
 * Output   :  none
 * Comment  :  Send Single byte 
 ***************************************************************************/

bool U1WriteSingleByte(const uint8_t inputByte){
    
    //First check if there is room available in the TX buffer
    uint8_t next = (Tx_In_Pos + 1) % UART_BUFF_SIZE;

    if (next == Tx_Out_Pos)             //Not enough room in buffer ?
        return false;               
                                    
    Tx_Buffer[Tx_In_Pos] = inputByte;   //Else - there is room
    Tx_In_Pos = next;
    
    repushTx();
    return true; 
    
}

/***************************************************************************
 * Function :  U1WriteManyBytes
 * Input    :  const uint8_t *buffer - pointer to input string
 *             const uint16_t length - number of bytes to TX
 * Output   :  uint8_t number of bytes actually written (regardless of requested)
 * Comment  :  Send Single byte 
 ***************************************************************************/

uint8_t U1WriteManyBytes(const uint8_t *buffer, const uint8_t length){
    
    uint8_t indx, size;
    
    size = (Tx_In_Pos - Tx_Out_Pos + UART_BUFF_SIZE) % UART_BUFF_SIZE;
    size = MIN(UART_BUFF_SIZE - size, length);

    //Copy over TX bytes
    for (indx = 0; indx < size; ++indx){           
        
        Tx_Buffer[Tx_In_Pos] = buffer[indx];
        Tx_In_Pos = (Tx_In_Pos + 1) % UART_BUFF_SIZE;
        
    }

    repushTx();                                        
    return size; //And let requestor know how many we TXfer'd     
    
}