//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - UART Demo
// Application Overview - The objective of this application is to showcase the 
//                        use of UART. The use case includes getting input from 
//                        the user and display information on the terminal. This 
//                        example take a string as input and display the same 
//                        when enter is received.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_UART_Demo_Application
// or
// docs\examples\CC32xx_UART_Demo_Application.pdf
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup uart_demo
//! @{
//
//*****************************************************************************

// Driverlib includes
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "uart.h"
#include "interrupt.h"
#include "pinmux.h"
#include "utils.h"
#include "prcm.h"
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
// Common interface include
#include "uart_if.h"
#include "gpio_if.h"
//*****************************************************************************
//                          MACROS                                  
//*****************************************************************************
#define APPLICATION_VERSION  "1.1.1"
#define APP_NAME             "UART Echo"
#define CONSOLE              UARTA0_BASE
#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)       MAP_UARTCharPut(CONSOLE,c)
#define MAX_STRING_LENGTH    80
#define numberOfElements     500

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile int g_iCounter = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// Set to: 0 if no new data received from the UART
//         !0 if new data received from UART
static int flag_DataReceivedFromUART = 0;

// Set to: 0 if data is not ready to sent
//         !0 if data is ready to be sent
static int flag_CompressedReadyToSend      = 0;
static int flag_OriginalReadyToSend = 1;
uint16_t dataFromUART[numberOfElements];
//static const int numberOfElements = 480;

typedef struct Compressed {
    uint32_t codeword;
    uint8_t numValidBits;
}compressed;

typedef struct Si {
    int16_t si;
    int numBits;
}si;
compressed compressedData[numberOfElements];

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//                      LOCAL DEFINITION                                   
//*****************************************************************************

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t        CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


uint32_t reverse(uint32_t toReverse)
{
    uint32_t reversed;
    int i;
    for (i = 31; i > 0; i)
    {
        reversed |= (toReverse & 1) << i;
        toReverse >>= 1;
        --i;
    }
    return reversed;
}

//*****************************************************************************
//
//! Main function handling the uart echo. It takes the input string from the
//! terminal while displaying each character of string. whenever enter command 
//! is received it will echo the string(display). if the input the maximum input
//! can be of 80 characters, after that the characters will be treated as a part
//! of next string.
//!
//! \param  None
//!
//! \return None
//! 
//*****************************************************************************

//Return absolute value of integer.
int abs (int num)
{
    if (num < 0)
        return (-1 * num);
    return num;
}


//Concatenate [num1, num2] where each number is 16 bits.
int concatenate (int num1, int num2)
{
    return (num1 * (2 ^ 15) + num2);

}

//Table to convert from n[i] to s[i]
si ni2si(int ni)
{
    si siToRet;
    int16_t siTable [15] = {0b00, 0b010, 0b011, 0b100, 0b101, 0b110, 0b1110, 0b11110, 0b111110, 0b1111110,
        0b11111110, 0b111111110, 0b1111111110, 0b11111111110, 0b111111111110};
    int numBitsTable [15] = {2, 3, 3, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    siToRet.si = siTable[ni];
    siToRet.numBits = numBitsTable[ni];

    return siToRet;
}

//Find max int in r
int findMax(uint8_t r[], int numElements)
{
    int max = INT_MIN;
    int i;
    for (i = 0; i < numElements; i++)
    {
        if (r[i] > max)
            max = r[i];

    }
    return max;
}

//Function to return lower n bits of 16 bit number.
int16_t lowernBits(uint8_t origNum, int n)
{
    uint8_t mask = 0;
    int i;
    for (i = 0; i < n; i++)
    {
        mask |= 1 << i;

    }
    //printf("%"PRIu16 "\n", mask);
    return (origNum & mask);
}


void lec(uint16_t r[], int numElements)
{
    //Begin compress

    //Declare integer arrays that will be used.

//Change to numElements.
//    int * d = (int *) malloc (sizeof(int) * 480);
    int d[500];
    int n[500];
    int s[500];
    int a[500];
    int bs[500];



    int max = findMax(r, numElements);
    int firstR = log((max));

    //Calcualte d[i] for all values.
    int i;
    for (i = 0; i < numElements; i++)
    {
        if (i == 0)
        {
            d[i] = r[i] - firstR;

        }
        else
            d[i] = r[i] - r[i-1];


    } //Compress state is complete.

    //Begin encode
    for (i = 0; i < numElements; i++)
    {
        //Evaluate n[i] for all elements.
        if (d[i] == 0)
            n[i] = 0;
        else
            n[i] = ceil(log(abs(d[i])));

        //Evaluate s[i] for all elements.
        s[i] = ni2si(n[i]).si;

        //Evaluate bs[i] for all elements.
        if (n[i] == 0)
        {
            compressedData[i].codeword = s[i];
            //compressedData.numValidBits[i] = ni2si(n[i]).numBits;
        }
        else
        {
            if (d[i] > 0)
            {
                a[i] = lowernBits(d[i], n[i]);
            }
            else
            {
                a[i] = lowernBits(d[i-1], n[i]);
            }
            compressedData[i].codeword = concatenate(s[i], a[i]);

        }
        compressedData[i].numValidBits = ni2si(n[i]).numBits + n[i];
    }

    for (i = 0; i < numElements; i++)
    {
        Report("\n\r codeword %d  ", compressedData[i].codeword);
        Report("numValidBits %d \n\r", compressedData[i].numValidBits);
    }

}

//Read data into board using UART.
void load ()
{

    uint16_t r[numberOfElements];
    int i;
    for (i = 0; i < numberOfElements; i++)
    {
        r[i] = i;
    }

    for (i = 0; i < numberOfElements; i++)
    {
        UartPutChar(r[i]);
        dataFromUART[i] = r[i];
    }

    flag_DataReceivedFromUART = 1;

}

void send()
{

    uint8_t txData = 0x00;
    uint8_t txDataBits = 0;
    uint32_t index;
    for (index = 0; index < numberOfElements; index++)
    {
        uint32_t codeword = compressedData[index].codeword;
        uint8_t validBits = compressedData[index].numValidBits;

        codeword = reverse(codeword);
        while (validBits > 0)
        {
            txData = (txData << 1) | (codeword * 0x01);
            codeword = codeword >> 1;
            txDataBits++;

            if (txDataBits == 8)
            {
                //SEND UART BYTE
                UartPutChar(codeword);
                //MAP_UtilsDelay(80000000);
//                MAP_UART_transmitData(EUSCI_A0_BASE, codeword);
            }
            validBits--;
        }
        //Send UART BYTE = 0;

    }
//int i;
//    for (i = 0; i < numberOfElements; i++)
//    {
////        MAP_UtilsDelay(800000);
//        UartPutChar(dataFromUART[i]);
//    }
}

void main()
{
    char cString[MAX_STRING_LENGTH+1];
    char cCharacter;
    int iStringLength = 0;
    //
    // Initailizing the board
    //
    BoardInit();
    //
    // Muxing for Enabling UART_TX and UART_RX.
    //
    PinMuxConfig();
    //
    // Initialising the Terminal.
    //
    InitTerm();
    //
    // Clearing the Terminal.
    //
    ClearTerm();
    DisplayBanner(APP_NAME);
    Message("\t\t****************************************************\n\r");
    Message("\t\t\t        CC3200 UART Echo Usage        \n\r");
    Message("\t\t Type in a string of alphanumeric characters and  \n\r");
    Message("\t\t pressenter, the string will be echoed. \n\r") ;
    Message("\t\t Note: if string length reaches 80 character it will \n\r");
    Message("\t\t echo the string without waiting for enter command \n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");
    Message("cmd#");


    GPIO_IF_LedConfigure(LED1|LED2|LED3);

        GPIO_IF_LedOff(MCU_ALL_LED_IND);




while (1)
{

    if (flag_OriginalReadyToSend)
    {
        load();
        flag_OriginalReadyToSend = 0;
    }

    if (flag_DataReceivedFromUART)
    {
        int i = 0;
        lec(dataFromUART, numberOfElements);
        flag_DataReceivedFromUART = 0;
        flag_CompressedReadyToSend = 1;

    }
    if (flag_CompressedReadyToSend)
    {
        send();
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        flag_CompressedReadyToSend = 0;

    }
}

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

    

