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
//    int r[2] = {0, 1};
//    uint16_t r[numberOfElements] = {358  ,
//    359 ,
//    360 ,
//    1   ,
//    2   ,
//    2   ,
//    3   ,
//    4   ,
//    5   ,
//    6   ,
//    6   ,
//    7   ,
//    8   ,
//    9   ,
//    9   ,
//    10  ,
//    11  ,
//    12  ,
//    13  ,
//    13  ,
//    14  ,
//    15  ,
//    16  ,
//    16  ,
//    17  ,
//    18  ,
//    19  ,
//    19  ,
//    20  ,
//    21  ,
//    22  ,
//    22  ,
//    23  ,
//    24  ,
//    25  ,
//    25  ,
//    26  ,
//    27  ,
//    27  ,
//    28  ,
//    29  ,
//    30  ,
//    30  ,
//    31  ,
//    32  ,
//    32  ,
//    33  ,
//    34  ,
//    34  ,
//    35  ,
//    36  ,
//    36  ,
//    37  ,
//    37  ,
//    38  ,
//    39  ,
//    39  ,
//    40  ,
//    41  ,
//    41  ,
//    42  ,
//    42  ,
//    43  ,
//    44  ,
//    44  ,
//    45  ,
//    45  ,
//    46  ,
//    46  ,
//    47  ,
//    48  ,
//    48  ,
//    49  ,
//    49  ,
//    50  ,
//    50  ,
//    51  ,
//    51  ,
//    52  ,
//    52  ,
//    53  ,
//    53  ,
//    54  ,
//    54  ,
//    55  ,
//    55  ,
//    56  ,
//    56  ,
//    57  ,
//    57  ,
//    58  ,
//    58  ,
//    59  ,
//    59  ,
//    60  ,
//    60  ,
//    61  ,
//    61  ,
//    62  ,
//    62  ,
//    63  ,
//    63  ,
//    64  ,
//    64  ,
//    64  ,
//    65  ,
//    65  ,
//    66  ,
//    66  ,
//    67  ,
//    67  ,
//    68  ,
//    68  ,
//    68  ,
//    69  ,
//    69  ,
//    70  ,
//    70  ,
//    71  ,
//    71  ,
//    71  ,
//    72  ,
//    72  ,
//    73  ,
//    73  ,
//    74  ,
//    74  ,
//    74  ,
//    75  ,
//    75  ,
//    76  ,
//    76  ,
//    77  ,
//    77  ,
//    77  ,
//    78  ,
//    78  ,
//    79  ,
//    79  ,
//    79  ,
//    80  ,
//    80  ,
//    81  ,
//    81  ,
//    82  ,
//    82  ,
//    82  ,
//    83  ,
//    83  ,
//    84  ,
//    84  ,
//    85  ,
//    85  ,
//    85  ,
//    86  ,
//    86  ,
//    87  ,
//    87  ,
//    88  ,
//    88  ,
//    89  ,
//    89  ,
//    90  ,
//    90  ,
//    90  ,
//    91  ,
//    91  ,
//    92  ,
//    92  ,
//    93  ,
//    93  ,
//    94  ,
//    94  ,
//    95  ,
//    95  ,
//    96  ,
//    96  ,
//    97  ,
//    97  ,
//    98  ,
//    99  ,
//    99  ,
//    100 ,
//    100 ,
//    101 ,
//    102 ,
//    102 ,
//    103 ,
//    103 ,
//    104 ,
//    105 ,
//    105 ,
//    106 ,
//    107 ,
//    107 ,
//    108 ,
//    109 ,
//    110 ,
//    110 ,
//    111 ,
//    112 ,
//    113 ,
//    114 ,
//    115 ,
//    116 ,
//    116 ,
//    117 ,
//    118 ,
//    119 ,
//    120 ,
//    122 ,
//    123 ,
//    124 ,
//    125 ,
//    126 ,
//    127 ,
//    129 ,
//    130 ,
//    131 ,
//    133 ,
//    134 ,
//    136 ,
//    137 ,
//    139 ,
//    141 ,
//    142 ,
//    144 ,
//    146 ,
//    148 ,
//    150 ,
//    152 ,
//    154 ,
//    156 ,
//    158 ,
//    161 ,
//    163 ,
//    165 ,
//    168 ,
//    170 ,
//    172 ,
//    175 ,
//    177 ,
//    180 ,
//    182 ,
//    185 ,
//    187 ,
//    190 ,
//    192 ,
//    194 ,
//    197 ,
//    199 ,
//    201 ,
//    203 ,
//    206 ,
//    208 ,
//    210 ,
//    212 ,
//    214 ,
//    215 ,
//    217 ,
//    219 ,
//    221 ,
//    222 ,
//    224 ,
//    225 ,
//    227 ,
//    228 ,
//    230 ,
//    231 ,
//    232 ,
//    234 ,
//    235 ,
//    236 ,
//    237 ,
//    238 ,
//    239 ,
//    240 ,
//    241 ,
//    242 ,
//    243 ,
//    244 ,
//    245 ,
//    246 ,
//    247 ,
//    248 ,
//    249 ,
//    249 ,
//    250 ,
//    251 ,
//    252 ,
//    252 ,
//    253 ,
//    254 ,
//    255 ,
//    255 ,
//    256 ,
//    257 ,
//    257 ,
//    258 ,
//    258 ,
//    259 ,
//    260 ,
//    260 ,
//    261 ,
//    261 ,
//    262 ,
//    262 ,
//    263 ,
//    264 ,
//    264 ,
//    265 ,
//    265 ,
//    266 ,
//    266 ,
//    267 ,
//    267 ,
//    268 ,
//    268 ,
//    269 ,
//    269 ,
//    270 ,
//    270 ,
//    270 ,
//    271 ,
//    271 ,
//    272 ,
//    272 ,
//    273 ,
//    273 ,
//    274 ,
//    274 ,
//    275 ,
//    275 ,
//    275 ,
//    276 ,
//    276 ,
//    277 ,
//    277 ,
//    278 ,
//    278 ,
//    278 ,
//    279 ,
//    279 ,
//    280 ,
//    280 ,
//    281 ,
//    281 ,
//    281 ,
//    282 ,
//    282 ,
//    283 ,
//    283 ,
//    283 ,
//    284 ,
//    284 ,
//    285 ,
//    285 ,
//    286 ,
//    286 ,
//    286 ,
//    287 ,
//    287 ,
//    288 ,
//    288 ,
//    289 ,
//    289 ,
//    289 ,
//    290 ,
//    290 ,
//    291 ,
//    291 ,
//    292 ,
//    292 ,
//    292 ,
//    293 ,
//    293 ,
//    294 ,
//    294 ,
//    295 ,
//    295 ,
//    295 ,
//    296 ,
//    296 ,
//    297 ,
//    297 ,
//    298 ,
//    298 ,
//    299 ,
//    299 ,
//    300 ,
//    300 ,
//    301 ,
//    301 ,
//    302 ,
//    302 ,
//    303 ,
//    303 ,
//    303 ,
//    304 ,
//    304 ,
//    305 ,
//    306 ,
//    306 ,
//    307 ,
//    307 ,
//    308 ,
//    308 ,
//    309 ,
//    309 ,
//    310 ,
//    310 ,
//    311 ,
//    311 ,
//    312 ,
//    312 ,
//    313 ,
//    314 ,
//    314 ,
//    315 ,
//    315 ,
//    316 ,
//    316 ,
//    317 ,
//    318 ,
//    318 ,
//    319 ,
//    319 ,
//    320 ,
//    321 ,
//    321 ,
//    322 ,
//    322 ,
//    323 ,
//    324 ,
//    324 ,
//    325 ,
//    326 ,
//    326 ,
//    327 ,
//    328 ,
//    328 ,
//    329 ,
//    330 ,
//    330 ,
//    331 ,
//    332 ,
//    332 ,
//    333 ,
//    334 ,
//    335 ,
//    335 ,
//    336 ,
//    337 ,
//    337 ,
//    338 ,
//    339 ,
//    340 ,
//    340 ,
//    341 ,
//    342 ,
//    343 ,
//    343 ,
//    344 ,
//    345 ,
//    346 ,
//    347 ,
//    347 ,
//    348 ,
//    349 ,
//    350 ,
//    350 ,
//    351 ,
//    352 ,
//    353 ,
//    354 ,
//    354 ,
//    355 ,
//    356 ,
//    357 ,
//    358};

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

    

