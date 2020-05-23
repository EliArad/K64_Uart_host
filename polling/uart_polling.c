/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_uart.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */

#define COMM_UART UART1
#define COMM_UART_CLKSRC UART1_CLK_SRC




/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */

#pragma pack(push, 1)
typedef struct Header
{

	uint16_t  UartPrefix;
	uint16_t  opcode;
	//uint8_t   msgSize;

} Header;

#pragma pack(pop)

#pragma pack(push, 1)
typedef struct MSG1
{

	Header header;
	//uint32_t  data;
	//uint8_t   value;
	//uint16_t  value1;
	//uint16_t  crc;
} MSG1;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct MSG2
{

	Header header;
	//uint16_t  data;
	//uint8_t   value;
	//uint32_t  value1;
	//uint16_t  crc;
} MSG2;
#pragma pack(pop)


typedef union _hw_uart_s2
{
    uint8_t U;
    struct _hw_uart_s2_bitfields
    {
        uint8_t RAF : 1;               /*!< [0] Receiver Active Flag */
        uint8_t LBKDE : 1;             /*!< [1] LIN Break Detection Enable */
        uint8_t BRK13 : 1;             /*!< [2] Break Transmit Character Length */
        uint8_t RWUID : 1;             /*!< [3] Receive Wakeup Idle Detect */
        uint8_t RXINV : 1;             /*!< [4] Receive Data Inversion */
        uint8_t MSBF : 1;              /*!< [5] Most Significant Bit First */
        uint8_t RXEDGIF : 1;           /*!< [6] RxD Pin Active Edge Interrupt Flag */
        uint8_t LBKDIF : 1;            /*!< [7] LIN Break Detect Interrupt Flag */
    } B;
} hw_uart_s2_t;

int main(void)
{

    uart_config_t config;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */

    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.txFifoWatermark = 0;
    config.rxFifoWatermark = 0;
    config.enableTx = true;
    config.enableRx = true;
    MSG1 msg;
    if (UART_Init(COMM_UART, &config, CLOCK_GetFreq(COMM_UART_CLKSRC)) != kStatus_Success)
    {
    	PRINTF("Uart1 not ready to receive\n");
    } else {
    	PRINTF("Uart1 ready to receive: %d bytes\n", sizeof(msg));
    }

    hw_uart_s2_t S2;
    S2.U = UART_ReadS2(COMM_UART);
    PRINTF("S2: 0x%x\n", S2.U);
    PRINTF("RXINV: %d\n", S2.B.RXINV);
    PRINTF("MSBF: %d\n", S2.B.MSBF);
    S2.B.RXINV = 1;
    S2.B.MSBF = 0;

    UART_WriteS2(COMM_UART,S2.U);
    S2.U = UART_ReadS2(COMM_UART);
	PRINTF("S2: 0x%x\n", S2.U);
	PRINTF("RXINV: %d\n", S2.B.RXINV);
	PRINTF("MSBF: %d\n", S2.B.MSBF);

	uint8_t C1 = UART_ReadC1(COMM_UART);
	PRINTF("C1: 0x%x\n", C1);
	uint8_t C3 = UART_ReadC3(COMM_UART);
	PRINTF("C3: 0x%x\n", C3);
	C3 = C3 | 0x10;
	UART_WriteC3(COMM_UART, C3);


    msg.header.UartPrefix = 0x1415;
    msg.header.opcode = 0x1235;

    UART_FlushFifo(COMM_UART);
    UART_WriteBlocking(COMM_UART, (uint8_t *)&msg, sizeof(msg));
    UART_WriteBlocking(COMM_UART, (uint8_t *)&msg, sizeof(msg));

    //UART_WriteBlocking(COMM_UART, (uint8_t *)&msg, sizeof(msg));


    while (1)
    {
        UART_ReadBlocking(COMM_UART, (uint8_t *)&msg, sizeof(msg));
#if 1
        PRINTF("UartPrefix 0x%x\n" , msg.header.UartPrefix);
        //PRINTF("msgSize 0x%x\n" , msg.header.msgSize);
        PRINTF("opcode 0x%x\n" , msg.header.opcode);

        //PRINTF("data 0x%x\n" , msg.data);
        //PRINTF("value1 0x%x\n" , msg.value);
        //PRINTF("value1 0x%x\n" , msg.value1);
        //PRINTF("crc 0x%x\n" , msg.crc);
        //UART_WriteBlocking(DEMO_UART, &ch, 1);
#endif
    }
}
