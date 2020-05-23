/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#include "fsl_uart_edma.h"
#include "fsl_dma_manager.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART1
#define DEMO_UART_CLKSRC UART1_CLK_SRC
#define UART_TX_DMA_CHANNEL 0U
#define UART_RX_DMA_CHANNEL 1U
#define UART_TX_DMA_REQUEST kDmaRequestMux0UART1Tx
#define UART_RX_DMA_REQUEST kDmaRequestMux0UART1Rx
#define ECHO_BUFFER_LENGTH 8

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/

uart_edma_handle_t g_uartEdmaHandle;
edma_handle_t g_uartTxEdmaHandle;
edma_handle_t g_uartRxEdmaHandle;
uint8_t g_tipString[] = "UART EDMA example\r\nSend back received data\r\nEcho every 8 characters\r\n";
uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_UART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_UART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}


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

/*!
 * @brief Main function
 */
int main(void)
{
    uart_config_t uartConfig;
    uart_transfer_t xfer;
    uart_transfer_t sendXfer;
    uart_transfer_t receiveXfer;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Initialize the UART. */
    /*
     * uartConfig.baudRate_Bps = 115200U;
     * uartConfig.parityMode = kUART_ParityDisabled;
     * uartConfig.stopBitCount = kUART_OneStopBit;
     * uartConfig.txFifoWatermark = 0;
     * uartConfig.rxFifoWatermark = 1;
     * uartConfig.enableTx = false;
     * uartConfig.enableRx = false;
     */
    UART_GetDefaultConfig(&uartConfig);
    uartConfig.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    uartConfig.enableTx = true;
    uartConfig.enableRx = true;

    UART_Init(DEMO_UART, &uartConfig, CLOCK_GetFreq(DEMO_UART_CLKSRC));



#if 1
    hw_uart_s2_t S2;
	S2.U = UART_ReadS2(DEMO_UART);
	PRINTF("S2: 0x%x\n", S2.U);
	PRINTF("RXINV: %d\n", S2.B.RXINV);
	PRINTF("MSBF: %d\n", S2.B.MSBF);
	S2.B.RXINV = 1;
	S2.B.MSBF = 0;

	UART_WriteS2(DEMO_UART,S2.U);
	S2.U = UART_ReadS2(DEMO_UART);
   	PRINTF("S2: 0x%x\n", S2.U);
   	PRINTF("RXINV: %d\n", S2.B.RXINV);
   	PRINTF("MSBF: %d\n", S2.B.MSBF);

   	uint8_t C1 = UART_ReadC1(DEMO_UART);
   	PRINTF("C1: 0x%x\n", C1);
   	uint8_t C3 = UART_ReadC3(DEMO_UART);
   	PRINTF("C3: 0x%x\n", C3);
   	C3 = C3 | 0x10;
   	UART_WriteC3(DEMO_UART, C3);
#endif



    /* Configure DMA. */
    DMAMGR_Init();

    /* Request dma channels from DMA manager. */
    DMAMGR_RequestChannel(UART_TX_DMA_REQUEST, UART_TX_DMA_CHANNEL, &g_uartTxEdmaHandle);
    DMAMGR_RequestChannel(UART_RX_DMA_REQUEST, UART_RX_DMA_CHANNEL, &g_uartRxEdmaHandle);

    /* Create UART DMA handle. */
    UART_TransferCreateHandleEDMA(DEMO_UART, &g_uartEdmaHandle, UART_UserCallback, NULL, &g_uartTxEdmaHandle,
                                  &g_uartRxEdmaHandle);

    PRINTF("Ready to send EDMA Data\n");

    MSG1 msg;
    msg.header.UartPrefix = 0x1415;
    msg.header.opcode = 0x1235;
    /* Send g_tipString out. */
    xfer.data = (uint8_t *)&msg;
    xfer.dataSize = sizeof(msg);
    txOnGoing = true;
    UART_SendEDMA(DEMO_UART, &g_uartEdmaHandle, &xfer);

    /* Wait send finished */
    while (txOnGoing)
    {
    }
    while (1)
    {

    }

    /* Start to echo. */
    sendXfer.data = g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

    while (1)
    {
        /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
        if ((!rxOnGoing) && rxBufferEmpty)
        {
            rxOnGoing = true;
            UART_ReceiveEDMA(DEMO_UART, &g_uartEdmaHandle, &receiveXfer);
        }

        /* If TX is idle and g_txBuffer is full, start to send data. */
        if ((!txOnGoing) && txBufferFull)
        {
            txOnGoing = true;
            UART_SendEDMA(DEMO_UART, &g_uartEdmaHandle, &sendXfer);
        }

        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
        if ((!rxBufferEmpty) && (!txBufferFull))
        {
            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
            rxBufferEmpty = true;
            txBufferFull = true;
        }
    }
}
