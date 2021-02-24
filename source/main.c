/******************************************************************************
* File Name:   main.c
*
* Description: This example demonstrates the UART transmit and receive
*              operation using DMA.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cycfg.h"
#include "UartDma.h"

/*******************************************************************************
*            Constants
*******************************************************************************/
#define LED_ON           (0u)
#define LED_OFF          (!LED_ON)

/*******************************************************************************
*            Global variables
*******************************************************************************/
uint8_t rx_dma_error;   /* RxDma error flag */
uint8_t tx_dma_error;   /* TxDma error flag */
uint8_t uart_error;     /* UART error flag */
uint8_t rx_dma_done;    /* RxDma done flag */


/*******************************************************************************
*            Forward declaration
*******************************************************************************/
void handle_error(void);
void Isr_UART(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* The main function performs the following actions:
*  1. Configures RX and TX DMAs to handle UART RX+TX direction.
*  2. Configures UART component.
*  3. If initialization of UART component fails then turn on RED_LED_ERROR.
*  4. Sends text header to the UART serial terminal.
*  5. Waits in an infinite loop (for DMA or UART error interrupt)
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Set up the device based on configurator selections */
    init_cycfg_all();

    uint8_t rx_dma_uart_buffer_a[BUFFER_SIZE];
    uint8_t rx_dma_uart_buffer_b[BUFFER_SIZE];
    cy_en_scb_uart_status_t init_status;
    cy_stc_scb_uart_context_t KIT_UART_context;
    uint32_t active_descr = DMA_DESCR0; /* flag to control which descriptor to use */

    cy_stc_sysint_t KIT_UART_INT_cfg =
    {
        .intrSrc      = KIT_UART_IRQ,
        .intrPriority = 7u,
    };

    cy_stc_sysint_t RX_DMA_INT_cfg =
    {
        .intrSrc      = (IRQn_Type)RxDma_IRQ,
        .intrPriority = 7u,
    };

    cy_stc_sysint_t TX_DMA_INT_cfg =
    {
        .intrSrc      = (IRQn_Type)TxDma_IRQ,
        .intrPriority = 7u,
    };

    /* Configure DMA Rx and Tx channels for operation */
    configure_rx_dma(rx_dma_uart_buffer_a, rx_dma_uart_buffer_b, &RX_DMA_INT_cfg);
    configure_tx_dma(rx_dma_uart_buffer_a, &TX_DMA_INT_cfg);

    /* Initialize and enable interrupt from UART. The UART interrupt sources
    *  are enabled in the Component GUI */
    Cy_SysInt_Init(&KIT_UART_INT_cfg, &Isr_UART);
    NVIC_EnableIRQ(KIT_UART_INT_cfg.intrSrc);

    /* Start UART operation */
    init_status = Cy_SCB_UART_Init(KIT_UART_HW, &KIT_UART_config, &KIT_UART_context);
    if (init_status!=CY_SCB_UART_SUCCESS)
    {
        handle_error();
    }
    Cy_SCB_UART_Enable(KIT_UART_HW);

    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(KIT_UART_HW, "\x1b[2J\x1b[;H");

    Cy_SCB_UART_PutString(KIT_UART_HW, "************************************************************\r\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, "PSoC 6 MCU UART Transmit and Receive using DMA\r\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, "************************************************************\r\n\n");
    Cy_SCB_UART_PutString(KIT_UART_HW, ">> Start typing to see the echo on the screen \r\n\n");

    /* Initialize flags */
    rx_dma_error = 0;
    tx_dma_error = 0;
    uart_error = 0;
    rx_dma_done = 0;
    __enable_irq();
 
    while (1)
    {
        /* Indicate status if RxDma error or TxDma error or UART error occurs */
        if ((rx_dma_error==1) | (tx_dma_error==1) | (uart_error==1))
        {
            handle_error();
        }

        /* Handle RxDma complete */
        if (rx_dma_done==1)
        {
            /* Ping Pong between rx_dma_uart_buffer_a and rx_dma_uart_buffer_b */
            /* Ping Pong buffers give firmware time to pull the data out of one or the other buffer */
            if (DMA_DESCR0 == active_descr)
            {
                /* Set source RX Buffer A as source for TxDMA */
                Cy_DMA_Descriptor_SetSrcAddress(&TxDma_Descriptor_0, (uint32_t *) rx_dma_uart_buffer_a);
                active_descr = DMA_DESCR1;
            }
            else
            {
                /* Set source RX Buffer B as source for TxDMA */
                Cy_DMA_Descriptor_SetSrcAddress(&TxDma_Descriptor_0, (uint32_t *) rx_dma_uart_buffer_b);
                active_descr = DMA_DESCR0;
            }

            Cy_DMA_Channel_SetDescriptor(TxDma_HW, TxDma_CHANNEL, &TxDma_Descriptor_0);
            Cy_DMA_Channel_Enable(TxDma_HW, TxDma_CHANNEL);
            rx_dma_done = 0;
        }
    }
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: Isr_UART
********************************************************************************
*
* Summary:
* Handles UART Rx underflow and overflow conditions. This conditions must never
* occur.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Isr_UART(void)
{
    uint32 intrSrcRx;
    uint32 intrSrcTx;

    /* Get RX interrupt sources */
    intrSrcRx = Cy_SCB_UART_GetRxFifoStatus(KIT_UART_HW);
    Cy_SCB_UART_ClearRxFifoStatus(KIT_UART_HW, intrSrcRx);

    /* Get TX interrupt sources */
    intrSrcTx = Cy_SCB_UART_GetTxFifoStatus(KIT_UART_HW);
    Cy_SCB_UART_ClearTxFifoStatus(KIT_UART_HW, intrSrcTx);

    /* RX overflow or RX underflow or RX overflow occured */
    uart_error = 1;
}
