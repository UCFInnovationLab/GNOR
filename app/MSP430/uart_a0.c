/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
 
#include <uart_a0.h>
#include "msp430.h"

// Receive buffer for the UART.  Incoming bytes need a place to go immediately,
// otherwise there might be an overrun when the next comes in.  The USCI ISR
// puts them here.
uint8_t  a0UartRcvBuf[A0_RXBUF_SIZE];

// The index within a0UartRcvBuf, where the next byte will be written.
uint16_t a0UartRcvBufIndex = 0;

// Boolean flag indicating whether a0UartRcvBufIndex has reached the
// threshold A0_RX_WAKE_THRESH.  0 = FALSE, 1 = TRUE
uint8_t  a0UartRxThreshReached = 0;


// Initializes the USCI_A0 module as a UART, using baudrate settings in
// a0Uart.h.  The baudrate is dependent on SMCLK speed.
void a0UartInit(void)
{
    // Always use the step-by-step init procedure listed in the USCI chapter of
    // the F5xx Family User's Guide
    UCA0CTL1 |= UCSWRST;        // Put the USCI state machine in reset
    UCA0CTL1 |= UCSSEL__SMCLK;  // Use SMCLK as the bit clock

    // Set the baudrate
    UCA0BR0 = UCA0_BR0;
    UCA0BR1 = UCA0_BR1;
    UCA0MCTL = (UCA0_BRF << 4) | (UCA0_BRS << 1) | (UCA0_OS);

    P3SEL |= BIT3+BIT4;         // Configure these pins as TXD/RXD

    UCA0CTL1 &= ~UCSWRST;       // Take the USCI out of reset
    UCA0IE |= UCRXIE;           // Enable the RX interrupt.  Now, when bytes are
                                // rcv'ed, the USCI_A0 vector will be generated.
}


// Sends 'len' bytes, starting at 'buf'
void a0UartSend(uint8_t * buf, uint8_t len)
{
    uint8_t i = 0;

    // Write each byte in buf to USCI TX buffer, which sends it out
    while (i < len)
    {
        UCA0TXBUF = *(buf+(i++));

        // Wait until each bit has been clocked out...
        while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));
    }
}


// Copies into 'buf' whatever bytes have been received on the UART since the
// last fetch.  Returns the number of bytes copied.
uint16_t a0UartReceiveBytesInBuffer(uint8_t* buf)
{
    uint16_t i, count;

    // Hold off ints for incoming data during the copy
    UCA0IE &= ~UCRXIE;

    for(i=0; i<a0UartRcvBufIndex; i++)
    {
        buf[i] = a0UartRcvBuf[i];
    }

    count = a0UartRcvBufIndex;
    a0UartRcvBufIndex = 0;     // Move index back to the beginning of the buffer
    a0UartRxThreshReached = 0;

    // Restore USCI interrupts, to resume receiving data.
    UCA0IE |= UCRXIE;

    return count;
}



// The USCI_A0 receive interrupt service routine (ISR).  Executes every time a
// byte is received on the back-channel UART.
#pragma vector=USCI_A0_VECTOR
__interrupt void a0UartISR(void)
{
    a0UartRcvBuf[a0UartRcvBufIndex++] = UCA0RXBUF;  // Fetch the byte, store
                                                    // it in the buffer.

    // Wake main, to fetch data from the buffer.
    if(a0UartRcvBufIndex >= A0_RX_WAKE_THRESH)
    {
        a0UartRxThreshReached = 1;
        __bic_SR_register_on_exit(LPM3_bits);       // Exit LPM0-3
    }
}
