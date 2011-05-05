/*
 * Copyright (c) 2011
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 *
 * Only hardware flowcontrol. RTS line is asserted when hardware buffer (16
 * bytes) has received the 15th byte and another one is about to be
 * transmitted. So you need to make sure that the hardware reacts quick
 * enough.
 */

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <stdint.h>
#include <stdbool.h>
#include <contiki.h>
#include <string.h>
#include "uart.h"
#include "leds.h"

#define UART_MCR_OFFSET   0x10
#define UART_LCR_OFFSET   0x0C
#define UART_DLM_OFFSET   0x04
#define UART_EFR_OFFSET   0x20

#ifdef __BA1__ /* jn5139 core */
# define UART0_START_ADDR 0x30000000UL
# define UART1_START_ADDR 0x40000000UL
#else          /* jn5148 core */
# define UART0_START_ADDR 0x20030000UL
# define UART1_START_ADDR 0x20040000UL
# define UART_AFC_OFFSET  0x2C
#endif

#define UART0_CTS E_AHI_DIO4_INT
#define UART0_RTS E_AHI_DIO5_INT
#define UART1_CTS E_AHI_DIO17_INT
#define UART1_RTS E_AHI_DIO18_INT

typedef struct { volatile uint8_t head, tail, elems[UART_SIZE]; } uart_queue_t;
static uart_queue_t uart0_rxq, uart1_rxq;
static bool rtscts_uart0=false, rtscts_uart1=false;

static void irq(uint32_t irqsrc, uint32_t map);
static void enable_rxirq(uint32_t uart)
{
  if (uart == E_AHI_UART_0) vAHI_Uart0RegisterCallback(irq);
  else                      vAHI_Uart1RegisterCallback(irq);

  vAHI_UartSetInterrupt(uart, false,  /* modem status         */
                              false,  /* rx line error status */
                              false,  /* tx fifo empty        */
                              true,   /* rx data there        */
                              E_AHI_UART_FIFO_LEVEL_8);
}

static void disable_rxirq(uint32_t uart)
{
  vAHI_UartSetInterrupt(uart, false,  /* modem status         */
                              false,  /* rx line error status */
                              false,  /* tx fifo empty        */
                              false,  /* rx data there        */
                              E_AHI_UART_FIFO_LEVEL_8);
}

static void
q_enqueue(uart_queue_t *q, uint8_t c)
{
  q->elems[q->tail] = c;
  q->tail = (q->tail + sizeof(uint8_t)) % sizeof(q->elems);
}

static uint8_t
q_dequeue(uart_queue_t *q)
{
  uint8_t c = q->elems[q->head];
  q->head = (q->head + sizeof(uint8_t)) % sizeof(q->elems);
  return c;
}

static bool
q_empty(uart_queue_t *q)
{
  return q->head == q->tail;
}

static bool
q_full(uart_queue_t *q)
{
  return (q->tail+sizeof(uint8_t)) % sizeof(q->elems) == q->head;
}

static void
set_rts(uint32_t uart, bool bEnable)
{
//  if (bEnable) leds_on(LEDS_ALL);
//  else leds_off(LEDS_ALL);

  if(uart==E_AHI_UART_0 && rtscts_uart0)
    vAHI_DioSetOutput( bEnable ? UART0_RTS : 0x00,
                       bEnable ? 0x00 : UART0_RTS);

  if(uart==E_AHI_UART_1 && rtscts_uart1)
    vAHI_DioSetOutput( bEnable ? UART1_RTS : 0x00,
                       bEnable ? 0x00 : UART1_RTS);
}

static void
irq(uint32_t irqsrc, uint32_t map)
{
  switch(irqsrc)
  {
  case E_AHI_DEVICE_UART0:
    while (u8AHI_UartReadLineStatus(E_AHI_UART_0)&E_AHI_UART_LS_DR)
    {
      if (q_full(&uart0_rxq))
      {
        set_rts(E_AHI_UART_0, true);
        disable_rxirq(E_AHI_UART_0);
        return;
      }

      q_enqueue(&uart0_rxq, u8AHI_UartReadData(E_AHI_UART_0));

    }
    break;

  case E_AHI_DEVICE_UART1:
    while (u8AHI_UartReadLineStatus(E_AHI_UART_1)&E_AHI_UART_LS_DR)
    {
      if (q_full(&uart1_rxq))
      {
        set_rts(E_AHI_UART_1, true);
        disable_rxirq(E_AHI_UART_1);
        return;
      }

      q_enqueue(&uart1_rxq, u8AHI_UartReadData(E_AHI_UART_1));
    }
    break;

  default:
    break;
  }
}

//static void
//vAHI_UartSetFlowControl(uint32_t handle, bool bEnable)
//  /* On the Jennic JN5139 autoflow is a bit too slow, rts is asserted before
//   * last byte is transmitted, this is too late for the ftdi serial cables for
//   * example. */
//{
//  uint8 *pu8Reg;
//  uint8   u8Val;
//  uint32_t UART_START_ADR;
//
//  if(handle==E_AHI_UART_0) UART_START_ADR=UART0_START_ADDR;
//  if(handle==E_AHI_UART_1) UART_START_ADR=UART1_START_ADDR;
//
//#ifdef __BA1__
//    /* Get offset to Modem Control Register */
//    pu8Reg    = (uint8 *)(UART_START_ADR + UART_EFR_OFFSET);
//    /* Get content of MCR */
//    u8Val = *pu8Reg;
//    /* Enabling ? */
//    if (bEnable)
//    {
//      /* Set automatic flow control */
//      u8Val |= 0x10;
//    }
//    else
//    {
//      /* Clear automatic flow control */
//      u8Val &= 0xEF;
//    }
//    /* Write new value back to register */
//    *pu8Reg   = u8Val;
//
//#else /* Use hardware flow control on JN5148 */
//    /* Get offset to Modem Control Register */
//    pu8Reg    = (uint8 *)(UART_START_ADR + UART_AFC_OFFSET);
//    /* Get content of MCR */
//    u8Val = *pu8Reg;
//    /* Enabling ? */
//    if (bEnable)
//    {
//      /* Set automatic flow control */
//      u8Val |= 0x13;
//    }
//    else
//    {
//      /* Clear automatic flow control */
//      u8Val = 0x0;
//    }
//    /* Write new value back to register */
//    *pu8Reg   = u8Val;
//#endif
//}

void
vAHI_UartSetBaudrate(uint32_t handle, uint32_t u32BaudRate)
{
    uint8 *pu8Reg;
    uint8  u8TempLcr;
    uint16 u16Divisor;
    uint32 u32Remainder;
    uint32 UART_START_ADR;

    if(handle==E_AHI_UART_0) UART_START_ADR=UART0_START_ADDR;
    if(handle==E_AHI_UART_1) UART_START_ADR=UART1_START_ADDR;

    /* Put UART into clock divisor setting mode */
    pu8Reg    = (uint8 *)(UART_START_ADR + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr | 0x80;

    /* Write to divisor registers:
       Divisor register = 16MHz / (16 x baud rate) */
    u16Divisor = (uint16)(16000000UL / (16UL * u32BaudRate));

    /* Correct for rounding errors */
    u32Remainder = (uint32)(16000000UL % (16UL * u32BaudRate));

    if (u32Remainder >= ((16UL * u32BaudRate) / 2))
    {
        u16Divisor += 1;
    }

    pu8Reg  = (uint8 *)UART_START_ADR;
    *pu8Reg = (uint8)(u16Divisor & 0xFF);
    pu8Reg  = (uint8 *)(UART_START_ADR + UART_DLM_OFFSET);
    *pu8Reg = (uint8)(u16Divisor >> 8);

    /* Put back into normal mode */
    pu8Reg    = (uint8 *)(UART_START_ADR + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr & 0x7F;
}

void
uart_init(uint32_t uart, uint32_t baudrate, uint8_t databits,
          uint8_t  parity, uint8_t stopbits, uint8_t flowcontrol)
  /* primes uart for transmission.
   *  - baudrate is rounded to nearest possible one
   *  - databits must be one of E_AHI_UART_WORD_LEN_5-E_AHI_UART_WORD_LEN_8
   *  - parity must be one of E_AHI_UART_EVEN_PARITY, E_AHI_UART_ODD_PARITY or
   *    E_AHI_UART_NO_PARITY
   *  - stopbits is one of E_AHI_UART_2_STOP_BITS or E_AHI_UART_1_STOP_BIT
   *  - flowcontrol is one of E_AHI_UART_NO_FLOWCTRL or
   *    E_AHI_UART_RTSCTS_FLOWCTRL
   *
   * parameters are *NOT* checked for sanity!
   */
{
  /* enable uart and put into reset */
  vAHI_UartEnable(uart);
  vAHI_UartReset(uart, true, true);

  /* set baudrate and configure mode */
  vAHI_UartSetBaudrate(uart, baudrate);

  /* parity mode and enabling parity are the first two parameters! */
  vAHI_UartSetControl(uart, parity==E_AHI_UART_EVEN_PARITY, parity!=E_AHI_UART_NO_PARITY,
                      databits, stopbits, true);

  /* set manual flow control */
  vAHI_UartSetRTSCTS(uart, false);
  if (flowcontrol==E_AHI_UART_RTSCTS_FLOWCTRL)
  {
    if (uart==E_AHI_UART_0)
    {
      rtscts_uart0 = true;
      vAHI_DioSetDirection(UART0_CTS, UART0_RTS);
    }

    if (uart==E_AHI_UART_1)
    {
      rtscts_uart1 = true;
      vAHI_DioSetDirection(UART1_CTS, UART1_RTS);
    }
  }

  /* come out of reset */
  vAHI_UartReset(uart, false, false);

  /* clear queue */
  memset(&uart0_rxq, 0x00, sizeof(uart0_rxq));
  memset(&uart1_rxq, 0x00, sizeof(uart1_rxq));

  /* prime rx irq, signal ready for transmission */
  enable_rxirq(uart);
  set_rts(uart, false);
}

size_t
uart_read(uint32_t dev, char *buf, size_t n)
{
  uart_queue_t *q = dev==E_AHI_UART_0 ? &uart0_rxq : &uart1_rxq;
  size_t i;

  if (n==0) return 0;

  disable_rxirq(dev);

  for (i=0; i<n && !q_empty(q); i++, buf++)
    *buf = q_dequeue(q);

  printf("head: %d, tail: %d, i: %d, n: %d\n", q->head, q->tail, i, n);

  /* empty the queue, then empty the hardware buffer and every character
   * that is rx'ed in between. */
  for (; i<n && u8AHI_UartReadLineStatus(dev)&E_AHI_UART_LS_DR; i++, buf++)
    *buf = u8AHI_UartReadData(dev);

  printf("head: %d, tail: %d, i: %d\n", q->head, q->tail, i);

  set_rts(dev, false);
  enable_rxirq(dev);

  return i;
}

size_t
uart_write(uint32_t dev, char *buf, size_t n)
{
  uint32_t cts = dev==E_AHI_UART_0 ? UART0_CTS : UART1_CTS;
  bool cts_en  = dev==E_AHI_UART_0 ? rtscts_uart0 : rtscts_uart1;
  size_t i;

  for(i=0; i<n ; i++, buf++)
  {
    while(cts_en && (u32AHI_DioReadInput() & cts))
      ; /* wait for cts pin to clear */

    while(!(u8AHI_UartReadLineStatus(dev)&E_AHI_UART_LS_THRE))
      ; /* wait for transmit buffer to empty */

    vAHI_UartWriteData(dev, *buf);
  }

  return i;
}
