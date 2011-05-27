/*
 * Copyright (c) 2010, 2011
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
 */

#include "AppHardwareApi.h"
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "dev/i2c.h"
#include "dev/irq.h"
#include "jts.h"
#include "gdb2.h"

#if defined(__BA1__)
# define BUS_ERROR           *((volatile uint32 *)(0x4000008))
# define UNALIGNED_ACCESS    *((volatile uint32 *)(0x4000018))
# define ILLEGAL_INSTRUCTION *((volatile uint32 *)(0x400001C))
# define UNALIGNED_ACCESS_HANDLER 0x00012D70

#elif defined(__BA2__)
# define BUS_ERROR           *((volatile uint32 *)(0x4000000))
# define UNALIGNED_ACCESS    *((volatile uint32 *)(0x4000008))
# define ILLEGAL_INSTRUCTION *((volatile uint32 *)(0x400000A))

# define TICK_TIMER          *((volatile uint32 *)(0x4000004))
# define SYS_CALL            *((volatile uint32 *)(0x4000014))
# define SYS_TRAP            *((volatile uint32 *)(0x4000018))
# define SYS_GENERIC         *((volatile uint32 *)(0x400001C))
# define STACK_OVERFLOW      *((volatile uint32 *)(0x4000020))

#else
# error "unkown arch"
#endif

#ifdef __BA2__
void
unaligned_access()
{
        /* Enable and reset required UART */
        vAHI_UartEnable(E_AHI_UART_0);
        vAHI_UartReset(E_AHI_UART_0, TRUE, TRUE);
        vAHI_UartReset(E_AHI_UART_0, FALSE, FALSE);

        vAHI_UartSetClockDivisor(E_AHI_UART_0, E_AHI_UART_RATE_115200);

        vAHI_UartSetControl(E_AHI_UART_0,
                            E_AHI_UART_EVEN_PARITY,
                            E_AHI_UART_PARITY_DISABLE,
                            E_AHI_UART_WORD_LEN_8,
                            E_AHI_UART_1_STOP_BIT,
                            E_AHI_UART_RTS_LOW);

        vAHI_UartSetInterrupt(E_AHI_UART_0,
                              FALSE,
                              FALSE,
                              FALSE,
                              FALSE,
                              E_AHI_UART_FIFO_LEVEL_1);

        while ((u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_THRE ) == 0);
        vAHI_UartWriteData(E_AHI_UART_0, 'c');

        while(1);
    vAHI_SwReset();
}

# define UNALIGNED_ACCESS_HANDLER unaligned_access
#endif

void
init_hardware()
{
  u32AHI_Init();

#ifdef GDB
  GDB2_STARTUP(E_AHI_UART_0, E_AHI_UART_RATE_38400);
  vAHI_UartSetBaudrate(E_AHI_UART_0, 38400);
//# ifdef __BA1__
  HAL_BREAKPOINT();
//# endif
#endif

  leds_init();
  irq_init();
  i2c_init();
  clock_init();
  ctimer_init();

  UNALIGNED_ACCESS = UNALIGNED_ACCESS_HANDLER;
  //BUS_ERROR =           (uint32_t) reset;
  //ILLEGAL_INSTRUCTION = (uint32_t) reset;
}
