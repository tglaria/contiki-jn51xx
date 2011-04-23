#include <jendefs.h>
#include <AppHardwareApi.h>
#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

#define UART_MCR_OFFSET   0x10
#define UART_LCR_OFFSET   0x0C
#define UART_DLM_OFFSET   0x04
#define UART_EER_OFFSET   0x20

#ifdef __BA1__ /* jn5139 core */
# define UART0_START_ADDR 0x30000000UL
# define UART1_START_ADDR 0x40000000UL
#else          /* jn5148 core */
# define UART0_START_ADDR 0x20030000UL
# define UART1_START_ADDR 0x20040000UL
#endif

#define UART0_CTS E_AHI_DIO17_INT
#define UART0_RTS E_AHI_DIO18_INT
#define UART1_CTS E_AHI_DIO4_INT
#define UART1_RTS E_AHI_DIO5_INT

static bool rtscts_uart0 = false,
            rtscts_uart1 = false;

//const struct irq_handle uart0_cts = {NULL, irq_cts, IRQ_DIO17, 0};
//const struct irq_handle uart1_cts = {NULL, irq_cts, IRQ_DIO4,  0};

//static void
//irq_uart(uint32_t a, uint32_t b)
//{
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

bool
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

  /* disable rts/cts line usage by uart, we control them by ourself
   * as there is no automatic flow ctrl and manual pin ctrl through
   * uart module does not work properly (w.r.t. the cts line) */
  vAHI_UartSetRTSCTS(uart, false);

  /* prime uart irq */
  vAHI_UartSetInterrupt(uart, false,  /* modem status         */
                              false,  /* rx line error status */
                              true,   /* tx fifo empty        */
                              true,   /* rx data there        */
                              E_AHI_UART_FIFO_LEVEL_8);

  /* come out of reset */
  vAHI_UartReset(uart, false, false);

  /* manual flowcontrol control and prime interrupt callbacks */
  if (uart == E_AHI_UART_0)
  {
    rtscts_uart0 = flowcontrol == E_AHI_UART_RTSCTS_FLOWCTRL;
    if (rtscts_uart0)
      vAHI_DioSetDirection(UART0_CTS, UART0_RTS);

//      /* enable rts line */
//      vAHI_DioSetOutput(0, UART0_RTS);
//
//      /* falling edge irq on cts line == reenable transmission */
//      irq_add(uart0_cts);
//      vAHI_DioInterruptEnable(UART0_CTS, 0);
//      vAHI_DioInterruptEdge(0, UART0_CTS);
//
//    vAHI_Uart0RegisterCallback(irq);
  }
  else if (uart == E_AHI_UART_1)
  {
    rtscts_uart1 = flowcontrol == E_AHI_UART_RTSCTS_FLOWCTRL;
    if (rtscts_uart1)
      vAHI_DioSetDirection(UART1_CTS, UART1_RTS);
//
//      /* falling edge irq on cts line == reenable transmission */
//      irq_add(uart1_cts);
//      vAHI_DioInterruptEnable(UART1_CTS, 0);
//      vAHI_DioInterruptEdge(0, UART1_CTS);
//
//    vAHI_Uart1RegisterCallback(irq_uart);
  }
}

size_t
uart_read(uint32_t dev, char *buf, size_t n)
{
  uint32_t rts = dev==E_AHI_UART_0 ? UART0_RTS : UART1_RTS;
  size_t i=0;

  vAHI_DioSetOutput(rts, 0); /* add a small delay here? */

  for(i=0; i<n && (u8AHI_UartReadLineStatus(dev)&E_AHI_UART_LS_DR); i++, buf++)
    *buf = u8AHI_UartReadData(dev);

  vAHI_DioSetOutput(0, rts);

  return i;
}

size_t
uart_write(uint32_t dev, char *buf, size_t n)
{
  uint32_t cts = dev==E_AHI_UART_0 ? UART0_CTS : UART1_CTS;
  size_t i;

  for(i=0; i<n && (u8AHI_UartReadLineStatus(dev)&E_AHI_UART_LS_THRE); i++, buf++)
  {
    while (!(u32AHI_DioReadInput() & cts))
      ;
    vAHI_UartWriteData(dev, *buf);
  }

  return i;
}
