#include <jendefs.h>
#include <AppHardwareApi.h>
#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

#define UART_LCR_OFFSET   0x0C
#define UART_DLM_OFFSET   0x04
#define UART_EER_OFFSET   0x20

void
vAHI_UartSetBaudrate(uint32_t handle, uint32_t u32BaudRate)
{
    uint8 *pu8Reg;
    uint8  u8TempLcr;
    uint16 u16Divisor;
    uint32 u32Remainder;
    uint32 UART_START_ADR;

    if(handle==E_AHI_UART_0) UART_START_ADR=0x30000000UL;
    if(handle==E_AHI_UART_1) UART_START_ADR=0x40000000UL;

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

/*
int32_t
uart_open(int32_t uart, uint32_t baud)
{
  uint32_t UART_START_ADR;
  uint8_t  *reg;

  if(uart!=E_AHI_UART_0 && uart!=E_AHI_UART_1) return -1;

  vAHI_UartEnable(uart);
  vAHI_UartReset(uart, true, true);
  vAHI_UartSetBaudrate(uart, baud);

  // enable hardware flowctrl
  vAHI_UartSetRTSCTS(uart, true);
  if(uart==E_AHI_UART_0) UART_START_ADR=0x30000000UL;
  if(uart==E_AHI_UART_1) UART_START_ADR=0x40000000UL;
  reg = (uint8_t*) UART_START_ADR + 0x20;
  *reg = *reg | (0x10);

  vAHI_UartReset(uart, false, false);

  return uart;
}

int32_t
uart_read(int32_t dev, char *buf, size_t n)
{
  size_t i=0;
  for(i=0; i<n && (u8AHI_UartReadLineStatus(dev)&E_AHI_UART_LS_DR); i++, buf++)
    *buf = u8AHI_UartReadData(dev);
  return i;
}

int32_t
uart_write(int32_t dev, char *buf, size_t n)
{
  size_t i;
  for(i=0; i<n && (u8AHI_UartReadLineStatus(dev)&E_AHI_UART_LS_THRE); i++, buf++)
    vAHI_UartWriteData(dev, *buf);
  return i;
}
*/
