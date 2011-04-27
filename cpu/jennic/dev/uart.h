#ifndef _UART_H_
# define _UART_H_
# include <stdint.h>
# include <stddef.h>
# include <stdbool.h>

# define E_AHI_UART_NO_PARITY       2
# define E_AHI_UART_NO_FLOWCTRL     0
# define E_AHI_UART_RTSCTS_FLOWCTRL 1

void
uart_init(uint32_t uart, uint32_t baudrate, uint8_t databits,
          uint8_t  parity, uint8_t stopbits, uint8_t flowcontrol);

size_t
uart_read(uint32_t handle, char *buf, size_t n);

size_t
uart_write(uint32_t handle, char *buf, size_t n);


/* low-level access for debugging support, don't use!*/
void
vAHI_UartSetBaudrate(uint32_t handle, uint32_t u32BaudRate);

#endif /* _file_ */
