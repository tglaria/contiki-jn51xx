/*
 * Copyright (c) 2009
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

#include "i2c.h"
#include <AppHardwareApi.h>
#include <stdbool.h>

#define I2C_10KHZ_SLOW_MODE (319)
#define I2C_400KHZ_FAST_MODE (31)

void
i2c_init()
{
  vAHI_SiConfigure(true, false, I2C_400KHZ_FAST_MODE);
}

static bool
i2c_wait()
{
  while (bAHI_SiPollTransferInProgress()) /* busy wait */
    ;

  if (bAHI_SiPollArbitrationLost() | bAHI_SiPollRxNack()) {
    // release bus
    vAHI_SiSetCmdReg(E_AHI_SI_NO_START_BIT, E_AHI_SI_STOP_BIT,
                     E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                     E_AHI_SI_SEND_ACK, E_AHI_SI_NO_IRQ_ACK);
    return true;
  }

  return false;
}
bool
i2c(u8_t addr, u8_t *wr, size_t n, u8_t *rd, size_t m, u8_t mode)
  /* put addr on bus, send *wr buffer, if repeated_start is set do not send
   * a stop condition prior to reading. If end_of_transmission is set send 
   * a nak after reading the last byte. */
{
  size_t i;
  bool repeated_start      = mode & I2C_REPEATED_START,
       end_of_transmission = mode & I2C_END_OF_TRANS;

  if (n) {
    /* send slave address, start condition */
    vAHI_SiWriteData8(addr);
    vAHI_SiSetCmdReg(E_AHI_SI_START_BIT,     E_AHI_SI_NO_STOP_BIT,
                     E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                     E_AHI_SI_SEND_ACK,      E_AHI_SI_NO_IRQ_ACK);
    if (i2c_wait()) return false;

    /* send wr data. If repeated_start and there is something to read, send no
     * stop condition, else send a stop condition on the last byte. */
    for (i=0; i<n; i++, wr++) {
      vAHI_SiWriteData8(*wr);
      vAHI_SiSetCmdReg(E_AHI_SI_NO_START_BIT,
                       (repeated_start && m!=0) ? E_AHI_SI_NO_STOP_BIT : i == n-1,
                       E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                       E_AHI_SI_SEND_ACK,      E_AHI_SI_NO_IRQ_ACK);
      if (i2c_wait()) return false;
    }
  }

  if (m) {
    /* send slave addr, start condition */
    vAHI_SiWriteData8(addr|1);
    vAHI_SiSetCmdReg(E_AHI_SI_START_BIT,     E_AHI_SI_NO_STOP_BIT,
                     E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                     E_AHI_SI_SEND_ACK,      E_AHI_SI_NO_IRQ_ACK);
    if (i2c_wait()) return false;

    /* read data, send stop condition on last byte, send nak on last byte when
     * end_of_transmission is set.*/
    for (i=0; i<m; i++, rd++) {
      vAHI_SiSetCmdReg(E_AHI_SI_NO_START_BIT, i==m-1,
                       E_AHI_SI_SLAVE_READ,   E_AHI_SI_NO_SLAVE_WRITE,
                       (end_of_transmission && i==m-1) ? E_AHI_SI_SEND_NACK : E_AHI_SI_SEND_ACK,
                       E_AHI_SI_NO_IRQ_ACK);
      while (bAHI_SiPollTransferInProgress())
        /* busy wait */;
      *rd = u8AHI_SiReadData8();
    }
  }

  return true;
}
