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
 * Driver for ADXL345 Acceleration sensor.
 */

#include "dev/i2c.h"
#include "dev/l3g4200d-sensor.h"
#include <AppHardwareApi.h>

/* ------- Register names ------- */
#define L3G4200D_ADDR          0xD0
#define L3G4200D_WHO_AM_I      0x0F
#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27
#define L3G4200D_OUT_X         0x28
#define L3G4200D_OUT_Y         0x2A
#define L3G4200D_OUT_Z         0x2C
#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F
#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_TSH_X    0x32
#define L3G4200D_INT1_TSH_Y    0x34
#define L3G4200D_INT1_TSH_Z    0x36
#define L3G4200D_INT1_TSH_DUR  0x38

static uint8_t
rreg(uint8_t r)
{
  uint8_t buf[2] = {r,0};
  i2cb(L3G4200D_ADDR, 1,1,buf);
  return buf[1];
}

static i2c_t gyro_txn;

static void
gyro_cb(bool status)
{
  i2c(&gyro_txn);
  sensors_changed(&l3g4200d_sensor);
}

static i2c_t gyro_txn = {
  .addr  = L3G4200D_ADDR,
  .cb    = gyro_cb,
  .wrlen = 1,
  .rdlen = 6,
  .buf   = {L3G4200D_OUT_X|(1<<7),0,0,0,0,0,0}
};

static int
value(int type)
{
  switch(type) {
  case GYRO_VALUE_X_DGS:
    return (int16_t) (gyro_txn.buf[2]<<8)|gyro_txn.buf[1];

  case GYRO_VALUE_Y_DGS:
    return (int16_t) (gyro_txn.buf[4]<<8)|gyro_txn.buf[3];

  case GYRO_VALUE_Z_DGS:
    return (int16_t) (gyro_txn.buf[6]<<8)|gyro_txn.buf[5];

  case GYRO_VALUE_TEMP_CELSIUS:
    return rreg(L3G4200D_OUT_TEMP);
  }

  return 0;
}

static int
configure(int type, int v)
{
  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (v)
    {
      if (!I2CW(L3G4200D_ADDR,L3G4200D_CTRL_REG1, 0x4F))
        return false;     /* power up, 200Hz, cut-off 12.5, all axes */
      I2CW(L3G4200D_ADDR,L3G4200D_CTRL_REG2, 0x01);     /* normal high-pass, 8Hz cutoff */
      I2CW(L3G4200D_ADDR,L3G4200D_CTRL_REG3, 0x00);     /* no irqs */
      I2CW(L3G4200D_ADDR,L3G4200D_CTRL_REG4, 0xB0);     /* block update, full scale, 2000 dps */
      I2CW(L3G4200D_ADDR,L3G4200D_CTRL_REG5, 0x40);     /* fifo enable */
      I2CW(L3G4200D_ADDR,L3G4200D_FIFO_CTRL_REG, 0x40); /* fifo stream mode */

      gyro_cb(true);

      return true;
    }
    else   /* power down, keep config */
      return I2CW(L3G4200D_ADDR,L3G4200D_CTRL_REG1,0x47);
  }

  return 0;
}

static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return rreg(L3G4200D_WHO_AM_I);
  case SENSORS_READY:
    /* check if data in fifo buffer, empty status of fifo */
    return !rreg(L3G4200D_FIFO_SRC_REG) & (1<<5);
  }

  return 0;
}

SENSORS_SENSOR(l3g4200d_sensor, GYRO_SENSOR, value, configure, status);
