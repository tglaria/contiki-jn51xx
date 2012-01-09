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

static bool
wreg(uint8_t r, uint8_t v, uint8_t mask)
{
  uint8_t buf[2] = {r};

  /* preserve last register value */
  if (!i2c(L3G4200D_ADDR, &r, 1, buf+1, 1,
        I2C_END_OF_TRANS|I2C_REPEATED_START))
    return false;

  buf[1] &= mask;
  buf[1] |= v;

  return i2c(L3G4200D_ADDR, buf, sizeof(buf), NULL, 0, I2C_END_OF_TRANS);
}

static uint8_t
rreg(uint8_t r)
{
  uint8_t val;
  i2c(L3G4200D_ADDR, &r, sizeof(uint8_t), &val, sizeof(uint8_t),
      I2C_REPEATED_START|I2C_END_OF_TRANS);
  return val;
}

static uint16_t
rreg16(uint8_t r)
{
  uint8_t buf[] = {0,0};
  /* here is a speciality: the MSB of the register address has to be
   * asserted, so the L3G4200D does subaddress updating, i.e. that it's
   * possible to read multiple bytes. */
  r |= (1<<7);
  i2c(L3G4200D_ADDR, &r, sizeof(uint8_t), buf, sizeof(buf),
      I2C_REPEATED_START|I2C_END_OF_TRANS);
  return (buf[1]<<8)|buf[0];
}

static int
value(int type)
{
  switch(type) {
  case GYRO_VALUE_X_DGS:
    return (int16_t) rreg16(L3G4200D_OUT_X);

  case GYRO_VALUE_Y_DGS:
    return (int16_t) rreg16(L3G4200D_OUT_Y);

  case GYRO_VALUE_Z_DGS:
    return (int16_t) rreg16(L3G4200D_OUT_Z);

  case GYRO_VALUE_TEMP_CELSIUS:
    return rreg(L3G4200D_OUT_TEMP);
  }

  return 0;
}

static int
configure(int type, int v)
{
  bool r = true;

  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (v)
    {
      wreg(L3G4200D_CTRL_REG1, 0xCF, 0xFF);     /* power up, 800Hz, cut-off 30, all axes */
      wreg(L3G4200D_CTRL_REG2, 0x00, 0xff);
      wreg(L3G4200D_CTRL_REG3, 0x08, 0xff);
      wreg(L3G4200D_CTRL_REG4, 0x30, 0xFF);     /* full scale, 2000 dps */
      wreg(L3G4200D_CTRL_REG5, 0x7F, 0xFF);     /* high and low-pass filtering */
      return wreg(L3G4200D_FIFO_CTRL_REG, 0x40, 0xFF); /* fifo stream mode */
      //wreg(L3G4200D_CTRL_REG1, 0x0C, 0xff);
      //wreg(L3G4200D_CTRL_REG2, 0x00, 0xff);
      //wreg(L3G4200D_CTRL_REG3, 0x08, 0xff);
      //wreg(L3G4200D_CTRL_REG4, 0x30, 0xff);
      //return wreg(L3G4200D_CTRL_REG5, 0x00, 0xff);
    }
    else   /* put to sleep, don't power down */
      return wreg(L3G4200D_CTRL_REG1, 0xC8, 0xFF);
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
    /* check if data in fifo buffer */
    return rreg(L3G4200D_FIFO_SRC_REG) & (1<<5);
  }

  return 0;
}

SENSORS_SENSOR(l3g4200d_sensor, GYRO_SENSOR, value, configure, status);
