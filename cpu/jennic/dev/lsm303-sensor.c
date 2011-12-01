/*
 * Copyright (c) 2010
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
#include "dev/acc-sensor.h"
#include "dev/mag-sensor.h"
#include "dev/irq.h"
#include <contiki-net.h>
#include <AppHardwareApi.h>

/* ------- Register names ------- */
#define LSM303_ADDR_A            0x32
#define LSM303_ADDR_M            0x3c
#define LSM303_CTRL_REG1         0x20
#define LSM303_CTRL_REG2         0x21
#define LSM303_CTRL_REG3         0x22
#define LSM303_CTRL_REG4         0x23
#define LSM303_CTRL_REG5         0x24
#define LSM303_REF               0x25
#define LSM303_STATUS            0x27
#define LSM303_OUT_X_A           0x28
#define LSM303_OUT_Y_A           0x30
#define LSM303_OUT_Z_A           0x32
#define LSM303_CRA_REG_M         0x00
#define LSM303_CRB_REG_M         0x01
#define LSM303_MR_REG_M          0x02
#define LSM303_OUT_X_M           0x03
#define LSM303_OUT_Z_M           0x05
#define LSM303_OUT_Y_M           0x07
#define LSM303_SR_REG_Mg         0x09
#define LSM303_IRA_REG_M         0x0A
#define LSM303_IRB_REG_M         0x0B
#define LSM303_IRC_REG_M         0x0C
#define LSM303_WHO_AM_I          0x0F

#define X_READ (1<<0)
#define Y_READ (1<<1)
#define Z_READ (1<<2)

static bool
awreg(uint8_t r, uint8_t v)
{
  uint8_t buf[] = {r,v};
  return i2c(LSM303_ADDR_A,buf,sizeof(buf),NULL,0,
             I2C_REPEATED_START|I2C_END_OF_TRANS);
}

static int
avalue(int type)
{
  /* weird implementation on the accelerometer, all values have to be read at
   * once to start a new sample!
   * So resample whenever the same value is read twice!
   * And set MSB on the register address to enable multiple bytes read. */
  static uint8_t bitmap = X_READ|Y_READ|Z_READ;
  static int16_t x,y,z;

  if (bitmap==(X_READ|Y_READ|Z_READ) ||
      (type==MAG_VALUE_X && (bitmap&X_READ)) ||
      (type==MAG_VALUE_Y && (bitmap&Y_READ)) ||
      (type==MAG_VALUE_Z && (bitmap&Z_READ)))
  {
    uint8_t buf[6], r=LSM303_OUT_X_A | (1<<7);
    i2c(LSM303_ADDR_A,&r,1, buf,sizeof(buf),
        I2C_REPEATED_START|I2C_END_OF_TRANS);
    x = (int16_t) (buf[1]<<8|buf[0])>>4;
    y = (int16_t) (buf[3]<<8|buf[2])>>4;
    z = (int16_t) (buf[5]<<8|buf[4])>>4;
  }

  switch(type) {
  case ACC_VALUE_X:
    bitmap |= X_READ;
    return (int16_t) x;

  case ACC_VALUE_Y:
    bitmap |= Y_READ;
    return (int16_t) y;

  case ACC_VALUE_Z:
    bitmap = Z_READ;
    return (int16_t) z;

  default:
    return 0;
  }

  return 0;
}

static int
aconfigure(int type, int v)
{
  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    i2c_init();
    if (v)
    {
      //awreg(LSM303_CTRL_REG1,0x3f,0xff); /* full rate, 1000Hz, 792Hz cut-off */
      //awreg(LSM303_CTRL_REG2,0x10,0xff); /* high-pass enable at 8Hz */
      //awreg(LSM303_CTRL_REG3,0x00,0xff); /* irq turned off? */
      //return awreg(LSM303_CTRL_REG4,0xb0,0xff); /* block update disabled for slower reading? */
      return awreg(LSM303_CTRL_REG1,0x27);
    }
    else
      return awreg(LSM303_CTRL_REG1,0x00);
  }

  return 0;
}

static int
astatus(int type)
{
  // TODO

  switch(type) {
  case SENSORS_ACTIVE:
    return 1;
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

static bool
mwreg(uint8_t r, uint8_t v)
{
  uint8_t buf[2] = {r,v};
  return i2c(LSM303_ADDR_M,buf,sizeof(buf),NULL,0,I2C_END_OF_TRANS);
}

static int
mvalue(int type)
{
  /* weird implementation on the magnetometer, all values have to be read at
   * once to start a new sample!
   *
   * So resample whenever the same value is read twice! */
  static uint8_t bitmap = X_READ|Y_READ|Z_READ;
  static uint16_t x,y,z;

  if (bitmap==(X_READ|Y_READ|Z_READ) ||
      (type==MAG_VALUE_X && (bitmap&X_READ)) ||
      (type==MAG_VALUE_Y && (bitmap&Y_READ)) ||
      (type==MAG_VALUE_Z && (bitmap&Z_READ)))
  {
    uint16_t buf[3];
    uint8_t r = LSM303_OUT_X_M;
    i2c(LSM303_ADDR_M, &r, 1, (uint8_t*) buf, sizeof(buf),
        I2C_REPEATED_START|I2C_END_OF_TRANS);
    x = buf[0]; z = buf[1]; y = buf[2];
  }

  switch(type) {
  case MAG_VALUE_X:
    bitmap |= X_READ;
    return (int16_t) x;

  case MAG_VALUE_Y:
    bitmap |= Y_READ;
    return (int16_t) y;

  case MAG_VALUE_Z:
    bitmap = Z_READ;
    return (int16_t) z;

  default:
    return 0;
  }

  return 0;
}

static int
mconfigure(int type, int v)
{
  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    i2c_init();
    if (v)
    {
      //mwreg(LSM303_CRA_REG_M,0x1c,0xff); /* output rate: 220Hz */
      //mwreg(LSM303_CRB_REG_M,0xe0,0xff); /* gain +8.1Gauss */
      //return mwreg(LSM303_MR_REG_M,0x00,0xff);  /* continuous mode */
      return mwreg(LSM303_MR_REG_M, 0x00);
    }
    else
      return mwreg(LSM303_MR_REG_M,0x03);  /* sleep mode */
    break;
  }

  return 0;
}

static int
mstatus(int type)
{
  // TODO

  switch(type) {
  case SENSORS_ACTIVE:
    return 1;
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

SENSORS_SENSOR(acc_sensor, ACC_SENSOR, avalue, aconfigure, astatus);
SENSORS_SENSOR(mag_sensor, MAG_SENSOR, mvalue, mconfigure, mstatus);
