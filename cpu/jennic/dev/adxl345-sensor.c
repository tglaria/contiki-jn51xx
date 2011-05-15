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
#include "dev/hrclock.h"
#include <AppHardwareApi.h>
#include <gdb.h>

/* ------- Register names ------- */
#define ADXL345_ADDR 0xA6
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

static int
value(int type)
{
  uint8_t rb[] = {0,0};

  switch(type) {
  case ACC_VALUE_X:
    rb[0] = ADXL345_DATAX0;
    i2c(ADXL345_ADDR, rb, 1, rb, sizeof(rb), I2C_REPEATED_START|I2C_END_OF_TRANS);
    return (int) *rb;

  case ACC_VALUE_Y:
    rb[0] = ADXL345_DATAY0;
    i2c(ADXL345_ADDR, rb, 1, rb, sizeof(rb), I2C_REPEATED_START|I2C_END_OF_TRANS);
    return (int) *rb;

  case ACC_VALUE_Z:
    rb[0] = ADXL345_DATAZ0;
    i2c(ADXL345_ADDR, rb, 1, rb, sizeof(rb), I2C_REPEATED_START|I2C_END_OF_TRANS);
    return (int) *rb;
  }

  return 0;
}

static int
configure(int type, int v)
{
  uint8_t poweron[] = {ADXL345_POWER_CTL, 0};

  if (v) poweron[1] = 1<<3;  /* bit 3 enables measurement */
  else   poweron[1] = 0;

  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    i2c_init();
    return i2c(ADXL345_ADDR, poweron, sizeof(poweron), NULL, 0, I2C_END_OF_TRANS);
  }

  return 0;
}

static int
status(int type)
{
  uint8_t buf[] = {ADXL345_POWER_CTL};

  switch(type) {
  case SENSORS_ACTIVE:
    i2c(ADXL345_POWER_CTL, buf, sizeof(buf), buf, sizeof(buf), I2C_REPEATED_START|I2C_END_OF_TRANS);
    return buf[0] & (1<<3);
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

SENSORS_SENSOR(acc_sensor, ACC_SENSOR, value, configure, status);
