/**
 * @file MAX30102.h
 * @author IotaHydrae (writeforever@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-21
 * 
 * MIT License
 * 
 * Copyright 2022 IotaHydrae(writeforever@foxmail.com)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * 
 */

#ifndef __MAX30102_H
#define __MAX30102_H

#define BIT(x) (1 << x)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
/*
#define MAX30102_REGISTER_READ_OPR(cmd,size) ({
    union i2c_smbus_data data;
})
*/

/**
 * Register Map of MAX30102
 * 0x00 ~ 0xFF
 */

/* -------------- BASIC -------------- */
#define MAX30102_INTERRUPT_STATUS_1 (0x00)
#define MAX30102_INTERRUPT_STATUS_2 (0x01)
#define MAX30102_INTERRUPT_ENABLE_1 (0x02)
#define MAX30102_INTERRUPT_ENABLE_2 (0x03)

/* -------------- FIFO -------------- */
#define MAX30102_FIFO_WRITE_POINTER (0x04)
#define MAX30102_OVERFLOW_COUNTER   (0x05)
#define MAX30102_FIFO_READ_POINTER  (0x06)
#define MAX30102_FIFO_DATA_REGISTER (0x07)

/* -------------- CONFIG -------------- */
#define MAX30102_FIFO_CONFIG        (0x08)
#define MAX30102_MODE_CONFIG        (0x09)
#define MAX30102_SPO2_CONFIG        (0x0A)
//      MAX30102_RESERVED           (0x0B)
#define MAX30102_LED1_PULSE_AMPLITUDE   (0x0C)
#define MAX30102_LED2_PULSE_AMPLITUDE   (0x0D)
//      MAX30102_RESERVED           (0x0E)
//      MAX30102_RESERVED           (0x0F)
#define MAX30102_PROXIMITY_MODE_LED_PULSE_AMPLITUDE (0x10)
#define MAX30102_MULTI_LED_MODE_CONTROL_REG1    (0x11)
#define MAX30102_MULTI_LED_MODE_CONTROL_REG2    (0x12)
//      MAX30102_RESERVED           (0x13 ~ 0x17)
//      MAX30102_RESERVED           (0x18 ~ 0x1E)

/* -------------- DIE TEMPERATURE -------------- */
#define MAX30102_DIE_TEMP_INTEGER    (0x1F)
#define MAX30102_DIE_TEMP_FRACTION   (0x20)
#define MAX30102_DIE_TEMP_CONFIG     (0x21)
//      MAX30102_RESERVED           (0x22 ~ 0x2F)

/* -------------- PROXIMITY FUNCTION -------------- */
#define MAX30102_PROXIMITY_INTERRUPT_THRESHOLD  (0x30)

//      Don't know, might be reserved    (0x31 ~ 0xFE)

/* -------------- PART ID -------------- */
#define MAX30102_REVISION_ID    (0xFE)
#define MAX30102_PART_ID        (0xFF)

/**
 * Important Register Bits of MAX30102
 */
#define MAX30102_TEMP_ENABLE    BIT(0)
#define MAX30102_RESET          BIT(6)

void max30102_reset(int fd);

__u8 max30102_read_revision_id(int fd);
__u8 max30102_read_part_id(int fd);

double max30102_read_temperture(int fd);

#endif /* __MAX30102_H */