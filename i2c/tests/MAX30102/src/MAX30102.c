/**
 * @file MAX30102.c
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

#include <stdio.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "MAX30102.h"

/**
 * @brief easy smbus access
 *
 * @param fd fd of i2c/smbus
 * @param read_write direction of operation
 * @param addr register addr of device
 * @param size how many bytes of this operation
 * @param data recv or send data
 * @return int return -errno on error, 0 on success
 */
static int smbus_access(int fd, char read_write, __u8 addr, int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data msgs;

    /* if i2c bus has not been openned, we don't need go further */
    if (fd < 0)
    {
        printf("i2c/smbus has not been openned!");
        return -1;
    }

    /* The read_write will be I2C_SMBUS_READ | I2C_SMBUS_WRTIE
     * the addr will be any add of device
     * the size will be I2C_SMBUS_BYTE | I2C_SMBUS_BYTE_DATA ..
     * te data will be used to IO data
     */
    msgs.read_write = read_write;
    msgs.command = addr;
    msgs.size = size;
    msgs.data = data;

    if (ioctl(fd, I2C_SMBUS, &msgs) < 0)
    {
        perror("error, failed to access smbus");
        return -errno;
    }

    return 0;
}

/**
 * @brief read the temperture sensor value
 *
 * @param fd fd of i2c/smbus
 * @return double
 */
double max30102_read_temperture(int fd)
{
    __u8 temp_integer, temp_fraction;
    __s32 rc;
    double temperture;
    union i2c_smbus_data data;

    /* 1. trigger temperature reading from the temperature sensor */
    data.byte = MAX30102_TEMP_ENABLE;
    rc = smbus_access(fd, I2C_SMBUS_WRITE, MAX30102_DIE_TEMP_CONFIG, I2C_SMBUS_BYTE_DATA, &data);

    /* 2. read integer value from MAX30102_DIE_TEMP_INTEGER */
    rc = smbus_access(fd, I2C_SMBUS_READ, MAX30102_DIE_TEMP_INTEGER, I2C_SMBUS_BYTE_DATA, &data);
    temp_integer = data.byte;

    /* 3. read fraction value from MAX30102_DIE_TEMP_FRACTION */
    rc = smbus_access(fd, I2C_SMBUS_READ, MAX30102_DIE_TEMP_FRACTION, I2C_SMBUS_BYTE_DATA, &data);
    temp_fraction = data.byte;

    /* Temperture = Tinteger + Tfraction*0.0625 ℃ */
    temperture = temp_integer + (temp_fraction * 0.0625);

    return temperture;
}

/**
 * @brief reset max30102
 * reset MAX30102 by set B6 of MAX30102_MODE_CONFIG
 * @param fd
 */
void max30102_reset(int fd)
{
    union i2c_smbus_data data;

    /* set B6 of MAX30102_MODE_CONFIG */
    data.byte = MAX30102_RESET;
    smbus_access(fd, I2C_SMBUS_WRITE, MAX30102_MODE_CONFIG, I2C_SMBUS_BYTE_DATA, &data);
}

/**
 * @brief read revision id from max30102
 *
 * @param fd fd of i2c bus
 * @return __u8
 */
__u8 max30102_read_revision_id(int fd)
{
    __s32 rc;
    union i2c_smbus_data data;

    rc = smbus_access(fd, I2C_SMBUS_READ, MAX30102_REVISION_ID, I2C_SMBUS_BYTE_DATA, &data);
    if (rc < 0)
        return -1;
    return data.byte;
}

/**
 * @brief read part id from max30102
 *
 * @param fd fd of i2c bus
 * @return __u8
 */
__u8 max30102_read_part_id(int fd)
{
    int rc;
    union i2c_smbus_data data;

    rc = smbus_access(fd, I2C_SMBUS_READ, MAX30102_PART_ID, I2C_SMBUS_BYTE_DATA, &data);
    if (rc < 0)
        return -1;
    return data.byte;
}