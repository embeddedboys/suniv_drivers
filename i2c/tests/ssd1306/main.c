/**
 * @file main.c
 * @author IotaHydrae (writeforever@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2022-08-01
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
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "include/ssd1306.h"

#define TEST_DOC "This document describes how to write an ALSA \
(Advanced Linux Sound Architecture) driver. The document focuses \
mainly on PCI soundcards. In the case of other device types, the \
API might be different, too. However, at least the ALSA kernel \
API is consistent, and therefore it would be still a bit help \
for writing them."

int main(int argc, char **argv)
{
    int fd;
    int rc;
    struct timespec ts;

    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "failed to open i2c bus\n");
        return -1;
    }

    rc = ioctl(fd, I2C_SLAVE_FORCE, SSD1306_ADDRESS);
    if (rc < 0)
    {
        fprintf(stderr, "failed to set slave addr\n");
        return -1;
    }

    ts.tv_nsec = 500 * 1000 * 1000;
    ssd1306_init(fd);
#if 0
    while (1)
    {
        for (int x = 0; x < SSD1306_HOR_RES_MAX; x++)
            for (int y = 0; y < SSD1306_VER_RES_MAX; y++)
                oled_set_pixel(fd, x, y, 1);

        oled_flush(fd);
        // nanosleep(&ts, NULL);
        sleep(1);

        for (int x = 0; x < SSD1306_HOR_RES_MAX; x++)
            for (int y = 0; y < SSD1306_VER_RES_MAX; y++)
                oled_set_pixel(fd, x, y, 0);

        oled_flush(fd);
        sleep(1);

    }
#else
    // for (int x = 0; x < 128; x += 8)
    // {
    //     for (int y = 0; y < 64; y += 16)
    //     {
    //         oled_put_ascii(fd, x, y, 'a');
    //     }
    // }
    while(1){
        oled_clear(fd);
        oled_putascii_string(fd, 0, 0, TEST_DOC);
        oled_flush(fd);
    }

#endif

    close(fd);

    return 0;
}