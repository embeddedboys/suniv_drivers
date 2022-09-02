
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

#include "include/MAX30102.h"

int main(int argc, char **argv)
{
    int fd;
    int rc;

    fd = open("/dev/i2c-1", O_RDWR);
    if(fd < 0) {
        fprintf(stderr, "failed to open i2c bus\n");
        return -1;
    }

    rc = ioctl(fd, I2C_SLAVE_FORCE, 0x57);
    if(rc < 0) {
        fprintf(stderr, "failed to set slave addr\n");
        return -1;
    }

    printf("MAX30102 demo APP\n");
    printf("Revision ID : 0x%x\n", max30102_read_revision_id(fd));
    printf("PART ID : 0x%x\n", max30102_read_part_id(fd));

    max30102_reset(fd);

    double temp;
    while(1) {
        printf("=========== ");
        temp = max30102_read_temperture(fd);
        printf("Temperture : %0.4f\n", temp);
        sleep(1);
    }

    close(fd);

    return 0;
}