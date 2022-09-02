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

/* the address of bh1750 depends on the level of pin ADDR*/
#ifndef BH1750_PIN_ADDR_VAL
    #warning "BH1750_PIN_ADDR_VAL is not defined, using default value 'L'"
    #define BH1750_PIN_ADDR_VAL 'L'
#endif

#if BH1750_PIN_ADDR_VAL == 'H'
    #define BH1750_SLACE_ADDRESS 0x53
#elif BH1750_PIN_ADDR_VAL == 'L'
    #define BH1750_SLACE_ADDRESS 0x23
#else
    #error "BH1750_PIN_ADDR_VAL is not defined correctly"
#endif

#define BH1750_COMMAND_POWER_DOWN   0x00
#define BH1750_COMMAND_POWER_ON     0x01
#define BH1750_COMMAND_RESET        0x07
#define BH1750_COMMAND_CONTINUOUS_HIGH_RES_MODE     0x10
#define BH1750_COMMAND_CONTINUOUS_HIGH_RES_MODE_2   0x11
#define BH1750_COMMAND_CONTINUOUS_LOW_RES_MODE      0x13
#define BH1750_COMMAND_ONE_TIME_HIGH_RES_MODE       0x20
#define BH1750_COMMAND_ONE_TIME_HIGH_RES_MODE_2     0x21
#define BH1750_COMMAND_ONE_TIME_LOW_RES_MODE        0x23

/**
 * @brief used to access the i2c bus
 * 
 * @param fd file descriptor of the i2c bus
 * @param rw read or write
 * @param cmd the command to send
 * @param size size of total bytes to read or write
 * @param data data to send or read
 * @return return 0 on success, -errno on error
 */
static int smbus_access(int fd, __u8 rw, __u8 cmd, __u32 size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data msgs;

    msgs.read_write = rw;
    msgs.command = cmd;
    msgs.size = size;
    msgs.data = data;

    if(ioctl(fd, I2C_SMBUS, &msgs) < 0){
        perror("error, failed to access smbus");
        return -errno;
    }

    return 0;
}

/**
 * @brief write a command to the bh1750
 * 
 * @param fd file descriptor of the i2c bus
 * @param cmd the command to send
 * @return return sumbus_access() call result
 */
int bh1750_write_cmd(int fd, __u8 cmd)
{
    return smbus_access(fd, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE, NULL);
}

int bh1750_read_word(int fd)
{
    union i2c_smbus_data data;
    struct i2c_smbus_ioctl_data msg;

    msg.read_write = I2C_SMBUS_READ;
    msg.size = I2C_SMBUS_WORD_DATA;
    msg.data = &data;

    if(ioctl(fd, I2C_SMBUS, &msg) < 0){
        perror("error, failed to access smbus");
        return -errno;
    }
    printf("raw data: %d\n", data.word);

    return data.word;
}

int main(int argc, char **argv)
{
    int fd;
    int rc;
    int loop = 1;

    /* open i2c bus */
    fd = open("/dev/i2c-1", O_RDWR);
	if(fd < 0){
		fprintf(stderr, "failed to open i2c bus\n");
		return -1;
	}

    /* set slave deivce addr */
    rc = ioctl(fd, I2C_SLAVE_FORCE, 0x23);
	if(rc < 0){
		fprintf(stderr, "failed to set slave addr\n");
		return -1;
	}

    __u16 raw_data;
    __u8 high_byte, low_byte;
    float lux;
    /* read loop */
    while(loop) {
        /* we do one time L-res mode */
        bh1750_write_cmd(fd, 0x23);
        /* need a delay, manual said max 24ms, we sleep for 50ms */
        usleep(50000);
        /* read data */
        raw_data = bh1750_read_word(fd);
        high_byte = (raw_data >> 8) & 0xFF;
        low_byte = raw_data & 0xFF;

        /* convert raw data to lux */
        // lux = (high_byte * 256 + low_byte) / 1.2;
        lux = raw_data / 1.2;
        printf("lux: %f\n", lux);

	/* sleep 200 ms for the next read */
	usleep(200 * 1000);
    }

    close(fd);

    return 0;
}
