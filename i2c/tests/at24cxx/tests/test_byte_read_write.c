#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

int waiting_write_cycle()
{
	int ret;
	struct timespec ts;
	/* waiting for at24cxx internal write cycle. 10ms max */
	ts.tv_sec =0;
	ts.tv_nsec = 10 * 1000 * 1000;
	ret = nanosleep(&ts, NULL);
	if(ret < 0){
		fprintf(stderr, "cannot sleep.\n");
		perror("ERRNO: ");
		return -errno;
	}
}

__s32 smbus_access(int file, char read_write, __u8 Waddr, int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data msgs;

    msgs.read_write = read_write;
    msgs.command = Waddr;
    msgs.size = size;
    msgs.data = data;

    if(ioctl(file, I2C_SMBUS, &msgs) < 0){
        perror("error, failed to access smbus");
        return -errno;
    }
}

__s32 write_byte_data(int file, __u32 Waddr, __u32 value)
{
    union i2c_smbus_data data;
    data.byte = value;
    return smbus_access(file,I2C_SMBUS_WRITE, Waddr, I2C_SMBUS_BYTE_DATA, &data);
}

__u32 random_read(int file, __u32 Waddr)
{
    union i2c_smbus_data data;
    __s32 ret;

    ret = smbus_access(file,I2C_SMBUS_READ, Waddr, I2C_SMBUS_BYTE_DATA, &data);
    if(ret < 0){
        return -1;
    }

    return data.byte;
}

__u32 write_string(int file, __u32 start_addr, const char *s, int size)
{
	for(int i=0; i < size; i++){
		write_byte_data(file, start_addr, *s);
		waiting_write_cycle();
		start_addr++;
		s++;
	}

	return 0;
}

__u32 read_string(int file, __u32 start_addr, int size)
{
	char buf[128];
	for(int i=0; i<size;i++){
		buf[i] = random_read(file, start_addr);
		waiting_write_cycle();
		start_addr++;
	}

	printf("%s", buf);
	return 0;
}


int main(int argc, char const *argv[])
{
    int file;
	__s32 ret;
    __s32 byte = 0x0;
	struct timespec ts;

    file = open("/dev/i2c-0", O_RDWR);
	if(file < 0){
		fprintf(stderr, "failed to open i2c bus\n");
		return -1;
	}
	
	fprintf(stderr, "i2c bus openned. fd:%d\n", file);

    ret = ioctl(file, I2C_SLAVE_FORCE, 0x50);
	if(ret < 0){
		fprintf(stderr, "failed to set slave addr\n");
		return -1;
	}

    write_byte_data(file, 0x00, 0x13);

	waiting_write_cycle();
	
    byte = random_read(file, 0x00);

    printf("0x%x\n", byte);

	char str[] = "I'm using a USB device usually connected on /dev/ttyUSB0\n";
	printf("origin : %s", str);

	write_string(file, 0x00, str, sizeof(str)/sizeof(char));
	
	read_string(file, 0x00,  sizeof(str)/sizeof(char));

    close(file);
    return 0;
}
