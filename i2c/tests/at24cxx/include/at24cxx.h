#ifndef __AT24CXX_H
#define __AT24CXX_H

#include <i2c/smbus.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

//#define AT24CXX_WRITE_CYCLE_MS 10

struct at24cxx_handle
{
    __u32 device_addr;
    int bus_file;
	char bus_name[20];

    at24cxx_handle()
    {
        device_addr = 255;
        bus_file = -1;
    }
};

class AT24CXX {

private:
	int fd;
    struct at24cxx_handle handle;
    __s32 access(char read_write, __u8 Waddr, int size, union i2c_smbus_data *data);

protected:

public:
    AT24CXX(char *bus_name, __u32 addr);
    ~AT24CXX();

	__s32 init();
    __s32 detect();

	__s32 waiting_write_cycle();

    __s32 write_byte_data(__u32 Waddr, __u32 value);

    __u32 random_read(__u32 Waddr);

    __s32 memory_reset(void);

    

};

#endif