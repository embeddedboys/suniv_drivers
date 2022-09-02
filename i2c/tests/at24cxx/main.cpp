
#include <stdio.h>

#include "at24cxx.h"




int main(int argc, char const *argv[])
{
    __u8 byte = 0;

	AT24CXX at24cxx = AT24CXX("/dev/i2c-1", 0x50);

	at24cxx.detect();

	at24cxx.write_byte_data(0x00, 0x13);
	at24cxx.waiting_write_cycle();
	byte = at24cxx.random_read(0x00);

	printf("0x%x\n", byte);

	//close(file);
	return 0;
}
