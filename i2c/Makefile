
# local kernel build dir
KERN_DIR:=/lib/modules/$(shell uname -r)/build

# users kernel dir
# KERN_DIR:=/home/user/linux

MODULE_NAME:=i2c-suniv

all:
	make -C $(KERN_DIR) M=`pwd` modules 

clean:
	make -C $(KERN_DIR) M=`pwd` modules clean

insmod: $(MODULE_NAME).ko
	sudo insmod $(MODULE_NAME).ko
	./klogcat.sh

rmmod: $(MODULE_NAME.ko)
	sudo rmmod $(MODULE_NAME)
	./klogcat.sh

obj-m+=$(MODULE_NAME).o
