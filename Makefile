obj-m += ex111touchscreen.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	cp $(PWD)/ex111touchscreen.ko /lib/modules/$(shell uname -r)/extra/
	depmod -a