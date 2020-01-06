#obj-m += vivi.o
obj-m += fbtft_device.o
lepton-y := palettes.o \
			lepton_cam.o
obj-m += lepton.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean
