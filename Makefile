IOT_HOME = /opt/iot-devkit/1.7.2/sysroots
#PWD:= $(shell pwd)

KDIR:=$(IOT_HOME)/i586-poky-linux/usr/src/kernel
PATH := $(PATH):$(IOT_HOME)/x86_64-pokysdk-linux/usr/bin/i586-poky-linux

CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SROOT=$(IOT_HOME)/i586-poky-linux/

APP = hcsr_tester
MODULE_NAME_1 = hcsr_platform_device.ko
MODULE_NAME_2 = hcsr_platform_driver.ko

GALILEO_PATH = /home/root/
GALILEO_IP = 192.168.0.5

SCRIPT_INSTALL = install_module.sh
SCRIPT_RUN = run_sysfs_script.sh

obj-m = hcsr_platform_device.o hcsr_platform_driver.o

galileo:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules
	$(SROOT) -o $(APP) hcsr_tester.c --sysroot=$(IOT_HOME) -Wall -lpthread
	
scp:
	scp $(MODULE_NAME_1) root@$(GALILEO_IP):$(GALILEO_PATH)
	scp $(MODULE_NAME_2) root@$(GALILEO_IP):$(GALILEO_PATH)
	scp $(SCRIPT_INSTALL) root@$(GALILEO_IP):$(GALILEO_PATH)
	scp $(APP) root@$(GALILEO_IP):$(GALILEO_PATH)
	scp $(SCRIPT_RUN) root@$(GALILEO_IP):$(GALILEO_PATH)
	ssh root@$(GALILEO_IP)

script:
	scp $(SCRIPT_INSTALL) root@$(GALILEO_IP):$(GALILEO_PATH)
	scp $(SCRIPT_RUN) root@$(GALILEO_IP):$(GALILEO_PATH)

clean:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SROOT)/usr/src/kernel M=$(PWD) clean
