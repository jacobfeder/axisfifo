obj-m := axis-fifo.o

SRC := $(shell pwd)
KERNEL_SRC ?= $(KDIR) # buildroot uses KDIR instead of KERNEL_SRC

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
