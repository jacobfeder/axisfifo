# Zynq Linux kernel driver for AXI-Stream FIFO IP
Supports only store-forward mode with a 32-bit AXI4-Lite interface. DOES NOT support
- cut-through mode
- AXI4 (non-lite)

You should find a character device in /dev (e.g. /dev/axis_fifo0) for each AXI-Stream fifo you create in your hardware. The device can be read and written to like a normal file.

# Quick 'n dirty

To quickly test things, you can write a packet to your fifo from the command line with  
`echo -n -e '\xDE\xAD\xBE\xEF' > /dev/axis_fifo<#>`  
and read from it with  
`cat /dev/axis_fifo<#> | hexdump -C`.

# API

When you write data to the fifo, the block of data is written as a packet as dictated by the length passed to write. When you call read(), a single packet is returned regardless of how many bytes were requested.

For example:
```c
// assuming the fifo is configured in loopback mode in hardware
// e.g. axis tx interface feeds back into axis rx interface
// note that in this case a single file descriptor could be opened for both read and write,
// but separate read/write descriptors are used for illustrative purposes
int f_wr = open("/dev/axis_fifo0", O_WRONLY);
int f_rd = open("/dev/axis_fifo0", O_RDONLY);

// writes a packet of 12 bytes (3 words)
write(f_wr, &write_buff, 12);

// returns 12 bytes
ssize_t bytes_read = read(f_rd, &read_buff, 100);

close(f_wr);
close(f_rd);
```

Data can only be written and read in multiples of words (4 bytes).

By default, read() and write() will block for one second before timing out. You can change this behavior by loading the module with command line arguments "read_timeout" and "write_timeout" (in milliseconds):

`insmod /lib/modules/4.9.0-xilinx-v2017.4/extra/axis-fifo.ko read_timeout=100 write_timeout=5000`

You can set "read_timeout" / "write_timeout" to a negative value if you want read() / write() to block indefinitely:

`insmod /lib/modules/4.9.0-xilinx-v2017.4/extra/axis-fifo.ko read_timeout=-1 write_timeout=-1`

You can also make non-blocking reads/writes by opening the file with the O_NONBLOCK flag set which will cause read() and write() to return immediately:

```c
int f = open("/dev/axis_fifo0", O_RDWR | O_NONBLOCK);
```

See fifo-test.c for more detailed usage code and to test functionality/throughput of your FIFO.

# Sysfs direct register access

You can access the IP registers directly if you wish using sysfs. They are located in  
`/sys/class/axis_fifo<#>_class/axis_fifo<#>/`  
For example, you can read the RDFO with  
`cat /sys/class/axis_fifo<#>_class/axis_fifo<#>/rdfo | hexdump -C`  
or write to the fifo/TDFD with  
`echo -n -e '\xDE\xAD\xBE\xEF' > /sys/class/axis_fifo<#>_class/axis_fifo<#>/tdfd`.