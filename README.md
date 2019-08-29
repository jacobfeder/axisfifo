# Xilinx AXI-Stream FIFO v4.1/v4.2 IP core driver

This IP core has read and write AXI-Stream FIFOs, the contents of which can be accessed from the AXI4 memory-mapped interface. This is useful for transferring data from a processor into the FPGA fabric. The driver creates a character device that can be read/written to with standard open/read/write/close.

See Xilinx PG080 document for IP details.

The driver currently supports only store-forward mode with a 32-bit
AXI4 Lite interface. DOES NOT support:
- cut-through mode
- AXI4 (non-lite)

You should find a character device in /dev (e.g. /dev/axis_fifo_########) for each AXI-Stream fifo you create in your hardware. The device can be read and written to like a normal file.

# API

To quickly test things, you can write a packet to your fifo from the command line with  
`echo -n -e '\xDE\xAD\xBE\xEF' > /dev/axis_fifo<#>`  
and read from it with  
`cat /dev/axis_fifo<#> | hexdump -C`.

When you write data to the fifo, the block of data is written as a packet as dictated by the length passed to write. When you call read(), a single packet is returned regardless of how many bytes were requested.

For example:
```c
// assuming the fifo is configured in loopback mode in hardware
// e.g. axis tx interface feeds back into axis rx interface
// note that in this case a single file descriptor could be opened for both read and write,
// but separate read/write descriptors are used for illustrative purposes
int f_wr = open("/dev/axis_fifo_43c00000", O_WRONLY);
int f_rd = open("/dev/axis_fifo_43c00000", O_RDONLY);

// writes a packet of 12 bytes (3 words)
write(f_wr, &write_buff, 12);

// returns 12 bytes
ssize_t bytes_read = read(f_rd, &read_buff, 100);

close(f_wr);
close(f_rd);
```
By default, read() and write() will block for one second before timing out. You can change this behavior by loading the module with command line arguments "read_timeout" and "write_timeout" (in milliseconds):

`insmod /lib/modules/4.9.0-xilinx-v2017.4/extra/axis-fifo.ko read_timeout=100 write_timeout=5000`

You can set "read_timeout" / "write_timeout" to a negative value if you want read() / write() to block indefinitely:

`insmod /lib/modules/4.9.0-xilinx-v2017.4/extra/axis-fifo.ko read_timeout=-1 write_timeout=-1`

You can also make non-blocking reads/writes by opening the file with the O_NONBLOCK flag set which will cause read() and write() to return immediately:

```c
int f = open("/dev/axis_fifo_43c00000", O_RDWR | O_NONBLOCK);
```

See fifo-test.c for more detailed usage code and to test functionality/throughput of your FIFO.

See fifo-test-eth.c for more detailed usage code and to test poll() and non-word boundary writes.

See axis-fifo.txt to see example device tree entry.

## Poll

The poll() mechanism is being implemented by considering a user defined minimum packet size using the FIFO's Programmable Empty Threshold values. When using poll to write to the FIFO it makes sure that this minimum number of bytes are available in the FIFO before asserting POLLOUT. When using poll to read from the FIFO it makes sure this minimum number of bytes are available before asserting POLLIN.

When setting up the device tree the values entered into tx-fifo-pe-threshold and rx-fifo-pe-threshold will define this behavior.

* POLLOUT set when Transmit Data FIFO Vacancy (TDFV) register > tx-fifo-pe-threshold

* POLLIN set when the Receive Data FIFO Occupancy (RDFO) register > rx-fifo-pe-threshold

# Sysfs direct register access

You can access the IP registers directly if you wish using sysfs. They are located in  
`/sys/class/axis_fifo/axis_fifo_########/ip_registers/`  
For example, you can read the RDFO with  
`cat /sys/class/axis_fifo/axis_fifo_########/ip_registers/rdfo`  
or write to the fifo/TDFD with  
`echo 0xdeadbeef > /sys/class/axis_fifo/axis_fifo_########/ip_registers/tdfd`.
