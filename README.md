# Zynq SoC Linux kernel driver for Xilinx AXI-Stream FIFO IP
Supports only store-forward mode with a 32-bit AXI4-Lite interface. DOES NOT support
- cut-through mode
- AXI4 (non-lite)

You should find a character device in /dev (e.g. /dev/axis_fifo0) for each AXI-Stream fifo you create in your hardware. The device can be read and written to like a normal file.

For example, you can write a packet to your fifo with
```bash
echo -n -e '\xDE\xAD\xBE\xEF' > /dev/axis_fifo<#>
```
and read from it with
```bash
cat /dev/axis_fifo<#> | hexdump -C
```

When you write data to the fifo, the block of data is written as a packet as dictated by the length passed to write. When you call read(), a single packet is returned regardless of how many bytes were requested.

For example:
```c
// assuming the fifo is configured in loopback mode in hardware
// e.g. axis tx interface feeds back into axis rx interface
int f_wr = open("/dev/axis_fifo0", O_WRONLY);
int f_rd = open("/dev/axis_fifo0", O_RDONLY);

// writes a packet of 12 bytes (3 words)
write(f_wr, &write_buff, 12);

// returns 12 bytes (assuming the packet is received in hardware at this point)
ssize_t bytes_read = read(f_rd, &read_buff, 100);

close(f_wr);
close(f_rd);
```

Data can only be written and read in multiples of words (4 bytes).

You can also access the registers directly if you wish using sysfs. They are located in
```bash
/sys/class/axis_fifo<#>_class/axis_fifo<#>/
```
For example, you can read the RDFO with
```bash
cat /sys/class/axis_fifo<#>_class/axis_fifo<#>/rdfo | hexdump -C
```
or write to the fifo/TDFR with
```bash
echo -n -e '\xDE\xAD\xBE\xEF' > /sys/class/axisfifo<#>_class/axisfifo<#>/tdfd
```

You can use fifo_test.c to test functionality/throughput of your FIFO.