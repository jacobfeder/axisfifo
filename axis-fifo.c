/*  axis-fifo.c
*
*	Copyright 2018 Jacob Feder
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program. If not, see <http://www.gnu.org/licenses/>.

*/

// See Xilinx PG080 document for IP details

// ----------------------------
//           includes
// ----------------------------

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <asm/uaccess.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

// ----------------------------
//          defines
// ----------------------------

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jacob Feder");
MODULE_DESCRIPTION("axis-fifo: interface to the Xilinx AXI-Stream FIFO v4.1 IP core.\n" \
					"Supports only store-forward mode with a 32-bit AXI4-Lite interface. DOES NOT support\n" \
					"- cut-through mode\n" \
					"- AXI4 (non-lite)");

#define DRIVER_NAME "axis_fifo"

// read buffer length in words
#define READ_BUFF_SIZE 128
// write buffer length in words
#define WRITE_BUFF_SIZE 128

// enable to turn on debugging messages
//#define DEBUG

// Macro for printing debug messages inside driver callbacks (e.g. open/close/read/write)
#ifdef DEBUG
	#define printkdbg(fmt, ...) printk(KERN_DEBUG "%s %u: " fmt, DRIVER_NAME, device_wrapper->id, ##__VA_ARGS__)
#else
	#define printkdbg(fmt, ...) ;
#endif

// Macro for printing error messages inside driver callbacks (e.g. open/close/read/write)
#define printkerr(fmt, ...) printk(KERN_ERR "%s %u: " fmt, DRIVER_NAME, device_wrapper->id, ##__VA_ARGS__)

// convert milliseconds to kernel jiffies
#define ms_to_jiffies(ms) ((HZ * ms) / 1000)

// write to IP register
#define write_reg(addr_offset, value) 	iowrite32(value, device_wrapper->base_addr + addr_offset)
#define read_reg(addr_offset) 			ioread32(device_wrapper->base_addr + addr_offset)

// ----------------------------
//     IP register offsets
// ----------------------------

#define XLLF_ISR_OFFSET  0x00000000  // Interrupt Status
#define XLLF_IER_OFFSET  0x00000004  // Interrupt Enable

#define XLLF_TDFR_OFFSET 0x00000008  // Transmit Reset
#define XLLF_TDFV_OFFSET 0x0000000c  // Transmit Vacancy
#define XLLF_TDFD_OFFSET 0x00000010  // Transmit Data
#define XLLF_TLR_OFFSET  0x00000014  // Transmit Length

#define XLLF_RDFR_OFFSET 0x00000018  // Receive Reset
#define XLLF_RDFO_OFFSET 0x0000001c  // Receive Occupancy
#define XLLF_RDFD_OFFSET 0x00000020  // Receive Data
#define XLLF_RLR_OFFSET  0x00000024  // Receive Length
#define XLLF_SRR_OFFSET  0x00000028  // Local Link Reset
#define XLLF_TDR_OFFSET  0x0000002C  // Transmit Destination 
#define XLLF_RDR_OFFSET  0x00000030  // Receive Destination 

// ----------------------------
//     IP register masks
// ----------------------------

#define XLLF_INT_RPURE_MASK       0x80000000 // Receive under-read
#define XLLF_INT_RPORE_MASK       0x40000000 // Receive over-read
#define XLLF_INT_RPUE_MASK        0x20000000 // Receive underrun (empty)
#define XLLF_INT_TPOE_MASK        0x10000000 // Transmit overrun
#define XLLF_INT_TC_MASK          0x08000000 // Transmit complete
#define XLLF_INT_RC_MASK          0x04000000 // Receive complete
#define XLLF_INT_TSE_MASK         0x02000000 // Transmit length mismatch
#define XLLF_INT_TRC_MASK         0x01000000 // Transmit reset complete
#define XLLF_INT_RRC_MASK         0x00800000 // Receive reset complete
#define XLLF_INT_TFPF_MASK        0x00400000 // Tx FIFO Programmable Full,
#define XLLF_INT_TFPE_MASK        0x00200000 // Tx FIFO Programmable Empty
#define XLLF_INT_RFPF_MASK        0x00100000 // Rx FIFO Programmable Full
#define XLLF_INT_RFPE_MASK        0x00080000 // Rx FIFO Programmable Empty
#define XLLF_INT_ALL_MASK         0xfff80000 // All the ints
#define XLLF_INT_ERROR_MASK       0xf2000000 // Error status ints
#define XLLF_INT_RXERROR_MASK     0xe0000000 // Receive Error status ints
#define XLLF_INT_TXERROR_MASK     0x12000000 // Transmit Error status ints

// associated with the reset registers
#define XLLF_RDFR_RESET_MASK        0x000000a5 // receive reset value
#define XLLF_TDFR_RESET_MASK        0x000000a5 // Transmit reset value
#define XLLF_SRR_RESET_MASK         0x000000a5 // Local Link reset value

// ----------------------------
//            types
// ----------------------------

struct axis_fifo_local {
	// interrupt
	int irq;
	// physical memory start address
	unsigned int mem_start;
	// physical memory end address
	unsigned int mem_end;
	// kernel space memory
	void __iomem *base_addr;

	// number of words that can be held in the receive fifo
	unsigned int rx_fifo_depth;
	// number of words that can be held in the transmit fifo
	unsigned int tx_fifo_depth;
	// whether the IP has the receive/transmit fifos enabled
	int has_rx_fifo;
	int has_tx_fifo;

	// mutex for preventing multiple processes opening for read
	struct mutex read_mutex;
	// mutex for preventing multiple processes opening for write
	struct mutex write_mutex;
	// wait queue for asynchronos read 
	wait_queue_head_t read_queue;
	// wait queue for asynchronos write 
	wait_queue_head_t write_queue;
	// write file flags
	unsigned int write_flags;
	// read file flags
	unsigned int read_flags;
	
	// device created by OS
	struct device *os_device;
	// our device
	struct device *device;
	// our unique id
	unsigned int id;
	// our char device number
	dev_t devt;
	// our char device class
	struct class *driver_class;
	// our char device
	struct cdev char_device;

	// flag indicating a fatal error has occured
	int fatal;
};

// ----------------------------
//           globals
// ----------------------------

// number of devices initalized thus far
static unsigned num_devices = 0;
// mutex for num_devices
static struct mutex num_devices_mutex;

// ----------------------------
//    function declarations
// ----------------------------

// device access
static int axis_fifo_open(struct inode *inod, struct file *fil);
static int axis_fifo_close(struct inode *inod, struct file *fil);
static ssize_t axis_fifo_read(struct file *device_file, char __user *buf, size_t len, loff_t *off);
static ssize_t axis_fifo_write(struct file *device_file, const char __user *buf, size_t len, loff_t *off);
static irqreturn_t axis_fifo_irq(int irq, void *device_wrapper);

// sysfs direct register read/write
static ssize_t sysfs_write_isr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_read_isr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_ier(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_read_ier(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t sysfs_write_tdfr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_read_tdfv(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_tdfd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_write_tlr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t sysfs_write_rdfr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_read_rdfo(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_read_rdfd(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_read_rlr(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_write_srr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_write_tdr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_read_rdr(struct device *dev, struct device_attribute *attr, char *buf);

// set up / clean up
static int axis_fifo_probe(struct platform_device *pdev);
static int axis_fifo_remove(struct platform_device *pdev);
static int __init axis_fifo_init(void);
static void __exit axis_fifo_exit(void);

// ----------------------------
//     register w/ kernel
// ----------------------------

// file operations
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = axis_fifo_open,
	.release = axis_fifo_close,
	.read = axis_fifo_read,
	.write = axis_fifo_write
};

#ifdef CONFIG_OF
static struct of_device_id axis_fifo_of_match[] = {
	{ .compatible = "xlnx,axi-fifo-mm-s-4.1", },
	{ },
};
MODULE_DEVICE_TABLE(of, axis_fifo_of_match);
#else
# define axis_fifo_of_match
#endif

static struct platform_driver axis_fifo_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= axis_fifo_of_match,
	},
	.probe		= axis_fifo_probe,
	.remove		= axis_fifo_remove,
};

static DEVICE_ATTR(isr,  S_IRUSR | S_IWUSR, sysfs_read_isr, sysfs_write_isr);
static DEVICE_ATTR(ier,  S_IRUSR | S_IWUSR, sysfs_read_ier, sysfs_write_ier);

static DEVICE_ATTR(tdfr, S_IWUSR, NULL, sysfs_write_tdfr);
static DEVICE_ATTR(tdfv, S_IRUSR, sysfs_read_tdfv, NULL);
static DEVICE_ATTR(tdfd, S_IWUSR, NULL, sysfs_write_tdfd);
static DEVICE_ATTR(tlr,  S_IWUSR, NULL, sysfs_write_tlr);

static DEVICE_ATTR(rdfr, S_IWUSR, NULL, sysfs_write_rdfr);
static DEVICE_ATTR(rdfo, S_IRUSR, sysfs_read_rdfo, NULL);
static DEVICE_ATTR(rdfd, S_IRUSR, sysfs_read_rdfd, NULL);
static DEVICE_ATTR(rlr,  S_IRUSR, sysfs_read_rlr, NULL);
static DEVICE_ATTR(srr,  S_IWUSR, NULL, sysfs_write_srr);
static DEVICE_ATTR(tdr,  S_IWUSR, NULL, sysfs_write_tdr);
static DEVICE_ATTR(rdr,  S_IRUSR, sysfs_read_rdr, NULL);

module_init(axis_fifo_init);
module_exit(axis_fifo_exit);

// ----------------------------
//        implementation
// ----------------------------

// called by the kernel when someone is trying to read from the device
// reads one packet at a time
static ssize_t axis_fifo_read(struct file *device_file, char __user *buf, size_t len, loff_t *off)
{
	// retrieve device struct
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)device_file->private_data;
	unsigned int bytes_available;
	unsigned int words_available;
	unsigned int word;
	unsigned int buff_word;
	int wait_ret;
	u32 read_buff[READ_BUFF_SIZE];

	if (device_wrapper->fatal) {
		printkerr("fatal error before read\n");
		return -EIO;
	}

	if (device_wrapper->read_flags & O_NONBLOCK) {
		// opened in non-blocking mode
		// return if there are no packets available
		if (!read_reg(XLLF_RDFO_OFFSET)) {
			return -EAGAIN;
		}
	} else {
		// opened in blocking mode
		if (!read_reg(XLLF_RDFO_OFFSET)) {
			// wait for a packet available interrupt (or timeout)
			// if nothing is currently available
			wait_ret = wait_event_interruptible_timeout(device_wrapper->read_queue,
									read_reg(XLLF_RDFO_OFFSET), ms_to_jiffies(10));
			if (device_wrapper->fatal) {
				printkerr("fatal error after waking up to read\n");
				return -EIO;
			}
			if (wait_ret == 0) {
				// timeout occured
				return 0;
			} else if (wait_ret == -ERESTARTSYS) {
				// signal received
				return -ERESTARTSYS;
			}
		}
	}

	bytes_available = read_reg(XLLF_RLR_OFFSET);
	if (!bytes_available) {
		printkerr("received a packet of length 0 - fifo core will be reset\n");
		// reset IP core
		write_reg(XLLF_SRR_OFFSET, XLLF_SRR_RESET_MASK);
		return -EIO;
	}

	if (bytes_available > len) {
		printkerr("user read buffer too small (available=%u len=%u) - fifo core will be reset\n", bytes_available, len);
		// reset IP core
		write_reg(XLLF_SRR_OFFSET, XLLF_SRR_RESET_MASK);
		return -EINVAL;
	}

	if (bytes_available % 4) {
		// this probably can't happen unless IP registers were previously mishandled
		printkerr("received a packet that isn't word-aligned - fifo core will be reset\n");
		// reset IP core
		write_reg(XLLF_SRR_OFFSET, XLLF_SRR_RESET_MASK);
		return -ENOSYS;
	}

	words_available = bytes_available / 4;

	// read data into an intermediate buffer, copying the contents
	// to userspace when the buffer is full
	for (word = 0; word < words_available; word++) {
		buff_word = word % READ_BUFF_SIZE;
		read_buff[buff_word] = read_reg(XLLF_RDFD_OFFSET);
		if ((buff_word == READ_BUFF_SIZE - 1) || (word == words_available - 1)) {
			if (copy_to_user(buf + (word - buff_word)*4, read_buff, (buff_word + 1)*4)) {
				printkerr("couldn't copy data to userspace buffer - fifo core will be reset\n");
				// reset IP core
				write_reg(XLLF_SRR_OFFSET, XLLF_SRR_RESET_MASK);
				return -EFAULT;
			}
		}
	}
	
	return bytes_available;
}

// called by the kernel when someone is trying to write to the device
static ssize_t axis_fifo_write(struct file *device_file, const char __user *buf, size_t len, loff_t *off)
{
	// retrieve device struct
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)device_file->private_data;
	unsigned int words_to_write;
	unsigned int word;
	unsigned int buff_word;
	int wait_ret;
	u32 write_buff[WRITE_BUFF_SIZE];

	if (device_wrapper->fatal) {
		printkerr("fatal error before write\n");
		return -EIO;
	}

	if (len % 4) {
		printkerr("tried to send a packet that isn't word-aligned\n");
		return -EINVAL;
	}

	words_to_write = len / 4;

	if (!words_to_write) {
		printkerr("tried to send a packet of length 0\n");
		return -EINVAL;	
	}

	if (words_to_write > device_wrapper->tx_fifo_depth) {
		printkerr("tried to write more words [%u] than slots in the fifo buffer [%u]\n",
					words_to_write, device_wrapper->tx_fifo_depth);
		return -EINVAL;
	}

	if (device_wrapper->write_flags & O_NONBLOCK) {
		// opened in non-blocking mode
		// return if there is not enough room available in the fifo
		if (words_to_write > read_reg(XLLF_TDFV_OFFSET)) {
			return -EAGAIN;
		}
	} else {
		// opened in blocking mode
		if (words_to_write > read_reg(XLLF_TDFV_OFFSET)) {
			// wait for an interrupt (or timeout) if there isn't currently enough room in the fifo
			wait_ret = wait_event_interruptible_timeout(device_wrapper->write_queue,
									words_to_write <= read_reg(XLLF_TDFV_OFFSET), ms_to_jiffies(10));
			if (device_wrapper->fatal) {
				printkerr("fatal error after waking up to write\n");
				return -EIO;
			}
			if (wait_ret == 0) {
				// timeout occured
				return 0;
			} else if (wait_ret == -ERESTARTSYS) {
				// signal received
				return -ERESTARTSYS;
			}	
		}
	}

	// write data from an intermediate buffer into the fifo IP, refilling
	// the buffer with userspace data as needed
	for (word = 0; word < words_to_write; word++) {
		buff_word = word % WRITE_BUFF_SIZE;
		if (buff_word == 0) {
			if (copy_from_user(write_buff, buf + word*4, word <= words_to_write - 
				WRITE_BUFF_SIZE ? WRITE_BUFF_SIZE*4 : (words_to_write % WRITE_BUFF_SIZE)*4)) {
				printkerr("couldn't copy data from userspace buffer - fifo core will be reset\n");
				// reset IP core
				write_reg(XLLF_SRR_OFFSET, XLLF_SRR_RESET_MASK);
				return -EFAULT;
			}
		}
		write_reg(XLLF_TDFD_OFFSET, write_buff[buff_word]);
	}

	// write packet size to fifo
	write_reg(XLLF_TLR_OFFSET, words_to_write*4);

	return (ssize_t)words_to_write*4;
}

// called by the kernel when an interrupt is received
static irqreturn_t axis_fifo_irq(int irq, void *dw)
{
	// retrieve device struct
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)dw;

	unsigned int pending_interrupts;
	do {
		pending_interrupts = read_reg(XLLF_IER_OFFSET) & read_reg(XLLF_ISR_OFFSET);
		if (pending_interrupts & XLLF_INT_RC_MASK) {
			// packet received

			// wake the reader process if it is waiting
			wake_up(&device_wrapper->read_queue);

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RC_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_TC_MASK) {
			// packet sent
			
			// wake the writer process if it is waiting
			wake_up(&device_wrapper->write_queue);

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_TC_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_TFPF_MASK) {
			// transmit fifo programmable full
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_TFPF_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_TFPE_MASK) {
			// transmit fifo programmable empty
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_TFPE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_RFPF_MASK) {
			// receive fifo programmable full
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RFPF_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_RFPE_MASK) {
			// receive fifo programmable empty
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RFPE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_TRC_MASK) {
			// transmit reset complete interrupt
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_TRC_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_RRC_MASK) {
			// receive reset complete interrupt
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RRC_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_RPURE_MASK) {
			// receive fifo under-read error interrupt
			printkerr("receive under-read interrupt\n");

			device_wrapper->fatal = 1;

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RPURE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_RPORE_MASK) {
			// receive over-read error interrupt
			printkerr("receive over-read interrupt\n");

			device_wrapper->fatal = 1;

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RPORE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_RPUE_MASK) {
			// receive underrun error interrupt
			printkerr("receive underrun error interrupt\n");
			
			device_wrapper->fatal = 1;

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_RPUE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_TPOE_MASK) {
			// transmit overrun error interrupt
			printkerr("transmit overrun error interrupt\n");
			
			device_wrapper->fatal = 1;

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_TPOE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts & XLLF_INT_TSE_MASK) {
			// transmit length mismatch error interrupt
			printkerr("transmit length mismatch error interrupt\n");
			
			device_wrapper->fatal = 1;

			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_TSE_MASK & XLLF_INT_ALL_MASK);
		} else if (pending_interrupts) {
			// unknown interrupt type
			printkerr("unknown interrupt(s) 0x%x\n", pending_interrupts);
			
			// clear interrupt
			write_reg(XLLF_ISR_OFFSET, XLLF_INT_ALL_MASK);
		}
	} while (pending_interrupts);

	return IRQ_HANDLED;
}

// called by the kernel when someone is trying to open the device
static int axis_fifo_open(struct inode *inod, struct file *device_file)
{
	// retrieve device wrapper struct
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)container_of(inod->i_cdev, struct axis_fifo_local, char_device);
	// set file attribute to our device wrapper so other functions (e.g. read/write) can access it
	device_file->private_data = device_wrapper;

	printkdbg("opening...\n");

	// lock write access
	if (((device_file->f_flags & O_ACCMODE) == O_WRONLY)) {

		if (!device_wrapper->has_tx_fifo) {
			printkerr("tried to open device for write but the transmit fifo is disabled\n");
			return -EPERM;
		}

		if (!mutex_trylock(&device_wrapper->write_mutex)) {
			printkerr("couldn't acquire write lock\n");
			return -EBUSY;
		}
		device_wrapper->write_flags = device_file->f_flags;

		printkdbg("acquired write lock\n");
	}

	// lock read access
	if ((device_file->f_flags & O_ACCMODE) == O_RDONLY) {

		if (!device_wrapper->has_rx_fifo) {
			printkerr("tried to open device for read but the receive fifo is disabled\n");
			return -EPERM;
		}

		if (!mutex_trylock(&device_wrapper->read_mutex)) {
			printkerr("couldn't acquire read lock\n");
			return -EBUSY;
		}
		device_wrapper->read_flags = device_file->f_flags;

		printkdbg("acquired read lock\n");
	}

	// lock read +  write access
	if ((device_file->f_flags & O_ACCMODE) == O_RDWR) {

		if (!device_wrapper->has_rx_fifo || !device_wrapper->has_tx_fifo) {
			printkerr("tried to open device for read/write but one or both of the receive/transmit fifos are disabled\n");
			return -EPERM;
		}

		if (!mutex_trylock(&device_wrapper->write_mutex)) {
			printkerr("couldn't acquire write lock\n");
			return -EBUSY;
		}
		if (!mutex_trylock(&device_wrapper->read_mutex)) {
			printkerr("couldn't acquire read lock\n");
			mutex_unlock(&device_wrapper->write_mutex);
			printkdbg("released write lock\n");
			return -EBUSY;
		}
		device_wrapper->write_flags = device_file->f_flags;
		device_wrapper->read_flags = device_file->f_flags;
		
		printkdbg("acquired write lock\n");
		printkdbg("acquired read lock\n");
	}

	printkdbg("opened\n");

	return 0;
}

// called by the kernel when someone is trying to close the device
static int axis_fifo_close(struct inode *inod, struct file *device_file)
{
	// retrieve device struct
	struct axis_fifo_local *device_wrapper = container_of(inod->i_cdev, struct axis_fifo_local, char_device);
	// unset file attribute
	device_file->private_data = NULL;

	printkdbg("closing...\n");

	// unlock write access
	if (((device_file->f_flags & O_ACCMODE) == O_WRONLY) ||
		((device_file->f_flags & O_ACCMODE) == O_RDWR)) {
		mutex_unlock(&device_wrapper->write_mutex);
		printkdbg("released write lock\n");
	}

	// unlock read access
	if (((device_file->f_flags & O_ACCMODE) == O_RDONLY) ||
		((device_file->f_flags & O_ACCMODE) == O_RDWR)) {
		mutex_unlock(&device_wrapper->read_mutex);
		printkdbg("released read lock\n");
	}

	printkdbg("closed\n");

	return 0;
}

// functions for reading/writing directly to registers via sysfs

// since all our sysfs read/write functions are essentially identical
#define sysfs_write_function(addr_offset)													\
	struct axis_fifo_local *device_wrapper = dev_get_drvdata(dev);							\
	if (!mutex_trylock(&device_wrapper->write_mutex)) {										\
		printkerr("couldn't acquire write lock\n");											\
		return -EBUSY;																		\
	}																						\
	if (!mutex_trylock(&device_wrapper->read_mutex)) {										\
		printkerr("couldn't acquire read lock\n");											\
		mutex_unlock(&device_wrapper->write_mutex);											\
		printkdbg("released write lock\n");													\
		return -EBUSY;																		\
	}																						\
	printkdbg("acquired locks\n");															\
	if (count != 4) { 																		\
		printkerr("error, sysfs write to address 0x%x expected 4 bytes\n", addr_offset); 	\
		mutex_unlock(&device_wrapper->write_mutex);											\
		mutex_unlock(&device_wrapper->read_mutex);											\
		return -EINVAL; 																	\
	} 																						\
	printkdbg("writing 0x%x to sysfs address 0x%x\n", *(unsigned int *)buf, addr_offset);	\
	write_reg(addr_offset, *(unsigned int __force *)buf);									\
	mutex_unlock(&device_wrapper->write_mutex);												\
	mutex_unlock(&device_wrapper->read_mutex);												\
	printkdbg("released locks\n");															\
	return 4;

#define sysfs_read_function(addr_offset)													\
	struct axis_fifo_local *device_wrapper = dev_get_drvdata(dev);							\
	unsigned int read_val;																	\
	if (!mutex_trylock(&device_wrapper->write_mutex)) {										\
		printkerr("couldn't acquire write lock\n");											\
		return -EBUSY;																		\
	}																						\
	if (!mutex_trylock(&device_wrapper->read_mutex)) {										\
		printkerr("couldn't acquire read lock\n");											\
		mutex_unlock(&device_wrapper->write_mutex);											\
		printkdbg("released write lock\n");													\
		return -EBUSY;																		\
	}																						\
	printkdbg("acquired locks\n");															\
	read_val = read_reg(addr_offset);														\
	printkdbg("read 0x%x from sysfs address 0x%x\n", read_val, addr_offset);				\
	*(unsigned int __force *)buf = read_val;												\
	mutex_unlock(&device_wrapper->write_mutex);												\
	mutex_unlock(&device_wrapper->read_mutex);												\
	printkdbg("released locks\n");															\
	return 4;

static ssize_t sysfs_write_isr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)  { sysfs_write_function(XLLF_ISR_OFFSET) }
static ssize_t sysfs_read_isr(struct device *dev, struct device_attribute *attr, char *buf) 					  { sysfs_read_function(XLLF_ISR_OFFSET) }
static ssize_t sysfs_write_ier(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)  { sysfs_write_function(XLLF_IER_OFFSET) }
static ssize_t sysfs_read_ier(struct device *dev, struct device_attribute *attr, char *buf)						  { sysfs_read_function(XLLF_IER_OFFSET) }

static ssize_t sysfs_write_tdfr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { sysfs_write_function(XLLF_TDFR_OFFSET) }
static ssize_t sysfs_read_tdfv(struct device *dev, struct device_attribute *attr, char *buf)					  { sysfs_read_function(XLLF_TDFV_OFFSET) }
static ssize_t sysfs_write_tdfd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { sysfs_write_function(XLLF_TDFD_OFFSET) }
static ssize_t sysfs_write_tlr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)  { sysfs_write_function(XLLF_TLR_OFFSET) }

static ssize_t sysfs_write_rdfr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { sysfs_write_function(XLLF_RDFR_OFFSET) }
static ssize_t sysfs_read_rdfo(struct device *dev, struct device_attribute *attr, char *buf)					  { sysfs_read_function(XLLF_RDFO_OFFSET) }
static ssize_t sysfs_read_rdfd(struct device *dev, struct device_attribute *attr, char *buf)					  { sysfs_read_function(XLLF_RDFD_OFFSET) }
static ssize_t sysfs_read_rlr(struct device *dev, struct device_attribute *attr, char *buf)						  { sysfs_read_function(XLLF_RLR_OFFSET) }
static ssize_t sysfs_write_srr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)  { sysfs_write_function(XLLF_SRR_OFFSET) }
static ssize_t sysfs_write_tdr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)  { sysfs_write_function(XLLF_TDR_OFFSET) }
static ssize_t sysfs_read_rdr(struct device *dev, struct device_attribute *attr, char *buf)						  { sysfs_read_function(XLLF_RDR_OFFSET) }

// called by the kernel while creating each device (as defined in the device tree)
static int axis_fifo_probe(struct platform_device *pdev)
{
	// interrupt resources
	struct resource *r_irq;
	// IO mem resources
	struct resource *r_mem;
	// OS provided device (from device tree)
	struct device *dev = &pdev->dev;
	struct axis_fifo_local *device_wrapper = NULL;

	// new device/class names
	char device_name[32];
	char class_name[32];

	// error return value
	int rc = 0;

	// IP properties from device tree
	unsigned int rxd_tdata_width;
	unsigned int txc_tdata_width;
	unsigned int txd_tdata_width;
	unsigned int tdest_width;
	unsigned int tid_width;
	unsigned int tuser_width;
	unsigned int data_interface_type;
	unsigned int has_tdest;
	unsigned int has_tid;
	unsigned int has_tkeep;
	unsigned int has_tstrb;
	unsigned int has_tuser;
	unsigned int rx_fifo_depth;
	unsigned int rx_programmable_empty_threshold;
	unsigned int rx_programmable_full_threshold;
	unsigned int axi_id_width;
	unsigned int axi4_data_width;
	unsigned int select_xpm;
	unsigned int tx_fifo_depth;
	unsigned int tx_programmable_empty_threshold;
	unsigned int tx_programmable_full_threshold;
	unsigned int use_rx_cut_through;
	unsigned int use_rx_data;
	unsigned int use_tx_control;
	unsigned int use_tx_cut_through;
	unsigned int use_tx_data;

	// ----------------------------
	//     init wrapper device
	// ----------------------------

	// allocate device wrapper memory
	device_wrapper = (struct axis_fifo_local *) devm_kzalloc(dev, sizeof(struct axis_fifo_local), GFP_KERNEL);
	if (!device_wrapper) {
		printk("couldn't allocate memory for axis-fifo device\n");
		return -ENOMEM;
	}

	// get unique id
	mutex_lock(&num_devices_mutex);
	device_wrapper->id = num_devices;
	num_devices++;
	mutex_unlock(&num_devices_mutex);

	printkdbg("acquired device number %i\n", device_wrapper->id);

	mutex_init(&device_wrapper->read_mutex);
	mutex_init(&device_wrapper->write_mutex);

	printkdbg("initialized mutexes\n");

	init_waitqueue_head(&device_wrapper->read_queue);
	init_waitqueue_head(&device_wrapper->write_queue);

	printkdbg("initialized queues\n");

	// create unique device and class names
	sprintf(device_name, DRIVER_NAME "%i", device_wrapper->id);
	sprintf(class_name, DRIVER_NAME "%i_class", device_wrapper->id);

	printkdbg("device name [%s] class name [%s]\n", device_name, class_name);

	dev_set_drvdata(dev, device_wrapper);
	device_wrapper->os_device = dev;

	// ----------------------------
	//   init device memory space
	// ----------------------------

	// get iospace for the device
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		printkerr("invalid address\n");
		rc = -ENODEV;
		goto error1;
	}

	device_wrapper->mem_start = r_mem->start;
	device_wrapper->mem_end = r_mem->end;

	// request physical memory
	if (!request_mem_region(device_wrapper->mem_start, device_wrapper->mem_end - device_wrapper->mem_start + 1, DRIVER_NAME)) {
		printkerr("couldn't lock memory region at %p\n", (void *)device_wrapper->mem_start);
		rc = -EBUSY;
		goto error1;
	}
	printkdbg("got memory location [0x%x - 0x%x]\n", device_wrapper->mem_start, device_wrapper->mem_end);

	// map physical memory to kernel virtual address space
	device_wrapper->base_addr = ioremap(device_wrapper->mem_start, device_wrapper->mem_end - device_wrapper->mem_start + 1);
	if (!device_wrapper->base_addr) {
		printkerr("couldn't map physical memory\n");
		rc = -EIO;
		goto error2;
	}
	printkdbg("remapped memory to 0x%x\n", (unsigned int)device_wrapper->base_addr);

	// ----------------------------
	//          init IP
	// ----------------------------
	
	#define get_dts_property(name, var)									\
		if (of_property_read_u32(dev->of_node, name, &var) < 0) { 		\
			printkerr("couldn't read IP dts property '" name "'");		\
		}
	get_dts_property("xlnx,axi-str-rxd-tdata-width", rxd_tdata_width)
	get_dts_property("xlnx,axi-str-txc-tdata-width", txc_tdata_width)
	get_dts_property("xlnx,axi-str-txd-tdata-width", txd_tdata_width)
	get_dts_property("xlnx,axis-tdest-width", tdest_width)
	get_dts_property("xlnx,axis-tid-width", tid_width)
	get_dts_property("xlnx,axis-tuser-width", tuser_width)
	get_dts_property("xlnx,data-interface-type", data_interface_type)
	get_dts_property("xlnx,has-axis-tdest", has_tdest)
	get_dts_property("xlnx,has-axis-tid", has_tid)
	get_dts_property("xlnx,has-axis-tkeep", has_tkeep)
	get_dts_property("xlnx,has-axis-tstrb", has_tstrb)
	get_dts_property("xlnx,has-axis-tuser", has_tuser)
	get_dts_property("xlnx,rx-fifo-depth", rx_fifo_depth)
	get_dts_property("xlnx,rx-fifo-pe-threshold", rx_programmable_empty_threshold)
	get_dts_property("xlnx,rx-fifo-pf-threshold", rx_programmable_full_threshold)
	get_dts_property("xlnx,s-axi-id-width", axi_id_width)
	get_dts_property("xlnx,s-axi4-data-width", axi4_data_width)
	get_dts_property("xlnx,select-xpm", select_xpm)
	get_dts_property("xlnx,tx-fifo-depth", tx_fifo_depth)
	get_dts_property("xlnx,tx-fifo-pe-threshold", tx_programmable_empty_threshold)
	get_dts_property("xlnx,tx-fifo-pf-threshold", tx_programmable_full_threshold)
	get_dts_property("xlnx,use-rx-cut-through", use_rx_cut_through)
	get_dts_property("xlnx,use-rx-data", use_rx_data)
	get_dts_property("xlnx,use-tx-ctrl", use_tx_control)
	get_dts_property("xlnx,use-tx-cut-through", use_tx_cut_through)
	get_dts_property("xlnx,use-tx-data", use_tx_data)

	#define assert_dts_property(cond, err, ...)		\
		if (cond) { 								\
			printkerr(err, ##__VA_ARGS__);			\
			rc = -ENOSYS;							\
			goto error2;							\
		}

	assert_dts_property(rxd_tdata_width != 32, "rxd_tdata_width width [%u] unsupported\n", rxd_tdata_width)
	assert_dts_property(txd_tdata_width != 32, "txd_tdata_width width [%u] unsupported\n", txd_tdata_width)
	assert_dts_property(has_tdest, "tdest not supported\n")
	assert_dts_property(has_tid, "tid not supported\n")
	assert_dts_property(has_tkeep, "tkeep not supported\n")
	assert_dts_property(has_tstrb, "tstrb not supported\n")
	assert_dts_property(has_tuser, "tuser not supported\n")
	assert_dts_property(use_rx_cut_through, "rx cut-through not supported\n")
	assert_dts_property(use_tx_cut_through, "tx cut-through not supported\n")
	assert_dts_property(use_tx_control, "tx control not supported\n")

	// TODO: what does select-xpm do?
	// TODO: data_interface_type

	// set device wrapper properties based on IP config
	device_wrapper->rx_fifo_depth = rx_fifo_depth;
	device_wrapper->tx_fifo_depth = tx_fifo_depth;
	device_wrapper->has_rx_fifo = use_rx_data;
	device_wrapper->has_tx_fifo = use_tx_data;
	
	device_wrapper->fatal = 0;

	// reset IP core
	write_reg(XLLF_SRR_OFFSET, XLLF_SRR_RESET_MASK);
	// enable interrupts
	write_reg(XLLF_IER_OFFSET, XLLF_INT_TC_MASK | XLLF_INT_RC_MASK | XLLF_INT_RPURE_MASK |
							   XLLF_INT_RPORE_MASK | XLLF_INT_RPUE_MASK | XLLF_INT_TPOE_MASK |
							   XLLF_INT_TSE_MASK);
	// clear interrupts
	write_reg(XLLF_ISR_OFFSET, XLLF_INT_ALL_MASK);

	// ----------------------------
	//    init device interrupts
	// ----------------------------

	// get IRQ resource
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r_irq) {
		printkerr("no IRQ found at 0x%08x mapped to 0x%08x\n", (unsigned int __force)device_wrapper->mem_start, (unsigned int __force)device_wrapper->base_addr);
		rc = -EIO;
		goto error2;
	}
	printkdbg("found IRQ\n");

	// request IRQ
	device_wrapper->irq = r_irq->start;
	rc = request_irq(device_wrapper->irq, &axis_fifo_irq, 0, DRIVER_NAME, device_wrapper);
	if (rc) {
		printkerr("couldn't allocate interrupt %i.\n", device_wrapper->irq);
		rc = -EIO;
		goto error2;
	}
	printkdbg("initialized IRQ %i\n", device_wrapper->irq);

	// ----------------------------
	//      init char device
	// ----------------------------

	// allocate device number
	if (alloc_chrdev_region(&device_wrapper->devt, 0, 1, DRIVER_NAME) < 0) {
		printkerr("couldn't allocate dev_t\n");
		rc = -EIO;
		goto error3;
	}
	printkdbg("allocated device number major %i minor %i\n", MAJOR(device_wrapper->devt), MINOR(device_wrapper->devt));

	// create driver class
	device_wrapper->driver_class = NULL;
	device_wrapper->driver_class = class_create(THIS_MODULE, class_name);
	if (device_wrapper->driver_class == NULL) {
		printkerr("couldn't create driver class\n");
		rc = -EIO;
		goto error4;
	}
	printkdbg("created driver class\n");

	// create driver file
	device_wrapper->device = NULL;
	device_wrapper->device = device_create(device_wrapper->driver_class, NULL, device_wrapper->devt, NULL, device_name);
	if (device_wrapper->device == NULL) {
		printkerr("couldn't create driver file\n");
		rc = -EIO;
		goto error5;
	}
	dev_set_drvdata(device_wrapper->device, device_wrapper);
	printkdbg("created device file\n");

	// create character device
	cdev_init(&device_wrapper->char_device, &fops);
	if (cdev_add(&device_wrapper->char_device, device_wrapper->devt, 1) < 0) {
		printkerr("couldn't create character device\n");
		rc = -EIO;
		goto error6;
	}
	printkdbg("created character device\n");

	// ----------------------------
	//  add sysfs register entries
	// ----------------------------

	if (device_create_file(device_wrapper->device, &dev_attr_isr)) {
		printkerr("couldn't create sysfs attribute isr\n");
		goto error7;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_ier)) {
		printkerr("couldn't create sysfs attribute ier\n");
		goto error8;
	}
	
	if (device_create_file(device_wrapper->device, &dev_attr_tdfr)) {
		printkerr("couldn't create sysfs attribute tdfr\n");
		goto error9;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tdfv)) {
		printkerr("couldn't create sysfs attribute tdfv\n");
		goto error10;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tdfd)) {
		printkerr("couldn't create sysfs attribute tdfd\n");
		goto error11;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tlr)) {
		printkerr("couldn't create sysfs attribute tlr\n");
		goto error12;
	}
	
	if (device_create_file(device_wrapper->device, &dev_attr_rdfr)) {
		printkerr("couldn't create sysfs attribute rdfr\n");
		goto error13;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rdfo)) {
		printkerr("couldn't create sysfs attribute rdfo\n");
		goto error14;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rdfd)) {
		printkerr("couldn't create sysfs attribute rdfd\n");
		goto error15;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rlr)) {
		printkerr("couldn't create sysfs attribute rlr\n");
		goto error16;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_srr)) {
		printkerr("couldn't create sysfs attribute srr\n");
		goto error17;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tdr)) {
		printkerr("couldn't create sysfs attribute tdr\n");
		goto error18;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rdr)) {
		printkerr("couldn't create sysfs attribute rdr\n");
		goto error19;
	}
	
	dev_info(dev,"axis-fifo created at 0x%08x mapped to 0x%08x, irq=%i, major=%i, minor=%i\n", (unsigned int __force)device_wrapper->mem_start, \
			(unsigned int __force)device_wrapper->base_addr, device_wrapper->irq, MAJOR(device_wrapper->devt), MINOR(device_wrapper->devt));
	
	return 0;

	error19:
	device_remove_file(device_wrapper->device, &dev_attr_tdr);
	error18:
	device_remove_file(device_wrapper->device, &dev_attr_srr);
	error17:
	device_remove_file(device_wrapper->device, &dev_attr_rlr);
	error16:
	device_remove_file(device_wrapper->device, &dev_attr_rdfd);
	error15:
	device_remove_file(device_wrapper->device, &dev_attr_rdfo);
	error14:
	device_remove_file(device_wrapper->device, &dev_attr_rdfr);
	error13:
	device_remove_file(device_wrapper->device, &dev_attr_tlr);
	error12:
	device_remove_file(device_wrapper->device, &dev_attr_tdfd);
	error11:
	device_remove_file(device_wrapper->device, &dev_attr_tdfv);
	error10:
	device_remove_file(device_wrapper->device, &dev_attr_tdfr);
	error9:
	device_remove_file(device_wrapper->device, &dev_attr_ier);
	error8:
	device_remove_file(device_wrapper->device, &dev_attr_isr);
	error7:
	cdev_del(&device_wrapper->char_device);
	error6:
	dev_set_drvdata(device_wrapper->device, NULL);
	device_destroy(device_wrapper->driver_class, device_wrapper->devt);
	error5:
	class_destroy(device_wrapper->driver_class);
	error4:
	unregister_chrdev_region(device_wrapper->devt, 1);
	error3:
	free_irq(device_wrapper->irq, device_wrapper);
	error2:
	release_mem_region(device_wrapper->mem_start, device_wrapper->mem_end - device_wrapper->mem_start + 1);
	error1:
	mutex_destroy(&device_wrapper->read_mutex);
	mutex_destroy(&device_wrapper->write_mutex);
	dev_set_drvdata(dev, NULL);
	return rc;
}

// called by the kernel while removing each device that is defined in the device tree
static int axis_fifo_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct axis_fifo_local *device_wrapper = dev_get_drvdata(dev);

	dev_info(dev, "removing\n");

	device_remove_file(device_wrapper->device, &dev_attr_rdr);
	device_remove_file(device_wrapper->device, &dev_attr_tdr);
	device_remove_file(device_wrapper->device, &dev_attr_srr);
	device_remove_file(device_wrapper->device, &dev_attr_rlr);
	device_remove_file(device_wrapper->device, &dev_attr_rdfd);
	device_remove_file(device_wrapper->device, &dev_attr_rdfo);
	device_remove_file(device_wrapper->device, &dev_attr_rdfr);
	device_remove_file(device_wrapper->device, &dev_attr_tlr);
	device_remove_file(device_wrapper->device, &dev_attr_tdfd);
	device_remove_file(device_wrapper->device, &dev_attr_tdfv);
	device_remove_file(device_wrapper->device, &dev_attr_tdfr);
	device_remove_file(device_wrapper->device, &dev_attr_ier);
	device_remove_file(device_wrapper->device, &dev_attr_isr);
	cdev_del(&device_wrapper->char_device);
	dev_set_drvdata(device_wrapper->device, NULL);
	device_destroy(device_wrapper->driver_class, device_wrapper->devt);
	class_destroy(device_wrapper->driver_class);
	unregister_chrdev_region(device_wrapper->devt, 1);
	free_irq(device_wrapper->irq, device_wrapper);
	release_mem_region(device_wrapper->mem_start, device_wrapper->mem_end - device_wrapper->mem_start + 1);
	mutex_destroy(&device_wrapper->read_mutex);
	mutex_destroy(&device_wrapper->write_mutex);
	dev_set_drvdata(dev, NULL);
	return 0;
}

// called by the kernel when this module is loaded
static int __init axis_fifo_init(void)
{
	printk(KERN_INFO "axis-fifo driver init\n");
	mutex_init(&num_devices_mutex);
	num_devices = 0;
	return platform_driver_register(&axis_fifo_driver);
}

// called by the kernel when this module is unloaded
static void __exit axis_fifo_exit(void)
{
	platform_driver_unregister(&axis_fifo_driver);
	mutex_destroy(&num_devices_mutex);
	printk(KERN_INFO "axis-fifo driver exit\n");
}