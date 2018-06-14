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
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jacob Feder");
MODULE_DESCRIPTION("axis-fifo: interface to the Xilinx AXI-Stream FIFO v4.1 " \
		"IP core.\nSupports only store-forward mode with a 32-bit " \
		"AXI4-Lite interface. DOES NOT support\n"										  \
			"- cut-through mode\n"											  \
			"- AXI4 (non-lite)");

// ----------------------------
//          defines
// ----------------------------

#define DRIVER_NAME "axis_fifo"

// read buffer length in words
#define READ_BUFF_SIZE 128
// write buffer length in words
#define WRITE_BUFF_SIZE 128

// convert milliseconds to kernel jiffies
#define ms_to_jiffies(ms) ((HZ * ms) / 1000)

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
	// lock for reading waitqueue
	spinlock_t read_queue_lock;
	// wait queue for asynchronos write 
	wait_queue_head_t write_queue;
	// lock for writing waitqueue
	spinlock_t write_queue_lock;
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
};

// ----------------------------
//           globals
// ----------------------------

// number of devices initalized thus far
static unsigned num_fifo_devices = 0;
// mutex for num_fifo_devices
static struct mutex num_fifo_devices_mutex;

// ms to wait before blocking read() times out
static int read_timeout = 1000;
// ms to wait before blocking write() times out
static int write_timeout = 1000;

// ----------------------------
//    function declarations
// ----------------------------

static void reset_ip_core(struct axis_fifo_local *device_wrapper);

// device access
static int axis_fifo_open(struct inode *inod, struct file *fil);
static int axis_fifo_close(struct inode *inod, struct file *fil);
static ssize_t axis_fifo_read(struct file *device_file, char __user *buf, 
				size_t len, loff_t *off);
static ssize_t axis_fifo_write(struct file *device_file, const char __user *buf,
				size_t len, loff_t *off);
static irqreturn_t axis_fifo_irq(int irq, void *device_wrapper);

// sysfs direct register read/write
static ssize_t sysfs_write_isr(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count);
static ssize_t sysfs_read_isr(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t sysfs_write_ier(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sysfs_read_ier(struct device *dev,
				struct device_attribute *attr,
				char *buf);

static ssize_t sysfs_write_tdfr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sysfs_read_tdfv(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t sysfs_write_tdfd(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sysfs_write_tlr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

static ssize_t sysfs_write_rdfr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sysfs_read_rdfo(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t sysfs_read_rdfd(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t sysfs_read_rlr(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t sysfs_write_srr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sysfs_write_tdr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sysfs_read_rdr(struct device *dev,
				struct device_attribute *attr,
				char *buf);

// set up / clean up
static int axis_fifo_probe(struct platform_device *pdev);
static int axis_fifo_remove(struct platform_device *pdev);
static int __init axis_fifo_init(void);
static void __exit axis_fifo_exit(void);

// ----------------------------
//     register w/ kernel
// ----------------------------

// command-line arguments
module_param(read_timeout, int, S_IRUGO);
MODULE_PARM_DESC(read_timeout, "ms to wait before blocking read() timing " \
				"out; set to -1 for no timeout");
module_param(write_timeout, int, S_IRUGO);
MODULE_PARM_DESC(write_timeout, "ms to wait before blocking write() timing " \
				"out; set to -1 for no timeout");

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

static DEVICE_ATTR(isr, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
					sysfs_read_isr, sysfs_write_isr);
static DEVICE_ATTR(ier, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP,
					sysfs_read_ier, sysfs_write_ier);

static DEVICE_ATTR(tdfr, S_IWUSR | S_IWGRP, NULL, sysfs_write_tdfr);
static DEVICE_ATTR(tdfv, S_IRUSR | S_IRGRP, sysfs_read_tdfv, NULL);
static DEVICE_ATTR(tdfd, S_IWUSR | S_IWGRP, NULL, sysfs_write_tdfd);
static DEVICE_ATTR(tlr,  S_IWUSR | S_IWGRP, NULL, sysfs_write_tlr);

static DEVICE_ATTR(rdfr, S_IWUSR | S_IWGRP, NULL, sysfs_write_rdfr);
static DEVICE_ATTR(rdfo, S_IRUSR | S_IRGRP, sysfs_read_rdfo, NULL);
static DEVICE_ATTR(rdfd, S_IRUSR | S_IRGRP, sysfs_read_rdfd, NULL);
static DEVICE_ATTR(rlr,  S_IRUSR | S_IRGRP, sysfs_read_rlr, NULL);
static DEVICE_ATTR(srr,  S_IWUSR | S_IWGRP, NULL, sysfs_write_srr);
static DEVICE_ATTR(tdr,  S_IWUSR | S_IWGRP, NULL, sysfs_write_tdr);
static DEVICE_ATTR(rdr,  S_IRUSR | S_IRGRP, sysfs_read_rdr, NULL);

module_init(axis_fifo_init);
module_exit(axis_fifo_exit);

// ----------------------------
//        implementation
// ----------------------------

static void reset_ip_core(struct axis_fifo_local *device_wrapper)
{
	iowrite32(XLLF_SRR_RESET_MASK,
		device_wrapper->base_addr + XLLF_SRR_OFFSET);
	iowrite32(XLLF_TDFR_RESET_MASK,
		device_wrapper->base_addr + XLLF_TDFR_OFFSET);
	iowrite32(XLLF_RDFR_RESET_MASK,
		device_wrapper->base_addr + XLLF_RDFR_OFFSET);
	iowrite32(XLLF_INT_TC_MASK | XLLF_INT_RC_MASK | XLLF_INT_RPURE_MASK |
		XLLF_INT_RPORE_MASK | XLLF_INT_RPUE_MASK | XLLF_INT_TPOE_MASK |
		XLLF_INT_TSE_MASK,
		device_wrapper->base_addr + XLLF_IER_OFFSET);
	iowrite32(XLLF_INT_ALL_MASK,
		device_wrapper->base_addr + XLLF_ISR_OFFSET);
}

// called by the kernel when someone is trying to read from the device
// reads one packet at a time
static ssize_t axis_fifo_read(struct file *device_file, char __user *buf,
				size_t len, loff_t *off)
{
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)device_file->private_data;
	unsigned int bytes_available;
	unsigned int words_available;
	unsigned int word;
	unsigned int buff_word;
	int wait_ret;
	u32 read_buff[READ_BUFF_SIZE];

	if (device_wrapper->read_flags & O_NONBLOCK) {
		// opened in non-blocking mode
		// return if there are no packets available
		if (!ioread32(device_wrapper->base_addr + XLLF_RDFO_OFFSET)) {
			return -EAGAIN;
		}
	} else {
		// opened in blocking mode

		// wait for a packet available interrupt (or timeout)
		// if nothing is currently available
		spin_lock_irq(&device_wrapper->read_queue_lock);
		if (read_timeout < 0) {
			wait_ret = wait_event_interruptible_lock_irq_timeout(
				device_wrapper->read_queue,
				ioread32(device_wrapper->base_addr +
					XLLF_RDFO_OFFSET),
				device_wrapper->read_queue_lock,
				MAX_SCHEDULE_TIMEOUT);
		} else {
			wait_ret = wait_event_interruptible_lock_irq_timeout(
				device_wrapper->read_queue,
				ioread32(device_wrapper->base_addr +
					XLLF_RDFO_OFFSET),
				device_wrapper->read_queue_lock,
				ms_to_jiffies(read_timeout));
		}
		spin_unlock_irq(&device_wrapper->read_queue_lock);

		if (wait_ret == 0) {
			// timeout occured
			dev_dbg(device_wrapper->os_device, "read timeout");
			return 0;
		} else if (wait_ret > 0) {
			// packet available
		} else if (wait_ret == -ERESTARTSYS) {
			// signal received
			return -ERESTARTSYS;
		} else {
			dev_err(device_wrapper->os_device,
					"wait_event_interruptible_timeout() error in read (wait_ret=%i)\n",
					wait_ret);
			return wait_ret;
		}
	}

	bytes_available = ioread32(device_wrapper->base_addr + XLLF_RLR_OFFSET);
	if (!bytes_available) {
		dev_err(device_wrapper->os_device,
				"received a packet of length 0 - fifo core will be reset\n");
		reset_ip_core(device_wrapper);
		return -EIO;
	}

	if (bytes_available > len) {
		dev_err(device_wrapper->os_device,
				"user read buffer too small (available bytes=%u " \
				"user buffer bytes=%u) - fifo core will be reset\n",
				bytes_available, len);
		reset_ip_core(device_wrapper);
		return -EINVAL;
	}

	if (bytes_available % 4) {
		// this probably can't happen unless IP
		// registers were previously mishandled
		dev_err(device_wrapper->os_device,
				"received a packet that isn't word-aligned - fifo core will be reset\n");
		reset_ip_core(device_wrapper);
		return -ENOSYS;
	}

	words_available = bytes_available / 4;

	// read data into an intermediate buffer, copying the contents
	// to userspace when the buffer is full
	for (word = 0; word < words_available; word++) {
		buff_word = word % READ_BUFF_SIZE;
		read_buff[buff_word] = ioread32(device_wrapper->base_addr +
						XLLF_RDFD_OFFSET);
		if ((buff_word == READ_BUFF_SIZE - 1) || (word == words_available - 1)) {
			if (copy_to_user(buf + (word - buff_word) * 4,
					read_buff,
					(buff_word + 1) * 4)) {
				// this occurs if the user passes an invalid pointer
				dev_err(device_wrapper->os_device,
					"couldn't copy data to userspace buffer - fifo core will be reset\n");
				reset_ip_core(device_wrapper);
				return -EFAULT;
			}
		}
	}
	
	return bytes_available;
}

// called by the kernel when someone is trying to write to the device
static ssize_t axis_fifo_write(struct file *device_file, const char __user *buf,
				size_t len, loff_t *off)
{
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)device_file->private_data;
	unsigned int words_to_write;
	unsigned int word;
	unsigned int buff_word;
	int wait_ret;
	u32 write_buff[WRITE_BUFF_SIZE];

	if (len % 4) {
		dev_err(device_wrapper->os_device,
				"tried to send a packet that isn't word-aligned\n");
		return -EINVAL;
	}

	words_to_write = len / 4;

	if (!words_to_write) {
		dev_err(device_wrapper->os_device,
				"tried to send a packet of length 0\n");
		return -EINVAL;	
	}

	if (words_to_write > device_wrapper->tx_fifo_depth) {
		dev_err(device_wrapper->os_device, "tried to write more words [%u] " \
			"than slots in the fifo buffer [%u]\n",
			words_to_write, device_wrapper->tx_fifo_depth);
		return -EINVAL;
	}

	if (device_wrapper->write_flags & O_NONBLOCK) {
		// opened in non-blocking mode
		// return if there is not enough room available in the fifo
		if (words_to_write > ioread32(device_wrapper->base_addr +
						XLLF_TDFV_OFFSET)) {
			return -EAGAIN;
		}
	} else {
		// opened in blocking mode

		// wait for an interrupt (or timeout) if there isn't
		// currently enough room in the fifo
		spin_lock_irq(&device_wrapper->write_queue_lock);
		if (write_timeout < 0) {
			wait_ret = wait_event_interruptible_lock_irq_timeout(
					device_wrapper->write_queue,
					ioread32(device_wrapper->base_addr +
						XLLF_TDFV_OFFSET) >= words_to_write,
					device_wrapper->write_queue_lock,
					MAX_SCHEDULE_TIMEOUT);
		} else {
			wait_ret = wait_event_interruptible_lock_irq_timeout(
					device_wrapper->write_queue,
					ioread32(device_wrapper->base_addr +
						XLLF_TDFV_OFFSET) >= words_to_write,
					device_wrapper->write_queue_lock,
					ms_to_jiffies(write_timeout));
		}
		spin_unlock_irq(&device_wrapper->write_queue_lock);
		
		if (wait_ret == 0) {
			// timeout occured
			dev_dbg(device_wrapper->os_device, "write timeout\n");
			return 0;
		} else if (wait_ret > 0) {
			// packet available
		} else if (wait_ret == -ERESTARTSYS) {
			// signal received
			return -ERESTARTSYS;
		} else {
			dev_err(device_wrapper->os_device,
				"wait_event_interruptible_timeout() error in write (wait_ret=%i)\n",
				wait_ret);
			return wait_ret;
		}
	}

	// write data from an intermediate buffer into the fifo IP, refilling
	// the buffer with userspace data as needed
	for (word = 0; word < words_to_write; word++) {
		buff_word = word % WRITE_BUFF_SIZE;
		if (buff_word == 0) {
			if (copy_from_user(write_buff, buf + word * 4,
				word <= words_to_write - WRITE_BUFF_SIZE ?
				WRITE_BUFF_SIZE * 4 :
				(words_to_write % WRITE_BUFF_SIZE) * 4)) {
				// this occurs if the user
				// passes an invalid pointer
				dev_err(device_wrapper->os_device,
					"couldn't copy data from userspace buffer - fifo core will be reset\n");
				reset_ip_core(device_wrapper);
				return -EFAULT;
			}
		}
		iowrite32(write_buff[buff_word],
				device_wrapper->base_addr + XLLF_TDFD_OFFSET);
	}

	// write packet size to fifo
	iowrite32(words_to_write*4,
			device_wrapper->base_addr + XLLF_TLR_OFFSET);

	return (ssize_t)words_to_write*4;
}

// called by the kernel when an interrupt is received
static irqreturn_t axis_fifo_irq(int irq, void *dw)
{
	// retrieve device struct
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)dw;

	unsigned int pending_interrupts;
	do {
		pending_interrupts = ioread32(device_wrapper->base_addr +
					XLLF_IER_OFFSET) &
					ioread32(device_wrapper->base_addr +
					XLLF_ISR_OFFSET);
		if (pending_interrupts & XLLF_INT_RC_MASK) {
			// packet received

			// wake the reader process if it is waiting
			wake_up(&device_wrapper->read_queue);

			// clear interrupt
			iowrite32(XLLF_INT_RC_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_TC_MASK) {
			// packet sent
			
			// wake the writer process if it is waiting
			wake_up(&device_wrapper->write_queue);

			// clear interrupt
			iowrite32(XLLF_INT_TC_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_TFPF_MASK) {
			// transmit fifo programmable full
			
			// clear interrupt
			iowrite32(XLLF_INT_TFPF_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_TFPE_MASK) {
			// transmit fifo programmable empty

			// clear interrupt
			iowrite32(XLLF_INT_TFPE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_RFPF_MASK) {
			// receive fifo programmable full
			
			// clear interrupt
			iowrite32(XLLF_INT_RFPF_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_RFPE_MASK) {
			// receive fifo programmable empty
			
			// clear interrupt
			iowrite32(XLLF_INT_RFPE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_TRC_MASK) {
			// transmit reset complete interrupt
			
			// clear interrupt
			iowrite32(XLLF_INT_TRC_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_RRC_MASK) {
			// receive reset complete interrupt
			
			// clear interrupt
			iowrite32(XLLF_INT_RRC_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_RPURE_MASK) {
			// receive fifo under-read error interrupt
			dev_err(device_wrapper->os_device,
				"receive under-read interrupt\n");

			// clear interrupt
			iowrite32(XLLF_INT_RPURE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_RPORE_MASK) {
			// receive over-read error interrupt
			dev_err(device_wrapper->os_device,
				"receive over-read interrupt\n");

			// clear interrupt
			iowrite32(XLLF_INT_RPORE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_RPUE_MASK) {
			// receive underrun error interrupt
			dev_err(device_wrapper->os_device,
				"receive underrun error interrupt\n");
			
			// clear interrupt
			iowrite32(XLLF_INT_RPUE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_TPOE_MASK) {
			// transmit overrun error interrupt
			dev_err(device_wrapper->os_device,
				"transmit overrun error interrupt\n");

			// clear interrupt
			iowrite32(XLLF_INT_TPOE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts & XLLF_INT_TSE_MASK) {
			// transmit length mismatch error interrupt
			dev_err(device_wrapper->os_device,
				"transmit length mismatch error interrupt\n");

			// clear interrupt
			iowrite32(XLLF_INT_TSE_MASK & XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		} else if (pending_interrupts) {
			// unknown interrupt type
			dev_err(device_wrapper->os_device,
				"unknown interrupt(s) 0x%x\n",
				pending_interrupts);
			
			// clear interrupt
			iowrite32(XLLF_INT_ALL_MASK,
				device_wrapper->base_addr + XLLF_ISR_OFFSET);
		}
	} while (pending_interrupts);

	return IRQ_HANDLED;
}

// called by the kernel when someone is trying to open the device
static int axis_fifo_open(struct inode *inod, struct file *device_file)
{
	struct axis_fifo_local *device_wrapper = (struct axis_fifo_local *)container_of(inod->i_cdev,
							struct axis_fifo_local, char_device);
	// set file attribute to our device wrapper so other
	// functions (e.g. read/write) can access it
	device_file->private_data = device_wrapper;

	dev_dbg(device_wrapper->os_device, "opening...\n");

	// lock write access
	if (((device_file->f_flags & O_ACCMODE) == O_WRONLY)) {

		if (!device_wrapper->has_tx_fifo) {
			dev_err(device_wrapper->os_device,
				"tried to open device for write but the transmit fifo is disabled\n");
			return -EPERM;
		}

		if (!mutex_trylock(&device_wrapper->write_mutex)) {
			dev_err(device_wrapper->os_device, "couldn't acquire write lock\n");
			return -EBUSY;
		}
		device_wrapper->write_flags = device_file->f_flags;

		dev_dbg(device_wrapper->os_device, "acquired write lock\n");
	}

	// lock read access
	if ((device_file->f_flags & O_ACCMODE) == O_RDONLY) {

		if (!device_wrapper->has_rx_fifo) {
			dev_err(device_wrapper->os_device,
				"tried to open device for read but the receive fifo is disabled\n");
			return -EPERM;
		}

		if (!mutex_trylock(&device_wrapper->read_mutex)) {
			dev_err(device_wrapper->os_device,
				"couldn't acquire read lock\n");
			return -EBUSY;
		}
		device_wrapper->read_flags = device_file->f_flags;

		dev_dbg(device_wrapper->os_device, "acquired read lock\n");
	}

	// lock read +  write access
	if ((device_file->f_flags & O_ACCMODE) == O_RDWR) {

		if (!device_wrapper->has_rx_fifo ||
			!device_wrapper->has_tx_fifo) {
			dev_err(device_wrapper->os_device,
				"tried to open device for read/write but one or both of the receive/transmit fifos are disabled\n");
			return -EPERM;
		}

		if (!mutex_trylock(&device_wrapper->write_mutex)) {
			dev_err(device_wrapper->os_device,
				"couldn't acquire write lock\n");
			return -EBUSY;
		}
		if (!mutex_trylock(&device_wrapper->read_mutex)) {
			dev_err(device_wrapper->os_device,
				"couldn't acquire read lock\n");
			mutex_unlock(&device_wrapper->write_mutex);
			dev_dbg(device_wrapper->os_device,
				"released write lock\n");
			return -EBUSY;
		}
		device_wrapper->write_flags = device_file->f_flags;
		device_wrapper->read_flags = device_file->f_flags;
		
		dev_dbg(device_wrapper->os_device, "acquired write lock\n");
		dev_dbg(device_wrapper->os_device, "acquired read lock\n");
	}

	dev_dbg(device_wrapper->os_device, "opened\n");

	return 0;
}

// called by the kernel when someone is trying to close the device
static int axis_fifo_close(struct inode *inod, struct file *device_file)
{
	struct axis_fifo_local *device_wrapper = container_of(inod->i_cdev,
							struct axis_fifo_local,
							char_device);
	// unset file attribute
	device_file->private_data = NULL;

	dev_dbg(device_wrapper->os_device, "closing...\n");

	// unlock write access
	if (((device_file->f_flags & O_ACCMODE) == O_WRONLY) ||
		((device_file->f_flags & O_ACCMODE) == O_RDWR)) {
		mutex_unlock(&device_wrapper->write_mutex);
		dev_dbg(device_wrapper->os_device, "released write lock\n");
	}

	// unlock read access
	if (((device_file->f_flags & O_ACCMODE) == O_RDONLY) ||
		((device_file->f_flags & O_ACCMODE) == O_RDWR)) {
		mutex_unlock(&device_wrapper->read_mutex);
		dev_dbg(device_wrapper->os_device, "released read lock\n");
	}

	dev_dbg(device_wrapper->os_device, "closed\n");

	return 0;
}

// functions for reading/writing directly to registers via sysfs
static ssize_t sysfs_write(struct device *dev, const char *buf,
				size_t count, unsigned addr_offset)
{
	struct axis_fifo_local *device_wrapper = dev_get_drvdata(dev);
	if (!mutex_trylock(&device_wrapper->write_mutex)) {	
		dev_err(device_wrapper->os_device,
			"couldn't acquire write lock\n");
		return -EBUSY;
	}
	if (!mutex_trylock(&device_wrapper->read_mutex)) {
		dev_err(device_wrapper->os_device,
			"couldn't acquire read lock\n");
		mutex_unlock(&device_wrapper->write_mutex);
		dev_dbg(device_wrapper->os_device, "released write lock\n");
		return -EBUSY;
	}
	dev_dbg(device_wrapper->os_device, "acquired locks\n");
	if (count != 4) {
		dev_err(device_wrapper->os_device,
			"error, sysfs write to address 0x%x expected 4 bytes\n",
			addr_offset);
		mutex_unlock(&device_wrapper->write_mutex);
		mutex_unlock(&device_wrapper->read_mutex);
		return -EINVAL;
	} 
	dev_dbg(device_wrapper->os_device,
		"writing 0x%x to sysfs address 0x%x\n",
		*(unsigned int *)buf, addr_offset);
	iowrite32(*(unsigned int __force *)buf,
			device_wrapper->base_addr + addr_offset);
	mutex_unlock(&device_wrapper->write_mutex);
	mutex_unlock(&device_wrapper->read_mutex);
	dev_dbg(device_wrapper->os_device, "released locks\n");
	return 4;
}

static ssize_t sysfs_read(struct device *dev, char *buf, unsigned addr_offset)
{
	struct axis_fifo_local *device_wrapper = dev_get_drvdata(dev);
	unsigned int read_val;
	if (!mutex_trylock(&device_wrapper->write_mutex)) {
		dev_err(device_wrapper->os_device,
			"couldn't acquire write lock\n");
		return -EBUSY;
	}
	if (!mutex_trylock(&device_wrapper->read_mutex)) {
		dev_err(device_wrapper->os_device,
			"couldn't acquire read lock\n");
		mutex_unlock(&device_wrapper->write_mutex);
		dev_dbg(device_wrapper->os_device, "released write lock\n");
		return -EBUSY;
	}
	dev_dbg(device_wrapper->os_device, "acquired locks\n");
	read_val = ioread32(device_wrapper->base_addr + addr_offset);
	dev_dbg(device_wrapper->os_device,
		"read 0x%x from sysfs address 0x%x\n",
		read_val, addr_offset);
	*(unsigned int __force *)buf = read_val;
	mutex_unlock(&device_wrapper->write_mutex);
	mutex_unlock(&device_wrapper->read_mutex);
	dev_dbg(device_wrapper->os_device, "released locks\n");
	return 4;
}

static ssize_t sysfs_write_isr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return sysfs_write(dev, buf, count, XLLF_ISR_OFFSET);
}

static ssize_t sysfs_read_isr(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sysfs_read(dev, buf, XLLF_ISR_OFFSET);
}

static ssize_t sysfs_write_ier(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) 
{
	return sysfs_write(dev, buf, count, XLLF_IER_OFFSET);
}

static ssize_t sysfs_read_ier(struct device *dev,
				struct device_attribute *attr, char *buf)						 
{
	return sysfs_read(dev, buf, XLLF_IER_OFFSET);
}

static ssize_t sysfs_write_tdfr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return sysfs_write(dev, buf, count, XLLF_TDFR_OFFSET);
}
static ssize_t sysfs_read_tdfv(struct device *dev,
				struct device_attribute *attr, char *buf)					 
{
	return sysfs_read(dev, buf, XLLF_TDFV_OFFSET);
}
static ssize_t sysfs_write_tdfd(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return sysfs_write(dev, buf, count, XLLF_TDFD_OFFSET);
}
static ssize_t sysfs_write_tlr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) 
{
	return sysfs_write(dev, buf, count, XLLF_TLR_OFFSET);
}

static ssize_t sysfs_write_rdfr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return sysfs_write(dev, buf, count, XLLF_RDFR_OFFSET);
}
static ssize_t sysfs_read_rdfo(struct device *dev,
				struct device_attribute *attr, char *buf)					 
{
	return sysfs_read(dev, buf, XLLF_RDFO_OFFSET);
}
static ssize_t sysfs_read_rdfd(struct device *dev,
				struct device_attribute *attr, char *buf)					 
{
	return sysfs_read(dev, buf, XLLF_RDFD_OFFSET);
}
static ssize_t sysfs_read_rlr(struct device *dev,
				struct device_attribute *attr, char *buf)						 
{
	return sysfs_read(dev, buf, XLLF_RLR_OFFSET);
}
static ssize_t sysfs_write_srr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) 
{
	return sysfs_write(dev, buf, count, XLLF_SRR_OFFSET);
}
static ssize_t sysfs_write_tdr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) 
{
	return sysfs_write(dev, buf, count, XLLF_TDR_OFFSET);
}
static ssize_t sysfs_read_rdr(struct device *dev,
				struct device_attribute *attr, char *buf)						 
{
	return sysfs_read(dev, buf, XLLF_RDR_OFFSET);
}

static int get_dts_property(struct axis_fifo_local *device_wrapper,
				char *name, unsigned int *var)
{
	if (of_property_read_u32(device_wrapper->os_device->of_node,
				name, var) < 0) {
		dev_err(device_wrapper->os_device,
			"couldn't read IP dts property '%s'", name);
		return -1;
	} else {
		dev_dbg(device_wrapper->os_device,
			"dts property '%s' = %u\n", name, *var);
	}

	return 0;
}

// called by the kernel while creating
// each device (as defined in the device tree)
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
	device_wrapper = (struct axis_fifo_local *) devm_kzalloc(dev,
			sizeof(struct axis_fifo_local), GFP_KERNEL);
	if (!device_wrapper) {
		printk(KERN_ERR "couldn't allocate memory for axis-fifo device\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, device_wrapper);
	device_wrapper->os_device = dev;

	// get unique id
	mutex_lock(&num_fifo_devices_mutex);
	device_wrapper->id = num_fifo_devices;
	num_fifo_devices++;
	mutex_unlock(&num_fifo_devices_mutex);

	dev_dbg(device_wrapper->os_device, "acquired device number %i\n",
			device_wrapper->id);

	mutex_init(&device_wrapper->read_mutex);
	mutex_init(&device_wrapper->write_mutex);

	dev_dbg(device_wrapper->os_device, "initialized mutexes\n");

	init_waitqueue_head(&device_wrapper->read_queue);
	init_waitqueue_head(&device_wrapper->write_queue);

	dev_dbg(device_wrapper->os_device, "initialized queues\n");

	spin_lock_init(&device_wrapper->read_queue_lock);
	spin_lock_init(&device_wrapper->write_queue_lock);

	dev_dbg(device_wrapper->os_device, "initialized spinlocks\n");

	// create unique device and class names
	snprintf(device_name, 32, DRIVER_NAME "%i", device_wrapper->id);
	snprintf(class_name, 32, DRIVER_NAME "%i_class", device_wrapper->id);

	dev_dbg(device_wrapper->os_device, "device name [%s] class name [%s]\n",
			device_name, class_name);

	// ----------------------------
	//   init device memory space
	// ----------------------------

	// get iospace for the device
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(device_wrapper->os_device, "invalid address\n");
		rc = -ENODEV;
		goto err_initial;
	}

	device_wrapper->mem_start = r_mem->start;
	device_wrapper->mem_end = r_mem->end;

	// request physical memory
	if (!request_mem_region(device_wrapper->mem_start,
			device_wrapper->mem_end - device_wrapper->mem_start + 1,
			DRIVER_NAME)) {
		dev_err(device_wrapper->os_device,
				"couldn't lock memory region at %p\n",
				(void *)device_wrapper->mem_start);
		rc = -EBUSY;
		goto err_initial;
	}
	dev_dbg(device_wrapper->os_device,
		"got memory location [0x%x - 0x%x]\n",
		device_wrapper->mem_start, device_wrapper->mem_end);

	// map physical memory to kernel virtual address space
	device_wrapper->base_addr = ioremap(device_wrapper->mem_start,
		device_wrapper->mem_end - device_wrapper->mem_start + 1);

	if (!device_wrapper->base_addr) {
		dev_err(device_wrapper->os_device,
			"couldn't map physical memory\n");
		rc = -EIO;
		goto err_mem;
	}
	dev_dbg(device_wrapper->os_device, "remapped memory to 0x%x\n",
			(unsigned int)device_wrapper->base_addr);

	// ----------------------------
	//          init IP
	// ----------------------------

	if (get_dts_property(device_wrapper, "xlnx,axi-str-rxd-tdata-width",
				&rxd_tdata_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,axi-str-txc-tdata-width",
				&txc_tdata_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,axi-str-txd-tdata-width",
				&txd_tdata_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,axis-tdest-width",
				&tdest_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,axis-tid-width",
				&tid_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,axis-tuser-width",
				&tuser_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,data-interface-type",
				&data_interface_type)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,has-axis-tdest",
				&has_tdest)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,has-axis-tid",
				&has_tid)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,has-axis-tkeep",
				&has_tkeep)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,has-axis-tstrb",
				&has_tstrb)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,has-axis-tuser",
				&has_tuser)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,rx-fifo-depth",
				&rx_fifo_depth)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,rx-fifo-pe-threshold",
				&rx_programmable_empty_threshold)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,rx-fifo-pf-threshold",
				&rx_programmable_full_threshold)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,s-axi-id-width",
				&axi_id_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,s-axi4-data-width",
				&axi4_data_width)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,select-xpm",
				&select_xpm)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,tx-fifo-depth",
				&tx_fifo_depth)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,tx-fifo-pe-threshold",
				&tx_programmable_empty_threshold)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,tx-fifo-pf-threshold",
				&tx_programmable_full_threshold)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,use-rx-cut-through",
				&use_rx_cut_through)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,use-rx-data",
				&use_rx_data)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,use-tx-ctrl",
				&use_tx_control)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,use-tx-cut-through",
				&use_tx_cut_through)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	if (get_dts_property(device_wrapper, "xlnx,use-tx-data",
				&use_tx_data)) {
		rc = -ENOSYS;
		goto err_mem;
	}
	

	if (rxd_tdata_width != 32) {
		dev_err(device_wrapper->os_device,
			"rxd_tdata_width width [%u] unsupported\n",
			rxd_tdata_width);
		rc = -ENOSYS;
		goto err_mem;
	}
	if (txd_tdata_width != 32) {
		dev_err(device_wrapper->os_device,
			"txd_tdata_width width [%u] unsupported\n",
			txd_tdata_width);
		rc = -ENOSYS;
		goto err_mem;
	}
	if (has_tdest) {
		dev_err(device_wrapper->os_device, "tdest not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (has_tid) {
		dev_err(device_wrapper->os_device, "tid not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (has_tkeep) {
		dev_err(device_wrapper->os_device, "tkeep not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (has_tstrb) {
		dev_err(device_wrapper->os_device, "tstrb not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (has_tuser) {
		dev_err(device_wrapper->os_device, "tuser not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (use_rx_cut_through) {
		dev_err(device_wrapper->os_device,
			"rx cut-through not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (use_tx_cut_through) {
		dev_err(device_wrapper->os_device,
			"tx cut-through not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	if (use_tx_control) {
		dev_err(device_wrapper->os_device,
			"tx control not supported\n");
		rc = -ENOSYS;
		goto err_mem;
	}
	
	// TODO
	// these exist in the device tree but it's unclear what they do
	// - select-xpm
	// - data-interface-type

	// set device wrapper properties based on IP config
	device_wrapper->rx_fifo_depth = rx_fifo_depth;
	// IP sets TDFV to fifo depth - 4
	device_wrapper->tx_fifo_depth = tx_fifo_depth - 4;
	device_wrapper->has_rx_fifo = use_rx_data;
	device_wrapper->has_tx_fifo = use_tx_data;

	reset_ip_core(device_wrapper);

	// ----------------------------
	//    init device interrupts
	// ----------------------------

	// get IRQ resource
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r_irq) {
		dev_err(device_wrapper->os_device,
			"no IRQ found at 0x%08x mapped to 0x%08x\n",
			(unsigned int __force)device_wrapper->mem_start,
			(unsigned int __force)device_wrapper->base_addr);
		rc = -EIO;
		goto err_mem;
	}
	dev_dbg(device_wrapper->os_device, "found IRQ\n");

	// request IRQ
	device_wrapper->irq = r_irq->start;
	rc = request_irq(device_wrapper->irq, &axis_fifo_irq, 0,
			DRIVER_NAME, device_wrapper);
	if (rc) {
		dev_err(device_wrapper->os_device,
			"couldn't allocate interrupt %i\n",
			device_wrapper->irq);
		rc = -EIO;
		goto err_mem;
	}
	dev_dbg(device_wrapper->os_device,
		"initialized IRQ %i\n",
		device_wrapper->irq);

	// ----------------------------
	//      init char device
	// ----------------------------

	// allocate device number
	if (alloc_chrdev_region(&device_wrapper->devt, 0, 1, DRIVER_NAME) < 0) {
		dev_err(device_wrapper->os_device, "couldn't allocate dev_t\n");
		rc = -EIO;
		goto err_irq;
	}
	dev_dbg(device_wrapper->os_device,
		"allocated device number major %i minor %i\n",
		MAJOR(device_wrapper->devt), MINOR(device_wrapper->devt));

	// create driver class
	device_wrapper->driver_class = NULL;
	device_wrapper->driver_class = class_create(THIS_MODULE, class_name);
	if (device_wrapper->driver_class == NULL) {
		dev_err(device_wrapper->os_device,
			"couldn't create driver class\n");
		rc = -EIO;
		goto err_chrdev_region;
	}
	dev_dbg(device_wrapper->os_device, "created driver class\n");

	// create driver file
	device_wrapper->device = NULL;
	device_wrapper->device = device_create(device_wrapper->driver_class,
						NULL, device_wrapper->devt,
						NULL, device_name);
	if (device_wrapper->device == NULL) {
		dev_err(device_wrapper->os_device,
			"couldn't create driver file\n");
		rc = -EIO;
		goto err_class;
	}
	dev_set_drvdata(device_wrapper->device, device_wrapper);
	dev_dbg(device_wrapper->os_device, "created device file\n");

	// create character device
	cdev_init(&device_wrapper->char_device, &fops);
	if (cdev_add(&device_wrapper->char_device,
			device_wrapper->devt, 1) < 0) {
		dev_err(device_wrapper->os_device,
				"couldn't create character device\n");
		rc = -EIO;
		goto err_dev;
	}
	dev_dbg(device_wrapper->os_device, "created character device\n");

	// ----------------------------
	//  add sysfs register entries
	// ----------------------------

	if (device_create_file(device_wrapper->device, &dev_attr_isr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute isr\n");
		goto err_cdev;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_ier)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute ier\n");
		goto err_isr;
	}
	
	if (device_create_file(device_wrapper->device, &dev_attr_tdfr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute tdfr\n");
		goto err_ier;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tdfv)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute tdfv\n");
		goto err_tdfr;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tdfd)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute tdfd\n");
		goto err_tdfv;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tlr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute tlr\n");
		goto err_tdfd;
	}
	
	if (device_create_file(device_wrapper->device, &dev_attr_rdfr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute rdfr\n");
		goto err_tlr;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rdfo)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute rdfo\n");
		goto err_rdfr;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rdfd)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute rdfd\n");
		goto err_rdfo;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rlr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute rlr\n");
		goto err_rdfd;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_srr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute srr\n");
		goto err_rlr;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_tdr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute tdr\n");
		goto err_srr;
	}
	if (device_create_file(device_wrapper->device, &dev_attr_rdr)) {
		dev_err(device_wrapper->os_device,
			"couldn't create sysfs attribute rdr\n");
		goto err_tdr;
	}
	
	dev_info(device_wrapper->os_device,
		"axis-fifo created at 0x%08x mapped " \
		"to 0x%08x, irq=%i, major=%i, minor=%i\n",
		(unsigned int __force)device_wrapper->mem_start,
		(unsigned int __force)device_wrapper->base_addr,
		device_wrapper->irq, MAJOR(device_wrapper->devt),
		MINOR(device_wrapper->devt));
	
	return 0;

err_tdr:
	device_remove_file(device_wrapper->device, &dev_attr_tdr);
err_srr:
	device_remove_file(device_wrapper->device, &dev_attr_srr);
err_rlr:
	device_remove_file(device_wrapper->device, &dev_attr_rlr);
err_rdfd:
	device_remove_file(device_wrapper->device, &dev_attr_rdfd);
err_rdfo:
	device_remove_file(device_wrapper->device, &dev_attr_rdfo);
err_rdfr:
	device_remove_file(device_wrapper->device, &dev_attr_rdfr);
err_tlr:
	device_remove_file(device_wrapper->device, &dev_attr_tlr);
err_tdfd:
	device_remove_file(device_wrapper->device, &dev_attr_tdfd);
err_tdfv:
	device_remove_file(device_wrapper->device, &dev_attr_tdfv);
err_tdfr:
	device_remove_file(device_wrapper->device, &dev_attr_tdfr);
err_ier:
	device_remove_file(device_wrapper->device, &dev_attr_ier);
err_isr:
	device_remove_file(device_wrapper->device, &dev_attr_isr);
err_cdev:
	cdev_del(&device_wrapper->char_device);
err_dev:
	dev_set_drvdata(device_wrapper->device, NULL);
	device_destroy(device_wrapper->driver_class, device_wrapper->devt);
err_class:
	class_destroy(device_wrapper->driver_class);
err_chrdev_region:
	unregister_chrdev_region(device_wrapper->devt, 1);
err_irq:
	free_irq(device_wrapper->irq, device_wrapper);
err_mem:
	release_mem_region(device_wrapper->mem_start,
		device_wrapper->mem_end - device_wrapper->mem_start + 1);
err_initial:
	mutex_destroy(&device_wrapper->read_mutex);
	mutex_destroy(&device_wrapper->write_mutex);
	dev_set_drvdata(dev, NULL);
	return rc;
}

// called by the kernel while removing each device
// that is defined in the device tree
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
	release_mem_region(device_wrapper->mem_start,
		device_wrapper->mem_end - device_wrapper->mem_start + 1);
	mutex_destroy(&device_wrapper->read_mutex);
	mutex_destroy(&device_wrapper->write_mutex);
	dev_set_drvdata(dev, NULL);
	return 0;
}

// called by the kernel when this module is loaded
static int __init axis_fifo_init(void)
{
	printk(KERN_INFO "axis-fifo driver loaded with parameters read_timeout = " \
		"%i, write_timeout = %i\n", read_timeout, write_timeout);
	mutex_init(&num_fifo_devices_mutex);
	num_fifo_devices = 0;
	return platform_driver_register(&axis_fifo_driver);
}

// called by the kernel when this module is unloaded
static void __exit axis_fifo_exit(void)
{
	platform_driver_unregister(&axis_fifo_driver);
	mutex_destroy(&num_fifo_devices_mutex);
	printk(KERN_INFO "axis-fifo driver exit\n");
}