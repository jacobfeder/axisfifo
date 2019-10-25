#ifndef AXIS_FIFO_H
#define AXIS_FIFO_H

#include <linux/ioctl.h>

/* ----------------------------
 *     IP register offsets
 * ----------------------------
 */

#define XLLF_ISR_OFFSET  0x00000000  /* Interrupt Status */
#define XLLF_IER_OFFSET  0x00000004  /* Interrupt Enable */

#define XLLF_TDFR_OFFSET 0x00000008  /* Transmit Reset */
#define XLLF_TDFV_OFFSET 0x0000000c  /* Transmit Vacancy */
#define XLLF_TDFD_OFFSET 0x00000010  /* Transmit Data */
#define XLLF_TLR_OFFSET  0x00000014  /* Transmit Length */

#define XLLF_RDFR_OFFSET 0x00000018  /* Receive Reset */
#define XLLF_RDFO_OFFSET 0x0000001c  /* Receive Occupancy */
#define XLLF_RDFD_OFFSET 0x00000020  /* Receive Data */
#define XLLF_RLR_OFFSET  0x00000024  /* Receive Length */
#define XLLF_SRR_OFFSET  0x00000028  /* Local Link Reset */
#define XLLF_TDR_OFFSET  0x0000002C  /* Transmit Destination */
#define XLLF_RDR_OFFSET  0x00000030  /* Receive Destination */


/* ----------------------------
 *     reset register masks
 * ----------------------------
 */

#define XLLF_RDFR_RESET_MASK        0x000000a5 /* receive reset value */
#define XLLF_TDFR_RESET_MASK        0x000000a5 /* Transmit reset value */
#define XLLF_SRR_RESET_MASK         0x000000a5 /* Local Link reset value */

/* ----------------------------
 *       interrupt masks
 * ----------------------------
 */

#define XLLF_INT_RPURE_MASK       0x80000000 /* Receive under-read */
#define XLLF_INT_RPORE_MASK       0x40000000 /* Receive over-read */
#define XLLF_INT_RPUE_MASK        0x20000000 /* Receive underrun (empty) */
#define XLLF_INT_TPOE_MASK        0x10000000 /* Transmit overrun */
#define XLLF_INT_TC_MASK          0x08000000 /* Transmit complete */
#define XLLF_INT_RC_MASK          0x04000000 /* Receive complete */
#define XLLF_INT_TSE_MASK         0x02000000 /* Transmit length mismatch */
#define XLLF_INT_TRC_MASK         0x01000000 /* Transmit reset complete */
#define XLLF_INT_RRC_MASK         0x00800000 /* Receive reset complete */
#define XLLF_INT_TFPF_MASK        0x00400000 /* Tx FIFO Programmable Full */
#define XLLF_INT_TFPE_MASK        0x00200000 /* Tx FIFO Programmable Empty */
#define XLLF_INT_RFPF_MASK        0x00100000 /* Rx FIFO Programmable Full */
#define XLLF_INT_RFPE_MASK        0x00080000 /* Rx FIFO Programmable Empty */
#define XLLF_INT_ALL_MASK         0xfff80000 /* All the ints */
#define XLLF_INT_ERROR_MASK       0xf2000000 /* Error status ints */
#define XLLF_INT_RXERROR_MASK     0xe0000000 /* Receive Error status ints */
#define XLLF_INT_TXERROR_MASK     0x12000000 /* Transmit Error status ints */

/* ----------------------------
 *      ioctls 
 * ----------------------------
 */

#define AXIS_FIFO_IOCTL_MAGIC 'Q'
#define AXIS_FIFO_NUM_IOCTLS 8


struct axis_fifo_kern_regInfo{
        uint32_t regNo;
        uint32_t regVal;
        };

#define AXIS_FIFO_GET_REG         _IOR(AXIS_FIFO_IOCTL_MAGIC, 0, struct axis_fifo_kern_regInfo)
#define AXIS_FIFO_SET_REG         _IOW(AXIS_FIFO_IOCTL_MAGIC, 1, struct axis_fifo_kern_regInfo)
#define AXIS_FIFO_GET_TX_MAX_PKT  _IOR(AXIS_FIFO_IOCTL_MAGIC, 2, uint32_t)
#define AXIS_FIFO_SET_TX_MAX_PKT  _IOW(AXIS_FIFO_IOCTL_MAGIC, 3,  uint32_t)
#define AXIS_FIFO_GET_RX_MIN_PKT  _IOR(AXIS_FIFO_IOCTL_MAGIC, 4,  uint32_t)
#define AXIS_FIFO_SET_RX_MIN_PKT  _IOW(AXIS_FIFO_IOCTL_MAGIC, 5,  uint32_t)
#define AXIS_FIFO_RESET_IP        _IO(AXIS_FIFO_IOCTL_MAGIC,6)
#define AXIS_FIFO_GET_FPGA_ADDR   _IOR(AXIS_FIFO_IOCTL_MAGIC,7, uint32_t)

#endif /* AXIS_FIFO_H */
