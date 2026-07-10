// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * USB CAN bus driver for USB to CAN controller chip CH9433.
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 */

#undef DEBUG
#undef VERBOSE

#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/ethtool.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/usb.h>
#include <linux/bits.h>
#include <linux/workqueue.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#define DRIVER_AUTHOR "WCH"
#define DRVNAME_CH9433 "ch9433"
#define DRIVER_DESC "USB CAN bus driver for CH9433, etc."
#define VERSION_DESC "V1.0 On 2026.06"

#define ENABLE_DRIVER_DEBUG 0

#if ENABLE_DRIVER_DEBUG
#define DRV_DEBUG(dev, format, ...) dev_dbg(dev, format, ##__VA_ARGS__)
#else
#define DRV_DEBUG(dev, format, ...)                          \
	do {                                                 \
		if (0)                                       \
			dev_dbg(dev, format, ##__VA_ARGS__); \
	} while (0)
#endif

#define CH9433_MAX_NUM 16

static struct ch9433 *ch9433_table[CH9433_MAX_NUM];
static struct cdev ch9433_cdev;
static struct class *ch9433_io_class;
static dev_t devt;
static int ch9433_major = 0x00;
#define CH9433_GPIODRV_NAME "ch9433_io"
#define IOCTL_MAGIC 'W'
#define IOCTL_CTRL_WRITE_REG _IOWR(IOCTL_MAGIC, 0x82, struct ch9433_reg_req)
#define IOCTL_CTRL_READ_REG _IOWR(IOCTL_MAGIC, 0x83, struct ch9433_reg_req)
#define IOCTL_CTRL_XFER _IOWR(IOCTL_MAGIC, 0x84, struct ch9433_raw_xfer_req)

static DEFINE_MUTEX(ch9433_minors_lock);

#define CH9433_TX_AGG_NUM 15
#define CH9433_RX_AGG_NUM 20
#define CH9433_USB_CTRL_GET_TIMEOUT 500
#define CH9433_USB_CTRL_SET_TIMEOUT 500
#define CH9433_CLK_FREQ (96 * 1000 * 1000)
#define CH9433_BULK_IN_PIPE 2
#define CH9433_BULK_OUT_PIPE 2
#define CH9433_CTRL_PIPE 3

/* control requests */
#define CH9433_CMD_BULK_IN 0xA1
#define CH9433_CMD_STAT_IN 0xA2
#define CH9433_CMD_CTRL_IN 0xA5
#define CH9433_CMD_BULK_OUT 0x5A
#define CH9433_IO_SEL_FUN_CFG 0x45

#define CH9433_MAX_FRAME 1024
/* Raw ioctl transfers may carry a full 1024-byte data area plus USB framing. */
#define CH9433_IOCTL_MAX_FRAME 2048

#define CH9433_REG_OP_WRITE 0x80
#define CH9433_REG_OP_READ 0x00

#define CH9433_CANREG_CMD 0x46
#define CH9433_CMD_HEAD_LEN 3
#define CH9433_CMD_CKSUM_LEN 2
#define CH9433_USB_BULK_RX_CAN_MSG_MAX 3
#define CH9433_CMD_OUT_LEN (sizeof(struct ch9433_usb_ctrlmsg_out))
#define CH9433_CMD_IN_LEN (sizeof(struct ch9433_usb_ctrlmsg_in))
#define CH9433_BULK_OUT_LEN (sizeof(struct ch9433_usb_bulkmsg_out))
#define CH9433_BULK_IN_LEN (sizeof(struct ch9433_usb_bulkmsg_in))

#define CH9433_DELAY_MS (5)

#define CH9433_CAN_CTLR 0x00
#define CAN_CTLR_INRQ BIT(0)
#define CAN_CTLR_SLEEP BIT(1)
#define CAN_CTLR_TXFP BIT(2)
#define CAN_CTLR_RFLM BIT(3)
#define CAN_CTLR_NART BIT(4)
#define CAN_CTLR_AWUM BIT(5)
#define CAN_CTLR_ABOM BIT(6)
#define CAN_CTLR_TTCM BIT(7)
#define CAN_CTLR_RESET BIT(15)

#define CH9433_CAN_STATR 0x01
#define CAN_STATR_INAK BIT(0)
#define CAN_STATR_SLAK BIT(1)
#define CAN_STATR_ERRI BIT(2)
#define CAN_STATR_WKUI BIT(3)
#define CAN_STATR_SLAKI BIT(4)

#define CH9433_CAN_INTENR 0x05
#define CAN_INTENR_FMPIE0 BIT(1)
#define CAN_INTENR_FFIE0 BIT(2)
#define CAN_INTENR_FOVIE0 BIT(3)
#define CAN_INTENR_FMPIE1 BIT(4)
#define CAN_INTENR_FFIE1 BIT(5)
#define CAN_INTENR_FOVIE1 BIT(6)
#define CAN_INTENR_EWGIE BIT(8)
#define CAN_INTENR_EPVIE BIT(9)
#define CAN_INTENR_BOFIE BIT(10)
#define CAN_INTENR_LECIE BIT(11)
#define CAN_INTENR_ERRIE BIT(15)
#define CAN_INTENR_WKUIE BIT(16)
#define CAN_INTENR_SLKIE BIT(17)

#define CH9433_CAN_ERRSR 0x06
#define CAN_ERRSR_EWGF BIT(0)
#define CAN_ERRSR_EPVF BIT(1)
#define CAN_ERRSR_BOFF BIT(2)
#define CAN_ERRSR_LEC_MASK ((u32)0x00000070)
#define CAN_ERRSR_TEC ((u32)0x00FF0000)
#define CAN_ERRSR_REC ((u32)0xFF000000)

#define CH9433_CAN_BTIMR 0x07
#define CAN_BTIMR_LBKM BIT(30)
#define CAN_BTIMR_SILM BIT(31)

#define CH9433_CAN_TX0WRITE_CONT 0x42

#define CH9433_CAN_FCTLR 0x1F
#define CAN_FCTLR_FINIT ((uint8_t)BIT(0))

#define CH9433_CAN_FMCFGR 0x20
#define CH9433_CAN_FSCFGR 0x21
#define CH9433_CAN_FAFIFOR 0x22
#define CH9433_CAN_FWR 0x23
#define CH9433_CAN_FxR1(x) (0x24 + x * 2)
#define CH9433_CAN_FxR2(x) (0x25 + x * 2)

#define CAN_TXMIRx_TXRQ BIT(0)
#define CAN_TXMIRx_IDE BIT(2)

static const struct can_bittiming_const ch9433_bittiming_const = {
	.name = DRVNAME_CH9433,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

/* ch9433 flags */
enum ch9433_flags {
	CH9433_RX_OVERFLOW = 0,
	CH9433_TX_ERR,
	CH9433_DEV_UNPLUG,
	CH9433_RX_HALT,
	CH9433_TX_HALT,
	CH9433_RECOVERING,
	CH9433_STOPPING,
};

struct ch9433_rx_agg {
	struct list_head list, info_list;
	struct urb *urb;
	struct ch9433 *context;
	void *buffer;
	dma_addr_t dma;
};

struct ch9433_tx_agg {
	struct list_head list, info_list;
	struct urb *urb;
	struct ch9433 *context;
	void *buffer;
	void *head;
	dma_addr_t dma;
	u32 index;
};

struct ch9433_usb_ctrlmsg_in {
	u8 header;
	__be16 len;
	__le32 data;
	__be16 checksum;
} __packed;

struct ch9433_usb_ctrlmsg_out {
	u8 header;
	__be16 len;
	u8 cmd;
	u8 reg;
	__be16 checksum;
} __packed;

struct ch9433_usb_regmsg_out {
	u8 header;
	__be16 len;
	u8 cmd;
	u8 reg;
	__le32 data;
	__be16 checksum;
} __packed;

struct ch9433_can_rx_msg {
	__le32 data_h;
	__le32 data_l;
	__le32 mdtr;
#define MDTR_DLC_MASK 0xfU
	__le32 mir;
#define MIR_IDE BIT(2)
#define MIR_RTR BIT(1)
#define MIR_STD_ID_SHIFT 21
#define MIR_STD_ID_MASK (0x7ffU << MIR_STD_ID_SHIFT)
#define MIR_EXT_ID_SHIFT 3
#define MIR_EXT_ID_MASK (0x1fffffffU << MIR_EXT_ID_SHIFT)
} __packed;

struct ch9433_can_tx_msg {
	__le32 data_h;
	__le32 data_l;
	__le32 len;
	__le32 id;
} __packed;

struct ch9433_usb_bulkmsg_in {
	u8 header;
	__be16 len;
	u8 box_id;
	u8 box_num; /* Number of valid CAN frames in this USB packet */
	struct ch9433_can_rx_msg can_msg[CH9433_USB_BULK_RX_CAN_MSG_MAX];
	__be16 checksum;
} __packed;

struct ch9433_usb_bulkmsg_out {
	u8 header;
	__be16 len;
	u8 cmd_h;
	u8 cmd_l;
	struct ch9433_can_tx_msg can_msg;
	__be16 checksum;
} __packed;

struct ch9433_usb_statsmsg {
	u8 header;
	__be16 len;
	u8 reserved[3];
	u8 flags;
#define TX_ERR_OR_WAKE BIT(7)
#define RX_MBOX1_OVERFLOW BIT(6)
#define RX_MBOX1_FULL BIT(5)
#define RX_MBOX0_OVERFLOW BIT(4)
#define RX_MBOX0_FULL BIT(3)
#define TX_MBOX2_FINISH BIT(2)
#define TX_MBOX1_FINISH BIT(1)
#define TX_MBOX0_FINISH BIT(0)
	__be16 checksum;
} __packed;

struct ch9433_raw_xfer_req {
	__u32 tx_len;
	__u32 rx_len;
	__u32 rx_actual_len;
	__u32 reserved;
	__aligned_u64 tx_buf;
	__aligned_u64 rx_buf;
};

struct ch9433_reg_req {
	__u8 reg;
	__aligned_u64 reg_val;
};

struct ch9433 {
	struct can_priv can;
	struct usb_device *udev;
	struct net_device *ndev;
	struct usb_interface *intf;
	struct work_struct rx_status_work;
	struct work_struct recover_work;
	struct list_head txq_free, txq_info, rxq_info;
	spinlock_t txq_lock, rxq_lock;
	struct mutex ctrl_lock;
	struct mutex state_lock;
	atomic_t refcnt;
	size_t n_tx_urbs, n_rx_urbs;
	size_t tx_urb_size, rx_urb_size;
	atomic_t rx_count;
	int minor;
	unsigned long flags;
	unsigned int pipe_in, pipe_out, pipe_ctrl_in, pipe_ctrl_out;
};

static u16 ch9433_calc_checksum(const void *buf, int len)
{
	const u8 *p = buf;
	u16 sum = 0;
	int i;

	for (i = 0; i < len - CH9433_CMD_CKSUM_LEN; i++) {
		sum += p[i];
	}

	return sum;
}

static int ch9433_rx_fixed_chk(struct ch9433 *dev, const char *tag,
			       const void *msg, int len, u8 type, int size)
{
	const u8 *raw = msg;
	u16 sum;
	u16 sum_rx;
	u16 pay_len;

	if (len < size) {
		dev_err(&dev->udev->dev,
			"%s packet truncated: exp %d, got %d\n", tag, size,
			len);
		return -EIO;
	}

	if (raw[0] != type) {
		dev_err(&dev->udev->dev, "%s invalid header: %02x\n", tag,
			raw[0]);
		return -EPROTO;
	}

	pay_len = ((u16)raw[1] << 8) | raw[2];
	if (pay_len != size - CH9433_CMD_HEAD_LEN) {
		dev_err(&dev->udev->dev,
			"%s invalid payload len: exp %d, got %u\n", tag,
			size - CH9433_CMD_HEAD_LEN, pay_len);
		return -EPROTO;
	}

	sum = ch9433_calc_checksum(raw, size);
	sum_rx = ((u16)raw[size - CH9433_CMD_CKSUM_LEN] << 8) | raw[size - 1];
	if (sum != sum_rx) {
		dev_err(&dev->udev->dev,
			"%s checksum failed: calc %04x != rx %04x\n", tag, sum,
			sum_rx);
		return -EBADMSG;
	}

	return 0;
}

static int ch9433_clear_halt(struct ch9433 *dev, unsigned int pipe)
{
	int ret;

	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags))
		return -ENODEV;

	ret = usb_clear_halt(dev->udev, pipe);
	if (ret && ret != -EPIPE && ret != -ESHUTDOWN && ret != -ENODEV)
		dev_warn(&dev->udev->dev, "failed to clear %s EP%d halt: %d\n",
			 usb_pipein(pipe) ? "IN" : "OUT",
			 usb_pipeendpoint(pipe), ret);

	return ret;
}

static void ch9433_set_unplugged(struct ch9433 *dev)
{
	if (!test_and_set_bit(CH9433_DEV_UNPLUG, &dev->flags))
		smp_mb__after_atomic();
}

static void ch9433_mark_unplugged(struct ch9433 *dev)
{
	ch9433_set_unplugged(dev);
	netif_device_detach(dev->ndev);
	netif_stop_queue(dev->ndev);
}

static bool ch9433_netdev_active(struct ch9433 *dev)
{
	struct net_device *ndev = dev->ndev;

	return netif_device_present(ndev) && netif_running(ndev) &&
	       !test_bit(CH9433_DEV_UNPLUG, &dev->flags) &&
	       !test_bit(CH9433_STOPPING, &dev->flags);
}

static int ch9433_read_reg(struct ch9433 *dev, u8 reg, u32 *val)
{
	struct ch9433_usb_ctrlmsg_out *msg_out = NULL;
	struct ch9433_usb_ctrlmsg_in *msg_in = NULL;
	u8 cmd = CH9433_REG_OP_READ | CH9433_CANREG_CMD;
	u16 payload_len;
	u16 checksum;
	int actual_len;
	int ret;

	msg_out = kzalloc(CH9433_CMD_OUT_LEN, GFP_KERNEL);
	if (!msg_out)
		return -ENOMEM;

	msg_in = kzalloc(CH9433_CMD_IN_LEN, GFP_KERNEL);
	if (!msg_in) {
		ret = -ENOMEM;
		goto free_msg;
	}

	msg_out->header = CH9433_CMD_BULK_OUT;
	payload_len = CH9433_CMD_OUT_LEN - CH9433_CMD_HEAD_LEN;
	msg_out->len = cpu_to_be16(payload_len);
	msg_out->cmd = cmd;
	msg_out->reg = reg;
	checksum = ch9433_calc_checksum(msg_out, CH9433_CMD_OUT_LEN);
	msg_out->checksum = cpu_to_be16(checksum);

	mutex_lock(&dev->ctrl_lock);
	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags)) {
		ret = -ENODEV;
		goto out;
	}

	ret = usb_bulk_msg(dev->udev, dev->pipe_ctrl_out, msg_out,
			   CH9433_CMD_OUT_LEN, &actual_len,
			   CH9433_USB_CTRL_SET_TIMEOUT);
	if (ret) {
		dev_dbg(&dev->udev->dev, "read reg 0x%02x request failed: %d\n",
			reg, ret);
		if (ret == -EPIPE)
			ch9433_clear_halt(dev, dev->pipe_ctrl_out);
		goto out;
	}

	if (actual_len != CH9433_CMD_OUT_LEN) {
		dev_err(&dev->udev->dev,
			"read reg 0x%02x request short write: %d/%zu\n", reg,
			actual_len, CH9433_CMD_OUT_LEN);
		ret = -EIO;
		goto out;
	}

	ret = usb_bulk_msg(dev->udev, dev->pipe_ctrl_in, msg_in,
			   CH9433_CMD_IN_LEN, &actual_len,
			   CH9433_USB_CTRL_GET_TIMEOUT);
	if (ret) {
		dev_dbg(&dev->udev->dev,
			"read reg 0x%02x response failed: %d\n", reg, ret);
		if (ret == -EPIPE)
			ch9433_clear_halt(dev, dev->pipe_ctrl_in);
		goto out;
	}

	ret = ch9433_rx_fixed_chk(dev, "ctrl rx", msg_in, actual_len,
				  CH9433_CMD_CTRL_IN, CH9433_CMD_IN_LEN);
	if (ret) {
		dev_err(&dev->udev->dev,
			"read reg 0x%02x response invalid: %d\n", reg, ret);
		goto out;
	}

	*val = le32_to_cpu(msg_in->data);
out:
	mutex_unlock(&dev->ctrl_lock);
free_msg:
	kfree(msg_out);
	kfree(msg_in);
	return ret;
}

static int ch9433_write_reg(struct ch9433 *dev, u8 reg, u32 val)
{
	struct ch9433_usb_regmsg_out *msg_out = NULL;
	u16 payload_len = sizeof(*msg_out) - CH9433_CMD_HEAD_LEN;
	u16 checksum;
	int ret, actual_len;

	msg_out = kzalloc(sizeof(*msg_out), GFP_KERNEL);
	if (!msg_out)
		return -ENOMEM;

	msg_out->header = CH9433_CMD_BULK_OUT;
	msg_out->len = cpu_to_be16(payload_len);
	msg_out->cmd = CH9433_REG_OP_WRITE | CH9433_CANREG_CMD;
	msg_out->reg = reg;
	msg_out->data = cpu_to_le32(val);
	checksum = ch9433_calc_checksum(msg_out, sizeof(*msg_out));
	msg_out->checksum = cpu_to_be16(checksum);

	mutex_lock(&dev->ctrl_lock);
	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags)) {
		ret = -ENODEV;
		goto out;
	}

	ret = usb_bulk_msg(dev->udev, dev->pipe_ctrl_out, msg_out,
			   sizeof(*msg_out), &actual_len,
			   CH9433_USB_CTRL_SET_TIMEOUT);
	if (ret) {
		dev_dbg(&dev->udev->dev, "write reg 0x%02x=0x%08x failed: %d\n",
			reg, val, ret);
		if (ret == -EPIPE)
			ch9433_clear_halt(dev, dev->pipe_ctrl_out);
		goto out;
	}
	if (actual_len != sizeof(*msg_out))
		dev_err(&dev->udev->dev,
			"write reg 0x%02x=0x%08x short write: %d/%zu\n", reg,
			val, actual_len, sizeof(*msg_out));

out:
	mutex_unlock(&dev->ctrl_lock);
	kfree(msg_out);
	return ret ? ret : (actual_len == sizeof(*msg_out) ? 0 : -EIO);
}

static int ch9433_write(struct ch9433 *dev, u8 cmd, u32 n_tx,
			const void *tx_buf)
{
	u8 *buf;
	u16 checksum;
	u32 payload_len;
	u32 frame_len;
	int ret, actual_len;

	if (n_tx >
	    CH9433_MAX_FRAME - (CH9433_CMD_HEAD_LEN + 1 + CH9433_CMD_CKSUM_LEN))
		return -EINVAL;

	payload_len = 1 + n_tx + 2; // cmd (1B) + data (n_tx) + cksum
	frame_len = 1 + 2 + payload_len; // head + len + payload

	buf = kzalloc(frame_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = CH9433_CMD_BULK_OUT;
	buf[1] = (payload_len >> 8) & 0xff;
	buf[2] = payload_len & 0xff;
	buf[3] = cmd;
	memcpy(&buf[4], tx_buf, n_tx);
	checksum = ch9433_calc_checksum(buf, frame_len);
	buf[frame_len - 2] = (checksum >> 8) & 0xff;
	buf[frame_len - 1] = checksum & 0xff;

	mutex_lock(&dev->ctrl_lock);
	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags)) {
		ret = -ENODEV;
		mutex_unlock(&dev->ctrl_lock);
		kfree(buf);
		return ret;
	}

	ret = usb_bulk_msg(dev->udev, dev->pipe_ctrl_out, buf, frame_len,
			   &actual_len, CH9433_USB_CTRL_SET_TIMEOUT);
	if (ret == -EPIPE)
		ch9433_clear_halt(dev, dev->pipe_ctrl_out);
	mutex_unlock(&dev->ctrl_lock);

	kfree(buf);

	if (ret) {
		dev_dbg(&dev->udev->dev, "write cmd 0x%02x failed: %d\n", cmd,
			ret);
		return ret;
	}

	if (actual_len != frame_len) {
		dev_err(&dev->udev->dev,
			"write cmd 0x%02x short write: %d/%u\n", cmd,
			actual_len, frame_len);
		return -EIO;
	}

	return 0;
}

static int ch9433_iofunc_set(struct ch9433 *dev, u8 io_cmd, u8 io_addr,
			     u8 enable)
{
	u8 cmd_buf[4] = { 0 };
	u8 cmd_w = CH9433_IO_SEL_FUN_CFG | 0x80;
	int ret;

	cmd_buf[0] = io_cmd;
	cmd_buf[1] = io_addr;
	cmd_buf[2] = enable;
	cmd_buf[3] = CH9433_CMD_BULK_OUT;

	ret = ch9433_write(dev, cmd_w, 4, cmd_buf);

	return ret;
}

static void ch9433_kill_tx_urb(struct ch9433 *dev)
{
	struct ch9433_tx_agg *tx_agg;

	list_for_each_entry(tx_agg, &dev->txq_info, info_list)
		usb_kill_urb(tx_agg->urb);
}

static void ch9433_free_rx_agg(struct ch9433 *dev, struct ch9433_rx_agg *rx_agg)
{
	list_del_init(&rx_agg->info_list);

	usb_free_urb(rx_agg->urb);
	usb_free_coherent(dev->udev, dev->rx_urb_size, rx_agg->buffer,
			  rx_agg->dma);
	kfree(rx_agg);

	atomic_dec(&dev->rx_count);
}

static void ch9433_free_tx_agg(struct ch9433 *dev, struct ch9433_tx_agg *tx_agg)
{
	if (!list_empty(&tx_agg->list))
		list_del_init(&tx_agg->list);
	list_del_init(&tx_agg->info_list);

	usb_free_urb(tx_agg->urb);
	usb_free_coherent(dev->udev, dev->tx_urb_size + 4, tx_agg->buffer,
			  tx_agg->dma);
	kfree(tx_agg);
}

static void ch9433_free_all_resources(struct ch9433 *dev)
{
	LIST_HEAD(rx_free);
	LIST_HEAD(tx_free);
	struct ch9433_rx_agg *rx_agg, *rx_agg_next;
	struct ch9433_tx_agg *tx_agg, *tx_agg_next;
	unsigned long flags;

	spin_lock_irqsave(&dev->rxq_lock, flags);

	list_for_each_entry_safe(rx_agg, rx_agg_next, &dev->rxq_info, info_list)
		list_move_tail(&rx_agg->info_list, &rx_free);

	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	list_for_each_entry_safe(rx_agg, rx_agg_next, &rx_free, info_list)
		ch9433_free_rx_agg(dev, rx_agg);

	WARN_ON(atomic_read(&dev->rx_count));

	spin_lock_irqsave(&dev->txq_lock, flags);

	list_for_each_entry_safe(tx_agg, tx_agg_next, &dev->txq_info,
				 info_list) {
		if (!list_empty(&tx_agg->list))
			list_del_init(&tx_agg->list);
		list_move_tail(&tx_agg->info_list, &tx_free);
	}

	spin_unlock_irqrestore(&dev->txq_lock, flags);

	list_for_each_entry_safe(tx_agg, tx_agg_next, &tx_free, info_list)
		ch9433_free_tx_agg(dev, tx_agg);
}

static struct ch9433_rx_agg *ch9433_alloc_rx_agg(struct ch9433 *dev,
						 gfp_t mflags)
{
	struct net_device *ndev = dev->ndev;
	int node = ndev->dev.parent ? dev_to_node(ndev->dev.parent) : -1;
	struct ch9433_rx_agg *rx_agg;
	unsigned long flags;

	rx_agg = kmalloc_node(sizeof(*rx_agg), mflags, node);
	if (!rx_agg)
		return NULL;

	rx_agg->buffer = usb_alloc_coherent(dev->udev, dev->rx_urb_size, mflags,
					    &rx_agg->dma);
	if (!rx_agg->buffer)
		goto free_rx;

	rx_agg->urb = usb_alloc_urb(0, mflags);
	if (!rx_agg->urb)
		goto free_buf;

	rx_agg->context = dev;

	INIT_LIST_HEAD(&rx_agg->list);
	INIT_LIST_HEAD(&rx_agg->info_list);
	spin_lock_irqsave(&dev->rxq_lock, flags);
	list_add_tail(&rx_agg->info_list, &dev->rxq_info);
	spin_unlock_irqrestore(&dev->rxq_lock, flags);

	atomic_inc(&dev->rx_count);

	return rx_agg;

free_buf:
	usb_free_coherent(dev->udev, dev->rx_urb_size, rx_agg->buffer,
			  rx_agg->dma);
free_rx:
	kfree(rx_agg);
	return NULL;
}

static int ch9433_alloc_all_resources(struct ch9433 *dev)
{
	struct net_device *ndev = dev->ndev;
	int node, i;
	int ret = 0;

	node = ndev->dev.parent ? dev_to_node(ndev->dev.parent) : -1;

	spin_lock_init(&dev->txq_lock);
	spin_lock_init(&dev->rxq_lock);
	INIT_LIST_HEAD(&dev->txq_free);
	INIT_LIST_HEAD(&dev->txq_info);
	INIT_LIST_HEAD(&dev->rxq_info);
	atomic_set(&dev->rx_count, 0);

	for (i = 0; i < dev->n_rx_urbs; i++) {
		if (!ch9433_alloc_rx_agg(dev, GFP_KERNEL))
			goto error;
	}

	for (i = 0; i < dev->n_tx_urbs; i++) {
		struct ch9433_tx_agg *tx_agg;
		struct urb *urb;
		dma_addr_t dma;
		u8 *buf;

		tx_agg = kmalloc_node(sizeof(*tx_agg), GFP_KERNEL, node);
		if (!tx_agg) {
			ret = -ENOMEM;
			goto error;
		}

		buf = usb_alloc_coherent(dev->udev, dev->tx_urb_size + 4,
					 GFP_KERNEL, &dma);
		if (!buf) {
			kfree(tx_agg);
			ret = -ENOMEM;
			goto error;
		}

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			usb_free_coherent(dev->udev, dev->tx_urb_size + 4, buf,
					  dma);
			kfree(tx_agg);
			ret = -ENOMEM;
			goto error;
		}

		INIT_LIST_HEAD(&tx_agg->list);
		INIT_LIST_HEAD(&tx_agg->info_list);
		tx_agg->context = dev;
		tx_agg->urb = urb;
		tx_agg->buffer = buf;
		tx_agg->head = buf;
		tx_agg->dma = dma;
		tx_agg->index = i;

		list_add_tail(&tx_agg->list, &dev->txq_free);
		list_add_tail(&tx_agg->info_list, &dev->txq_info);
	}

	return ret;

error:
	ch9433_free_all_resources(dev);

	return -ENOMEM;
}

static struct ch9433_tx_agg *ch9433_get_tx_agg(struct ch9433 *dev)
{
	struct ch9433_tx_agg *tx_agg = NULL;
	unsigned long flags;

	spin_lock_irqsave(&dev->txq_lock, flags);
	if (!list_empty(&dev->txq_free)) {
		struct list_head *cursor;

		cursor = dev->txq_free.next;
		list_del_init(cursor);
		tx_agg = list_entry(cursor, struct ch9433_tx_agg, list);
	}
	spin_unlock_irqrestore(&dev->txq_lock, flags);

	return tx_agg;
}

static void ch9433_put_tx_agg(struct ch9433 *dev, struct ch9433_tx_agg *tx_agg)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->txq_lock, flags);
	list_add_tail(&tx_agg->list, &dev->txq_free);
	spin_unlock_irqrestore(&dev->txq_lock, flags);
}

static const struct ethtool_ops ch9433_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static int ch9433_wait_reg_state(struct ch9433 *dev, u8 reg, u32 mask, u32 val,
				 bool check_present)
{
	unsigned long timeout = jiffies + HZ;
	u32 reg_val;
	int ret;

	while (time_before(jiffies, timeout)) {
		if (check_present && !netif_device_present(dev->ndev))
			return -ENODEV;

		ret = ch9433_read_reg(dev, reg, &reg_val);
		if (ret)
			return ret;

		if ((reg_val & mask) == val)
			return 0;

		usleep_range(CH9433_DELAY_MS * 1000,
			     CH9433_DELAY_MS * 1000 * 2);
	}

	return -EBUSY;
}

static int ch9433_set_bittiming(struct ch9433 *dev)
{
	struct can_bittiming *bt = &dev->can.bittiming;
	u32 reg_val;

	DRV_DEBUG(
		&dev->udev->dev,
		"bitrate:%d sample_point:%d tq:%d prop_seg:%d phase_seg1:%d phase_seg2:%d sjw:%d brp:%d\n",
		bt->bitrate, bt->sample_point, bt->tq, bt->prop_seg,
		bt->phase_seg1, bt->phase_seg2, bt->sjw, bt->brp);

	reg_val = ((bt->sjw - 1) << 24) |
		  ((bt->prop_seg + bt->phase_seg1 - 1) << 16) |
		  ((bt->phase_seg2 - 1) << 20) | ((bt->brp - 1) & 0x000003FF);
	if (dev->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		reg_val |= CAN_BTIMR_LBKM;
	else if (dev->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		reg_val |= CAN_BTIMR_SILM;

	DRV_DEBUG(&dev->udev->dev, "CAN_BTIMR val: %x\n", reg_val);

	return ch9433_write_reg(dev, CH9433_CAN_BTIMR, reg_val);
}

static int ch9433_can_init(struct ch9433 *dev)
{
	u32 reg_val;
	int ret;

	DRV_DEBUG(&dev->udev->dev, "%s\n", __func__);

	if (!netif_device_present(dev->ndev))
		return -ENODEV;

	ret = ch9433_read_reg(dev, CH9433_CAN_CTLR, &reg_val);
	if (ret)
		return ret;
	reg_val &= ~CAN_CTLR_SLEEP;
	reg_val |= CAN_CTLR_INRQ;
	ret = ch9433_write_reg(dev, CH9433_CAN_CTLR, reg_val);
	if (ret)
		return ret;

	ret = ch9433_wait_reg_state(dev, CH9433_CAN_STATR, CAN_STATR_INAK,
				    CAN_STATR_INAK, true);
	if (ret) {
		if (ret == -EBUSY)
			dev_err(&dev->udev->dev,
				"didn't enter in conf mode after reset\n");
		return ret;
	}

	ret = ch9433_read_reg(dev, CH9433_CAN_CTLR, &reg_val);
	if (ret)
		return ret;
	reg_val &= ~(CAN_CTLR_TTCM | CAN_CTLR_ABOM | CAN_CTLR_AWUM |
		     CAN_CTLR_RFLM);
	reg_val |= CAN_CTLR_TXFP;
	if (dev->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		reg_val |= CAN_CTLR_NART;
	else
		reg_val &= ~CAN_CTLR_NART;
	ret = ch9433_write_reg(dev, CH9433_CAN_CTLR, reg_val);
	if (ret)
		return ret;

	ret = ch9433_set_bittiming(dev);
	if (ret)
		return ret;

	ret = ch9433_read_reg(dev, CH9433_CAN_CTLR, &reg_val);
	if (ret)
		return ret;
	reg_val &= ~CAN_CTLR_INRQ;
	ret = ch9433_write_reg(dev, CH9433_CAN_CTLR, reg_val);
	if (ret)
		return ret;

	ret = ch9433_wait_reg_state(dev, CH9433_CAN_STATR, CAN_STATR_INAK, 0,
				    true);
	if (ret) {
		if (ret == -EBUSY)
			dev_err(&dev->udev->dev,
				"didn't leave conf mode after reset\n");
		return ret;
	}

	return 0;
}

static int ch9433_can_filterinit(struct ch9433 *dev)
{
	u32 filter_num = 0;
	u32 filter_bit = 0;
	u32 reg_val;
	int ret;

	DRV_DEBUG(&dev->udev->dev, "%s\n", __func__);

	for (filter_num = 0; filter_num < 1; filter_num++) {
		filter_bit = BIT(filter_num);

		ret = ch9433_read_reg(dev, CH9433_CAN_FCTLR, &reg_val);
		if (ret)
			return ret;
		reg_val |= CAN_FCTLR_FINIT;
		ret = ch9433_write_reg(dev, CH9433_CAN_FCTLR, reg_val);
		if (ret)
			return ret;

		ret = ch9433_read_reg(dev, CH9433_CAN_FWR, &reg_val);
		if (ret)
			return ret;
		reg_val &= ~filter_bit;
		ret = ch9433_write_reg(dev, CH9433_CAN_FWR, reg_val);
		if (ret)
			return ret;

		ret = ch9433_read_reg(dev, CH9433_CAN_FSCFGR, &reg_val);
		if (ret)
			return ret;
		reg_val |= filter_bit;
		ret = ch9433_write_reg(dev, CH9433_CAN_FSCFGR, reg_val);
		if (ret)
			return ret;

		reg_val = 0x00000000;
		ret = ch9433_write_reg(dev, CH9433_CAN_FxR1(filter_num),
				       reg_val);
		if (ret)
			return ret;
		reg_val = 0x00000000;
		ret = ch9433_write_reg(dev, CH9433_CAN_FxR2(filter_num),
				       reg_val);
		if (ret)
			return ret;

		ret = ch9433_read_reg(dev, CH9433_CAN_FMCFGR, &reg_val);
		if (ret)
			return ret;
		reg_val &= ~filter_bit;
		ret = ch9433_write_reg(dev, CH9433_CAN_FMCFGR, reg_val);
		if (ret)
			return ret;

		ret = ch9433_read_reg(dev, CH9433_CAN_FAFIFOR, &reg_val);
		if (ret)
			return ret;
		reg_val &= ~filter_bit;
		ret = ch9433_write_reg(dev, CH9433_CAN_FAFIFOR, reg_val);
		if (ret)
			return ret;

		ret = ch9433_read_reg(dev, CH9433_CAN_FWR, &reg_val);
		if (ret)
			return ret;
		reg_val |= filter_bit;
		ret = ch9433_write_reg(dev, CH9433_CAN_FWR, reg_val);
		if (ret)
			return ret;

		ret = ch9433_read_reg(dev, CH9433_CAN_FCTLR, &reg_val);
		if (ret)
			return ret;
		reg_val &= ~CAN_FCTLR_FINIT;
		ret = ch9433_write_reg(dev, CH9433_CAN_FCTLR, reg_val);
		if (ret)
			return ret;
	}

	return 0;
}

static int ch9433_hw_sleep(struct ch9433 *dev, bool check_present)
{
	int ret;

	DRV_DEBUG(&dev->udev->dev, "%s\n", __func__);

	if (check_present && !netif_device_present(dev->ndev))
		return -ENODEV;

	ret = ch9433_write_reg(dev, CH9433_CAN_CTLR, CAN_CTLR_RESET);
	if (ret)
		return ret;

	ret = ch9433_wait_reg_state(dev, CH9433_CAN_STATR, CAN_STATR_SLAK,
				    CAN_STATR_SLAK, check_present);
	if (ret == -EBUSY)
		dev_err(&dev->udev->dev,
			"didn't enter in sleep mode after reset\n");

	return ret;
}

static int ch9433_setup(struct ch9433 *dev)
{
	int ret;

	ret = ch9433_can_init(dev);
	if (ret)
		return ret;

	ret = ch9433_can_filterinit(dev);
	if (ret)
		return ret;

	return 0;
}

static int ch9433_set_normal_mode(struct ch9433 *dev)
{
	u32 reg_val;
	int ret;

	DRV_DEBUG(&dev->udev->dev, "%s ctrlmode:0x%x\n", __func__,
		  dev->can.ctrlmode);

	if (!netif_device_present(dev->ndev))
		return -ENODEV;

	/* Enable interrupts */
	ret = ch9433_read_reg(dev, CH9433_CAN_INTENR, &reg_val);
	if (ret)
		return ret;
	reg_val |= (CAN_INTENR_FFIE0 | CAN_INTENR_FOVIE0 | CAN_INTENR_FFIE1 |
		    CAN_INTENR_FOVIE1 | CAN_INTENR_EWGIE | CAN_INTENR_EPVIE |
		    CAN_INTENR_BOFIE | CAN_INTENR_LECIE | CAN_INTENR_ERRIE |
		    CAN_INTENR_WKUIE | CAN_INTENR_SLKIE | CAN_INTENR_FMPIE0 |
		    CAN_INTENR_FMPIE1);
	ret = ch9433_write_reg(dev, CH9433_CAN_INTENR, reg_val);
	if (ret)
		return ret;

	ret = ch9433_read_reg(dev, CH9433_CAN_CTLR, &reg_val);
	if (ret)
		return ret;
	reg_val &= ~CAN_CTLR_INRQ;
	ret = ch9433_write_reg(dev, CH9433_CAN_CTLR, reg_val);
	if (ret)
		return ret;

	ret = ch9433_wait_reg_state(dev, CH9433_CAN_STATR, CAN_STATR_INAK, 0,
				    true);
	if (ret) {
		if (ret == -EBUSY)
			dev_err(&dev->udev->dev, "didn't enter normal mode\n");
		return ret;
	}

	dev->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

static void ch9433_can_err(struct work_struct *wq)
{
	struct ch9433 *dev = container_of(wq, struct ch9433, rx_status_work);
	struct net_device *ndev = dev->ndev;
	struct sk_buff *skb;
	struct can_frame *frame;
	struct net_device_stats *stats = &ndev->stats;
	enum can_state new_state;
	u32 err_state = 0;
	u8 txerr = 0, rxerr = 0;
	bool send_skb = false;
	bool tx_state = false;
	int ret = 0;

	DRV_DEBUG(&dev->udev->dev, "%s, dev->flags: 0x%lx\n", __func__,
		  dev->flags);

	mutex_lock(&dev->state_lock);
	if (!ch9433_netdev_active(dev)) {
		mutex_unlock(&dev->state_lock);
		return;
	}

	new_state = dev->can.state;
	skb = alloc_can_err_skb(ndev, &frame);
	if (!skb) {
		mutex_unlock(&dev->state_lock);
		return;
	}

	if (test_and_clear_bit(CH9433_RX_OVERFLOW, &dev->flags)) {
		DRV_DEBUG(&dev->udev->dev, "%s: RX overflow\n", __func__);
		dev_err(&dev->udev->dev, "CAN data overrun\n");
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;
		send_skb = true;
	}

	tx_state = test_and_clear_bit(CH9433_TX_ERR, &dev->flags);
	if (tx_state) {
		u32 reg_val = 0, tmp = 0, clear_val = 0;

		DRV_DEBUG(&dev->udev->dev, "%s: TX error flag set\n", __func__);

		ret = ch9433_read_reg(dev, CH9433_CAN_STATR, &reg_val);
		if (ret)
			goto out_free_skb;
		DRV_DEBUG(&dev->udev->dev, "%s: CAN_STATR=0x%08x\n", __func__,
			  reg_val);

		tmp = reg_val;
		if (reg_val & CAN_STATR_ERRI)
			tmp |= CAN_STATR_ERRI;
		if (reg_val & CAN_STATR_WKUI)
			tmp |= CAN_STATR_WKUI;
		if (reg_val & CAN_STATR_SLAKI)
			tmp |= CAN_STATR_SLAKI;

		if (reg_val &
		    (CAN_STATR_ERRI | CAN_STATR_WKUI | CAN_STATR_SLAKI))
			ret = ch9433_write_reg(dev, CH9433_CAN_STATR, tmp);
		if (ret)
			goto out_free_skb;

		ret = ch9433_read_reg(dev, CH9433_CAN_ERRSR, &err_state);
		if (ret)
			goto out_free_skb;
		DRV_DEBUG(&dev->udev->dev, "%s: CAN_ERRSR=0x%08x\n", __func__,
			  err_state);

		if (err_state) {
			if (err_state & CAN_ERRSR_LEC_MASK)
				clear_val = CAN_ERRSR_LEC_MASK;
			DRV_DEBUG(&dev->udev->dev,
				  "%s: clear CAN_ERRSR=0x%08x\n", __func__,
				  clear_val);
			ret = ch9433_write_reg(dev, CH9433_CAN_ERRSR,
					       clear_val);
			if (ret)
				goto out_free_skb;
		}
	}

	txerr = (err_state & CAN_ERRSR_TEC) >> 16;
	rxerr = (err_state & CAN_ERRSR_REC) >> 24;
	frame->data[6] = txerr;
	frame->data[7] = rxerr;
	DRV_DEBUG(&dev->udev->dev,
		  "%s: err_state=0x%08x txerr=%u rxerr=%u state=%d\n", __func__,
		  err_state, txerr, rxerr, dev->can.state);

	if (err_state & CAN_ERRSR_BOFF) {
		new_state = CAN_STATE_BUS_OFF;
		frame->can_id |= CAN_ERR_BUSOFF;
		dev->can.can_stats.bus_off++;
		send_skb = true;
		netif_stop_queue(ndev);
		can_bus_off(ndev);
		ch9433_hw_sleep(dev, true);
	} else if (err_state & CAN_ERRSR_EPVF) {
		new_state = CAN_STATE_ERROR_PASSIVE;
		frame->can_id |= CAN_ERR_CRTL;
		if (txerr > rxerr)
			frame->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
		else
			frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		send_skb = true;
	} else if (err_state & CAN_ERRSR_EWGF) {
		new_state = CAN_STATE_ERROR_WARNING;
		frame->can_id |= CAN_ERR_CRTL;
		if (txerr > rxerr)
			frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;
		else
			frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;
		send_skb = true;
	} else if (tx_state && dev->can.state != CAN_STATE_ERROR_ACTIVE) {
		new_state = CAN_STATE_ERROR_ACTIVE;
		frame->can_id |= CAN_ERR_CRTL;
		frame->data[1] |= CAN_ERR_CRTL_ACTIVE;
		send_skb = true;
	}

	if (new_state != dev->can.state) {
		DRV_DEBUG(&dev->udev->dev, "%s: state change %d -> %d\n",
			  __func__, dev->can.state, new_state);
		switch (dev->can.state) {
		case CAN_STATE_ERROR_ACTIVE:
			if (new_state >= CAN_STATE_ERROR_WARNING &&
			    new_state <= CAN_STATE_BUS_OFF)
				dev->can.can_stats.error_warning++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
			fallthrough;
#endif
		case CAN_STATE_ERROR_WARNING:
			if (new_state >= CAN_STATE_ERROR_PASSIVE &&
			    new_state <= CAN_STATE_BUS_OFF)
				dev->can.can_stats.error_passive++;
			break;
		default:
			break;
		}
		dev->can.state = new_state;
	}

	DRV_DEBUG(&dev->udev->dev,
		  "%s: send_skb=%d can_id=0x%08x data1=0x%02x\n", __func__,
		  send_skb, frame->can_id, frame->data[1]);

	if (!send_skb)
		goto out_free_skb;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	mutex_unlock(&dev->state_lock);
	netif_rx(skb);
#else
	mutex_unlock(&dev->state_lock);
	netif_rx_ni(skb);
#endif
	return;

out_free_skb:
	if (ret && tx_state) {
		DRV_DEBUG(&dev->udev->dev,
			  "%s: ret=%d, restore TX error flag\n", __func__, ret);
		set_bit(CH9433_TX_ERR, &dev->flags);
	}
	kfree_skb(skb);
	mutex_unlock(&dev->state_lock);
}

static int ch9433_rx_submit(struct ch9433 *dev, struct ch9433_rx_agg *rx_agg,
			    gfp_t flags);
static int ch9433_stop_rx(struct ch9433 *dev);

static void ch9433_schedule_recovery(struct ch9433 *dev, bool tx)
{
	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags) ||
	    test_bit(CH9433_STOPPING, &dev->flags))
		return;

	if (tx)
		set_bit(CH9433_TX_HALT, &dev->flags);
	else
		set_bit(CH9433_RX_HALT, &dev->flags);
	set_bit(CH9433_RECOVERING, &dev->flags);
	netif_stop_queue(dev->ndev);
	schedule_work(&dev->recover_work);
}

static int ch9433_get_rx_msg_info(struct ch9433 *dev,
				  const struct ch9433_usb_bulkmsg_in *msg,
				  int len)
{
	const u8 *raw = (const u8 *)msg;
	const int meta_len = offsetof(struct ch9433_usb_bulkmsg_in, can_msg) -
			     CH9433_CMD_HEAD_LEN;
	int msg_bytes;
	int avail;
	int pkt_len;
	int cnt;
	u16 sum;
	u16 sum_rx;
	u16 pay_len;

	if (len < offsetof(struct ch9433_usb_bulkmsg_in, can_msg) +
			  CH9433_CMD_CKSUM_LEN) {
		netdev_warn(dev->ndev, "short bulk rx packet: %d\n", len);
		return -EIO;
	}

	if (msg->header != CH9433_CMD_BULK_IN) {
		netdev_warn(dev->ndev, "invalid bulk rx header: %x\n",
			    msg->header);
		return -EPROTO;
	}

	pay_len = be16_to_cpu(msg->len);
	if (pay_len < meta_len + CH9433_CMD_CKSUM_LEN) {
		netdev_warn(dev->ndev, "invalid bulk rx payload len: %u\n",
			    pay_len);
		return -EPROTO;
	}

	pkt_len = CH9433_CMD_HEAD_LEN + pay_len;
	if (pkt_len > len) {
		netdev_warn(dev->ndev,
			    "truncated bulk rx packet: exp %d, got %d\n",
			    pkt_len, len);
		return -EIO;
	}

	sum = ch9433_calc_checksum(raw, pkt_len);
	sum_rx = ((u16)raw[pkt_len - CH9433_CMD_CKSUM_LEN] << 8) |
		 raw[pkt_len - 1];
	if (sum != sum_rx) {
		netdev_warn(dev->ndev,
			    "bulk rx checksum failed: calc %04x != rx %04x\n",
			    sum, sum_rx);
		return -EBADMSG;
	}

	msg_bytes = pay_len - meta_len - CH9433_CMD_CKSUM_LEN;
	if (msg_bytes < 0 || msg_bytes % sizeof(msg->can_msg[0])) {
		netdev_warn(dev->ndev, "invalid bulk rx msg bytes: %d\n",
			    msg_bytes);
		return -EPROTO;
	}

	avail = msg_bytes / sizeof(msg->can_msg[0]);
	avail = min_t(int, avail, CH9433_USB_BULK_RX_CAN_MSG_MAX);

	/* box_num reports valid CAN frames in this USB packet. */
	cnt = min_t(int, msg->box_num, avail);

	return cnt;
}

static void ch9433_rx_fixup(struct ch9433 *dev,
			    struct ch9433_can_rx_msg *can_msg)
{
	struct sk_buff *skb;
	struct can_frame *frame;
	u32 mir, mdtr, data_l, data_h;

	skb = alloc_can_skb(dev->ndev, &frame);
	if (!skb) {
		dev_err(&dev->udev->dev, "%s, cannot allocate RX skb\n",
			__func__);
		dev->ndev->stats.rx_dropped++;
		return;
	}

	mir = le32_to_cpu(can_msg->mir);
	mdtr = le32_to_cpu(can_msg->mdtr);
	data_l = le32_to_cpu(can_msg->data_l);
	data_h = le32_to_cpu(can_msg->data_h);

	if (mir & MIR_IDE) {
		/* Extended ID format */
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |= ((mir & MIR_EXT_ID_MASK) >> MIR_EXT_ID_SHIFT);
	} else {
		/* Standard ID format */
		frame->can_id = ((mir & MIR_STD_ID_MASK) >> MIR_STD_ID_SHIFT);
	}

	/* Remote transmission request */
	if (mir & MIR_RTR)
		frame->can_id |= CAN_RTR_FLAG;

		/* Data length */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	frame->len = can_cc_dlc2len(mdtr & MDTR_DLC_MASK);
#else
	frame->can_dlc = get_can_dlc(mdtr & MDTR_DLC_MASK);
#endif
	if (!(frame->can_id & CAN_RTR_FLAG)) {
		put_unaligned_le32(data_l, &frame->data[0]);
		put_unaligned_le32(data_h, &frame->data[4]);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
		dev->ndev->stats.rx_bytes += frame->len;
#else
		dev->ndev->stats.rx_bytes += frame->can_dlc;
#endif
	}
	dev->ndev->stats.rx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	netif_rx(skb);
#else
	netif_rx_ni(skb);
#endif
}

static void ch9433_rx_bulk(struct ch9433 *dev, struct ch9433_rx_agg *rx_agg)
{
	struct ch9433_usb_bulkmsg_in *usb_msg;
	int msg_count;
	int i;

	/* analyze the data packet */
	usb_msg = (struct ch9433_usb_bulkmsg_in *)rx_agg->buffer;
	msg_count = ch9433_get_rx_msg_info(dev, usb_msg,
					   rx_agg->urb->actual_length);
	if (msg_count <= 0)
		return;

	for (i = 0; i < msg_count; i++) {
		ch9433_rx_fixup(dev, &usb_msg->can_msg[i]);
	}
}

static void ch9433_rx_status(struct ch9433 *dev, struct ch9433_rx_agg *rx_agg)
{
	struct ch9433_usb_statsmsg *usb_msg;
	bool need_work = false;
	int ret;

	/* analyze the data packet */
	usb_msg = (struct ch9433_usb_statsmsg *)rx_agg->buffer;
	ret = ch9433_rx_fixed_chk(dev, "status rx", usb_msg,
				  rx_agg->urb->actual_length,
				  CH9433_CMD_STAT_IN, sizeof(*usb_msg));
	if (ret)
		return;

	if (usb_msg->flags & TX_ERR_OR_WAKE) {
		set_bit(CH9433_TX_ERR, &dev->flags);
		need_work = true;
	}

	if ((usb_msg->flags & RX_MBOX0_OVERFLOW) ||
	    (usb_msg->flags & RX_MBOX1_OVERFLOW)) {
		set_bit(CH9433_RX_OVERFLOW, &dev->flags);
		need_work = true;
	}

	if (need_work && !test_bit(CH9433_STOPPING, &dev->flags))
		schedule_work(&dev->rx_status_work);
}

static void ch9433_rx_complete(struct urb *urb)
{
	struct net_device *ndev;
	struct ch9433 *dev;
	struct ch9433_rx_agg *rx_agg;
	int urb_status = urb->status;
	int ret;

	rx_agg = urb->context;
	if (!rx_agg)
		return;

	dev = rx_agg->context;
	if (!dev)
		return;

	ndev = dev->ndev;

	switch (urb_status) {
	case 0:
		if (!ch9433_netdev_active(dev))
			return;

		if (urb->actual_length < 1) {
			netdev_warn(ndev, "empty rx urb\n");
			break;
		}

		switch (((u8 *)rx_agg->buffer)[0]) {
		case CH9433_CMD_STAT_IN:
			ch9433_rx_status(dev, rx_agg);
			break;
		case CH9433_CMD_BULK_IN:
			ch9433_rx_bulk(dev, rx_agg);
			break;
		default:
			netdev_warn(dev->ndev, "invalid message header: %x\n",
				    ((u8 *)rx_agg->buffer)[0]);
			break;
		}
		break;
	case -ECONNRESET:
	case -ENOENT:
		return;
	case -ENODEV:
	case -ESHUTDOWN:
		ch9433_mark_unplugged(dev);
		return;
	case -EPIPE:
		ndev->stats.rx_errors++;
		netdev_dbg(ndev, "RX endpoint stalled, scheduling recovery\n");
		ch9433_schedule_recovery(dev, false);
		return;
	case -EPROTO:
	case -ETIME:
		ndev->stats.rx_errors++;
		netdev_dbg(ndev, "transient RX URB status %d, resubmitting\n",
			   urb_status);
		break;
	default:
		ndev->stats.rx_errors++;
		if (net_ratelimit())
			netdev_warn(ndev, "RX URB status %d, resubmitting\n",
				    urb_status);
		break;
	}

	if (!ch9433_netdev_active(dev))
		return;

	ret = ch9433_rx_submit(dev, rx_agg, GFP_ATOMIC);
	if (ret == -EPIPE)
		ch9433_schedule_recovery(dev, false);
}

static int ch9433_rx_submit(struct ch9433 *dev, struct ch9433_rx_agg *rx_agg,
			    gfp_t flags)
{
	struct net_device *ndev = dev->ndev;
	int ret = 0;
	size_t size = dev->rx_urb_size;

	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags) ||
	    test_bit(CH9433_STOPPING, &dev->flags))
		return -ENODEV;

	usb_fill_bulk_urb(rx_agg->urb, dev->udev, dev->pipe_in, rx_agg->buffer,
			  size, ch9433_rx_complete, rx_agg);
	rx_agg->urb->transfer_dma = rx_agg->dma;
	rx_agg->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	ret = usb_submit_urb(rx_agg->urb, flags);
	if (!ret)
		return 0;

	rx_agg->urb->actual_length = 0;

	switch (ret) {
	case -ENODEV:
	case -ESHUTDOWN:
		ch9433_mark_unplugged(dev);
		netdev_dbg(ndev, "RX URB submit saw USB gone: %d\n", ret);
		break;
	case -EPROTO:
	case -ETIME:
		ndev->stats.rx_errors++;
		netdev_dbg(ndev, "transient RX submit error %d\n", ret);
		break;
	default:
		ndev->stats.rx_errors++;
		if (net_ratelimit())
			netdev_warn(ndev, "RX URB submit failed: %d\n", ret);
		break;
	}

	return ret;
}

static int ch9433_start_rx(struct ch9433 *dev)
{
	struct ch9433_rx_agg *rx_agg, *rx_agg_next;
	int ret = 0;

	list_for_each_entry_safe(rx_agg, rx_agg_next, &dev->rxq_info,
				 info_list) {
		ret = ch9433_rx_submit(dev, rx_agg, GFP_KERNEL);
		if (ret)
			goto stop_rx;
	}

	return ret;

stop_rx:
	ch9433_stop_rx(dev);
	return ret;
}

static int ch9433_stop_rx(struct ch9433 *dev)
{
	struct ch9433_rx_agg *rx_agg, *rx_agg_next;

	/* The usb_kill_urb() couldn't be used in atomic.
	 * rxq_info is the allocation list and remains stable until resources
	 * are freed, so URBs can be killed without moving list nodes.
	 */

	list_for_each_entry_safe(rx_agg, rx_agg_next, &dev->rxq_info, info_list)
		usb_kill_urb(rx_agg->urb);

	return 0;
}

static void ch9433_recover_work_handler(struct work_struct *wq)
{
	struct ch9433 *dev = container_of(wq, struct ch9433, recover_work);
	struct net_device *ndev = dev->ndev;
	int ret;

	mutex_lock(&dev->state_lock);
	if (!ch9433_netdev_active(dev))
		goto out_unlock;

	/* Unlink all pending URBs on a halted endpoint before clearing it. */
	if (test_bit(CH9433_TX_HALT, &dev->flags)) {
		ch9433_kill_tx_urb(dev);
		ret = ch9433_clear_halt(dev, dev->pipe_out);
		if (ret == -ENODEV || ret == -ESHUTDOWN) {
			ch9433_mark_unplugged(dev);
			goto out_unlock;
		}
		if (ret && ret != -EPIPE) {
			netdev_err(ndev, "TX endpoint recovery failed: %d\n",
				   ret);
		} else {
			clear_bit(CH9433_TX_HALT, &dev->flags);
		}
	}

	if (test_bit(CH9433_STOPPING, &dev->flags))
		goto out_unlock;

	if (test_bit(CH9433_RX_HALT, &dev->flags)) {
		int clear_ret;

		ch9433_stop_rx(dev);
		clear_ret = ch9433_clear_halt(dev, dev->pipe_in);
		if (clear_ret == -ENODEV || clear_ret == -ESHUTDOWN) {
			ch9433_mark_unplugged(dev);
			goto out_unlock;
		}
		ret = clear_ret;
		if ((!clear_ret || clear_ret == -EPIPE) &&
		    !test_bit(CH9433_STOPPING, &dev->flags))
			ret = ch9433_start_rx(dev);
		if ((clear_ret && clear_ret != -EPIPE) || ret) {
			netdev_err(ndev, "RX endpoint recovery failed: %d\n",
				   ret);
		} else {
			clear_bit(CH9433_RX_HALT, &dev->flags);
		}
	}

	if (!test_bit(CH9433_RX_HALT, &dev->flags) &&
	    !test_bit(CH9433_TX_HALT, &dev->flags)) {
		clear_bit(CH9433_RECOVERING, &dev->flags);
		if (!test_bit(CH9433_STOPPING, &dev->flags) &&
		    dev->can.state != CAN_STATE_BUS_OFF &&
		    dev->can.state != CAN_STATE_STOPPED)
			netif_wake_queue(ndev);
	}

out_unlock:
	mutex_unlock(&dev->state_lock);
}

static void ch9433_cancel_deferred_work(struct ch9433 *dev)
{
	cancel_work_sync(&dev->rx_status_work);
	cancel_work_sync(&dev->recover_work);
	cancel_delayed_work_sync(&dev->can.restart_work);
}

/*
 * Open can device
 * Called when the can device is marked active, such as a user executing
 * 'ifconfig candev up' on the device
 */
static int ch9433_open(struct net_device *ndev)
{
	struct ch9433 *dev = netdev_priv(ndev);
	int ret;

	DRV_DEBUG(&dev->udev->dev, "%s\n", __func__);

	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags))
		return -ENODEV;
	clear_bit(CH9433_STOPPING, &dev->flags);
	clear_bit(CH9433_RX_HALT, &dev->flags);
	clear_bit(CH9433_TX_HALT, &dev->flags);
	clear_bit(CH9433_RECOVERING, &dev->flags);

	ret = open_candev(ndev);
	if (ret) {
		dev_err(&dev->udev->dev, "%s, Unable to init baudrate!\n",
			__func__);
		return ret;
	}

	mutex_lock(&dev->state_lock);

	ret = ch9433_iofunc_set(dev, 0x03, 3, 1);
	if (ret)
		goto out_close;

	ret = ch9433_setup(dev);
	if (ret)
		goto out_close;

	ret = ch9433_alloc_all_resources(dev);
	if (ret) {
		netdev_err(ndev, "%s: resource allocation failed: %d\n",
			   __func__, ret);
		goto out_sleep;
	}

	ret = ch9433_start_rx(dev);
	if (ret) {
		netdev_err(ndev, "%s: RX start failed: %d\n", __func__, ret);
		goto out_free_resources;
	}

	ret = ch9433_set_normal_mode(dev);
	if (ret)
		goto out_free_resources;

	netif_wake_queue(ndev);
	mutex_unlock(&dev->state_lock);

	return 0;

out_free_resources:
	ch9433_stop_rx(dev);
	ch9433_free_all_resources(dev);
out_sleep:
	if (netif_device_present(ndev)) {
		ch9433_write_reg(dev, CH9433_CAN_INTENR, 0);
		ch9433_hw_sleep(dev, true);
	}
out_close:
	dev->can.state = CAN_STATE_STOPPED;
	mutex_unlock(&dev->state_lock);
	close_candev(ndev);
	netdev_err(ndev, "%s : open failed, ret: %d\n", __func__, ret);
	return ret;
}

static int ch9433_stop(struct net_device *ndev)
{
	struct ch9433 *dev = netdev_priv(ndev);
	bool present = netif_device_present(ndev);

	DRV_DEBUG(&dev->udev->dev, "%s\n", __func__);

	set_bit(CH9433_STOPPING, &dev->flags);
	netif_stop_queue(ndev);
	ch9433_stop_rx(dev);
	ch9433_cancel_deferred_work(dev);
	ch9433_stop_rx(dev);

	mutex_lock(&dev->state_lock);

	if (present)
		ch9433_write_reg(dev, CH9433_CAN_INTENR, 0);

	ch9433_kill_tx_urb(dev);
	ch9433_free_all_resources(dev);

	dev->can.state = CAN_STATE_STOPPED;
	if (present)
		ch9433_hw_sleep(dev, true);
	ch9433_iofunc_set(dev, 0x03, 3, 0);
	clear_bit(CH9433_RX_HALT, &dev->flags);
	clear_bit(CH9433_TX_HALT, &dev->flags);
	clear_bit(CH9433_RECOVERING, &dev->flags);
	clear_bit(CH9433_STOPPING, &dev->flags);
	mutex_unlock(&dev->state_lock);

	close_candev(ndev);

	return 0;
}

static bool ch9433_tx_ready(struct ch9433 *dev);

static void ch9433_tx_complete(struct urb *urb)
{
	struct net_device_stats *stats;
	struct net_device *ndev;
	struct ch9433_tx_agg *tx_agg;
	struct ch9433 *dev;

	tx_agg = urb->context;
	if (!tx_agg)
		return;

	dev = tx_agg->context;
	if (!dev)
		return;

	ndev = dev->ndev;
	stats = &ndev->stats;

	switch (urb->status) {
	case 0:
		if (dev->can.state == CAN_STATE_BUS_OFF ||
		    dev->can.state == CAN_STATE_STOPPED) {
			stats->tx_dropped++;
			goto free_echo;
		}
		stats->tx_packets++;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
		stats->tx_bytes += can_get_echo_skb(ndev, tx_agg->index, NULL);
#else
		stats->tx_bytes += can_get_echo_skb(ndev, tx_agg->index);
#endif
		break;
	case -ENODEV:
	case -ESHUTDOWN:
		ch9433_mark_unplugged(dev);
		if (net_ratelimit())
			netdev_dbg(ndev, "USB device unavailable, status %d\n",
				   urb->status);
		goto free_echo;
	case -ENOENT:
	case -ECONNRESET:
		if (test_bit(CH9433_RECOVERING, &dev->flags))
			stats->tx_dropped++;
		goto free_echo;
	case -EPIPE:
		stats->tx_errors++;
		stats->tx_dropped++;
		ch9433_schedule_recovery(dev, true);
		goto free_echo;
	default:
		if (net_ratelimit())
			netdev_dbg(ndev, "Tx status %d\n", urb->status);
		stats->tx_errors++;
		goto free_echo;
	}

	goto put_tx;

free_echo:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
	can_free_echo_skb(ndev, tx_agg->index, NULL);
#else
	can_free_echo_skb(ndev, tx_agg->index);
#endif

put_tx:
	ch9433_put_tx_agg(dev, tx_agg);

	if (ch9433_tx_ready(dev) && netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static int ch9433_tx_fixup(struct ch9433_tx_agg *tx_agg, struct sk_buff *skb)
{
	struct can_frame *frame = (struct can_frame *)skb->data;
	struct ch9433_usb_bulkmsg_out *usb_msg;
	u8 frame_len;
	u16 msg_len;
	u16 checksum;
	u32 sid = 0;
	u32 eid = 0;
	u32 exide;
	u32 rtr;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0))
	frame_len = frame->len;
#else
	frame_len = frame->can_dlc;
#endif
	msg_len = 2 + sizeof(struct ch9433_can_tx_msg) + 2;
	usb_msg = (struct ch9433_usb_bulkmsg_out *)tx_agg->head;
	usb_msg->header = CH9433_CMD_BULK_OUT;
	usb_msg->len = cpu_to_be16(msg_len);
	usb_msg->cmd_h = CH9433_REG_OP_WRITE | CH9433_CANREG_CMD;
	usb_msg->cmd_l = CH9433_CAN_TX0WRITE_CONT;

	exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0;
	if (exide) {
		eid = frame->can_id & CAN_EFF_MASK;
		sid = (frame->can_id & CAN_EFF_MASK) >> 18;
	} else {
		sid = frame->can_id & CAN_SFF_MASK;
	}
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;

	if (exide)
		usb_msg->can_msg.id =
			cpu_to_le32(((eid << 3) | CAN_TXMIRx_IDE | (rtr << 1)) |
				    CAN_TXMIRx_TXRQ);
	else
		usb_msg->can_msg.id = cpu_to_le32(((sid << 21) | (rtr << 1)) |
						  CAN_TXMIRx_TXRQ);

	usb_msg->can_msg.data_l = cpu_to_le32(get_unaligned_le32(frame->data));
	usb_msg->can_msg.data_h =
		cpu_to_le32(get_unaligned_le32(frame->data + 4));
	usb_msg->can_msg.len = cpu_to_le32(frame_len & 0x0000000F);

	checksum = ch9433_calc_checksum(tx_agg->head, CH9433_BULK_OUT_LEN);
	usb_msg->checksum = cpu_to_be16(checksum);

	return CH9433_BULK_OUT_LEN;
}

static netdev_tx_t ch9433_tx_submit(struct ch9433 *dev,
				    struct ch9433_tx_agg *tx_agg,
				    struct sk_buff *skb, int tx_len)
{
	struct net_device *ndev = dev->ndev;
	struct net_device_stats *stats = &ndev->stats;
	int ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
	can_put_echo_skb(skb, ndev, tx_agg->index, 0);
#else
	can_put_echo_skb(skb, ndev, tx_agg->index);
#endif

	usb_fill_bulk_urb(tx_agg->urb, dev->udev, dev->pipe_out, tx_agg->head,
			  tx_len, ch9433_tx_complete, tx_agg);
	tx_agg->urb->transfer_dma = tx_agg->dma;
	tx_agg->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	ret = usb_submit_urb(tx_agg->urb, GFP_ATOMIC);
	if (!ret)
		return NETDEV_TX_OK;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 0))
	can_free_echo_skb(ndev, tx_agg->index, NULL);
#else
	can_free_echo_skb(ndev, tx_agg->index);
#endif
	ch9433_put_tx_agg(dev, tx_agg);
	stats->tx_dropped++;

	switch (ret) {
	case -ENODEV:
	case -ESHUTDOWN:
		ch9433_mark_unplugged(dev);
		if (net_ratelimit())
			netdev_warn(ndev, "USB device unavailable, status %d\n",
				    ret);
		break;
	case -EPIPE:
		ch9433_schedule_recovery(dev, true);
		break;
	default:
		break;
	}

	return NETDEV_TX_OK;
}

static bool ch9433_tx_stopped(struct ch9433 *dev)
{
	struct net_device *ndev = dev->ndev;

	return !netif_device_present(ndev) ||
	       test_bit(CH9433_DEV_UNPLUG, &dev->flags) ||
	       test_bit(CH9433_STOPPING, &dev->flags) ||
	       dev->can.state == CAN_STATE_BUS_OFF ||
	       dev->can.state == CAN_STATE_STOPPED;
}

static bool ch9433_tx_ready(struct ch9433 *dev)
{
	return !ch9433_tx_stopped(dev) &&
	       !test_bit(CH9433_RECOVERING, &dev->flags) &&
	       netif_running(dev->ndev);
}

static bool ch9433_dropped_skb(struct net_device *ndev, struct sk_buff *skb)
{
	struct ch9433 *dev = netdev_priv(ndev);

	if (dev->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		kfree_skb(skb);
		ndev->stats.tx_dropped++;
		return true;
	}

	return can_dropped_invalid_skb(ndev, skb);
}

static netdev_tx_t ch9433_start_xmit(struct sk_buff *skb,
				     struct net_device *ndev)
{
	struct ch9433 *dev = netdev_priv(ndev);
	struct ch9433_tx_agg *tx_agg = NULL;
	struct net_device_stats *stats = &dev->ndev->stats;
	int tx_len;

	if (ch9433_dropped_skb(ndev, skb))
		return NETDEV_TX_OK;

	if (ch9433_tx_stopped(dev)) {
		stats->tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	tx_agg = ch9433_get_tx_agg(dev);
	if (!tx_agg) {
		netif_stop_queue(ndev);
		tx_agg = ch9433_get_tx_agg(dev);
		if (!tx_agg)
			return NETDEV_TX_BUSY;
		if (ch9433_tx_ready(dev))
			netif_wake_queue(ndev);
	}

	if (ch9433_tx_stopped(dev)) {
		ch9433_put_tx_agg(dev, tx_agg);
		stats->tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (test_bit(CH9433_RECOVERING, &dev->flags)) {
		ch9433_put_tx_agg(dev, tx_agg);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

	tx_len = ch9433_tx_fixup(tx_agg, skb);
	return ch9433_tx_submit(dev, tx_agg, skb, tx_len);
}

static const struct net_device_ops ch9433_netdev_ops = {
	.ndo_open = ch9433_open,
	.ndo_stop = ch9433_stop,
	.ndo_start_xmit = ch9433_start_xmit,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	.ndo_change_mtu = can_change_mtu,
#endif
};

static int ch9433_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct ch9433 *dev = netdev_priv(ndev);
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		mutex_lock(&dev->state_lock);
		if (!ch9433_netdev_active(dev)) {
			ret = -ENODEV;
			goto out_unlock;
		}

		ret = ch9433_hw_sleep(dev, true);
		if (ret)
			goto out_unlock;

		ret = ch9433_setup(dev);
		if (ret)
			goto out_unlock;

		ret = ch9433_set_normal_mode(dev);
		if (!ret) {
			netif_device_attach(ndev);
			netif_wake_queue(ndev);
		}

out_unlock:
		mutex_unlock(&dev->state_lock);
		return ret;
	default:
		return -EOPNOTSUPP;
	}
}

static void ch9433_free_dev(struct ch9433 *dev)
{
	struct net_device *ndev = dev->ndev;

	usb_put_dev(dev->udev);
	free_candev(ndev);
}

static bool ch9433_get_dev(struct ch9433 *dev)
{
	return atomic_inc_not_zero(&dev->refcnt);
}

static void ch9433_put_dev(struct ch9433 *dev)
{
	if (atomic_dec_and_test(&dev->refcnt))
		ch9433_free_dev(dev);
}

static int ch9433_alloc_minor(struct ch9433 *dev)
{
	int minor;

	mutex_lock(&ch9433_minors_lock);
	for (minor = 0; minor < CH9433_MAX_NUM; minor++) {
		if (!ch9433_table[minor]) {
			ch9433_table[minor] = dev;
			break;
		}
	}
	mutex_unlock(&ch9433_minors_lock);

	return minor < CH9433_MAX_NUM ? minor : -ENOSPC;
}

static void ch9433_release_minor(struct ch9433 *dev)
{
	mutex_lock(&ch9433_minors_lock);
	ch9433_set_unplugged(dev);
	if (dev->minor >= 0 && dev->minor < CH9433_MAX_NUM &&
	    ch9433_table[dev->minor] == dev)
		ch9433_table[dev->minor] = NULL;
	mutex_unlock(&ch9433_minors_lock);
}

static int ch9433_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct ch9433 *dev;
	struct net_device *ndev;
	struct usb_device *udev;
	struct device *class_dev = NULL;
	int ret;

	udev = interface_to_usbdev(intf);

	/* select CAN interface */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0) {
		dev_info(&intf->dev, "skipping non-CAN interface %d\n",
			 intf->cur_altsetting->desc.bInterfaceNumber);
		return -ENODEV;
	}

	/* Allocate can/net device */
	ndev = alloc_candev(sizeof(struct ch9433), CH9433_TX_AGG_NUM);
	if (!ndev) {
		dev_err(&intf->dev, "Couldn't alloc candev\n");
		return -ENOMEM;
	}

	ndev->netdev_ops = &ch9433_netdev_ops;
	ndev->ethtool_ops = &ch9433_ethtool_ops;
	ndev->flags |= IFF_ECHO;
	dev = netdev_priv(ndev);
	dev->can.bittiming_const = &ch9433_bittiming_const;
	dev->can.do_set_mode = ch9433_do_set_mode;
	dev->can.clock.freq = CH9433_CLK_FREQ;
	dev->can.ctrlmode_supported = CAN_CTRLMODE_ONE_SHOT |
				      CAN_CTRLMODE_LOOPBACK |
				      CAN_CTRLMODE_LISTENONLY;
	dev->ndev = ndev;
	dev->udev = usb_get_dev(udev);
	dev->intf = intf;
	dev->pipe_ctrl_in = usb_rcvbulkpipe(udev, CH9433_CTRL_PIPE);
	dev->pipe_ctrl_out = usb_sndbulkpipe(udev, CH9433_CTRL_PIPE);
	dev->pipe_in = usb_rcvbulkpipe(udev, CH9433_BULK_IN_PIPE);
	dev->pipe_out = usb_sndbulkpipe(udev, CH9433_BULK_OUT_PIPE);
	dev->n_tx_urbs = CH9433_TX_AGG_NUM;
	dev->n_rx_urbs = CH9433_RX_AGG_NUM;
	dev->tx_urb_size = CH9433_BULK_OUT_LEN;
	dev->rx_urb_size = CH9433_BULK_IN_LEN;
	dev->minor = -1;
	atomic_set(&dev->refcnt, 1);
	mutex_init(&dev->ctrl_lock);
	mutex_init(&dev->state_lock);

	usb_set_intfdata(intf, dev);
	SET_NETDEV_DEV(ndev, &intf->dev);

	INIT_WORK(&dev->rx_status_work, ch9433_can_err);
	INIT_WORK(&dev->recover_work, ch9433_recover_work_handler);

	dev->can.state = CAN_STATE_STOPPED;
	dev->minor = ch9433_alloc_minor(dev);
	if (dev->minor < 0) {
		ret = dev->minor;
		goto out_clear_intfdata;
	}

	ret = ch9433_iofunc_set(dev, 0x03, 3, 1);
	if (ret)
		goto out_release_minor;

	ret = ch9433_hw_sleep(dev, false);
	if (ret)
		goto out_release_minor;

	/* register the device */
	ret = register_candev(ndev);
	if (ret) {
		netdev_err(ndev, "couldn't register CAN device: %d\n", ret);
		goto out_release_minor;
	}

	class_dev = device_create(ch9433_io_class, &intf->dev,
				  MKDEV(MAJOR(devt), dev->minor), dev,
				  "ch9433_iodev%d", dev->minor);
	if (IS_ERR(class_dev)) {
		ret = PTR_ERR(class_dev);
		dev_err(&dev->udev->dev, "Could not create device node: %d\n",
			ret);
		goto out_unregister_candev;
	}
	dev_info(&dev->udev->dev, "iodev%d: character device\n", dev->minor);

	dev_info(&dev->udev->dev, "device probe, driver version: %s\n",
		 VERSION_DESC);

	return 0;

out_unregister_candev:
	unregister_candev(ndev);
out_release_minor:
	ch9433_release_minor(dev);
out_clear_intfdata:
	usb_set_intfdata(intf, NULL);
	ch9433_put_dev(dev);
	return ret;
}

static void ch9433_disconnect(struct usb_interface *intf)
{
	struct ch9433 *dev = usb_get_intfdata(intf);

	if (!dev)
		return;

	usb_set_intfdata(intf, NULL);
	ch9433_mark_unplugged(dev);
	ch9433_release_minor(dev);
	device_destroy(ch9433_io_class, MKDEV(MAJOR(devt), dev->minor));
	unregister_candev(dev->ndev);

	ch9433_cancel_deferred_work(dev);

	/* Wait for any ctrl_lock-protected transfer started before unplug. */
	mutex_lock(&dev->ctrl_lock);
	mutex_unlock(&dev->ctrl_lock);

	ch9433_put_dev(dev);
}

static const struct usb_device_id ch9433_ids[] = {
	{
		USB_DEVICE_INTERFACE_NUMBER(0x1a86, 0x5610, 0),
	},

	{},
};

MODULE_DEVICE_TABLE(usb, ch9433_ids);

static struct usb_driver ch9433_driver = {
	.name = DRVNAME_CH9433,
	.id_table = ch9433_ids,
	.probe = ch9433_probe,
	.disconnect = ch9433_disconnect,
};

static int ch9433_ioctl_xfer(struct ch9433 *dev,
			     struct ch9433_raw_xfer_req *req)
{
	void __user *tx_user = u64_to_user_ptr(req->tx_buf);
	void __user *rx_user = u64_to_user_ptr(req->rx_buf);
	u8 *tx_buffer = NULL;
	u8 *rx_buffer = NULL;
	int ret = 0, actual_len = 0;

	req->rx_actual_len = 0;
	req->reserved = 0;

	if (!req->tx_len || req->tx_len > CH9433_IOCTL_MAX_FRAME ||
	    req->rx_len > CH9433_IOCTL_MAX_FRAME)
		return -EINVAL;

	if (!tx_user)
		return -EINVAL;

	tx_buffer = kmalloc(req->tx_len, GFP_KERNEL);
	if (!tx_buffer)
		return -ENOMEM;

	ret = copy_from_user(tx_buffer, tx_user, req->tx_len);
	if (ret) {
		dev_err(&dev->udev->dev, "%s copy_from_user error. retval:%d\n",
			__func__, ret);
		ret = -EFAULT;
		goto out;
	}

	if (req->rx_len) {
		if (!rx_user) {
			ret = -EINVAL;
			goto out;
		}

		rx_buffer = kmalloc(req->rx_len, GFP_KERNEL);
		if (!rx_buffer) {
			ret = -ENOMEM;
			goto out;
		}
	}

	mutex_lock(&dev->ctrl_lock);
	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags)) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = usb_bulk_msg(dev->udev, dev->pipe_ctrl_out, tx_buffer,
			   req->tx_len, &actual_len,
			   CH9433_USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		dev_dbg(&dev->udev->dev, "%s ioctl xfer write failed: %d\n",
			__func__, ret);
		if (ret == -EPIPE)
			ch9433_clear_halt(dev, dev->pipe_ctrl_out);
		goto unlock;
	}

	if (actual_len != req->tx_len) {
		ret = -EIO;
		goto unlock;
	}

	if (req->rx_len) {
		actual_len = 0;
		ret = usb_bulk_msg(dev->udev, dev->pipe_ctrl_in, rx_buffer,
				   req->rx_len, &actual_len,
				   CH9433_USB_CTRL_GET_TIMEOUT);
		if (ret < 0) {
			dev_dbg(&dev->udev->dev,
				"%s ioctl xfer read failed: %d\n", __func__,
				ret);
			if (ret == -EPIPE)
				ch9433_clear_halt(dev, dev->pipe_ctrl_in);
			goto unlock;
		}

		req->rx_actual_len = actual_len;
	}

unlock:
	mutex_unlock(&dev->ctrl_lock);

	if (!ret && req->rx_len) {
		ret = copy_to_user(rx_user, rx_buffer, req->rx_actual_len);
		if (ret) {
			dev_err(&dev->udev->dev,
				"%s copy_to_user error. ret:%d\n", __func__,
				ret);
			ret = -EFAULT;
		}
	}
out:
	kfree(tx_buffer);
	kfree(rx_buffer);
	return ret;
}

static int __ch9433_io_ioctl(struct ch9433 *dev, unsigned int cmd,
			     unsigned long arg)
{
	struct ch9433_raw_xfer_req req_xfer;
	struct ch9433_reg_req reg_req;
	u32 reg_val;
	int ret;

	switch (cmd) {
	case IOCTL_CTRL_XFER:
		if (copy_from_user(&req_xfer, (void __user *)arg,
				   sizeof(req_xfer)))
			return -EFAULT;
		ret = ch9433_ioctl_xfer(dev, &req_xfer);
		if (!ret && copy_to_user((void __user *)arg, &req_xfer,
					 sizeof(req_xfer)))
			ret = -EFAULT;
		break;
	case IOCTL_CTRL_READ_REG:
		if (copy_from_user(&reg_req, (void __user *)arg,
				   sizeof(reg_req)))
			return -EFAULT;
		ret = ch9433_read_reg(dev, reg_req.reg, &reg_val);
		if (!ret && copy_to_user(u64_to_user_ptr(reg_req.reg_val),
					 &reg_val, sizeof(reg_val)))
			ret = -EFAULT;
		break;
	case IOCTL_CTRL_WRITE_REG:
		if (copy_from_user(&reg_req, (void __user *)arg,
				   sizeof(reg_req)))
			return -EFAULT;
		if (copy_from_user(&reg_val, u64_to_user_ptr(reg_req.reg_val),
				   sizeof(reg_val)))
			return -EFAULT;
		ret = ch9433_write_reg(dev, reg_req.reg, reg_val);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static struct ch9433 *ch9433_get_by_index(unsigned int index)
{
	struct ch9433 *dev;

	if (index >= CH9433_MAX_NUM)
		return NULL;

	mutex_lock(&ch9433_minors_lock);
	dev = ch9433_table[index];
	if (dev &&
	    (test_bit(CH9433_DEV_UNPLUG, &dev->flags) || !ch9433_get_dev(dev)))
		dev = NULL;
	mutex_unlock(&ch9433_minors_lock);

	return dev;
}

static int ch9433_io_open(struct inode *inode, struct file *fp)
{
	unsigned int minor = iminor(inode);
	struct ch9433 *dev = ch9433_get_by_index(minor);

	if (!dev)
		return -ENODEV;

	fp->private_data = dev;

	dev_dbg(&dev->udev->dev, "%s\n", __func__);

	return 0;
}

static int ch9433_io_release(struct inode *inode, struct file *fp)
{
	struct ch9433 *dev = fp->private_data;

	if (dev && !test_bit(CH9433_DEV_UNPLUG, &dev->flags))
		dev_dbg(&dev->udev->dev, "%s\n", __func__);

	fp->private_data = NULL;
	if (dev)
		ch9433_put_dev(dev);

	return 0;
}

static long ch9433_io_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct ch9433 *dev = file->private_data;
	int retval;

	if (!dev)
		return -ENODEV;
	if (test_bit(CH9433_DEV_UNPLUG, &dev->flags))
		return -ENODEV;

	retval = __ch9433_io_ioctl(dev, cmd, arg);

	return retval;
}

#ifdef CONFIG_COMPAT
static long ch9433_io_compat_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	return ch9433_io_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations ch9433_io_fops = {
	.owner = THIS_MODULE,
	.open = ch9433_io_open,
	.release = ch9433_io_release,
	.unlocked_ioctl = ch9433_io_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ch9433_io_compat_ioctl,
#endif
};

static int __init ch9433_init(void)
{
	int ret;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0))
	ch9433_io_class = class_create(THIS_MODULE, "ch9433_io_class");
#else
	ch9433_io_class = class_create("ch9433_io_class");
#endif
	if (IS_ERR(ch9433_io_class)) {
		return PTR_ERR(ch9433_io_class);
	}

	ret = alloc_chrdev_region(&devt, 0, CH9433_MAX_NUM,
				  CH9433_GPIODRV_NAME);
	if (ret)
		goto destroy_class;

	ch9433_major = MAJOR(devt);
	cdev_init(&ch9433_cdev, &ch9433_io_fops);
	ret = cdev_add(&ch9433_cdev, MKDEV(ch9433_major, 0), CH9433_MAX_NUM);
	if (ret)
		goto unregister_chrdev;

	ret = usb_register(&ch9433_driver);
	if (ret)
		goto del_cdev;

	return 0;

del_cdev:
	cdev_del(&ch9433_cdev);
unregister_chrdev:
	unregister_chrdev_region(MKDEV(ch9433_major, 0), CH9433_MAX_NUM);
destroy_class:
	class_destroy(ch9433_io_class);

	return ret;
}

static void __exit ch9433_exit(void)
{
	usb_deregister(&ch9433_driver);
	cdev_del(&ch9433_cdev);
	unregister_chrdev_region(MKDEV(ch9433_major, 0), CH9433_MAX_NUM);
	class_destroy(ch9433_io_class);
}

module_init(ch9433_init);
module_exit(ch9433_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
