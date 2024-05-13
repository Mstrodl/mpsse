// SPDX-License-Identifier: GPL-2.0
/*
 * FTDI FT2232H MPSSE Driver
 *
 * Copyright (C) 2017 - 2018 DENX Software Engineering
 * Anatolij Gustschin <agust@denx.de>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb.h>
#include "mpsse.h"
#include <linux/version.h>
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/kobject.h>
#include <linux/kdev_t.h>

#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>

int usb_wait_msec = 0;
module_param(usb_wait_msec, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(usb_wait_msec, "Wait after USB transfer in msec");

struct mpsse_gpio_port {
	u8	states;
	u8	directions;
};

struct mpsse_priv {
	char				*name;
	struct usb_interface		*intf;
	struct usb_device		*udev;
	struct mutex			io_mutex; /* sync I/O with disconnect */
	struct mutex			ops_mutex;
	int				bitbang_enabled;
	int				id;
	int				index;
	u8				bulk_in;
	u8				bulk_out;
	size_t				bulk_in_sz;
	void				*bulk_in_buf;

	const struct usb_device_id	*usb_dev_id;
	struct mpsse_info		*info;
	struct platform_device		*fifo_pdev;
	struct platform_device		*spi_pdev;
	struct gpiod_lookup_table	*lookup_fifo;
	struct gpiod_lookup_table	*lookup_cs;

	u8				pinbuf[4];

	struct gpio_chip		mpsse_gpio;

	struct gpio_desc		*ce_gpio;
	int				old_value;
	struct mpsse_gpio_port		gpio_ports[2]; /* 0 = low, 1 = high */
	u8				tx_buf[4];
	unsigned int			offset;
	int				num;

	const struct cpumask		*aff_mask;

	int		ftmodel;
	int		numgpio;
	u8		eeprom[FTDI_MAX_EEPROM_SIZE];
};

/* Device info struct used for device specific init. */
struct mpsse_info {
  int (*probe)(struct usb_interface *intf, const void *plat_data);
  int (*remove)(struct usb_interface *intf);
  const void *plat_data; /* optional, passed to probe() */
  int			ftmodel;
  int 		numgpio;
};

dev_t dev = 0;
struct kobject *kobj_ref;

static DEFINE_IDA(ftdi_devid_ida);

static int ftdi_read_eeprom(struct mpsse_priv *priv);

/* Use baudrate calculation borrowed from libftdi */
static int ftdi_to_clkbits(int baudrate, unsigned int clk, int clk_div,
			   unsigned long *encoded_divisor)
{
	static const char frac_code[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
	int best_baud = 0;
	int div, best_div;

	if (baudrate >= clk / clk_div) {
		*encoded_divisor = 0;
		best_baud = clk / clk_div;
	} else if (baudrate >= clk / (clk_div + clk_div / 2)) {
		*encoded_divisor = 1;
		best_baud = clk / (clk_div + clk_div / 2);
	} else if (baudrate >= clk / (2 * clk_div)) {
		*encoded_divisor = 2;
		best_baud = clk / (2 * clk_div);
	} else {
		/*
		 * Divide by 16 to have 3 fractional bits and
		 * one bit for rounding
		 */
		div = clk * 16 / clk_div / baudrate;
		if (div & 1)	/* Decide if to round up or down */
			best_div = div / 2 + 1;
		else
			best_div = div / 2;
		if (best_div > 0x20000)
			best_div = 0x1ffff;
		best_baud = clk * 16 / clk_div / best_div;
		if (best_baud & 1)	/* Decide if to round up or down */
			best_baud = best_baud / 2 + 1;
		else
			best_baud = best_baud / 2;
		*encoded_divisor = (best_div >> 3) |
			(frac_code[best_div & 0x7] << 14);
	}
	return best_baud;
}

#define H_CLK	120000000
#define C_CLK	48000000
static int ftdi_convert_baudrate(struct mpsse_priv *priv, int baud,
				 u16 *value, u16 *index)
{
	unsigned long encoded_divisor = 0;
	int best_baud = 0;

	if (baud <= 0)
		return -EINVAL;

	/*
	 * On H Devices, use 12000000 baudrate when possible.
	 * We have a 14 bit divisor, a 1 bit divisor switch (10 or 16),
	 * three fractional bits and a 120 MHz clock. Assume AN_120
	 * "Sub-integer divisors between 0 and 2 are not allowed" holds
	 * for DIV/10 CLK too, so /1, /1.5 and /2 can be handled the same
	 */
	if (baud * 10 > H_CLK / 0x3fff) {
		best_baud = ftdi_to_clkbits(baud, H_CLK, 10, &encoded_divisor);
		encoded_divisor |= 0x20000;	/* switch on CLK/10 */
	} else {
		best_baud = ftdi_to_clkbits(baud, C_CLK, 16, &encoded_divisor);
	}

	if (best_baud <= 0) {
		pr_err("Invalid baudrate: %d\n", best_baud);
		return -EINVAL;
	}

	/* Check within tolerance (about 5%) */
	if ((best_baud * 2 < baud) ||
			(best_baud < baud
			 ? (best_baud * 21 < baud * 20)
			 : (baud * 21 < best_baud * 20))) {
		pr_err("Unsupported baudrate.\n");
		return -EINVAL;
	}

	/* Split into "value" and "index" values */
	*value = (u16)(encoded_divisor & 0xffff);
	*index = (u16)(((encoded_divisor >> 8) & 0xff00) | priv->index);

	dev_dbg(&priv->intf->dev, "best baud %d, v/i: %d, %d\n",
		best_baud, *value, *index);
	return best_baud;
}

/*
 * ftdi_ctrl_xfer - FTDI control endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for control transfer
 *
 * Return:
 * Return: If successful, the number of bytes transferred. Otherwise,
 * a negative error number.
 */
static int ftdi_ctrl_xfer(struct usb_interface *intf, struct ctrl_desc *desc)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (!desc->data && desc->size)
		desc->data = priv->bulk_in_buf;

	if (desc->dir_out)
		pipe = usb_sndctrlpipe(udev, 0);
	else
		pipe = usb_rcvctrlpipe(udev, 0);

	ret = usb_control_msg(udev, pipe, desc->request, desc->requesttype,
			desc->value, desc->index, desc->data, desc->size,
			desc->timeout);
	if (ret < 0)
		dev_dbg(&udev->dev, "ctrl msg failed: %d\n", ret);
exit:
	mutex_unlock(&priv->io_mutex);
	return ret;
}

/*
 * ftdi_bulk_xfer - FTDI bulk endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for bulk-in or bulk-out transfer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number. The number of
 * actual bytes transferred will be stored in the @desc->act_len field
 * of the descriptor struct.
 */
static int ftdi_bulk_xfer(struct usb_interface *intf, struct bulk_desc *desc)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (desc->dir_out)
		pipe = usb_sndbulkpipe(udev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe(udev, priv->bulk_in);

	ret = usb_bulk_msg(udev, pipe, desc->data, desc->len,
			   &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg(&udev->dev, "bulk msg failed: %d\n", ret);

exit:
	mutex_unlock(&priv->io_mutex);
	if (usb_wait_msec > 0) {
		usleep_range(usb_wait_msec * 1000, usb_wait_msec * 1000 + 1000);
	}
	return ret;
}

/*
 * ftdi_set_baudrate - set the device baud rate
 * @intf: USB interface pointer
 * @baudrate: baud rate value to set
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_baudrate(struct usb_interface *intf, int baudrate)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	u16 index, value;
	int ret;

	if (priv->bitbang_enabled)
		baudrate *= 4;

	ret = ftdi_convert_baudrate(priv, baudrate, &value, &index);
	if (ret < 0)
		return ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_BAUDRATE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = value;
	desc.index = index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set baudrate: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_set_clock(struct usb_interface *intf, int clock_freq_hz)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	uint8_t *buf = priv->tx_buf;
	uint32_t value = 0;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	switch (priv->usb_dev_id->idProduct) {
	case 0x6001: /* FT232 */
		if (clock_freq_hz >= FTDI_CLK_6MHZ)
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		break;

	case 0x6010: /* FT2232 */
	case 0x6011: /* FT4232 */
	case 0x6041: /* FT4233 */
	case 0x6014: /* FT232H */
	case 0x0146: /* GW16146 */
		desc.len = 1;
		if (clock_freq_hz <= (FTDI_CLK_30MHZ/65535)) {
			buf[0] = EN_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret)
				return ret;
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		} else {
			buf[0] = DIS_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret)
				return ret;
			value = (FTDI_CLK_30MHZ/clock_freq_hz) - 1;
		}

		break;
	}

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(value & 0xff);
	buf[2] = (uint8_t)(value >> 8);
	desc.act_len = 0;
	desc.len = 3;
	ret = ftdi_bulk_xfer(intf, &desc);

	return ret;
}

/*
 * ftdi_set_latency - set the device latency (Bulk-In interval)
 * @intf: USB interface pointer
 * @latency_msec: latency value to set, 1-255
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_latency(struct usb_interface *intf, int latency_msec)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_LATENCY_TIMER_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = latency_msec;
	desc.index = priv->index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set latency: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * ftdi_read_data - read from FTDI bulk-in endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to read
 *
 * The two modem status bytes transferred in every read will
 * be removed and will not appear in the data buffer.
 *
 * Return:
 * If successful, the number of data bytes received (can be 0).
 * Otherwise, a negative error number.
 */
static int ftdi_read_data(struct usb_interface *intf, void *buf, size_t len)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;
	/* Device sends 2 additional status bytes, read at least len + 2 */
	desc.len = min_t(size_t, len + 2, priv->bulk_in_sz);
	desc.timeout = FTDI_USB_READ_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret)
		return ret;

	/* Only status bytes and no data? */
	if (desc.act_len <= 2)
		return 0;

	/* Skip first two status bytes */
	ret = desc.act_len - 2;
	if (ret > len)
		ret = len;
	memcpy(buf, desc.data + 2, ret);
	return ret;
}

/*
 * ftdi_write_data - write to FTDI bulk-out endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to send
 *
 * Return:
 * If successful, the number of bytes transferred. Otherwise a negative
 * error number.
 */
static int ftdi_write_data(struct usb_interface *intf,
			   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.len = len;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	return desc.act_len;
}

/*
 * ftdi_set_bitmode - configure bitbang mode
 * @intf: USB interface pointer
 * @bitmask: line configuration bitmask
 * @mode: bitbang mode to set
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_bitmode(struct usb_interface *intf, unsigned char bitmask,
			    unsigned char mode)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.data = NULL;
	desc.request = FTDI_SIO_SET_BITMODE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.index = priv->index;
	desc.value = (mode << 8) | bitmask;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	switch (mode) {
	case BITMODE_BITBANG:
	case BITMODE_CBUS:
	case BITMODE_SYNCBB:
	case BITMODE_SYNCFF:
		priv->bitbang_enabled = 1;
		break;
	case BITMODE_MPSSE:
	case BITMODE_RESET:
	default:
		priv->bitbang_enabled = 0;
		break;
	}

	return 0;
}

/*
 * ftdi_disable_bitbang - disable bitbang mode
 * @intf: USB interface pointer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int ftdi_disable_bitbang(struct usb_interface *intf)
{
	int ret;

	ret = ftdi_set_bitmode(intf, 0, BITMODE_RESET);
	if (ret < 0) {
		dev_dbg(&intf->dev, "disable bitbang failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_read_eeprom(struct mpsse_priv *priv)
{
	struct ctrl_desc desc;
	unsigned int i;
	int ret;

	desc.dir_out = false;
	desc.request = FTDI_SIO_READ_EEPROM_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN;
	desc.value = 0;
	desc.size = 2;
	desc.timeout = USB_CTRL_GET_TIMEOUT;

	for (i = 0; i < FTDI_MAX_EEPROM_SIZE / 2; i++) {
		desc.index = i;
		desc.data = &priv->eeprom[i * 2];

		ret = ftdi_ctrl_xfer(priv->intf, &desc);
		if (ret < 0) {
			dev_dbg(&priv->intf->dev, "EEPROM read failed: %d\n",
				ret);
			return ret;
		}
	}


	print_hex_dump(KERN_DEBUG, "EEPROM: ", DUMP_PREFIX_OFFSET, 16, 1, priv->eeprom, sizeof(priv->eeprom), 1);

	return 0;
}

/*
 * MPSSE CS and GPIO-L/-H support
 */
#define SET_BITS_LOW	0x80
#define GET_BITS_LOW	0x81
#define SET_BITS_HIGH	0x82
#define GET_BITS_HIGH	0x83

/* Upper bit tells us which port within the MPSSE we want */
#define PORT_ID_BIT (3)
/* Lower 3 bits make up the offset within the high or low byte */
#define OFFSET_MASK (0b111)

struct ftdi_mpsse_ops {
	u8	get_cmd;
	u8	set_cmd;
};

static struct ftdi_mpsse_ops ftdi_mpsse_ops[2] = {
	{GET_BITS_LOW, SET_BITS_LOW},
	{GET_BITS_HIGH, SET_BITS_HIGH},
};

static int ftdi_mpsse_get_port_pins(struct mpsse_priv *priv, int port_id)
{
	struct device *dev = &priv->intf->dev;
	int ret, timeout = 25;

	priv->tx_buf[0] = ftdi_mpsse_ops[port_id].get_cmd;

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 1);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Writing port pins cmd failed: %d\n",
				    ret);
		return ret;
	}

	priv->tx_buf[0] = 0;
	do {
		usleep_range(5000, 5200);
		ret = ftdi_read_data(priv->intf, priv->tx_buf, 1);
		timeout--;
		if (!timeout) {
			dev_err(dev, "Timeout when getting port pins\n");
			return -ETIMEDOUT;
		}
	} while (ret == 0);

	if (ret < 0)
		return ret;

	if (ret != 1)
		return -EINVAL;

	priv->gpio_ports[port_id].states = priv->tx_buf[0];

	return 0;
}

static int ftdi_mpsse_set_port_pins(struct mpsse_priv *priv, int port_id)
{
	struct device *dev = &priv->intf->dev;
	int ret;

	priv->tx_buf[0] = ftdi_mpsse_ops[port_id].set_cmd;
	priv->tx_buf[1] = priv->gpio_ports[port_id].states;
	priv->tx_buf[2] = priv->gpio_ports[port_id].directions;

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 3);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Failed to set GPIO pins: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_mpsse_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct mpsse_priv *priv = gpiochip_get_data(chip);
	int ret, val, port_id;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d\n", __func__, offset);

	port_id = (offset & BIT(PORT_ID_BIT)) >> PORT_ID_BIT;

	mutex_lock(&priv->ops_mutex);

	ret = ftdi_mpsse_get_port_pins(priv, port_id);
	if (ret < 0) {
		mutex_unlock(&priv->ops_mutex);
		return ret;
	}

	val = priv->gpio_ports[port_id].states & (offset & OFFSET_MASK);

	mutex_unlock(&priv->ops_mutex);

	return !!val;
}

static void ftdi_mpsse_gpio_set(struct gpio_chip *chip, unsigned int offset,
				int value)
{
	struct mpsse_priv *priv = gpiochip_get_data(chip);
	int port_id;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	port_id = (offset & BIT(PORT_ID_BIT)) >> PORT_ID_BIT;

	if (value) {
		priv->gpio_ports[port_id].states |= BIT(offset & OFFSET_MASK);
	} else {
		priv->gpio_ports[port_id].states &= ~(BIT(offset & OFFSET_MASK));
	}

	ftdi_mpsse_set_port_pins(priv, port_id);

	mutex_unlock(&priv->ops_mutex);
}

static int ftdi_mpsse_gpio_direction_input(struct gpio_chip *chip,
					   unsigned int offset)
{
	struct mpsse_priv *priv = gpiochip_get_data(chip);
	int port_id;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d\n", __func__, offset);

	mutex_lock(&priv->ops_mutex);

	port_id = (offset & BIT(PORT_ID_BIT)) >> PORT_ID_BIT;
	priv->gpio_ports[port_id].directions &= ~(BIT(offset));

	ret = ftdi_mpsse_set_port_pins(priv, port_id);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static int ftdi_mpsse_gpio_direction_output(struct gpio_chip *chip,
					    unsigned int offset, int value)
{
	struct mpsse_priv *priv = gpiochip_get_data(chip);
	int ret, port_id;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	port_id = (offset & BIT(PORT_ID_BIT)) >> PORT_ID_BIT;

	priv->gpio_ports[port_id].directions |= BIT(offset & OFFSET_MASK);
	if (value) {
		priv->gpio_ports[port_id].states |= BIT(offset & OFFSET_MASK);
	} else {
		priv->gpio_ports[port_id].states &= ~(BIT(offset & OFFSET_MASK));
	}

	ret = ftdi_mpsse_set_port_pins(priv, port_id);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

/* Prepare this MPSSE for use as a serial interface (e.g. SPI, I2C) */
static int ftdi_mpsse_cfg_bus_pins(struct usb_interface *intf,
				   u8 dir_bits, u8 value_bits)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	priv->gpio_ports[0].directions &= ~7;
	priv->gpio_ports[0].directions |= (dir_bits & 7);

	priv->gpio_ports[0].states &= ~7;
	priv->gpio_ports[0].states |= (value_bits & 7);

	ret = ftdi_mpsse_set_port_pins(priv, 0);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static int mpsse_add_mpsse_gpio(struct mpsse_priv *priv)
{
	struct device *dev = &priv->intf->dev;

	char *label;
	int MPSSE_GPIOS;
	int ret;
		
	ret = ftdi_read_eeprom(priv);
	if (ret < 0)
		return ret;

	MPSSE_GPIOS = mpsse_get_numgpio(priv->intf);

	int interfaceNum = priv->intf->cur_altsetting->desc.bInterfaceNumber;
	label = devm_kasprintf(dev, GFP_KERNEL, "ftdi-mpsse-gpio.%d.%d", interfaceNum, priv->id);
	if (!label)
		return -ENOMEM;

	priv->mpsse_gpio.label = label;
	priv->mpsse_gpio.parent = NULL;
	priv->mpsse_gpio.owner = THIS_MODULE;
	priv->mpsse_gpio.base = -1;
	priv->mpsse_gpio.ngpio = MPSSE_GPIOS;
	priv->mpsse_gpio.can_sleep = true;
	priv->mpsse_gpio.set = ftdi_mpsse_gpio_set;
	priv->mpsse_gpio.get = ftdi_mpsse_gpio_get;
	priv->mpsse_gpio.direction_input = ftdi_mpsse_gpio_direction_input;
	priv->mpsse_gpio.direction_output = ftdi_mpsse_gpio_direction_output;
	priv->mpsse_gpio.names = NULL;

	ret = ftdi_set_bitmode(priv->intf, 0x00, BITMODE_RESET);
	if (ret < 0) {
		dev_err(dev, "Failed to set reset mode\n");
		return ret;
	}

		
	ret = ftdi_set_bitmode(priv->intf, 0x00, BITMODE_MPSSE);
	if (ret < 0) {
		dev_err(dev, "Failed to set MPSSE mode\n");
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &priv->mpsse_gpio, priv);
	if (ret < 0) {
		dev_err(dev, "Failed to add MPSSE GPIO chip: %d\n", ret);
		return ret;
	}
	
	return 0;
}

static void ftdi_lock(struct usb_interface *intf)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->ops_mutex);
}

static void ftdi_unlock(struct usb_interface *intf)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);

	mutex_unlock(&priv->ops_mutex);
}

static const struct mpsse_ops mpsse_ops = {
	.ctrl_xfer = ftdi_ctrl_xfer,
	.bulk_xfer = ftdi_bulk_xfer,
	.read_data = ftdi_read_data,
	.write_data = ftdi_write_data,
	.lock = ftdi_lock,
	.unlock = ftdi_unlock,
	.set_bitmode = ftdi_set_bitmode,
	.set_baudrate = ftdi_set_baudrate,
	.disable_bitbang = ftdi_disable_bitbang,
	.cfg_bus_pins = ftdi_mpsse_cfg_bus_pins,
	.set_clock = ftdi_set_clock,
	.set_latency = ftdi_set_latency,
};

/*
 * FPGA config interface: PS-SPI via MPSSE
 */
#define SPI_INTF_DEVNAME	"spi-ftdi-mpsse"

// csn is actually on CS0 but ce is correct on 1 IE AD4 and IRQ on AD6
static struct dev_io_desc_data ftdi_spi_bus_dev_io[] = {
	{ "CS", 0, GPIO_ACTIVE_LOW },
// 	{ "rst", 1, GPIO_ACTIVE_HIGH },
// 	{ "csn", 2, GPIO_ACTIVE_LOW },
};

static const struct mpsse_spi_dev_data ftdi_spi_dev_data[] = {
	{
		.magic		= FTDI_MPSSE_IO_DESC_MAGIC,
		.desc		= ftdi_spi_bus_dev_io,
		.desc_len	= ARRAY_SIZE(ftdi_spi_bus_dev_io),
	},
};

static struct spi_board_info ftdi_spi_bus_info[] = {
	{
//		.modalias	= "yx240qv29",
//	.modalias	= "ili9341",
		.modalias	= "max6662",
//		 .modalias	= "sx1278",
//		.modalias	= "spi-nor",
//	.modalias	= "mcp2515",
//		.modalias	= "spi-petra",
//		.modalias	= "nrf24",
//	.modalias	= "ili9341",
		.mode		= SPI_MODE_0,
//		.mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
//		.max_speed_hz	= 4000000,
		.max_speed_hz	= 3000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data	= ftdi_spi_dev_data,
// 	.swnode	= &nrf24_node,		//changed from properties to swnode i dunno aroun kernel 5.15ish
//		.properties	= mcp2515_properties,
//	.swnode	=	&worldsemi_node,
//		.swnode = &sx1276_node,
		//	 .swnode = &w25q32_node,
//	.irq		 = 0,
	},
	{
//		 .modalias	= "spi-petra",		//use instead of spidev for spidev no-longer enumerates
		.modalias	= "spi-petra",
//		.modalias	= "spidev",
		.mode		= SPI_MODE_0,
//		.mode		= SPI_MODE_0 | SPI_LSB_FIRST | SPI_CS_HIGH,
		.max_speed_hz	= 30000000,
		.bus_num	= 0,
		.chip_select	= 4, // GPIOH0 at ACBUS0
	},

};

static const struct mpsse_spi_platform_data ftdi_spi_bus_plat_data = {
	.ops		= &mpsse_ops,
	.spi_info	= ftdi_spi_bus_info,
	.spi_info_len	= ARRAY_SIZE(ftdi_spi_bus_info),
};

static struct platform_device *mpsse_dev_register(struct mpsse_priv *priv,
						  const struct mpsse_spi_platform_data *pd)
{
	struct device *parent = &priv->intf->dev;
	struct platform_device *pdev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size, tbl_size;
	int i, ret;
		
	ret = mpsse_add_mpsse_gpio(priv);
	if (ret < 0) {
		goto err;
	}
		
	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;
	
	priv->spi_pdev = pdev;
		
	tbl_size = pd->spi_info_len + 1;
	lookup_size = sizeof(*lookup) + tbl_size * sizeof(struct gpiod_lookup);
	lookup = devm_kzalloc(parent, lookup_size, GFP_KERNEL);
	if (!lookup) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		dev_dbg(parent, "INFO: %s cs %d\n",
			pd->spi_info[i].modalias, pd->spi_info[i].chip_select);
	}


	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
	if (ret)
		goto err;

	pdev->id = priv->id;


	lookup->dev_id = devm_kasprintf(parent, GFP_KERNEL, "%s.%d",
					pdev->name, pdev->id);
	if (!lookup->dev_id) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < pd->spi_info_len; i++) {
		lookup->table[i].key = priv->mpsse_gpio.label;
		lookup->table[i].chip_hwnum = pd->spi_info[i].chip_select;
		lookup->table[i].idx = i;
		lookup->table[i].con_id = NULL;
		if (pd->spi_info[i].mode & SPI_CS_HIGH)
			lookup->table[i].flags = GPIO_ACTIVE_HIGH;
		else
			lookup->table[i].flags = GPIO_ACTIVE_LOW;
	}

	priv->lookup_cs = lookup;

	gpiod_add_lookup_table(priv->lookup_cs);


	ret = platform_device_add(pdev);
	if (ret < 0)
		goto err_add;

	dev_dbg(&pdev->dev, "%s done\n", __func__);
	return pdev;

err_add:
	gpiod_remove_lookup_table(priv->lookup_cs);
err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}

static int mpsse_spi_probe(struct usb_interface *intf,
				 const void *plat_data)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;
	struct platform_device *pdev;

	pdev = mpsse_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create MPSSE SPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;

	return 0;
}

static int mpsse_spi_remove(struct usb_interface *intf)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	gpiod_remove_lookup_table(priv->lookup_cs);
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct mpsse_info ftdi_spi_bus_intf_info = {
	.probe	= mpsse_spi_probe,
	.remove	= mpsse_spi_remove,
	.plat_data	= &ftdi_spi_bus_plat_data,
};

int mpsse_get_model(struct usb_interface *intf)
{
	return 2232;
}
EXPORT_SYMBOL_GPL(mpsse_get_model);

int mpsse_get_numgpio(struct usb_interface *intf)
{
	return 16;
}
EXPORT_SYMBOL_GPL(mpsse_get_numgpio);

static int mpsse_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct mpsse_priv *priv;
	struct device *dev = &intf->dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	const struct mpsse_info *info;
	unsigned int i;
	int ret = 0;
	int inf;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));
	inf = intf->cur_altsetting->desc.bInterfaceNumber;

	mpsse_get_model(intf);
	mpsse_get_numgpio(intf);

//	create_sysfs_attrs(intf);

	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(endpoint))
			priv->bulk_out = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			priv->bulk_in = endpoint->bEndpointAddress;
			priv->bulk_in_sz = usb_endpoint_maxp(endpoint);
		}
	}

	priv->usb_dev_id = id;
	priv->index = intf->cur_altsetting->desc.bInterfaceNumber + 1;
	priv->intf = intf;
	priv->info = (struct mpsse_info *)id->driver_info;

	info = priv->info;
	if (!info) {
		dev_err(dev, "Missing device specific driver info...\n");
		return -ENODEV;
	}

	mutex_init(&priv->io_mutex);
	mutex_init(&priv->ops_mutex);
	usb_set_intfdata(intf, priv);

	priv->bulk_in_buf = devm_kmalloc(dev, priv->bulk_in_sz, GFP_KERNEL);
	if (!priv->bulk_in_buf)
		return -ENOMEM;

	priv->udev = usb_get_dev(interface_to_usbdev(intf));

	priv->id = ida_simple_get(&ftdi_devid_ida, 0, 0, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;

	if (info->probe) {
		ret = info->probe(intf, info->plat_data);
		if (ret < 0)
			goto err;
		return 0;
	}

	ret = mpsse_add_mpsse_gpio(priv);
	if (!ret)
		return 0;

err:
	ida_simple_remove(&ftdi_devid_ida, priv->id);
	return ret;
}

static void mpsse_disconnect(struct usb_interface *intf)
{
	struct mpsse_priv *priv = usb_get_intfdata(intf);
	const struct mpsse_info *info;
		
	usleep_range(7000, 7200);
		
		
	info = (struct mpsse_info *)priv->usb_dev_id->driver_info;
	if (info && info->remove)
		info->remove(intf);


	gpiochip_remove(&priv->mpsse_gpio);
 
	mutex_lock(&priv->io_mutex);
	priv->intf = NULL;
	usb_set_intfdata(intf, NULL);
	mutex_unlock(&priv->io_mutex);

	usb_put_dev(priv->udev);
	ida_simple_remove(&ftdi_devid_ida, priv->id);
	mutex_destroy(&priv->io_mutex);
	mutex_destroy(&priv->ops_mutex);
	kfree (priv);
}


static struct usb_device_id mpsse_device_table[] = {
	{ USB_DEVICE(0x0c52, 0xa064),
		.driver_info = (kernel_ulong_t)&ftdi_spi_bus_intf_info },
	{}
};
MODULE_DEVICE_TABLE(usb, mpsse_device_table);

static struct usb_driver mpsse_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mpsse_device_table,
	.probe		= mpsse_probe,
	.disconnect	= mpsse_disconnect,
};

module_usb_driver(mpsse_driver);

MODULE_ALIAS("mpsse");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_DESCRIPTION("FTDI Multi-protocol Synchronous Serial Engine");
MODULE_LICENSE("GPL v2");
