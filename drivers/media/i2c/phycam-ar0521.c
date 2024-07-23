// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 * Copyright (C) 2021 PHYTEC Messtechnik GmbH
 * Author: Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 * Author: Stefan Riedmueller <s.riedmueller@phytec.de>
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include <linux/of_graph.h>

#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>

#define	AR0521_DATA_FORMAT_BITS			0x0112
#define		BIT_DATA_FMT_IN(n)		((n) << 8)
#define		BIT_DATA_FMT_OUT(n)		(n)
#define	AR0521_VT_PIX_CLK_DIV			0x0300
#define	AR0521_VT_SYS_CLK_DIV			0x0302
#define	AR0521_PRE_PLL_CLK_DIV			0x0304
#define		BIT_PLL_DIV2(n)			((n) << 8)
#define		BIT_PLL_DIV1(n)			(n)
#define	AR0521_PLL_MUL				0x0306
#define		BIT_PLL_MUL2(n)			((n) << 8)
#define		BIT_PLL_MUL1(n)			(n)
#define	AR0521_OP_PIX_CLK_DIV			0x0308
#define	AR0521_OP_SYS_CLK_DIV			0x030a
#define	AR0521_X_ADDR_START			0x0344
#define	AR0521_Y_ADDR_START			0x0346
#define	AR0521_X_ADRR_END			0x0348
#define	AR0521_Y_ADRR_END			0x034a
#define	AR0521_X_OUTPUT_SIZE			0x034c
#define	AR0521_Y_OUTPUT_SIZE			0x034e

#define	AR0521_MODEL_ID				0x3000
#define	AR0521_FRAME_LENGTH_LINES		0x300a
#define	AR0521_LINE_LENGTH_PCK			0x300c
#define	AR0521_COARSE_INT_TIME			0x3012
#define	AR0521_EXTRA_DELAY			0x3018
#define	AR0521_RESET_REGISTER			0x301a
#define		BIT_GROUPED_PARAM_HOLD		BIT(15)
#define		BIT_GAIN_INSERT_ALL		BIT(14)
#define		BIT_SMIA_SER_DIS		BIT(12)
#define		BIT_FORCED_PLL_ON		BIT(11)
#define		BIT_RESTART_BAD			BIT(10)
#define		BIT_MASK_BAD			BIT(9)
#define		BIT_GPI_EN			BIT(8)
#define		BIT_LOCK_REG			BIT(3)
#define		BIT_STREAM			BIT(2)
#define		BIT_RESTART			BIT(1)
#define		BIT_RESET			BIT(0)
#define	AR0521_DATA_PEDESTAL			0x301e
#define	AR0521_GPI_STATUS			0x3026
#define		BIT_TRIGGER_PIN_SEL(n)		((n) << 7)
#define		BIT_TRIGGER_PIN_SEL_MASK	GENMASK(9, 7)
#define	AR0521_FRAME_STATUS			0x303c
#define		BIT_PLL_LOCKED			BIT(3)
#define		BIT_FRAME_START_DURING_GPH	BIT(2)
#define		BIT_STANDBY_STATUS		BIT(1)
#define		BIT_FRAMESYNC			BIT(0)
#define	AR0521_READ_MODE			0x3040
#define		BIT_VERT_FLIP			BIT(15)
#define		BIT_HORIZ_MIRR			BIT(14)
#define		BIT_X_BIN_EN			BIT(11)
#define	AR0521_FLASH				0x3046
#define		BIT_XENON_FLASH			BIT(13)
#define		BIT_LED_FLASH			BIT(8)
#define		BIT_INVERT_FLASH		BIT(7)
#define	AR0521_FLASH_COUNT			0x3048
#define	AR0521_GREENR_GAIN			0x3056
#define	AR0521_BLUE_GAIN			0x3058
#define	AR0521_RED_GAIN				0x305a
#define	AR0521_GREENB_GAIN			0x305c
#define	AR0521_GLOBAL_GAIN			0x305e
#define		BIT_DIGITAL_GAIN(n)		((n) << 7)
#define		BIT_DIGITAL_GAIN_MASK		GENMASK(15, 7)
#define		BIT_ANA_COARSE_GAIN(n)		((n) << 4)
#define		BIT_ANA_COARSE_GAIN_MASK	GENMASK(6, 4)
#define		BIT_ANA_FINE_GAIN(n)		(n)
#define		BIT_ANA_FINE_GAIN_MASK		GENMASK(3, 0)
#define	AR0521_TEST_PATTERN			0x3070
#define	AR0521_TEST_DATA_RED			0x3072
#define	AR0521_TEST_DATA_GREENR			0x3074
#define	AR0521_TEST_DATA_BLUE			0x3076
#define	AR0521_TEST_DATA_GREENB			0x3078
#define	AR0521_X_ODD_INC			0x30a2
#define	AR0521_Y_ODD_INC			0x30a6

#define	AR0521_SLAVE_MODE_CTRL			0x3158
#define		BIT_VD_TRIG_NEW_FRAME		BIT(15)
#define		BIT_VD_TRIG_GRST		BIT(13)
#define		BIT_VD_NEW_FRAME_ONLY		BIT(11)
#define	AR0521_GLOBAL_SEQ_TRIGGER		0x315e
#define		BIT_GLOBAL_TRIGGER		BIT(0)
#define		BIT_SEQ_TRIGGER_GLOBAL_FLASH	BIT(2)
#define	AR0521_SERIAL_FORMAT			0x31ae
#define		BIT_TYPE(n)			((n) << 8)
#define		BIT_LANES(n)			(n)
#define	AR0521_MIPI_TIMING_0			0x31b4
#define		BIT_HS_PREP(n)			((n) << 12)
#define		BIT_HS_ZERO(n)			((n) << 6)
#define		BIT_HS_TRAIL(n)			((n) << 1)
#define	AR0521_MIPI_TIMING_1			0x31b6
#define		BIT_CLK_PREP(n)			((n) << 12)
#define		BIT_CLK_ZERO(n)			((n) << 5)
#define		BIT_CLK_TRAIL(n)		(n)
#define	AR0521_MIPI_TIMING_2			0x31b8
#define		BIT_BGAP(n)			((n) << 10)
#define		BIT_CLK_PRE(n)			((n) << 4)
#define		BIT_CLK_POST_MSBS(n)		(n)
#define	AR0521_MIPI_TIMING_3			0x31ba
#define		BIT_LPX(n)			((n) << 10)
#define		BIT_WAKEUP(n)			((n) << 3)
#define		BIT_CLK_POST(n)			(n)
#define	AR0521_MIPI_TIMING_4			0x31bc
#define		BIT_CONT_TX_CLK			BIT(15)
#define		BIT_VREG_MODE			BIT(13)
#define		BIT_HS_EXIT(n)			((n) << 7)
#define		BIT_INIT(n)			(n)
#define	AR0521_SER_CTRL_STAT			0x31c6
#define		BIT_FRAMER_TEST_MODE		BIT(7)
#define	AR0521_SERIAL_TEST			0x3066
#define	AR0521_PIX_DEF_ID			0x31e0
#define		BIT_PIX_DEF_2D_COUPLE_EN	BIT(10)
#define		BIT_PIX_DEF_2D_SINGLE_EN	BIT(9)
#define		BIT_PIX_DEF_2D_FUSE_EN		BIT(8)
#define		BIT_PIX_DEF_ID_LOC_CORR_EN	BIT(7)
#define		BIT_PIX_DEF_ID_EN		BIT(0)
#define	AR0521_CUSTOMER_REV			0x31fe

#define AR0521_MIPI_CNTRL			0x3354

#define AR0521_TP_NO_TESTPATTERN	0
#define AR0521_TP_SOLIDCOLOR		1
#define AR0521_TP_FULL_COLOR_BAR	2
#define AR0521_TP_FADE_TO_GRAY		3
#define AR0521_TP_PN9_LINK_INT		4
#define AR0521_TP_WALKING_ONES_10BIT	256
#define AR0521_TP_WALKING_ONES_8BIT	257

#define AR0521_TYPE_MIPI		2

#define AR0521_TEST_LANE_0		(0x1 << 6)
#define AR0521_TEST_LANE_1		(0x2 << 6)
#define AR0521_TEST_LANE_2		(0x4 << 6)
#define AR0521_TEST_LANE_3		(0x8 << 6)
#define AR0521_TEST_MODE_LP11		(0x1 << 2)

#define AR0521_MAX_LINK_FREQ		600000000ULL

#define AR0521_CSI2_DT_RAW8		0x2a
#define AR0521_CSI2_DT_RAW10		0x2b
#define AR0521_CSI2_DT_RAW12		0x2c

#define AR0521_CHIP_ID			0x0457
#define AR0522_CHIP_ID			0x1457
#define AR0521_DEF_WIDTH		2592
#define AR0521_DEF_HEIGHT		1944

#define AR0521_TRIGGER_OFF		0
#define AR0521_TRIGGER_GRR		1
#define AR0521_TRIGGER_ERS		2

#define AR0521_FREQ_MENU_8BIT		0
#define AR0521_FREQ_MENU_10BIT		1
#define AR0521_FREQ_MENU_12BIT		2


enum {
	V4L2_CID_USER_BASE_AR0521		= V4L2_CID_USER_BASE + 0x2500,

	V4L2_CID_X_BINNING_COL,
	V4L2_CID_X_EXTRA_BLANKING,

	V4L2_CID_X_DIGITAL_GAIN_RED,
	V4L2_CID_X_DIGITAL_GAIN_GREENR,
	V4L2_CID_X_DIGITAL_GAIN_BLUE,
	V4L2_CID_X_DIGITAL_GAIN_GREENB,

	V4L2_CID_X_DYNAMIC_PIXEL_CORRECTION,

	V4L2_CID_X_FLASH_INVERT,
	V4L2_CID_X_FLASH_XENON_WIDTH,
	V4L2_CID_X_TRIGGER_MODE,
	V4L2_CID_X_TRIGGER_PIN,
};

enum ar0521_model {
	AR0521_MODEL_UNKNOWN,
	AR0521_MODEL_COLOR,
	AR0521_MODEL_MONOCHROME,
};

struct ar0521_format {
	unsigned int code;
	unsigned int bpp;
};

static const struct ar0521_format ar0521_mono_formats[] = {
	{
		.code	= MEDIA_BUS_FMT_Y8_1X8,
		.bpp	= 8,
	}, {
		.code	= MEDIA_BUS_FMT_Y10_1X10,
		.bpp	= 10,
	}, {
		.code	= MEDIA_BUS_FMT_Y12_1X12,
		.bpp	= 12,
	},
};

static const struct ar0521_format ar0521_col_formats[] = {
	{
		.code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.bpp	= 8,
	}, {
		.code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp	= 10,
	}, {
		.code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.bpp	= 12,
	},
};

struct limit_range {
	unsigned long min;
	unsigned long max;
};

struct ar0521_sensor_limits {
	struct limit_range x;
	struct limit_range y;
	struct limit_range hlen;
	struct limit_range vlen;
	struct limit_range hblank;
	struct limit_range vblank;
	struct limit_range ext_clk;
};

struct ar0521_businfo {
	struct v4l2_mbus_config_mipi_csi2 buscfg;
	const s64 *link_freqs;

	u16 t_hs_prep;
	u16 t_hs_zero;
	u16 t_hs_trail;
	u16 t_clk_prep;
	u16 t_clk_zero;
	u16 t_clk_trail;
	u16 t_bgap;
	u16 t_clk_pre;
	u16 t_clk_post_msbs;
	u16 t_lpx;
	u16 t_wakeup;
	u16 t_clk_post;
	u16 t_hs_exit;
	u16 t_init;
	bool cont_tx_clk;
	bool vreg_mode;
};

struct ar0521_register {
	u16 reg;
	u16 val;
};

struct ar0521_pll_config {
	unsigned int pll2_div;
	unsigned int pll2_mul;
	unsigned int pll_div;
	unsigned int pll_mul;
	unsigned int vt_sys_div;
	unsigned int vt_pix_div;
	unsigned int op_sys_div;
	unsigned int op_pix_div;
	unsigned long vco_freq;
	unsigned long pix_freq;
	unsigned long ser_freq;
};

struct ar0521_gains {
	struct v4l2_ctrl *dig_ctrl;
	struct v4l2_ctrl *ana_ctrl;
	struct v4l2_ctrl *red_ctrl;
	struct v4l2_ctrl *greenb_ctrl;
	struct v4l2_ctrl *greenr_ctrl;
	struct v4l2_ctrl *blue_ctrl;
	unsigned int red;
	unsigned int greenb;
	unsigned int greenr;
	unsigned int blue;
	unsigned int min_ref;
};

struct ar0521 {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler ctrls;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	unsigned int bpp;
	unsigned int w_skip;
	unsigned int h_skip;
	unsigned int hlen;
	unsigned int vlen;

	struct ar0521_businfo info;
	struct ar0521_pll_config pll[4];
	struct ar0521_sensor_limits limits;
	enum ar0521_model model;

	const struct ar0521_format *formats;
	unsigned int num_fmts;

	struct v4l2_ctrl *exp_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct ar0521_gains gains;

#ifdef DEBUG
	struct dentry *debugfs_root;
	bool manual_pll;
#endif /* ifdef DEBUG */

	struct clk *extclk;
	struct gpio_desc *reset_gpio;

	struct mutex lock;

	int power_user;
	int trigger_pin;
	int trigger;
	bool is_streaming;
};

static inline struct ar0521 *to_ar0521(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar0521, subdev);
}

static inline unsigned int index_to_bpp(int index)
{
	return index * 2 + 8;
}

static inline int bpp_to_index(unsigned int bpp)
{
	return (bpp - 8) / 2;
}

static int ar0521_read(struct ar0521 *sensor, u16 reg, u16 *val)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	unsigned char reg_buf[2], read_buf[2];
	int ret;
	u16 result;
	struct i2c_msg xfer[] = {
		[0] = {
			.addr = i2c->addr,
			.flags = 0,
			.len = 2,
			.buf = reg_buf,
		},
		[1] = {
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = read_buf,
		},
	};

	reg_buf[0] = (reg >> 8) & 0xff;
	reg_buf[1] = reg & 0xff;

	ret = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret >= 0 && ret != ARRAY_SIZE(xfer))
		ret = -EIO;

	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to read i2c message (%d)\n", ret);
		return ret;
	}

	result = read_buf[0] << 8;
	result |= read_buf[1];

	*val = result;

	return 0;
}

static int ar0521_write(struct ar0521 *sensor, u16 reg, u16 val)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	unsigned char write_buf[4];
	int ret;
	struct i2c_msg xfer[] = {
		[0] = {
			.addr = i2c->addr,
			.flags = 0,
			.len = 4,
			.buf = write_buf,
		},
	};

	write_buf[0] = (reg >> 8) & 0xff;
	write_buf[1] = reg & 0xff;

	write_buf[2] = (val >> 8) & 0xff;
	write_buf[3] = val & 0xff;

	ret = i2c_transfer(i2c->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret >= 0 && ret != ARRAY_SIZE(xfer))
		ret = -EIO;

	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to write i2c message (%d)\n", ret);
		return ret;
	}

	dev_dbg(&i2c->dev, "Wrote i2c message 0x%02x at 0x%02x\n", val, reg);

	return 0;
}

static int ar0521_update_bits(struct ar0521 *sensor, u16 reg,
			      u16 mask, u16 val)
{
	u16 orig, tmp;
	int ret;

	ret = ar0521_read(sensor, reg, &orig);
	if (ret)
		return ret;

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		ret = ar0521_write(sensor, reg, tmp);

	return ret;
}

static int ar0521_set_bits(struct ar0521 *sensor, u16 reg, u16 val)
{
	return ar0521_update_bits(sensor, reg, val, val);
}

static int ar0521_clear_bits(struct ar0521 *sensor, u16 reg, u16 val)
{
	return ar0521_update_bits(sensor, reg, val, 0);
}

#ifdef DEBUG
static int ar0521_debugfs_init(struct ar0521 *sensor)
{
	if (!debugfs_initialized())
		return -ENODEV;

	sensor->debugfs_root = debugfs_create_dir(dev_name(sensor->subdev.dev),
						  NULL);
	if (!sensor->debugfs_root)
		return -ENOMEM;

	debugfs_create_bool("manual_pll", 0600, sensor->debugfs_root,
			    &sensor->manual_pll);
	debugfs_create_u32("pll2_div", 0600, sensor->debugfs_root,
			   &sensor->pll[3].pll2_div);
	debugfs_create_u32("pll_div", 0600, sensor->debugfs_root,
			   &sensor->pll[3].pll_div);
	debugfs_create_u32("pll2_mul", 0600, sensor->debugfs_root,
			   &sensor->pll[3].pll2_mul);
	debugfs_create_u32("pll_mul", 0600, sensor->debugfs_root,
			   &sensor->pll[3].pll_mul);
	debugfs_create_u32("vt_sys_div", 0600, sensor->debugfs_root,
			   &sensor->pll[3].vt_sys_div);
	debugfs_create_u32("vt_pix_div", 0600, sensor->debugfs_root,
			   &sensor->pll[3].vt_pix_div);
	debugfs_create_u32("op_sys_div", 0600, sensor->debugfs_root,
			   &sensor->pll[3].op_sys_div);
	debugfs_create_u32("op_pix_div", 0600, sensor->debugfs_root,
			   &sensor->pll[3].op_pix_div);

	return 0;
}

static void ar0521_debugfs_remove(struct ar0521 *sensor)
{
	debugfs_remove_recursive(sensor->debugfs_root);
}
#endif /* ifdef DEBUG */

static const struct ar0521_format *ar0521_find_format(struct ar0521 *sensor,
						      u32 code)
{
	int i;

	for (i = 0; i < sensor->num_fmts; i++)
		if (sensor->formats[i].code == code)
			return &sensor->formats[i];

	return &sensor->formats[sensor->num_fmts - 1];
}

static int ar0521_enter_standby(struct ar0521 *sensor)
{
	unsigned int timeout = 1000;
	int ret;
	u16 val;

	ret = ar0521_clear_bits(sensor, AR0521_RESET_REGISTER,
				BIT_STREAM);
	if (ret)
		return ret;

	while (timeout) {
		ar0521_read(sensor, AR0521_FRAME_STATUS, &val);

		if (val & BIT_STANDBY_STATUS)
			break;

		timeout--;

		if (timeout == 0) {
			v4l2_warn(&sensor->subdev,
				  "timeout while trying to enter standby\n");
			break;
		}

		usleep_range(2000, 3000);
	}

	msleep(100);
	ret = ar0521_set_bits(sensor, AR0521_RESET_REGISTER, BIT_SMIA_SER_DIS);
	if (ret)
		return ret;

	ret = ar0521_clear_bits(sensor, AR0521_SER_CTRL_STAT,
				BIT_FRAMER_TEST_MODE);

	return ret;
}

static int ar0521_mipi_enter_lp11(struct ar0521 *sensor)
{
	int ret;

	ret = ar0521_write(sensor, AR0521_SERIAL_TEST,
			   AR0521_TEST_MODE_LP11 |
			   AR0521_TEST_LANE_0 | AR0521_TEST_LANE_1 |
			   AR0521_TEST_LANE_2 | AR0521_TEST_LANE_3);
	if (ret)
		return ret;

	ret = ar0521_set_bits(sensor, AR0521_SER_CTRL_STAT,
			      BIT_FRAMER_TEST_MODE);
	if (ret)
		return ret;

	ret = ar0521_update_bits(sensor, AR0521_RESET_REGISTER,
				 BIT_STREAM | BIT_SMIA_SER_DIS,
				 BIT_STREAM);
	return ret;
}

static void ar0521_reset(struct ar0521 *sensor)
{
	unsigned long ext_freq = clk_get_rate(sensor->extclk);
	unsigned long ext_freq_mhz = ext_freq / 1000000;
	unsigned long wait_usecs;

	if (sensor->reset_gpio) {
		gpiod_set_value_cansleep(sensor->reset_gpio, 1);
		usleep_range(1000, 1100);
		gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	} else {
		ar0521_set_bits(sensor, AR0521_RESET_REGISTER, BIT_RESET);
	}

	wait_usecs = 160000 / ext_freq_mhz;
	usleep_range(wait_usecs, wait_usecs + 1000);
}

static int ar0521_unset_trigger(struct ar0521 *sensor)
{
	int ret;

	ret = ar0521_set_bits(sensor, AR0521_GPI_STATUS,
			      BIT_TRIGGER_PIN_SEL(7));
	if (ret)
		return ret;

	ret = ar0521_clear_bits(sensor, AR0521_SLAVE_MODE_CTRL,
				BIT_VD_TRIG_NEW_FRAME |
				BIT_VD_TRIG_GRST |
				BIT_VD_NEW_FRAME_ONLY);
	if (ret)
		return ret;

	ret = ar0521_clear_bits(sensor, AR0521_GLOBAL_SEQ_TRIGGER,
				BIT_GLOBAL_TRIGGER |
				BIT_SEQ_TRIGGER_GLOBAL_FLASH);
	if (ret)
		return ret;

	ret = ar0521_clear_bits(sensor, AR0521_RESET_REGISTER,
				BIT_GAIN_INSERT_ALL |
				BIT_FORCED_PLL_ON |
				BIT_GPI_EN);
	return ret;
}

static int ar0521_set_trigger_mode(struct ar0521 *sensor, int mode)
{
	int pin = sensor->trigger_pin;
	int ret;

	if (sensor->is_streaming) {
		ret = ar0521_clear_bits(sensor, AR0521_RESET_REGISTER,
					BIT_STREAM);
		if (ret)
			return ret;
	}

	switch (mode) {
	case AR0521_TRIGGER_OFF:
		ret = ar0521_unset_trigger(sensor);
		if (ret)
			return ret;
		break;
	case AR0521_TRIGGER_GRR:
		ret = ar0521_set_bits(sensor, AR0521_RESET_REGISTER,
				      BIT_GPI_EN);
		if (ret)
			return ret;

		ret = ar0521_update_bits(sensor, AR0521_GPI_STATUS,
					 BIT_TRIGGER_PIN_SEL_MASK,
					 BIT_TRIGGER_PIN_SEL(pin));
		if (ret)
			return ret;

		ret = ar0521_set_bits(sensor, AR0521_SLAVE_MODE_CTRL,
				      BIT_VD_TRIG_NEW_FRAME |
				      BIT_VD_NEW_FRAME_ONLY);
		if (ret)
			return ret;

		break;
	case AR0521_TRIGGER_ERS:
		ret = ar0521_set_bits(sensor, AR0521_RESET_REGISTER,
				      BIT_GAIN_INSERT_ALL |
				      BIT_FORCED_PLL_ON |
				      BIT_GPI_EN);
		if (ret)
			return ret;

		ret = ar0521_update_bits(sensor, AR0521_GPI_STATUS,
					 BIT_TRIGGER_PIN_SEL_MASK,
					 BIT_TRIGGER_PIN_SEL(pin));
		if (ret)
			return ret;

		ret = ar0521_set_bits(sensor, AR0521_SLAVE_MODE_CTRL,
				      BIT_VD_TRIG_NEW_FRAME |
				      BIT_VD_TRIG_GRST |
				      BIT_VD_NEW_FRAME_ONLY);
		if (ret)
			return ret;

		ret = ar0521_set_bits(sensor, AR0521_GLOBAL_SEQ_TRIGGER,
				      BIT_GLOBAL_TRIGGER |
				      BIT_SEQ_TRIGGER_GLOBAL_FLASH);
		if (ret)
			return ret;

		break;
	default:
		return -EINVAL;
	}

	if (sensor->is_streaming)
		return ar0521_set_bits(sensor, AR0521_RESET_REGISTER,
				       BIT_STREAM);
	else
		return 0;
}

static int ar0521_power_on(struct ar0521 *sensor)
{
	/* TODO: Enable power, clocks, etc... */
	return 0;
}

static void ar0521_power_off(struct ar0521 *sensor)
{
	/* TODO: Disable power, clocks, etc... */
}

/* V4L2 subdev core ops */
static int ar0521_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0521 *sensor = to_ar0521(sd);
	int ret = 0;

	dev_dbg(sd->dev, "%s on: %d\n", __func__, on);

	mutex_lock(&sensor->lock);

	if (on) {
		if (sensor->power_user > 0) {
			sensor->power_user++;
			goto out;
		}

		ret = ar0521_power_on(sensor);
		if (ret)
			goto out;

		/* Enable MIPI LP-11 test mode as required by e.g. i.MX 6 */
		if (!sensor->is_streaming) {
			ret = ar0521_mipi_enter_lp11(sensor);
			if (ret) {
				ar0521_power_off(sensor);
				goto out;
			}
		}

		sensor->power_user++;

	} else {
		sensor->power_user--;
		if (sensor->power_user < 0) {
			dev_err(sd->dev, "More s_power OFF than ON\n");
			ret = -EINVAL;
			goto out;
		}

		if (sensor->power_user == 0) {
			ar0521_enter_standby(sensor);
			ar0521_power_off(sensor);
		}
	}

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ar0521_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct ar0521 *sensor = to_ar0521(sd);

	dev_dbg(sd->dev, "%s\n", __func__);

	return ar0521_write(sensor, reg->reg, reg->val);
}

static int ar0521_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct ar0521 *sensor = to_ar0521(sd);

	dev_dbg(sd->dev, "%s\n", __func__);

	return ar0521_read(sensor, reg->reg, (u16 *)&reg->val);
}
#endif

static int ar0521_config_pll(struct ar0521 *sensor)
{
	int index;
	int ret;

	index = bpp_to_index(sensor->bpp);

#ifdef DEBUG
	if (sensor->manual_pll)
		index = 3;
#endif /* ifdef DEBUG */

	ret = ar0521_write(sensor, AR0521_VT_PIX_CLK_DIV,
			   sensor->pll[index].vt_pix_div);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_VT_SYS_CLK_DIV,
			   sensor->pll[index].vt_sys_div);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_PRE_PLL_CLK_DIV,
			   BIT_PLL_DIV2(sensor->pll[index].pll2_div) |
			   BIT_PLL_DIV1(sensor->pll[index].pll_div));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_PLL_MUL,
			   BIT_PLL_MUL2(sensor->pll[index].pll2_mul) |
			   BIT_PLL_MUL1(sensor->pll[index].pll_mul));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_OP_PIX_CLK_DIV,
			   sensor->pll[index].op_pix_div);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_OP_SYS_CLK_DIV,
			   sensor->pll[index].op_sys_div);
	if (ret)
		return ret;

	usleep_range(1000, 1500);

	return 0;
}

static int ar0521_config_frame(struct ar0521 *sensor)
{
	unsigned int height = sensor->fmt.height * sensor->h_skip;
	unsigned int width = sensor->fmt.width * sensor->w_skip;
	int ret;
	u16 x_end, y_end;

	ret = ar0521_write(sensor, AR0521_Y_ADDR_START, sensor->crop.top);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_X_ADDR_START, sensor->crop.left);
	if (ret)
		return ret;

	y_end = sensor->crop.top + height - 1;
	ret = ar0521_write(sensor, AR0521_Y_ADRR_END, y_end);
	if (ret)
		return ret;

	x_end = sensor->crop.left + width - 1;
	ret = ar0521_write(sensor, AR0521_X_ADRR_END, x_end);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_FRAME_LENGTH_LINES, sensor->vlen);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_LINE_LENGTH_PCK, sensor->hlen);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_X_OUTPUT_SIZE, sensor->fmt.width);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_Y_OUTPUT_SIZE, sensor->fmt.height);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_X_ODD_INC,
			   (sensor->w_skip << 1) - 1);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_Y_ODD_INC,
			   (sensor->h_skip << 1) - 1);

	return ret;
}

static int ar0521_config_mipi(struct ar0521 *sensor)
{
	int ret;
	u16 val;

	switch (sensor->bpp) {
	case 8:
		val = AR0521_CSI2_DT_RAW8;
		break;
	case 10:
		val = AR0521_CSI2_DT_RAW10;
		break;
	case 12:
		val = AR0521_CSI2_DT_RAW12;
		break;
	default:
		return -EINVAL;
	}

	ret = ar0521_write(sensor, AR0521_MIPI_CNTRL, val);
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_MIPI_TIMING_0,
			   BIT_HS_PREP(sensor->info.t_hs_prep) |
			   BIT_HS_ZERO(sensor->info.t_hs_zero) |
			   BIT_HS_TRAIL(sensor->info.t_hs_trail));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_MIPI_TIMING_1,
			   BIT_CLK_PREP(sensor->info.t_clk_prep) |
			   BIT_CLK_ZERO(sensor->info.t_clk_zero) |
			   BIT_CLK_TRAIL(sensor->info.t_clk_trail));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_MIPI_TIMING_2,
			   BIT_BGAP(sensor->info.t_bgap) |
			   BIT_CLK_PRE(sensor->info.t_clk_pre) |
			   BIT_CLK_POST_MSBS(sensor->info.t_clk_post_msbs));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_MIPI_TIMING_3,
			   BIT_LPX(sensor->info.t_lpx) |
			   BIT_WAKEUP(sensor->info.t_wakeup) |
			   BIT_CLK_POST(sensor->info.t_clk_post));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_MIPI_TIMING_4,
			   (sensor->info.cont_tx_clk ? BIT_CONT_TX_CLK : 0) |
			   (sensor->info.vreg_mode ? BIT_VREG_MODE : 0) |
			   BIT_HS_EXIT(sensor->info.t_hs_exit) |
			   BIT_INIT(sensor->info.t_init));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_DATA_FORMAT_BITS,
			   BIT_DATA_FMT_IN(sensor->bpp) |
			   BIT_DATA_FMT_OUT(sensor->bpp));
	if (ret)
		return ret;

	ret = ar0521_write(sensor, AR0521_SERIAL_FORMAT,
			   BIT_TYPE(AR0521_TYPE_MIPI) |
			   BIT_LANES(sensor->info.buscfg.num_data_lanes));
	if (ret)
		return ret;

	if (sensor->trigger) {
		ret = ar0521_set_trigger_mode(sensor, sensor->trigger);
		if (ret)
			return ret;
	}

	ret = ar0521_set_bits(sensor, AR0521_RESET_REGISTER,
			      BIT_STREAM | BIT_MASK_BAD);
	if (ret)
		return ret;

	ret = ar0521_clear_bits(sensor, AR0521_RESET_REGISTER,
				BIT_SMIA_SER_DIS);
	if (ret)
		return ret;

	ret = ar0521_update_bits(sensor, 0x3f20, BIT(3), 0);

	return ret;
}

static int ar0521_stream_on(struct ar0521 *sensor)
{
	int ret;

	ret = ar0521_enter_standby(sensor);
	if (ret)
		return ret;

	ret = ar0521_config_pll(sensor);
	if (ret)
		return ret;

	ret = ar0521_config_frame(sensor);
	if (ret)
		return ret;

	ret = ar0521_config_mipi(sensor);
	if (ret)
		return ret;

	sensor->is_streaming = true;
	return 0;
}

static int ar0521_stream_off(struct ar0521 *sensor)
{
	int ret;

	if (sensor->trigger) {
		ret = ar0521_unset_trigger(sensor);
		if (ret)
			dev_warn(sensor->subdev.dev,
				 "Failed to unset trigger mode\n");
	}

	ret = ar0521_enter_standby(sensor);

	sensor->is_streaming = false;
	return ret;
}

/* V4L2 subdev video ops */
static int ar0521_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar0521 *sensor = to_ar0521(sd);
	int ret = 0;

	dev_dbg(sd->dev, "%s enable: %d\n", __func__, enable);

	mutex_lock(&sensor->lock);

	if (enable && sensor->is_streaming) {
		ret = -EBUSY;
		goto out;
	}

	if (!enable && !sensor->is_streaming)
		goto out;

	if (enable)
		ret = ar0521_stream_on(sensor);
	else
		ret = ar0521_stream_off(sensor);

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ar0521_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct ar0521 *sensor = to_ar0521(sd);
	unsigned long pix_freq;
	int index;

	mutex_lock(&sensor->lock);

	index = bpp_to_index(sensor->bpp);
	pix_freq = sensor->pll[index].pix_freq;

	interval->interval.numerator = 10;
	interval->interval.denominator = div_u64(pix_freq * 10ULL,
						 sensor->vlen * sensor->hlen);

	mutex_unlock(&sensor->lock);

	return 0;
}

static struct v4l2_rect *ar0521_get_pad_crop(struct ar0521 *sensor,
					     struct v4l2_subdev_state *state,
					     unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&sensor->subdev, state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->crop;
	default:
		return NULL;
	}
}

static struct v4l2_mbus_framefmt *ar0521_get_pad_fmt(struct ar0521 *sensor,
					    struct v4l2_subdev_state *state,
					    unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&sensor->subdev, state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->fmt;
	default:
		return NULL;
	}
}

static unsigned int ar0521_find_skipfactor(unsigned int input,
					   unsigned int output)
{
	int i;

	/*
	 * We need to determine a matching supported power-of-two skip
	 * factor. If no exact match is found. the next bigger matching
	 * factor is returned.
	 * Supported factors are:
	 * No Skip
	 * Skip 2
	 * Skip 4
	 */

	for (i = 0; i < 2; i++)
		if ((input >> i) <= output)
			break;

	return (1 << i);
}

/* V4L2 subdev pad ops */
static int ar0521_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0521 *sensor = to_ar0521(sd);

	if (code->index < sensor->num_fmts) {
		code->code = sensor->formats[code->index].code;
		return 0;
	} else {
		return -EINVAL;
	}
}

static int ar0521_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0521 *sensor = to_ar0521(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	int ret = 0;

	mutex_lock(&sensor->lock);

	fmt = ar0521_get_pad_fmt(sensor, state, fse->pad, fse->which);
	crop = ar0521_get_pad_crop(sensor, state, fse->pad, fse->which);

	if (fse->index >= 4 || fse->code != fmt->code) {
		ret = -EINVAL;
		goto out;
	}

	fse->min_width = crop->width / (1u << fse->index);
	fse->max_width = fse->min_width;
	fse->min_height = crop->height / (1u << fse->index);
	fse->max_height = fse->min_height;

	if (fse->min_width <= 1 || fse->min_height <= 1)
		ret = -EINVAL;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static void ar0521_update_blankings(struct ar0521 *sensor)
{
	const struct ar0521_sensor_limits *limits = &sensor->limits;
	unsigned int width = sensor->fmt.width;
	unsigned int height = sensor->fmt.height;
	unsigned int hblank_min, hblank_max;
	unsigned int vblank_min, vblank_max;
	unsigned int hblank_value, hblank_default;
	unsigned int vblank_value, vblank_default;

	hblank_min = limits->hblank.min;
	if (width + limits->hblank.min < limits->hlen.min)
		hblank_min = limits->hlen.min - width;

	vblank_min = limits->vblank.min;
	if (height + limits->vblank.min < limits->vlen.min)
		vblank_min = limits->vlen.min - height;

	hblank_max = limits->hlen.max - width;
	vblank_max = limits->vlen.max - height;

	hblank_value = sensor->hblank_ctrl->cur.val;
	hblank_default = sensor->hblank_ctrl->default_value;

	vblank_value = sensor->vblank_ctrl->cur.val;
	vblank_default = sensor->vblank_ctrl->default_value;

	if (hblank_value < hblank_min)
		__v4l2_ctrl_s_ctrl(sensor->hblank_ctrl, hblank_min);

	if (hblank_value > hblank_max)
		__v4l2_ctrl_s_ctrl(sensor->hblank_ctrl, hblank_max);

	if (vblank_value < vblank_min)
		__v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, vblank_min);

	if (vblank_value > vblank_max)
		__v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, vblank_max);

	__v4l2_ctrl_modify_range(sensor->hblank_ctrl, hblank_min, hblank_max,
				 sensor->hblank_ctrl->step,
				 hblank_min);

	__v4l2_ctrl_modify_range(sensor->vblank_ctrl, vblank_min, vblank_max,
				 sensor->vblank_ctrl->step,
				 vblank_min);


	/*
	 * If the previous value was equal the previous default value, readjust
	 * the updated value to the new default value as well.
	 */
	if (hblank_value == hblank_default)
		__v4l2_ctrl_s_ctrl(sensor->hblank_ctrl, hblank_min);

	if (vblank_value == vblank_default)
		__v4l2_ctrl_s_ctrl(sensor->vblank_ctrl, vblank_min);
}

static int ar0521_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct ar0521 *sensor = to_ar0521(sd);
	const struct ar0521_format *sensor_format;
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	unsigned int width, height;
	unsigned int w_skip, h_skip;

	dev_dbg(sd->dev, "%s\n", __func__);

	mutex_lock(&sensor->lock);

	if (sensor->is_streaming &&
	    format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		mutex_unlock(&sensor->lock);
		return -EBUSY;
	}

	fmt = ar0521_get_pad_fmt(sensor, state, format->pad, format->which);
	crop = ar0521_get_pad_crop(sensor, state, format->pad,
				   V4L2_SUBDEV_FORMAT_ACTIVE);

	if (sensor->model == AR0521_MODEL_COLOR)
		fmt->colorspace = V4L2_COLORSPACE_RAW;
	else
		fmt->colorspace = V4L2_COLORSPACE_SRGB;

	fmt->field = V4L2_FIELD_NONE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);

	sensor_format = ar0521_find_format(sensor, format->format.code);
	fmt->code = sensor_format->code;

	width = clamp_t(unsigned int, format->format.width,
			1, crop->width);
	height = clamp_t(unsigned int, format->format.height,
			 1, crop->height);

	w_skip = ar0521_find_skipfactor(crop->width, width);
	h_skip = ar0521_find_skipfactor(crop->height, height);

	fmt->width = crop->width / w_skip;
	fmt->height = crop->height / h_skip;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		sensor->bpp = sensor_format->bpp;
		sensor->w_skip = w_skip;
		sensor->h_skip = h_skip;

		ar0521_update_blankings(sensor);
		sensor->hlen = fmt->width + sensor->hblank_ctrl->cur.val;
		sensor->vlen = fmt->height + sensor->vblank_ctrl->cur.val;
	}

	format->format = *fmt;

	mutex_unlock(&sensor->lock);
	return 0;
}

static int ar0521_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct ar0521 *sensor = to_ar0521(sd);
	struct v4l2_mbus_framefmt *fmt;

	dev_dbg(sd->dev, "%s\n", __func__);

	mutex_lock(&sensor->lock);

	fmt = ar0521_get_pad_fmt(sensor, state, format->pad, format->which);
	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int ar0521_group_param_hold(struct ar0521 *sensor)
{
	return ar0521_set_bits(sensor, AR0521_RESET_REGISTER,
			       BIT_GROUPED_PARAM_HOLD);
}

static int ar0521_group_param_release(struct ar0521 *sensor)
{
	return ar0521_clear_bits(sensor, AR0521_RESET_REGISTER,
				 BIT_GROUPED_PARAM_HOLD);
}

static int ar0521_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0521 *sensor = to_ar0521(sd);
	struct v4l2_rect *_crop;
	unsigned int max_w, max_h;
	int ret = 0;

	dev_dbg(sd->dev, "%s\n", __func__);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->is_streaming &&
	    (sel->r.width != sensor->crop.width ||
	     sel->r.height != sensor->crop.height)) {
		ret = -EBUSY;
		goto out;
	}

	_crop = ar0521_get_pad_crop(sensor, state, sel->pad, sel->which);

	max_w = sensor->limits.x.max - sensor->limits.x.min - 1;
	max_h = sensor->limits.y.max - sensor->limits.y.min - 1;

	_crop->top = min_t(unsigned int, ALIGN(sel->r.top, 2), max_h);
	_crop->left = min_t(unsigned int, ALIGN(sel->r.left, 2), max_w);
	_crop->width = min_t(unsigned int, sel->r.width, max_w - _crop->left);
	_crop->height = min_t(unsigned int, sel->r.height, max_h - _crop->top);

	if (sensor->is_streaming) {
		ret = ar0521_group_param_hold(sensor);
		if (ret)
			goto out;

		ret = ar0521_config_frame(sensor);
		if (ret)
			goto out;

		ret = ar0521_group_param_release(sensor);
		if (ret)
			goto out;
	}

	sel->r = *_crop;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int ar0521_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct ar0521 *sensor = to_ar0521(sd);
	struct v4l2_rect *_crop;
	unsigned int x_min = sensor->limits.x.min;
	unsigned int y_min = sensor->limits.y.min;
	unsigned int x_max = sensor->limits.x.max;
	unsigned int y_max = sensor->limits.y.max;

	dev_dbg(sd->dev, "%s\n", __func__);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		mutex_lock(&sensor->lock);

		_crop = ar0521_get_pad_crop(sensor, state, sel->pad, sel->which);
		sel->r = *_crop;

		mutex_unlock(&sensor->lock);
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = x_min;
		sel->r.top = y_min;
		sel->r.width = (x_max - x_min + 1);
		sel->r.height = (y_max - y_min + 1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ar0521_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct ar0521 *sensor = to_ar0521(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2 = sensor->info.buscfg;

	return 0;
}

static const struct v4l2_subdev_core_ops ar0521_subdev_core_ops = {
	.s_power		= ar0521_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.s_register		= ar0521_s_register,
	.g_register		= ar0521_g_register,
#endif
};

static const struct v4l2_subdev_video_ops ar0521_subdev_video_ops = {
	.s_stream		= ar0521_s_stream,
	.g_frame_interval	= ar0521_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0521_subdev_pad_ops = {
	.enum_mbus_code		= ar0521_enum_mbus_code,
	.enum_frame_size	= ar0521_enum_frame_size,
	.set_fmt		= ar0521_set_fmt,
	.get_fmt		= ar0521_get_fmt,
	.set_selection		= ar0521_set_selection,
	.get_selection		= ar0521_get_selection,
	.get_mbus_config	= ar0521_get_mbus_config,
};

static const struct v4l2_subdev_ops ar0521_subdev_ops = {
	.core			= &ar0521_subdev_core_ops,
	.video			= &ar0521_subdev_video_ops,
	.pad			= &ar0521_subdev_pad_ops,
};

static int ar0521_set_analogue_gain(struct ar0521 *sensor, unsigned int val)
{
	unsigned int coarse, fine;

	for (coarse = 4; coarse >= 0; coarse--)
		if ((1u << coarse) * 1000 <= val)
			break;

	val = val / (1u << (coarse));
	fine = ((val * 16) / 1000) - 16;

	if (fine > 15)
		fine = 15;

	ar0521_update_bits(sensor, AR0521_GREENR_GAIN,
			   BIT_ANA_COARSE_GAIN_MASK | BIT_ANA_FINE_GAIN_MASK,
			   BIT_ANA_COARSE_GAIN(coarse) |
			   BIT_ANA_FINE_GAIN(fine));

	return 1000 * (1u << coarse) * (16 + fine) / 16;
}

unsigned int ar0521_get_min_color_gain(struct ar0521 *sensor)
{
	unsigned int gains[4];
	int min_idx = 0;
	int i;

	gains[0] = sensor->gains.red_ctrl->val;
	gains[1] = sensor->gains.greenr_ctrl->val;
	gains[2] = sensor->gains.greenb_ctrl->val;
	gains[3] = sensor->gains.blue_ctrl->val;

	for (i = 0; i < 4; i++) {
		if (gains[i] < gains[min_idx])
			min_idx = i;
	}

	return gains[min_idx];
}

static int ar0521_set_digital_gain(struct ar0521 *sensor,
				   struct v4l2_ctrl *ctrl)
{
	unsigned int gain;
	unsigned int gain_min;
	int ret;
	u16 val, mask;

	val = BIT_DIGITAL_GAIN((ctrl->val * 64) / 1000);
	mask = BIT_DIGITAL_GAIN_MASK;

	switch (ctrl->id) {
	case V4L2_CID_DIGITAL_GAIN:
		if (sensor->model == AR0521_MODEL_MONOCHROME) {
			ret = ar0521_update_bits(sensor, AR0521_GLOBAL_GAIN,
						 mask, val);
			return ret;
		}

		gain = sensor->gains.red * ctrl->val;
		gain = gain / sensor->gains.min_ref;
		gain = clamp_t(unsigned int, gain, 1000, 7999);
		val = BIT_DIGITAL_GAIN((gain * 64) / 1000);
		ret = ar0521_update_bits(sensor, AR0521_RED_GAIN, mask, val);
		if (ret)
			return ret;

		sensor->gains.red_ctrl->val = gain;
		sensor->gains.red_ctrl->cur.val = gain;

		gain = sensor->gains.greenr * ctrl->val;
		gain = gain / sensor->gains.min_ref;
		gain = clamp_t(unsigned int, gain, 1000, 7999);
		val = BIT_DIGITAL_GAIN((gain * 64) / 1000);
		ret = ar0521_update_bits(sensor, AR0521_GREENR_GAIN, mask, val);
		if (ret)
			return ret;

		sensor->gains.greenr_ctrl->val = gain;
		sensor->gains.greenr_ctrl->cur.val = gain;

		gain = sensor->gains.greenb * ctrl->val;
		gain = gain / sensor->gains.min_ref;
		gain = clamp_t(unsigned int, gain, 1000, 7999);
		val = BIT_DIGITAL_GAIN((gain * 64) / 1000);
		ret = ar0521_update_bits(sensor, AR0521_GREENB_GAIN, mask, val);
		if (ret)
			return ret;

		sensor->gains.greenb_ctrl->val = gain;
		sensor->gains.greenb_ctrl->cur.val = gain;

		gain = sensor->gains.blue * ctrl->val;
		gain = gain / sensor->gains.min_ref;
		gain = clamp_t(unsigned int, gain, 1000, 7999);
		val = BIT_DIGITAL_GAIN((gain * 64) / 1000);
		ret = ar0521_update_bits(sensor, AR0521_BLUE_GAIN, mask, val);
		if (ret)
			return ret;

		sensor->gains.blue_ctrl->val = gain;
		sensor->gains.blue_ctrl->cur.val = gain;

		break;
	case V4L2_CID_X_DIGITAL_GAIN_RED:
		ret = ar0521_update_bits(sensor, AR0521_RED_GAIN, mask, val);
		if (ret)
			return ret;
		break;
	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
		ret = ar0521_update_bits(sensor, AR0521_GREENR_GAIN, mask, val);
		if (ret)
			return ret;
		break;
	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		ret = ar0521_update_bits(sensor, AR0521_GREENB_GAIN, mask, val);
		if (ret)
			return ret;
		break;
	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
		ret = ar0521_update_bits(sensor, AR0521_BLUE_GAIN, mask, val);
		if (ret)
			return ret;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	switch (ctrl->id) {
	case V4L2_CID_X_DIGITAL_GAIN_RED:
	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
		gain_min = ar0521_get_min_color_gain(sensor);
		sensor->gains.red = sensor->gains.red_ctrl->val;
		sensor->gains.greenr = sensor->gains.greenr_ctrl->val;
		sensor->gains.greenb = sensor->gains.greenb_ctrl->val;
		sensor->gains.blue = sensor->gains.blue_ctrl->val;
		sensor->gains.min_ref = gain_min;
		sensor->gains.dig_ctrl->val = gain_min;
		sensor->gains.dig_ctrl->cur.val = gain_min;
		break;
	default:
		break;
	}

	return 0;
}

static int ar0521_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0521 *sensor = ctrl->priv;
	unsigned int vlen_old;
	unsigned int hlen_old;
	int ret = 0;
	u16 val;
	u16 mask;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		if (sensor->is_streaming) {
			ret = ar0521_group_param_hold(sensor);
			if (ret)
				break;
		}

		vlen_old = sensor->vlen;
		sensor->vlen = sensor->fmt.height + ctrl->val;

		if (sensor->is_streaming) {
			ret = ar0521_config_frame(sensor);
			if (ret) {
				sensor->vlen = vlen_old;
				break;
			}

			ret = ar0521_group_param_release(sensor);
			if (ret)
				sensor->vlen = vlen_old;
		}

		break;
	case V4L2_CID_HBLANK:
		if (sensor->is_streaming) {
			ret = ar0521_group_param_hold(sensor);
			if (ret)
				break;
		}

		hlen_old = sensor->hlen;
		sensor->hlen = sensor->fmt.width + ctrl->val;

		if (sensor->is_streaming) {
			ret = ar0521_config_frame(sensor);
			if (ret) {
				sensor->hlen = hlen_old;
				break;
			}

			ret = ar0521_group_param_release(sensor);
			if (ret)
				sensor->hlen = hlen_old;
		}

		break;
	case V4L2_CID_HFLIP:
		ret = ar0521_update_bits(sensor, AR0521_READ_MODE,
					 BIT_HORIZ_MIRR,
					 ctrl->val ? BIT_HORIZ_MIRR : 0);
		break;
	case V4L2_CID_VFLIP:
		ret = ar0521_update_bits(sensor, AR0521_READ_MODE,
					 BIT_VERT_FLIP,
					 ctrl->val ? BIT_VERT_FLIP : 0);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ar0521_write(sensor, AR0521_COARSE_INT_TIME, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = ar0521_write(sensor, AR0521_TEST_DATA_RED, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = ar0521_write(sensor, AR0521_TEST_DATA_GREENR, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = ar0521_write(sensor, AR0521_TEST_DATA_BLUE, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = ar0521_write(sensor, AR0521_TEST_DATA_GREENB, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		/* TODO: This needs fixing */
		switch (ctrl->val) {
		case 5:
			val = AR0521_TP_WALKING_ONES_10BIT;
			break;
		case 6:
			val = AR0521_TP_WALKING_ONES_8BIT;
			break;
		default:
			val = ctrl->val;
			break;
		}

		ret = ar0521_write(sensor, AR0521_TEST_PATTERN, val);
		break;
	case V4L2_CID_X_BINNING_COL:
		ret = ar0521_update_bits(sensor, AR0521_READ_MODE,
					 BIT_X_BIN_EN,
					 ctrl->val ? BIT_X_BIN_EN : 0);
		break;
	case V4L2_CID_X_EXTRA_BLANKING:
		ret = ar0521_write(sensor, AR0521_EXTRA_DELAY, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
	case V4L2_CID_X_DIGITAL_GAIN_RED:
	case V4L2_CID_X_DIGITAL_GAIN_GREENR:
	case V4L2_CID_X_DIGITAL_GAIN_BLUE:
	case V4L2_CID_X_DIGITAL_GAIN_GREENB:
		ret = ar0521_set_digital_gain(sensor, ctrl);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ctrl->val = ar0521_set_analogue_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_X_DYNAMIC_PIXEL_CORRECTION:
		if (sensor->is_streaming)
			return -EBUSY;

		mask = BIT_PIX_DEF_2D_COUPLE_EN |
		       BIT_PIX_DEF_2D_SINGLE_EN |
		       BIT_PIX_DEF_2D_FUSE_EN |
		       BIT_PIX_DEF_ID_LOC_CORR_EN |
		       BIT_PIX_DEF_ID_EN;

		val = ctrl->val ? BIT_PIX_DEF_2D_COUPLE_EN |
				  BIT_PIX_DEF_2D_SINGLE_EN |
				  BIT_PIX_DEF_2D_FUSE_EN : 0;

		val |= BIT_PIX_DEF_ID_LOC_CORR_EN |
		       BIT_PIX_DEF_ID_EN;

		ret = ar0521_update_bits(sensor, AR0521_PIX_DEF_ID, mask, val);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		switch (ctrl->val) {
		case V4L2_FLASH_LED_MODE_NONE:
			val = 0;
			break;

		case V4L2_FLASH_LED_MODE_FLASH:
			val = BIT_XENON_FLASH;
			break;

		case V4L2_FLASH_LED_MODE_TORCH:
			val = BIT_LED_FLASH;
			break;
		}

		mask = BIT_XENON_FLASH | BIT_LED_FLASH;

		ret = ar0521_update_bits(sensor, AR0521_FLASH, mask, val);
		break;
	case V4L2_CID_X_FLASH_INVERT:
		val = ctrl->val ? BIT_INVERT_FLASH : 0;
		ret = ar0521_update_bits(sensor, AR0521_FLASH,
					 BIT_INVERT_FLASH, val);
		break;
	case V4L2_CID_X_FLASH_XENON_WIDTH:
		ret = ar0521_write(sensor, AR0521_FLASH_COUNT, ctrl->val);
		break;
	case V4L2_CID_X_TRIGGER_MODE:
		sensor->trigger = ctrl->val;
		ret = ar0521_set_trigger_mode(sensor, sensor->trigger);
		break;
	case V4L2_CID_X_TRIGGER_PIN:
		if (sensor->trigger)
			return -EBUSY;

		sensor->trigger_pin = ctrl->val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ar0521_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0521 *sensor = ctrl->priv;
	int index;

	index = bpp_to_index(sensor->bpp);

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		ctrl->val = index;
		break;
	case V4L2_CID_PIXEL_RATE:
		*ctrl->p_new.p_s64 = sensor->pll[index].pix_freq;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ar0521_ctrl_ops = {
	.s_ctrl			= ar0521_s_ctrl,
	.g_volatile_ctrl	= ar0521_g_volatile_ctrl,
};

static const char * const ar0521_test_pattern_menu[] = {
	"disabled",
	"solid color",
	"color bar",
	"fade to gray",
	"pn9 link integrity",
	"walking 1 (10 bit)",
	"walking 1 (8 bit)",
};

static const char * const ar0521_binning_menu[] = {
	"disable",
	"enable",
};

static const char * const ar0521_trigger_menu[] = {
	"off",
	"Global Reset Release",
	"Electronic Rolling Shutter",
};

static const struct v4l2_ctrl_config ar0521_ctrls[] = {
	{
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_VBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 22,
		.max		= 65535,
		.step		= 1,
		.def		= 22,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_HBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 208,
		.max		= 65535,
		.step		= 1,
		.def		= 208,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 65535,
		.step		= 1,
		.def		= 1817,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_GREENR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_GREENB,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 0,
		.max		= 4095,
		.step		= 1,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_TEST_PATTERN,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= ARRAY_SIZE(ar0521_test_pattern_menu) - 1,
		.qmenu		= ar0521_test_pattern_menu,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_BINNING_COL,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Column Binning",
		.min		= 0,
		.max		= ARRAY_SIZE(ar0521_binning_menu) - 1,
		.qmenu		= ar0521_binning_menu,
		.def		= 0,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_EXTRA_BLANKING,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Extra Vertical Blanking",
		.min		= 0,
		.step		= 1,
		.max		= 65535,
		.def		= 0,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_ANALOGUE_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 1000,
		.step		= 1,
		.max		= 14000,
		.def		= 2000,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_DIGITAL_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.min		= 1000,
		.step		= 1,
		.max		= 7999,
		.def		= 1000,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_RED,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Red",
		.min		= 1000,
		.step		= 1,
		.max		= 7999,
		.def		= 1000,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_GREENR,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Green (Red)",
		.min		= 1000,
		.step		= 1,
		.max		= 7999,
		.def		= 1000,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_BLUE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Blue",
		.min		= 1000,
		.step		= 1,
		.max		= 7999,
		.def		= 1000,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_DIGITAL_GAIN_GREENB,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Digital Gain Green (Blue)",
		.min		= 1000,
		.step		= 1,
		.max		= 7999,
		.def		= 1000,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_LINK_FREQ,
		.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
		.min		= AR0521_FREQ_MENU_8BIT,
		.max		= AR0521_FREQ_MENU_12BIT,
		.def		= AR0521_FREQ_MENU_12BIT,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_PIXEL_RATE,
		.type		= V4L2_CTRL_TYPE_INTEGER64,
		.min		= 0,
		.max		= INT_MAX,
		.step		= 1,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_DYNAMIC_PIXEL_CORRECTION,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Dynamic Defect Pixel Correction",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_FLASH_LED_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.min		= 0,
		.max		= V4L2_FLASH_LED_MODE_TORCH,
		.def		= V4L2_FLASH_LED_MODE_NONE,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_FLASH_INVERT,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Invert Flash",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_FLASH_XENON_WIDTH,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash Xenon Width",
		.min		= 0,
		.step		= 1,
		.max		= 65535,
		.def		= 256,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_TRIGGER_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Trigger mode",
		.min		= AR0521_TRIGGER_OFF,
		.max		= ARRAY_SIZE(ar0521_trigger_menu) - 1,
		.qmenu		= ar0521_trigger_menu,
		.def		= AR0521_TRIGGER_OFF,
	}, {
		.ops		= &ar0521_ctrl_ops,
		.id		= V4L2_CID_X_TRIGGER_PIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Trigger pin",
		.min		= 0,
		.step		= 1,
		.max		= 3,
		.def		= 2,
	},
};

static int ar0521_create_ctrls(struct ar0521 *sensor)
{
	struct v4l2_ctrl_config ctrl_cfg;
	struct v4l2_ctrl *ctrl;
	struct ar0521_sensor_limits *limits = &sensor->limits;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ar0521_ctrls); i++) {
		ctrl_cfg = ar0521_ctrls[i];

		switch (ctrl_cfg.id) {
		case V4L2_CID_X_DIGITAL_GAIN_RED:
		case V4L2_CID_X_DIGITAL_GAIN_GREENR:
		case V4L2_CID_X_DIGITAL_GAIN_BLUE:
		case V4L2_CID_X_DIGITAL_GAIN_GREENB:
			if (sensor->model == AR0521_MODEL_MONOCHROME)
				continue;

			break;
		case V4L2_CID_HBLANK:
			ctrl_cfg.min = limits->hlen.min - AR0521_DEF_WIDTH;
			ctrl_cfg.def = ctrl_cfg.min;
			break;
		case V4L2_CID_VBLANK:
			ctrl_cfg.min = limits->vblank.min;
			ctrl_cfg.def = ctrl_cfg.min;
			break;
		case V4L2_CID_LINK_FREQ:
			ctrl_cfg.qmenu_int = sensor->info.link_freqs;
		default:
			break;
		}

		ctrl = v4l2_ctrl_new_custom(&sensor->ctrls,
					    &ctrl_cfg, sensor);

		ret = sensor->ctrls.error;
		if (ret) {
			v4l2_warn(&sensor->subdev,
				  "failed to register control '%s' (0x%x): %d\n",
				  ctrl_cfg.name ? ctrl_cfg.name :
				  v4l2_ctrl_get_name(ctrl_cfg.id),
				  ctrl_cfg.id, ret);
			return ret;
		}

		switch (ctrl->id) {
		case V4L2_CID_PIXEL_RATE:
			ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY |
				       V4L2_CTRL_FLAG_VOLATILE;
			break;
		case V4L2_CID_LINK_FREQ:
			ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY |
				       V4L2_CTRL_FLAG_VOLATILE;
			break;
		case V4L2_CID_EXPOSURE:
			sensor->exp_ctrl = ctrl;
			break;
		case V4L2_CID_VBLANK:
			sensor->vblank_ctrl = ctrl;
			break;
		case V4L2_CID_HBLANK:
			sensor->hblank_ctrl = ctrl;
			break;
		case V4L2_CID_ANALOGUE_GAIN:
			sensor->gains.ana_ctrl = ctrl;
			break;
		case V4L2_CID_DIGITAL_GAIN:
			ctrl->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE |
				       V4L2_CTRL_FLAG_UPDATE;
			sensor->gains.dig_ctrl = ctrl;
			break;
		case V4L2_CID_X_DIGITAL_GAIN_RED:
			if (sensor->model == AR0521_MODEL_COLOR)
				sensor->gains.red_ctrl = ctrl;
			break;
		case V4L2_CID_X_DIGITAL_GAIN_GREENB:
			if (sensor->model == AR0521_MODEL_COLOR)
				sensor->gains.greenb_ctrl = ctrl;
			break;
		case V4L2_CID_X_DIGITAL_GAIN_GREENR:
			if (sensor->model == AR0521_MODEL_COLOR)
				sensor->gains.greenr_ctrl = ctrl;
			break;
		case V4L2_CID_X_DIGITAL_GAIN_BLUE:
			if (sensor->model == AR0521_MODEL_COLOR)
				sensor->gains.blue_ctrl = ctrl;
			break;
		default:
			break;
		}
	}

	return 0;
}

static void ar0521_set_defaults(struct ar0521 *sensor)
{
	sensor->limits = (struct ar0521_sensor_limits) {
					/* mim		max      */
		.x			= {0,		2603     },
		.y			= {0,		1955     },
		.hlen			= {3080,	65532    },
		.vlen			= {48,		65535    },
		.hblank			= {240,		65535    },
		.vblank			= {28,		65535    },
		.ext_clk		= {5000000,	64000000 },
	};

	sensor->crop.left = 4;
	sensor->crop.top = 4;
	sensor->crop.width = AR0521_DEF_WIDTH;
	sensor->crop.height = AR0521_DEF_HEIGHT;

	sensor->fmt.width = AR0521_DEF_WIDTH;
	sensor->fmt.height = AR0521_DEF_HEIGHT;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_RAW;

	if (sensor->model == AR0521_MODEL_MONOCHROME) {
		sensor->formats = ar0521_mono_formats;
		sensor->num_fmts = ARRAY_SIZE(ar0521_mono_formats);
	} else {
		sensor->formats = ar0521_col_formats;
		sensor->num_fmts = ARRAY_SIZE(ar0521_col_formats);
	}

	sensor->fmt.code = sensor->formats[sensor->num_fmts - 1].code;
	sensor->bpp = sensor->formats[sensor->num_fmts - 1].bpp;

	sensor->w_skip = 1;
	sensor->h_skip = 1;
	sensor->hlen = sensor->limits.hlen.min;
	sensor->vlen = sensor->fmt.height + sensor->limits.vblank.min;
	sensor->gains.red = 1000;
	sensor->gains.greenr = 1000;
	sensor->gains.greenb = 1000;
	sensor->gains.blue = 1000;
	sensor->gains.min_ref = 1000;

#ifdef DEBUG
	sensor->manual_pll = false;
#endif /* ifdef DEBUG */
}

static const struct ar0521_register sequencer[] = {
	{ 0x3d00, 0x043e },
	{ 0x3d02, 0x4760 },
	{ 0x3d04, 0xffff },
	{ 0x3d06, 0xffff },
	{ 0x3d08, 0x8000 },
	{ 0x3d0a, 0x0510 },
	{ 0x3d0c, 0xaf08 },
	{ 0x3d0e, 0x0252 },
	{ 0x3d10, 0x486f },
	{ 0x3d12, 0x5d5d },
	{ 0x3d14, 0x8056 },
	{ 0x3d16, 0x8313 },
	{ 0x3d18, 0x0087 },
	{ 0x3d1a, 0x6a48 },
	{ 0x3d1c, 0x6982 },
	{ 0x3d1e, 0x0280 },
	{ 0x3d20, 0x8359 },
	{ 0x3d22, 0x8d02 },
	{ 0x3d24, 0x8020 },
	{ 0x3d26, 0x4882 },
	{ 0x3d28, 0x4269 },
	{ 0x3d2a, 0x6a95 },
	{ 0x3d2c, 0x5988 },
	{ 0x3d2e, 0x5a83 },
	{ 0x3d30, 0x5885 },
	{ 0x3d32, 0x6280 },
	{ 0x3d34, 0x6289 },
	{ 0x3d36, 0x6097 },
	{ 0x3d38, 0x5782 },
	{ 0x3d3a, 0x605c },
	{ 0x3d3c, 0xbf18 },
	{ 0x3d3e, 0x0961 },
	{ 0x3d40, 0x5080 },
	{ 0x3d42, 0x2090 },
	{ 0x3d44, 0x4390 },
	{ 0x3d46, 0x4382 },
	{ 0x3d48, 0x5f8a },
	{ 0x3d4a, 0x5d5d },
	{ 0x3d4c, 0x9c63 },
	{ 0x3d4e, 0x8063 },
	{ 0x3d50, 0xa960 },
	{ 0x3d52, 0x9757 },
	{ 0x3d54, 0x8260 },
	{ 0x3d56, 0x5cff },
	{ 0x3d58, 0xbf10 },
	{ 0x3d5a, 0x1681 },
	{ 0x3d5c, 0x0802 },
	{ 0x3d5e, 0x8000 },
	{ 0x3d60, 0x141c },
	{ 0x3d62, 0x6000 },
	{ 0x3d64, 0x6022 },
	{ 0x3d66, 0x4d80 },
	{ 0x3d68, 0x5c97 },
	{ 0x3d6a, 0x6a69 },
	{ 0x3d6c, 0xac6f },
	{ 0x3d6e, 0x4645 },
	{ 0x3d70, 0x4400 },
	{ 0x3d72, 0x0513 },
	{ 0x3d74, 0x8069 },
	{ 0x3d76, 0x6ac6 },
	{ 0x3d78, 0x5f95 },
	{ 0x3d7a, 0x5f70 },
	{ 0x3d7c, 0x8040 },
	{ 0x3d7e, 0x4a81 },
	{ 0x3d80, 0x0300 },
	{ 0x3d82, 0xe703 },
	{ 0x3d84, 0x0088 },
	{ 0x3d86, 0x4a83 },
	{ 0x3d88, 0x40ff },
	{ 0x3d8a, 0xffff },
	{ 0x3d8c, 0xfd70 },
	{ 0x3d8e, 0x8040 },
	{ 0x3d90, 0x4a85 },
	{ 0x3d92, 0x4fa8 },
	{ 0x3d94, 0x4f8c },
	{ 0x3d96, 0x0070 },
	{ 0x3d98, 0xbe47 },
	{ 0x3d9a, 0x8847 },
	{ 0x3d9c, 0xbc78 },
	{ 0x3d9e, 0x6b89 },
	{ 0x3da0, 0x6a80 },
	{ 0x3da2, 0x6986 },
	{ 0x3da4, 0x6b8e },
	{ 0x3da6, 0x6b80 },
	{ 0x3da8, 0x6980 },
	{ 0x3daa, 0x6a88 },
	{ 0x3dac, 0x7c9f },
	{ 0x3dae, 0x866b },
	{ 0x3db0, 0x8765 },
	{ 0x3db2, 0x46ff },
	{ 0x3db4, 0xe365 },
	{ 0x3db6, 0xa679 },
	{ 0x3db8, 0x4a40 },
	{ 0x3dba, 0x4580 },
	{ 0x3dbc, 0x44bc },
	{ 0x3dbe, 0x7000 },
	{ 0x3dc0, 0x8040 },
	{ 0x3dc2, 0x0802 },
	{ 0x3dc4, 0x10ef },
	{ 0x3dc6, 0x0104 },
	{ 0x3dc8, 0x3860 },
	{ 0x3dca, 0x5d5d },
	{ 0x3dcc, 0x5682 },
	{ 0x3dce, 0x1300 },
	{ 0x3dd0, 0x8648 },
	{ 0x3dd2, 0x8202 },
	{ 0x3dd4, 0x8082 },
	{ 0x3dd6, 0x598a },
	{ 0x3dd8, 0x0280 },
	{ 0x3dda, 0x2048 },
	{ 0x3ddc, 0x3060 },
	{ 0x3dde, 0x8042 },
	{ 0x3de0, 0x9259 },
	{ 0x3de2, 0x865a },
	{ 0x3de4, 0x8258 },
	{ 0x3de6, 0x8562 },
	{ 0x3de8, 0x8062 },
	{ 0x3dea, 0x8560 },
	{ 0x3dec, 0x9257 },
	{ 0x3dee, 0x8221 },
	{ 0x3df0, 0x10ff },
	{ 0x3df2, 0xb757 },
	{ 0x3df4, 0x9361 },
	{ 0x3df6, 0x1019 },
	{ 0x3df8, 0x8020 },
	{ 0x3dfa, 0x9043 },
	{ 0x3dfc, 0x8b43 },
	{ 0x3dfe, 0x875f },
	{ 0x3e00, 0x835d },
	{ 0x3e02, 0x805d },
	{ 0x3e04, 0x8163 },
	{ 0x3e06, 0x8063 },
	{ 0x3e08, 0xa060 },
	{ 0x3e0a, 0x9157 },
	{ 0x3e0c, 0x8260 },
	{ 0x3e0e, 0x5cff },
	{ 0x3e10, 0xffff },
	{ 0x3e12, 0xffe5 },
	{ 0x3e14, 0x1016 },
	{ 0x3e16, 0x2048 },
	{ 0x3e18, 0x0802 },
	{ 0x3e1a, 0x1c60 },
	{ 0x3e1c, 0x0014 },
	{ 0x3e1e, 0x0060 },
	{ 0x3e20, 0x2205 },
	{ 0x3e22, 0x8120 },
	{ 0x3e24, 0x908f },
	{ 0x3e26, 0x6a80 },
	{ 0x3e28, 0x6982 },
	{ 0x3e2a, 0x5f9f },
	{ 0x3e2c, 0x6f46 },
	{ 0x3e2e, 0x4544 },
	{ 0x3e30, 0x0005 },
	{ 0x3e32, 0x8013 },
	{ 0x3e34, 0x8069 },
	{ 0x3e36, 0x6a80 },
	{ 0x3e38, 0x7000 },
	{ 0x3e3a, 0x0000 },
	{ 0x3e3c, 0x0000 },
	{ 0x3e3e, 0x0000 },
	{ 0x3e40, 0x0000 },
	{ 0x3e42, 0x0000 },
	{ 0x3e44, 0x0000 },
	{ 0x3e46, 0x0000 },
	{ 0x3e48, 0x0000 },
	{ 0x3e4a, 0x0000 },
	{ 0x3e4c, 0x0000 },
	{ 0x3e4e, 0x0000 },
	{ 0x3e50, 0x0000 },
	{ 0x3e52, 0x0000 },
	{ 0x3e54, 0x0000 },
	{ 0x3e56, 0x0000 },
	{ 0x3e58, 0x0000 },
	{ 0x3e5a, 0x0000 },
	{ 0x3e5c, 0x0000 },
	{ 0x3e5e, 0x0000 },
	{ 0x3e60, 0x0000 },
	{ 0x3e62, 0x0000 },
	{ 0x3e64, 0x0000 },
	{ 0x3e66, 0x0000 },
	{ 0x3e68, 0x0000 },
	{ 0x3e6a, 0x0000 },
	{ 0x3e6c, 0x0000 },
	{ 0x3e6e, 0x0000 },
	{ 0x3e70, 0x0000 },
	{ 0x3e72, 0x0000 },
	{ 0x3e74, 0x0000 },
	{ 0x3e76, 0x0000 },
	{ 0x3e78, 0x0000 },
	{ 0x3e7a, 0x0000 },
	{ 0x3e7c, 0x0000 },
	{ 0x3e7e, 0x0000 },
	{ 0x3e80, 0x0000 },
	{ 0x3e82, 0x0000 },
	{ 0x3e84, 0x0000 },
	{ 0x3e86, 0x0000 },
	{ 0x3e88, 0x0000 },
	{ 0x3e8a, 0x0000 },
	{ 0x3e8c, 0x0000 },
	{ 0x3e8e, 0x0000 },
	{ 0x3e90, 0x0000 },
	{ 0x3e92, 0x0000 },
	{ 0x3e94, 0x0000 },
	{ 0x3e96, 0x0000 },
	{ 0x3e98, 0x0000 },
	{ 0x3e9a, 0x0000 },
	{ 0x3e9c, 0x0000 },
	{ 0x3e9e, 0x0000 },
	{ 0x3ea0, 0x0000 },
	{ 0x3ea2, 0x0000 },
	{ 0x3ea4, 0x0000 },
	{ 0x3ea6, 0x0000 },
	{ 0x3ea8, 0x0000 },
	{ 0x3eaa, 0x0000 },
	{ 0x3eac, 0x0000 },
	{ 0x3eae, 0x0000 },
	{ 0x3eb0, 0x0000 },
	{ 0x3eb2, 0x0000 },
	{ 0x3eb4, 0x0000 },
};

static int ar0521_init_sequencer(struct ar0521 *sensor)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(sequencer); i++) {
		ret = ar0521_write(sensor, sequencer[i].reg, sequencer[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int ar0521_subdev_registered(struct v4l2_subdev *sd)
{
	struct ar0521 *sensor = to_ar0521(sd);
	int ret;

	ar0521_set_defaults(sensor);

#ifdef DEBUG
	ar0521_debugfs_init(sensor);
#endif /* ifdef DEBUG */

	ret = ar0521_init_sequencer(sensor);
	if (ret)
		return ret;

	ret = ar0521_create_ctrls(sensor);
	if (ret)
		return ret;

	v4l2_ctrl_handler_setup(&sensor->ctrls);

	return 0;
}

static const struct v4l2_subdev_internal_ops ar0521_subdev_internal_ops = {
	.registered		= ar0521_subdev_registered,
};

static int ar0521_check_chip_id(struct ar0521 *sensor)
{
	struct device *dev = sensor->subdev.dev;
	int ret;
	u16 model_id, customer_rev;

	ret = ar0521_power_on(sensor);
	if (ret) {
		dev_err(dev, "Failed to power on sensor (%d)\n", ret);
		return ret;
	}

	ar0521_reset(sensor);

	ret = ar0521_read(sensor, AR0521_MODEL_ID, &model_id);
	if (ret)
		return ret;

	if (model_id != AR0521_CHIP_ID && model_id != AR0522_CHIP_ID) {
		dev_err(dev, "Wrong chip version: 0x%04x\n", model_id);
		return -ENOENT;
	}

	ret = ar0521_read(sensor, AR0521_CUSTOMER_REV, &customer_rev);
	if (ret)
		return ret;

	dev_info(dev, "Device ID: 0x%04x customer rev: 0x%04x\n",
		 model_id, customer_rev);

	if (sensor->model == AR0521_MODEL_UNKNOWN) {
		if (customer_rev & BIT(4))
			sensor->model = AR0521_MODEL_COLOR;
		else
			sensor->model = AR0521_MODEL_MONOCHROME;
	}

	return 0;
}

static unsigned long ar0521_clk_mul_div(unsigned long freq,
					unsigned int mul,
					unsigned int div)
{
	uint64_t result;

	if (WARN_ON(div == 0))
		return 0;

	result = freq;
	result *= mul;
	result = div_u64(result, div);

	return result;
}

static int ar0521_calculate_pll(struct device *dev,
				struct ar0521_pll_config *pll,
				unsigned long ext_freq,
				u64 link_freq,
				unsigned int bpp,
				unsigned int lanes)
{
	unsigned long op_clk;
	unsigned long vco;
	unsigned long pix_clk;
	unsigned long pix_clk_target;
	unsigned long diff, diff_old;
	unsigned int div, mul;
	const struct limit_range div_lim = {.min = 1, .max = 63};
	const struct limit_range mul_lim = {.min = 32, .max = 254};
	const struct limit_range pix_lim = {.min = 84000000, .max = 207000000};
	const struct limit_range vco_lim = {
		.min = 320000000,
		.max = 1280000000
	};

	pix_clk_target = ar0521_clk_mul_div(link_freq, 2 * lanes, bpp);
	diff_old = pix_clk_target;

	pll->pll_div = 3;
	pll->pll_mul = 89;
	pll->pll2_div = 1;
	pll->pll2_mul = 0;
	pll->op_sys_div = 1;
	pll->op_pix_div = 2;
	pll->vt_sys_div = 1;
	pll->vt_pix_div = bpp / 2;

	div = div_lim.min;
	mul = mul_lim.min;

	if (pix_clk_target < (2 * pix_lim.min)) {
		dev_warn(dev, "Link target too small, %d bit pll not valid\n",
			 bpp);
		return 0;
	}

	while (div <= div_lim.max) {
		if (mul % 2 != 0)
			mul++;

		if (mul > mul_lim.max) {
			mul = mul_lim.min;
			div++;
			if (div > div_lim.max)
				break;
		}

		vco = ar0521_clk_mul_div(ext_freq, mul, div);

		if (vco < vco_lim.min || vco > vco_lim.max) {
			mul++;
			continue;
		}

		pix_clk = ar0521_clk_mul_div(vco, 2, pll->vt_pix_div);
		op_clk = ar0521_clk_mul_div(pix_clk, 1, 4);

		if (pix_clk < (2 * pix_lim.min) ||
		    pix_clk > (2 * pix_lim.max)) {
			mul++;
			continue;
		}

		if (pix_clk > pix_clk_target) {
			mul++;
			continue;
		}

		diff = pix_clk_target - pix_clk;
		if (diff >= diff_old) {
			mul++;
			continue;
		}

		dev_dbg(dev, "%s: vco: %lu pix_clk: %lu op_clk: %lu\n",
			__func__, vco, pix_clk, op_clk);
		dev_dbg(dev, "%s pll2_div: %d pll2_mul: %d\n",
			__func__, div, mul);

		diff_old = diff;

		pll->pll2_div = div;
		pll->pll2_mul = mul;
		pll->vco_freq = vco;
		pll->pix_freq = pix_clk;
		pll->ser_freq = ar0521_clk_mul_div(pix_clk, bpp, 2 * lanes);

		mul++;
	}

	if (pll->pll2_mul == 0) {
		dev_err(dev, "Unable to find matching pll config\n");
		return -EINVAL;
	}

	dev_dbg(dev, "PLL: bpp: %u VCO: %lu, PIX: %lu, SER: %lu\n",
		bpp, pll->vco_freq, pll->pix_freq, pll->ser_freq);

	return 0;
}

static int ar0521_parse_endpoint(struct device *dev, struct ar0521 *sensor,
				 struct fwnode_handle *ep)
{
	struct v4l2_fwnode_endpoint buscfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	u64 *link_freqs;
	unsigned long ext_freq = clk_get_rate(sensor->extclk);
	unsigned int tmp;
	int i;
	int ret;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &buscfg);
	if (ret) {
		dev_err(dev, "Failed to parse MIPI endpoint (%d)\n", ret);
		return ret;
	}

	sensor->info.buscfg = buscfg.bus.mipi_csi2;

	if (buscfg.nr_of_link_frequencies != 1) {
		dev_err(dev, "MIPI link frequency required\n");
		ret = -EINVAL;
		goto out;
	}

	if (buscfg.link_frequencies[0] > AR0521_MAX_LINK_FREQ) {
		dev_err(dev, "MIPI link frequency exceeds maximum\n");
		ret = -EINVAL;
		goto out;
	}

	link_freqs = devm_kcalloc(dev, 3, sizeof(*sensor->info.link_freqs),
				  GFP_KERNEL);
	if (!link_freqs) {
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < 3; i++) {
		ret = ar0521_calculate_pll(dev, &sensor->pll[i], ext_freq,
					   buscfg.link_frequencies[0],
					   index_to_bpp(i),
					   sensor->info.buscfg.num_data_lanes);
		if (ret)
			goto out;

		link_freqs[i] = sensor->pll[i].ser_freq;
	}

	sensor->info.link_freqs = link_freqs;
	sensor->pll[3] = sensor->pll[AR0521_FREQ_MENU_12BIT];

	tmp = 2;
	fwnode_property_read_u32(ep, "onsemi,t-hs-prep", &tmp);
	sensor->info.t_hs_prep = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 15;
	fwnode_property_read_u32(ep, "onsemi,t-hs-zero", &tmp);
	sensor->info.t_hs_zero = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 9;
	fwnode_property_read_u32(ep, "onsemi,t-hs-trail", &tmp);
	sensor->info.t_hs_trail = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 2;
	fwnode_property_read_u32(ep, "onsemi,t-clk-prep", &tmp);
	sensor->info.t_clk_prep = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 34;
	fwnode_property_read_u32(ep, "onsemi,t-clk-zero", &tmp);
	sensor->info.t_clk_zero = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 10;
	fwnode_property_read_u32(ep, "onsemi,t-clk-trail", &tmp);
	sensor->info.t_clk_trail = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 10;
	fwnode_property_read_u32(ep, "onsemi,t-bgap", &tmp);
	sensor->info.t_bgap = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 1;
	fwnode_property_read_u32(ep, "onsemi,t-clk-pre", &tmp);
	sensor->info.t_clk_pre = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 3;
	fwnode_property_read_u32(ep, "onsemi,t-clk-post-msbs", &tmp);
	sensor->info.t_clk_post_msbs = clamp_t(unsigned int, tmp, 0, 0xf);

	tmp = 7;
	fwnode_property_read_u32(ep, "onsemi,t-lpx", &tmp);
	sensor->info.t_lpx = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 15;
	fwnode_property_read_u32(ep, "onsemi,t-wakeup", &tmp);
	sensor->info.t_wakeup = clamp_t(unsigned int, tmp, 0, 0x7f);

	tmp = 1;
	fwnode_property_read_u32(ep, "onsemi,t-clk-post", &tmp);
	sensor->info.t_clk_post = clamp_t(unsigned int, tmp, 0, 0x3);

	tmp = 1;
	fwnode_property_read_u32(ep, "onsemi,cont-tx-clk", &tmp);
	sensor->info.cont_tx_clk = tmp ? true : false;

	tmp = 0;
	fwnode_property_read_u32(ep, "onsemi,vreg-mode", &tmp);
	sensor->info.vreg_mode = tmp ? true : false;

	tmp = 13;
	fwnode_property_read_u32(ep, "onsemi,t-hs-exit", &tmp);
	sensor->info.t_hs_exit = clamp_t(unsigned int, tmp, 0, 0x3f);

	tmp = 12;
	fwnode_property_read_u32(ep, "onsemi,t-init", &tmp);
	sensor->info.t_init = clamp_t(unsigned int, tmp, 0, 0x7f);

out:
	v4l2_fwnode_endpoint_free(&buscfg);
	return ret;
}

static int ar0521_of_probe(struct device *dev, struct ar0521 *sensor)
{
	struct fwnode_handle *ep;
	struct clk *clk;
	struct gpio_desc *gpio;
	int ret;

	clk = devm_clk_get(dev, "ext");
	ret = PTR_ERR_OR_ZERO(clk);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret < 0) {
		dev_err(dev, "Failed to get external clock (%d)\n", ret);
		return ret;
	}

	sensor->extclk = clk;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	ret = PTR_ERR_OR_ZERO(gpio);
	if (ret < 0) {
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}

	sensor->reset_gpio = gpio;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);

	if (!ep) {
		dev_err(dev, "Failed to find endpoint\n");
		return -ENODEV;
	}

	ret = ar0521_parse_endpoint(dev, sensor, ep);

	fwnode_handle_put(ep);
	return ret;
}

static int ar0521_probe(struct i2c_client *i2c,
			const struct i2c_device_id *did)
{
	struct ar0521 *sensor;
	struct v4l2_subdev *sd;
	int ret;

	sensor = devm_kzalloc(&i2c->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	dev_info(&i2c->dev, "Probing ar0521 Driver\n");

	sd = &sensor->subdev;
	sensor->model = did->driver_data;

	ret = ar0521_of_probe(&i2c->dev, sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	v4l2_i2c_subdev_init(sd, i2c, &ar0521_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &ar0521_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret)
		goto out_media;

	ret = v4l2_ctrl_handler_init(&sensor->ctrls, 10);
	if (ret)
		goto out;

	sensor->subdev.ctrl_handler = &sensor->ctrls;
	sensor->ctrls.lock = &sensor->lock;

	ret = ar0521_check_chip_id(sensor);
	if (ret)
		goto out;

	ret = v4l2_async_register_subdev_sensor(&sensor->subdev);
	if (ret)
		goto out;

	return 0;

out:
	v4l2_ctrl_handler_free(&sensor->ctrls);
out_media:
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static void ar0521_remove(struct i2c_client *i2c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(i2c);
	struct ar0521 *sensor = to_ar0521(sd);

#ifdef DEBUG
	ar0521_debugfs_remove(sensor);
#endif /* ifdef DEBUG */
	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);
}

static const struct i2c_device_id ar0521_id_table[] = {
	{ "ar0521", AR0521_MODEL_UNKNOWN },
	{ "ar0521c", AR0521_MODEL_COLOR },
	{ "ar0521m", AR0521_MODEL_MONOCHROME },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ar0521_id_table);

static const struct of_device_id ar0521_of_match[] = {
	{ .compatible = "onsemi,ar0521" },
	{ .compatible = "onsemi,ar0521c" },
	{ .compatible = "onsemi,ar0521m" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0521_of_match);

static struct i2c_driver ar0521_i2c_driver = {
	.driver		= {
		.name	= "ar0521",
		.of_match_table = of_match_ptr(ar0521_of_match),
	},
	.probe		= ar0521_probe,
	.remove		= ar0521_remove,
	.id_table	= ar0521_id_table,
};
module_i2c_driver(ar0521_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefan Riedmueller <s.riedmueller@phytec.de>");
