/*
 * Copyright (c) 2010 SAMSUNG
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/sensor/sensors_core.h>
#include "tmd3725.h"

#define MODULE_NAME_PROX          "proximity_sensor"
#define MODULE_NAME_LIGHT         "light_sensor"
#define VENDOR_NAME               "TAOS"
#define CHIP_NAME                 "TMD3725"

#define ATIME_MS                  1596 /* 159.6ms */
#define DGF_DEFAULT               1200
#define R_COEF1_DEFAULT           270  /* 0.27 */
#define G_COEF1_DEFAULT           1000 /* 1 */
#define B_COEF1_DEFAULT           90 /* 0.09 */
#define CT_COEF1_DEFAULT          3193
#define CT_OFFSET1_DEFAULT        1950
#define LUX_MULTIPLE_DEFAULT      10
#define PROX_THRESH_HI_DEFAULT    0x64
#define PROX_THRESH_LOW_DEFAULT   0x46
#define ALS_TIME_DEFAULT          0x38
#define INTR_FILTER_DEFAULT       0
#define PROX_PULSECNT_DEFAULT     0x06
#define ALS_GAIN_DEFAULT          0x02

#define TMD3725_CHIP_ID           0xE4
#define LIGHT_LOG_TIME            30 /* lightsnesor log time 6SEC 200mec X 30 */
#define PROX_AVG_COUNT            40
#define MAX_LUX                   150000
#define MAX_PROX                  1023
#define MIN_PROX                  0

#define AZ_CONFIG_SET             0x7f /* one-shot autozero */
#define PERSISTENCE_FILTER_SET    0x10
#define PPULSE_SET                0x8c /* ppulse_len: 16us , ppluse : 13 */
#define PRATE_SET                 P_TIME_US(20000)
#define WTIME_SET                 0x00 /* wtime 2.8 msec */
#define PGCFG1_SET                0x90 /* pagin 4x , pgldrive 102mA */
#define BINARY_SEARCH_SET         (0x03 << 5)
#define CALIBRATION_SET           0x01

/* driver data */
struct tmd3725_data {
	struct i2c_client *i2c_client;
	struct input_dev *prox_input_dev;
	struct input_dev *light_input_dev;
	struct device *light_dev;
	struct device *prox_dev;
	struct work_struct work_light;
	struct work_struct work_prox;
	struct work_struct work_prox_avg;
	struct mutex prox_mutex;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct hrtimer timer;
	struct hrtimer prox_avg_timer;
	struct workqueue_struct *wq;
	struct workqueue_struct *wq_avg;
	struct regulator *vled;
	ktime_t light_poll_delay;
	ktime_t prox_polling_time;
	u8 power_state;
	int irq;
	int avg[3];
	int prox_avg_enable;
	u16 op_mode_state;
	s32 clrdata;
	s32 reddata;
	s32 grndata;
	s32 bludata;
	s32 irdata;
	int lux;
	int prox_thresh_hi;
	int prox_thresh_low;
	int prox_state;
	int count_log_time;
	bool cal_complete;
	int vled_ldo_pin;
	int dgf;
	int cct_coef;
	int cct_offset;
	int coef_r;
	int coef_g;
	int coef_b;
	int als_time;
	int als_gain;
	int intr_filter;
	int prox_pulsecnt;
	int lux_multiple;
	int prox_irq_gpio;
	int offset;
};

static int tmd3725_prox_vled_onoff(struct tmd3725_data *taos, int onoff)
{
	int err;

	SENSOR_INFO("%s, ldo:%d\n", (onoff) ? "on" : "off", taos->vled_ldo_pin);

	/* ldo control */
	if (taos->vled_ldo_pin) {
		gpio_set_value(taos->vled_ldo_pin, onoff);
		if (onoff)
			msleep(20);
		return 0;
	}

	/* regulator(PMIC) control */
	if (!taos->vled) {
		SENSOR_INFO("VLED get regulator\n");
		taos->vled = regulator_get(&taos->i2c_client->dev, "taos,vled");
		if (IS_ERR(taos->vled)) {
			SENSOR_ERR("regulator_get fail\n");
			taos->vled = NULL;
			return -ENODEV;
		}
	}

	if (onoff) {
		if (regulator_is_enabled(taos->vled)) {
			SENSOR_INFO("Regulator already enabled\n");
			return 0;
		}

		err = regulator_enable(taos->vled);
		if (err)
			SENSOR_ERR("Failed to enable vled.\n");

		usleep_range(10000, 11000);
	} else {
		err = regulator_disable(taos->vled);
		if (err)
			SENSOR_ERR("Failed to disable vled.\n");
	}

	return 0;
}

static int tmd3725_i2c_read(struct tmd3725_data *taos, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(taos->i2c_client, reg);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);

	return ret;
}

static int tmd3725_i2c_write(struct tmd3725_data *taos, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(taos->i2c_client,
		(CMD_REG | reg), val);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);

	return ret;
}

static int tmd3725_i2c_modify_write(struct tmd3725_data *taos,
	u8 reg, u8 mask, u8 val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(taos->i2c_client, reg);
	if (ret < 0) {
		SENSOR_ERR("read failed %d\n", ret);
		return ret;
	}

	ret = (ret & (~mask)) | val;
	ret = i2c_smbus_write_byte_data(taos->i2c_client,
		(CMD_REG | reg), ret);
	if (ret < 0)
		SENSOR_ERR("write failed %d\n", ret);

	return ret;
}

static void tmd3725_interrupt_clear(struct tmd3725_data *taos)
{
	int ret;

	ret = tmd3725_i2c_read(taos, STATUS);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);
}

static int tmd3725_get_cct(struct tmd3725_data *taos)
{
	int bp1 = taos->bludata - taos->irdata;
	int rp1 = taos->reddata - taos->irdata;
	int cct = 0;

	if (rp1 != 0)
		cct = taos->cct_coef * bp1 / rp1 + taos->cct_offset;

	return cct;
}

static int tmd3725_get_lux(struct tmd3725_data *taos)
{
	u8 reg_gain;
	s32 rp1, gp1, bp1;
	s32 clrdata, reddata, grndata, bludata, calculated_lux;
	int gain;
	int ret;

	ret = i2c_smbus_read_word_data(taos->i2c_client, CFG1);
	if (ret < 0) {
		SENSOR_ERR("failed %d\n", ret);
		return taos->lux;
	}
	reg_gain = (u16)ret & 0xff;
	clrdata = i2c_smbus_read_word_data(taos->i2c_client, CLR_CHAN0LO);
	reddata = i2c_smbus_read_word_data(taos->i2c_client, RED_CHAN1LO);
	grndata = i2c_smbus_read_word_data(taos->i2c_client, GRN_CHAN1LO);
	bludata = i2c_smbus_read_word_data(taos->i2c_client, BLU_CHAN1LO);

	taos->clrdata = clrdata;
	taos->reddata = reddata;
	taos->grndata = grndata;
	taos->bludata = bludata;

	switch (reg_gain & 0x03) {
	case 0x00:
		gain = 1;
		break;
	case 0x01:
		gain = 4;
		break;
	case 0x02:
		gain = 16;
		break;
/*	case 0x03:
		gain = 64;
		break;
*/
	default:
		gain = 1;
		break;
	}

	if (gain == 1 && clrdata < 25) {
		reg_gain = 0x02; /* Gain 16x */

		ret = tmd3725_i2c_write(taos, CFG1, reg_gain);
		if (ret < 0)
			SENSOR_ERR("failed %d\n", ret);

		return taos->lux;
	} else if (gain == 16 && clrdata > 15000) {
		reg_gain = 0x00; /* Gain 1x */

		ret = tmd3725_i2c_write(taos, CFG1, reg_gain);
		if (ret < 0)
			SENSOR_ERR("failed %d\n", ret);

		return taos->lux;
	}

	if ((clrdata >= 18500) && (gain == 1))
		return MAX_LUX;

	/* calculate lux */
	taos->irdata = (reddata + grndata + bludata - clrdata) / 2;
	if (taos->irdata < 0)
		taos->irdata = 0;

	/* remove ir from counts*/
	rp1 = (taos->reddata - taos->irdata) * taos->coef_r;
	gp1 = (taos->grndata - taos->irdata) * taos->coef_g;
	bp1 = (taos->bludata - taos->irdata) * taos->coef_b;

	calculated_lux = (rp1 + gp1 + bp1) / 1000;
	if (calculated_lux < 0)
		calculated_lux = 0;
	else {
		/* divide by CPL, CPL = (ATIME_MS * ALS_GAIN / DGF); */
		calculated_lux = calculated_lux * taos->dgf;
		calculated_lux *= taos->lux_multiple; /* ATIME_MS */
		calculated_lux /= ATIME_MS;
		calculated_lux /= gain;
	}

	taos->lux = (int)calculated_lux;

	return taos->lux;
}

static int tmd3725_chip_init(struct tmd3725_data *taos)
{
	int ret = 0;

	ret = tmd3725_i2c_write(taos, CFG3, INT_READ_CLEAR);
	if (ret < 0)
		SENSOR_ERR("failed to write cfg3 reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, AZ_CONFIG, AZ_CONFIG_SET);
	if (ret < 0)
		SENSOR_ERR("failed to write auto zero cfg reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, CFG1, taos->als_gain);
	if (ret < 0)
		SENSOR_ERR("failed to write als gain ctrl reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, ALS_TIME, taos->als_time);
	if (ret < 0)
		SENSOR_ERR("failed to write als time reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, PPERS, PERSISTENCE_FILTER_SET);
	if (ret < 0)
		SENSOR_ERR("failed to write proximity persistence %d\n", ret);

	ret = tmd3725_i2c_write(taos, PGCFG0, PPULSE_SET);
	if (ret < 0)
		SENSOR_ERR("failed to write proximity pulse %d\n", ret);

	ret = tmd3725_i2c_write(taos, PRX_RATE, PRATE_SET);
	if (ret < 0)
		SENSOR_ERR("failed to write proximity sample rate %d\n", ret);

	ret = tmd3725_i2c_write(taos, WAIT_TIME, WTIME_SET);
	if (ret < 0)
		SENSOR_ERR("failed to write als wait time reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, PGCFG1, PGCFG1_SET);
	if (ret < 0)
		SENSOR_ERR("failed to write prox pulse reg %d\n", ret);

	ret = tmd3725_i2c_modify_write(taos, CALIBCFG,
		AUTO_OFFSET_ADJ, AUTO_OFFSET_ADJ);
	if (ret < 0)
		SENSOR_ERR("failed to write enable state reg %d\n", ret);

	return ret;
}

static int tmd3725_chip_off(struct tmd3725_data *taos)
{
	int ret;

	SENSOR_INFO("\n");
	tmd3725_interrupt_clear(taos);

	ret = tmd3725_i2c_write(taos, INTENAB, CNTL_REG_CLEAR);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);

	ret = tmd3725_i2c_write(taos, CMD_REG, CNTL_PWRON);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);

	return ret;
}

static int tmd3725_light_chip_on(struct tmd3725_data *taos)
{
	int ret;

	SENSOR_INFO("\n");

	ret = tmd3725_i2c_write(taos, INTENAB, CNTL_REG_CLEAR);
	if (ret < 0)
		SENSOR_ERR("failed to write int enable reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, ENABLE, PON | AEN);
	if (ret < 0)
		SENSOR_ERR("failed to write enable state reg %d\n", ret);

	return ret;
}

static int tmd3725_prox_chip_on(struct tmd3725_data *taos)
{
	int ret;

	SENSOR_INFO("\n");
	tmd3725_interrupt_clear(taos);

	ret = tmd3725_i2c_write(taos, INTENAB, PIEN | ZIEN);
	if (ret < 0)
		SENSOR_ERR("failed to write int enable reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, ENABLE, PEN | PON | WEN);
	if (ret < 0)
		SENSOR_ERR("failed to write enable state reg %d\n", ret);

	return ret;
}

static int tmd3725_prox_light_chip_on(struct tmd3725_data *taos)
{
	int ret;

	SENSOR_INFO("\n");
	tmd3725_interrupt_clear(taos);

	ret = tmd3725_i2c_write(taos, INTENAB, PIEN | ZIEN);
	if (ret < 0)
		SENSOR_ERR("failed to write int enable reg %d\n", ret);

	ret = tmd3725_i2c_write(taos, ENABLE, PEN | PON | AEN | WEN);
	if (ret < 0)
		SENSOR_ERR("failed to write enable state reg %d\n", ret);

	return ret;
}

static int tmd3725_set_op_mode(struct tmd3725_data *taos, u16 op_mode, u8 state)
{
	int ret = -1;

	if (state)
		taos->op_mode_state |= (op_mode << 0);
	else
		taos->op_mode_state &= ~(op_mode << 0);

	switch (taos->op_mode_state) {
	case MODE_OFF:
		ret = tmd3725_chip_off(taos);
		if (ret < 0)
			SENSOR_ERR("failed to chip off %d\n", ret);
		break;
	case MODE_ALS:
		ret = tmd3725_light_chip_on(taos);
		if (ret < 0)
			SENSOR_ERR("failed to als enable %d\n", ret);
		break;
	case MODE_PROX:
		ret = tmd3725_prox_chip_on(taos);
		if (ret < 0)
			SENSOR_ERR("failed to prox enable %d\n", ret);
		break;
	case MODE_ALS_PROX:
		ret = tmd3725_prox_light_chip_on(taos);
		if (ret < 0)
			SENSOR_ERR("failed to als+prox enable %d\n", ret);
		break;
	default:
		break;
	}

	return ret;
}

static void tmd3725_light_enable(struct tmd3725_data *taos)
{
	SENSOR_INFO("start poll timer\n");
	taos->count_log_time = LIGHT_LOG_TIME;
	hrtimer_start(&taos->timer, taos->light_poll_delay, HRTIMER_MODE_REL);
}

static void tmd3725_light_disable(struct tmd3725_data *taos)
{
	SENSOR_INFO("cancelling poll timer\n");
	hrtimer_cancel(&taos->timer);
	cancel_work_sync(&taos->work_light);
}

static void tmd3725_work_func_light(struct work_struct *work)
{
	struct tmd3725_data *taos = container_of(work, struct tmd3725_data,
					      work_light);
	int adc = tmd3725_get_lux(taos);
	int cct = tmd3725_get_cct(taos);

	input_report_rel(taos->light_input_dev, REL_MISC, adc + 1);
	input_report_rel(taos->light_input_dev, REL_WHEEL, cct);
	input_sync(taos->light_input_dev);

	if (taos->count_log_time >= LIGHT_LOG_TIME) {
		SENSOR_INFO("R %d, G %d, B %d, C %d, ir %d, lux %d, cct %d\n",
			taos->reddata, taos->grndata, taos->bludata,
			taos->clrdata, taos->irdata, adc, cct);
		taos->count_log_time = 0;
	} else
		taos->count_log_time++;
}

static int tmd3725_prox_get_adc(struct tmd3725_data *taos)
{
	int adc;

	adc = tmd3725_i2c_read(taos, PRX_DATA_HIGH);
	if (adc < 0)
		return MIN_PROX;
	if (adc > MAX_PROX)
		adc = MAX_PROX;

	return adc;
}

static int tmd3725_prox_get_threshold(struct tmd3725_data *taos, u8 buf)
{
	int threshold;

	threshold = tmd3725_i2c_read(taos, buf);
	if (threshold < 0)
		SENSOR_ERR("failed %d\n", threshold);

	return threshold;
}

static void tmd3725_prox_set_threshold(struct tmd3725_data *taos)
{
	u8 prox_int_thresh[2];
	int ret;

	if (taos->prox_state == STATE_CLOSE) {
		/* Low -> threshod_low , High -> 0xff */
		prox_int_thresh[0] = taos->prox_thresh_low;
		prox_int_thresh[1] = 0xFF;
	} else {
		/* Low -> 0, Hight -> threshod_high */
		prox_int_thresh[0] = 0;
		prox_int_thresh[1] = taos->prox_thresh_hi;
	}

	ret = tmd3725_i2c_write(taos, PRX_MINTHRESH, prox_int_thresh[0]);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);

	ret = tmd3725_i2c_write(taos, PRX_MAXTHRESH, prox_int_thresh[1]);
	if (ret < 0)
		SENSOR_ERR("failed %d\n", ret);
}

static void tmd3725_prox_target_initialization(struct tmd3725_data *taos)
{
	int ret;

	SENSOR_INFO("Calibration Start !!!\n");

	ret = tmd3725_i2c_write(taos, ENABLE, PON);
	if (ret < 0)
		SENSOR_ERR("ENABLE failed %d\n", ret);

	ret = tmd3725_i2c_write(taos, INTENAB, CIEN);
	if (ret < 0)
		SENSOR_ERR("INTENAB failed %d\n", ret);

	/* < BINARY SEARCH TARGET > */
	/*
		value   :   TARGET
		0       :      0
		1       :      1
		2       :      3
		3       :      7
		4       :      15
		5       :      31
		6       :      63
		7       :      127
	*/
	ret = tmd3725_i2c_modify_write(taos, CALIBCFG,
		BINSRCH_TARGET, BINARY_SEARCH_SET);
	if (ret < 0)
		SENSOR_ERR("CALIBCFG failed %d\n", ret);

	ret = tmd3725_i2c_write(taos, CALIB , CALIBRATION_SET);
	if (ret < 0)
		SENSOR_ERR("CALIB failed %d\n", ret);

	taos->cal_complete = false;
}

static void tmd3725_prox_report_state(struct work_struct *work)
{
	struct tmd3725_data *taos =
		container_of(work, struct tmd3725_data, work_prox);
	int adc_data;
	int prox_thresh_hi;
	int prox_thresh_low;

	/* change Threshold */
	mutex_lock(&taos->prox_mutex);
	adc_data = tmd3725_prox_get_adc(taos);
	mutex_unlock(&taos->prox_mutex);

	prox_thresh_hi = tmd3725_prox_get_threshold(taos, PRX_MAXTHRESH);
	prox_thresh_low = tmd3725_prox_get_threshold(taos, PRX_MINTHRESH);
	SENSOR_INFO("hi = %d, low = %d, adc_data = %d\n",
		taos->prox_thresh_hi, taos->prox_thresh_low, adc_data);

	if ((taos->prox_state == STATE_FAR)
			&& (adc_data >= taos->prox_thresh_hi)) {
		taos->prox_state = STATE_CLOSE;
		input_report_abs(taos->prox_input_dev,
			ABS_DISTANCE, taos->prox_state);
		input_sync(taos->prox_input_dev);
		SENSOR_INFO("CLOSE adc %d\n", adc_data);
	} else if ((taos->prox_state == STATE_CLOSE)
			&& (adc_data <= (taos->prox_thresh_low))) {
		taos->prox_state = STATE_FAR;
		input_report_abs(taos->prox_input_dev,
			ABS_DISTANCE, taos->prox_state);
		input_sync(taos->prox_input_dev);
		SENSOR_INFO("FAR adc %d\n", adc_data);
	} else {
		SENSOR_ERR("Error Case!adc=[%X], th_high=[%d], th_min=[%d]\n",
			 adc_data, prox_thresh_hi, prox_thresh_low);
	}

	tmd3725_prox_set_threshold(taos);
}

static void tmd3725_work_func_prox(struct work_struct *work)
{
	u8 status;
	int ret;
	struct tmd3725_data *taos =
		container_of(work, struct tmd3725_data, work_prox);

	/* disable INT */
	disable_irq_nosync(taos->irq);

	/* Calibration Handle Event */
	ret = tmd3725_i2c_read(taos, STATUS);
	if (ret < 0) {
		SENSOR_ERR("STATUS failed %d\n", ret);
		goto exit;
	}

	status = (u8)ret;
	if (status & CINT) {
		u8 offset_l = 0;
		u8 offset_h = 0;
		int n_offset;

		SENSOR_INFO("CINT 0x%x\n", status);
		taos->cal_complete = true;

		ret = tmd3725_i2c_modify_write(taos,
			CALIBSTAT, CALIB_FINISHED, CALIB_FINISHED);
		if (ret < 0)
			SENSOR_ERR("CALIBSTAT failed %d\n", ret);

		tmd3725_set_op_mode(taos, MODE_PROX, ON);

		ret = tmd3725_i2c_read(taos, POFFSET_L);
		if (ret < 0)
			SENSOR_ERR("POFFSET_L failed %d\n", ret);
		else
			offset_l = (u8)ret;

		ret = tmd3725_i2c_read(taos, POFFSET_H);
		if (ret < 0)
			SENSOR_ERR("POFFSET_H failed %d\n", ret);
		else
			offset_h = (u8)ret;

		if (offset_h > 0)
			n_offset = offset_l * (-1);
		else
			n_offset = offset_l;

		taos->offset = n_offset;
		SENSOR_INFO("n_offset %d, 0x%x, 0x%x\n",
			n_offset, offset_l, offset_h);
	}

	if (status & ZINT) {
		u8 offset_l = 0;
		u8 offset_h = 0;
		int n_offset;

		SENSOR_INFO("ZINT 0x%x\n", status);

		ret = tmd3725_i2c_read(taos, POFFSET_L);
		if (ret < 0)
			SENSOR_ERR("POFFSET_L failed %d\n", ret);
		else
			offset_l = (u8)ret;

		ret = tmd3725_i2c_read(taos, POFFSET_H);
		if (ret < 0)
			SENSOR_ERR("POFFSET_H failed %d\n", ret);
		else
			offset_h = (u8)ret;

		if (offset_h > 0)
			n_offset = offset_l * (-1);
		else
			n_offset = offset_l;

		SENSOR_INFO("n_offset %d, 0x%x, 0x%x\n",
			n_offset, offset_l, offset_h);

		tmd3725_prox_target_initialization(taos);
	}

	if (status & PINT)
		if (taos->cal_complete == true)
			tmd3725_prox_report_state(work);

exit:
	tmd3725_interrupt_clear(taos);
	/* enable INT */
	enable_irq(taos->irq);
}

static void tmd3725_work_func_prox_avg(struct work_struct *work)
{
	struct tmd3725_data *taos = container_of(work, struct tmd3725_data,
		work_prox_avg);
	int prox_state;
	int min = 0, max = 0, avg = 0;
	int i;

	for (i = 0; i < PROX_AVG_COUNT; i++) {
		mutex_lock(&taos->prox_mutex);
		prox_state = tmd3725_prox_get_adc(taos);
		mutex_unlock(&taos->prox_mutex);
		if (prox_state > MIN_PROX) {
			avg += prox_state;
			if (!i)
				min = prox_state;
			if (prox_state < min)
				min = prox_state;
			if (prox_state > max)
				max = prox_state;
		} else {
			prox_state = MIN_PROX;
		}
		msleep(40);
	}
	avg /= i;
	taos->avg[0] = min;
	taos->avg[1] = avg;
	taos->avg[2] = max;
}

irqreturn_t tmd3725_irq_handler(int irq, void *data)
{
	struct tmd3725_data *taos = data;

	if (taos->irq != -1) {
		wake_lock_timeout(&taos->prx_wake_lock, 3 * HZ);
		queue_work(taos->wq, &taos->work_prox);
	}

	SENSOR_INFO("taos interrupt handler is called\n");
	return IRQ_HANDLED;
}

static enum hrtimer_restart tmd3725_timer_func(struct hrtimer *timer)
{
	struct tmd3725_data *taos =
		container_of(timer, struct tmd3725_data, timer);

	queue_work(taos->wq, &taos->work_light);
	hrtimer_forward_now(&taos->timer, taos->light_poll_delay);

	return HRTIMER_RESTART;
}

static enum hrtimer_restart tmd3725_prox_timer_func(struct hrtimer *timer)
{
	struct tmd3725_data *taos = container_of(timer, struct tmd3725_data,
					prox_avg_timer);
	queue_work(taos->wq_avg, &taos->work_prox_avg);
	hrtimer_forward_now(&taos->prox_avg_timer, taos->prox_polling_time);

	return HRTIMER_RESTART;
}

static ssize_t tmd3725_light_poll_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%lld\n",
		ktime_to_ns(taos->light_poll_delay));
}

static ssize_t tmd3725_light_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = kstrtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	SENSOR_INFO("new delay = %lldns, old delay = %lldns\n",
		new_delay, ktime_to_ns(taos->light_poll_delay));

	mutex_lock(&taos->power_lock);
	if (new_delay != ktime_to_ns(taos->light_poll_delay)) {
		taos->light_poll_delay = ns_to_ktime(new_delay);
		if (taos->power_state & LIGHT_ENABLED) {
			tmd3725_light_disable(taos);
			tmd3725_light_enable(taos);
		}
	}
	mutex_unlock(&taos->power_lock);

	return size;
}

static ssize_t tmd3725_light_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		(taos->power_state & LIGHT_ENABLED) ? 1 : 0);
}

static ssize_t tmd3725_light_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	SENSOR_INFO("new_value = %d, old state = %d\n",
		new_value, (taos->power_state & LIGHT_ENABLED) ? 1 : 0);

	mutex_lock(&taos->power_lock);
	if (new_value && !(taos->power_state & LIGHT_ENABLED)) {
		taos->power_state |= LIGHT_ENABLED;
		tmd3725_set_op_mode(taos, MODE_ALS, ON);
		tmd3725_light_enable(taos);
	} else if (!new_value && (taos->power_state & LIGHT_ENABLED)) {
		tmd3725_light_disable(taos);
		tmd3725_set_op_mode(taos, MODE_ALS, OFF);
		taos->power_state &= ~LIGHT_ENABLED;
	}
	mutex_unlock(&taos->power_lock);
	return size;
}

static ssize_t tmd3725_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		(taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

static ssize_t tmd3725_prox_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	SENSOR_INFO("new_value = %d, old state = %d\n",
	    new_value, (taos->power_state & PROXIMITY_ENABLED) ? 1 : 0);

	mutex_lock(&taos->power_lock);
	if (new_value && !(taos->power_state & PROXIMITY_ENABLED)) {
		tmd3725_prox_vled_onoff(taos, ON);
		tmd3725_prox_target_initialization(taos);
		SENSOR_INFO("th_hi = %d, th_low = %d\n",
			taos->prox_thresh_hi, taos->prox_thresh_low);

		taos->power_state |= PROXIMITY_ENABLED;
		taos->prox_state = STATE_FAR;
		tmd3725_prox_set_threshold(taos);

		input_report_abs(taos->prox_input_dev, ABS_DISTANCE, 1);
		input_sync(taos->prox_input_dev);

		enable_irq(taos->irq);
		enable_irq_wake(taos->irq);
	} else if (!new_value && (taos->power_state & PROXIMITY_ENABLED)) {
		disable_irq_wake(taos->irq);
		disable_irq(taos->irq);

		tmd3725_set_op_mode(taos, MODE_PROX, OFF);
		tmd3725_prox_vled_onoff(taos, OFF);
		taos->power_state &= ~PROXIMITY_ENABLED;
	}
	mutex_unlock(&taos->power_lock);

	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   tmd3725_light_poll_delay_show,
		   tmd3725_light_poll_delay_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       tmd3725_light_enable_show, tmd3725_light_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       tmd3725_prox_enable_show, tmd3725_prox_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static ssize_t get_vendor_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t get_chip_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_NAME);
}

static ssize_t tmd3725_prox_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", tmd3725_prox_get_adc(taos));
}

static ssize_t tmd3725_prox_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
		taos->avg[0], taos->avg[1], taos->avg[2]);
}

static ssize_t tmd3725_prox_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);
	int new_value = 0;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	if (taos->prox_avg_enable == new_value)
		SENSOR_INFO("same status\n");
	else if (new_value == 1) {
		SENSOR_INFO("starting poll timer, delay %lldns\n",
		ktime_to_ns(taos->prox_polling_time));
		hrtimer_start(&taos->prox_avg_timer,
			taos->prox_polling_time, HRTIMER_MODE_REL);
		taos->prox_avg_enable = 1;
	} else {
		SENSOR_INFO("cancelling prox avg poll timer\n");
		hrtimer_cancel(&taos->prox_avg_timer);
		cancel_work_sync(&taos->work_prox_avg);
		taos->prox_avg_enable = 0;
	}

	return size;
}

static ssize_t tmd3725_prox_thresh_high_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", taos->prox_thresh_hi);
}

static ssize_t tmd3725_prox_thresh_high_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);
	int thresh_value, ret;

	ret = kstrtoint(buf, 10, &thresh_value);
	if (ret < 0) {
		SENSOR_ERR("kstrtoint failed. %d", ret);
		return size;
	}

	SENSOR_INFO("thresh_hi = %d\n", thresh_value);
	taos->prox_thresh_hi = thresh_value;
	tmd3725_prox_set_threshold(taos);

	return size;
}
static ssize_t tmd3725_prox_thresh_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", taos->prox_thresh_low);
}

static ssize_t tmd3725_prox_thresh_low_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);
	int thresh_value, ret;

	ret = kstrtoint(buf, 10, &thresh_value);
	if (ret < 0) {
		SENSOR_ERR("kstrtoint failed. %d", ret);
		return size;
	}

	SENSOR_INFO("thresh_low = %d\n", thresh_value);
	taos->prox_thresh_low = thresh_value;
	tmd3725_prox_set_threshold(taos);

	return size;
}

static ssize_t tmd3725_prox_trim_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", taos->offset);
}

static ssize_t tmd3725_prox_trim_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int trim_value, ret;

	ret = kstrtoint(buf, 10, &trim_value);
	if (ret < 0) {
		SENSOR_ERR("kstrtoint failed. %d", ret);
		return size;
	}

	SENSOR_INFO("prox_trim = %d\n", trim_value);

	return size;
}

static DEVICE_ATTR(vendor, S_IRUGO, get_vendor_name, NULL);
static DEVICE_ATTR(name, S_IRUGO, get_chip_name, NULL);
static struct device_attribute dev_attr_proximity_raw_data =
	__ATTR(raw_data, S_IRUGO, tmd3725_prox_state_show, NULL);
static DEVICE_ATTR(prox_avg, S_IRUGO | S_IWUSR | S_IWGRP,
	tmd3725_prox_avg_show, tmd3725_prox_avg_store);
static DEVICE_ATTR(state, S_IRUGO, tmd3725_prox_state_show, NULL);
static DEVICE_ATTR(thresh_high, S_IRUGO | S_IWUSR | S_IWGRP,
	tmd3725_prox_thresh_high_show, tmd3725_prox_thresh_high_store);
static DEVICE_ATTR(thresh_low, S_IRUGO | S_IWUSR | S_IWGRP,
	tmd3725_prox_thresh_low_show, tmd3725_prox_thresh_low_store);
static DEVICE_ATTR(prox_trim, S_IRUGO | S_IWUSR | S_IWGRP,
	tmd3725_prox_trim_show, tmd3725_prox_trim_store);

static struct device_attribute *prox_sensor_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_state,
	&dev_attr_prox_avg,
	&dev_attr_proximity_raw_data,
	&dev_attr_thresh_high,
	&dev_attr_thresh_low,
	&dev_attr_prox_trim,
	NULL
};

static ssize_t tmd3725_light_get_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", tmd3725_get_lux(taos));
}

static ssize_t tmd3725_light_get_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd3725_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u,%u,%u,%u\n",
		taos->reddata, taos->grndata, taos->bludata, taos->clrdata);
}

static DEVICE_ATTR(adc, S_IRUGO, tmd3725_light_get_lux_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO, tmd3725_light_get_lux_show, NULL);
static struct device_attribute dev_attr_light_raw_data =
	__ATTR(raw_data, S_IRUGO, tmd3725_light_get_raw_data_show, NULL);

static struct device_attribute *lightsensor_additional_attributes[] = {
	&dev_attr_adc,
	&dev_attr_lux,
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_light_raw_data,
	NULL
};

static int tmd3725_setup_irq(struct tmd3725_data *taos)
{
	int rc = -EIO;
	int irq;

	rc = gpio_request(taos->prox_irq_gpio, "gpio_proximity_irq");
	if (rc < 0) {
		SENSOR_ERR("gpio %d request failed %d\n",
			taos->prox_irq_gpio, rc);
		return rc;
	}

	rc = gpio_direction_input(taos->prox_irq_gpio);
	if (rc < 0) {
		SENSOR_ERR("failed to set gpio %d as input %d\n",
			taos->prox_irq_gpio, rc);
		goto err_gpio_direction_input;
	}

	irq = gpio_to_irq(taos->prox_irq_gpio);

	rc = request_threaded_irq(irq, NULL, tmd3725_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "proximity_int", taos);
	if (rc < 0) {
		SENSOR_ERR("request_irq %d failed for gpio %d %d\n",
			irq, taos->prox_irq_gpio, rc);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	disable_irq(irq);
	taos->irq = irq;

	goto done;

err_request_irq:
err_gpio_direction_input:
	gpio_free(taos->prox_irq_gpio);
done:
	return rc;
}

static int tmd3725_parse_dt(struct tmd3725_data *taos, struct device *dev)
{
	int ret = 0;
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;


	taos->prox_irq_gpio = of_get_named_gpio_flags(np,
				"taos,irq_gpio", 0, &flags);
	if (taos->prox_irq_gpio < 0) {
		SENSOR_ERR("fail to get prox_irq_gpio\n");
		return -ENODEV;
	}

	taos->vled_ldo_pin = of_get_named_gpio_flags(np,
				"taos,vled_ldo_pin", 0, &flags);
	if (taos->vled_ldo_pin < 0) {
		SENSOR_ERR("fail to get vled_ldo_pin\n");
		taos->vled_ldo_pin = 0;
	} else {
		ret = gpio_request(taos->vled_ldo_pin, "prox_vled_en");
		if (ret < 0)
			SENSOR_ERR("gpio %d request failed %d\n",
				taos->vled_ldo_pin, ret);
		else
			gpio_direction_output(taos->vled_ldo_pin, 0);
	}

	if (of_property_read_u32(np, "taos,prox_thresh_hi",
		&taos->prox_thresh_hi) < 0)
		taos->prox_thresh_hi = PROX_THRESH_HI_DEFAULT;
	if (of_property_read_u32(np, "taos,prox_thresh_low",
		&taos->prox_thresh_low) < 0)
		taos->prox_thresh_low = PROX_THRESH_LOW_DEFAULT;
	if (of_property_read_u32(np, "taos,als_time", &taos->als_time) < 0)
		taos->als_time = ALS_TIME_DEFAULT;
	if (of_property_read_u32(np, "taos,intr_filter",
		&taos->intr_filter) < 0)
		taos->intr_filter = INTR_FILTER_DEFAULT;
	if (of_property_read_u32(np, "taos,prox_pulsecnt",
		&taos->prox_pulsecnt) < 0)
		taos->prox_pulsecnt = PROX_PULSECNT_DEFAULT;
	if (of_property_read_u32(np, "taos,als_gain", &taos->als_gain) < 0)
		taos->als_gain = ALS_GAIN_DEFAULT;
	if (of_property_read_u32(np, "taos,lux_multiple",
		&taos->lux_multiple) < 0)
		taos->lux_multiple = LUX_MULTIPLE_DEFAULT;

	SENSOR_INFO("th_hi:%d, th_l:%d, at:%d, if:%d, ppc:%d, ag:%d, lm:%d\n",
		taos->prox_thresh_hi, taos->prox_thresh_low, taos->als_time,
		taos->intr_filter, taos->prox_pulsecnt,
		taos->als_gain, taos->lux_multiple);

	if (of_property_read_u32(np, "taos,dgf", &taos->dgf) < 0)
		taos->dgf = DGF_DEFAULT;
	if (of_property_read_u32(np, "taos,cct_coef", &taos->cct_coef) < 0)
		taos->cct_coef = CT_COEF1_DEFAULT;
	if (of_property_read_u32(np, "taos,cct_offset", &taos->cct_offset) < 0)
		taos->cct_offset = CT_OFFSET1_DEFAULT;
	if (of_property_read_u32(np, "taos,coef_r", &taos->coef_r) < 0)
		taos->coef_r = R_COEF1_DEFAULT;
	if (of_property_read_u32(np, "taos,coef_g", &taos->coef_g) < 0)
		taos->coef_g = G_COEF1_DEFAULT;
	if (of_property_read_u32(np, "taos,coef_b", &taos->coef_b) < 0)
		taos->coef_b = B_COEF1_DEFAULT;

	SENSOR_INFO("dgf:%d, cctcoef:%d, cctoff:%d, c_r:%d, c_g:%d, c_b:%d\n",
		taos->dgf, taos->cct_coef, taos->cct_offset, taos->coef_r,
		taos->coef_g, taos->coef_b);

	return 0;
}

static int tmd3725_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct tmd3725_data *taos;

	SENSOR_INFO("Start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_ERR("i2c functionality check failed!\n");
		return ret;
	}

	taos = kzalloc(sizeof(struct tmd3725_data), GFP_KERNEL);
	if (!taos) {
		SENSOR_ERR("failed to alloc memory for module data\n");
		ret = -ENOMEM;
		goto done;
	}

	if (client->dev.of_node) {
		ret = tmd3725_parse_dt(taos, &client->dev);
		if (ret < 0)
			goto err_tmd3725_data_free;
	}

	taos->i2c_client = client;
	i2c_set_clientdata(client, taos);
	taos->op_mode_state = 0;
	taos->prox_avg_enable = 0;
	taos->vled = NULL;

	/* ID Check */
	ret = i2c_smbus_read_byte_data(client, CMD_REG | CHIPID);
	if (ret < 0) {
		SENSOR_ERR("i2c read error %d\n", ret);
		goto err_chip_id_or_i2c_error;
	} else if (ret != TMD3725_CHIP_ID) {
		SENSOR_ERR("chip id error 0x[%X]\n", ret);
		ret = -ENXIO;
		goto err_chip_id_or_i2c_error;
	}

	/* wake lock init */
	wake_lock_init(&taos->prx_wake_lock, WAKE_LOCK_SUSPEND,
		"prx_wake_lock");
	mutex_init(&taos->prox_mutex);
	mutex_init(&taos->power_lock);

	/* allocate proximity input_device */
	taos->prox_input_dev = input_allocate_device();
	if (!taos->prox_input_dev) {
		ret = -ENOMEM;
		SENSOR_ERR("could not allocate input device\n");
		goto err_input_allocate_device_proximity;
	}

	input_set_drvdata(taos->prox_input_dev, taos);
	taos->prox_input_dev->name = MODULE_NAME_PROX;
	input_set_capability(taos->prox_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(taos->prox_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(taos->prox_input_dev);
	if (ret < 0) {
		SENSOR_ERR("could not register input device\n");
		input_free_device(taos->prox_input_dev);
		goto err_input_register_device_proximity;
	}
	ret = sensors_register(taos->prox_dev, taos, prox_sensor_attrs,
		MODULE_NAME_PROX); /* factory attributs */
	if (ret < 0) {
		SENSOR_ERR("could not registersensors_register\n");
		goto err_sensor_register_device_proximity;
	}
	ret = sensors_create_symlink(&taos->prox_input_dev->dev.kobj,
		taos->prox_input_dev->name);
	if (ret < 0) {
		SENSOR_ERR("fail: sensors_create_symlink\n");
		goto err_symlink_device_proximity;
	}
	ret = sysfs_create_group(&taos->prox_input_dev->dev.kobj,
		&proximity_attribute_group);
	if (ret < 0) {
		SENSOR_ERR("could not create sysfs group\n");
		goto err_create_sensorgoup_proximity;
	}

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&taos->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	taos->timer.function = tmd3725_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	taos->wq = create_singlethread_workqueue("tmd3725_wq");
	if (!taos->wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create workqueue\n");
		goto err_create_workqueue;
	}

	taos->wq_avg = create_singlethread_workqueue("tmd3725_wq_avg");
	if (!taos->wq_avg) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create workqueue\n");
		goto err_create_avg_workqueue;
	}

	/* this is the thread function we run on the work queue */
	INIT_WORK(&taos->work_light, tmd3725_work_func_light);
	INIT_WORK(&taos->work_prox, tmd3725_work_func_prox);
	INIT_WORK(&taos->work_prox_avg, tmd3725_work_func_prox_avg);

	hrtimer_init(&taos->prox_avg_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos->prox_polling_time = ns_to_ktime(2000 * NSEC_PER_MSEC);
	taos->prox_avg_timer.function = tmd3725_prox_timer_func;

	/* allocate lightsensor-level input_device */
	taos->light_input_dev = input_allocate_device();
	if (!taos->light_input_dev) {
		SENSOR_ERR("could not allocate input device\n");
		ret = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(taos->light_input_dev, taos);
	taos->light_input_dev->name = MODULE_NAME_LIGHT;
	input_set_capability(taos->light_input_dev, EV_REL, REL_MISC);
	input_set_capability(taos->light_input_dev, EV_REL, REL_WHEEL);

	SENSOR_INFO("registering lightsensor-level input device\n");
	ret = input_register_device(taos->light_input_dev);
	if (ret < 0) {
		SENSOR_ERR("could not register input device\n");
		input_free_device(taos->light_input_dev);
		goto err_input_register_device_light;
	}
	ret = sensors_register(taos->light_dev, taos,
		lightsensor_additional_attributes, MODULE_NAME_LIGHT);
	if (ret < 0) {
		SENSOR_ERR("cound not register light sensor device %d\n",
			ret);
		goto err_sensor_register_device_light;
	}

	ret = sensors_create_symlink(&taos->light_input_dev->dev.kobj,
		taos->light_input_dev->name);
	if (ret < 0) {
		SENSOR_ERR("cound not sensors_create_symlink %d.\n", ret);
		goto err_symlink_device_light;
	}

	ret = sysfs_create_group(&taos->light_input_dev->dev.kobj,
		&light_attribute_group);
	if (ret < 0) {
		SENSOR_ERR("could not create sysfs group\n");
		goto err_create_sensorgoup_light;
	}

	/* setup irq */
	ret = tmd3725_setup_irq(taos);
	if (ret < 0) {
		SENSOR_ERR("could not setup irq\n");
		goto err_setup_irq;
	}

	tmd3725_chip_init(taos);

	SENSOR_INFO("success\n");
	return ret;

err_setup_irq:
	sysfs_remove_group(&taos->light_input_dev->dev.kobj,
		&light_attribute_group);
err_create_sensorgoup_light:
	sensors_remove_symlink(&taos->light_input_dev->dev.kobj,
		taos->prox_input_dev->name);
err_symlink_device_light:
	sensors_unregister(taos->light_dev, lightsensor_additional_attributes);
err_sensor_register_device_light:
	input_unregister_device(taos->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(taos->wq_avg);
err_create_avg_workqueue:
	destroy_workqueue(taos->wq);
err_create_workqueue:
	sysfs_remove_group(&taos->prox_input_dev->dev.kobj,
		&proximity_attribute_group);
err_create_sensorgoup_proximity:
	sensors_remove_symlink(&taos->prox_input_dev->dev.kobj,
		taos->prox_input_dev->name);
err_symlink_device_proximity:
	sensors_unregister(taos->prox_dev, prox_sensor_attrs);
err_sensor_register_device_proximity:
	input_unregister_device(taos->prox_input_dev);
err_input_register_device_proximity:
err_input_allocate_device_proximity:
	mutex_destroy(&taos->power_lock);
	mutex_destroy(&taos->prox_mutex);
	wake_lock_destroy(&taos->prx_wake_lock);
err_chip_id_or_i2c_error:
	if (taos->vled_ldo_pin)
		gpio_free(taos->vled_ldo_pin);
err_tmd3725_data_free:
	kfree(taos);
done:
	SENSOR_ERR("failed %d\n", ret);
	return ret;
}

static void tmd3725_i2c_shutdown(struct i2c_client *client)
{
	struct tmd3725_data *taos = i2c_get_clientdata(client);

	if (taos->power_state & LIGHT_ENABLED) {
		tmd3725_light_disable(taos);
		tmd3725_set_op_mode(taos, MODE_ALS, OFF);
		taos->power_state &= ~LIGHT_ENABLED;
	}

	if (taos->power_state & PROXIMITY_ENABLED) {
		disable_irq_wake(taos->irq);
		disable_irq(taos->irq);

		tmd3725_set_op_mode(taos, MODE_PROX, OFF);
		tmd3725_prox_vled_onoff(taos, OFF);
		taos->power_state &= ~PROXIMITY_ENABLED;
	}

	SENSOR_INFO("is called\n");
}

static int tmd3725_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd3725_data *taos = i2c_get_clientdata(client);

	if (taos->power_state & LIGHT_ENABLED) {
		SENSOR_INFO("is called\n");
		tmd3725_light_disable(taos);
		tmd3725_set_op_mode(taos, MODE_ALS, OFF);
	}

	if (taos->power_state & PROXIMITY_ENABLED) {
		SENSOR_INFO("is called\n");
		disable_irq(taos->irq);
	}

	return 0;
}

static int tmd3725_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmd3725_data *taos = i2c_get_clientdata(client);

	if (taos->power_state & LIGHT_ENABLED) {
		SENSOR_INFO("is called\n");
		tmd3725_light_enable(taos);
		tmd3725_set_op_mode(taos, MODE_ALS, ON);
	}

	if (taos->power_state & PROXIMITY_ENABLED) {
		SENSOR_INFO("is called\n");
		enable_irq(taos->irq);
	}
	return 0;
}

static int tmd3725_i2c_remove(struct i2c_client *client)
{
	SENSOR_INFO("\n");
	return 0;
}

static const struct i2c_device_id tmd3725_device_id[] = {
	{ CHIP_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tmd3725_device_id);

static const struct dev_pm_ops tmd3725_pm_ops = {
	.suspend = tmd3725_suspend,
	.resume = tmd3725_resume
};

#ifdef CONFIG_OF
static struct of_device_id tm3725_match_table[] = {
	{ .compatible = "taos,tmd3725",},
};
#else
#define tm3725_match_table NULL
#endif

static struct i2c_driver tmd3725_i2c_driver = {
	.driver = {
		.name = CHIP_NAME,
		.owner = THIS_MODULE,
		.pm = &tmd3725_pm_ops,
		.of_match_table = tm3725_match_table,
	},
	.probe = tmd3725_i2c_probe,
	.remove = tmd3725_i2c_remove,
	.shutdown = tmd3725_i2c_shutdown,
	.id_table = tmd3725_device_id,
};


static int __init tmd3725_init(void)
{
	return i2c_add_driver(&tmd3725_i2c_driver);
}

static void __exit tmd3725_exit(void)
{
	i2c_del_driver(&tmd3725_i2c_driver);
}

module_init(tmd3725_init);
module_exit(tmd3725_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for tmd3725");
MODULE_LICENSE("GPL");
