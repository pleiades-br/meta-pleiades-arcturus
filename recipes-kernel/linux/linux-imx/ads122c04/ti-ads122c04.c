// SPDX-License-Identifier: MIT License
/*
 * IIO driver for ADS122C04 chip
 *
 *
 * Author: Aluisio Leonello Victal <alvictal@gmail.com>
 *
 * Datasheet for ADS122C04 can be found here:
 * https://www.ti.com/lit/ds/symlink/ads122c04.pdf
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/property.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define ADS122C04_DRV_NAME "ti-ads122c04"

#define ADS122C04_CHANNELS 12
#define ADS122C04_VREF_INTERNAL_REF_IN 2048 /* in mv */

#define FIXED_POINT_SHIFT                20  // Using 20 bits for decimal part
#define FIXED_POINT_SCALE               (1 << FIXED_POINT_SHIFT)
#define ADS122C04_LSB_CALC_CONST         0xFFFFFF /* 2^24 */
#define ADS122C04_DRDY_TRIES             5


#define ADS122C04_POWERDOWN	    0x02    /* 0000 001X */
#define ADS122C04_RESET_CFGS    0x06    /* 0000 011X */
#define ADS122C04_STARTSYNC	    0x08    /* 0000 100X */
#define ADS122C04_RDATA	        0x10    /* 0001 XXXX */
#define ADS122C04_RREG_CGF0_REG	0x20    /* 0010 00XX */
#define ADS122C04_RREG_CGF1_REG	0x24    /* 0010 01XX */
#define ADS122C04_RREG_CGF2_REG	0x28    /* 0010 10XX */
#define ADS122C04_RREG_CGF3_REG	0x2c    /* 0010 11XX */
#define ADS122C04_WREG_CGF0_REG	0x40    /* 0100 00XX */
#define ADS122C04_WREG_CGF1_REG	0x44    /* 0100 01XX */
#define ADS122C04_WREG_CGF2_REG	0x48    /* 0100 10XX */
#define ADS122C04_WREG_CGF3_REG	0x4c    /* 0100 11XX */



#define ADS122C04_CFG0_GAIN_SHIFT   1
#define ADS122C04_CFG0_MUX_SHIFT    4

#define ADS122C04_CFG0_PGA_MASK     BIT(0)
#define ADS122C04_CFG0_GAIN_MASK    GENMASK(3, 1)
#define ADS122C04_CFG0_MUX_MASK     GENMASK(7, 4)


#define ADS122C04_CFG1_VREF_SHIFT   1
#define ADS122C04_CFG1_CM_SHIFT     3
#define ADS122C04_CFG1_MODE_SHIFT   4
#define ADS122C04_CFG1_DR_SHIFT     5

#define ADS122C04_CFG1_TS_MASK      BIT(0)
#define ADS122C04_CFG1_VREF_MASK    GENMASK(2, 1)
#define ADS122C04_CFG1_CM_MASK      BIT(3)
#define ADS122C04_CFG1_MODE_MASK    BIT(4)
#define ADS122C04_CFG1_DR_MASK      GENMASK(7, 5)


#define ADS122C04_CFG2_BCS_SHIFT   3
#define ADS122C04_CFG2_CRC_SHIFT   4
#define ADS122C04_CFG2_DCNT_SHIFT  6
#define ADS122C04_CFG2_DRDY_SHIFT  7

#define ADS122C04_CFG2_IDAC_MASK   GENMASK(2, 0)
#define ADS122C04_CFG2_BCS_MASK    BIT(3)
#define ADS122C04_CFG2_CRC_MASK    GENMASK(5, 4)
#define ADS122C04_CFG2_DCNT_MASK   BIT(6)
#define ADS122C04_CFG2_DRDY_MASK   BIT(7)


#define ADS122C04_CFG3_I2MUX_SHIFT  2
#define ADS122C04_CFG3_I1MUX_SHIFT  5

#define ADS122C04_CFG3_I2MUX_MASK   GENMASK(4, 2)
#define ADS122C04_CFG3_I1MUX_MASK   GENMASK(7, 5)


#define CHANNEL_DISABLED                0
#define CHANNEL_ENABLED                 1

#define ADS122C04_PGA_ON                0    
#define ADS122C04_PGA_OFF               1 

#define ADS122C04_TURBO_MODE_OFF        0    
#define ADS122C04_TURBO_MODE_ON         1

#define ADS122C04_TEMPERATURE_MODE_OFF  0    
#define ADS122C04_TEMPERATURE_MODE_ON   1    

#define ADS122C04_CONV_MODE_SINGLE      0
#define ADS122C04_CONV_MODE_CONTINUES   1

#define ADS122C04_BURN_OUT_MODE_OFF     0
#define ADS122C04_BURN_OUT_MODE_ON      1


#define ADS122C04_DEFAULT_PGA                   ADS122C04_PGA_ON
#define ADS122C04_DEFAULT_GAIN                  0 /*1*/        
#define ADS122C04_DEFAULT_DATA_RATE             0 /*20*/
#define ADS122C04_DEFAULT_TURBO_MODE            ADS122C04_TURBO_MODE_OFF
#define ADS122C04_DEFAULT_CONV_MODE             ADS122C04_CONV_MODE_SINGLE
#define ADS122C04_DEFAULT_TEMPERATURE_MODE      ADS122C04_TEMPERATURE_MODE_OFF
#define ADS122C04_DEFAULT_MAIN_VREF_REFERENCE   ADS122C04_VREF_INTERNAL

enum chip_ids {
	ADS122C04 = 0,
    ADSXXXXXX,
};

enum ads122c04_channels {
	ADS122C04_AIN0_AIN1 = 0,
	ADS122C04_AIN0_AIN2,
	ADS122C04_AIN0_AIN3,
    ADS122C04_AIN1_AIN0,
    ADS122C04_AIN1_AIN2,
    ADS122C04_AIN1_AIN3,
    ADS122C04_AIN2_AIN3,
    ADS122C04_AIN3_AIN2,
    ADS122C04_AIN0_AVSS,
    ADS122C04_AIN1_AVSS,
    ADS122C04_AIN2_AVSS,
    ADS122C04_AIN3_AVSS,
    ADS122C04_VREF_MON, 
    ADS122C04_AVDD_AVSS,
    ADS122C04_AINP_AINN_SHORTED, 
    ADS122C04_RESERVED, 
};

enum ads122c04_vref {
    ADS122C04_VREF_INTERNAL = 0,
    ADS122C04_VREF_EXTERNAL,
    ADS122C04_VREF_ANALOG1,
    ADS122C04_VREF_ANALOG2,
};

enum ads122c04_integrity_check {
    ADS122C04_ICHECK_DISABLED = 0,
    ADS122C04_ICHECK_INVERTED,
    ADS122C04_ICHECK_CRC16,
    ADS122C04_ICHECK_RESERVED,
};

enum ads122c04_idac_rounting {
    ADS122C04_IDAC_DISABLED = 0,
    ADS122C04_IDAC_AIN0,
    ADS122C04_IDAC_AIN1,
    ADS122C04_IDAC_AIN2,
    ADS122C04_IDAC_AIN3,
    ADS122C04_IDAC_REFP,
    ADS122C04_IDAC_REFN,
    ADS122C04_IDAC_RESERVED,
};


static const unsigned int ads122c04_gain_cfg[] = {
	1, 2, 4, 8, 16, 32 , 64, 128
};

static const unsigned int ads122c04_data_rate[] = {
	20, 45, 90, 175, 330, 600, 1000
};

static const unsigned int ads122c04_data_rate_turbo[] = {
	40, 90, 180, 350, 660, 1200, 2000
};

static const unsigned int ads122c04_idac_current[] = {
	0, 10, 50, 100, 250, 500, 1000, 1500
};

#define ADS122C04_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
                BIT(IIO_CHAN_INFO_SCALE) |          \
                BIT(IIO_CHAN_INFO_SAMP_FREQ) |      \
                BIT(IIO_CHAN_INFO_HARDWAREGAIN),    \
	.scan_index = _addr,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 32,				\
		.endianness = IIO_CPU,			\
	},							        \
	.datasheet_name = "AIN"#_chan"-AVSS",		\
}

#define ADS122C04_DIFF_CHAN(_chan, _chan2, _addr) {	\
	.type = IIO_VOLTAGE,				\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
    	        BIT(IIO_CHAN_INFO_SCALE) |          \
                BIT(IIO_CHAN_INFO_SAMP_FREQ) |      \
                BIT(IIO_CHAN_INFO_HARDWAREGAIN),    \
	.scan_index = _addr,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 24,					\
		.storagebits = 32,				\
		.endianness = IIO_CPU,			\
	},							        \
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}


struct ads122c04_channel_data {
	bool enabled; /* is channel enabled */
    u8 gain; /* gain configuration */
    bool pga_enabled;
    u8 data_rate;
    bool turbo_mode; /* false - normal mode; true -  Turbo mode*/
    bool conv_mode; /* false - single ; true - continues*/
    bool temperature_mode;
    u8 vref_type; 
};


struct ads122c04_st {
	struct i2c_client *client;
	/*
	 * Protects ADC ops, e.g: concurrent sysfs/buffered
	 * data reads, configuration updates
	 */
	struct mutex lock;
    uint32_t vref_mv;

	struct ads122c04_channel_data channel_data[ADS122C04_CHANNELS];
    
    bool counter_enabled;    
    u8 crc_check;
    bool burnout;
    u8 idac;

    u8 idac1_mux;
    u8 idac2_mux;
};

static int32_t offset_calibration = 0;

static const struct iio_chan_spec ads122c04_channels[] = {
	ADS122C04_DIFF_CHAN(0, 1, ADS122C04_AIN0_AIN1),
    ADS122C04_DIFF_CHAN(0, 2, ADS122C04_AIN0_AIN2),
    ADS122C04_DIFF_CHAN(0, 3, ADS122C04_AIN0_AIN3),
    ADS122C04_DIFF_CHAN(1, 0, ADS122C04_AIN1_AIN0),
    ADS122C04_DIFF_CHAN(1, 2, ADS122C04_AIN1_AIN2),
    ADS122C04_DIFF_CHAN(1, 3, ADS122C04_AIN1_AIN3),
    ADS122C04_DIFF_CHAN(2, 3, ADS122C04_AIN2_AIN3),
    ADS122C04_DIFF_CHAN(3, 2, ADS122C04_AIN3_AIN2),
	ADS122C04_CHAN(0, ADS122C04_AIN0_AVSS),
	ADS122C04_CHAN(1, ADS122C04_AIN1_AVSS),
	ADS122C04_CHAN(2, ADS122C04_AIN2_AVSS),
	ADS122C04_CHAN(3, ADS122C04_AIN3_AVSS),
};


static int ads122c04_write_byte(const struct ads122c04_st *st, const u8 cmd) 
{
    int ret = i2c_smbus_write_byte(st->client, cmd);

    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to write configuration: %d cmd 0x%x\n", ret, cmd);
        return ret;
    }

    return 0;
}


static int ads122c04_write_byte_data(const struct ads122c04_st *st, const u8 cmd, const u8 *value) 
{
    int ret = i2c_smbus_write_byte_data(st->client, cmd, *value);
    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to write data. Ret: %d; cmd: 0x%x; byte: \
                0x%x;\n", ret, cmd, *value);
        return ret;
    }

    return 0;
}

static int ads122c04_read_reg_value(const struct ads122c04_st *st, const u8 reg)
{
    if (reg < ADS122C04_RREG_CGF0_REG || reg > ADS122C04_RREG_CGF3_REG) 
        return -EINVAL;

    return  i2c_smbus_read_byte_data(st->client, reg);
}


static int ads122c04_check_data_drdy(const struct ads122c04_st *st)
{
    int tries = 0;
    u8 drdy = 0;
        
    while(!(drdy & 0x80) && (++tries <= ADS122C04_DRDY_TRIES)) {
        drdy = ads122c04_read_reg_value(st, ADS122C04_RREG_CGF2_REG);

        /* Wait a bit and check if data is ready to collect*/
        msleep(200); 
    }

    if (tries > ADS122C04_DRDY_TRIES) {
        dev_err(&st->client->dev, "Data is not getting ready to collect\n");
        return -EBUSY;        
    }

    return 0; 
}



static int ads122c04_read_adc(const struct ads122c04_st *st, int *value)
{
    
    int ret;
    int32_t adc_value = 0;

    u8 data[3] = {0}; 

    ret = ads122c04_check_data_drdy(st);
    if (ret < 0) 
        return ret;
    
    // Read the ADC result from the data registers of ADS122C04
    ret = i2c_smbus_read_i2c_block_data(st->client, ADS122C04_RDATA, 3, data);
    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to read ADC result: %d\n", ret);
        return ret;
    }

    // Combine the 3 bytes into a 24-bit signed integer
    adc_value = (int32_t)((data[0] << 16) | (data[1] << 8) | data[2]);

    // Check if the result is negative (sign-extend to 32 bits)
    if (adc_value & 0x800000) {
        adc_value |= 0xFF000000;  // Sign-extend the 24-bit result to 32-bit
    }
   
    *value = adc_value;

    return 0;
}


static void ads122c04_clear_cfg(const struct ads122c04_st *st)
{
    ads122c04_write_byte(st, ADS122C04_RESET_CFGS);    
}


static int ads122c04_send_start_command(const struct ads122c04_st *st)
{
    return ads122c04_write_byte(st, ADS122C04_STARTSYNC);
}


static int ads122c04_write_cfg0(const struct ads122c04_st *st, const uint8_t channel,
                            const uint8_t gain, const bool pga)
{
    u8 cfg = 0;

    cfg = channel << ADS122C04_CFG0_MUX_SHIFT |
        gain << ADS122C04_CFG0_GAIN_SHIFT |
        pga;

    return ads122c04_write_byte_data(st, ADS122C04_WREG_CGF0_REG, &cfg);
}


static int ads122c04_write_cfg1(const struct ads122c04_st *st, const uint8_t data_rate, 
                                const bool turbo_mode, const bool conv_mode, 
                                const uint8_t vref_type, const bool temperature_mode)
{
    u8 cfg = 0;

    cfg = data_rate << ADS122C04_CFG1_DR_SHIFT | 
        turbo_mode << ADS122C04_CFG1_MODE_SHIFT | 
        conv_mode << ADS122C04_CFG1_CM_SHIFT |
        vref_type << ADS122C04_CFG1_VREF_SHIFT |
        temperature_mode;

    return ads122c04_write_byte_data(st, ADS122C04_WREG_CGF1_REG, &cfg);
}

static int ads122c04_write_cfg2(const struct ads122c04_st *st, const bool counter_enabled,
                                const uint8_t crc_check, const bool burnout, 
                                const uint8_t idac_config)
{
    u8 cfg = 0;
    
    cfg = counter_enabled << ADS122C04_CFG2_DCNT_SHIFT | 
        crc_check << ADS122C04_CFG2_CRC_SHIFT | 
        burnout << ADS122C04_CFG2_BCS_SHIFT |
        idac_config;

    return ads122c04_write_byte_data(st, ADS122C04_WREG_CGF2_REG, &cfg);
}


static int ads122c04_write_cfg3(const struct ads122c04_st *st, const uint8_t idac1_mux,
                                const uint8_t idac2_mux)
{ 
    u8 cfg = 0;

    cfg = idac1_mux << ADS122C04_CFG3_I1MUX_SHIFT | 
        idac2_mux << ADS122C04_CFG3_I2MUX_SHIFT;

    return ads122c04_write_byte_data(st, ADS122C04_WREG_CGF3_REG, &cfg);
}

static int ads122c04_calibrate_offset(const struct ads122c04_st *st, const uint8_t gain, 
                                       const uint8_t pga)
{
    int ret;
    int i =0;
    int32_t val;
    unsigned int tentatives = 0;
   

    ret = ads122c04_write_cfg0(st, 
                            ADS122C04_AINP_AINN_SHORTED,
                            gain,
                            pga);
    if (ret) {
        dev_err(&st->client->dev, "Failed to write register 0. Err %d", ret);
        return ret;
    }

    offset_calibration = 0;
    for (i = 0; i < 10; i++) {
        val = 0;
        /* Start/Sync depends of the convertion mode and the
        Datasheet recomends send de STARTSYNC cmd when the cfg1 is modified */
	    if (ads122c04_send_start_command(st)) {
            continue;
        }

        if (ads122c04_read_adc(st, &val)) {
            continue;
        }

        tentatives += 1;
	    offset_calibration += val;
    }

    if (offset_calibration == 0)
        return 0;

    offset_calibration = (int32_t) div_s64(offset_calibration, tentatives);
    return 0;
}


static int ads122c04_get_special_vref_monitor(const struct ads122c04_st *st, const int mux, 
                                                int32_t *val)
{
    int ret;
    int vref_type = ADS122C04_VREF_INTERNAL;
    int32_t adc_value;
    
    if ((mux < ADS122C04_VREF_MON)|| (mux > ADS122C04_AVDD_AVSS))
        return -EINVAL;

    /* According to Texas Instrument to measure the VREF_MON the Voltage reference must be
    configured as external: link
    https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/1064654/ads122c04-system-monitor-issue---when-monitoring-the-external-reference-voltage-source-mux-3-0-1100/3939126*/
    if (mux == ADS122C04_VREF_MON) 
        vref_type = ADS122C04_VREF_EXTERNAL;
    
    ret = ads122c04_write_cfg1(st, 
                            ADS122C04_DEFAULT_DATA_RATE, 
                            ADS122C04_DEFAULT_TURBO_MODE,
                            ADS122C04_DEFAULT_CONV_MODE,
                            vref_type,
                            ADS122C04_DEFAULT_TEMPERATURE_MODE);
    if (ret) {
         dev_err(&st->client->dev,"Failed to write register 1. Err %d", ret);
        return ret;
    }

    ret = ads122c04_write_cfg2(st,
                            st->counter_enabled,
                            st->crc_check,
                            st->burnout,
                            st->idac);
    if (ret) {
         dev_err(&st->client->dev, "Failed to write register 2. Err %d", ret);
        return ret;
    }

    ret = ads122c04_write_cfg3(st,
                            st->idac1_mux,
                            st->idac2_mux);
    if (ret) {
        dev_err(&st->client->dev, "Failed to write register 3. Err %d", ret);
        return ret;
    }

    ads122c04_calibrate_offset(st, ADS122C04_DEFAULT_GAIN, ADS122C04_PGA_ON);

    ret = ads122c04_write_cfg0(st, mux, ADS122C04_DEFAULT_GAIN, ADS122C04_PGA_ON);
    if (ret) {
        dev_err(&st->client->dev, "Failed to write register 0. Err %d", ret);
        return ret;
    }

    /* Start/Sync depends of the convertion mode and the
       Datasheet recomends send de STARTSYNC cmd when the cfg1 is modified */
    ret = ads122c04_send_start_command(st);
	if (ret) {
         dev_err(&st->client->dev, "Failed to send a start command on Special Mux %d. Err %d", mux ,ret);
		return ret;
    }

    ret = ads122c04_read_adc(st, &adc_value);
	if (ret) {
        dev_err(&st->client->dev, "Failed to get adc value %d. Err %d", mux ,ret);
		return ret;
    }

    *val = adc_value;
    printk(KERN_INFO "ADS122c04: ads122c04_get_special_vref_monitor--- \
        adc_value:%d offset_calibration:%d\n" , adc_value, offset_calibration);
	return 0;
}


static int ads122c04_get_chan_adc_result(const struct ads122c04_st *st, const int channel, 
                                        int32_t *val)
{
    int ret;
    int32_t adc_value = 0;

	if (channel < 0 || channel >= ADS122C04_CHANNELS)
		return -EINVAL;

    ret = ads122c04_write_cfg1(st, 
                            st->channel_data[channel].data_rate,
                            st->channel_data[channel].turbo_mode,
                            st->channel_data[channel].conv_mode,
                            st->channel_data[channel].vref_type,
                            st->channel_data[channel].temperature_mode);
    if (ret) {
         dev_err(&st->client->dev, "Failed to write register 1. Err %d", ret);
        return ret;
    }

    ret = ads122c04_write_cfg2(st,
                            st->counter_enabled,
                            st->crc_check,
                            st->burnout,
                            st->idac);
    if (ret) {
         dev_err(&st->client->dev, "Failed to write register 2. Err %d", ret);
        return ret;
    }

    ret = ads122c04_write_cfg3(st,
                            st->idac1_mux,
                            st->idac2_mux);
    if (ret) {
         dev_err(&st->client->dev,"Failed to write register 3. Err %d", ret);
        return ret;
    }

    ads122c04_calibrate_offset(st, 
                            st->channel_data[channel].gain, 
                            st->channel_data[channel].pga_enabled);

    ret = ads122c04_write_cfg0(st, 
                            channel,
                            st->channel_data[channel].gain,
                            st->channel_data[channel].pga_enabled);
    if (ret) {
        dev_err(&st->client->dev, "Failed to write register 0. Err %d", ret);
        return ret;
    }

    /* Start/Sync depends of the convertion mode and the
       Datasheet recomends send de STARTSYNC cmd when the cfg1 is modified */
    ret = ads122c04_send_start_command(st);
	if (ret) {
        dev_err(&st->client->dev, "Failed to send a start command on channel %d. Err %d", channel ,ret);
		return ret;
    }


    ret = ads122c04_read_adc(st, &adc_value);
	if (ret) {
        dev_err(&st->client->dev, "Failed to get adc value %d. Err %d", channel ,ret);
		return ret;
    }

    *val = adc_value;
    printk(KERN_INFO "ADS122c04: ads122c04_get_chan_adc_result \
        adc_value:%d offset_calibration:%d\n" , adc_value, offset_calibration);
	return 0;
}


static int ads122c04_set_data_rate(struct ads122c04_st *st, int chan, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads122c04_data_rate); i++) {
        if (st->channel_data[chan].turbo_mode) {
		    if (ads122c04_data_rate[i] == rate) {
			    st->channel_data[chan].data_rate = i;
			    return 0;
		    }
        } else {
            if (ads122c04_data_rate_turbo[i] == rate) {
			    st->channel_data[chan].data_rate = i;
			    return 0;
		    }
        }
	}

	return -EINVAL;
}


static int ads122c04_set_gain(struct ads122c04_st *st, int chan, int gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads122c04_gain_cfg); i++) {
		if (ads122c04_gain_cfg[i] == gain) {
			st->channel_data[chan].data_rate = i;
			return 0;
		}
	}

	return -EINVAL;
}


// Function to calculate LSB value in microvolts to maintain precision
// Returns LSB * FIXED_POINT_SCALE to vref_mv precision in integer math
int32_t calculate_lsb_fixed(int32_t vref_mv, int32_t gain) {
    int64_t temp;
    
    /* LSB = (2 * VREF_MV / Gain) / 2^24 */
    /* Multiply by FIXED_POINT_SCALE to maintain precision */
    temp = 2LL * (int64_t)vref_mv * FIXED_POINT_SCALE;
    
    /* Apply gain using div_s64 */
    temp = div_s64(temp, gain);
    
    /* Divide by 2^24 using div_s64 */
    temp = div_s64(temp, 1LL << 24);
    
    return (int32_t)temp;
}


// Function to convert ADC raw value to voltage in millivolts
int32_t adc_to_millivolts(int32_t adc_value, int32_t vref_mv, int32_t gain) {
    // Get LSB in fixed-point format
    int32_t lsb_fixed = calculate_lsb_fixed(vref_mv, gain);
    
    // Convert ADC value to voltage
    // We need to use 64-bit to prevent overflow in multiplication
    int64_t mv_fixed = (int64_t)adc_value * lsb_fixed;

    // Convert back from fixed-point to regular integer (millivolts)
    return (int32_t)(mv_fixed >> FIXED_POINT_SHIFT);
}


static uint32_t ads122c04_calculate_vref(struct ads122c04_st *st, const int mux)
{
    int32_t adc_result = 0;
    uint32_t vref_mv = 0;

    ads122c04_get_special_vref_monitor(st, mux, &adc_result);

    vref_mv = adc_to_millivolts(adc_result, ADS122C04_VREF_INTERNAL_REF_IN, 1);
    /* Datasheet 8.3.9 the final result is (V(REFP) – V(REFN)) / 4 or 
    (AVDD – AVSS) / 4*/
    return vref_mv * 4;
    
}

static uint32_t ads122c04_get_vref(struct ads122c04_st *st, int chan)
{
    if (st->channel_data[chan].vref_type == ADS122C04_VREF_INTERNAL) {
        return ADS122C04_VREF_INTERNAL_REF_IN;
    } else if (st->channel_data[chan].vref_type == ADS122C04_VREF_EXTERNAL) {
        return ads122c04_calculate_vref(st, ADS122C04_VREF_MON);
    } else { 
        return ads122c04_calculate_vref(st, ADS122C04_AVDD_AVSS);
    }

    return 0;
}


static int ads122c04_read_raw(struct iio_dev *indio_dev,
                    struct iio_chan_spec const *chan, int *val,
                    int *val2, long mask)
{
	int ret;
    int32_t vref_mv = 0;
	struct ads122c04_st *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			break;

        if (st->channel_data[chan->address].enabled == CHANNEL_DISABLED) {
            *val = 0;
            goto release_direct;
        }

		ret = ads122c04_get_chan_adc_result(st, chan->address, val);
		if (ret < 0) {
			goto release_direct;
		}

		ret = IIO_VAL_INT;

release_direct:
		iio_device_release_direct_mode(indio_dev);
        break;
    case IIO_CHAN_INFO_SCALE:
        vref_mv = ads122c04_get_vref(st, chan->address);
        *val = 2 * vref_mv;
        *val2 = ads122c04_gain_cfg[st->channel_data[chan->address].gain] << 24;
        ret = IIO_VAL_FRACTIONAL;
        break;
    case IIO_CHAN_INFO_HARDWAREGAIN:
    	*val = ads122c04_gain_cfg[st->channel_data[chan->address].gain];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ads122c04_data_rate[st->channel_data[chan->address].data_rate];
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static int ads122c04_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads122c04_st *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	switch (mask) {
    case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ads122c04_set_gain(st, chan->address, val);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads122c04_set_data_rate(st, chan->address, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static ssize_t ads122c04_vrefp_vrefn_mon_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads122c04_st *st = iio_priv(indio_dev);
	int len = 0;
	mutex_lock(&st->lock);

	len = sprintf(buf, "%d\n", ads122c04_calculate_vref(st, ADS122C04_VREF_MON));
    mutex_unlock(&st->lock);
	return len;
}


static ssize_t ads122c04_avdd_avss_mon_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads122c04_st *st = iio_priv(indio_dev);
	int len = 0;

    mutex_lock(&st->lock);
	len = sprintf(buf, "%d\n", ads122c04_calculate_vref(st, ADS122C04_AVDD_AVSS));
    mutex_unlock(&st->lock);
	return len;
}

static IIO_CONST_ATTR_NAMED(hwgain_available,
	hwgain_available, "1 2 4 8 16 32 64 128");

static IIO_CONST_ATTR_NAMED(sampling_frequency_available,
	sampling_frequency_available, "20 40 45 90 180 175 350 330 600 660 1000 1200 2000");

static IIO_DEVICE_ATTR(vrefp_vrefn_mon, 0444, ads122c04_vrefp_vrefn_mon_show, NULL, 0);
static IIO_DEVICE_ATTR(avdd_avss_mon, 0444, ads122c04_avdd_avss_mon_show, NULL, 0);

static struct attribute *ads122c04_attributes[] = {
    &iio_const_attr_hwgain_available.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
    &iio_dev_attr_vrefp_vrefn_mon.dev_attr.attr,
    &iio_dev_attr_avdd_avss_mon.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads122c04_attribute_group = {
	.attrs = ads122c04_attributes,
};


static const struct iio_info ads122c04_info = {
	.read_raw	= ads122c04_read_raw,
	.write_raw	= ads122c04_write_raw,
    .attrs      = &ads122c04_attribute_group,
};


static int ads122c04_client_get_channels_config(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122c04_st *data = iio_priv(indio_dev);
	struct device *dev = &client->dev;
	struct fwnode_handle *node;
	int i = -1;
    int channel;

	device_for_each_child_node(dev, node) {
		u32 pval;

		if (fwnode_property_read_u32(node, "reg", &pval)) {
			dev_err(dev, "invalid reg on %pfw\n", node);
			continue;
		}

		channel = pval;
		if (channel >= ADS122C04_CHANNELS) {
			dev_err(dev, "invalid channel index %d on %pfw\n",
				channel, node);
			continue;
		}

        data->channel_data[channel].enabled = CHANNEL_ENABLED;

        if (fwnode_property_present(node, "ti,pga-disable"))
            data->channel_data[channel].pga_enabled = ADS122C04_PGA_OFF;

		if (!fwnode_property_read_u32(node, "ti,gain", &pval)) {
            if (data->channel_data[channel].pga_enabled == ADS122C04_PGA_ON) {
			    if (pval > 7) {
				    dev_err(dev, "invalid gain on %pfw\n", node);
				    fwnode_handle_put(node);
				    return -EINVAL;
			    }
            } else {
                	if (pval > 3) {
				    dev_err(dev, "invalid gain on %pfw. pga is off\n", node);
				    fwnode_handle_put(node);
				    return -EINVAL;
			    }
            }
            data->channel_data[channel].gain = pval;
		}

		if (!fwnode_property_read_u32(node, "ti,datarate", &pval)) {
			
			if (pval > 7) {
				dev_err(dev, "invalid data_rate on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
            data->channel_data[channel].data_rate = pval;
		}

        if (!fwnode_property_read_u32(node, "ti,vref", &pval)) {
			if (pval > 4) {
				dev_err(dev, "invalid vref on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
            data->channel_data[channel].vref_type = pval;
		}

        if (fwnode_property_present(node, "ti,turbo-mode-enabled"))
            data->channel_data[channel].turbo_mode = ADS122C04_TURBO_MODE_ON;

        if (fwnode_property_present(node, "ti,temperature-mode-enabled"))
            data->channel_data[channel].temperature_mode = ADS122C04_TEMPERATURE_MODE_ON;

        if (fwnode_property_present(node, "ti,continues-mode"))
            data->channel_data[channel].conv_mode = ADS122C04_CONV_MODE_CONTINUES;

		i++;
	}

	return i < 0 ? -EINVAL : 0;
}


static void ads122c04_get_channels_config(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122c04_st *data = iio_priv(indio_dev);
    struct device *dev = &client->dev;
    struct ads122c04_channel_data chan_default = {
            .enabled = CHANNEL_DISABLED,
            .pga_enabled = ADS122C04_DEFAULT_PGA,
            .gain = ADS122C04_DEFAULT_GAIN,
            .data_rate  = ADS122C04_DEFAULT_DATA_RATE,
            .turbo_mode = ADS122C04_DEFAULT_TURBO_MODE,
            .conv_mode = ADS122C04_DEFAULT_CONV_MODE,
            .temperature_mode = ADS122C04_DEFAULT_TEMPERATURE_MODE,
            .vref_type = ADS122C04_DEFAULT_MAIN_VREF_REFERENCE,
    };
    int i = 0;
    int pval = 0;

    
	/* Default configuration */
	for(i = 0; i < ADS122C04_CHANNELS; ++i) {
		data->channel_data[i] = chan_default;
	}


    if (of_property_read_bool(client->dev.of_node, "ti,burn-out-mode-enabled")) { 
        data->burnout = ADS122C04_BURN_OUT_MODE_ON;
    }

	if (!of_property_read_u32(client->dev.of_node, "ti,idac-cfg", &pval)) {		
	    if (pval > 7) {
		    dev_err(dev, "invalid data_rate on %pfw\n", client->dev.of_node);
		    return;
	    }
        data->idac = pval;
    }

    if (data->idac != 0) {
        pval = 0;
	    if (!of_property_read_u32(client->dev.of_node, "ti,idac1-route", &pval)) {		
	        if (pval > 6) {
		        dev_err(dev, "invalid data_rate on %pfw\n", client->dev.of_node);
		        return;
	        }
            data->idac1_mux = pval;
	    }

        pval = 0;
	    if (!of_property_read_u32(client->dev.of_node, "ti,idac2-route", &pval)) {		
	        if (pval > 6) {
		        dev_err(dev, "invalid data_rate on %pfw\n", client->dev.of_node);
		        return;
	        }
            data->idac2_mux = pval;
	    }
    }


	if (!ads122c04_client_get_channels_config(client))
		return;

}

static int ads122c04_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	//struct ads122c04_st *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static int ads122c04_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ads122c04_st *data;
	enum chip_ids chip;
    int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

    data->client = client;
	mutex_init(&data->lock);

	indio_dev->name = ADS122C04_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	chip = (enum chip_ids)device_get_match_data(&client->dev);
	if ( chip == ADS122C04) {
		indio_dev->channels = ads122c04_channels;
		indio_dev->num_channels = ARRAY_SIZE(ads122c04_channels);
		indio_dev->info = &ads122c04_info;
    } else {
		dev_err(&client->dev, "Unknown chip %d\n", chip);
		return -EINVAL;
    }

	ads122c04_get_channels_config(client);
    ads122c04_clear_cfg(data);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id ads122c04_id[] = {
	{"ads122c04", ADS122C04},
	{}
};

MODULE_DEVICE_TABLE(i2c, ads122c04_id);


static const struct of_device_id ads122c04_of_match[] = {
	{
		.compatible = "ti,ads122c04",
	},
	{}
};
MODULE_DEVICE_TABLE(of, ads122c04_of_match);

static struct i2c_driver ads122c04_driver = {
	.driver = {
		.name = ADS122C04_DRV_NAME,
		.of_match_table = ads122c04_of_match,
	},
	.probe		= ads122c04_probe,
	.remove		= ads122c04_remove,
	.id_table	= ads122c04_id,
};

module_i2c_driver(ads122c04_driver);

MODULE_AUTHOR("Aluisio Leonello Victal <alvictal@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS122C04 ADC driver");
MODULE_LICENSE("GPL v2");
