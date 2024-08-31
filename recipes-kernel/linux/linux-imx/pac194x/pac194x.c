// SPDX-License-Identifier: GPL-2.0+
/*
 * IIO driver for PAC194X and PAC195X series chips
 *
 * Copyright (C) 2022 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Marius Cristea marius.cristea@microchip.com
 * Author: Victor Tudose
 *
 * Datasheet for PAC1941, PAC1942, PAC1943 and PAC1944 can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/PAC194X-Family-Data-Sheet-DS20006543.pdf
 * Datasheet for PAC1951, PAC1952, PAC1953 and PAC1954 can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/PAC195X-Family-Data-Sheet-DS20006539.pdf
 *
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/util_macros.h>

/* Maximum (1092 * 60 * 1000), around 1092 minutes@1024 sps
 * We will keep the refresh lower
 */
#define PAC194X5X_MAX_RFSH_LIMIT			300000

/* 50msec is the timeout for validity of the cached registers */
#define PAC194X5X_MIN_POLLING_TIME			50
#define SHUNT_UOHMS_DEFAULT				100000

/* 32000mV */
#define PAC195X_VOLTAGE_MILLIVOLTS_MAX			32000
/* 9000mV */
#define PAC194X_VOLTAGE_MILLIVOLTS_MAX			9000

/* Voltage bits resolution when set for unsigned values and
 * HALF FSR signed values
 */
#define PAC194X5X_VOLTAGE_16B_RES			16
/* Voltage bits resolution when set for signed values */
#define PAC194X5X_VOLTAGE_15B_RES			15

/* 100mV maximum voltage drop over the sense resistors */
#define PAC194X5X_VSENSE_MILLIVOLTS_MAX			100

/* Current bits resolution when set for unsigned values and
 * HALF FSR signed values
 */
#define PAC194X5X_CURRENT_16B_RES			16

/* Current bits resolution when set for signed values */
#define PAC194X5X_CURRENT_15B_RES			15

/* Power resolution is 30 bits when unsigned and HALF FSR signed values */
#define PAC194X5X_POWER_30B_RES				30

/* Power resolution is 29 bits when signed */
#define PAC194X5X_POWER_29B_RES				29

/* Accumulation register is 56 bits long for unipolar range */
#define PAC194X5X_ENERGY_56B_RES			56

/* Accumulation register is 56 bits long for bipolar range */
#define PAC194X5X_ENERGY_55B_RES			55

/* Maximum power-product value - 32 V * 0.1 V */
#define PAC195X_PRODUCT_VOLTAGE_PV_FSR			3200000000000UL

/* Maximum power-product value - 9 V * 0.1 V */
#define PAC194X_PRODUCT_VOLTAGE_PV_FSR			900000000000UL

#define PAC194151_NUM_CHANNELS				1
#define PAC194252_NUM_CHANNELS				2
#define PAC194353_NUM_CHANNELS				3
#define PAC194454_NUM_CHANNELS				4
#define PAC194X5X_MEAS_REG_SNAPSHOT_LEN			80
#define PAC194X5X_CTRL_REG_SNAPSHOT_LEN			24

/* 1000usec is the minimum wait time for normal conversions when sample
 * rate doesn't change
 */
#define PAC194X5X_MIN_UPDATE_WAIT_TIME			1000

#define PAC194X5X_DEFAULT_CHIP_SAMP_SPEED		1024

/* Device register address map */
#define PAC194X5X_REFRESH_REG_ADDR			0x00
#define PAC194X5X_CTRL_REG_ADDR				0x01
#define PAC194X5X_ACC_COUNT_REG				0x02

/* Start of configurations registers */
#define PAC194X5X_CTRL_STAT_REGS_ADDR			0x1C

#define PAC194X5X_SMBUS_SETTINGS_REGS_ADDR		0x1C
#define PAC194X5X_NEG_PWR_FSR_REG_ADDR			0x1D
#define PAC194X5X_REFRESG_V_REG				0x1E
#define PAC194X5X_REFRESH_V_REG				0x1F
#define PAC194X5X_CTRL_ACT_REG_ADDR			0x21
#define PAC194X5X_CTRL_LAT_REG_ADDR			0x23
#define PAC194X5X_NEG_PWR_FSR_LAT_REG_ADDR		0x24
#define PAC194X5X_ACCUM_REG_ADDR			0x25
#define PAC194X5X_PID_REG_ADDR				0xFD

#define PAC194X5X_VBUS_1_ADDR				0x03
#define PAC194X5X_VBUS_2_ADDR				0x04
#define PAC194X5X_VBUS_3_ADDR				0x05
#define PAC194X5X_VBUS_4_ADDR				0x06
#define PAC194X5X_VSENSE_1_ADDR				0x07
#define PAC194X5X_VSENSE_2_ADDR				0x08
#define PAC194X5X_VSENSE_3_ADDR				0x09
#define PAC194X5X_VSENSE_4_ADDR				0x0A
#define PAC194X5X_VBUS_AVG_1_ADDR			0x0B
#define PAC194X5X_VBUS_AVG_2_ADDR			0x0C
#define PAC194X5X_VBUS_AVG_3_ADDR			0x0D
#define PAC194X5X_VBUS_AVG_4_ADDR			0x0E
#define PAC194X5X_VSENSE_AVG_1_ADDR			0x0F
#define PAC194X5X_VSENSE_AVG_2_ADDR			0x10
#define PAC194X5X_VSENSE_AVG_3_ADDR			0x11
#define PAC194X5X_VSENSE_AVG_4_ADDR			0x12
#define PAC194X5X_VPOWER_1_ADDR				0x13
#define PAC194X5X_VPOWER_2_ADDR				0x14
#define PAC194X5X_VPOWER_3_ADDR				0x15
#define PAC194X5X_VPOWER_4_ADDR				0x16

#define PAC194X5X_ACPI_ARG_COUNT			4
#define PAC194X5X_ACPI_GET_NAMES_AND_MOHMS_VALS		1
#define PAC194X5X_ACPI_GET_UOHMS_VALS			2
#define PAC194X5X_ACPI_GET_BIPOLAR_SETTINGS		4
#define PAC194X5X_ACPI_GET_SAMP				5

/*
 * These indexes are exactly describing the element order within a single
 * PAC194X5X physical channel IIO channel descriptor; see the static const struct
 * iio_chan_spec pac194x5x_single_channel[] declaration
 */
#define IIO_POW						0
#define IIO_VOLT					1
#define IIO_CRT						2
#define IIO_VOLTAVG					3
#define IIO_CRTAVG					4

#define PAC194X5X_ACC_REG_LEN				4
#define PAC194X5X_VACC_REG_LEN				7
#define PAC194X5X_VBUS_SENSE_REG_LEN			2
#define PAC194X5X_VPOWER_REG_LEN			4
#define PAC194X5X_CTRL_ACT_REG_LEN			2
#define PAC194X5X_CTRL_LAT_REG_LEN			2
#define PAC194X5X_MAX_REGISTER_LENGTH			6

#define PAC194X5X_COMMON_DEVATTR			2
#define PAC194X5X_DEVATTR_FOR_CHANNEL			4
#define PAC194X5X_SHARED_DEVATTRS_COUNT			1

/*
 * Relative offsets when using multi-byte reads/writes even though these
 * bytes are read one after the other, they are not at adjacent memory
 * locations within the I2C memory map. The chip can skip some addresses
 */
#define PAC194X5X_SMBUS_SETTINGS_REG_OFF		0
#define PAC194X5X_NEG_PWR_REG_OFF			1

/*
 * when reading/writing multiple bytes from offset of SMBUS SETTINGS (1CH) REGISTER,
 * the chip jumps over the 0x1E (REFRESH_G) and 0x1F (REFRESH_V) offsets
 */
#define PAC194X5X_SLOW_REG_OFF				3

#define PAC194X5X_NEG_PWR_FSR_ACT_REG_OFF		6
#define PAC194X5X_CTRL_LAT_REG_OFF			8
#define PAC194X5X_NEG_PWR_FSR_LAT_REG_OFF		10
#define PAC194X5X_ACCUM_CONFIG_REG_OFF			12
#define PAC194X5X_ALERT_STATUS_REG_OFF			13
#define PAC194X5X_SLOW_ALERT1_REG_OFF			16
#define PAC194X5X_GPIO_ALERT2_REG_OFF			19
#define PAC194X5X_ACC_FULLNESS_LIMITS_REG_OFF		22
#define PAC194X5X_OC_LIMIT1_REG_OFF			24
#define PAC194X5X_OC_LIMIT2_REG_OFF			26
#define PAC194X5X_OC_LIMIT3_REG_OFF			28

#define PAC194X5X_CTRL_STATUS_INFO_LEN			30

#define PAC194X5X_CH_DIS_NOSKIP_VAL			0x02

#define PAC194X5X_MID					0x5D
#define PAC194x54_PID					0x5B
#define PAC194x53_PID					0x5A
#define PAC194x52_PID					0x59
#define PAC194x51_PID					0x58

#define PAC194X5X_MAX_NUM_CHANNELS			4
/* PAC194X5X family */
#define PAC_PRODUCT_ID_1941_1				0x68
#define PAC_PRODUCT_ID_1942_1				0x69
#define PAC_PRODUCT_ID_1943_1				0x6A
#define PAC_PRODUCT_ID_1944_1				0x6B
#define PAC_PRODUCT_ID_1941_2				0x6C
#define PAC_PRODUCT_ID_1942_2				0x6D
/* PAC195x family */
#define PAC_PRODUCT_ID_1951_1				0x78
#define PAC_PRODUCT_ID_1952_1				0x79
#define PAC_PRODUCT_ID_1953_1				0x7A
#define PAC_PRODUCT_ID_1954_1				0x7B
#define PAC_PRODUCT_ID_1951_2				0x7C
#define PAC_PRODUCT_ID_1952_2				0x7D

#define PAC194X5X_1024_SPS_ADDAPTIVE_SAMP_MODE		0x0
#define PAC194X5X_256_SPS_ADDAPTIVE_SAMP_MODE		0x1
#define PAC194X5X_64_SPS_ADDAPTIVE_SAMP_MODE		0x2
#define PAC194X5X_8_SPS_ADDAPTIVE_SAMP_MODE		0x3
#define PAC194X5X_1024_SPS_SAMP_MODE			0x4
#define PAC194X5X_256_SPS_SAMP_MODE			0x5
#define PAC194X5X_64_SPS_SAMP_MODE			0x6
#define PAC194X5X_8_SPS_SAMP_MODE			0x7
#define PAC194X5X_SINGLE_SHOT_SAMP_MODE			0x8
#define PAC194X5X_SINGLE_SHOT_8X_SAMP_MODE		0x9
#define PAC194X5X_1024_FAST_SAMP_MODE			0xA
#define PAC194X5X_1024_BURST_SAMP_MODE			0xB
#define PAC194X5X_1024_SLEEP_SAMP_MODE			0xF

#define PAC194X5X_ALERT					0x00
#define PAC194X5X_GPIO_INPUT				0x01
#define PAC194X5X_GPIO_OUTPUT				0x02
#define PAC194X5X_SLOW					0x03

#define CTRL_REG(samp, gpio, slow, ch1_activ, ch2_activ, ch3_activ, ch4_activ) \
			((((u8)(samp) & 0xf) << 12)		|   \
			(((u8)(gpio) & 0x03) << 10)		|   \
			(((u8)(slow) & 0x03) << 8)		|   \
			(((u8)ch1_activ ? 0x00 : 0x01) << 7)	|   \
			(((u8)ch2_activ ? 0x00 : 0x01) << 6)	|   \
			(((u8)ch3_activ ? 0x00 : 0x01) << 5)	|   \
			(((u8)ch4_activ ? 0x00 : 0x01) << 4))

#define PAC194X5X_UNIPOLAR_FSR_CFG			0
#define PAC194X5X_BIPOLAR_FSR_CFG			1
#define PAC194X5X_BIPOLAR_HALF_FSR_CFG			2

#define PAC194X5X_ACCMODE_VPOWER			0
#define PAC194X5X_ACCMODE_VSENSE			1
#define PAC194X5X_ACCMODE_VBUS				2

#define PAC194X5X_CFG_VB4_OFFSET			0
#define PAC194X5X_CFG_VB3_OFFSET			2
#define PAC194X5X_CFG_VB2_OFFSET			4
#define PAC194X5X_CFG_VB1_OFFSET			6
#define PAC194X5X_CFG_VS4_OFFSET			8
#define PAC194X5X_CFG_VS3_OFFSET			10
#define PAC194X5X_CFG_VS2_OFFSET			12
#define PAC194X5X_CFG_VS1_OFFSET			18

#define NEG_PWR_REG(cfg_vs1, cfg_vs2, cfg_vs3, cfg_vs4, cfg_vb1, cfg_vb2,     \
		    cfg_vb3, cfg_vb4)					      \
			((((cfg_vs1) & 0x03) << PAC194X5X_CFG_VS1_OFFSET) |   \
			 (((cfg_vs2) & 0x03) << PAC194X5X_CFG_VS2_OFFSET) |   \
			 (((cfg_vs3) & 0x03) << PAC194X5X_CFG_VS3_OFFSET) |   \
			 (((cfg_vs4) & 0x03) << PAC194X5X_CFG_VS4_OFFSET) |   \
			 (((cfg_vb1) & 0x03) << PAC194X5X_CFG_VB1_OFFSET) |   \
			 (((cfg_vb2) & 0x03) << PAC194X5X_CFG_VB2_OFFSET) |   \
			 (((cfg_vb3) & 0x03) << PAC194X5X_CFG_VB3_OFFSET) |   \
			 (((cfg_vb4) & 0x03) << PAC194X5X_CFG_VB4_OFFSET))

#define PAC194X5X_CFG_ACC4_OFFSET			0
#define PAC194X5X_CFG_ACC3_OFFSET			2
#define PAC194X5X_CFG_ACC2_OFFSET			4
#define PAC194X5X_CFG_ACC1_OFFSET			6

#define ACCUM_REG(acc1_cfg, acc2_cfg, acc3_cfg, acc4_cfg)		      \
			((((acc1_cfg) & 0x03) << PAC194X5X_CFG_ACC1_OFFSET) | \
			 (((acc2_cfg) & 0x03) << PAC194X5X_CFG_ACC2_OFFSET) | \
			 (((acc3_cfg) & 0x03) << PAC194X5X_CFG_ACC3_OFFSET) | \
			 (((acc4_cfg) & 0x03) << PAC194X5X_CFG_ACC4_OFFSET))

/*
 * Accumulated power/energy formula (in mW-seconds):
 * Energy = (Vacc/10^9)*[(10^9/2^30)*2^9]*3.2*10^3/Rsense
 * Vacc - is the accumulated value per second
 * Rsense - value of the shunt resistor in microOhms
 *
 * PAC195X_MAX_VPOWER_RSHIFTED_BY_29B = 3.2*((10^9)/(2^29))*10^9
 * will be used to calculate the scale for accumulated power/energy
 */
#define PAC195X_MAX_VPOWER_RSHIFTED_BY_29B		5960464478UL

/*
 * PAC194X_MAX_VPOWER_RSHIFTED_BY_29B = 0.9*((10^9)/(2^29))*10^9
 * will be used to calculate the scale for accumulated power/energy
 */
#define PAC194X_MAX_VPOWER_RSHIFTED_BY_29B		1676380634UL

/* (100mV * 1000000) / (2^15)  used to calculate the scale for current */
#define PAC194X5X_MAX_VSENSE_RSHIFTED_BY_15B		3052

/*
 * [(100mV * 1000000) / (2^15)]*10^9  used to calculate the scale
 * for accumulated current/Coulomb counter
 */
#define PAC194X5X_MAX_VSENSE_NANO			3051757812500UL

#define to_pac194x5x_chip_info(d) container_of(d, struct pac194x5x_chip_info, work_chip_rfsh)

/* Macros to extract the parameters */
#define MACC_COUNT(addr) (((u32)(*(u8 *)((addr) + 0)) << 24) |	\
			 ((u32)(*(u8 *)((addr) + 1)) << 16)  |	\
			 ((u32)(*(u8 *)((addr) + 2)) << 8)   |	\
			 ((u32)(*(u8 *)((addr) + 3)) << 0))

#define MVACCU(addr)    (((u64)(*(u8 *)((addr) + 0)) << 48)  |	\
			((u64)(*(u8 *)((addr) + 1)) << 40)   |	\
			((u64)(*(u8 *)((addr) + 2)) << 32)   |	\
			((u64)(*(u8 *)((addr) + 3)) << 24)   |	\
			((u64)(*(u8 *)((addr) + 4)) << 16)   |	\
			((u64)(*(u8 *)((addr) + 5)) << 8)    |	\
			((u64)(*(u8 *)((addr) + 6)) << 0))
#define MVACCS(addr)    sign_extend64(MVACCU(addr), 55)

#define MVPOWERU(addr)  (((u32)(*(u8 *)(addr + 0)) << 22)  |	\
			((u32)(*(u8 *)(addr + 1)) << 14)   |	\
			((u32)(*(u8 *)(addr + 2)) << 6)    |	\
			((u32)(*(u8 *)(addr + 3)) >> 2))

#define MVPOWERS(addr)        sign_extend32(MVPOWERU(addr), 29)

#define MVBUS_SENSEU(addr)    (((u16)(*(u8 *)((addr) + 0)) << 8)   |    \
			      ((u16)(*(u8 *)((addr) + 1)) << 0))

#define MVBUS_SENSES(addr)    ((__s16)MVBUS_SENSEU(addr))

#define INDEX_IN_POWER_ACC_NAME				12
#define INDEX_IN_CURRENT_ACC_NAME			14
#define INDEX_IN_VOLTAGE_AVERAGE_NAME			18

enum pac194x5x_ids {
	pac1944_1,
	pac1943_1,
	pac1942_1,
	pac1941_1,
	pac1942_2,
	pac1941_2,
	pac1954_1,
	pac1953_1,
	pac1952_1,
	pac1951_1,
	pac1952_2,
	pac1951_2
};

enum pac194x5x_samps {
	pac194x5x_samp_1024sps_adapt,
	pac194x5x_samp_256sps_adapt,
	pac194x5x_samp_64sps_adapt,
	pac194x5x_samp_8sps_adapt,

	pac194x5x_samp_1024sps,
	pac194x5x_samp_256sps,
	pac194x5x_samp_64sps,
	pac194x5x_samp_8sps,

	pac194x5x_samp_single_shot,
	pac194x5x_samp_single_shot_8x,
	pac194x5x_samp_fast_mode,
	pac194x5x_samp_burst_mode,

	pac194x5x_reserved1,
	pac194x5x_reserved2,
	pac194x5x_reserved3,
	pac194x5x_reserved4
};

enum pac194x5x_number_of_active_channels {
	pac194x5x_1_channel_active,
	pac194x5x_2_channels_active,
	pac194x5x_3_channels_active,
	pac194x5x_4_channels_active,
};

static const unsigned int samp_rate_map_tbl[] = {
	[pac194x5x_samp_1024sps_adapt] = 1024,
	[pac194x5x_samp_256sps_adapt] = 256,
	[pac194x5x_samp_64sps_adapt] = 64,
	[pac194x5x_samp_8sps_adapt] = 8,
	[pac194x5x_samp_1024sps] = 1024,
	[pac194x5x_samp_256sps] = 256,
	[pac194x5x_samp_64sps] = 64,
	[pac194x5x_samp_8sps] = 8,
	[pac194x5x_samp_single_shot] = 1,
	[pac194x5x_samp_single_shot_8x] = 8,
	[pac194x5x_samp_fast_mode] = 0xff,
	[pac194x5x_samp_burst_mode] = 0xff,
	[pac194x5x_reserved1] = 0xff,
	[pac194x5x_reserved2] = 0xff,
	[pac194x5x_reserved3] = 0xff,
	[pac194x5x_reserved4] = 0xff
};

static const unsigned int shift_map_tbl[] = {
	[pac194x5x_samp_1024sps_adapt] = 10,
	[pac194x5x_samp_256sps_adapt] = 10,
	[pac194x5x_samp_64sps_adapt] = 10,
	[pac194x5x_samp_8sps_adapt] = 10,
	[pac194x5x_samp_1024sps] = 10,
	[pac194x5x_samp_256sps] = 8,
	[pac194x5x_samp_64sps] = 6,
	[pac194x5x_samp_8sps] = 3,
	[pac194x5x_samp_single_shot] = 10,
	[pac194x5x_samp_single_shot_8x] = 8,
	[pac194x5x_samp_fast_mode] = 6,
	[pac194x5x_samp_burst_mode] = 3,
	[pac194x5x_reserved1] = 0xff,
	[pac194x5x_reserved2] = 0xff,
	[pac194x5x_reserved3] = 0xff,
	[pac194x5x_reserved4] = 0xff
};

static const unsigned int samp_rate_burst_mode_tbl[] = {
	[pac194x5x_1_channel_active] = 5120,
	[pac194x5x_2_channels_active] = 2560,
	[pac194x5x_3_channels_active] = 1706,
	[pac194x5x_4_channels_active] = 1280
};

static const unsigned int samp_rate_fast_mode_tbl[] = {
	[pac194x5x_1_channel_active] = 2560,
	[pac194x5x_2_channels_active] = 1707,
	[pac194x5x_3_channels_active] = 1280,
	[pac194x5x_4_channels_active] = 1024
};

/* Available Sample Modes */
static const char * const pac194x5x_frequency_avail[] = {
	"1024_ADAP",
	"256_ADAP",
	"64_ADAP",
	"8_ADAP",
	"1024",
	"256",
	"64",
	"8",
	"single_shot_1x",
	"single_shot_8x",
	"fast",
	"burst"
};

/**
 * struct reg_data - data from the registers
 * @active_channels: array of values, true means that channel is active
 * @vsense_mode:array of values, FSR mode for V Sense
 * @vbus_mode: array of values, FSR mode for V Bus
 * @accumulation_mode: array of values, accumulation mode for V Acc
 * @meas_regs: snapshot of raw measurements registers
 * @ctrl_act_reg: snapshot of the ctrl_act register
 * @ctrl_lat_reg: snapshot of the ctrl_lat register
 * @acc_count: snapshot of the acc_count register
 * @total_samples_nr: accumulated values for acc_count (total number of samples)
 * @acc_val: accumulated values per second
 * @vacc: accumulated vpower values
 * @vpower: snapshot of vpower registers
 * @vbus: snapshot of vbus registers
 * @vbus_avg: averages of vbus registers
 * @vsense: snapshot of vsense registers
 * @vsense_avg: averages of vsense registers
 * @jiffies_tstamp: chip's uptime
 */
struct reg_data {
	bool    active_channels[PAC194X5X_MAX_NUM_CHANNELS];
	u8      vbus_mode[PAC194X5X_MAX_NUM_CHANNELS];
	u8      vsense_mode[PAC194X5X_MAX_NUM_CHANNELS];
	u8      accumulation_mode[PAC194X5X_MAX_NUM_CHANNELS];
	u8      meas_regs[PAC194X5X_MEAS_REG_SNAPSHOT_LEN];
	u16     ctrl_act_reg;
	u16     ctrl_lat_reg;
	u32     acc_count;
	u32     total_samples_nr[PAC194X5X_MAX_NUM_CHANNELS];
	s64     acc_val[PAC194X5X_MAX_NUM_CHANNELS];
	s64     vacc[PAC194X5X_MAX_NUM_CHANNELS];
	s32     vpower[PAC194X5X_MAX_NUM_CHANNELS];
	s32     vbus[PAC194X5X_MAX_NUM_CHANNELS];
	s32     vbus_avg[PAC194X5X_MAX_NUM_CHANNELS];
	s32     vsense[PAC194X5X_MAX_NUM_CHANNELS];
	s32     vsense_avg[PAC194X5X_MAX_NUM_CHANNELS];
	unsigned long jiffies_tstamp;
};

/**
 * struct pac194x5x_chip_info - chip configuration
 * @channels: array of values, true means that channel is active
 * @indio_info: array of bools, true means that channel is bidirectional
 * @client: a pointer to the i2c client associated with the device
 * @lock: lock to prevent concurrent reads/writes
 * @tmr_forced_update: timer used for periodic read from device
 * @wq_chip: workqueue for periodic chip readings to prevent saturation
 * @work_chip_rfsh: chip refresh workqueue implementation
 * @phys_channels: number of physical channels for the device
 * @chip_variant: stores the type of the device
 * @chip_revision: store the silicon revision version of the device
 * @shunts: array of values, shunt resistor values
 * @chip_reg_data: pointer to structure, containing data from the device registers
 * @sample_rate_value: sampling frequency
 * @channel_names: array of string, name of each channel
 * @pac194x5x_info: pointer to iio_info structure
 * @is_pac195x_family: true if device is part of the PAC195x family
 * @sampling_mode: sampling mode used by the device
 * @num_enabled_channels: count of how many chip channels are currently enabled
 */
struct pac194x5x_chip_info {
	const struct iio_chan_spec  *channels;
	const struct iio_info	*indio_info;
	struct i2c_client	*client;
	struct mutex		lock; /*lock to prevent concurrent reads/writes */
	struct timer_list	tmr_forced_update;
	/* workqueue for periodic chip readings to prevent saturation */
	struct workqueue_struct	*wq_chip;
	struct work_struct	work_chip_rfsh;
	u8		phys_channels;
	u8		chip_variant;
	u8		chip_revision;
	u32		shunts[PAC194X5X_MAX_NUM_CHANNELS];
	struct reg_data   chip_reg_data;
	char		*sample_rate_value;
	char		*channel_names[PAC194X5X_MAX_NUM_CHANNELS];
	struct		iio_info pac194x5x_info;
	bool		is_pac195x_family;
	u8		sampling_mode;
	u8		num_enabled_channels;
};

/**
 * struct pac194x5x_features - features of a pac194x5x instance
 * @phys_channels: number of physical channels supported by the chip
 * @prod_id: product ID
 */
struct pac194x5x_features {
	u8 phys_channels;
	u8 prod_id;
};

static const struct pac194x5x_features pac194x5x_chip_config[] = {
	/* PAC195X Family */
	[pac1954_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS,
		.prod_id = PAC_PRODUCT_ID_1954_1,
	},
	[pac1953_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 1,
		.prod_id = PAC_PRODUCT_ID_1953_1,
	},
	[pac1952_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 2,
		.prod_id = PAC_PRODUCT_ID_1952_1,
	},
	[pac1951_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 3,
		.prod_id = PAC_PRODUCT_ID_1951_1,
	},
	[pac1952_2] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 2,
		.prod_id = PAC_PRODUCT_ID_1952_2,
	},
	[pac1951_2] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 3,
		.prod_id = PAC_PRODUCT_ID_1951_2,
	},
	/* PAC194X Family */
	[pac1944_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS,
		.prod_id = PAC_PRODUCT_ID_1944_1,
	},
	[pac1943_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 1,
		.prod_id = PAC_PRODUCT_ID_1943_1,
	},
	[pac1942_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 2,
		.prod_id = PAC_PRODUCT_ID_1942_1,
	},
	[pac1941_1] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 3,
		.prod_id = PAC_PRODUCT_ID_1941_1,
	},
	[pac1942_2] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 2,
		.prod_id = PAC_PRODUCT_ID_1942_2,
	},
	[pac1941_2] = {
		.phys_channels = PAC194X5X_MAX_NUM_CHANNELS - 3,
		.prod_id = PAC_PRODUCT_ID_1941_2,
	},
};

static const struct i2c_device_id pac194x5x_id[] = {
	{ "pac1954_1", pac1954_1 },
	{ "pac1953_1", pac1953_1 },
	{ "pac1952_1", pac1952_1 },
	{ "pac1951_1", pac1951_1 },
	{ "pac1952_2", pac1952_2 },
	{ "pac1951_2", pac1951_2 },
	{ "pac1944_1", pac1944_1 },
	{ "pac1943_1", pac1943_1 },
	{ "pac1942_1", pac1942_1 },
	{ "pac1941_1", pac1941_1 },
	{ "pac1942_2", pac1942_2 },
	{ "pac1941_2", pac1941_2 },
	{}
};

MODULE_DEVICE_TABLE(i2c, pac194x5x_id);

static const struct of_device_id pac194x5x_of_match[] = {
	{ .compatible = "microchip,pac1954_1",
	 .data = (void *)&pac194x5x_chip_config[pac1954_1]
	},
	{ .compatible = "microchip,pac1953_1",
	 .data = (void *)&pac194x5x_chip_config[pac1953_1]
	},
	{ .compatible = "microchip,pac1952_1",
	 .data = (void *)&pac194x5x_chip_config[pac1952_1]
	},
	{ .compatible = "microchip,pac1951_1",
	 .data = (void *)&pac194x5x_chip_config[pac1951_1]
	},
	{ .compatible = "microchip,pac1952_2",
	 .data = (void *)&pac194x5x_chip_config[pac1952_2]
	},
	{ .compatible = "microchip,pac1951_2",
	 .data = (void *)&pac194x5x_chip_config[pac1951_2]
	},
	{ .compatible = "microchip,pac1944_1",
	 .data = (void *)&pac194x5x_chip_config[pac1944_1]
	},
	{ .compatible = "microchip,pac1943_1",
	 .data = (void *)&pac194x5x_chip_config[pac1943_1]
	},
	{ .compatible = "microchip,pac1942_1",
	 .data = (void *)&pac194x5x_chip_config[pac1942_1]
	},
	{ .compatible = "microchip,pac1941_1",
	 .data = (void *)&pac194x5x_chip_config[pac1941_1]
	},
	{ .compatible = "microchip,pac1942_2",
	 .data = (void *)&pac194x5x_chip_config[pac1942_2]
	},
	{ .compatible = "microchip,pac1941_2",
	 .data = (void *)&pac194x5x_chip_config[pac1941_2]
	},
	{}
};
MODULE_DEVICE_TABLE(of, pac194x5x_of_match);

static int pac194x5x_i2c_read(struct i2c_client *client, u8 reg_addr,
			      void *databuf, u8 len)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .len = 1,
		 .buf = (u8 *)&reg_addr, .flags = 0 },
		{ .addr = client->addr, .len = len,
		 .buf = databuf, .flags = I2C_M_RD } };

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev,
			"failed reading data from register 0x%02X\n",
			reg_addr);
		return ret;
	}

	return 0;
}

static int pac194x5x_i2c_send_byte(struct i2c_client *client, u8 reg_addr)
{
	int ret;
	u8 buf;
	struct i2c_msg msg = { .addr = client->addr,
			.len = sizeof(buf),
			.buf = (u8 *)&buf,
			.flags = 0 };
	buf = reg_addr;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed sending byte to register 0x%02X\n",
			reg_addr);
		return ret;
	}
	return 0;
}

static int pac194x5x_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data,
			       int len)
{
	int ret;
	u8 send[PAC194X5X_MAX_REGISTER_LENGTH + 1];
	struct i2c_msg msg = { .addr = client->addr,
			      .len = len + 1, .flags = 0 };

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	msg.buf = send;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed writing data to register 0x%02X\n",
			reg_addr);
		return ret;
	}
	return 0;
}

/* Custom IIO Device Attributes */
static ssize_t shunt_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int len = 0;
	int target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0') - 1;

	len = sprintf(buf, "%d\n", chip_info->shunts[target]);
	return len;
}

static ssize_t channel_name_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int len = 0;
	int target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0') - 1;

	len = sprintf(buf, "%s\n", chip_info->channel_names[target]);
	return len;
}

static ssize_t shunt_value_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int sh_val;
	int target;

	target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0');
	if (kstrtoint(buf, 10, &sh_val)) {
		dev_err(dev,
			"Shunt value is not a number\n");
		return -EINVAL;
	}
	if (sh_val < 0) {
		dev_err(dev, "%s: Negative shunt values not allowed\n",
			"__func__");
		return -EINVAL;
	}
	mutex_lock(&chip_info->lock);
	chip_info->shunts[target] = sh_val;
	mutex_unlock(&chip_info->lock);
	return count;
}

static ssize_t reset_acc_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int ret;
	int i;
	u8 refresh_cmd = PAC194X5X_REFRESH_REG_ADDR;

	ret = pac194x5x_i2c_send_byte(chip_info->client, refresh_cmd);
	if (ret < 0) {
		dev_err(&indio_dev->dev,
			"%s - cannot send byte to PAC194X5X 0x%02X reg\n",
			__func__, refresh_cmd);
	}

	for (i = 0 ; i < chip_info->phys_channels; i++) {
		chip_info->chip_reg_data.acc_val[i] = 0;
		chip_info->chip_reg_data.total_samples_nr[i] = 0;
	}

	return count;
}

static IIO_DEVICE_ATTR(shunt_value_1, 0644, shunt_value_show, shunt_value_store, 0);

static IIO_DEVICE_ATTR(shunt_value_2, 0644, shunt_value_show, shunt_value_store, 0);

static IIO_DEVICE_ATTR(shunt_value_3, 0644, shunt_value_show, shunt_value_store, 0);

static IIO_DEVICE_ATTR(shunt_value_4, 0644, shunt_value_show, shunt_value_store, 0);

static IIO_DEVICE_ATTR(channel_name_1, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(channel_name_2, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(channel_name_3, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(channel_name_4, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(reset_accumulators, 0220, NULL, reset_acc_store, 0);

#define PAC194X5X_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)

static struct attribute *pac194x5x_all_attrs[] = {
	PAC194X5X_DEV_ATTR(shunt_value_1),
	PAC194X5X_DEV_ATTR(channel_name_1),
	PAC194X5X_DEV_ATTR(shunt_value_2),
	PAC194X5X_DEV_ATTR(channel_name_2),
	PAC194X5X_DEV_ATTR(shunt_value_3),
	PAC194X5X_DEV_ATTR(channel_name_3),
	PAC194X5X_DEV_ATTR(shunt_value_4),
	PAC194X5X_DEV_ATTR(channel_name_4),
	PAC194X5X_DEV_ATTR(reset_accumulators),
	NULL
};

#ifdef TEST
#define PAC194X5X_VACC_CHANNEL(_index, _si, _address) {		\
	.type = IIO_ENERGY,					\
	.address = (_address) + 6,				\
	.indexed = 1,						\
	.channel = (_index) + 6,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = (_si),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = PAC194X5X_ENERGY_56B_RES,		\
		.storagebits = PAC194X5X_ENERGY_56B_RES,	\
		.shift = 0,					\
		.endianness = IIO_CPU,				\
	},							\
	.ext_info =  pac194x5x_ext_info				\
}
#endif

#define PAC194X5X_VBUS_CHANNEL(_index, _si, _address) {		\
	.type = IIO_VOLTAGE,					\
	.address = (_address),					\
	.indexed = 1,						\
	.channel = (_index) + 1,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_index = (_si),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = PAC194X5X_VOLTAGE_16B_RES,		\
		.storagebits = PAC194X5X_VOLTAGE_16B_RES,	\
		.shift = 0,					\
		.endianness = IIO_CPU,				\
	},							\
	.ext_info =  pac194x5x_ext_info				\
}

#define PAC194X5X_VBUS_AVG_CHANNEL(_index, _si, _address) {	\
	.type = IIO_VOLTAGE,					\
	.address = (_address),					\
	.indexed = 1,						\
	.channel = (_index) + 1,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW) |	\
			BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = (_si),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = PAC194X5X_VOLTAGE_16B_RES,		\
		.storagebits = PAC194X5X_VOLTAGE_16B_RES,	\
		.shift = 0,					\
		.endianness = IIO_CPU,				\
	},							\
	.ext_info =  pac194x5x_ext_info				\
}

#define PAC194X5X_VSENSE_CHANNEL(_index, _si, _address) {	\
	.type = IIO_CURRENT,					\
	.address = (_address),					\
	.indexed = 1,						\
	.channel = (_index) + 1,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_index = (_si),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = PAC194X5X_CURRENT_16B_RES,		\
		.storagebits = PAC194X5X_CURRENT_16B_RES,	\
		.shift = 0,					\
		.endianness = IIO_CPU,				\
	},							\
	.ext_info =  pac194x5x_ext_info				\
}

#define PAC194X5X_VSENSE_AVG_CHANNEL(_index, _si, _address) {	\
	.type = IIO_CURRENT,					\
	.address = (_address),					\
	.indexed = 1,						\
	.channel = (_index) + 1,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW) |	\
			BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = (_si),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = PAC194X5X_CURRENT_16B_RES,		\
		.storagebits = PAC194X5X_CURRENT_16B_RES,	\
		.shift = 0,					\
		.endianness = IIO_CPU,				\
	},							\
	.ext_info =  pac194x5x_ext_info				\
}

#define PAC194X5X_VPOWER_CHANNEL(_index, _si, _address) {	\
	.type = IIO_POWER,					\
	.address = (_address),					\
	.indexed = 1,						\
	.channel = (_index) + 1,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = (_si),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = PAC194X5X_POWER_30B_RES,		\
		.storagebits = 32,				\
		.shift = 4,					\
		.endianness = IIO_CPU,				\
	},							\
	.ext_info =  pac194x5x_ext_info				\
}

#define PAC194X5X_SOFT_TIMESTAMP(_index) {			\
	.type = IIO_TIMESTAMP,					\
	.channel = -1,						\
	.scan_index = (_index),					\
}

static const struct iio_chan_spec pac194x5x_ts[] = {
	PAC194X5X_SOFT_TIMESTAMP(0),
};

static int pac194x5x_send_refresh(struct pac194x5x_chip_info *chip_info,
				  u8 refresh_cmd, u32 wait_time)
{
	struct i2c_client *client = chip_info->client;
	int ret;
	/* Writing a REFRESH or a REFRESH_V command */
	ret = pac194x5x_i2c_send_byte(client, refresh_cmd);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot send Refresh cmd (0x%02X) to PAC194X5X\n",
			__func__, refresh_cmd);
		return ret;
	}

	/* Register data retrieval timestamp */
	chip_info->chip_reg_data.jiffies_tstamp = jiffies;
	/* Wait till the data is available */
	usleep_range(wait_time, wait_time + 100);
	return ret;
}

static int pac194x5x_reg_snapshot(struct pac194x5x_chip_info *chip_info,
				  bool do_refresh, u8 refresh_cmd,
				  u32 wait_time)
{
	int ret;
	struct i2c_client *client = chip_info->client;
	u8 offset_reg_data, shift, idx;
	int cnt;
	u32 count, inc_count, fs;
	s64 stored_value;
	s64 inc;
	u16 tmp;

	mutex_lock(&chip_info->lock);

	if (do_refresh) {
		ret = pac194x5x_send_refresh(chip_info, refresh_cmd, wait_time);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s - cannot send refresh towards PAC194X5X\n",
				__func__);
			goto reg_snapshot_err;
		}
	}
	/* Read the ctrl/status registers for this snapshot */
	ret = pac194x5x_i2c_read(client, PAC194X5X_CTRL_ACT_REG_ADDR,
				 (u8 *)&tmp, PAC194X5X_CTRL_ACT_REG_LEN);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot read PAC194X5X regs from 0x%02X\n",
			__func__, PAC194X5X_CTRL_ACT_REG_ADDR);
		goto reg_snapshot_err;
	}

	be16_to_cpus(&tmp);
	chip_info->chip_reg_data.ctrl_act_reg = tmp;

	/* Read the ctrl/status registers for this snapshot */
	ret = pac194x5x_i2c_read(client, PAC194X5X_CTRL_LAT_REG_ADDR,
				 (u8 *)&tmp, PAC194X5X_CTRL_LAT_REG_LEN);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot read PAC194X5X regs from 0x%02X\n",
			__func__, PAC194X5X_CTRL_LAT_REG_ADDR);
		goto reg_snapshot_err;
	}

	be16_to_cpus(&tmp);
	chip_info->chip_reg_data.ctrl_lat_reg = tmp;

	/* Read the data registers */
	ret = pac194x5x_i2c_read(client, PAC194X5X_ACC_COUNT_REG,
				 (u8 *)chip_info->chip_reg_data.meas_regs,
				 PAC194X5X_MEAS_REG_SNAPSHOT_LEN);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot read PAC194X5X regs from 0x%02X\n",
			__func__, PAC194X5X_ACC_COUNT_REG);
		goto reg_snapshot_err;
	}

	offset_reg_data = 0;
	chip_info->chip_reg_data.acc_count =
	MACC_COUNT(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);

	offset_reg_data += PAC194X5X_ACC_REG_LEN;

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* Check if the channel is active(within the data read from
		 * the chip), skip all fields if disabled
		 */
		if (((chip_info->chip_reg_data.ctrl_act_reg << cnt) & 0x80) == 0) {
			stored_value = chip_info->chip_reg_data.acc_val[cnt];

			if (chip_info->chip_reg_data.vbus_mode[cnt] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG &&
			    chip_info->chip_reg_data.vsense_mode[cnt] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG)
				chip_info->chip_reg_data.vacc[cnt] =
				MVACCU(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			else
				chip_info->chip_reg_data.vacc[cnt] =
				MVACCS(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);

			if (chip_info->chip_reg_data.accumulation_mode[cnt] !=
			    PAC194X5X_ACCMODE_VBUS) {
				/* Integrate the accumulated power or current over
				 * the elapsed interval.
				 */
				tmp = chip_info->chip_reg_data.ctrl_lat_reg >> 12;

				if (tmp < pac194x5x_samp_fast_mode) {
					/* Find how much shift is required by the sample rate */
					/* The chip's sampling rate is 2^shift samples/sec */
					shift = shift_map_tbl[tmp];
					inc = chip_info->chip_reg_data.vacc[cnt] >> shift;
				} else {
					idx = chip_info->num_enabled_channels - 1;

					if (tmp == pac194x5x_samp_fast_mode) {
						fs = samp_rate_fast_mode_tbl[idx];
					} else if (tmp == pac194x5x_samp_burst_mode) {
						fs = samp_rate_burst_mode_tbl[idx];
					} else {
						dev_err(&client->dev,
							"Invalid sample rate index: %d!\n",
							tmp);
					}

					inc = div_u64(abs(chip_info->chip_reg_data.vacc[cnt]), fs);
					if (chip_info->chip_reg_data.vacc[cnt] < 0)
						inc = -inc;
				}
			} else {
				count = chip_info->chip_reg_data.total_samples_nr[cnt];
				inc_count = chip_info->chip_reg_data.acc_count;

				/* Check if total number of samples will overflow */
				if (unlikely(check_add_overflow(count, inc_count, &count))) {
					dev_err(&client->dev,
						"Number of samples on channel [%d] overflow!\n",
						cnt + 1);
					chip_info->chip_reg_data.total_samples_nr[cnt] = 0;
					chip_info->chip_reg_data.acc_val[cnt] = 0;
				}

				chip_info->chip_reg_data.total_samples_nr[cnt] += inc_count;

				inc = chip_info->chip_reg_data.vacc[cnt];
			}

			if (unlikely(check_add_overflow(stored_value, inc, &stored_value))) {
				if (is_negative(stored_value))
					chip_info->chip_reg_data.acc_val[cnt] = S64_MIN;
				else
					chip_info->chip_reg_data.acc_val[cnt] = S64_MAX;

				dev_err(&client->dev,
					"Overflow detected on channel [%d]!\n",
					cnt + 1);
			} else {
				chip_info->chip_reg_data.acc_val[cnt] += inc;
			}

			offset_reg_data += PAC194X5X_VACC_REG_LEN;
		}
	}

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((chip_info->chip_reg_data.ctrl_act_reg << cnt) & 0x80) == 0) {
			if (chip_info->chip_reg_data.vbus_mode[cnt] == PAC194X5X_UNIPOLAR_FSR_CFG)
				chip_info->chip_reg_data.vbus[cnt] =
				MVBUS_SENSEU(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			else
				chip_info->chip_reg_data.vbus[cnt] =
				MVBUS_SENSES(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			offset_reg_data += PAC194X5X_VBUS_SENSE_REG_LEN;
		}
	}

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((chip_info->chip_reg_data.ctrl_act_reg << cnt) & 0x80) == 0) {
			if (chip_info->chip_reg_data.vbus_mode[cnt] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG)
				chip_info->chip_reg_data.vsense[cnt] =
				MVBUS_SENSEU(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			else
				chip_info->chip_reg_data.vsense[cnt] =
				MVBUS_SENSES(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);

			offset_reg_data += PAC194X5X_VBUS_SENSE_REG_LEN;
		}
	}

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((chip_info->chip_reg_data.ctrl_act_reg << cnt) & 0x80) == 0) {
			if (chip_info->chip_reg_data.vbus_mode[cnt] == PAC194X5X_UNIPOLAR_FSR_CFG)
				chip_info->chip_reg_data.vbus_avg[cnt] =
				MVBUS_SENSEU(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			else
				chip_info->chip_reg_data.vbus_avg[cnt] =
				MVBUS_SENSES(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);

			offset_reg_data += PAC194X5X_VBUS_SENSE_REG_LEN;
		}
	}

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((chip_info->chip_reg_data.ctrl_act_reg << cnt) & 0x80) == 0) {
			if (chip_info->chip_reg_data.vsense_mode[cnt] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG)
				chip_info->chip_reg_data.vsense_avg[cnt] =
				MVBUS_SENSEU(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			else
				chip_info->chip_reg_data.vsense_avg[cnt] =
				MVBUS_SENSES(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);

			offset_reg_data += PAC194X5X_VBUS_SENSE_REG_LEN;
		}
	}

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((chip_info->chip_reg_data.ctrl_act_reg << cnt) & 0x80) == 0) {
			if (chip_info->chip_reg_data.vbus_mode[cnt] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG &&
			    chip_info->chip_reg_data.vsense_mode[cnt] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG) {
				chip_info->chip_reg_data.vpower[cnt] =
				MVPOWERU(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			} else {
				chip_info->chip_reg_data.vpower[cnt] =
				MVPOWERS(&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
			}
			offset_reg_data += PAC194X5X_VPOWER_REG_LEN;
		}
	}
reg_snapshot_err:
	mutex_unlock(&chip_info->lock);
	return ret;
}

static int pac194x5x_retrieve_data(struct pac194x5x_chip_info *chip_info,
				   u32 wait_time)
{
	int ret = 0;
	struct i2c_client *client = chip_info->client;
	/* Check if the minimal elapsed time has passed and if so,
	 * re-read the chip, otherwise the cached info is just fine
	 */
	if (time_after(jiffies, chip_info->chip_reg_data.jiffies_tstamp +
		       msecs_to_jiffies(PAC194X5X_MIN_POLLING_TIME))) {
		/* We need to re-read the chip values
		 * call the pac194x5x_reg_snapshot
		 */
		ret = pac194x5x_reg_snapshot(chip_info, true,
					     PAC194X5X_REFRESH_REG_ADDR,
					     wait_time);
		/* Re-schedule the work for the read registers timeout
		 * (to prevent chip regs saturation)
		 */
		ret = mod_timer(&chip_info->tmr_forced_update,
				chip_info->chip_reg_data.jiffies_tstamp +
				msecs_to_jiffies(PAC194X5X_MAX_RFSH_LIMIT));
		if (ret < 0)
			dev_err(&client->dev,
				"forced read timer cannot be modified!\n");
	}
	return ret;
}

static ssize_t in_power_acc_raw_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int idx;
	int ret;
	s64 curr_energy, int_part;
	int rem;

	idx = (int)(attr->attr.name[INDEX_IN_POWER_ACC_NAME] - '0') - 1;

	ret = pac194x5x_retrieve_data(chip_info, PAC194X5X_MIN_UPDATE_WAIT_TIME);
	if (ret < 0)
		return 0;

	/* Expresses the 64 bit energy value as a
	 * 64 bit integer and a 32 bit nano value
	 */
	curr_energy = chip_info->chip_reg_data.acc_val[idx];
	int_part = div_s64_rem(curr_energy, 1000000000, &rem);

	if (rem < 0)
		return sprintf(buf, "-%lld.%09u\n", abs(int_part),
				-rem);
	else
		return sprintf(buf, "%lld.%09u\n", int_part, abs(rem));
}

static ssize_t in_power_acc_scale_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int idx;
	unsigned int shunt, rem;
	u64 tmp, ref;

	if (chip_info->is_pac195x_family)
		ref = (u64)PAC195X_MAX_VPOWER_RSHIFTED_BY_29B;
	else
		ref = (u64)PAC194X_MAX_VPOWER_RSHIFTED_BY_29B;

	idx = (int)(attr->attr.name[INDEX_IN_POWER_ACC_NAME] - '0') - 1;

	if ((chip_info->chip_reg_data.vbus_mode[idx] ==
	    PAC194X5X_UNIPOLAR_FSR_CFG &&
	    chip_info->chip_reg_data.vsense_mode[idx] ==
	    PAC194X5X_UNIPOLAR_FSR_CFG) ||
	    chip_info->chip_reg_data.vbus_mode[idx] ==
	    PAC194X5X_BIPOLAR_HALF_FSR_CFG ||
	    chip_info->chip_reg_data.vsense_mode[idx] ==
	    PAC194X5X_BIPOLAR_HALF_FSR_CFG)
		ref = ref >> 1;

	shunt = chip_info->shunts[idx];

	tmp = div_u64((u64)ref * 1000000000LL, shunt);
	rem = do_div(tmp, 1000000000LL);

	return sprintf(buf, "%llu.%09u\n", tmp, rem);
}

static ssize_t in_current_acc_raw_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int idx;
	int ret;

	idx = (int)(attr->attr.name[INDEX_IN_CURRENT_ACC_NAME] - '0') - 1;

	ret = pac194x5x_retrieve_data(chip_info, PAC194X5X_MIN_UPDATE_WAIT_TIME);
	if (ret < 0)
		return 0;

	return sprintf(buf, "%lld\n", chip_info->chip_reg_data.acc_val[idx]);
}

static ssize_t in_current_acc_scale_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int idx;
	int shunt, rem;
	u64 tmp, ref;

	/* Currents - scale for mA - depends on the channel's shunt value
	 * (100mV * 1000000) / (2^16 * shunt(uohm))
	 */
	ref = (u64)PAC194X5X_MAX_VSENSE_NANO;

	idx = (int)(attr->attr.name[INDEX_IN_CURRENT_ACC_NAME] - '0') - 1;

	switch (chip_info->chip_reg_data.vsense_mode[idx]) {
	case PAC194X5X_UNIPOLAR_FSR_CFG:
	case PAC194X5X_BIPOLAR_HALF_FSR_CFG:
		shunt = chip_info->shunts[idx];
		break;
	case PAC194X5X_BIPOLAR_FSR_CFG:
		ref = ref << 1;
		shunt = chip_info->shunts[idx] >> 1;
		break;
	default:
		return 0;
	}

	/* Increasing precision
	 * (100mV * 1000000 * 1000000000) / 2^16 )
	 */
	tmp = div_u64(ref, shunt);
	rem = do_div(tmp, 1000000000LL);

	return sprintf(buf, "%lld.%09u\n", tmp, rem);
}

static ssize_t in_voltage_average_raw_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int idx;
	int ret;
	s64 acc_voltage;
	u32 samples_count;
	u64 tmp;

	idx = (int)(attr->attr.name[INDEX_IN_VOLTAGE_AVERAGE_NAME] - '0') - 1;

	ret = pac194x5x_retrieve_data(chip_info, PAC194X5X_MIN_UPDATE_WAIT_TIME);
	if (ret < 0)
		return 0;

	acc_voltage = chip_info->chip_reg_data.acc_val[idx];
	samples_count = chip_info->chip_reg_data.total_samples_nr[idx];

	tmp = div_u64(abs(acc_voltage), samples_count);

	if (unlikely(is_negative(acc_voltage)))
		return sprintf(buf, "-%lld\n", tmp);
	else
		return sprintf(buf, "%lld\n", tmp);
}

static ssize_t in_voltage_average_scale_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int idx;
	int vals[2];
	unsigned long long tmp;

	if (chip_info->is_pac195x_family)
		vals[0] = PAC195X_VOLTAGE_MILLIVOLTS_MAX;
	else
		vals[0] = PAC194X_VOLTAGE_MILLIVOLTS_MAX;

	idx = (int)(attr->attr.name[INDEX_IN_VOLTAGE_AVERAGE_NAME] - '0') - 1;

	switch (chip_info->chip_reg_data.vbus_mode[idx]) {
	case PAC194X5X_UNIPOLAR_FSR_CFG:
	case PAC194X5X_BIPOLAR_HALF_FSR_CFG:
		vals[1] = PAC194X5X_VOLTAGE_16B_RES;
		break;
	case PAC194X5X_BIPOLAR_FSR_CFG:
		vals[1] = PAC194X5X_VOLTAGE_15B_RES;
		break;
	default:
		break;
	}

	tmp = (s64)vals[0] * 1000000000LL >> vals[1];
	vals[1] = do_div(tmp, 1000000000LL);
	vals[0] = tmp;

	return sprintf(buf, "%d.%09u\n", vals[0], vals[1]);
}

static IIO_DEVICE_ATTR(in_power_acc1_raw, 0444,
		       in_power_acc_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_power_acc2_raw, 0444,
		       in_power_acc_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_power_acc3_raw, 0444,
		       in_power_acc_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_power_acc4_raw, 0444,
		       in_power_acc_raw_show, NULL, 0);

static IIO_DEVICE_ATTR(in_power_acc1_scale, 0444, in_power_acc_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_power_acc2_scale, 0444, in_power_acc_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_power_acc3_scale, 0444, in_power_acc_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_power_acc4_scale, 0444, in_power_acc_scale_show, NULL, 0);

static IIO_DEVICE_ATTR(in_current_acc1_raw, 0444,
		       in_current_acc_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_current_acc2_raw, 0444,
		       in_current_acc_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_current_acc3_raw, 0444,
		       in_current_acc_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_current_acc4_raw, 0444,
		       in_current_acc_raw_show, NULL, 0);

static IIO_DEVICE_ATTR(in_current_acc1_scale, 0444,
		       in_current_acc_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_current_acc2_scale, 0444,
		       in_current_acc_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_current_acc3_scale, 0444,
		       in_current_acc_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_current_acc4_scale, 0444,
		       in_current_acc_scale_show, NULL, 0);

static IIO_DEVICE_ATTR(in_voltage_average1_raw, 0444,
		       in_voltage_average_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_voltage_average2_raw, 0444,
		       in_voltage_average_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_voltage_average3_raw, 0444,
		       in_voltage_average_raw_show, NULL, 0);
static IIO_DEVICE_ATTR(in_voltage_average4_raw, 0444,
		       in_voltage_average_raw_show, NULL, 0);

static IIO_DEVICE_ATTR(in_voltage_average1_scale, 0444,
		       in_voltage_average_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_voltage_average2_scale, 0444,
		       in_voltage_average_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_voltage_average3_scale, 0444,
		       in_voltage_average_scale_show, NULL, 0);
static IIO_DEVICE_ATTR(in_voltage_average4_scale, 0444,
		       in_voltage_average_scale_show, NULL, 0);

static struct attribute *pac194x5x_power_acc_attributes[] = {
	PAC194X5X_DEV_ATTR(in_power_acc1_raw),
	PAC194X5X_DEV_ATTR(in_power_acc2_raw),
	PAC194X5X_DEV_ATTR(in_power_acc3_raw),
	PAC194X5X_DEV_ATTR(in_power_acc4_raw),
	PAC194X5X_DEV_ATTR(in_power_acc1_scale),
	PAC194X5X_DEV_ATTR(in_power_acc2_scale),
	PAC194X5X_DEV_ATTR(in_power_acc3_scale),
	PAC194X5X_DEV_ATTR(in_power_acc4_scale),
	NULL
};

static struct attribute *pac194x5x_current_acc_attributes[] = {
	PAC194X5X_DEV_ATTR(in_current_acc1_raw),
	PAC194X5X_DEV_ATTR(in_current_acc2_raw),
	PAC194X5X_DEV_ATTR(in_current_acc3_raw),
	PAC194X5X_DEV_ATTR(in_current_acc4_raw),
	PAC194X5X_DEV_ATTR(in_current_acc1_scale),
	PAC194X5X_DEV_ATTR(in_current_acc2_scale),
	PAC194X5X_DEV_ATTR(in_current_acc3_scale),
	PAC194X5X_DEV_ATTR(in_current_acc4_scale),
	NULL
};

static struct attribute *pac194x5x_voltage_acc_attributes[] = {
	PAC194X5X_DEV_ATTR(in_voltage_average1_raw),
	PAC194X5X_DEV_ATTR(in_voltage_average2_raw),
	PAC194X5X_DEV_ATTR(in_voltage_average3_raw),
	PAC194X5X_DEV_ATTR(in_voltage_average4_raw),
	PAC194X5X_DEV_ATTR(in_voltage_average1_scale),
	PAC194X5X_DEV_ATTR(in_voltage_average2_scale),
	PAC194X5X_DEV_ATTR(in_voltage_average3_scale),
	PAC194X5X_DEV_ATTR(in_voltage_average4_scale),
	NULL
};

static int pac194x5x_prep_custom_attributes(struct pac194x5x_chip_info *chip_info,
					    struct iio_dev *indio_dev)
{
	int i, j;
	int active_channels_count = 0;
	struct attribute **pac194x5x_custom_attrs;
	struct attribute_group *pac194x5x_group;
	int custom_attr_cnt;

	for (i = 0 ; i < chip_info->phys_channels; i++)
		if (chip_info->chip_reg_data.active_channels[i])
			active_channels_count++;

	pac194x5x_group = kzalloc(sizeof(*pac194x5x_group), GFP_KERNEL);

	/*
	 * Attributes for channel X:
	 *	- shunt_value_X, channel_name_X,
	 *	- one of pair atributes:
	 *		- in_power_accX_raw and in_power_acc1_scale
	 *		- in_current_accX_raw and in_current_accX_scale
	 *		- in_voltage_accX_raw and in_voltage_accX_scale
	 *	- reset_accumulators (shared attributs)
	 */
	custom_attr_cnt = PAC194X5X_DEVATTR_FOR_CHANNEL * active_channels_count;
	custom_attr_cnt += PAC194X5X_SHARED_DEVATTRS_COUNT;

	pac194x5x_custom_attrs = kzalloc(custom_attr_cnt *
					 sizeof(struct attribute *) + 1, GFP_KERNEL);

	j = 0;

	for (i = 0 ; i < chip_info->phys_channels; i++) {
		if (chip_info->chip_reg_data.active_channels[i]) {
			pac194x5x_custom_attrs[j++] =
				pac194x5x_all_attrs[PAC194X5X_COMMON_DEVATTR * i];
			pac194x5x_custom_attrs[j++] =
				pac194x5x_all_attrs[PAC194X5X_COMMON_DEVATTR * i + 1];

			if (chip_info->chip_reg_data.accumulation_mode[i] ==
			    PAC194X5X_ACCMODE_VPOWER) {
				pac194x5x_custom_attrs[j++] = pac194x5x_power_acc_attributes[i];
				pac194x5x_custom_attrs[j++] =
				pac194x5x_power_acc_attributes[PAC194X5X_MAX_NUM_CHANNELS + i];
			} else if (chip_info->chip_reg_data.accumulation_mode[i] ==
				   PAC194X5X_ACCMODE_VSENSE) {
				pac194x5x_custom_attrs[j++] = pac194x5x_current_acc_attributes[i];
				pac194x5x_custom_attrs[j++] =
				pac194x5x_current_acc_attributes[PAC194X5X_MAX_NUM_CHANNELS + i];
			} else if (chip_info->chip_reg_data.accumulation_mode[i] ==
				   PAC194X5X_ACCMODE_VBUS) {
				pac194x5x_custom_attrs[j++] = pac194x5x_voltage_acc_attributes[i];
				pac194x5x_custom_attrs[j++] =
				pac194x5x_voltage_acc_attributes[PAC194X5X_MAX_NUM_CHANNELS + i];
			}
		}
	}

	for (i = 0; i < PAC194X5X_SHARED_DEVATTRS_COUNT; i++)
		pac194x5x_custom_attrs[j++] =
			pac194x5x_all_attrs[PAC194X5X_COMMON_DEVATTR *
			chip_info->phys_channels + i];

	pac194x5x_group->attrs = pac194x5x_custom_attrs;
	chip_info->pac194x5x_info.attrs = pac194x5x_group;
	return 0;
}

static int pac194x5x_match_samp_mode(struct pac194x5x_chip_info *chip_info,
				     char *new_samp_rate)
{
	int cnt;

	for (cnt = 0; cnt < ARRAY_SIZE(pac194x5x_frequency_avail); cnt++) {
		if (!strcmp(new_samp_rate, pac194x5x_frequency_avail[cnt])) {
			chip_info->sampling_mode = cnt;
			break;
		}
	}
	if (cnt == ARRAY_SIZE(pac194x5x_frequency_avail))
		return cnt;

	return 0;
}

static int pac194x5x_frequency_set(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   unsigned int mode)
{
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int ret;
	u16 ctr_reg = 0;
	u16 tmp;

	ret = pac194x5x_i2c_read(chip_info->client, PAC194X5X_CTRL_ACT_REG_ADDR,
				 (u8 *)&tmp, PAC194X5X_CTRL_ACT_REG_LEN);
	if (ret < 0) {
		dev_err(&indio_dev->dev,
			"%s - cannot read PAC194X5X regs from 0x%02X\n",
			__func__, PAC194X5X_CTRL_ACT_REG_ADDR);
		return ret;
	}

	be16_to_cpus(&tmp);
	tmp &= (u16)0x00ff;
	tmp |= (u16)(mode << 12);
	ctr_reg = tmp;

	mutex_lock(&chip_info->lock);

	cpu_to_be16s(&ctr_reg);
	ret = pac194x5x_i2c_write(chip_info->client, PAC194X5X_CTRL_REG_ADDR, (u8 *)&ctr_reg, 2);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Failed to configure sampling mode\n");
		return ret;
	}
	chip_info->sampling_mode = mode;
	chip_info->chip_reg_data.ctrl_act_reg = tmp;
	mutex_unlock(&chip_info->lock);
	ret = pac194x5x_retrieve_data(chip_info, PAC194X5X_MIN_UPDATE_WAIT_TIME);
	if (ret < 0)
		return ret;
	return 0;
}

static int pac194x5x_frequency_get(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct pac194x5x_chip_info *chip_info;

	chip_info = iio_priv(indio_dev);
	return chip_info->sampling_mode;
}

static const struct iio_enum sampling_mode_enum = {
	.items = pac194x5x_frequency_avail,
	.num_items = ARRAY_SIZE(pac194x5x_frequency_avail),
	.set = pac194x5x_frequency_set,
	.get = pac194x5x_frequency_get,
};

static const struct iio_chan_spec_ext_info pac194x5x_ext_info[] = {
	IIO_ENUM("sampling_frequency", IIO_SHARED_BY_ALL, &sampling_mode_enum),
	{
		.name = "sampling_frequency_available",
		.shared = IIO_SHARED_BY_ALL,
		.read = iio_enum_available_read,
		.private = (uintptr_t)&sampling_mode_enum,
	},
	{}
};

/*
 * pac194x5x_read_raw() - data read function.
 * @indio_dev:    the struct iio_dev associated with this device instance
 * @chan:   the channel whose data is to be read
 * @val:    first element of returned value (typically INT)
 * @val2:   second element of returned value (typically MICRO)
 * @mask:   what we actually want to read as per the info_mask_*
 *          in iio_chan_spec.
 */
static int pac194x5x_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);
	int ret = -EINVAL;
	u64 tmp;
	int idx;

	ret = pac194x5x_retrieve_data(chip_info, PAC194X5X_MIN_UPDATE_WAIT_TIME);
	if (ret < 0)
		return ret;

	ret = -EINVAL;

	idx = chan->channel - 1;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			switch (chan->address) {
			case PAC194X5X_VBUS_1_ADDR:
			case PAC194X5X_VBUS_2_ADDR:
			case PAC194X5X_VBUS_3_ADDR:
			case PAC194X5X_VBUS_4_ADDR:
				*val = chip_info->chip_reg_data.vbus[idx];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_CURRENT:
			switch (chan->address) {
			case PAC194X5X_VSENSE_1_ADDR:
			case PAC194X5X_VSENSE_2_ADDR:
			case PAC194X5X_VSENSE_3_ADDR:
			case PAC194X5X_VSENSE_4_ADDR:
				*val = chip_info->chip_reg_data.vsense[idx];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_POWER:
			switch (chan->address) {
			case PAC194X5X_VPOWER_1_ADDR:
			case PAC194X5X_VPOWER_2_ADDR:
			case PAC194X5X_VPOWER_3_ADDR:
			case PAC194X5X_VPOWER_4_ADDR:
				*val = chip_info->chip_reg_data.vpower[idx];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			switch (chan->address) {
			case PAC194X5X_VBUS_AVG_1_ADDR:
			case PAC194X5X_VBUS_AVG_2_ADDR:
			case PAC194X5X_VBUS_AVG_3_ADDR:
			case PAC194X5X_VBUS_AVG_4_ADDR:
				*val = chip_info->chip_reg_data.vbus_avg[idx];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_CURRENT:
			switch (chan->address) {
			case PAC194X5X_VSENSE_AVG_1_ADDR:
			case PAC194X5X_VSENSE_AVG_2_ADDR:
			case PAC194X5X_VSENSE_AVG_3_ADDR:
			case PAC194X5X_VSENSE_AVG_4_ADDR:
				*val = chip_info->chip_reg_data.vsense_avg[idx];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->address) {
		case PAC194X5X_VBUS_1_ADDR:
		case PAC194X5X_VBUS_2_ADDR:
		case PAC194X5X_VBUS_3_ADDR:
		case PAC194X5X_VBUS_4_ADDR:
		case PAC194X5X_VBUS_AVG_1_ADDR:
		case PAC194X5X_VBUS_AVG_2_ADDR:
		case PAC194X5X_VBUS_AVG_3_ADDR:
		case PAC194X5X_VBUS_AVG_4_ADDR:
			if (chip_info->is_pac195x_family)
				*val = PAC195X_VOLTAGE_MILLIVOLTS_MAX;
			else
				*val = PAC194X_VOLTAGE_MILLIVOLTS_MAX;

			switch (chip_info->chip_reg_data.vbus_mode[idx]) {
			case PAC194X5X_UNIPOLAR_FSR_CFG:
			case PAC194X5X_BIPOLAR_HALF_FSR_CFG:
				*val2 = PAC194X5X_VOLTAGE_16B_RES;
				break;
			case PAC194X5X_BIPOLAR_FSR_CFG:
				*val2 = PAC194X5X_VOLTAGE_15B_RES;
				break;
			default:
				break;
			}
			return IIO_VAL_FRACTIONAL_LOG2;
		/*
		 * Currents - scale for mA - depends on the
		 * channel's shunt value
		 * (100mV * 1000000) / (2^16 * shunt(microOhm))
		 */
		case PAC194X5X_VSENSE_1_ADDR:
		case PAC194X5X_VSENSE_2_ADDR:
		case PAC194X5X_VSENSE_3_ADDR:
		case PAC194X5X_VSENSE_4_ADDR:
		case PAC194X5X_VSENSE_AVG_1_ADDR:
		case PAC194X5X_VSENSE_AVG_2_ADDR:
		case PAC194X5X_VSENSE_AVG_3_ADDR:
		case PAC194X5X_VSENSE_AVG_4_ADDR:
			*val = PAC194X5X_MAX_VSENSE_RSHIFTED_BY_15B;
			switch (chip_info->chip_reg_data.vsense_mode[idx]) {
			case PAC194X5X_UNIPOLAR_FSR_CFG:
			case PAC194X5X_BIPOLAR_HALF_FSR_CFG:
				*val = *val >> 1;
				*val2 = chip_info->shunts[idx];
				break;
			case PAC194X5X_BIPOLAR_FSR_CFG:
				*val2 = chip_info->shunts[idx];
				break;
			default:
				break;
			}
			return IIO_VAL_FRACTIONAL;
		/*
		 * Power - mW - it will use the combined scale
		 * for current and voltage
		 * current(mA) * voltage(mV) = power (uW)
		 */
		case PAC194X5X_VPOWER_1_ADDR:
		case PAC194X5X_VPOWER_2_ADDR:
		case PAC194X5X_VPOWER_3_ADDR:
		case PAC194X5X_VPOWER_4_ADDR:
			if (chip_info->is_pac195x_family)
				tmp = PAC195X_PRODUCT_VOLTAGE_PV_FSR;
			else
				tmp = PAC194X_PRODUCT_VOLTAGE_PV_FSR;

			do_div(tmp, chip_info->shunts[idx]);
			*val = (int)tmp;
			if ((chip_info->chip_reg_data.vbus_mode[idx] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG &&
			    chip_info->chip_reg_data.vsense_mode[idx] ==
			    PAC194X5X_UNIPOLAR_FSR_CFG) ||
			    chip_info->chip_reg_data.vbus_mode[idx] ==
			    PAC194X5X_BIPOLAR_HALF_FSR_CFG ||
			    chip_info->chip_reg_data.vsense_mode[idx] ==
			    PAC194X5X_BIPOLAR_HALF_FSR_CFG)
				*val2 = PAC194X5X_POWER_30B_RES;
			else
				*val2 = PAC194X5X_POWER_29B_RES;

			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static void pac194x5x_work_periodic_rfsh(struct work_struct *work)
{
	struct pac194x5x_chip_info *chip_info = to_pac194x5x_chip_info(work);

	pac194x5x_reg_snapshot(chip_info, true, PAC194X5X_REFRESH_REG_ADDR,
			       PAC194X5X_MIN_UPDATE_WAIT_TIME);
}

static void pac194x5x_read_reg_timeout(struct timer_list *t)
{
	int ret;
	struct pac194x5x_chip_info *chip_info = from_timer(chip_info, t, tmr_forced_update);
	struct i2c_client *client = chip_info->client;

	ret = mod_timer(&chip_info->tmr_forced_update,
			jiffies + msecs_to_jiffies(PAC194X5X_MAX_RFSH_LIMIT));
	if (ret < 0)
		dev_err(&client->dev, "forced read timer cannot be modified!\n");

	queue_work(chip_info->wq_chip, &chip_info->work_chip_rfsh);
}

static int pac194x5x_chip_identify(struct pac194x5x_chip_info *chip_info)
{
	int ret = 0;
	struct i2c_client *client = chip_info->client;
	u8 chip_rev_info[3];

	ret = pac194x5x_i2c_read(client, PAC194X5X_PID_REG_ADDR, (u8 *)chip_rev_info, 3);
	if (ret < 0) {
		dev_err(&client->dev, "cannot read PAC194X5X revision\n");
		goto chip_identify_err;
	}
	if (chip_rev_info[0] != pac194x5x_chip_config[chip_info->chip_variant].prod_id) {
		ret = -EINVAL;
		dev_err(&client->dev,
			"product ID (0x%02X, 0x%02X, 0x%02X) for this part doesn't match\n",
			chip_rev_info[0], chip_rev_info[1], chip_rev_info[2]);
		goto chip_identify_err;
	}
	dev_info(&client->dev, "Chip revision: 0x%02X\n", chip_rev_info[2]);
	chip_info->chip_revision = chip_rev_info[2];

	switch (chip_rev_info[0]) {
	case PAC_PRODUCT_ID_1941_1:
	case PAC_PRODUCT_ID_1942_1:
	case PAC_PRODUCT_ID_1943_1:
	case PAC_PRODUCT_ID_1944_1:
	case PAC_PRODUCT_ID_1941_2:
	case PAC_PRODUCT_ID_1942_2:
		chip_info->is_pac195x_family = false;
		break;
	case PAC_PRODUCT_ID_1951_1:
	case PAC_PRODUCT_ID_1952_1:
	case PAC_PRODUCT_ID_1953_1:
	case PAC_PRODUCT_ID_1954_1:
	case PAC_PRODUCT_ID_1951_2:
	case PAC_PRODUCT_ID_1952_2:
		chip_info->is_pac195x_family = true;
		break;
	default:
		break;
	}

chip_identify_err:
	return ret;
}

static int pac194x5x_setup_periodic_refresh(struct pac194x5x_chip_info *chip_info)
{
	int ret = 0;

	chip_info->wq_chip = create_workqueue("wq_pac194x5x");
	INIT_WORK(&chip_info->work_chip_rfsh, pac194x5x_work_periodic_rfsh);

	/* Setup the latest moment for reading the regs before saturation */
	timer_setup(&chip_info->tmr_forced_update, pac194x5x_read_reg_timeout, 0);

	mod_timer(&chip_info->tmr_forced_update,
		  jiffies + msecs_to_jiffies(PAC194X5X_MAX_RFSH_LIMIT));

	return ret;
}

struct attribute_group pac194x5x_group = {
	.attrs = pac194x5x_all_attrs
};

static const char *pac194x5x_get_of_match_entry(struct i2c_client *client)
{
	const struct of_device_id *match;

	match = of_match_node(pac194x5x_of_match, client->dev.of_node);
	return match->compatible;
}

static const char *pac194x5x_match_of_device(struct i2c_client *client,
					     struct pac194x5x_chip_info *chip_info)
{
	struct device_node *node;
	unsigned int current_channel;
	const char *ptr_name;
	int idx;
	int temp;

	ptr_name = pac194x5x_get_of_match_entry(client);

	if (of_property_read_string(client->dev.of_node, "microchip,samp-rate",
				    (const char **)&chip_info->sample_rate_value)) {
		dev_err(&client->dev, "Cannot read sample rate value ...\n");
		return NULL;
	}

	if (pac194x5x_match_samp_mode(chip_info, chip_info->sample_rate_value)) {
		dev_err(&client->dev,
			"The given sample rate value is not supported: %s\n",
			chip_info->sample_rate_value);
		return NULL;
	}
	current_channel = 1;
	for_each_child_of_node(client->dev.of_node, node) {
		if (of_property_read_u32(node, "reg", &idx)) {
			dev_err(&client->dev,
				"invalid channel_index %d value on %s\n",
				idx, node->full_name);
			return NULL;
		}
		idx--;
		if (current_channel >= (chip_info->phys_channels + 1) ||
		    idx >= chip_info->phys_channels || idx < 0) {
			dev_err(&client->dev,
				"invalid channel_index %d value on %s\n",
				(idx + 1), node->full_name);
			return NULL;
		}

		chip_info->chip_reg_data.active_channels[idx] = true;
		if (of_property_read_u32(node, "microchip,uohms-shunt-res",
					 &chip_info->shunts[idx])) {
			dev_err(&client->dev,
				"invalid shunt-resistor value on %s\n",
				node->full_name);
			return NULL;
		}
		if (of_property_read_string(node, "microchip,rail-name",
					    (const char **)&chip_info->channel_names[idx])) {
			dev_err(&client->dev,
				"invalid rail-name value on %s\n",
				node->full_name);
			return NULL;
		}

		if (of_property_read_u32(node, "microchip,vbus-mode", &temp)) {
			dev_err(&client->dev,
				"invalid vbus-mode value on %s\n",
				node->full_name);
			return NULL;
		}
		if (temp == PAC194X5X_UNIPOLAR_FSR_CFG ||
		    temp == PAC194X5X_BIPOLAR_FSR_CFG  ||
		    temp == PAC194X5X_BIPOLAR_HALF_FSR_CFG) {
			chip_info->chip_reg_data.vbus_mode[idx] = temp;
		} else {
			dev_err(&client->dev,
				"invalid vbus-mode value on %s\n",
				node->full_name);
			return NULL;
		}

		if (of_property_read_u32(node, "microchip,vsense-mode", &temp)) {
			dev_err(&client->dev,
				"invalid vsense-mode value on %s\n",
				node->full_name);
			return NULL;
		}
		if (temp == PAC194X5X_UNIPOLAR_FSR_CFG ||
		    temp == PAC194X5X_BIPOLAR_FSR_CFG  ||
		    temp == PAC194X5X_BIPOLAR_HALF_FSR_CFG) {
			chip_info->chip_reg_data.vsense_mode[idx] = temp;
		} else {
			dev_err(&client->dev,
				"invalid vsense-mode value on %s\n",
				node->full_name);
			return NULL;
		}

		if (of_property_read_u32(node, "microchip,accumulation-mode", &temp)) {
			dev_err(&client->dev,
				"invalid accumulation-mode value on %s\n",
				node->full_name);
			return NULL;
		}
		if (temp == PAC194X5X_ACCMODE_VPOWER ||
		    temp == PAC194X5X_ACCMODE_VSENSE ||
		    temp == PAC194X5X_ACCMODE_VBUS) {
			chip_info->chip_reg_data.accumulation_mode[idx] = temp;
		} else {
			dev_err(&client->dev,
				"invalid mode for accumulator value on %s\n",
				node->full_name);
		}
		current_channel++;
	}
	return ptr_name;
}

static int pac194x5x_chip_configure(struct pac194x5x_chip_info *chip_info)
{
	int cnt, ret = 0;
	struct i2c_client *client = chip_info->client;
	u8 regs[PAC194X5X_CTRL_STATUS_INFO_LEN];
	u32 wait_time;
	u8 val_accum;
	u16 val_ctrl;
	u16 tmp;

	/*
	 * Counting how many channels are enabled and store
	 * this information within the driver data
	 */
	cnt = 0;
	chip_info->num_enabled_channels = 0;
	while (cnt < chip_info->phys_channels) {
		if (chip_info->chip_reg_data.active_channels[cnt])
			chip_info->num_enabled_channels++;
		cnt++;
	}

	ret = pac194x5x_i2c_read(client, PAC194X5X_CTRL_STAT_REGS_ADDR,
				 (u8 *)regs, PAC194X5X_CTRL_STATUS_INFO_LEN);
	if (ret < 0) {
		dev_err(&client->dev, "%s - cannot read PAC194X5X regs from 0x%02X\n",
			__func__, PAC194X5X_CTRL_STAT_REGS_ADDR);
		return ret;
	}

	/*
	 * The current/voltage can be measured uni or bi-dir or half FSR
	 * no SLOW triggered REFRESH, clear POR
	 */
	tmp = NEG_PWR_REG(chip_info->chip_reg_data.vsense_mode[0],
			  chip_info->chip_reg_data.vsense_mode[1],
			  chip_info->chip_reg_data.vsense_mode[2],
			  chip_info->chip_reg_data.vsense_mode[3],
			  chip_info->chip_reg_data.vbus_mode[0],
			  chip_info->chip_reg_data.vbus_mode[1],
			  chip_info->chip_reg_data.vbus_mode[2],
			  chip_info->chip_reg_data.vbus_mode[3]);

	regs[PAC194X5X_NEG_PWR_REG_OFF] = (u8)(tmp >> 8);
	regs[PAC194X5X_NEG_PWR_REG_OFF + 1] = (u8)tmp & 0xff;

	regs[PAC194X5X_SLOW_REG_OFF] = 0;

	ret = pac194x5x_i2c_write(client, PAC194X5X_CTRL_STAT_REGS_ADDR,
				  (u8 *)regs, 4);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot write PAC194X5X regs from 0x%02X\n",
			__func__, PAC194X5X_CTRL_STAT_REGS_ADDR);
		return ret;
	}
	/* Write the CHANNEL_N_OFF from CTRL REGISTER */
	val_ctrl = CTRL_REG(chip_info->sampling_mode, 0, 0,
			    chip_info->chip_reg_data.active_channels[0],
			    chip_info->chip_reg_data.active_channels[1],
			    chip_info->chip_reg_data.active_channels[2],
			    chip_info->chip_reg_data.active_channels[3]);

	cpu_to_be16s(&val_ctrl);

	ret = pac194x5x_i2c_write(client, PAC194X5X_CTRL_REG_ADDR, ((u8 *)&val_ctrl), 2);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot write PAC194X5X ctrl reg at 0x%02X\n",
			__func__, PAC194X5X_CTRL_REG_ADDR);
		return ret;
	}

	val_accum = ACCUM_REG(chip_info->chip_reg_data.accumulation_mode[0],
			      chip_info->chip_reg_data.accumulation_mode[1],
			      chip_info->chip_reg_data.accumulation_mode[2],
			      chip_info->chip_reg_data.accumulation_mode[3]);

	ret = pac194x5x_i2c_write(client, PAC194X5X_ACCUM_REG_ADDR, ((u8 *)&val_accum), 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot write PAC194X5X ctrl reg at 0x%02X\n",
			__func__, PAC194X5X_ACCUM_REG_ADDR);
		return ret;
	}

	/*
	 * Sending a REFRESH to the chip, so the new settings take place
	 * as well as resetting the accumulators
	 */
	ret = pac194x5x_i2c_send_byte(client, PAC194X5X_REFRESH_REG_ADDR);
	if (ret < 0) {
		dev_err(&client->dev, "%s - cannot send byte to PAC194X5X 0x%02X reg\n",
			__func__, PAC194X5X_REFRESH_REG_ADDR);
		return ret;
	}

	/*
	 * Get the current(in the chip) sampling speed and compute the
	 * required timeout based on its value the timeout is 1/sampling_speed
	 * wait the maximum amount of time to be on the safe side - the
	 * maximum wait time is for 8sps
	 */
	wait_time = (1024 / samp_rate_map_tbl[chip_info->sampling_mode]) * 1000;
	usleep_range(wait_time, wait_time + 100);

	ret = pac194x5x_setup_periodic_refresh(chip_info);
	return ret;
}

static const struct iio_chan_spec pac194x5x_single_channel[] = {
	PAC194X5X_VPOWER_CHANNEL(0, 0, PAC194X5X_VPOWER_1_ADDR),
	PAC194X5X_VBUS_CHANNEL(0, 0, PAC194X5X_VBUS_1_ADDR),
	PAC194X5X_VSENSE_CHANNEL(0, 0, PAC194X5X_VSENSE_1_ADDR),
	PAC194X5X_VBUS_AVG_CHANNEL(0, 0, PAC194X5X_VBUS_AVG_1_ADDR),
	PAC194X5X_VSENSE_AVG_CHANNEL(0, 0, PAC194X5X_VSENSE_AVG_1_ADDR),
};

static int pac194x5x_prep_iio_channels(struct pac194x5x_chip_info *chip_info,
				       struct iio_dev *indio_dev)
{
	struct i2c_client *client = chip_info->client;
	struct iio_chan_spec *ch_sp;
	int channel_size, channel_attribute_count, attribute_count;
	int cnt;
	void *dyn_ch_struct, *tmp_data;

	/* Finding out dynamically how many IIO channels we need */
	channel_attribute_count = 0;
	channel_size = 0;
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (chip_info->chip_reg_data.active_channels[cnt]) {
			/* add the size of the properties of one chip physical channel */
			channel_size += sizeof(pac194x5x_single_channel);
			/* count how many enabled channels we have */
			channel_attribute_count += ARRAY_SIZE(pac194x5x_single_channel);
			dev_info(&client->dev, ":%s: Channel %d active\n", __func__, cnt + 1);
		}
	}
	/* Adding the timestamp channel size */
	channel_size += sizeof(pac194x5x_ts);
	/* Adding one more channel which is the timestamp */
	attribute_count = channel_attribute_count + 1;

	dev_info(&client->dev, ":%s: Active chip attributes: %d\n", __func__, attribute_count);
	dyn_ch_struct = kzalloc(channel_size, GFP_KERNEL);
	if (!dyn_ch_struct)
		return -EINVAL;

	tmp_data = dyn_ch_struct;
	/* Populate the dynamic channels and make all the adjustments */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (chip_info->chip_reg_data.active_channels[cnt]) {
			memcpy(tmp_data, pac194x5x_single_channel,
			       sizeof(pac194x5x_single_channel));
			ch_sp = (struct iio_chan_spec *)tmp_data;
			ch_sp[IIO_POW].channel = cnt + 1;
			ch_sp[IIO_POW].scan_index = cnt;
			ch_sp[IIO_POW].address = cnt + PAC194X5X_VPOWER_1_ADDR;
			ch_sp[IIO_VOLT].channel = cnt + 1;
			ch_sp[IIO_VOLT].scan_index = cnt;
			ch_sp[IIO_VOLT].address = cnt + PAC194X5X_VBUS_1_ADDR;
			ch_sp[IIO_CRT].channel = cnt + 1;
			ch_sp[IIO_CRT].scan_index = cnt;
			ch_sp[IIO_CRT].address = cnt + PAC194X5X_VSENSE_1_ADDR;
			ch_sp[IIO_VOLTAVG].channel = cnt + 1;
			ch_sp[IIO_VOLTAVG].scan_index = cnt;
			ch_sp[IIO_VOLTAVG].address = cnt + PAC194X5X_VBUS_AVG_1_ADDR;
			ch_sp[IIO_CRTAVG].channel = cnt + 1;
			ch_sp[IIO_CRTAVG].scan_index = cnt;
			ch_sp[IIO_CRTAVG].address = cnt + PAC194X5X_VSENSE_AVG_1_ADDR;

			/* advance the pointer */
			tmp_data += sizeof(pac194x5x_single_channel);
		}
	}
	/* Copy the timestamp channel */
	memcpy(tmp_data, pac194x5x_ts, sizeof(pac194x5x_ts));
	ch_sp = (struct iio_chan_spec *)tmp_data;
	ch_sp[0].scan_index = attribute_count - 1;

	/*
	 * Send the updated dynamic channel structure information towards IIO
	 * prepare the required field for IIO class registration
	 */
	indio_dev->num_channels = attribute_count;
	indio_dev->channels = kmemdup((const struct iio_chan_spec *)dyn_ch_struct,
				      channel_size, GFP_KERNEL);
	if (!indio_dev->channels)
		return -EINVAL;

	kfree(dyn_ch_struct);

	return 0;
}

static int pac194x5x_remove(struct i2c_client *client)
{
	int ret;
	struct iio_dev *indio_dev = dev_get_drvdata(&client->dev);
	struct pac194x5x_chip_info *chip_info = iio_priv(indio_dev);

	kfree(indio_dev->channels);
	ret = try_to_del_timer_sync(&chip_info->tmr_forced_update);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot delete the forced readout timer\n",
			__func__);
		return ret;
	}
	if (chip_info->wq_chip) {
		cancel_work_sync(&chip_info->work_chip_rfsh);
		flush_workqueue(chip_info->wq_chip);
		destroy_workqueue(chip_info->wq_chip);
	}
	kfree(chip_info->pac194x5x_info.attrs->attrs);
	kfree(chip_info->pac194x5x_info.attrs);

	return 0;
}

static int pac194x5x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pac194x5x_chip_info *chip_info;
	struct iio_dev *indio_dev;
	const char *name = NULL;
	int cnt, ret = 0;
	int dev_id = 0;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip_info));
	if (!indio_dev)
		return -ENOMEM;

	chip_info = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	chip_info->client = client;

	name = id->name;
	dev_id = id->driver_data;
	chip_info->chip_variant = dev_id;
	chip_info->phys_channels = pac194x5x_chip_config[dev_id].phys_channels;

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		chip_info->chip_reg_data.active_channels[cnt] = false;
		chip_info->shunts[cnt] = SHUNT_UOHMS_DEFAULT;
	}

	ret = pac194x5x_chip_identify(chip_info);
	if (ret < 0)
		return -EINVAL;

	if (!client->dev.of_node || (!of_get_next_child(client->dev.of_node, NULL)))
		return -EINVAL;

	name = pac194x5x_match_of_device(client, chip_info);

	if (!name) {
		dev_err(&client->dev,
			"DT parameter parsing returned an error\n");
		return -EINVAL;
	}

	mutex_init(&chip_info->lock);

	ret = pac194x5x_chip_configure(chip_info);
	ret = pac194x5x_prep_iio_channels(chip_info, indio_dev);
	if (ret < 0)
		goto free_chan_attr_mem;

	ret = pac194x5x_prep_custom_attributes(chip_info, indio_dev);

	chip_info->pac194x5x_info.read_raw = pac194x5x_read_raw;

	indio_dev->info = &chip_info->pac194x5x_info;
	indio_dev->name = name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = pac194x5x_reg_snapshot(chip_info, true, false,
				     PAC194X5X_MIN_UPDATE_WAIT_TIME);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
free_chan_attr_mem:
		pac194x5x_remove(client);
	}
	return 0;
}

static struct i2c_driver pac194x5x_driver = {
	.driver = {
		.name = "pac194x5x",
		.of_match_table = pac194x5x_of_match,
	},
	.probe = pac194x5x_probe,
	.remove = pac194x5x_remove,
	.id_table = pac194x5x_id,
};

module_i2c_driver(pac194x5x_driver);

MODULE_AUTHOR("Victor Tudose");
MODULE_AUTHOR("Marius Cristea <marius.cristea@microchip.com>");
MODULE_DESCRIPTION("Microchip PAC194X and PAC195X Power Monitor");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1.0");
