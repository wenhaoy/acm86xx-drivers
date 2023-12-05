// SPDX-License-Identifier: GPL-2.0
//
// Driver for the ACM8623 Audio Amplifier
//
// Author: Wenhao Yang <wenhaoy@acme-semi.com>
//

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

/* register address */
#define REG_PAGE		0x00
#define REG_DEVICE_STATE	0x04
#define REG_STATE_REPORT	0x16
#define REG_GLOBAL_FAULT1	0x17
#define REG_GLOBAL_FAULT2	0x18
#define REG_GLOBAL_FAULT3	0x19

/* DEVICE_STATE register values */
#define DEVICE_STATE_DEEP_SLEEP	0x00
#define DEVICE_STATE_SLEEP	0x01
#define DEVICE_STATE_HIZ	0x02
#define DEVICE_STATE_PLAY	0x03

#define DEVICE_STATE_MUTE	0x0C

/* This sequence of register writes must always be sent, prior to the
 * 5ms delay while we wait for the DSP to boot.
 */
static const uint8_t dsp_cfg_preboot[] = {
	0x00, 0x00, 0x04, 0x00, 0xfc, 0x86, 0xfd, 0x22,
	0xfe, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t dsp_cfg_default[] = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b,
	0x5c, 0x80, 0x5d, 0x9e, 0x5e, 0x02, 0x5f, 0x9e,
	0x60, 0x80, 0x61, 0x9e, 0x62, 0x03, 0x63, 0x9e,
	0x00, 0x05, 0xb0, 0x08, 0xb1, 0x00, 0xb2, 0x00,
	0xb3, 0x00, 0xb4, 0x00, 0xb5, 0x00, 0xb6, 0x00,
	0xb7, 0x00, 0xb8, 0x00, 0xb9, 0x00, 0xba, 0x00,
	0xbb, 0x00, 0xbc, 0x08, 0xbd, 0x00, 0xbe, 0x00,
	0xbf, 0x00, 0xc0, 0x08, 0xc1, 0x00, 0xc2, 0x00,
	0xc3, 0x00, 0xc4, 0x08, 0xc5, 0x00, 0xc6, 0x00,
	0xc7, 0x00, 0xc8, 0x00, 0xc9, 0xe2, 0xca, 0xc4,
	0xcb, 0x6b, 0x00, 0x06, 0x38, 0x08, 0x39, 0x00,
	0x3a, 0x00, 0x3b, 0x00, 0x3c, 0x00, 0x3d, 0x00,
	0x3e, 0x00, 0x3f, 0x00, 0x40, 0x00, 0x41, 0x00,
	0x42, 0x00, 0x43, 0x00, 0x44, 0x00, 0x45, 0x00,
	0x46, 0x00, 0x47, 0x00, 0x48, 0x00, 0x49, 0x00,
	0x4a, 0x00, 0x4b, 0x00, 0xb0, 0x08, 0xb1, 0x00,
	0xb2, 0x00, 0xb3, 0x00, 0xb4, 0x00, 0xb5, 0x00,
	0xb6, 0x00, 0xb7, 0x00, 0xb8, 0x00, 0xb9, 0x00,
	0xba, 0x00, 0xbb, 0x00, 0xbc, 0x00, 0xbd, 0x00,
	0xbe, 0x00, 0xbf, 0x00, 0xc0, 0x00, 0xc1, 0x00,
	0xc2, 0x00, 0xc3, 0x00, 0x4c, 0x08, 0x4d, 0x00,
	0x4e, 0x00, 0x4f, 0x00, 0x50, 0x00, 0x51, 0x00,
	0x52, 0x00, 0x53, 0x00, 0x54, 0x00, 0x55, 0x00,
	0x56, 0x00, 0x57, 0x00, 0x58, 0x00, 0x59, 0x00,
	0x5a, 0x00, 0x5b, 0x00, 0x5c, 0x00, 0x5d, 0x00,
	0x5e, 0x00, 0x5f, 0x00, 0xc4, 0x08, 0xc5, 0x00,
	0xc6, 0x00, 0xc7, 0x00, 0xc8, 0x00, 0xc9, 0x00,
	0xca, 0x00, 0xcb, 0x00, 0xcc, 0x00, 0xcd, 0x00,
	0xce, 0x00, 0xcf, 0x00, 0xd0, 0x00, 0xd1, 0x00,
	0xd2, 0x00, 0xd3, 0x00, 0xd4, 0x00, 0xd5, 0x00,
	0xd6, 0x00, 0xd7, 0x00, 0x10, 0x08, 0x11, 0x00,
	0x12, 0x00, 0x13, 0x00, 0x14, 0x00, 0x15, 0x00,
	0x16, 0x00, 0x17, 0x00, 0x18, 0x00, 0x19, 0x00,
	0x1a, 0x00, 0x1b, 0x00, 0x1c, 0x00, 0x1d, 0x00,
	0x1e, 0x00, 0x1f, 0x00, 0x20, 0x00, 0x21, 0x00,
	0x22, 0x00, 0x23, 0x00, 0x88, 0x08, 0x89, 0x00,
	0x8a, 0x00, 0x8b, 0x00, 0x8c, 0x00, 0x8d, 0x00,
	0x8e, 0x00, 0x8f, 0x00, 0x90, 0x00, 0x91, 0x00,
	0x92, 0x00, 0x93, 0x00, 0x94, 0x00, 0x95, 0x00,
	0x96, 0x00, 0x97, 0x00, 0x98, 0x00, 0x99, 0x00,
	0x9a, 0x00, 0x9b, 0x00, 0x24, 0x08, 0x25, 0x00,
	0x26, 0x00, 0x27, 0x00, 0x28, 0x00, 0x29, 0x00,
	0x2a, 0x00, 0x2b, 0x00, 0x2c, 0x00, 0x2d, 0x00,
	0x2e, 0x00, 0x2f, 0x00, 0x30, 0x00, 0x31, 0x00,
	0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x35, 0x00,
	0x36, 0x00, 0x37, 0x00, 0x9c, 0x08, 0x9d, 0x00,
	0x9e, 0x00, 0x9f, 0x00, 0xa0, 0x00, 0xa1, 0x00,
	0xa2, 0x00, 0xa3, 0x00, 0xa4, 0x00, 0xa5, 0x00,
	0xa6, 0x00, 0xa7, 0x00, 0xa8, 0x00, 0xa9, 0x00,
	0xaa, 0x00, 0xab, 0x00, 0xac, 0x00, 0xad, 0x00,
	0xae, 0x00, 0xaf, 0x00, 0x00, 0x05, 0xe4, 0x08,
	0xe5, 0x00, 0xe6, 0x00, 0xe7, 0x00, 0xe8, 0x00,
	0xe9, 0x00, 0xea, 0x00, 0xeb, 0x00, 0xec, 0x00,
	0xed, 0x00, 0xee, 0x00, 0xef, 0x00, 0xf0, 0x00,
	0xf1, 0x00, 0xf2, 0x00, 0xf3, 0x00, 0xf4, 0x00,
	0xf5, 0x00, 0xf6, 0x00, 0xf7, 0x00, 0x00, 0x06,
	0x60, 0x08, 0x61, 0x00, 0x62, 0x00, 0x63, 0x00,
	0x64, 0x00, 0x65, 0x00, 0x66, 0x00, 0x67, 0x00,
	0x68, 0x00, 0x69, 0x00, 0x6a, 0x00, 0x6b, 0x00,
	0x6c, 0x00, 0x6d, 0x00, 0x6e, 0x00, 0x6f, 0x00,
	0x70, 0x00, 0x71, 0x00, 0x72, 0x00, 0x73, 0x00,
	0x00, 0x05, 0xf8, 0x08, 0xf9, 0x00, 0xfa, 0x00,
	0xfb, 0x00, 0xfc, 0x00, 0xfd, 0x00, 0xfe, 0x00,
	0xff, 0x00, 0x00, 0x06, 0x04, 0x00, 0x05, 0x00,
	0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00,
	0x0a, 0x00, 0x0b, 0x00, 0x0c, 0x00, 0x0d, 0x00,
	0x0e, 0x00, 0x0f, 0x00, 0x74, 0x08, 0x75, 0x00,
	0x76, 0x00, 0x77, 0x00, 0x78, 0x00, 0x79, 0x00,
	0x7a, 0x00, 0x7b, 0x00, 0x7c, 0x00, 0x7d, 0x00,
	0x7e, 0x00, 0x7f, 0x00, 0x80, 0x00, 0x81, 0x00,
	0x82, 0x00, 0x83, 0x00, 0x84, 0x00, 0x85, 0x00,
	0x86, 0x00, 0x87, 0x00, 0xd8, 0x00, 0xd9, 0x22,
	0xda, 0x1d, 0xdb, 0x95, 0xdc, 0x04, 0xdd, 0x0c,
	0xde, 0x37, 0xdf, 0x14, 0xe0, 0x1c, 0xe1, 0x1b,
	0xe2, 0xf0, 0xe3, 0x41, 0xe4, 0x00, 0xe5, 0x22,
	0xe6, 0x1d, 0xe7, 0x95, 0x00, 0x04, 0x04, 0x08,
	0x05, 0x13, 0x06, 0x85, 0x07, 0x62, 0x0c, 0x00,
	0x0d, 0x01, 0x0e, 0x5d, 0x0f, 0x86, 0x08, 0x00,
	0x09, 0x46, 0x0a, 0xff, 0x0b, 0x51, 0x10, 0x05,
	0x11, 0x39, 0x12, 0x47, 0x13, 0xa6, 0x14, 0x7a,
	0x15, 0xc6, 0x16, 0xb8, 0x17, 0x5a, 0x00, 0x00,
	0x04, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x11, 0xc3, 0x02, 0x00, 0x03, 0x05, 0x01, 0x84,
	0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x03
};

static const uint32_t acm8623_volume[] = {
    0x000001A8, /*   0, -110dB */ 0x000001DC, /*   1, -109dB */
    0x00000216, /*   2, -108dB */ 0x00000258, /*   3, -107dB */
    0x000002A1, /*   4, -106dB */ 0x000002F3, /*   5, -105dB */
    0x0000034F, /*   6, -104dB */ 0x000003B6, /*   7, -103dB */
    0x0000042A, /*   8, -102dB */ 0x000004AC, /*   9, -101dB */
    0x0000053E, /*  10, -100dB */ 0x000005E2, /*  11,  -99dB */
    0x0000069A, /*  12,  -98dB */ 0x00000768, /*  13,  -97dB */
    0x0000084F, /*  14,  -96dB */ 0x00000953, /*  15,  -95dB */
    0x00000A76, /*  16,  -94dB */ 0x00000BBD, /*  17,  -93dB */
    0x00000D2B, /*  18,  -92dB */ 0x00000EC7, /*  19,  -91dB */
    0x00001094, /*  20,  -90dB */ 0x0000129A, /*  21,  -89dB */
    0x000014DF, /*  22,  -88dB */ 0x0000176B, /*  23,  -87dB */
    0x00001A47, /*  24,  -86dB */ 0x00001D7C, /*  25,  -85dB */
    0x00002115, /*  26,  -84dB */ 0x0000251E, /*  27,  -83dB */
    0x000029A5, /*  28,  -82dB */ 0x00002EBA, /*  29,  -81dB */
    0x0000346E, /*  30,  -80dB */ 0x00003AD3, /*  31,  -79dB */
    0x00004201, /*  32,  -78dB */ 0x00004A0F, /*  33,  -77dB */
    0x00005318, /*  34,  -76dB */ 0x00005D3C, /*  35,  -75dB */
    0x0000689C, /*  36,  -74dB */ 0x00007560, /*  37,  -73dB */
    0x000083B2, /*  38,  -72dB */ 0x000093C4, /*  39,  -71dB */
    0x0000A5CB, /*  40,  -70dB */ 0x0000BA06, /*  41,  -69dB */
    0x0000D0B9, /*  42,  -68dB */ 0x0000EA31, /*  43,  -67dB */
    0x000106C4, /*  44,  -66dB */ 0x000126D4, /*  45,  -65dB */
    0x00014ACE, /*  46,  -64dB */ 0x0001732B, /*  47,  -63dB */
    0x0001A075, /*  48,  -62dB */ 0x0001D346, /*  49,  -61dB */
    0x00020C4A, /*  50,  -60dB */ 0x00024C43, /*  51,  -59dB */
    0x0002940A, /*  52,  -58dB */ 0x0002E494, /*  53,  -57dB */
    0x00033EF1, /*  54,  -56dB */ 0x0003A455, /*  55,  -55dB */
    0x00041618, /*  56,  -54dB */ 0x000495BC, /*  57,  -53dB */
    0x000524F4, /*  58,  -52dB */ 0x0005C5A5, /*  59,  -51dB */
    0x000679F2, /*  60,  -50dB */ 0x0007443E, /*  61,  -49dB */
    0x0008273A, /*  62,  -48dB */ 0x000925E9, /*  63,  -47dB */
    0x000A43AA, /*  64,  -46dB */ 0x000B844A, /*  65,  -45dB */
    0x000CEC09, /*  66,  -44dB */ 0x000E7FAD, /*  67,  -43dB */
    0x00104491, /*  68,  -42dB */ 0x001240B9, /*  69,  -41dB */
    0x00147AE1, /*  70,  -40dB */ 0x0016FA9C, /*  71,  -39dB */
    0x0019C865, /*  72,  -38dB */ 0x001CEDC4, /*  73,  -37dB */
    0x00207568, /*  74,  -36dB */ 0x00246B4E, /*  75,  -35dB */
    0x0028DCEC, /*  76,  -34dB */ 0x002DD959, /*  77,  -33dB */
    0x00337185, /*  78,  -32dB */ 0x0039B872, /*  79,  -31dB */
    0x0040C371, /*  80,  -30dB */ 0x0048AA71, /*  81,  -29dB */
    0x00518848, /*  82,  -28dB */ 0x005B7B16, /*  83,  -27dB */
    0x0066A4A5, /*  84,  -26dB */ 0x00732AE2, /*  85,  -25dB */
    0x00813856, /*  86,  -24dB */ 0x0090FCBF, /*  87,  -23dB */
    0x00A2ADAD, /*  88,  -22dB */ 0x00B68738, /*  89,  -21dB */
    0x00CCCCCD, /*  90,  -20dB */ 0x00E5CA15, /*  91,  -19dB */
    0x0101D3F3, /*  92,  -18dB */ 0x012149A6, /*  93,  -17dB */
    0x0144960C, /*  94,  -16dB */ 0x016C310E, /*  95,  -15dB */
    0x0198A135, /*  96,  -14dB */ 0x01CA7D76, /*  97,  -13dB */
    0x02026F31, /*  98,  -12dB */ 0x0241346F, /*  99,  -11dB */
    0x0287A26C, /* 100,  -10dB */ 0x02D6A867, /* 101,   -9dB */
    0x032F52D0, /* 102,   -8dB */ 0x0392CED9, /* 103,   -7dB */
    0x04026E74, /* 104,   -6dB */ 0x047FACCF, /* 105,   -5dB */
    0x050C335D, /* 106,   -4dB */ 0x05A9DF7B, /* 107,   -3dB */
    0x065AC8C3, /* 108,   -2dB */ 0x0721482C, /* 109,   -1dB */
    0x08000000, /* 110,    0dB */ 0x08F9E4D0, /* 111,    1dB */
    0x0A12477C, /* 112,    2dB */ 0x0B4CE07C, /* 113,    3dB */
    0x0CADDC7B, /* 114,    4dB */ 0x0E39EA8E, /* 115,    5dB */
    0x0FF64C17, /* 116,    6dB */ 0x11E8E6A1, /* 117,    7dB */
    0x141857EA, /* 118,    8dB */ 0x168C0C5A, /* 119,    9dB */
    0x194C583B, /* 120,   10dB */ 0x1C629406, /* 121,   11dB */
    0x1FD93C1F, /* 122,   12dB */ 0x23BC1479, /* 123,   13dB */
    0x28185086, /* 124,   14dB */ 0x2CFCC016, /* 125,   15dB */
    0x327A01A4, /* 126,   16dB */ 0x38A2BACB, /* 127,   17dB */
    0x3F8BD79E, /* 128,   18dB */ 0x474CD1B8, /* 129,   19dB */
    0x50000000, /* 130,   20dB */ 0x59C2F01D, /* 131,   21dB */
    0x64B6CADD, /* 132,   22dB */ 0x7100C4D8, /* 133,   23dB */
    0x7ECA9CD2, /* 134,   24dB */
};

#define ACM8623_VOLUME_MAX	((int)ARRAY_SIZE(acm8623_volume) - 1)
#define ACM8623_VOLUME_MIN	0

#define ACM8623_VOLUME_0DB	110

struct acm8623_priv {
	struct i2c_client		*i2c;

	uint8_t					*dsp_cfg_data;
	int		 				dsp_cfg_len;

	struct regmap			*regmap;

	int						vol[2];
	bool					is_powered;
	bool					is_muted;

	struct work_struct		work;
	struct mutex			lock;
};

static void set_dsp_scale(struct regmap *rm, int offset, int vol)
{
	uint8_t v[4];
	uint32_t x = acm8623_volume[vol];
	int i;

	for (i = 0; i < 4; i++) {
		v[3 - i] = x;
		x >>= 8;
	}

	regmap_bulk_write(rm, offset, v, ARRAY_SIZE(v));
}

static void acm8623_refresh(struct acm8623_priv *acm8623)
{
	struct regmap *rm = acm8623->regmap;

	dev_dbg(&acm8623->i2c->dev, "refresh: is_muted=%d, vol=%d/%d\n",
		acm8623->is_muted, acm8623->vol[0], acm8623->vol[1]);

	regmap_write(rm, REG_PAGE, 0x05);

	set_dsp_scale(rm, 0xc4, acm8623->vol[0]);
	set_dsp_scale(rm, 0xc0, acm8623->vol[1]);

	regmap_write(rm, REG_PAGE, 0x00);

	/* Set/clear digital soft-mute */
	regmap_write(rm, REG_DEVICE_STATE,
		(acm8623->is_muted ? DEVICE_STATE_MUTE : 0) |
		DEVICE_STATE_PLAY);
}

static int acm8623_vol_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;

	uinfo->value.integer.min = ACM8623_VOLUME_MIN;
	uinfo->value.integer.max = ACM8623_VOLUME_MAX;
	return 0;
}

static int acm8623_vol_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct acm8623_priv *acm8623 =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&acm8623->lock);
	ucontrol->value.integer.value[0] = acm8623->vol[0];
	ucontrol->value.integer.value[1] = acm8623->vol[1];
	mutex_unlock(&acm8623->lock);

	return 0;
}

static inline int volume_is_valid(int v)
{
	return (v >= ACM8623_VOLUME_MIN) && (v <= ACM8623_VOLUME_MAX);
}

static int acm8623_vol_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct acm8623_priv *acm8623 =
		snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (!(volume_is_valid(ucontrol->value.integer.value[0]) &&
	      volume_is_valid(ucontrol->value.integer.value[1])))
		return -EINVAL;

	mutex_lock(&acm8623->lock);
	if (acm8623->vol[0] != ucontrol->value.integer.value[0] ||
	    acm8623->vol[1] != ucontrol->value.integer.value[1]) {
		acm8623->vol[0] = ucontrol->value.integer.value[0];
		acm8623->vol[1] = ucontrol->value.integer.value[1];
		dev_dbg(component->dev, "set vol=%d/%d (is_powered=%d)\n",
			acm8623->vol[0], acm8623->vol[1],
			acm8623->is_powered);
		if (acm8623->is_powered)
			acm8623_refresh(acm8623);
		ret = 1;
	}
	mutex_unlock(&acm8623->lock);

	return ret;
}

static const struct snd_kcontrol_new acm8623_snd_controls[] = {
	{
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Master Playback Volume",
		.access	= SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= acm8623_vol_info,
		.get	= acm8623_vol_get,
		.put	= acm8623_vol_put,
	},
};

static void send_cfg(struct regmap *rm,
		     const uint8_t *s, unsigned int len)
{
	unsigned int i;

	for (i = 0; i + 1 < len; i += 2) {
		regmap_write(rm, s[i], s[i + 1]);
	}
}

static int acm8623_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct acm8623_priv *acm8623 =
		snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(component->dev, "clock start\n");
		schedule_work(&acm8623->work);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void do_work(struct work_struct *work)
{
	struct acm8623_priv *acm8623 =
	       container_of(work, struct acm8623_priv, work);
	struct regmap *rm = acm8623->regmap;

	dev_dbg(&acm8623->i2c->dev, "DSP startup\n");

	mutex_lock(&acm8623->lock);
	/* We mustn't issue any I2C transactions until the I2S
	 * clock is stable. Furthermore, we must allow a 5ms
	 * delay after the first set of register writes to
	 * allow the DSP to boot before configuring it.
	 */
	usleep_range(5000, 10000);
	send_cfg(rm, dsp_cfg_preboot, ARRAY_SIZE(dsp_cfg_preboot));
	usleep_range(5000, 15000);
	if (acm8623->dsp_cfg_data)
		send_cfg(rm, acm8623->dsp_cfg_data, acm8623->dsp_cfg_len);
	else
		send_cfg(rm, dsp_cfg_default, ARRAY_SIZE(dsp_cfg_default));

	acm8623->is_powered = true;
	acm8623_refresh(acm8623);
	mutex_unlock(&acm8623->lock);
}

static int acm8623_dac_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct acm8623_priv *acm8623 =
		snd_soc_component_get_drvdata(component);
	struct regmap *rm = acm8623->regmap;

	if (event & SND_SOC_DAPM_PRE_PMD) {
		unsigned int channel_state, global1, global2, global3;

		dev_dbg(component->dev, "DSP shutdown\n");
		cancel_work_sync(&acm8623->work);

		mutex_lock(&acm8623->lock);
		if (acm8623->is_powered) {
			acm8623->is_powered = false;

			regmap_write(rm, REG_PAGE, 0x00);

			regmap_read(rm, REG_STATE_REPORT, &channel_state);
			regmap_read(rm, REG_GLOBAL_FAULT1, &global1);
			regmap_read(rm, REG_GLOBAL_FAULT2, &global2);
			regmap_read(rm, REG_GLOBAL_FAULT3, &global3);

			dev_dbg(component->dev, "fault regs: CHANNEL=%02x, "
				"GLOBAL1=%02x, GLOBAL2=%02x, GLOBAL3=%02x\n",
				channel_state, global1, global2, global3);

			regmap_write(rm, REG_DEVICE_STATE, DEVICE_STATE_HIZ);
		}
		mutex_unlock(&acm8623->lock);
	}

	return 0;
}

static const struct snd_soc_dapm_route acm8623_audio_map[] = {
	{ "DAC", NULL, "DAC IN" },
	{ "OUT", NULL, "DAC" },
};

static const struct snd_soc_dapm_widget acm8623_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DAC IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0,
		acm8623_dac_event, SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_component_driver soc_codec_dev_acm8623 = {
	.controls			= acm8623_snd_controls,
	.num_controls		= ARRAY_SIZE(acm8623_snd_controls),
	.dapm_widgets		= acm8623_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(acm8623_dapm_widgets),
	.dapm_routes		= acm8623_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(acm8623_audio_map),
	.use_pmdown_time	= 1,
	.endianness			= 1,
};

static int acm8623_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct acm8623_priv *acm8623 =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&acm8623->lock);
	dev_dbg(component->dev, "set mute=%d (is_powered=%d)\n",
		mute, acm8623->is_powered);

	acm8623->is_muted = mute;
	if (acm8623->is_powered)
		acm8623_refresh(acm8623);
	mutex_unlock(&acm8623->lock);

	return 0;
}

static const struct snd_soc_dai_ops acm8623_dai_ops = {
	.trigger			= acm8623_trigger,
	.mute_stream		= acm8623_mute,
	.no_capture_mute	= 1,
};

static struct snd_soc_dai_driver acm8623_dai = {
	.name		= "acm8623-hifi",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates			= SNDRV_PCM_RATE_48000,
		.formats		= SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops		= &acm8623_dai_ops,
};

static const struct regmap_config acm8623_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,

	/* We have quite a lot of multi-level bank switching and a
	 * relatively small number of register writes between bank
	 * switches.
	 */
	.cache_type	= REGCACHE_NONE,
};

static int acm8623_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;
	struct acm8623_priv *acm8623;

	char filename[128];
	const char *config_name;
	const struct firmware *fw;
	int ret;

	dev_info(dev, "acm8623_i2c_probe(): Start I2C Probe\n");

	regmap = devm_regmap_init_i2c(i2c, &acm8623_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "unable to allocate register map: %d\n", ret);
		return ret;
	}

	acm8623 = devm_kzalloc(dev, sizeof(struct acm8623_priv), GFP_KERNEL);
	if (!acm8623)
		return -ENOMEM;

	acm8623->i2c = i2c;

	dev_set_drvdata(dev, acm8623);
	acm8623->regmap = regmap;
	
	if (device_property_read_string(dev, "acme,dsp-config-name",
					&config_name))
		config_name = "default";

	snprintf(filename, sizeof(filename), "acm8623_dsp_%s.bin",
		 config_name);
	ret = request_firmware(&fw, filename, dev);
	if (!ret) {
		if ((fw->size < 2) || (fw->size & 1)) {
			dev_err(dev, "firmware is invalid\n");
			release_firmware(fw);
			return -EINVAL;
		}

		acm8623->dsp_cfg_len = fw->size;
		acm8623->dsp_cfg_data = devm_kmalloc(dev, fw->size, GFP_KERNEL);
		if (!acm8623->dsp_cfg_data) {
			release_firmware(fw);
			return -ENOMEM;
		}
		memcpy(acm8623->dsp_cfg_data, fw->data, fw->size);

		release_firmware(fw);
	} else {
		acm8623->dsp_cfg_len = 0;
		acm8623->dsp_cfg_data = NULL;
	}

	acm8623->vol[0] = ACM8623_VOLUME_0DB;
	acm8623->vol[1] = ACM8623_VOLUME_0DB;

	usleep_range(100000, 150000);

	INIT_WORK(&acm8623->work, do_work);
	mutex_init(&acm8623->lock);

	/* Don't register through devm. We need to be able to unregister
	 * the component prior to deasserting PDN#
	 */
	ret = snd_soc_register_component(dev, &soc_codec_dev_acm8623,
					 &acm8623_dai, 1);
	if (ret < 0) {
		dev_err(dev, "unable to register codec: %d\n", ret);
		return ret;
	}

	return 0;
}

static void acm8623_i2c_remove(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct acm8623_priv *acm8623 = dev_get_drvdata(dev);

	cancel_work_sync(&acm8623->work);
	snd_soc_unregister_component(dev);
	usleep_range(10000, 15000);
}

static const struct i2c_device_id acm8623_i2c_id[] = {
	{ "acm8623", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, acm8623_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id acme8625s_of_match[] = {
	{ .compatible = "acme,acm8623", },
	{ }
};
MODULE_DEVICE_TABLE(of, acme8625s_of_match);
#endif

static struct i2c_driver acm8623_i2c_driver = {
	.probe_new	= acm8623_i2c_probe,
	.remove		= acm8623_i2c_remove,
	.id_table	= acm8623_i2c_id,
	.driver		= {
		.name		= "acm8623",
		.of_match_table = of_match_ptr(acme8625s_of_match),
	},
};

module_i2c_driver(acm8623_i2c_driver);

MODULE_AUTHOR("Wenhao Yang <wenhaoy@acme-semi.com>");
MODULE_DESCRIPTION("ACM8623 Audio Amplifier Driver");
MODULE_LICENSE("GPL v2");
