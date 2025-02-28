// SPDX-License-Identifier: GPL-2.0
//
// Driver for the ACM8635 Audio Amplifier
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
	0x00, 0x00, 0x04, 0x00, 0xfc, 0x86, 0xfd, 0x25,
	0xfe, 0x53, 0x00, 0x01, 0x02, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t dsp_cfg_default[] = {
	0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
	0xe4, 0x80, 0xe5, 0x9e, 0xe6, 0x02, 0xe7, 0x9e,
	0xe8, 0x80, 0xe9, 0x9e, 0xea, 0x03, 0xeb, 0x9e,
	0x00, 0x04, 0x94, 0x00, 0x95, 0xe2, 0x96, 0xc4,
	0x97, 0x6b, 0x28, 0x00, 0x29, 0x40, 0x2a, 0x26,
	0x2b, 0xe7, 0x2c, 0x00, 0x2d, 0x40, 0x2e, 0x26,
	0x2f, 0xe7, 0x00, 0x0c, 0x60, 0x00, 0x61, 0x1b,
	0x62, 0x4b, 0x63, 0x98, 0x64, 0x00, 0x65, 0x22,
	0x66, 0x1d, 0x67, 0x95, 0x68, 0x00, 0x69, 0x06,
	0x6a, 0xd3, 0x6b, 0x72, 0x6c, 0x00, 0x6d, 0x00,
	0x6e, 0x00, 0x6f, 0x00, 0x70, 0x00, 0x71, 0x00,
	0x72, 0x00, 0x73, 0x00, 0x74, 0xff, 0x75, 0x81,
	0x76, 0x47, 0x77, 0xae, 0x78, 0xf5, 0x79, 0xb3,
	0x7a, 0xb7, 0x7b, 0xc8, 0x7c, 0xfe, 0x7d, 0x01,
	0x7e, 0xc0, 0x7f, 0x79, 0x80, 0x00, 0x81, 0x00,
	0x82, 0x00, 0x83, 0x00, 0x84, 0x00, 0x85, 0x00,
	0x86, 0x00, 0x87, 0x00, 0x00, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x11, 0x03, 0x02, 0x00, 0x06, 0xb0,
	0x05, 0xf0, 0x28, 0x03, 0x03, 0x05, 0x01, 0x84,
	0x00, 0x01, 0x09, 0x04, 0x00, 0x00, 0x04, 0x02,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x04, 0x03
 };

static  const uint32_t acm8635_volume[] = {
	0x0000001B, /*   0, -110dB */ 0x0000001E, /*   1, -109dB */
	0x00000021, /*   2, -108dB */ 0x00000025, /*   3, -107dB */
	0x0000002A, /*   4, -106dB */ 0x0000002F, /*   5, -105dB */
	0x00000035, /*   6, -104dB */ 0x0000003B, /*   7, -103dB */
	0x00000043, /*   8, -102dB */ 0x0000004B, /*   9, -101dB */
	0x00000054, /*  10, -100dB */ 0x0000005E, /*  11,  -99dB */
	0x0000006A, /*  12,  -98dB */ 0x00000076, /*  13,  -97dB */
	0x00000085, /*  14,  -96dB */ 0x00000095, /*  15,  -95dB */
	0x000000A7, /*  16,  -94dB */ 0x000000BC, /*  17,  -93dB */
	0x000000D3, /*  18,  -92dB */ 0x000000EC, /*  19,  -91dB */
	0x00000109, /*  20,  -90dB */ 0x0000012A, /*  21,  -89dB */
	0x0000014E, /*  22,  -88dB */ 0x00000177, /*  23,  -87dB */
	0x000001A4, /*  24,  -86dB */ 0x000001D8, /*  25,  -85dB */
	0x00000211, /*  26,  -84dB */ 0x00000252, /*  27,  -83dB */
	0x0000029A, /*  28,  -82dB */ 0x000002EC, /*  29,  -81dB */
	0x00000347, /*  30,  -80dB */ 0x000003AD, /*  31,  -79dB */
	0x00000420, /*  32,  -78dB */ 0x000004A1, /*  33,  -77dB */
	0x00000532, /*  34,  -76dB */ 0x000005D4, /*  35,  -75dB */
	0x0000068A, /*  36,  -74dB */ 0x00000756, /*  37,  -73dB */
	0x0000083B, /*  38,  -72dB */ 0x0000093C, /*  39,  -71dB */
	0x00000A5D, /*  40,  -70dB */ 0x00000BA0, /*  41,  -69dB */
	0x00000D0C, /*  42,  -68dB */ 0x00000EA3, /*  43,  -67dB */
	0x0000106C, /*  44,  -66dB */ 0x0000126D, /*  45,  -65dB */
	0x000014AD, /*  46,  -64dB */ 0x00001733, /*  47,  -63dB */
	0x00001A07, /*  48,  -62dB */ 0x00001D34, /*  49,  -61dB */
	0x000020C5, /*  50,  -60dB */ 0x000024C4, /*  51,  -59dB */
	0x00002941, /*  52,  -58dB */ 0x00002E49, /*  53,  -57dB */
	0x000033EF, /*  54,  -56dB */ 0x00003A45, /*  55,  -55dB */
	0x00004161, /*  56,  -54dB */ 0x0000495C, /*  57,  -53dB */
	0x0000524F, /*  58,  -52dB */ 0x00005C5A, /*  59,  -51dB */
	0x0000679F, /*  60,  -50dB */ 0x00007444, /*  61,  -49dB */
	0x00008274, /*  62,  -48dB */ 0x0000925F, /*  63,  -47dB */
	0x0000A43B, /*  64,  -46dB */ 0x0000B845, /*  65,  -45dB */
	0x0000CEC1, /*  66,  -44dB */ 0x0000E7FB, /*  67,  -43dB */
	0x00010449, /*  68,  -42dB */ 0x0001240C, /*  69,  -41dB */
	0x000147AE, /*  70,  -40dB */ 0x00016FAA, /*  71,  -39dB */
	0x00019C86, /*  72,  -38dB */ 0x0001CEDC, /*  73,  -37dB */
	0x00020756, /*  74,  -36dB */ 0x000246B5, /*  75,  -35dB */
	0x00028DCF, /*  76,  -34dB */ 0x0002DD96, /*  77,  -33dB */
	0x00033718, /*  78,  -32dB */ 0x00039B87, /*  79,  -31dB */
	0x00040C37, /*  80,  -30dB */ 0x00048AA7, /*  81,  -29dB */
	0x00051884, /*  82,  -28dB */ 0x0005B7B1, /*  83,  -27dB */
	0x00066A4A, /*  84,  -26dB */ 0x000732AE, /*  85,  -25dB */
	0x00081385, /*  86,  -24dB */ 0x00090FCC, /*  87,  -23dB */
	0x000A2ADB, /*  88,  -22dB */ 0x000B6873, /*  89,  -21dB */
	0x000CCCCD, /*  90,  -20dB */ 0x000E5CA1, /*  91,  -19dB */
	0x00101D3F, /*  92,  -18dB */ 0x0012149A, /*  93,  -17dB */
	0x00144961, /*  94,  -16dB */ 0x0016C311, /*  95,  -15dB */
	0x00198A13, /*  96,  -14dB */ 0x001CA7D7, /*  97,  -13dB */
	0x002026F3, /*  98,  -12dB */ 0x00241347, /*  99,  -11dB */
	0x00287A27, /* 100,  -10dB */ 0x002D6A86, /* 101,   -9dB */
	0x0032F52D, /* 102,   -8dB */ 0x00392CEE, /* 103,   -7dB */
	0x004026E7, /* 104,   -6dB */ 0x0047FACD, /* 105,   -5dB */
	0x0050C336, /* 106,   -4dB */ 0x005A9DF8, /* 107,   -3dB */
	0x0065AC8C, /* 108,   -2dB */ 0x00721483, /* 109,   -1dB */
	0x00800000, /* 110,    0dB */ 0x008F9E4D, /* 111,    1dB */
	0x00A12478, /* 112,    2dB */ 0x00B4CE08, /* 113,    3dB */
	0x00CADDC8, /* 114,    4dB */ 0x00E39EA9, /* 115,    5dB */
	0x00FF64C1, /* 116,    6dB */ 0x011E8E6A, /* 117,    7dB */
	0x0141857F, /* 118,    8dB */ 0x0168C0C6, /* 119,    9dB */
	0x0194C584, /* 120,   10dB */ 0x01C62940, /* 121,   11dB */
	0x01FD93C2, /* 122,   12dB */ 0x023BC148, /* 123,   13dB */
	0x02818508, /* 124,   14dB */ 0x02CFCC01, /* 125,   15dB */
	0x0327A01A, /* 126,   16dB */ 0x038A2BAD, /* 127,   17dB */
	0x03F8BD7A, /* 128,   18dB */ 0x0474CD1B, /* 129,   19dB */
	0x05000000, /* 130,   20dB */ 0x059C2F02, /* 131,   21dB */
	0x064B6CAE, /* 132,   22dB */ 0x07100C4D, /* 133,   23dB */
	0x07ECA9CD, /* 134,   24dB */ 0x08E43299, /* 135,   25dB */
	0x09F9EF8E, /* 136,   26dB */ 0x0B319025, /* 137,   27dB */
	0x0C8F36F2, /* 138,   28dB */ 0x0E1787B8, /* 139,   29dB */
	0x0FCFB725, /* 140,   30dB */ 0x11BD9C84, /* 141,   31dB */
	0x13E7C594, /* 142,   32dB */ 0x16558CCB, /* 143,   33dB */
	0x190F3254, /* 144,   34dB */ 0x1C1DF80E, /* 145,   35dB */
	0x1F8C4107, /* 146,   36dB */ 0x2365B4BF, /* 147,   37dB */
	0x27B766C2, /* 148,   38dB */ 0x2C900313, /* 149,   39dB */
	0x32000000, /* 150,   40dB */ 0x3819D612, /* 151,   41dB */
	0x3EF23ECA, /* 152,   42dB */ 0x46A07B07, /* 153,   43dB */
	0x4F3EA203, /* 154,   44dB */ 0x58E9F9F9, /* 155,   45dB */
	0x63C35B8E, /* 156,   46dB */ 0x6FEFA16D, /* 157,   47dB */
	0x7D982575, /* 158,   48dB */
};

#define ACM8635_VOLUME_MAX	((int)ARRAY_SIZE(acm8635_volume) - 1)
#define ACM8635_VOLUME_MIN	0

#define ACM8635_VOLUME_0DB	110

struct acm8635_priv {
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
	uint32_t x = acm8635_volume[vol];
	int i;

	for (i = 0; i < 4; i++) {
		v[3 - i] = x;
		x >>= 8;
	}

	regmap_bulk_write(rm, offset, v, ARRAY_SIZE(v));
}

static void acm8635_refresh(struct acm8635_priv *acm8635)
{
	struct regmap *rm = acm8635->regmap;

	dev_dbg(&acm8635->i2c->dev, "refresh: is_muted=%d, vol=%d/%d\n",
		acm8635->is_muted, acm8635->vol[0], acm8635->vol[1]);

	regmap_write(rm, REG_PAGE, 0x04);

	set_dsp_scale(rm, 0x7c, acm8635->vol[0]);
	set_dsp_scale(rm, 0x80, acm8635->vol[1]);

	regmap_write(rm, REG_PAGE, 0x00);

	/* Set/clear digital soft-mute */
	regmap_write(rm, REG_DEVICE_STATE,
		(acm8635->is_muted ? DEVICE_STATE_MUTE : 0) |
		DEVICE_STATE_PLAY);
}

static int acm8635_vol_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;

	uinfo->value.integer.min = ACM8635_VOLUME_MIN;
	uinfo->value.integer.max = ACM8635_VOLUME_MAX;
	return 0;
}

static int acm8635_vol_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct acm8635_priv *acm8635 =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&acm8635->lock);
	ucontrol->value.integer.value[0] = acm8635->vol[0];
	ucontrol->value.integer.value[1] = acm8635->vol[1];
	mutex_unlock(&acm8635->lock);

	return 0;
}

static inline int volume_is_valid(int v)
{
	return (v >= ACM8635_VOLUME_MIN) && (v <= ACM8635_VOLUME_MAX);
}

static int acm8635_vol_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct acm8635_priv *acm8635 =
		snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (!(volume_is_valid(ucontrol->value.integer.value[0]) &&
	      volume_is_valid(ucontrol->value.integer.value[1])))
		return -EINVAL;

	mutex_lock(&acm8635->lock);
	if (acm8635->vol[0] != ucontrol->value.integer.value[0] ||
	    acm8635->vol[1] != ucontrol->value.integer.value[1]) {
		acm8635->vol[0] = ucontrol->value.integer.value[0];
		acm8635->vol[1] = ucontrol->value.integer.value[1];
		dev_dbg(component->dev, "set vol=%d/%d (is_powered=%d)\n",
			acm8635->vol[0], acm8635->vol[1],
			acm8635->is_powered);
		if (acm8635->is_powered)
			acm8635_refresh(acm8635);
		ret = 1;
	}
	mutex_unlock(&acm8635->lock);

	return ret;
}

static const struct snd_kcontrol_new acm8635_snd_controls[] = {
	{
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Master Playback Volume",
		.access	= SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info	= acm8635_vol_info,
		.get	= acm8635_vol_get,
		.put	= acm8635_vol_put,
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

static int acm8635_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct acm8635_priv *acm8635 =
		snd_soc_component_get_drvdata(component);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_dbg(component->dev, "clock start\n");
		schedule_work(&acm8635->work);
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
	struct acm8635_priv *acm8635 =
	       container_of(work, struct acm8635_priv, work);
	struct regmap *rm = acm8635->regmap;

	dev_dbg(&acm8635->i2c->dev, "DSP startup\n");

	mutex_lock(&acm8635->lock);
	/* We mustn't issue any I2C transactions until the I2S
	 * clock is stable. Furthermore, we must allow a 5ms
	 * delay after the first set of register writes to
	 * allow the DSP to boot before configuring it.
	 */
	usleep_range(5000, 10000);
	send_cfg(rm, dsp_cfg_preboot, ARRAY_SIZE(dsp_cfg_preboot));
	usleep_range(5000, 15000);
	if (acm8635->dsp_cfg_data)
		send_cfg(rm, acm8635->dsp_cfg_data, acm8635->dsp_cfg_len);
	else
		send_cfg(rm, dsp_cfg_default, ARRAY_SIZE(dsp_cfg_default));

	acm8635->is_powered = true;
	acm8635_refresh(acm8635);
	mutex_unlock(&acm8635->lock);
}

static int acm8635_dac_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct acm8635_priv *acm8635 =
		snd_soc_component_get_drvdata(component);
	struct regmap *rm = acm8635->regmap;

	if (event & SND_SOC_DAPM_PRE_PMD) {
		unsigned int channel_state, global1, global2, global3;

		dev_dbg(component->dev, "DSP shutdown\n");
		cancel_work_sync(&acm8635->work);

		mutex_lock(&acm8635->lock);
		if (acm8635->is_powered) {
			acm8635->is_powered = false;

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
		mutex_unlock(&acm8635->lock);
	}

	return 0;
}

static const struct snd_soc_dapm_route acm8635_audio_map[] = {
	{ "DAC", NULL, "DAC IN" },
	{ "OUT", NULL, "DAC" },
};

static const struct snd_soc_dapm_widget acm8635_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DAC IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0,
		acm8635_dac_event, SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_component_driver soc_codec_dev_acm8635 = {
	.controls			= acm8635_snd_controls,
	.num_controls		= ARRAY_SIZE(acm8635_snd_controls),
	.dapm_widgets		= acm8635_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(acm8635_dapm_widgets),
	.dapm_routes		= acm8635_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(acm8635_audio_map),
	.use_pmdown_time	= 1,
	.endianness			= 1,
};

static int acm8635_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct acm8635_priv *acm8635 =
		snd_soc_component_get_drvdata(component);

	mutex_lock(&acm8635->lock);
	dev_dbg(component->dev, "set mute=%d (is_powered=%d)\n",
		mute, acm8635->is_powered);

	acm8635->is_muted = mute;
	if (acm8635->is_powered)
		acm8635_refresh(acm8635);
	mutex_unlock(&acm8635->lock);

	return 0;
}

static const struct snd_soc_dai_ops acm8635_dai_ops = {
	.trigger			= acm8635_trigger,
	.mute_stream		= acm8635_mute,
	.no_capture_mute	= 1,
};

static struct snd_soc_dai_driver acm8635_dai = {
	.name		= "acm8635-hifi",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates			= SNDRV_PCM_RATE_48000,
		.formats		= SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops		= &acm8635_dai_ops,
};

static const struct regmap_config acm8635_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,

	/* We have quite a lot of multi-level bank switching and a
	 * relatively small number of register writes between bank
	 * switches.
	 */
	.cache_type	= REGCACHE_NONE,
};

static int acm8635_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;
	struct acm8635_priv *acm8635;

	char filename[128];
	const char *config_name;
	const struct firmware *fw;
	int ret;

	dev_info(dev, "acm8635_i2c_probe(): Start I2C Probe\n");

	regmap = devm_regmap_init_i2c(i2c, &acm8635_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "unable to allocate register map: %d\n", ret);
		return ret;
	}

	acm8635 = devm_kzalloc(dev, sizeof(struct acm8635_priv), GFP_KERNEL);
	if (!acm8635)
		return -ENOMEM;

	acm8635->i2c = i2c;

	dev_set_drvdata(dev, acm8635);
	acm8635->regmap = regmap;
	
	if (device_property_read_string(dev, "acme,dsp-config-name",
					&config_name))
		config_name = "default";

	snprintf(filename, sizeof(filename), "acm8635_dsp_%s.bin",
		 config_name);
	ret = request_firmware(&fw, filename, dev);
	if (!ret) {
		if ((fw->size < 2) || (fw->size & 1)) {
			dev_err(dev, "firmware is invalid\n");
			release_firmware(fw);
			return -EINVAL;
		}

		acm8635->dsp_cfg_len = fw->size;
		acm8635->dsp_cfg_data = devm_kmalloc(dev, fw->size, GFP_KERNEL);
		if (!acm8635->dsp_cfg_data) {
			release_firmware(fw);
			return -ENOMEM;
		}
		memcpy(acm8635->dsp_cfg_data, fw->data, fw->size);

		release_firmware(fw);
	} else {
		acm8635->dsp_cfg_len = 0;
		acm8635->dsp_cfg_data = NULL;
	}

	acm8635->vol[0] = ACM8635_VOLUME_0DB;
	acm8635->vol[1] = ACM8635_VOLUME_0DB;

	usleep_range(100000, 150000);

	INIT_WORK(&acm8635->work, do_work);
	mutex_init(&acm8635->lock);

	/* Don't register through devm. We need to be able to unregister
	 * the component prior to deasserting PDN#
	 */
	ret = snd_soc_register_component(dev, &soc_codec_dev_acm8635,
					 &acm8635_dai, 1);
	if (ret < 0) {
		dev_err(dev, "unable to register codec: %d\n", ret);
		return ret;
	}

	return 0;
}

static void acm8635_i2c_remove(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct acm8635_priv *acm8635 = dev_get_drvdata(dev);

	cancel_work_sync(&acm8635->work);
	snd_soc_unregister_component(dev);
	usleep_range(10000, 15000);
}

static const struct i2c_device_id acm8635_i2c_id[] = {
	{ "acm8635", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, acm8635_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id acme8635_of_match[] = {
	{ .compatible = "acme,acm8635", },
	{ }
};
MODULE_DEVICE_TABLE(of, acme8635_of_match);
#endif

static struct i2c_driver acm8635_i2c_driver = {
	.probe_new	= acm8635_i2c_probe,
	.remove		= acm8635_i2c_remove,
	.id_table	= acm8635_i2c_id,
	.driver		= {
		.name		= "acm8635",
		.of_match_table = of_match_ptr(acme8635_of_match),
	},
};

module_i2c_driver(acm8635_i2c_driver);

MODULE_AUTHOR("Wenhao Yang <wenhaoy@acme-semi.com>");
MODULE_DESCRIPTION("ACM8635 Audio Amplifier Driver");
MODULE_LICENSE("GPL v2");
