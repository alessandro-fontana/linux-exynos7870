// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC machine driver for the Samsung Galaxy Tab A 10.1 (2016),
 * "gtaxlwifi": Exynos 7870 I2S0 -> s1402x audio mixer -> RT5659.
 *
 * The codec hangs off the mixer's AMP port, so both the mixer's AP0
 * DAI and the codec's AIF1 sit as codecs on the same DAI link.  The
 * clocking recipe follows the vendor universal7870-rt5659.c:
 *
 *   48 kHz:  RCLK = 512fs (24.576 MHz), BCLK = 32fs
 *   192 kHz: RCLK = 128fs (49.152 MHz), BCLK = 64fs
 *
 * with the CPU as clock provider.
 *
 * Copyright (C) 2026 The LineageOS Project
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "i2s.h"
#include "../codecs/rt5659.h"

/*
 * ASRC tracking sources.  rt5659.h defines the fields of ASRC_2 and
 * ASRC_3 but not their values, since mainline never programs them.
 * 1 = track I2S1.
 */
#define RT5659_DA_STO_T_I2S1		(0x1 << RT5659_DA_STO_T_SFT)
#define RT5659_AD_STO1_T_I2S1		(0x1 << RT5659_AD_STO1_T_SFT)
#define RT5659_AD_MONO_L_T_I2S1		(0x1 << RT5659_AD_MONO_L_T_SFT)
#define RT5659_AD_MONO_R_T_I2S1		(0x1 << RT5659_AD_MONO_R_T_SFT)

#define GTAXL_RFS_48KHZ		512
#define GTAXL_BFS_48KHZ		32
#define GTAXL_PLL_48KHZ		24576000U
#define GTAXL_RFS_192KHZ	128
#define GTAXL_BFS_192KHZ	64
#define GTAXL_PLL_192KHZ	49152000U

/*
 * The codec does not sit on the AP-side bus.  It hangs off the mixer's
 * AMP port, which the s1402x probe fixes at 64fs / 24 bit
 * (INAMP_CTL = 0x2a) and never reprograms at runtime, neither here nor
 * in the vendor driver.  The BCLK arriving at the codec is therefore
 * always 64fs, independent of the AP -> mixer ratio above.
 */
#define GTAXL_BFS_AMP		64

/*
 * The codec MCLK pin is wired to the SoC PMU CLKOUT pad.  The vendor
 * enables it as the "oscclk_aud" clock: PMU_DEBUG CLKOUT_SEL[12:8] all
 * set (0x1f selects the 26 MHz OSCCLK) and CLKOUT_DISABLE (bit 0)
 * cleared.
 */
#define GTAXL_MCLK_FREQ		26000000U
#define GTAXL_PMU_DEBUG		0x10480a00
#define GTAXL_PMU_CLKOUT_MASK	0x1f01
#define GTAXL_PMU_CLKOUT_ON	0x1f00

static int gtaxl_aif1_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *amixer_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 1);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int rate = params_rate(params);
	unsigned int rfs, bfs, codec_pll, pll_in;
	int pll_src;
	int ret;

	switch (rate) {
	case 48000:
		rfs = GTAXL_RFS_48KHZ;
		bfs = GTAXL_BFS_48KHZ;
		codec_pll = GTAXL_PLL_48KHZ;
		break;
	case 192000:
		rfs = GTAXL_RFS_192KHZ;
		bfs = GTAXL_BFS_192KHZ;
		codec_pll = GTAXL_PLL_192KHZ;
		break;
	default:
		dev_err(rtd->card->dev, "unsupported rate %u\n", rate);
		return -EINVAL;
	}

	/* CPU: CDCLK out at rfs, root clock from CMU, BCLK divider */
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
				     rfs, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->card->dev, "failed to set CDCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
				     0, SAMSUNG_I2S_OPCLK_CDCLK_OUT);
	if (ret < 0) {
		dev_err(rtd->card->dev, "failed to set OPCLK: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	if (ret < 0) {
		dev_err(rtd->card->dev, "failed to set BCLK div: %d\n", ret);
		return ret;
	}

	/* Mixer: XFS follows the bus BCLK ratio */
	ret = snd_soc_dai_set_bclk_ratio(amixer_dai, bfs);
	if (ret < 0) {
		dev_err(rtd->card->dev, "failed to set mixer ratio: %d\n",
			ret);
		return ret;
	}

	/*
	 * Codec: PLL1 -> SYSCLK, with the vendor's per-rate reference:
	 * the 26 MHz MCLK at 48 kHz, BCLK1 at 192 kHz.  BCLK1 is the
	 * AMP-port bit clock, always 64fs (GTAXL_BFS_AMP), not the
	 * AP-side bfs.  With PLL1 on BCLK1 at 48 kHz the codec I2S1 TX
	 * sends all-zero frames.
	 */
	if (rate == 192000) {
		pll_src = RT5659_PLL1_S_BCLK1;
		pll_in = rate * GTAXL_BFS_AMP;
	} else {
		pll_src = RT5659_PLL1_S_MCLK;
		pll_in = GTAXL_MCLK_FREQ;
	}
	ret = snd_soc_dai_set_pll(codec_dai, 0, pll_src, pll_in, codec_pll);
	if (ret < 0) {
		dev_err(rtd->card->dev, "failed to set codec PLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5659_SCLK_S_PLL1,
				     codec_pll, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->card->dev, "failed to set codec sysclk: %d\n",
			ret);
		return ret;
	}

	/*
	 * Run the converter filters off the I2S1 clock instead of SYSCLK,
	 * as the vendor mixer_paths.xml does.  The rt5659 driver enables
	 * its ASRC supply widgets only if ASRC_2/ASRC_3 select a tracking
	 * source, and never writes them itself.  Written here, before the
	 * DAPM stream event evaluates is_using_asrc(), and through the
	 * codec regmap so that it survives a cache resync.
	 */
	snd_soc_component_write(codec_dai->component, RT5659_ASRC_2,
				RT5659_DA_STO_T_I2S1 | RT5659_AD_STO1_T_I2S1);
	snd_soc_component_write(codec_dai->component, RT5659_ASRC_3,
				RT5659_AD_MONO_L_T_I2S1 |
				RT5659_AD_MONO_R_T_I2S1);

	return 0;
}

static const struct snd_soc_ops gtaxl_aif1_ops = {
	.hw_params = gtaxl_aif1_hw_params,
};

static const struct snd_kcontrol_new gtaxl_controls[] = {
	SOC_DAPM_PIN_SWITCH("SPK"),
	SOC_DAPM_PIN_SWITCH("HP"),
	SOC_DAPM_PIN_SWITCH("RCV"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

static const struct snd_soc_dapm_widget gtaxl_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("SPK", NULL),
	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_SPK("RCV", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static struct snd_soc_jack gtaxl_headset_jack;

/*
 * Headphone insertion drives gpa0-5 low; the polarity lives in the DT
 * flag, so no software inversion here.  Debounce matches the vendor
 * det-dbtime property.
 */
static struct snd_soc_jack_gpio gtaxl_jack_gpio = {
	.name		= "headset-det",
	.report		= SND_JACK_HEADPHONE,
	.debounce_time	= 50,
};

/*
 * Routing the same report at the speaker pin with invert set gives
 * automatic speaker muting on insertion, without any userspace help.
 */
static struct snd_soc_jack_pin gtaxl_jack_pins[] = {
	{
		.pin	= "HP",
		.mask	= SND_JACK_HEADPHONE,
	}, {
		.pin	= "SPK",
		.mask	= SND_JACK_HEADPHONE,
		.invert	= 1,
	},
};

static int gtaxl_late_probe(struct snd_soc_card *card)
{
	int ret;

	gtaxl_jack_gpio.gpiod_dev = card->dev;

	ret = snd_soc_card_jack_new_pins(card, "Headphone Jack",
					 SND_JACK_HEADPHONE,
					 &gtaxl_headset_jack,
					 gtaxl_jack_pins,
					 ARRAY_SIZE(gtaxl_jack_pins));
	if (ret)
		return dev_err_probe(card->dev, ret,
				     "failed to create the headphone jack\n");

	ret = snd_soc_jack_add_gpios(&gtaxl_headset_jack, 1,
				     &gtaxl_jack_gpio);
	if (ret) {
		/*
		 * Detection is an accessory, the card is not.  Losing the
		 * gpio must not cost the sound card: outputs then stay
		 * under the pin switches.
		 */
		dev_warn(card->dev,
			 "no jack detect gpio (%d), outputs stay manual\n",
			 ret);
		return 0;
	}

	return 0;
}

SND_SOC_DAILINK_DEFS(aif1,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "AP0"),
			   COMP_CODEC(NULL, "rt5659-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link gtaxl_dai_links[] = {
	{
		.name		= "RT5659 AIF1",
		.stream_name	= "HiFi Primary",
		.ops		= &gtaxl_aif1_ops,
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBC_CFC,
		SND_SOC_DAILINK_REG(aif1),
	},
};

static struct snd_soc_card gtaxl_card = {
	.owner			= THIS_MODULE,
	.dai_link		= gtaxl_dai_links,
	.num_links		= ARRAY_SIZE(gtaxl_dai_links),
	.controls		= gtaxl_controls,
	.num_controls		= ARRAY_SIZE(gtaxl_controls),
	.dapm_widgets		= gtaxl_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(gtaxl_dapm_widgets),
	.fully_routed		= true,
	.late_probe		= gtaxl_late_probe,
};

static int gtaxl_audio_probe(struct platform_device *pdev)
{
	struct device_node *cpu_node = NULL, *mixer_node = NULL;
	struct device_node *codec_node = NULL;
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card = &gtaxl_card;
	struct snd_soc_dai_link *link = &gtaxl_dai_links[0];
	void __iomem *pmu_debug;
	int ret;

	card->dev = dev;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0) {
		dev_err(dev, "card name is not specified\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret < 0) {
		dev_err(dev, "audio routing is not specified or invalid\n");
		return ret;
	}

	cpu_node = of_parse_phandle(dev->of_node, "i2s-controller", 0);
	mixer_node = of_parse_phandle(dev->of_node, "audio-mixer", 0);
	codec_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!cpu_node || !mixer_node || !codec_node) {
		dev_err(dev, "i2s-controller, audio-mixer and audio-codec properties are all required\n");
		ret = -EINVAL;
		goto out_put;
	}

	link->cpus->of_node = cpu_node;
	link->platforms->of_node = cpu_node;
	link->codecs[0].of_node = mixer_node;
	link->codecs[1].of_node = codec_node;

	/*
	 * Feed the codec MCLK pin from the PMU CLKOUT pad (see
	 * GTAXL_PMU_DEBUG).  The pad is left always on.
	 *
	 * TODO: model it as an exynos-clkout clock referenced from the
	 * card node and gated per stream.
	 */
	pmu_debug = ioremap(GTAXL_PMU_DEBUG, 4);
	if (!pmu_debug) {
		ret = -ENOMEM;
		goto out_put;
	}
	writel((readl(pmu_debug) & ~GTAXL_PMU_CLKOUT_MASK) |
	       GTAXL_PMU_CLKOUT_ON, pmu_debug);
	iounmap(pmu_debug);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0)
		dev_err_probe(dev, ret, "failed to register card\n");

out_put:
	of_node_put(cpu_node);
	of_node_put(mixer_node);
	of_node_put(codec_node);

	return ret;
}

static const struct of_device_id gtaxl_audio_of_match[] = {
	{ .compatible = "samsung,gtaxl-rt5659" },
	{ },
};
MODULE_DEVICE_TABLE(of, gtaxl_audio_of_match);

static struct platform_driver gtaxl_audio_driver = {
	.driver = {
		.name		= "gtaxl-audio",
		.of_match_table	= gtaxl_audio_of_match,
		.pm		= &snd_soc_pm_ops,
	},
	.probe = gtaxl_audio_probe,
};
module_platform_driver(gtaxl_audio_driver);

MODULE_DESCRIPTION("ALSA SoC Samsung gtaxl RT5659 machine driver");
MODULE_LICENSE("GPL");
