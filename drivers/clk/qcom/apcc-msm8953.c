// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,apcc-msm8953.h>

#include "clk-alpha-pll.h"
#include "clk-regmap-mux-div.h"

static const char *apcc_clk_names[APCC_NUM_CLOCKS] = {
	[APCC_CPU0_CLK] = "cpu0-rcg-clk",
	[APCC_CPU4_CLK] = "cpu4-rcg-clk",
	[APCC_CCI_CLK] = "cci-rcg-clk",
	[APCC_CPU0_PLL] = "cpu0-pll-clk",
	[APCC_CPU4_PLL] = "cpu4-pll-clk",
	[APCC_CCI_PLL] = "cci-pll-clk",
};

static const struct pll_vco apcc_msm8953_pll_vcos[] = {
	VCO(0, 652800000, 2208000000),
};

static const u8 apcc_msm8953_alpha_pll_regs[PLL_OFF_MAX_REGS] = {
	[PLL_OFF_L_VAL]		= 0x08,
	[PLL_OFF_ALPHA_VAL]	= 0x10,
	[PLL_OFF_USER_CTL]	= 0x18,
	[PLL_OFF_CONFIG_CTL]	= 0x20,
	[PLL_OFF_CONFIG_CTL_U]	= 0x24,
	[PLL_OFF_STATUS]	= 0x28,
	[PLL_OFF_TEST_CTL]	= 0x30,
	[PLL_OFF_TEST_CTL_U]	= 0x34,
};

static const struct alpha_pll_config apcc_msm8953_pll_cfg = {
	.l			= 34,
	.config_ctl_val		= 0x200d4828,
	.config_ctl_hi_val	= 0x6,
	.test_ctl_val		= 0x1c000000,
	.test_ctl_hi_val	= 0x4000,
	.main_output_mask	= BIT(0),
	.early_output_mask	= BIT(3),
};

static const struct pll_vco apcc_sdm632_pll_vcos[] = {
	VCO(0, 614400000, 2016000000), /* Pwr cluster PLL */
	VCO(0, 633600000, 2016000000), /* Perf cluster PLL */
	VCO(2, 500000000, 1000000000), /* CCI PLL */
};

static const struct alpha_pll_config apcc_sdm632_cci_pll_cfg = {
	.l			= 28,
	.config_ctl_val		= 0x4001055b,
	.early_output_mask	= BIT(3),
};

static int apcc_clk_regmap_init(struct device *dev, struct clk_regmap* rclk,
				unsigned max_reg)
{
	struct regmap_config cfg = { 0 };
	void __iomem *base;

	cfg.fast_io = true;
	cfg.reg_bits = 32;
	cfg.reg_stride = 4;
	cfg.val_bits = 32;
	cfg.max_register = max_reg;
	cfg.name = rclk->hw.init->name;

	base = devm_platform_ioremap_resource_byname(to_platform_device(dev),
						     cfg.name);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				"failed to map memory for \"%s\"\n", cfg.name);

	rclk->regmap = devm_regmap_init_mmio(dev, base, &cfg);
	if (IS_ERR(rclk->regmap))
		return dev_err_probe(dev, PTR_ERR(rclk->regmap),
				"failed to init regmap for \"%s\"\n", cfg.name);
	return 0;
}

static struct clk_ops msm8953_alpha_pll_ops;
struct cpu_mux_div {
	struct clk_regmap_mux_div md;
	struct notifier_block nb, pll_nb;
	struct clk_notifier_data pll_cnd;
	int var_pll_index;
};

/*
 * Clock framework doesn't care about intermediate frequencies and always
 * changes rate of parent clock first. For CPU clocks this is not
 * acceptable.
 */
static int cpu_mux_div_pll_notifier(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct cpu_mux_div *cmd = container_of(nb, struct cpu_mux_div, pll_nb);
	struct clk_notifier_data *cnd = data;

	/* Catch when we're about to increase variable PLL rate while it's
	 * selected in the mux. */
	if (action == PRE_RATE_CHANGE && cnd->new_rate > cnd->old_rate &&
	    cmd->var_pll_index == clk_hw_get_parent_index(&cmd->md.clkr.hw))
		cmd->pll_cnd = *cnd;

	return NOTIFY_OK;
}

static int cpu_mux_div_notifier(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct cpu_mux_div *cmd = container_of(nb, struct cpu_mux_div, nb);
	unsigned long max_rate, max_div = BIT(cmd->md.hid_width);
	unsigned long imd_rate, imd_div = cmd->md.div + 1;
	struct clk_notifier_data *cnd = data;

	if (action != PRE_RATE_CHANGE ||
	    cmd->pll_cnd.new_rate == cmd->pll_cnd.old_rate)
		return NOTIFY_OK;

	max_rate = max(cnd->old_rate, cnd->new_rate);

	do {
		imd_rate = mult_frac(cmd->pll_cnd.new_rate, 2, imd_div);
	} while (imd_rate > max_rate && ++imd_div <= max_div);

	if (imd_div != cmd->md.div + 1)
		mux_div_set_src_div(&cmd->md, cmd->md.src, imd_div - 1);

	cmd->pll_cnd.old_rate = cmd->pll_cnd.new_rate;
	return NOTIFY_OK;
}

static int apcc_msm8953_register_pll(struct device *dev, int pll_idx, int num_plls)
{
	struct clk_hw_onecell_data *data = dev_get_drvdata(dev);
	const struct alpha_pll_config *pll_config;
	int clk_idx = APCC_CPU0_PLL + pll_idx;
	struct clk_init_data init = { 0 };
	struct clk_alpha_pll *apll;
	int ret;

	if (pll_idx >= num_plls) {
		data->hws[clk_idx] = data->hws[APCC_CPU0_PLL];
		return 0;
	}

	apll = devm_kzalloc(dev, sizeof(*apll), GFP_KERNEL);
	if (!apll)
		return -ENOMEM;

	init.name = apcc_clk_names[clk_idx],
	init.num_parents = 1,
	init.ops = &msm8953_alpha_pll_ops,
	init.parent_data = &(const struct clk_parent_data) { .fw_name = "xo" },
	apll->clkr.hw.init = &init;
	apll->flags = SUPPORTS_DYNAMIC_UPDATE;
	apll->vco_table = &apcc_msm8953_pll_vcos[pll_idx];
	apll->num_vco = 1;
	apll->regs = apcc_msm8953_alpha_pll_regs;
	pll_config = &apcc_msm8953_pll_cfg;
	data->hws[clk_idx] = &apll->clkr.hw;

	if (num_plls) {
		apll->vco_table = &apcc_sdm632_pll_vcos[pll_idx];
		if (clk_idx == APCC_CCI_PLL) {
			apll->regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT];
			pll_config = &apcc_sdm632_cci_pll_cfg;
		}
	}

	ret = apcc_clk_regmap_init(dev, &apll->clkr, 0x34);
	if (ret)
		return ret;

	clk_alpha_pll_configure(apll, apll->clkr.regmap, pll_config);

	ret = devm_clk_hw_register(dev, &apll->clkr.hw);
	if (ret)
		return dev_err_probe(dev, ret,
				"failed to register clock: %s\n", init.name);

	return 0;
}

static int apcc_msm8953_register_mux(struct device *dev, int clk_idx, int num_plls)
{
	struct clk_hw_onecell_data *data = dev_get_drvdata(dev);
	static const u32 parent_map[] = { 5, 4, 2 };
	struct clk_parent_data pdata[3] = { };
	struct clk_init_data init = { 0 };
	struct clk_regmap_mux_div *md;
	struct cpu_mux_div *cmd;
	int ret, parent_idx = 0;

	ret = apcc_msm8953_register_pll(dev, clk_idx, num_plls);
	if (ret)
		return ret;

	cmd = devm_kzalloc(dev, sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->pll_nb.notifier_call = cpu_mux_div_pll_notifier;
	cmd->nb.notifier_call = cpu_mux_div_notifier;
	cmd->var_pll_index = parent_idx;
	pdata[parent_idx++].hw = data->hws[APCC_CPU0_PLL + clk_idx];
	pdata[parent_idx++].fw_name = "muxsrc4";
	pdata[parent_idx++].fw_name = "muxsrc2";
	init.flags = CLK_IS_CRITICAL;
	if (clk_idx < APCC_CCI_CLK || num_plls > 2)
		init.flags |= CLK_SET_RATE_PARENT;
	init.name = apcc_clk_names[clk_idx];
	init.num_parents = parent_idx;
	init.parent_data = pdata;
	init.ops = &clk_regmap_mux_div_ops;
	md = &cmd->md;
	md->parent_map = parent_map;
	md->clkr.hw.init = &init;
	md->clkr.enable_reg = 8;
	md->clkr.enable_mask = BIT(0);
	md->hid_width = 5;
	md->src_shift = 8;
	md->src_width = 3;
	md->reg_offset = 0;
	data->hws[clk_idx] = &md->clkr.hw;

	ret = apcc_clk_regmap_init(dev, &md->clkr, 0xc);
	if (ret)
		return ret;

	ret = devm_clk_hw_register(dev, &md->clkr.hw);
	if (ret)
		return dev_err_probe(dev, ret,
				"failed to register clock: %s\n", init.name);

	ret = devm_clk_notifier_register(dev, pdata[cmd->var_pll_index].hw->clk,
					 &cmd->pll_nb);
	ret = ret ?: devm_clk_notifier_register(dev, md->clkr.hw.clk, &cmd->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				"failed to register clock notifier\n");
	return 0;
}

static long apcc_msm8953_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *prate)
{
	 return clk_alpha_pll_ops.round_rate(hw, rounddown(rate, *prate), prate);
}

static const struct of_device_id apcc_soc_match_table[] = {
	{ .compatible = "qcom,msm8953", .data = (void*) 1 },
	{ .compatible = "qcom,sdm450", .data = (void*) 1 },
	{ .compatible = "qcom,sdm632", .data = (void*) 3 },
	{},
};

static int apcc_msm8953_probe(struct platform_device *pdev)
{
	struct dev_pm_opp_config config = { 0 };
	const struct of_device_id *match;
	struct clk_hw_onecell_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	int ret, clk_idx, num_plls;

	np = of_find_node_by_path("/");
	if (!np)
		return -ENOENT;

	match = of_match_node(apcc_soc_match_table, np);
	of_node_put(np);
	if (!match)
		return dev_err_probe(dev, -ENODEV,
				"couldn't match SoC compatible\n");

	num_plls = (long) match->data;

	msm8953_alpha_pll_ops = clk_alpha_pll_ops;
	msm8953_alpha_pll_ops.round_rate = apcc_msm8953_pll_round_rate;

	data = devm_kzalloc(dev, offsetof(struct clk_hw_onecell_data, hws[APCC_NUM_CLOCKS]),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(dev, data);
	data->num = APCC_NUM_CLOCKS;

	ret = 0;
	for (clk_idx = 0; clk_idx <= APCC_CCI_CLK; clk_idx++)
		ret = ret ?: apcc_msm8953_register_mux(dev, clk_idx, num_plls);
	if (ret)
		return ret;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
	if (ret)
		return dev_err_probe(dev, ret, "couldn't register clock provider\n");


	if (!of_property_read_bool(dev->of_node, "operating-points-v2"))
		return 0;

	config.genpd_names = (const char *[]) { "cpr", NULL };
	config.clk_names = (const char *[]) { "cci", NULL };

	ret = devm_pm_opp_set_config(dev, &config);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set OPP config\n");

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add OPP table\n");

	return 0;
}

static const struct of_device_id apcc_msm8953_match_table[] = {
	{ .compatible = "qcom,apcc-msm8953" },
	{}
};

static struct platform_driver apcc_msm8953_driver = {
	.probe = apcc_msm8953_probe,
	.driver = {
		.name = "apcc-msm8953",
		.of_match_table = apcc_msm8953_match_table,
	},
};

module_platform_driver(apcc_msm8953_driver);
MODULE_DEVICE_TABLE(of, apcc_msm8953_match_table);
MODULE_AUTHOR("Vladimir Lypak <vladimir.lypak@gmail.com>");
MODULE_LICENSE("GPL v2");
