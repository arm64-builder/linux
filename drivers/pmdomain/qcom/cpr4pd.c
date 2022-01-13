/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#define REG_APM_DLY_CNT			0xac
#define APM_SEL_SWITCH_DLY_MASK		GENMASK(7, 0)
#define APM_RESUME_CLK_DLY_MASK		GENMASK(15, 8)
#define APM_HALT_CLK_DLY_MASK		GENMASK(23, 16)
#define APM_POST_HALT_DLY_MASK		GENMASK(31, 24)

#define REG_APM_MODE			0xa8
#define APM_MODE_MASK			GENMASK(1, 0)
#define APM_MODE_MX			0
#define APM_MODE_APCC			2

#define REG_APM_STS			0xb0
#define APM_STS_MASK			GENMASK(4, 0)
#define APM_STS_MX			0
#define APM_STS_APCC			3

#define VREG_STEP_UV			5000
#define NUM_FUSE_REVS			8
#define NUM_SPEED_BINS			8
#define CPR_REF_POINTS			4
#define PD_COUNT			2
#define CPR_VOLTAGE_CEILING_UV		1065000U

#define to_cpr_pd(gpd) container_of(gpd, struct cpr_pd, pd)

struct cpr_pd_info {
	u8 efuse_offsets[CPR_REF_POINTS];
	u8 efuse_shifts[CPR_REF_POINTS];
	u16 ref_pstates[CPR_REF_POINTS];
	u16 ref_mvolts[CPR_REF_POINTS];
	u16 ref_mv_adj_bins_mask;
	s16 ref_mv_adj_by_rev[NUM_FUSE_REVS][CPR_REF_POINTS];
	u16 max_pstate_override_bin_mask;
	u16 max_pstate_override;
};

struct cpr_info {
	u32 apm_threshold_uv;
	const struct cpr_pd_info *pds[PD_COUNT];
};

static const struct cpr_pd_info msm8953_pd_info = {
	.efuse_offsets = { 71, 71, 71, 71 },
	.efuse_shifts = { 24, 18, 12, 6 },
	.ref_pstates  = { 6528, 10368, 16896, 20160 },
	.ref_mvolts = { 645, 720, 865, 1065 },
	/* Bin 0 and 7 use 2208MHz for last reference point */
	.max_pstate_override_bin_mask = BIT(0) | BIT(7),
	.max_pstate_override = 22080,
	/* Closed-loop voltage adjustement for speed bins 0, 2, 6, 7
	 * with fusing revisions of 1-3 */
	.ref_mv_adj_bins_mask = BIT(0) | BIT(2) | BIT(6) | BIT(7),
	.ref_mv_adj_by_rev = {
		[1] = { 25, 0, 5, 40 },
		[2] = { 25, 0, 5, 40 },
		[3] = { 25, 0, 5, 40 },
	},
};

static const struct cpr_info msm8953_info = {
	.apm_threshold_uv = 850000,
	.pds = {
		&msm8953_pd_info,
		&msm8953_pd_info,
	},
};

static const struct cpr_pd_info sdm632_pwr_pd_info = {
	.efuse_offsets = { 74, 71, 74, 74 },
	.efuse_shifts = { 18, 24, 6, 0 },
	.ref_pstates = { 6144, 10368, 13632, 18048 },
	.ref_mvolts = { 645, 790, 865, 1075 },
};

static const struct cpr_pd_info sdm632_perf_pd_info = {
	.efuse_offsets =  { 74, 71, 71, 71 },
	.efuse_shifts = { 18, 18, 12, 6 },
	.ref_pstates = { 6336, 10944, 14016, 20160 },
	.ref_mvolts = { 645, 790, 865, 1065 },
	/* Open-loop voltage adjustement for speed bins 0, 2, 6
	 * with fusing revisions of 0-2 */
	.ref_mv_adj_bins_mask = BIT(0) | BIT(2) | BIT(6),
	.ref_mv_adj_by_rev = {
		[0] = { 30, 0, 10, 20 },
		[1] = { 30, 0, 10, 20 },
		[2] = {  0, 0, 10, 20 },
	},
};

static const struct cpr_info sdm632_info = {
	.apm_threshold_uv = 875000,
	.pds = {
		&sdm632_pwr_pd_info,
		&sdm632_perf_pd_info,
	},
};

static const struct of_device_id soc_match_table[] = {
	{ .compatible = "qcom,msm8953", .data = &msm8953_info },
	{ .compatible = "qcom,sdm450", .data = &msm8953_info },
	{ .compatible = "qcom,sdm632", .data = &sdm632_info },
	{},
};

struct cpr_pd {
	struct generic_pm_domain pd;
	struct regulator *vreg;
	u32 pstates[CPR_REF_POINTS];
	u32 uV[CPR_REF_POINTS];
	u32 uV_per_pstate[CPR_REF_POINTS-1];
};

struct cpr_drv {
	const struct cpr_info *info;
	struct device *dev;
	struct generic_pm_domain *pds[PD_COUNT];
	struct genpd_onecell_data cell_data;
	struct notifier_block vreg_nb, policy_nb;
	void __iomem	*apm; /* Array Power Mux */
	u32 max_uv;
};

static unsigned int cpr_pstate_get_voltage(struct cpr_pd *cpd, unsigned int pstate)
{
	u32 uVolt, i = 0;

	for (;i < (CPR_REF_POINTS - 1) && pstate > cpd->pstates[i + 1]; i++)
		continue;

	pstate = clamp(pstate, cpd->pstates[i], cpd->pstates[i + 1]);
	uVolt = cpd->uV[i] + cpd->uV_per_pstate[i] * (pstate - cpd->pstates[i]);
	uVolt = clamp(uVolt, cpd->uV[i], cpd->uV[i + 1]);
	return min(roundup(uVolt, VREG_STEP_UV), CPR_VOLTAGE_CEILING_UV);
}

static int cpr_pd_set_pstate(struct generic_pm_domain *domain,
			     unsigned int state)
{
	struct cpr_drv *drv = dev_get_drvdata(domain->dev.parent);
	struct cpr_pd *cpd = to_cpr_pd(domain);
	int ret, uv;

	uv = cpr_pstate_get_voltage(cpd, state);
	if (WARN_ON(uv > drv->max_uv || uv < 500000 || uv > 1065000))
		return -EINVAL;

	ret = regulator_set_voltage(cpd->vreg, uv, drv->max_uv);
	if (ret)
		dev_err(&cpd->pd.dev, "failed to set voltage %u uV: %d\n", uv, ret);

	return ret;
}

static unsigned int cpr_pd_opp_to_pstate(struct generic_pm_domain *genpd,
					 struct dev_pm_opp *opp)
{
	return dev_pm_opp_get_level(opp);
}

static int cpr_pd_attach_dev(struct generic_pm_domain *domain,
			     struct device *dev)
{
	pm_runtime_disable(dev);
	return 0;
}

static void cpr_remove_domain(void *data)
{
	pm_genpd_remove((struct generic_pm_domain *) data);
}

static int cpr_apm_init(struct cpr_drv *drv)
{
	struct regulator *vreg;
	u32 val;
	int ret;

	drv->apm = devm_platform_ioremap_resource_byname(
			to_platform_device(drv->dev), "apm");
	if (IS_ERR_OR_NULL(drv->apm))
		return dev_err_probe(drv->dev, PTR_ERR(drv->apm) ?: -ENODATA,
				"could not map APM memory\n");

	val = readl_relaxed(drv->apm + REG_APM_DLY_CNT);
	val &= ~APM_POST_HALT_DLY_MASK & ~APM_HALT_CLK_DLY_MASK &
	       ~APM_RESUME_CLK_DLY_MASK & ~APM_SEL_SWITCH_DLY_MASK;
	val |= FIELD_PREP(APM_POST_HALT_DLY_MASK, 0x02) |
	       FIELD_PREP(APM_HALT_CLK_DLY_MASK, 0x11) |
	       FIELD_PREP(APM_RESUME_CLK_DLY_MASK, 0x10) |
	       FIELD_PREP(APM_SEL_SWITCH_DLY_MASK, 0x01);
	writel_relaxed(val, drv->apm + REG_APM_DLY_CNT);

	vreg = devm_regulator_get(drv->dev, "apc");
	if (IS_ERR(vreg))
		return dev_err_probe(drv->dev, PTR_ERR(vreg),
				"could not get regulator\n");

	ret = devm_regulator_register_notifier(vreg, &drv->vreg_nb);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				"could not register regulator notifier\n");

	return 0;
}

static int cpr_apm_switch_supply(struct cpr_drv *drv, u32 val, u32 done_status)
{
	int ret;

	writel_relaxed(val, drv->apm + REG_APM_MODE);

	ret = readl_relaxed_poll_timeout_atomic(drv->apm + REG_APM_STS, val,
			(val & APM_STS_MASK) == done_status, 1, 500);
	if (!ret)
		return NOTIFY_OK;

	dev_err(drv->dev, "failed to switch APM: %d", ret);
	return NOTIFY_BAD;
}

static int cpr_vreg_notifier(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct cpr_drv *drv = container_of(nb, struct cpr_drv, vreg_nb);
	unsigned long voltage;
	u32 mode;

	mode = readl_relaxed(drv->apm + REG_APM_MODE) & APM_MODE_MASK;

	if (action == REGULATOR_EVENT_PRE_VOLTAGE_CHANGE) {
		voltage = ((struct pre_voltage_change_data*) data)->min_uV;
		if (mode != APM_MODE_MX &&
		    voltage < drv->info->apm_threshold_uv)
			return cpr_apm_switch_supply(drv, APM_MODE_MX, APM_STS_MX);
	} else if (action == REGULATOR_EVENT_VOLTAGE_CHANGE) {
		voltage = (unsigned long) data;
		if (mode != APM_MODE_APCC && voltage >= drv->info->apm_threshold_uv)
			return cpr_apm_switch_supply(drv, APM_MODE_APCC, APM_STS_APCC);
	}

	return NOTIFY_OK;
}

static struct generic_pm_domain* cpr_init_domain(struct device *dev,
		const struct cpr_pd_info *info,
		unsigned int index)
{
	struct cpr_drv *drv = dev_get_drvdata(dev);
	struct nvmem_device *nvmem;
	u8 speed_bin, fusing_rev;
	struct cpr_pd *cpd;
	int ret, i;

	cpd = devm_kzalloc(dev, sizeof(*cpd), GFP_KERNEL);
	if (!cpd)
		return ERR_PTR(-ENOMEM);

	cpd->pd.name = index ? "cpr_pd_perf" : "cpr_pd_pwr";
	cpd->pd.flags = GENPD_FLAG_RPM_ALWAYS_ON;
	cpd->pd.attach_dev = cpr_pd_attach_dev;
	cpd->pd.opp_to_performance_state = cpr_pd_opp_to_pstate;
	cpd->pd.set_performance_state = cpr_pd_set_pstate;

	ret = pm_genpd_init(&cpd->pd, NULL, false);
	if (ret)
		return ERR_PTR(ret);

	ret = devm_add_action_or_reset(dev, cpr_remove_domain, &cpd->pd);
	if (ret) {
		cpr_remove_domain(&cpd->pd);
		return ERR_PTR(ret);
	}

	cpd->pd.dev.parent = dev;
	cpd->pd.dev.of_node = dev->of_node;
	cpd->vreg = devm_regulator_get(dev, "apc");
	if (IS_ERR(cpd->vreg))
		return ERR_PTR(PTR_ERR(cpd->vreg));

	ret = nvmem_cell_read_u8(dev, "fusing_rev", &fusing_rev);
	ret = ret ?: nvmem_cell_read_u8(dev, "speed_bin", &speed_bin);
	if (ret)
		return ERR_PTR(dev_err_probe(dev, ret,
				"failed to read speed bin and fusing revision\n"));

	nvmem = devm_nvmem_device_get(dev, NULL);
	if (IS_ERR(nvmem))
		return ERR_PTR(PTR_ERR(nvmem));

	for (i = 0; i < CPR_REF_POINTS; i ++) {
		unsigned int uVolt, pstate;
		u64 efuse;

		pstate = info->ref_pstates[i];
		uVolt = info->ref_mvolts[i] * 1000;

		ret = nvmem_device_read(nvmem, 8 * info->efuse_offsets[i],
				sizeof(efuse), &efuse);
		if (ret < 0)
			return ERR_PTR(dev_err_probe(dev, ret,
						"Failed to read nvmem\n"));

		/* decode reference voltage offset */
		efuse >>= info->efuse_shifts[i];

		uVolt += ((efuse & 0x20) ? -10 : 10) * (efuse & 0x1f) * 1000;
		if (info->ref_mv_adj_bins_mask & BIT(speed_bin))
			uVolt += info->ref_mv_adj_by_rev[fusing_rev][i];

		if (i && uVolt < cpd->uV[i - 1])
			uVolt = cpd->uV[i - 1];

		drv->max_uv = max(drv->max_uv, uVolt);
		cpd->pstates[i] = pstate;
		cpd->uV[i] = uVolt;
	}

	devm_nvmem_device_put(dev, nvmem);

	if (info->max_pstate_override_bin_mask & BIT(speed_bin))
		cpd->pstates[CPR_REF_POINTS - 1] = info->max_pstate_override;

	if (!index)
		dev_info(dev, "speed_bin=%d fusing_revision=%d\n",
			 speed_bin, fusing_rev);

	for (i = 0; i < (CPR_REF_POINTS - 1); i++) {
		u32 step = cpd->uV[i + 1] - cpd->uV[i];
		step /= (cpd->pstates[i + 1] - cpd->pstates[i]);
		cpd->uV_per_pstate[i] = step;
		dev_info(&cpd->pd.dev, "level=%5u uV=%6u step=%2u\n",
			 cpd->pstates[i], cpd->uV[i], cpd->uV_per_pstate[i]);
	}

	dev_info(&cpd->pd.dev, "level=%5u uV=%2u\n", cpd->pstates[i], cpd->uV[i]);

	return &cpd->pd;
}

/* Update voltages on CPU devices for Energy model */
static int cpr_cpufreq_policy_notifier(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct cpr_drv *drv = container_of(nb, struct cpr_drv, policy_nb);
	struct cpufreq_policy *policy = data;
	unsigned long freq;
	struct device *dev;
	struct dev_pm_opp *opp;
	struct cpr_pd *cpd;
	unsigned cpu;

	if (action != CPUFREQ_CREATE_POLICY)
		return NOTIFY_OK;

	cpu = cpumask_first(policy->related_cpus);
	dev = get_cpu_device(cpu / 4);
	if (cpu >= 8 || IS_ERR_OR_NULL(dev))
		return NOTIFY_OK;

	cpd = to_cpr_pd(drv->pds[cpu / 4]);

	for (freq = 0, opp = dev_pm_opp_find_freq_ceil(dev, &freq);
	     !IS_ERR_OR_NULL(opp);
	     freq ++, opp = dev_pm_opp_find_freq_ceil(dev, &freq)) {
		unsigned int pstate, uVolt;
		pstate = dev_pm_opp_get_required_pstate(opp, 0);
		dev_pm_opp_put(opp);
		uVolt = cpr_pstate_get_voltage(cpd, pstate);
		dev_info(&cpd->pd.dev, "Freq=%lu uV=%u\n", freq / 1000, uVolt);
		dev_pm_opp_adjust_voltage(dev, freq, uVolt, uVolt, drv->max_uv);
	}

	return NOTIFY_OK;
}

static int cpr_probe(struct platform_device *pdev)
{

	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	const struct cpr_info *info;
	struct device_node *np;
	struct cpr_drv *drv;
	int i, ret;

	np = of_find_node_by_path("/");
	if (!np)
		return -ENOENT;

	match = of_match_node(soc_match_table, np);
	of_node_put(np);
	if (!match)
		return dev_err_probe(dev, -EINVAL,
				"couldn't match SoC compatible\n");
	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	dev_set_drvdata(dev, drv);

	drv->dev = dev;
	drv->info = info = match->data;
	drv->vreg_nb.notifier_call = cpr_vreg_notifier;
	drv->policy_nb.notifier_call = cpr_cpufreq_policy_notifier;
	drv->cell_data.domains = drv->pds;
	drv->cell_data.num_domains = PD_COUNT;

	ret = cpr_apm_init(drv);
	if (ret)
		return ret;

	for (i = 0; i < PD_COUNT; i ++) {
		if (i && info->pds[i] == info->pds[i - 1])
			drv->pds[i] = drv->pds[i - 1];
		else
			drv->pds[i] = cpr_init_domain(dev, info->pds[i], i);

		if(IS_ERR(drv->pds[i]))
			return PTR_ERR(drv->pds[i]);
	}

	drv->max_uv = min(drv->max_uv, CPR_VOLTAGE_CEILING_UV);

	ret = cpufreq_register_notifier(&drv->policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return dev_err_probe(drv->dev, ret,
				"could not register cpufreq notifier\n");

	return of_genpd_add_provider_onecell(dev->of_node, &drv->cell_data);
}

static int cpr_remove(struct platform_device *pdev)
{
	struct cpr_drv *drv = platform_get_drvdata(pdev);
	of_genpd_del_provider(pdev->dev.of_node);
	cpufreq_unregister_notifier(&drv->policy_nb, CPUFREQ_POLICY_NOTIFIER);
	return 0;
}

static const struct of_device_id cpr_match_table[] = {
	{ .compatible = "qcom,msm8953-cpr4pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, cpr_match_table);

static struct platform_driver cpr_driver = {
	.probe		= cpr_probe,
	.remove		= cpr_remove,
	.driver		= {
		.name	= "qcom-cpr4pd",
		.of_match_table = cpr_match_table,
	},
};
module_platform_driver(cpr_driver);

MODULE_DESCRIPTION("Core Power Reduction (CPR) v4 driver for MSM8953");
MODULE_LICENSE("GPL v2");
