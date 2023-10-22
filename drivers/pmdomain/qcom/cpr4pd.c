/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/bitfield.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
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
#define NUM_CPUS_IN_CLUSTER		4
#define NUM_CPUS			8
#define CPR_REF_POINTS			4
#define CPR_VOLT_CEILING_UV		1065000U
#define CPR_VOLT_FLOOR_UV		500000U
#define CPR_PD_COUNT			2

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

struct cpr_acc_config {
	unsigned pstate_max;
	const u32 *regs;
};

struct cpr_info {
	u32 apm_thr_uv;
	const char *acc_reg_name;
	const struct cpr_acc_config *acc_configs;
	unsigned int num_acc_configs, acc_regs_size;
	const struct cpr_pd_info *pds[CPR_PD_COUNT];
};

static const struct cpr_pd_info msm8953_pd_info = {
	.efuse_offsets = { 71, 71, 71, 71 },
	.efuse_shifts = { 24, 18, 12, 6 },
	.ref_pstates  = { 6528, 10368, 16896, 20160 },
	.ref_mvolts = { 645, 720, 865, 1065 },
	/* Bin 0 and 7 use 2208MHz for last reference point */
	.max_pstate_override_bin_mask = BIT(0) | BIT(7),
	.max_pstate_override = 22080,
	/* Open-loop voltage adjustement for speed bins 0, 2, 6, 7
	 * with fusing revisions of 1-3 */
	.ref_mv_adj_bins_mask = BIT(0) | BIT(2) | BIT(6) | BIT(7),
	.ref_mv_adj_by_rev = {
		[1] = { 25, 0, 5, 40 },
		[2] = { 25, 0, 5, 40 },
		[3] = { 25, 0, 5, 40 },
	},
};

static const struct cpr_acc_config msm8953_acc_configs[] = {
	{
		.pstate_max = 12192,
		.regs = (const u32[]) { 1, 1 },
	},
	{
		.pstate_max = 22080,
		.regs = (const u32[]) { 0, 0 },
	},
};

static const struct cpr_info msm8953_info = {
	.apm_thr_uv = 850000,
	.acc_reg_name = "mem-acc",
	.acc_configs = msm8953_acc_configs,
	.num_acc_configs = ARRAY_SIZE(msm8953_acc_configs),
	.acc_regs_size = sizeof(u32) * 2,
	.pds = { &msm8953_pd_info, &msm8953_pd_info },
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

static const struct cpr_acc_config sdm632_acc_configs[] = {
	{
		.pstate_max = 9600,
		.regs = (const u32[]) { 0, BIT(31), 0, 0, BIT(31) },
	},
	{
		.pstate_max = 17400,
		.regs = (const u32[]) { 0, 0, 0, 0, 0, },
	},
	{
		.pstate_max = 20160,
		.regs = (const u32[]) { 0, 1, 0, BIT(16), 0 },
	}
};

static const struct cpr_info sdm632_info = {
	.apm_thr_uv = 875000,
	.acc_reg_name = "apcs-acc",
	.acc_configs = sdm632_acc_configs,
	.num_acc_configs = ARRAY_SIZE(sdm632_acc_configs),
	.acc_regs_size = sizeof(u32) * 5,
	.pds = { &sdm632_pwr_pd_info, &sdm632_perf_pd_info },
};

static const struct of_device_id soc_match_table[] = {
	{ .compatible = "qcom,msm8953", .data = &msm8953_info },
	{ .compatible = "qcom,sdm450", .data = &msm8953_info },
	{ .compatible = "qcom,sdm632", .data = &sdm632_info },
	{},
};

struct cpr_pd_data {
	u32 pstates[CPR_REF_POINTS];
	u32 uV[CPR_REF_POINTS];
	u32 uV_per_pstate[CPR_REF_POINTS-1];
};

struct cpr_pd {
	const struct cpr_pd_data *data;
	struct generic_pm_domain pd;
	u32 uv, pstate;
};

struct cpr_drv {
	const struct cpr_info *info;
	struct device *dev;
	struct generic_pm_domain *pds[CPR_PD_COUNT];
	struct genpd_onecell_data cell_data;
	struct mutex lock;
	struct notifier_block policy_nb;
	struct regulator *vreg;
	/* Array Power Mux and Memory Accelecrator regulators */
	void __iomem *apm, *acc;
	/* Floor value used during boot until consumers are synced */
	u32 boot_up_uv;
	u32 uv, pstate;
};

static void cpr_config_mem_acc(struct cpr_drv *drv, u32 pstate)
{
	const struct cpr_acc_config *cfg = drv->info->acc_configs;
	int num = drv->info->num_acc_configs;

	while (num > 0 && pstate > cfg->pstate_max)
		cfg += --num > 0;

	__iowrite32_copy(drv->acc, cfg->regs, drv->info->acc_regs_size);
}

static void cpr_apm_init(struct cpr_drv *drv)
{
	u32 val = FIELD_PREP(APM_POST_HALT_DLY_MASK, 0x02)
		| FIELD_PREP(APM_HALT_CLK_DLY_MASK, 0x11)
		| FIELD_PREP(APM_RESUME_CLK_DLY_MASK, 0x10)
		| FIELD_PREP(APM_SEL_SWITCH_DLY_MASK, 0x01);
	writel_relaxed(val, drv->apm + REG_APM_DLY_CNT);
}

static int cpr_apm_switch_supply(struct cpr_drv *drv, bool high)
{
	u32 done_status = high ? APM_STS_APCC : APM_STS_MX;
	u32 val = high ? APM_MODE_APCC : APM_MODE_MX;
	int ret;

	writel_relaxed(val, drv->apm + REG_APM_MODE);
	ret = readl_relaxed_poll_timeout_atomic(drv->apm + REG_APM_STS,
			val, (val & APM_STS_MASK) == done_status, 1, 500);
	if (ret)
		dev_err(drv->dev, "failed to switch APM: %d", ret);

	return ret;;
}


static u32 cpr_pstate_get_voltage(const struct cpr_pd_data *data, u32 pstate)
{
	u32 uVolt, i = 0;
	for (;i < (CPR_REF_POINTS - 1) && pstate > data->pstates[i + 1]; i++)
		continue;

	pstate = clamp(pstate, data->pstates[i], data->pstates[i + 1]);
	uVolt = data->uV[i] + data->uV_per_pstate[i] * (pstate - data->pstates[i]);
	uVolt = clamp(uVolt, data->uV[i], data->uV[i + 1]);
	uVolt = roundup(uVolt, VREG_STEP_UV);
	return clamp(uVolt, CPR_VOLT_FLOOR_UV, CPR_VOLT_CEILING_UV);
}

static int cpr_vreg_set_voltage(struct cpr_drv *drv, u32 uv)
{
	u32 apm_thr_uv;
	int ret;

	apm_thr_uv = clamp(drv->info->apm_thr_uv,
			   min(uv, drv->uv), max(uv, drv->uv));

	/* Crossing APM threshold */
	if (apm_thr_uv == drv->info->apm_thr_uv) {
		ret = regulator_set_voltage(drv->vreg, apm_thr_uv, apm_thr_uv);
		if (ret)
			return ret;

		ret = cpr_apm_switch_supply(drv, uv > apm_thr_uv);
		if (ret)
			goto fail_restore_uv;
	}

	ret = regulator_set_voltage(drv->vreg, uv, uv);
	if (ret && apm_thr_uv == drv->info->apm_thr_uv)
		goto fail_restore_uv;

	return 0;

fail_restore_uv:
	dev_err(drv->dev, "failed to set voltage %u uV: %d\n", uv, ret);
	(void) regulator_set_voltage(drv->vreg, drv->uv, drv->uv);
	return ret;
}

static int cpr_pd_set_pstate_unlocked(struct cpr_drv *drv, struct cpr_pd *cpd,
				      unsigned int pstate)
{
	u32 pstate_aggr, uv_aggr, uv;
	int ret, i;

	pstate_aggr = pstate;
	uv_aggr = uv = cpr_pstate_get_voltage(cpd->data, pstate);

	for (i = 0; i < CPR_PD_COUNT; i ++) {
		struct cpr_pd *other = to_cpr_pd(drv->pds[i]);
		uv_aggr = max(other->uv, uv_aggr);
		pstate_aggr = max(other->pstate, pstate_aggr);
	}

	uv_aggr = max(uv_aggr, drv->boot_up_uv);
	if (uv_aggr < drv->uv)
		cpr_config_mem_acc(drv, pstate_aggr);

	if (pstate_aggr == drv->pstate && uv_aggr == drv->uv)
		goto skip_update;

	ret = cpr_vreg_set_voltage(drv, uv_aggr);
	if (ret) {
		if (pstate_aggr < drv->pstate)
			cpr_config_mem_acc(drv, drv->pstate);
		return ret;
	}

	if (uv_aggr > drv->uv)
		cpr_config_mem_acc(drv, pstate_aggr);

skip_update:
	cpd->uv = uv;
	cpd->pstate = pstate;
	drv->uv = uv_aggr;
	drv->pstate = pstate_aggr;
	return 0;
}


static int cpr_pd_set_pstate(struct generic_pm_domain *domain,
			     unsigned int pstate)
{
	struct cpr_drv *drv = dev_get_drvdata(domain->dev.parent);
	struct cpr_pd *cpd = to_cpr_pd(domain);
	int ret;

	mutex_lock(&drv->lock);
	ret = cpr_pd_set_pstate_unlocked(drv, cpd, pstate);
	mutex_unlock(&drv->lock);
	return ret;
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

static struct generic_pm_domain* cpr_init_domain(struct device *dev,
		const struct cpr_pd_info *info,
		unsigned int index)
{
	struct nvmem_device *nvmem;
	u8 speed_bin, fusing_rev;
	struct cpr_pd_data *data;
	struct cpr_pd *cpd;
	int ret, i;

	cpd = devm_kzalloc(dev, sizeof(*cpd), GFP_KERNEL);
	if (!cpd)
		return ERR_PTR(-ENOMEM);

	cpd->data = data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	cpd->pd.name = index ? "cpr_pd_perf" : "cpr_pd_pwr";
	cpd->pd.flags = GENPD_FLAG_RPM_ALWAYS_ON;
	cpd->pd.attach_dev = cpr_pd_attach_dev;
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

		if (i && uVolt < data->uV[i - 1])
			uVolt = data->uV[i - 1];

		data->pstates[i] = pstate;
		data->uV[i] = uVolt;
	}

	devm_nvmem_device_put(dev, nvmem);

	if (info->max_pstate_override_bin_mask & BIT(speed_bin))
		data->pstates[CPR_REF_POINTS - 1] = info->max_pstate_override;

	if (!index)
		dev_info(dev, "speed_bin=%d fusing_revision=%d\n",
			 speed_bin, fusing_rev);

	for (i = 0; i < (CPR_REF_POINTS - 1); i++) {
		u32 step = data->uV[i + 1] - data->uV[i];
		step /= (data->pstates[i + 1] - data->pstates[i]);
		data->uV_per_pstate[i] = step;
		dev_info(&cpd->pd.dev, "level=%5u uV=%6u step=%2u\n",
			 data->pstates[i], data->uV[i], data->uV_per_pstate[i]);
	}

	dev_info(&cpd->pd.dev, "level=%5u uV=%2u\n", data->pstates[i], data->uV[i]);

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
	dev = get_cpu_device(cpu);
	if (cpu >= NUM_CPUS || IS_ERR_OR_NULL(dev))
		return NOTIFY_OK;

	cpd = to_cpr_pd(drv->pds[cpu / NUM_CPUS_IN_CLUSTER]);
	for (freq = 0, opp = dev_pm_opp_find_freq_ceil(dev, &freq);
	     !IS_ERR_OR_NULL(opp);
	     freq ++, opp = dev_pm_opp_find_freq_ceil(dev, &freq)) {
		unsigned int pstate, uVolt;
		pstate = dev_pm_opp_get_required_pstate(opp, 0);
		dev_pm_opp_put(opp);
		uVolt = cpr_pstate_get_voltage(cpd->data, pstate);
		dev_info(&cpd->pd.dev, "Freq=%lu uV=%u\n", freq / 1000, uVolt);
		dev_pm_opp_adjust_voltage(dev, freq, uVolt, uVolt, uVolt);
	}

	return NOTIFY_OK;
}

static void cpr_sync_state(struct device *dev)
{
	struct cpr_drv *drv = dev_get_drvdata(dev);
	struct cpr_pd *cpd = to_cpr_pd(drv->pds[0]);

	mutex_lock(&drv->lock);
	drv->boot_up_uv = 0;
	cpr_pd_set_pstate_unlocked(drv, cpd, cpd->pstate);
	mutex_unlock(&drv->lock);
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
	mutex_init(&drv->lock);
	drv->dev = dev;
	drv->info = info = match->data;
	drv->policy_nb.notifier_call = cpr_cpufreq_policy_notifier;
	drv->cell_data.domains = drv->pds;
	drv->cell_data.num_domains = CPR_PD_COUNT;
	drv->vreg = devm_regulator_get(drv->dev, "apc");
	if (IS_ERR(drv->vreg))
		return dev_err_probe(drv->dev, PTR_ERR(drv->vreg),
				"could not get regulator\n");

	ret = regulator_get_voltage(drv->vreg);
	if (ret < 0)
		return ret;

	drv->boot_up_uv = drv->uv = ret;
	dev_info(dev, "boot time voltage: %u uV\n", drv->boot_up_uv);
	drv->acc = devm_platform_ioremap_resource_byname(pdev, info->acc_reg_name);
	if (IS_ERR_OR_NULL(drv->acc))
		return dev_err_probe(drv->dev, PTR_ERR(drv->acc) ?: -ENODATA,
				"could not map ACC memory\n");

	drv->apm = devm_platform_ioremap_resource_byname(pdev, "apm");
	if (IS_ERR_OR_NULL(drv->apm))
		return dev_err_probe(drv->dev, PTR_ERR(drv->apm) ?: -ENODATA,
				"could not map APM memory\n");

	cpr_apm_init(drv);

	for (i = 0; i < CPR_PD_COUNT; i ++) {
		if (i && info->pds[i] == info->pds[i - 1])
			drv->pds[i] = drv->pds[i - 1];
		else
			drv->pds[i] = cpr_init_domain(dev, info->pds[i], i);

		if (IS_ERR(drv->pds[i]))
			return PTR_ERR(drv->pds[i]);
	}

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
		.name		= "qcom-cpr4pd",
		.of_match_table = cpr_match_table,
		.sync_state	= cpr_sync_state,
	},
};
module_platform_driver(cpr_driver);

MODULE_DESCRIPTION("Core Power Reduction (CPR) v4 driver for MSM8953");
MODULE_AUTHOR("Vladimir Lypak <vladimir.lypak@gmail.com>");
MODULE_LICENSE("GPL v2");
