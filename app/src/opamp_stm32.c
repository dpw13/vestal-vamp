
#define DT_DRV_COMPAT st_stm32_opamp

#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/toolchain.h>

#include <stm32h7xx_ll_opamp.h>

LOG_MODULE_REGISTER(opamp_stm32, LOG_LEVEL_INF);

struct opamp_stm32_config {
	const struct stm32_pclken *pclken;
	const struct pinctrl_dev_config *pcfg;
	size_t pclk_len;
	LL_OPAMP_InitTypeDef init;
};

struct opamp_stm32_data {
	OPAMP_TypeDef *opamp;
};

static int opamp_stm32_init(const struct device *dev)
{
	struct opamp_stm32_data *data = (struct opamp_stm32_data *)dev->data;
	const struct opamp_stm32_config *cfg = (const struct opamp_stm32_config *)dev->config;
	ErrorStatus ret;
	int err;

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* TODO: we don't currently support clock selection */
	err = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			       (clock_control_subsys_t)&cfg->pclken[0]);
	if (err < 0) {
		LOG_ERR("Could not enable OPAMP clock");
		return err;
	}

	ret = LL_OPAMP_Init(data->opamp, &cfg->init);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to initialize Opamp");
		return ret;
	}

	/* Configure ADC inputs as specified in Device Tree, if any */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if ((err < 0) && (err != -ENOENT)) {
		/*
		 * If the ADC is used only with internal channels, then no pinctrl is
		 * provided in Device Tree, and pinctrl_apply_state returns -ENOENT,
		 * but this should not be treated as an error.
		 */
		LOG_ERR("ADC pinctrl setup failed (%d)", err);
		return err;
	}

	LL_OPAMP_Enable(data->opamp);
	LOG_INF("%s: enabled", dev->name);

	return 0;
}

#define STM32_OPAMP_INIT(id)                                                                       \
                                                                                                   \
	static const struct stm32_pclken pclken_##id[] = STM32_DT_INST_CLOCKS(id);                 \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(id);                                                                \
                                                                                                   \
	static const struct opamp_stm32_config opamp_stm32_cfg_##id = {                            \
		.pclken = pclken_##id,                                                             \
		.pclk_len = DT_INST_NUM_CLOCKS(id),                                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),                                        \
		.init =                                                                            \
			{                                                                          \
				.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED,                         \
				.FunctionalMode = LL_OPAMP_MODE_FUNCTIONAL,                        \
				.InputNonInverting = DT_INST_PROP_OR(id, pos_term, 0)              \
						     << OPAMP_CSR_VPSEL_Pos,                       \
				.InputInverting = DT_INST_PROP_OR(id, neg_term, 0)                 \
						  << OPAMP_CSR_VMSEL_Pos,                          \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct opamp_stm32_data opamp_stm32_data_##id = {                                   \
		.opamp = (OPAMP_TypeDef *)DT_INST_REG_ADDR(id),                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &opamp_stm32_init, NULL, &opamp_stm32_data_##id,                 \
			      &opamp_stm32_cfg_##id, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(STM32_OPAMP_INIT)
