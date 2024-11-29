#include <zephyr/logging/log.h>
#include <stm32_ll_opamp.h>
#include "opamp.h"

LOG_MODULE_REGISTER(opamp, LOG_LEVEL_INF);

/* Both opamps configured as trivial followers */
const static LL_OPAMP_InitTypeDef op1_init = {
        .PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED,
        .FunctionalMode = LL_OPAMP_MODE_FOLLOWER,
        .InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO0, /* PB0 */
        .InputInverting = LL_OPAMP_INPUT_INVERT_CONNECT_NO, /* direct connection to Vout */
        /* output: PC4 = adc1_inp4_pc4 */
};

/* OP2 driven by DAC1 ch2 */
const static LL_OPAMP_InitTypeDef op2_init = {
        .PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED,
        .FunctionalMode = LL_OPAMP_MODE_FOLLOWER,
        .InputNonInverting = LL_OPAMP_INPUT_NONINVERT_DAC, /* internal, but maybe dac1_out2_pa5? */
        .InputInverting = LL_OPAMP_INPUT_INVERT_CONNECT_NO,
        /* output: PE7 */
};

int opamp_init(void) {
        ErrorStatus ret;
        
        ret = LL_OPAMP_Init(OPAMP1, &op1_init);
        if (ret != SUCCESS)
                LOG_ERR("Failed to configure opamp1");
        ret = LL_OPAMP_Init(OPAMP2, &op2_init);
        if (ret != SUCCESS)
                LOG_ERR("Failed to configure opamp2");

        return 0;
}