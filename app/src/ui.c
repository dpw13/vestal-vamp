#include <stdio.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include "display.h"
#include "settings.h"
#include "filter.h"
#include "ifft_dma.h"
#include "math_support.h"

LOG_MODULE_REGISTER(ui, LOG_LEVEL_INF);

static const uint8_t anim_arrow[] = {
	0x16, 0x17, 0x19, 0x18,
};

static inline uint8_t animate(uint8_t i) {
	return anim_arrow[i & 0x03];
}

static inline void render_spectrum(void) {
        uint8_t bins[10];

        for (int i=0; i < 10; i++) {
                /* Translate from IFFT magnitude buffer. We start with a uint32_t
                 * that is a sum of 2**i q15_t magnitudes. Take an average by
                 * shifting right i bits. Shift another 13 bits to retain only
                 * the top 3 (linear) bits of the magnitude.
                 */
                int32_t log_pwr = arm_scalar_log_q31(ifft_mag_buf[i]);
                log_pwr = (log_pwr >> 16) + 0x6000;
                bins[i] = log_pwr >> 11;
                LOG_DBG("Mag %d = %08x | %08x", i, ifft_mag_buf[i], log_pwr);
                /* Character 0 is unusable so use a minimum value of 1 */
                bins[i] = bins[i] == 0 ? 1 : bins[i];
                bins[i] = bins[i] > 7 ? 7 : bins[i];
        }

        /* We are left with the characters to display for the spectrum */
        display_text(3, bins);
}

static char buf[10];

int ui_init(void) {
        return 0;
}

void ui_display(uint16_t frame) {
        memcpy(buf, "   V  V   ", sizeof(buf));
        buf[1] = animate(frame);
        buf[8] = animate(frame >> 2);
        display_text(0, buf);		

        int32_t log_amp = filter_get_log_amp();
        float flog_amp = (float)((10.0f/(1 << 26)) * log_amp);

        snprintf(buf, sizeof(buf), "%3.1f dB", (double)flog_amp);
        display_text(1, buf);

        // Max of about +3.5 dB (rail to rail input) 0x0180909e
        // Min of -30 dB                             0xf40491c0
        log_amp = (log_amp >> 16) + 0x1400;
        log_amp *= (0x01000000U / 0x1580U);
        int16_t bar = log_amp >> 16;
        if (bar < 0)
                bar = 0;
        if (bar > 255)
                bar = 255;
        display_bar(2, bar);

        render_spectrum();
}