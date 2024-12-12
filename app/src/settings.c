#include <zephyr/kernel.h>
#include <zephyr/app_version.h>
#include <zephyr/version.h>
#include <zephyr/logging/log.h>
#include <zephyr/retention/blinfo.h>
#include "settings.h"
#include "audio.h"
#include "freq_buffer.h"
#include "timer.h"

LOG_MODULE_REGISTER(vv_settings, LOG_LEVEL_INF);

static char blinfo_version[64];

const struct enum_spec bool_enum_spec = {
	.def_count = 2,
	.def = (const struct enum_def[2]) {
		{ .name = "OFF", .value = 0 },
		{ .name = "ON", .value = 1 },
	},
};

const struct setting_top
	setting_top = START_TOP(3) {
	START_GROUP("Vocoder", 4) {
		FXP_SETTING("Pitch Shift", 1.0, VV_FLAG_LOG_SCALE, 1, 1, 8),
		FXP_SETTING("Time Scale", 1.0, VV_FLAG_LOG_SCALE, 0, 1, 8),
		FXP_SETTING("Phase Reset Thresh", 0.001, VV_FLAG_LIN_SCALE, 0, 1, 16),
		ENUM_SETTING("Hold", 0, 0, &bool_enum_spec),
	END_GROUP() },
	START_RO_GROUP("Version", 3) {
		STR_SETTING("Bootloader", blinfo_version),
		STR_SETTING("Zephyr", STRINGIFY(BUILD_VERSION)),
		STR_SETTING("Application", APP_VERSION_STRING "-" STRINGIFY(APP_BUILD_VERSION))
	END_GROUP() },
	START_RO_GROUP("Build Opts", 5) {
		INT_SETTING_RO("Sample Rate (Hz)", AUDIO_TIMER_HZ),
		INT_SETTING_RO("Oversampling", AUDIO_OVERSAMPLE),
		INT_SETTING_RO("FFT Size", FFT_SIZE),
		INT_SETTING_RO("Overlap (%)", (100 - 100 / INV_OVERLAP)),
		INT_SETTING_RO("Window Count", WINDOW_COUNT)
	END_GROUP() },
END_TOP() };

int snprint_setting_int(char *buf, int buflen, int32_t val, const struct int_spec *spec)
{
	return snprintf(buf, buflen, "%d", val);
}

int snprint_setting_fxp(char *buf, int buflen, int32_t val, const struct fxp_spec *spec)
{
	/* For whatever reason our snprintf only accepts doubles */
	return snprintf(buf, buflen, "%f", (double)val / (1 << spec->frac));
}

int snprint_setting_float(char *buf, int buflen, float val, const struct float_spec *spec)
{
	return snprintf(buf, buflen, "%f", (double)val);
}

int snprint_setting_enum(char *buf, int buflen, uint32_t val, const struct enum_spec *spec)
{
	for (int i = 0; i < spec->def_count; i++) {
		if (val == spec->def[i].value) {
			return snprintf(buf, buflen, "%s (%d)", spec->def[i].name,
					spec->def[i].value);
		}
	}
	return snprintf(buf, buflen, "Invalid (%d)", val);
}

int snprint_setting_str(char *buf, int buflen, const char *val)
{
	return snprintf(buf, buflen, "%s", val);
}

int snprint_setting(char *buf, int buflen, const struct setting *set)
{
	enum setting_type type = VV_GET_TYPE(set->flags);

	switch (type) {
	case VV_TYPE_INT:
		return snprint_setting_int(buf, 128, set->value.int_val, set->spec.int_spec);
	case VV_TYPE_FXP:
		return snprint_setting_fxp(buf, 128, set->value.int_val, set->spec.fxp_spec);
	case VV_TYPE_FLOAT:
		return snprint_setting_float(buf, 128, set->value.float_val, set->spec.float_spec);
	case VV_TYPE_ENUM:
		return snprint_setting_enum(buf, 128, set->value.uint_val, set->spec.enum_spec);
	case VV_TYPE_STR:
		return snprint_setting_str(buf, 128, set->value.str_val);
	default:
		const char *inv = "Invalid type";
		strcpy(buf, inv);
		return strlen(inv);
	}
}

void print_setting(const struct setting *set)
{
	char buf[128];

	snprint_setting(buf, ARRAY_SIZE(buf), set);
	LOG_INF("    %s = %s", set->name, buf);
}

int settings_init(void)
{
	int len = blinfo_lookup(BLINFO_BOOTLOADER_VERSION_STR, blinfo_version,
				sizeof(blinfo_version));
	LOG_INF("Read %d bytes of bootloader version", len);

	LOG_INF("Settings:");
	for (int i = 0; i < setting_top.group_count; i++) {
		const struct setting_grp *grp = &setting_top.group[i];
		LOG_INF("  %s:", grp->name);
		for (int j = 0; j < grp->setting_count; j++) {
			const struct setting *set = &grp->setting[j];
			print_setting(set);
		}
	}
	return 0;
}