#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <stdint.h>
#include "audio.h"

enum setting_type {
	VV_TYPE_INT = 0,
	VV_TYPE_FLOAT = 1,
	VV_TYPE_ENUM = 2,
	VV_TYPE_STR = 3,
	VV_TYPE_FXP = 4,
};

#define VV_FLAG_RW        BIT(1)
#define VV_FLAG_RO        0
#define VV_FLAG_LOG_SCALE BIT(2)
#define VV_FLAG_LIN_SCALE 0

#define VV_TYPE_MASK       (0x7 << 4)
#define VV_GET_TYPE(flags) ((enum setting_type)(((flags) & VV_TYPE_MASK) >> 4))
#define VV_SET_TYPE(t)     (((t) & 0x7) << 4)

union setting_value {
	uint32_t uint_val;
	int32_t int_val;
	float float_val;
	const char *str_val;
};

struct float_spec {
	float min;
	float max;
};

struct int_spec {
	int32_t min;
	int32_t max;
};

struct enum_def {
	const char *name;
	uint32_t value;
};

struct enum_spec {
	uint32_t def_count;
	const struct enum_def *def;
};

struct fxp_spec {
	int32_t min;
	int32_t max;
	uint8_t frac; /* The number of fractional bits */
};

union spec_ptr {
	const struct int_spec *int_spec;
	const struct float_spec *float_spec;
	const struct enum_spec *enum_spec;
	const struct fxp_spec *fxp_spec;
};

struct setting {
	const char *name;
	union setting_value value;
	union spec_ptr spec;
	uint16_t flags;
};

struct setting_grp {
	const char *name;
	uint32_t setting_count;
	const struct setting *setting;
};

struct setting_top {
	uint32_t group_count;
	const struct setting_grp *group;
};

#define START_GROUP(_name, _count)                                                                 \
	{                                                                                          \
		.name = (_name), .setting_count = (_count), .setting = (struct setting[_count])

#define START_RO_GROUP(_name, _count)                                                              \
	{                                                                                          \
		.name = (_name), .setting_count = (_count),                                        \
		.setting = (const struct setting[_count])

#define INT_SETTING(_name, init_val, _flags, _min, _max)                                           \
	{                                                                                          \
		.name = (_name), .value = {.int_val = (init_val)},                                 \
		.flags = (_flags) | VV_FLAG_RW | VV_SET_TYPE(VV_TYPE_INT), .spec = {                            \
			.int_spec = &(struct int_spec){.min = _min, .max = _max}                   \
		}                                                                                  \
	}

/**
 * Shifts the input float by frac bits and converts to integer
 */
#define FXP_SHIFT(val, frac) (int32_t)((val) * (1 << (frac)))

#define FXP_SETTING(_name, init_val, _flags, _min, _max, _frac)                                    \
	{                                                                                          \
		.name = (_name), .value = {.int_val = FXP_SHIFT(init_val, _frac)},                 \
		.flags = (_flags) | VV_FLAG_RW | VV_SET_TYPE(VV_TYPE_FXP), .spec = {                            \
			.fxp_spec = &(struct fxp_spec){.min = FXP_SHIFT(_min, _frac), .max = FXP_SHIFT(_max, _frac), .frac = _frac}    \
		}                                                                                  \
	}

#define INT_SETTING_RO(_name, init_val)                                                            \
	{                                                                                          \
		.name = (_name), .value = {.int_val = (init_val)},                                 \
		.flags = VV_FLAG_RO | VV_SET_TYPE(VV_TYPE_INT), .spec = 0                          \
	}

#define FLOAT_SETTING(_name, init_val, _flags, _min, _max)                                         \
	{                                                                                          \
		.name = (_name), .value = {.float_val = (init_val)},                               \
		.flags = (_flags) | VV_FLAG_RW | VV_SET_TYPE(VV_TYPE_FLOAT), .spec = {                          \
			.float_spec = &(struct float_spec){.min = _min, .max = _max}               \
		}                                                                                  \
	}

#define ENUM_SETTING(_name, init_val, _flags, _spec)                                               \
	{                                                                                          \
		.name = (_name), .value = {.int_val = (init_val)},                                 \
		.flags = (_flags) | VV_FLAG_RW | VV_SET_TYPE(VV_TYPE_ENUM), .spec = {.enum_spec = _spec },                       \
	}

#define STR_SETTING(_name, init_val)                                                               \
	{                                                                                          \
		.name = _name, .value = {.str_val = init_val},                                     \
		.flags = VV_FLAG_RO | VV_SET_TYPE(VV_TYPE_STR), .spec = 0,                         \
	}

#define END_GROUP()                                                                                \
	}

#define START_TOP(_count)                                                                          \
	(const struct setting_top)                                                                 \
	{                                                                                          \
		.group_count = _count, .group = (const struct setting_grp[_count])
#define END_TOP()                                                                                  \
	}

extern const struct setting_top setting_top;

#define INST_SETTING_ACCESSOR(_type, _name) \
	static inline _type get_ ## _name ## _setting(uint16_t pack_idx) { \
		int grp = pack_idx >> 8; \
		int idx = pack_idx & 0xFF; \
		return setting_top.group[grp].setting[idx].value._name ## _val; \
	} \
	\
	static inline void set_ ## _name ## _setting(uint16_t pack_idx, _type val) { \
		int grp = pack_idx >> 8; \
		int idx = pack_idx & 0xFF; \
		/* Explicitly discard const */ \
		struct setting *s = (struct setting *)&setting_top.group[grp].setting[idx]; \
		if (s->flags & VV_FLAG_RW) { \
			s->value._name ## _val = val; \
		} \
	}

INST_SETTING_ACCESSOR(uint32_t, uint)
INST_SETTING_ACCESSOR(int32_t, int)
INST_SETTING_ACCESSOR(float, float)
INST_SETTING_ACCESSOR(const char *, str)


#define SETTINGS_IDX(_grp, _idx)	(((uint16_t)_grp << 8) | ((uint16_t)_idx & 0xFF))
#define SETTINGS_VOCODER_PITCH_SHIFT		SETTINGS_IDX(0, 0)
#define SETTINGS_VOCODER_TIME_SCALE		SETTINGS_IDX(0, 1)
#define SETTINGS_VOCODER_PHASE_RESET_THRESH	SETTINGS_IDX(0, 2)
#define SETTINGS_VOCODER_HOLD			SETTINGS_IDX(0, 3)

int settings_init(void);

#endif /* __SETTINGS_H__ */