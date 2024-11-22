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

#define VV_FLAG_RW              BIT(1)
#define VV_FLAG_RO              0
#define VV_FLAG_LOG_SCALE       BIT(2)
#define VV_FLAG_LIN_SCALE       0

#define VV_TYPE_MASK            (0x7 << 4)
#define VV_GET_TYPE(flags)      ((enum setting_type)((flags) & VV_TYPE_MASK) >> 4)
#define VV_SET_TYPE(t)          (((t) & 0x7) << 4)

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
        struct enum_def *def;
};

struct fxp_spec {
        int32_t min;
        int32_t max;
        uint8_t frac; /* The number of fractional bits */
};

union spec_ptr {
        struct int_spec *int_spec;
        struct float_spec *float_spec;
        struct enum_spec *enum_spec;
        struct fxp_spec *fxp_spec;
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

#define START_GROUP(_name, _count)       {.name = (_name), .setting_count = (_count), .setting = (struct setting[_count]){

#define START_RO_GROUP(_name, _count)       {.name = (_name), .setting_count = (_count), .setting = (const struct setting[_count]){

#define INT_SETTING(_name, init_val, _flags, _min, _max) { \
                                        .name = (_name), \
                                        .value = {.int_val = (init_val)}, \
                                        .flags = (_flags) | VV_SET_TYPE(VV_TYPE_INT), \
                                        .spec = { \
                                                .int_spec = &(struct int_spec){ .min = _min, .max = _max } \
                                                } \
                                        }

/**
 * Shifts the input float by frac bits and converts to integer
 */
#define FXP_SHIFT(val, frac)    (int32_t)((val) * (1 << (frac)))

#define FXP_SETTING(_name, init_val, _flags, _min, _max, _frac) { \
                                        .name = (_name), \
                                        .value = {.int_val = FXP_SHIFT(init_val, _frac)}, \
                                        .flags = (_flags) | VV_SET_TYPE(VV_TYPE_FXP), \
                                        .spec = { \
                                                .fxp_spec = &(struct fxp_spec){ .min = _min, .max = _max, .frac = _frac } \
                                                } \
                                        }

#define INT_SETTING_RO(_name, init_val) { \
                                        .name = (_name), \
                                        .value = {.int_val = (init_val)}, \
                                        .flags = VV_FLAG_RO | VV_SET_TYPE(VV_TYPE_INT), \
                                        .spec = 0 \
                                        }

#define FLOAT_SETTING(_name, init_val, _flags, _min, _max) { \
                                        .name = (_name), \
                                        .value = {.float_val = (init_val)}, \
                                        .flags = (_flags) | VV_SET_TYPE(VV_TYPE_FLOAT), \
                                        .spec = { \
                                                .float_spec = &(struct float_spec){ .min = _min, .max = _max } \
                                                } \
                                        }

#define ENUM_SETTING(_name, init_val, _flags, _spec) { \
                                        .name = (_name), \
                                        .value = {.int_val = (init_val)}, \
                                        .flags = (_flags) | VV_SET_TYPE(VV_TYPE_ENUM), \
                                        .spec = _spec \
                                        }

#define STR_SETTING(_name, init_val) { \
                                        .name = _name, \
                                        .value = {.str_val = init_val}, \
                                        .flags = VV_FLAG_RO | VV_SET_TYPE(VV_TYPE_STR), \
                                        .spec = 0, \
                                        }

#define END_GROUP()                     }}

#define START_TOP(_count)               (const struct setting_top){.group_count = _count, .group = (const struct setting_grp[_count]){
#define END_TOP()                       }}

extern const struct setting_top setting_top;

int settings_init(void);

#endif /* __SETTINGS_H__ */