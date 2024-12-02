#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

int timer_init(void);
int timer_start(void);
int timer_stop(void);
int timer_stats(void);

#endif /* __TIMER_H__ */