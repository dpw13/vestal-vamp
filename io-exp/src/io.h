#ifndef __IO_H__
#define __IO_H__

/* MISO */
#define PORTA_DIR_VAL   (_BV(2))
/* TXD */
#define PORTB_DIR_VAL   (_BV(2))
/* LEDs */
#define PORTC_DIR_VAL   (_BV(0) | _BV(1))

#define DI_PINCTRL  (PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc)
#define AI_PINCTRL  (PORT_ISC_INPUT_DISABLE_gc)

#define QUAD_COUNT  2

extern uint16_t quad_pos[QUAD_COUNT];
extern uint8_t quad_skip[QUAD_COUNT];
extern uint8_t io_update;

void io_init(void);
void io_print_state(void);

#endif /*__IO_H__*/