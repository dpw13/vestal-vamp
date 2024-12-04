#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <stdint.h>

/* Internal */
#define THIN_BAR_FULL  0xD0
#define THICK_BAR_FULL 0xD6
#define BAR_FULL       THICK_BAR_FULL

#define PARTIAL_BAR(n) (BAR_FULL + DISPLAY_CHAR_W - (n))

/* External */
#define DISPLAY_HRES   10
#define DISPLAY_VRES   4
#define DISPLAY_CHAR_W 5
#define DISPLAY_CHAR_H 8

void display_init(void);

void display_text(uint8_t line, const char *text);
void display_bar(uint8_t line, uint8_t lvl);
void display_init_menu(const char *title, char **options, int opt_len, uint16_t start_pos);
void display_update_menu(uint16_t pos);
uint8_t display_select_menu(void);

#endif /* __DISPLAY_H__ */