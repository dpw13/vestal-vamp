#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/auxdisplay.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "display.h"

#define DISP_STACK_SIZE 128
#define DISP_PRIORITY 5

LOG_MODULE_REGISTER(vv_display, LOG_LEVEL_INF);

static const struct device *const aux_dev = DEVICE_DT_GET(DT_NODELABEL(auxdisplay_0));

#define MENU_BUF_LEN    128
#define MENU_MAX_OPTS   16

static char menu_buf[MENU_BUF_LEN+1];
static uint8_t menu_opt_offset[MENU_MAX_OPTS];
static uint8_t menu_opt_count;
static uint8_t menu_opt_selected;
static int8_t menu_opt_max_scroll;

static const char menu_opt_sep[] = " | ";

typedef struct {
    uint8_t line;
    uint8_t chars[DISPLAY_HRES];
} disp_action_t;

#define DISP_ACTION_ARRAY_SIZE    4

/* Define the message queue */
K_MSGQ_DEFINE(display_action_msgq,
        sizeof(disp_action_t),
        DISP_ACTION_ARRAY_SIZE,
        4);

void display_action_work_handler(struct k_work *work);

/* Define the work handler */
K_WORK_DEFINE(display_action_work, display_action_work_handler);

static inline void _display_queue(disp_action_t *action, bool start) {
    int ret;

    LOG_DBG("Queue write to line %d", action->line);
    if (action->line >= DISPLAY_VRES) {
        LOG_ERR("Invalid line %d", action->line);
        return;
    }
    ret = k_msgq_put(&display_action_msgq, action, K_NO_WAIT);
    if (ret < 0) {
        LOG_ERR("Error pushing message queue: %d", ret);
        /* Still allow the worker to be submitted as that may clear the message queue */
    }

    if (start) {
        ret = k_work_submit(&display_action_work);
        if (ret < 0) {
            LOG_ERR("Error submitting work: %d", ret);
        }
    }
}

/* Work handler to process display updates */
void display_action_work_handler(struct k_work *work) {
    disp_action_t action;
    int rc;

    /* Process everything in the message queue */
    while (k_msgq_num_used_get(&display_action_msgq)) {
        k_msgq_get(&display_action_msgq, &action, K_NO_WAIT);
        LOG_DBG("Got display on line %d", action.line);

        rc = auxdisplay_cursor_position_set(aux_dev, AUXDISPLAY_POSITION_ABSOLUTE, 0, action.line);
        if (rc != 0) {
            LOG_ERR("Failed to set cursor position to line %d: %d", action.line, rc);
        }

        /* Always write the entire line */
        rc = auxdisplay_write(aux_dev, &action.chars[0], DISPLAY_HRES);
        if (rc != 0) {
            LOG_ERR("Failed to write to display: %d", rc);
        }

        /* Give the schedule a chance to run other tasks */
        k_yield();
    }
}

/**
 * Use a single line to display a bar graph to indicate a level.
 * @param line The line to display the bar on
 * @param lvl The level to display, between 0 and 255.
 */
void display_bar(uint8_t line, uint8_t lvl) {
    disp_action_t action;
    /*
     * Calculate the number of columns to darken using fixed-point math.
     * The number of columns will be in the upper byte of this word. Note
     * the rounding by adding (1 << 7).
     */
    const uint16_t tmp = (uint16_t)lvl * DISPLAY_CHAR_W * DISPLAY_HRES + 0x80;
    /* The number of columns to darken. */
    uint8_t cols = (uint8_t)(tmp >> 8);
    action.line = line;

    memset(&action.chars[0], ' ', DISPLAY_HRES);
    uint8_t *ptr = &action.chars[0];
    for (int col=0; col < DISPLAY_CHAR_W * DISPLAY_HRES; col += DISPLAY_CHAR_W) {
        if (cols == 0) {
            break;
        } else if (cols <= DISPLAY_CHAR_W) {
            /* Display one of the partial characters */
            *ptr++ = PARTIAL_BAR(cols);
            cols = 0;
            break;
        } else {
            /* Display full character */
            *ptr++ = BAR_FULL;
            cols -= DISPLAY_CHAR_W;
        }
    }

    _display_queue(&action, true);
}

void display_text(uint8_t line, const char *text) {
    disp_action_t action;
    int len = strlen(text);

    if (len > DISPLAY_HRES) {
        len = DISPLAY_HRES;
    }
    LOG_DBG("Displaying on line %d: %s", line, text);
    action.line = line;
    memcpy(&action.chars[0], text, len);
    if (len < DISPLAY_HRES) {
        /* Pad with spaces */
        memset(&action.chars[len], ' ', DISPLAY_HRES - len);
    }

    _display_queue(&action, true);
}

/**
 * Display a new menu
 * @param title The title of the menu, displayed on the top line
 * @param options An array of menu options
 * @param start_pos The starting position to select, from 0 to 65535
 */
void display_init_menu(const char *title, char **options, int opt_len, uint16_t start_pos) {
    disp_action_t action;
    int buf_len = sizeof(menu_buf);
    char *ptr = &menu_buf[0];

    action.line = 0;
    strncpy(&action.chars[0], title, DISPLAY_HRES);
    /* Display title but do not start worker yet */
    _display_queue(&action, false);

    if (opt_len > MENU_MAX_OPTS) {
        LOG_ERR("Number of menu options (%d) exceeds max", opt_len);
        opt_len = MENU_MAX_OPTS;
    }
    menu_opt_count = opt_len;
    for (int i = 0; i < opt_len; i++) {
        int len = strlen(options[i]);
        /* Don't exceed total buffer space */
        if (len > buf_len) {
            len = buf_len;
        }
        /* Copy option text into menu string */
        memcpy(ptr, options[i], len);

        ptr += len;
        buf_len -= len;

        /* Re-use variable */
        len = sizeof(menu_opt_sep);
        if (buf_len < len) {
            LOG_ERR("Menu options truncated at option %d", i);
            menu_opt_count = i;
            break;
        }

        if (i < (opt_len - 1)) {
            /* Add separator */
            memcpy(ptr, menu_opt_sep, len);
            /* Record position of end of option including separator */
            menu_opt_offset[i] = (ptr - options[i]) + 2;
            ptr += len;
            buf_len -= len;
        } else {
            /* Do not add separator on last entry */
            menu_opt_offset[i] = (ptr - options[i]);
        }
    }
    /* This is the maximum amount of movement that the options can scroll */
    menu_opt_max_scroll = (ptr - &menu_buf[0]) - DISPLAY_HRES;
    /* Null-terminate string */
    *ptr = 0;

    /* Render the options at the start position */
    display_update_menu(start_pos);
}

/**
 * Update the current position for selection view
 * @param pos The position to select, from 0 to 65535
 *
 * @todo highlight or accentuate selected option
 */
void display_update_menu(uint16_t pos) {
    disp_action_t action;

    action.line = 1;

    /* Calculate the amount of scroll from current position */
    uint32_t fxp_offs = (int32_t)pos * (-menu_opt_max_scroll) + 0x8000U;
    /* Current scroll amount starts at bit 16 */
    int16_t offset = fxp_offs >> 16;
    if (offset < 0) {
        /* Pad option string into center of display */
        memset(&action.chars[0], ' ', DISPLAY_HRES);
        strncpy(&action.chars[DISPLAY_HRES + offset], &menu_buf[0], DISPLAY_HRES);
    } else {
        /* Render substring visible in window */
        strncpy(&action.chars[0], &menu_buf[offset], DISPLAY_HRES);
    }
    /* Re-use offset variable as character position of middle of window */
    offset += DISPLAY_HRES/2;
    /* Find option associated with that position */
    for (int i = 0; i < menu_opt_count; i++) {
        /* Find the first option with its end greater than the current position */
        if (offset < menu_opt_offset[i]) {
            menu_opt_selected = i;
            break;
        }
    }

    /* Queue update and start worker */
    _display_queue(&action, true);
}

/**
 * Select the current menu option.
 * @return The index of the currently selected item
 *
 * @todo Any kind of selection animation
 */
uint8_t display_select_menu(void) {
    return menu_opt_selected;
}

void display_init(void) {
    int rc;

	if (!device_is_ready(aux_dev)) {
		LOG_ERR("Auxdisplay device is not ready.");
		return;
	}

    /*
    auxdisplay_position_blinking_set_enabled(aux_dev, true);
	rc = auxdisplay_cursor_set_enabled(aux_dev, true);
	if (rc != 0) {
		LOG_ERR("Failed to enable cursor: %d", rc);
		return;
	}

    rc = auxdisplay_display_on(aux_dev);
	if (rc != 0) {
		LOG_ERR("Failed to turn on display: %d", rc);
		return;
	}

    rc = auxdisplay_clear(aux_dev);
	if (rc != 0) {
		LOG_ERR("Failed to clear display: %d", rc);
		return;
	}
    */

    /* Initialize state variables */
    menu_opt_count = 0;
}