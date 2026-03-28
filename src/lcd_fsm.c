#include "lcd_fsm.h"
#include "lcd.h"
#include "config.h"
#include "data_buffers.h"

// ---- Joystick thresholds ----
#define JOY_HIGH     200
#define JOY_LOW       50
#define JOY_MID_LO   130
#define JOY_MID_HI   180

// ---- Public state ----
lcd_state_t   lcd_state    = LCD_S0;
unsigned char selected_mode = 0;   // 0=auto, 1=remote, 2=pathfind
unsigned char selected_path = 0;   // 0=path1, 1=path2, 2=path3  (navigation cursor)
unsigned char active_mode   = 0;   // locked in when running starts: 0=auto, 1=remote, 2=pathfind
unsigned char active_path   = 0;   // locked in when running starts: 0=path1, 1=path2, 2=path3

// ---- Internal ----
static lcd_state_t prev_state = LCD_NUM_STATES; // force redraw on first call

// Joystick latches: must return to center before next trigger
static bit joy_x_ready = 1;
static bit joy_y_ready = 1;

// Button latches: must release before next press registers
static bit pb0_ready = 1;
static bit pb1_ready = 1;
static Waypoint_t xdata path_test_waypoint;
static unsigned char path_test_count = 0;
static char xdata lcd_line_buf[17];

static void format_joystick_status(char *buf, unsigned char x_byte, unsigned char y_byte)
{
    buf[0] = 'X';
    buf[1] = ':';
    buf[2] = (char)('0' + (x_byte / 100u));
    buf[3] = (char)('0' + ((x_byte / 10u) % 10u));
    buf[4] = (char)('0' + (x_byte % 10u));
    buf[5] = ' ';
    buf[6] = 'Y';
    buf[7] = ':';
    buf[8] = (char)('0' + (y_byte / 100u));
    buf[9] = (char)('0' + ((y_byte / 10u) % 10u));
    buf[10] = (char)('0' + (y_byte % 10u));
    buf[11] = ' ';
    buf[12] = ' ';
    buf[13] = ' ';
    buf[14] = ' ';
    buf[15] = ' ';
    buf[16] = '\0';
}

static void format_uint3(char *buf, unsigned int value)
{
    if (value > 999u)
        value = 999u;

    buf[0] = (char)('0' + (value / 100u));
    buf[1] = (char)('0' + ((value / 10u) % 10u));
    buf[2] = (char)('0' + (value % 10u));
}

static void format_coord4(char *buf, int value)
{
    unsigned int magnitude;

    if (value < 0) {
        buf[0] = '-';
        magnitude = (unsigned int)(-value);
    } else {
        buf[0] = '+';
        magnitude = (unsigned int)value;
    }

    if (magnitude > 999u)
        magnitude = 999u;

    buf[1] = (char)('0' + (magnitude / 100u));
    buf[2] = (char)('0' + ((magnitude / 10u) % 10u));
    buf[3] = (char)('0' + (magnitude % 10u));
}

static void format_path_buffer_status(char *buf)
{
    if (!PathBuffer_is_loaded()) {
        LCDprint("Path:not loaded ", 1, 0);
        LCDprint("Wait PATH_END   ", 2, 0);
        return;
    }

    path_test_count = PathBuffer_get_count();
    if ((path_test_count == 0u) || !PathBuffer_get(0u, &path_test_waypoint)) {
        LCDprint("Path N:000      ", 1, 0);
        LCDprint("No waypoint     ", 2, 0);
        return;
    }

    buf[0] = 'P';
    buf[1] = 'a';
    buf[2] = 't';
    buf[3] = 'h';
    buf[4] = ' ';
    buf[5] = 'N';
    buf[6] = ':';
    format_uint3(&buf[7], path_test_count);
    buf[10] = ' ';
    buf[11] = ' ';
    buf[12] = ' ';
    buf[13] = ' ';
    buf[14] = ' ';
    buf[15] = ' ';
    buf[16] = '\0';
    LCDprint(buf, 1, 0);

    buf[0] = 'X';
    buf[1] = ':';
    format_coord4(&buf[2], path_test_waypoint.x_cm);
    buf[6] = ' ';
    buf[7] = 'Y';
    buf[8] = ':';
    format_coord4(&buf[9], path_test_waypoint.y_cm);
    buf[13] = ' ';
    buf[14] = ' ';
    buf[15] = ' ';
    buf[16] = '\0';
    LCDprint(buf, 2, 0);
}

// ---- Redraw current state to LCD ----
static void redraw(void)
{
    switch (lcd_state)
    {
        case LCD_S0:
            LCDprint("Welcome!        ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S1:
            LCDprint("Choose Mode:    ", 1, 0);
            if (selected_mode == 0)
                LCDprint(">Auto Rem Path  ", 2, 0);
            else if (selected_mode == 1)
                LCDprint(" Auto>Rem Path  ", 2, 0);
            else
                LCDprint(" Auto Rem>Path  ", 2, 0);
            break;

        case LCD_S2:
            LCDprint("Choose Path:    ", 1, 0);
            if (selected_path == 0)
                LCDprint(">P1  P2  P3     ", 2, 0);
            else if (selected_path == 1)
                LCDprint(" P1 >P2  P3     ", 2, 0);
            else
                LCDprint(" P1  P2 >P3     ", 2, 0);
            break;

        case LCD_S3:
            LCDprint("Ready (Auto)    ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S4:
            LCDprint("Ready (Remote)  ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S5:
            LCDprint("Running (Auto)  ", 1, 0);
            LCDprint("PB1: Pause      ", 2, 0);
            break;

        case LCD_S6:
            LCDprint("Running (Remote)", 1, 0);
            LCDprint("                ", 2, 0);
            break;

        case LCD_S7:
            LCDprint("Paused          ", 1, 0);
            LCDprint("PB0:Go  PB1:Rst ", 2, 0);
            break;

        case LCD_S8:
            LCDprint("Ready (Path)    ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S9:
            LCDprint("Path test mode  ", 1, 0);
            LCDprint("Wait PATH_END   ", 2, 0);
            break;

        default:
            break;
    }
}

// ---- Init ----
void LCD_FSM_init(void)
{
    lcd_state     = LCD_S0;
    prev_state    = LCD_NUM_STATES;
    selected_mode = 0;
    selected_path = 0;
    active_mode   = 0;
    active_path   = 0;
    joy_x_ready   = 1;
    joy_y_ready   = 1;
    pb0_ready     = 1;
    pb1_ready     = 1;
}

// ---- Update — call every main loop iteration ----
void LCD_FSM_update(unsigned char x_byte, unsigned char y_byte)
{
    unsigned char pb0_pressed = 0;
    unsigned char pb1_pressed = 0;
    unsigned char joy_up      = 0;
    unsigned char joy_down    = 0;
    unsigned char joy_right   = 0;
    unsigned char joy_left    = 0;
    unsigned char need_redraw = 0;

    // ---- Read buttons (active low, latch until release) ----
    if (JoyStick_SW == 0) {
        if (pb0_ready) { pb0_pressed = 1; pb0_ready = 0; }
    } else {
        pb0_ready = 1;
    }

    if (PB1 == 0) {
        if (pb1_ready) { pb1_pressed = 1; pb1_ready = 0; }
    } else {
        pb1_ready = 1;
    }

    // ---- Read joystick (latch until returns to center) ----
    if (x_byte > JOY_HIGH && joy_x_ready) { joy_up   = 1; joy_x_ready = 0; }
    if (x_byte < JOY_LOW  && joy_x_ready) { joy_down = 1; joy_x_ready = 0; }
    if (x_byte >= JOY_MID_LO && x_byte <= JOY_MID_HI) joy_x_ready = 1;

    if (y_byte > JOY_HIGH && joy_y_ready) { joy_right = 1; joy_y_ready = 0; }
    if (y_byte < JOY_LOW  && joy_y_ready) { joy_left  = 1; joy_y_ready = 0; }
    if (y_byte >= JOY_MID_LO && y_byte <= JOY_MID_HI) joy_y_ready = 1;

    // ---- FSM transitions ----
    switch (lcd_state)
    {
        case LCD_S0:
            if (pb0_pressed) lcd_state = LCD_S1;
            break;

        case LCD_S1:
            if (joy_right && selected_mode < 2) { selected_mode++; need_redraw = 1; }
            if (joy_left  && selected_mode > 0) { selected_mode--; need_redraw = 1; }
            if (pb0_pressed) {
                if (selected_mode == 0)
                    lcd_state = LCD_S2;
                else if (selected_mode == 1)
                    lcd_state = LCD_S4;
                else
                    lcd_state = LCD_S8;
            }
            break;

        case LCD_S2:
            if (joy_right && selected_path < 2) { selected_path++; need_redraw = 1; }
            if (joy_left  && selected_path > 0) { selected_path--; need_redraw = 1; }
            if (pb0_pressed) lcd_state = LCD_S3;
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S3:
            if (pb0_pressed) { active_mode = selected_mode; active_path = selected_path; lcd_state = LCD_S5; }
            if (joy_up)      lcd_state = LCD_S2;
            break;

        case LCD_S4:
            if (pb0_pressed) { active_mode = selected_mode; active_path = selected_path; lcd_state = LCD_S6; }
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S8:
            if (pb0_pressed) { active_mode = selected_mode; active_path = selected_path; lcd_state = LCD_S9; }
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S5:
            if (pb1_pressed) lcd_state = LCD_S7;
            break;

        case LCD_S6:
            if (pb1_pressed) lcd_state = LCD_S7;
            break;

        case LCD_S9:
            if (pb1_pressed) lcd_state = LCD_S7;
            break;

        case LCD_S7:
            if (pb0_pressed) {
                if (selected_mode == 0)
                    lcd_state = LCD_S5;
                else if (selected_mode == 1)
                    lcd_state = LCD_S6;
                else
                    lcd_state = LCD_S9;
            }
            if (pb1_pressed) {
                selected_mode = 0;
                selected_path = 0;
                lcd_state = LCD_S0;
            }
            break;

        default:
            break;
    }

    // ---- Redraw ----
    if (lcd_state != prev_state) {
        redraw();
        prev_state = lcd_state;
    } else if (need_redraw) {
        redraw();
    }

    // Live joystick update in S6
    if (lcd_state == LCD_S6 && prev_state == LCD_S6) {
        format_joystick_status(lcd_line_buf, x_byte, y_byte);
        LCDprint(lcd_line_buf, 2, 0);
    }

    if (lcd_state == LCD_S9 && prev_state == LCD_S9) {
        format_path_buffer_status(lcd_line_buf);
    }
}
