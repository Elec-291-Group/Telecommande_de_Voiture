#include <stdio.h>
#include "lcd_fsm.h"
#include "lcd.h"
#include "config.h"

// ---- Joystick thresholds ----
#define JOY_HIGH     200
#define JOY_LOW       50
#define JOY_MID_LO   130
#define JOY_MID_HI   180

// ---- Public state ----
lcd_state_t   lcd_state    = LCD_S0;
unsigned char selected_mode = 0;   // 0=field tracking, 1=remote, 2=path tracking  (navigation cursor)
unsigned char selected_path = 0;   // 0=path1, 1=path2, 2=path3  (navigation cursor)
unsigned char active_mode   = 0;   // locked in when running starts: 0=field tracking, 1=remote, 2=path tracking
unsigned char active_path   = 0;   // locked in when running starts: 0=path1, 1=path2, 2=path3

// ---- Internal ----
static lcd_state_t prev_state = LCD_NUM_STATES; // force redraw on first call

// Joystick latches: must return to center before next trigger
static bit joy_x_ready = 1;
static bit joy_y_ready = 1;

// Button latches: must release before next press registers
static bit pb0_ready = 1;
static bit pb1_ready = 1;

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
                LCDprint(">Fld  Rem  Path ", 2, 0);
            else if (selected_mode == 1)
                LCDprint(" Fld >Rem  Path ", 2, 0);
            else
                LCDprint(" Fld  Rem >Path ", 2, 0);
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
            LCDprint("Ready (Field)   ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S4:
            LCDprint("Ready (Remote)  ", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S8:
            LCDprint("Ready (Pathfind)", 1, 0);
            LCDprint("Press PB0       ", 2, 0);
            break;

        case LCD_S5:
            LCDprint("Running (Field) ", 1, 0);
            LCDprint("PB1: Pause      ", 2, 0);
            break;

        case LCD_S6:
            LCDprint("Running (Remote)", 1, 0);
            LCDprint("                ", 2, 0);
            break;

        case LCD_S9:
            LCDprint("Running (Path)  ", 1, 0);
            LCDprint("PB1: Pause      ", 2, 0);
            break;

        case LCD_S7:
            LCDprint("Paused          ", 1, 0);
            LCDprint("PB0:Go  PB1:Rst ", 2, 0);
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
    char buf[17];
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
                if      (selected_mode == 0) lcd_state = LCD_S2;
                else if (selected_mode == 1) lcd_state = LCD_S4;
                else                         lcd_state = LCD_S8;
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

        case LCD_S5:
            if (pb1_pressed) lcd_state = LCD_S7;
            break;

        case LCD_S6:
            if (pb1_pressed) lcd_state = LCD_S7;
            break;

        case LCD_S8:
            if (pb0_pressed) { active_mode = selected_mode; active_path = selected_path; lcd_state = LCD_S9; }
            if (joy_up)      lcd_state = LCD_S1;
            break;

        case LCD_S9:
            if (pb1_pressed) lcd_state = LCD_S7;
            break;

        case LCD_S7:
            if (pb0_pressed) {
                if      (active_mode == 0) lcd_state = LCD_S5;
                else if (active_mode == 1) lcd_state = LCD_S6;
                else                       lcd_state = LCD_S9;
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
        sprintf(buf, "X:%3u Y:%3u     ", (unsigned int)x_byte, (unsigned int)y_byte);
        LCDprint(buf, 2, 0);
    }
}
