#ifndef LCD_FSM_H
#define LCD_FSM_H

typedef enum {
    LCD_S0 = 0,  // Welcome
    LCD_S1,      // Choose Mode (Auto/Remote)
    LCD_S2,      // Choose Path (Auto only)
    LCD_S3,      // Ready (Auto)
    LCD_S4,      // Ready (Remote)
    LCD_S5,      // Running (Auto)
    LCD_S6,      // Running (Remote)
    LCD_S7,      // Pause
    LCD_NUM_STATES
} lcd_state_t;

extern lcd_state_t    lcd_state;
extern unsigned char  selected_mode;  // 0=auto, 1=remote  (navigation cursor)
extern unsigned char  selected_path;  // 0=path1, 1=path2, 2=path3  (navigation cursor)
extern unsigned char  active_mode;    // locked in at start: 0=auto, 1=remote
extern unsigned char  active_path;    // locked in at start: 0=path1, 1=path2, 2=path3

void LCD_FSM_init(void);
void LCD_FSM_update(unsigned char x_byte, unsigned char y_byte);

#endif
