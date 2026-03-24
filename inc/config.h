#ifndef CONFIG_H
#define CONFIG_H

// ---- System clock / UART / ADC constants ----
#define SYSCLK   72000000L
#define BAUDRATE  115200L
#define SARCLK   18000000L

// ---- ADC reference voltage ----
#define VDD 4.637f // Measured VDD in volts

/*
// ---- LCD pin assignments ----
#define LCD_RS         P1_7
// #define LCD_RW Px_x  // Not used. Connect to GND
#define LCD_E          P2_0
#define LCD_D4         P1_3
#define LCD_D5         P1_2
#define LCD_D6         P1_1
#define LCD_D7         P1_0
*/

// ---- IR Command Name Nibbles (4-bit, address always 0xB) ----
#define IR_CMD_START      0x0   // data = 0xFF
#define IR_CMD_PAUSE      0x1   // data = 0xFF
#define IR_CMD_RESET      0x2   // data = 0xFF
#define IR_CMD_MODE       0x3   // data: 0x00=auto, 0x01=remote
#define IR_CMD_PATH       0x4   // data: 0x00=path1, 0x01=path2, 0x02=path3
#define IR_CMD_JOYSTICK_X 0x5   // data = joystick_x byte
#define IR_CMD_JOYSTICK_Y 0x6   // data = joystick_y byte

#define IR_ADDR           0xB   // device address nibble

#define CHARS_PER_LINE 16
#define JOYSTICK_X     QFP32_MUX_P1_5      // P1.5 = ADC0MX channel 13
#define JOYSTICK_Y     QFP32_MUX_P1_4      // P1.4 = ADC0MX channel 12
#define JoyStick_SW    P2_3
#define IR_IN          P2_1


#endif
