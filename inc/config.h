#ifndef CONFIG_H
#define CONFIG_H

// ---- System clock / UART / ADC constants ----
#define SYSCLK   72000000L
#define BAUDRATE  115200L
#define SARCLK   18000000L

// ---- ADC reference voltage ----
#define VDD 3.302f // Measured VDD in volts

// ---- LCD pin assignments ----
#define LCD_RS         P1_7
// #define LCD_RW Px_x  // Not used. Connect to GND
#define LCD_E          P2_0
#define LCD_D4         P1_3
#define LCD_D5         P1_2
#define LCD_D6         P1_1
#define LCD_D7         P1_0
#define CHARS_PER_LINE 16

#endif
