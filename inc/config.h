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

// ---- IR addresses ----
#define IR_ADDR1           0x6   // EFM8 TX address (received by STM32)
#define IR_ADDR2           0x7   // STM32 TX address (received by EFM8)
#define IR_ADDR            IR_ADDR1

// ---- IR Command bytes (EFM8 TX → STM32 RX), address = IR_ADDR1 ----
#define IR_CMD_START      0   // val = 0x0000
#define IR_CMD_PAUSE      1   // val = 0x0000
#define IR_CMD_RESET      2   // val = 0x0000
#define IR_CMD_MODE       3   // val: 0x0000=auto, 0x0001=remote, 0x0002=pathfind
#define IR_CMD_PATH       4   // val: 0x0001=path1, 0x0002=path2, 0x0003=path3
#define IR_CMD_JOYSTICK_X 5   // val = joystick_x byte
#define IR_CMD_JOYSTICK_Y 6   // val = joystick_y byte

// ---- IMU register array indices (0-17 → imu_regs[]) ----
// Command byte sent over IR = IMU_CMD_BASE + index
#define IMU_CMD_BASE        7   /* command byte offset for IMU registers */
#define IMU_REG_ACCEL_X     0   /* Raw accelerometer X   → cmd 7        */
#define IMU_REG_ACCEL_Y     1   /* Raw accelerometer Y   → cmd 8        */
#define IMU_REG_ACCEL_Z     2   /* Raw accelerometer Z   → cmd 9        */
#define IMU_REG_GYRO_X      3   /* Raw gyroscope X       → cmd 10       */
#define IMU_REG_GYRO_Y      4   /* Raw gyroscope Y       → cmd 11       */
#define IMU_REG_GYRO_Z      5   /* Raw gyroscope Z       → cmd 12       */
#define IMU_REG_MAG_X       6   /* Magnetometer X        → cmd 13       */
#define IMU_REG_MAG_Y       7   /* Magnetometer Y        → cmd 14       */
#define IMU_REG_MAG_Z       8   /* Magnetometer Z        → cmd 15       */
#define IMU_REG_EULER_H     9   /* Euler heading (yaw)   → cmd 16       */
#define IMU_REG_EULER_R     10  /* Euler roll            → cmd 17       */
#define IMU_REG_EULER_P     11  /* Euler pitch           → cmd 18       */
#define IMU_REG_QUAT_W      12  /* Quaternion W          → cmd 19       */
#define IMU_REG_QUAT_X      13  /* Quaternion X          → cmd 20       */
#define IMU_REG_QUAT_Y      14  /* Quaternion Y          → cmd 21       */
#define IMU_REG_QUAT_Z      15  /* Quaternion Z          → cmd 22       */
#define IMU_REG_LIN_ACCEL_X 16  /* Linear accel X        → cmd 23       */
#define IMU_REG_LIN_ACCEL_Y 17  /* Linear accel Y        → cmd 24       */
#define IMU_REG_COUNT       18

#define CHARS_PER_LINE 16
#define JOYSTICK_X     QFP32_MUX_P1_5      // P1.5 = ADC0MX channel 13
#define JOYSTICK_Y     QFP32_MUX_P1_4      // P1.4 = ADC0MX channel 12
#define JoyStick_SW    P2_3
#define IR_IN          P0_7
#define PB1            P3_0


#endif
