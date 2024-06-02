#ifndef CONSTANTS_H

// Definicja pinów dla sterowania silnikami

// PRAWY - 19
// PRAWY - 16
#define IN1 25          // Pin IN1 dla kanału A
#define IN2 26          // Pin IN2 dla kanału A
#define IN3 27          // Pin IN3 dla kanału B
#define IN4 14          // Pin IN4 dla kanału B
#define ENA 12          // PWM dla kanału A
#define ENB 13          // PWM dla kanału B
#define ENCODER_PIN_RIGHT 16 // Pin enkodera 1 - GPIO 0 na ESP32
#define ENCODER_PIN_LEFT 17 // Pin enkodera 2 - GPIO 2 na ESP32
#define BOTTOM_IR_B_LEFT_PIN 18
#define BOTTOM_IR_A_RIGHT_PIN 19
// Stała przeliczeniowa dla enkodera (ile milimetrów na impuls)
#define MM_PER_PULSE 1.35 // Przykładowa wartość, należy dostosować do rzeczywistego enkodera
#define PI_K_A 1.2
#define PI_I_A 0.5
#define PI_IMIN_A -200.0 / PI_I_A
#define PI_IMAX_A 200.0 / PI_I_A
#define PI_K_B 1.2
#define PI_I_B 0.5
#define PI_IMIN_B -200.0 / PI_I_B
#define PI_IMAX_B 200.0 / PI_I_B

#define CONSTANTS_H
#endif