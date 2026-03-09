# STM32F722 Hardware Pin & Peripheral Usage

## Timers

| Timer / Source              | Function                                        | Frequency       |
|-----------------------------|-------------------------------------------------|-----------------|
| SysTick                     | Timer, delays, basic timing                     | 1000 Hz         |
| TIM7 (`TIM7_IRQHandler`)   | Motor FOC control loop (incl. I2C encoder read) | 2000 Hz         |
| TIM8/TIM13 (`TIM8_UP_TIM13_IRQHandler`) | Sensor sampling (line sensor, proximity) | 200 – 400 Hz |

---

## Turbine Fan PWM

| Signal | Pin  | Peripheral |
|--------|------|------------|
| PWM    | PB10 | TIM2_CH3   |

---

## Left Motor – 3-Phase BLDC PWM (TIM3)

| Signal | Pin | Peripheral |
|--------|-----|------------|
| PWMA   | PC6 | TIM3_CH1   |
| PWMB   | PC7 | TIM3_CH2   |
| PWMC   | PC8 | TIM3_CH3   |
| Enable | PC9 | GPIO       |

## Right Motor – 3-Phase BLDC PWM (TIM4)

| Signal | Pin | Peripheral |
|--------|-----|------------|
| PWMA   | PB6 | TIM4_CH1   |
| PWMB   | PB7 | TIM4_CH2   |
| PWMC   | PB8 | TIM4_CH3   |
| Enable | PB9 | GPIO       |

---

## Motor Encoders (I2C, bit-bang)

| Encoder       | SDA  | SCL  |
|---------------|------|------|
| Left encoder  | PC11 | PC10 |
| Right encoder | PB5  | PC12 |

---

## Line Sensor

| Signal | Pin | Type |
|--------|-----|------|
| SDA    | PA5 | I2C  |
| SCL    | PA4 | I2C  |
| Reset  | PA3 | GPIO |

---

## Keys

| Key  | Pin |
|------|-----|
| KEY0 | PB2 |
| KEY1 | PB1 |

## RGB LEDs (active low)

| LED   | R   | G   | B   |
|-------|-----|-----|-----|
| LED A | PB0 | PC4 | PA7 |
| LED B | PA0 | PA1 | PA2 |
