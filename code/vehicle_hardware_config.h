#ifndef VEHICLE_HARDWARE_CONFIG_H
#define VEHICLE_HARDWARE_CONFIG_H

/*
 * Pin assignments traced from SCH_Schematic1_1_2026-08-09.pdf.
 * This file is the only application-level hardware mapping.
 */
#define VEHICLE_LEFT_PWM_PIN                 ATOM0_CH2_P21_4
#define VEHICLE_LEFT_DIRECTION_PIN           P21_5
#define VEHICLE_RIGHT_PWM_PIN                ATOM0_CH1_P21_3
#define VEHICLE_RIGHT_DIRECTION_PIN          P21_2
#define VEHICLE_STEERING_PWM_PIN             ATOM0_CH4_P02_4
#define VEHICLE_STEERING_DIRECTION_PIN       P02_5

#define VEHICLE_LEFT_ENCODER_INDEX           TIM6_ENCODER
#define VEHICLE_LEFT_ENCODER_A_PIN           TIM6_ENCODER_CH1_P20_3
#define VEHICLE_LEFT_ENCODER_B_PIN           TIM6_ENCODER_CH2_P20_0
#define VEHICLE_RIGHT_ENCODER_INDEX          TIM3_ENCODER
#define VEHICLE_RIGHT_ENCODER_A_PIN          TIM3_ENCODER_CH1_P02_6
#define VEHICLE_RIGHT_ENCODER_B_PIN          TIM3_ENCODER_CH2_P02_7

#define VEHICLE_KEY_1_PIN                    P20_6
#define VEHICLE_KEY_2_PIN                    P20_7

#define VEHICLE_MOTOR_PWM_HZ                 (20000U)

/*
 * P5 has P10.1/P10.3/P10.2/P10.5, but the supplied schematic does not
 * label their MT6701 signal names or selected output protocol.  Both usable
 * implementations are built; keep UNCONFIRMED selected until P5 is verified.
 */
#define VEHICLE_MT6701_INTERFACE_UNCONFIRMED (0U)
#define VEHICLE_MT6701_INTERFACE_SSI         (1U)
#define VEHICLE_MT6701_INTERFACE_AB          (2U)
#ifndef VEHICLE_MT6701_INTERFACE
#define VEHICLE_MT6701_INTERFACE             VEHICLE_MT6701_INTERFACE_UNCONFIRMED
#endif

#define VEHICLE_MT6701_SPI_INDEX             SPI_1
#define VEHICLE_MT6701_SPI_MODE              SPI_MODE2
#define VEHICLE_MT6701_SPI_HZ                (2000000U)
#define VEHICLE_MT6701_SPI_SCLK              SPI1_SCLK_P10_2
#define VEHICLE_MT6701_SPI_MOSI              SPI1_MOSI_P10_3
#define VEHICLE_MT6701_SPI_MISO              SPI1_MISO_P10_1
#define VEHICLE_MT6701_SPI_CS                SPI1_CS9_P10_5
#define VEHICLE_MT6701_AB_INDEX              TIM5_ENCODER
#define VEHICLE_MT6701_AB_A_PIN              TIM5_ENCODER_CH1_P10_3
#define VEHICLE_MT6701_AB_B_PIN              TIM5_ENCODER_CH2_P10_1

/* The schematic connector matches the stock IPS200 SPI pin map. */
#ifndef VEHICLE_IPS200_ENABLE
#define VEHICLE_IPS200_ENABLE                (1U)
#endif

#endif
