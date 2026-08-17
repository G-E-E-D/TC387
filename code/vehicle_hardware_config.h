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
#define VEHICLE_LEFT_ENCODER_DIR_PIN         TIM6_ENCODER_CH2_P20_0
#define VEHICLE_RIGHT_ENCODER_INDEX          TIM3_ENCODER
#define VEHICLE_RIGHT_ENCODER_A_PIN          TIM3_ENCODER_CH1_P02_6
#define VEHICLE_RIGHT_ENCODER_DIR_PIN        TIM3_ENCODER_CH2_P02_7

#define VEHICLE_KEY_1_PIN                    P20_6
#define VEHICLE_KEY_2_PIN                    P20_7

constexpr auto VEHICLE_MOTOR_PWM_HZ = 20000U;

/*
 * Production steering sensor: 12-bit absolute encoder on schematic P5.
 * P10.1/P10.2/P10.3/P10.5 are reserved for the SPI1 Mode-0 transaction;
 * the old TIM5 AB/DIR mapping is intentionally not part of the normal HAL.
 */
#define VEHICLE_STEERING_ENCODER_SPI_INDEX  SPI_1
#define VEHICLE_STEERING_ENCODER_SPI_MODE   SPI_MODE0
constexpr auto VEHICLE_STEERING_ENCODER_SPI_HZ = 2000000U;
#define VEHICLE_STEERING_ENCODER_SCLK       SPI1_SCLK_P10_2
#define VEHICLE_STEERING_ENCODER_MISO       SPI1_MISO_P10_1
#define VEHICLE_STEERING_ENCODER_MOSI       SPI1_MOSI_P10_3
#define VEHICLE_STEERING_ENCODER_CS         SPI1_CS9_P10_5

/* Historical constants are kept only so the retained bench adapter can be
 * compiled independently; vehicle_app never selects these interfaces. */
#define VEHICLE_MT6701_INTERFACE_UNCONFIRMED 0U
#define VEHICLE_MT6701_INTERFACE_SSI         1U
#define VEHICLE_MT6701_INTERFACE_AB          2U

/* The schematic connector matches the stock IPS200 SPI pin map. */
#ifndef VEHICLE_IPS200_ENABLE
constexpr auto VEHICLE_IPS200_ENABLE = 1U;
#endif

#endif
