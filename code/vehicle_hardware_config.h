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
 * The confirmed steering sensor is the AB-output MT6701 module shown in the
 * supplied pinout.  From the module's Z-to-DIR order, P5 routes
 * Z=P10.5, B=P10.2, A=P10.3 and DIR=P10.1.  VCC/GND use the matching
 * power pins on the vehicle schematic.  A and DIR exactly match GPT12 TIM5,
 * so TIM5 provides the primary hardware count.  B and Z remain independent
 * phase/index monitors.
 */
constexpr auto VEHICLE_MT6701_INTERFACE_UNCONFIRMED = 0U;
constexpr auto VEHICLE_MT6701_INTERFACE_SSI = 1U;
constexpr auto VEHICLE_MT6701_INTERFACE_AB = 2U;
#ifndef VEHICLE_MT6701_INTERFACE
#define VEHICLE_MT6701_INTERFACE             VEHICLE_MT6701_INTERFACE_AB
#endif

#define VEHICLE_MT6701_SPI_INDEX             SPI_1
#define VEHICLE_MT6701_SPI_MODE              SPI_MODE2
constexpr auto VEHICLE_MT6701_SPI_HZ = 2000000U;
#define VEHICLE_MT6701_SPI_SCLK              SPI1_SCLK_P10_2
#define VEHICLE_MT6701_SPI_MOSI              SPI1_MOSI_P10_3
#define VEHICLE_MT6701_SPI_MISO              SPI1_MISO_P10_1
#define VEHICLE_MT6701_SPI_CS                SPI1_CS9_P10_5
#define VEHICLE_MT6701_AB_A_PIN              P10_3
#define VEHICLE_MT6701_AB_B_PIN              P10_2
#define VEHICLE_MT6701_AB_Z_PIN              P10_5
#define VEHICLE_MT6701_AB_DIR_PIN            P10_1
#define VEHICLE_MT6701_AB_COUNTER_INDEX       TIM5_ENCODER
#define VEHICLE_MT6701_AB_COUNTER_A_PIN       TIM5_ENCODER_CH1_P10_3
#define VEHICLE_MT6701_AB_COUNTER_DIR_PIN     TIM5_ENCODER_CH2_P10_1
/*
 * Tentative B-phase monitor polarity.  It never affects the primary counter:
 * hardware TIM5 consumes DIR directly.  Confirm or revise this after a
 * deliberately slow B-phase test.
 */
constexpr auto VEHICLE_MT6701_DIR_HIGH_IS_POSITIVE = 0U;
constexpr auto VEHICLE_MT6701_Z_ACTIVE_HIGH = 1U;

/* The schematic connector matches the stock IPS200 SPI pin map. */
#ifndef VEHICLE_IPS200_ENABLE
constexpr auto VEHICLE_IPS200_ENABLE = 1U;
#endif

#endif
