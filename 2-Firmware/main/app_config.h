#pragma once

#include <stdint.h>

#define KIA_FW_VERSION_MAJOR 1
#define KIA_FW_VERSION_MINOR 0
#define KIA_FW_VERSION_PATCH 0

/* === GPIO map (locked, see docs/02-pinout.md) ============================ */
#define KIA_GPIO_POT_BASE     1
#define KIA_GPIO_POT_SHOULDER 2
#define KIA_GPIO_POT_ELBOW    4
#define KIA_GPIO_POT_WRIST    5
#define KIA_GPIO_POT_GRIP     6

#define KIA_GPIO_PWM_BASE     7
#define KIA_GPIO_PWM_SHOULDER 8
#define KIA_GPIO_PWM_ELBOW    9
#define KIA_GPIO_PWM_WRIST    10
#define KIA_GPIO_PWM_GRIP     11

#define KIA_GPIO_I2C_SDA 12
#define KIA_GPIO_I2C_SCL 13

#define KIA_GPIO_SPI_SCLK 14
#define KIA_GPIO_SPI_MOSI 15
#define KIA_GPIO_SPI_MISO 16
#define KIA_GPIO_SPI_CS0  17

#define KIA_GPIO_ESTOP     18
#define KIA_GPIO_MODE_SEL  21
#define KIA_GPIO_BOOT      0

#define KIA_GPIO_LED_STATUS 38
#define KIA_GPIO_LED_FAULT  39
#define KIA_GPIO_SERVO_EN   40
#define KIA_GPIO_BUZZER     41
#define KIA_GPIO_WS2812     48

/* === I2C ================================================================= */
#define KIA_I2C_PORT     0
#define KIA_I2C_FREQ_HZ  400000
#define KIA_I2C_INA219_ADDR 0x40

/* === Joints ============================================================== */
#define KIA_JOINT_COUNT 5
enum {
    KIA_J_BASE = 0,
    KIA_J_SHOULDER,
    KIA_J_ELBOW,
    KIA_J_WRIST,
    KIA_J_GRIP,
};

/* === Servo electrical config ============================================= */
#define KIA_SERVO_PWM_FREQ_HZ      50
#define KIA_SERVO_PULSE_MIN_US     500
#define KIA_SERVO_PULSE_MAX_US     2500
#define KIA_SERVO_PULSE_NEUTRAL_US 1500
#define KIA_SERVO_RESOLUTION_HZ    1000000  /* 1 MHz -> 1 us per tick */

/* === Joint geometry (DH-like, in mm) — TO MEASURE/CALIBRATE ============== */
#define KIA_LINK_BASE_HEIGHT_MM  60.0f
#define KIA_LINK_UPPER_ARM_MM    105.0f
#define KIA_LINK_FOREARM_MM      100.0f
#define KIA_LINK_TOOL_MM         85.0f

/* === Joint software limits (deg) ========================================= */
#define KIA_LIM_BASE_MIN_DEG     -90.0f
#define KIA_LIM_BASE_MAX_DEG     90.0f
#define KIA_LIM_SHOULDER_MIN_DEG -10.0f
#define KIA_LIM_SHOULDER_MAX_DEG 170.0f
#define KIA_LIM_ELBOW_MIN_DEG    -135.0f
#define KIA_LIM_ELBOW_MAX_DEG    135.0f
#define KIA_LIM_WRIST_MIN_DEG    -90.0f
#define KIA_LIM_WRIST_MAX_DEG    90.0f
#define KIA_LIM_GRIP_MIN_DEG     0.0f
#define KIA_LIM_GRIP_MAX_DEG     90.0f

/* === Slew rate (deg/s) =================================================== */
#define KIA_SLEW_MAX_DEGPS 240.0f

/* === ADC pots ============================================================ */
#define KIA_ADC_UNIT             0  /* ADC1 */
#define KIA_ADC_ATTEN            ADC_ATTEN_DB_12
#define KIA_ADC_BITWIDTH         12
#define KIA_ADC_SAMPLE_RATE_HZ   32000
#define KIA_ADC_OVERSAMPLE       64
#define KIA_ADC_EMA_ALPHA_Q16    13107  /* 0.20 in Q16 */
#define KIA_ADC_HYST_BASE_LSB    8
#define KIA_ADC_HYST_GAIN_Q16    16384  /* 0.25 in Q16 */

/* === Safety ============================================================== */
#define KIA_SAFETY_RATE_HZ       200
#define KIA_SAFETY_I_LIMIT_MA    8000     /* over-current threshold */
#define KIA_SAFETY_I_AVG_LIMIT_MA 5000    /* sustained current */
#define KIA_SAFETY_I_AVG_WINDOW_MS 1500
#define KIA_SAFETY_V_BUS_MIN_MV  4500     /* INA219 bus V should be ~6 V */

/* === Motion control ====================================================== */
#define KIA_CTRL_RATE_HZ         1000
#define KIA_TRAJECTORY_QUEUE_LEN 32

/* === Comms =============================================================== */
#define KIA_COMM_RX_BUF          1024
#define KIA_COMM_TX_BUF          1024
#define KIA_COMM_FRAME_MAX       512

/* === Task priorities (FreeRTOS, 0=lowest) ================================ */
#define KIA_PRIO_IDLE      0
#define KIA_PRIO_TELEMETRY 3
#define KIA_PRIO_COMM      6
#define KIA_PRIO_MODE_FSM  7
#define KIA_PRIO_MOTION    18  /* hard real-time */
#define KIA_PRIO_SAFETY    22  /* highest */

/* === Task stacks (bytes) ================================================= */
#define KIA_STACK_MOTION    8192
#define KIA_STACK_SAFETY    4096
#define KIA_STACK_COMM      6144
#define KIA_STACK_FSM       4096
#define KIA_STACK_TELEMETRY 4096

/* === Cores =============================================================== */
#define KIA_CORE_RT  1  /* APP_CPU */
#define KIA_CORE_IO  0  /* PRO_CPU */

/* === NVS namespaces ====================================================== */
#define KIA_NVS_NS_CAL      "kia_cal"
#define KIA_NVS_NS_CFG      "kia_cfg"
