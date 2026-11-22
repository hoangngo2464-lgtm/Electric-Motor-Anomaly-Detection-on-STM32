/* ============================================================
 * ina219.h — Thư viện INA219 cho STM32 HAL
 *
 * Cấu hình phần cứng:
 *   - Nguồn: 0 – 32V DC (ví dụ: 12V)
 *   - Shunt: R = 0.1 Ω  (in chữ R100 trên module)
 *   - Dòng tối đa: 3.2 A
 *
 * Cấu hình thanh ghi:
 *   - BRNG = 1        → Bus 32V range   (an toàn cho nguồn 12V)
 *   - PGA  = /8       → ±320 mV shunt   (đủ cho 3.2 A × 0.1 Ω = 320 mV)
 *   - BADC = 1111     → 128 samples AVG (68 ms, giảm nhiễu tốt)
 *   - SADC = 1111     → 128 samples AVG
 *   - MODE = 111      → Continuous shunt + bus
 *
 * Calibration:
 *   Current_LSB = 100 µA/bit  (làm tròn từ 3.2/32768 = 97.66 µA)
 *   Cal = 0.04096 / (100e-6 × 0.1) = 4096
 * ============================================================ */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32f1xx_hal.h"

/* ==================== ĐỊA CHỈ I2C ==================== */
/* A0 = GND, A1 = GND → 0x40 (7-bit) → 0x80 (8-bit cho HAL) */
#define INA219_I2C_ADDR       (0x40 << 1)

/* ==================== THANH GHI ==================== */
#define INA219_REG_CONFIG     0x00
#define INA219_REG_SHUNT      0x01
#define INA219_REG_BUS        0x02
#define INA219_REG_POWER      0x03
#define INA219_REG_CURRENT    0x04
#define INA219_REG_CALIB      0x05

/* ==================== CẤU HÌNH ==================== */
/*
 * 0x3FFF = 0011 1111 1111 1111
 * bit 13   (BRNG)   = 1  → 32V bus range
 * bit 12:11 (PGA)   = 11 → /8, ±320mV shunt
 * bit 10:7  (BADC)  = 1111 → 128 samples (~68ms)
 * bit 6:3   (SADC)  = 1111 → 128 samples (~68ms)
 * bit 2:0   (MODE)  = 111  → Continuous shunt + bus
 */
#define INA219_CONFIG_VALUE   0x3FFF

/*
 * Calibration = trunc(0.04096 / (Current_LSB × R_shunt))
 *             = trunc(0.04096 / (100e-6 × 0.1))
 *             = 4096
 */
#define INA219_CALIB_VALUE    4096

/* ==================== HỆ SỐ CHUYỂN ĐỔI ==================== */
/* Shunt voltage LSB = 10 µV (PGA /8 theo datasheet INA219) */
#define INA219_SHUNT_LSB_UV   10.0f

/* Bus voltage LSB = 4 mV */
#define INA219_BUS_LSB_MV     4.0f

/* Current LSB = 100 µA = 0.0001 A */
#define INA219_CURRENT_LSB_A  100e-6f

/* Power LSB = 20 × Current_LSB = 2000 µW = 0.002 W */
#define INA219_POWER_LSB_W    2000e-6f

/* ==================== API ==================== */
HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef INA219_Reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef INA219_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val);
HAL_StatusTypeDef INA219_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *val);

float INA219_GetBusVoltage_V(I2C_HandleTypeDef *hi2c);
float INA219_GetShuntVoltage_mV(I2C_HandleTypeDef *hi2c);
float INA219_GetCurrent_A(I2C_HandleTypeDef *hi2c);
float INA219_GetPower_W(I2C_HandleTypeDef *hi2c);

#endif /* INC_INA219_H_ */
