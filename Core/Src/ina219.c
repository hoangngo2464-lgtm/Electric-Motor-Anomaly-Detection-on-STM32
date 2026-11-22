/* ============================================================
 * ina219.c — Triển khai thư viện INA219
 * ============================================================ */

#include "ina219.h"

/* ==================== HÀM CƠ BẢN ==================== */

/**
 * @brief  Ghi 16-bit vào thanh ghi INA219
 * @param  hi2c: con trỏ I2C handle
 * @param  reg:  địa chỉ thanh ghi (0x00 - 0x05)
 * @param  val:  giá trị 16-bit cần ghi
 * @retval HAL status
 */
HAL_StatusTypeDef INA219_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8);    /* Byte cao */
    buf[1] = (uint8_t)(val & 0xFF);  /* Byte thấp */

    return HAL_I2C_Mem_Write(hi2c, INA219_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
}

/**
 * @brief  Đọc 16-bit từ thanh ghi INA219
 * @param  hi2c: con trỏ I2C handle
 * @param  reg:  địa chỉ thanh ghi
 * @param  val:  con trỏ nhận giá trị đọc được
 * @retval HAL status
 */
HAL_StatusTypeDef INA219_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c, INA219_I2C_ADDR, reg,
                                             I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if (st == HAL_OK)
    {
        *val = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return st;
}

/* ==================== KHỞI TẠO ==================== */

/**
 * @brief  Khởi tạo INA219 với cấu hình 32V bus, PGA /8 (±320mV), 128 samples
 * @param  hi2c: con trỏ I2C handle
 * @retval HAL status
 */
HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef st;

    /* Bước 1: Reset chip (tùy chọn, khuyến nghị) */
    st = INA219_Reset(hi2c);
    if (st != HAL_OK) return st;

    HAL_Delay(1);  /* Đợi chip reset xong */

    /* Bước 2: Ghi thanh ghi Config = 0x3FFF
     * (32V, PGA /8, BADC=SADC=128 samples, Continuous shunt+bus) */
    st = INA219_WriteReg(hi2c, INA219_REG_CONFIG, INA219_CONFIG_VALUE);
    if (st != HAL_OK) return st;

    /* Bước 3: Ghi thanh ghi Calibration = 4096
     * (Current_LSB = 100µA, R_shunt = 0.1Ω, Cal = 0.04096/(100e-6×0.1)) */
    st = INA219_WriteReg(hi2c, INA219_REG_CALIB, INA219_CALIB_VALUE);
    if (st != HAL_OK) return st;

    HAL_Delay(1);  /* Đợi chip ổn định */

    return HAL_OK;
}

/**
 * @brief  Reset INA219 (ghi bit RST = 1 vào Config)
 */
HAL_StatusTypeDef INA219_Reset(I2C_HandleTypeDef *hi2c)
{
    return INA219_WriteReg(hi2c, INA219_REG_CONFIG, 0x8000);
}

/* ==================== ĐỌC GIÁ TRỊ ==================== */

/**
 * @brief  Đọc điện áp bus (V)
 * @note   Bus Voltage Register (02h) — Datasheet INA219:
 *         - Bits 15:3  → giá trị điện áp 13-bit (BD12..BD0)
 *         - Bit 2      → reserved
 *         - Bit 1      → CNVR (Conversion Ready)
 *         - Bit 0      → OVF (Math Overflow)
 *         LSB = 4mV, cần dịch phải 3 bit trước khi nhân.
 */
float INA219_GetBusVoltage_V(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw = 0;
    if (INA219_ReadReg(hi2c, INA219_REG_BUS, &raw) != HAL_OK)
        return -1.0f;

    /* Dịch phải 3 bit loại bỏ bit OVF/CNVR/Reserved, rồi nhân 4mV
     * Dùng float trực tiếp tránh tràn số khi kết quả trung gian vượt uint16_t */
    return (float)(raw >> 3) * 4.0f / 1000.0f;  /* mV → V */
}

/**
 * @brief  Đọc điện áp shunt (mV)
 * @note   Shunt Voltage Register (01h) — Datasheet INA219:
 *         LSB cố định = 10µV cho mọi cấu hình PGA.
 *         Giá trị có dấu (int16_t, 2's complement) vì dòng có thể đảo chiều.
 *         Với PGA /8 (±320mV): full scale = ±32000 raw = ±320mV.
 */
float INA219_GetShuntVoltage_mV(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw = 0;
    if (INA219_ReadReg(hi2c, INA219_REG_SHUNT, &raw) != HAL_OK)
        return -1.0f;

    int16_t signed_raw = (int16_t)raw;  /* Chuyển sang có dấu */
    return (float)signed_raw * INA219_SHUNT_LSB_UV / 1000.0f;  /* µV → mV */
}

/**
 * @brief  Đọc dòng điện (A)
 * @note   Current Register (04h) — Datasheet INA219:
 *         Current (A) = Current_Register × Current_LSB
 *         Current_LSB = 100µA = 0.0001 A/bit  (Cal = 4096, R_shunt = 0.1Ω)
 *         Giá trị có dấu (int16_t).
 */
float INA219_GetCurrent_A(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw = 0;
    if (INA219_ReadReg(hi2c, INA219_REG_CURRENT, &raw) != HAL_OK)
        return -1.0f;

    int16_t signed_raw = (int16_t)raw;
    return (float)signed_raw * INA219_CURRENT_LSB_A;
}

/**
 * @brief  Đọc công suất (W)
 * @note   Power Register (03h) — Datasheet INA219:
 *         Power_LSB = 20 × Current_LSB = 20 × 100µA = 2000µW = 0.002 W/bit
 *         Giá trị không dấu (uint16_t).
 */
float INA219_GetPower_W(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw = 0;
    if (INA219_ReadReg(hi2c, INA219_REG_POWER, &raw) != HAL_OK)
        return -1.0f;

    return (float)raw * INA219_POWER_LSB_W;
}
