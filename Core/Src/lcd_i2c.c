#include "lcd_i2c.h"

static uint8_t slave_address_lcd = 0;
extern I2C_HandleTypeDef hi2c1;

static void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
  if (slave_address_lcd == 0)
    return; /* Bảo vệ */

  uint8_t data_t[2];
  uint8_t mask = (rs ? 0x01 : 0x00) | 0x08;
  uint8_t data = (nibble & 0xF0) | mask;

  data_t[0] = data | 0x04;
  data_t[1] = data & ~0x04;

  HAL_I2C_Master_Transmit(&hi2c1, slave_address_lcd, data_t, 2, 100);
}

void lcd_send_cmd(char cmd) {
  lcd_write_nibble(cmd & 0xF0, 0);
  lcd_write_nibble((cmd << 4) & 0xF0, 0);
}

void lcd_send_data(char data) {
  lcd_write_nibble(data & 0xF0, 1);
  lcd_write_nibble((data << 4) & 0xF0, 1);
}

void LCD_Clear(void) {
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row) {
  uint8_t row_offsets[] = {0x00, 0x40};
  if (row > 1)
    row = 1;
  if (col > 15)
    col = 15;
  lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

uint8_t LCD_Init(void) {
  /* Dò địa chỉ 8-bit cho HAL */
  uint8_t possible_addrs[] = {0x4E, 0x7E, 0x70, 0x40, 0x42, 0x44, 0x46};
  uint8_t num_addrs = sizeof(possible_addrs) / sizeof(possible_addrs[0]);

  slave_address_lcd = 0;
  for (int i = 0; i < num_addrs; i++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, possible_addrs[i], 1, 10) == HAL_OK) {
      slave_address_lcd = possible_addrs[i];
      break;
    }
  }

  if (slave_address_lcd == 0) {
    return 0; /* Không tìm thấy LCD */
  }

  /* Chuỗi khởi tạo HD44780 */
  HAL_Delay(50);

  lcd_write_nibble(0x30, 0);
  HAL_Delay(5);
  lcd_write_nibble(0x30, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x30, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x20, 0);
  HAL_Delay(1);

  lcd_send_cmd(0x28); /* 4-bit, 2 hàng, 5x8 */
  HAL_Delay(1);
  lcd_send_cmd(0x0C); /* Hiển thị ON, cursor OFF */
  HAL_Delay(1);
  lcd_send_cmd(0x01); /* Clear */
  HAL_Delay(2);
  lcd_send_cmd(0x06); /* Tự động tăng */
  HAL_Delay(1);

  return 1;
}

void LCD_Print(char *str) {
  uint8_t count = 0;
  while (*str && count < 16) {
    lcd_send_data(*str++);
    count++;
  }
  /* Đệm khoảng trắng cho đủ 16 ký tự */
  while (count < 16) {
    lcd_send_data(' ');
    count++;
  }
}