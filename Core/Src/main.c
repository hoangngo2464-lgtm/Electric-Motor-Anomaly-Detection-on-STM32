/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @project        : Industrial Equipment Monitoring & Anomaly Detection
 * @version        : 1.0 - Fixed PWM 20kHz, encoder x4, INA219 calib
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina219.h"
#include "lcd_i2c.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================== THÔNG SỐ HỆ THỐNG ==================== */

/* PWM: 72MHz / ((0+1) * (3599+1)) = 20 kHz */
#define PWM_PERIOD 3599   /* ARR = 3599 -> duty 0..3599 */
#define PWM_DUTY_100 3599 /* Duty 100% */
#define PWM_DUTY_50 1799  /* Duty 50% */
#define PWM_DUTY_0 0      /* Duty 0% (dừng) */

/* Encoder GA25-370-1260-EN (12V, 45:1, 11 PPR):
   - 11 PPR (xung/vòng motor)
   - Giảm tốc 45:1
   - Xung/vòng trục ra = 11 * 45 = 495
   - Dùng TI12 (x4 decoding) -> 495 * 4 = 1980 */
#define PULSES_PER_REV 1980.0f

/* Chu kỳ lấy mẫu (ms) */
#define SAMPLE_PERIOD_MS 100

/* Ngưỡng bảo vệ GA25-370 12V (Định mức ~0.3A, Kẹt tải/Stall ~1.3A) */
#define WARNING_CURRENT_A 0.9f /* Dòng quá tải cảnh báo */
#define CRITICAL_CURRENT_A                                                     \
  1.3f /* Dòng kẹt tải tức thời -> Ngắt khẩn cấp Motor */
#define WARNING_VOLTAGE_V 9.2f  /* Sụt áp cảnh báo (cho phép thử tải không bị giảm PWM) */
#define CRITICAL_VOLTAGE_V 8.5f /* Sụt áp ngắt khẩn cấp (8.5V an toàn cho DRV8871 và Pin 3S) */

/* Thời gian duy trì lỗi để chống nhiễu / gai sụt áp giả (Debounce ms) */
#define FAULT_LOWV_TIMEOUT_MS 800  /* Sụt áp phải duy trì liên tục >= 800ms */
#define FAULT_OCUR_TIMEOUT_MS 500  /* Quá dòng phải duy trì liên tục >= 500ms */

/* Ngưỡng bảo vệ kẹt động cơ (Motor Stall Protection):
 * Tải bình thường: 250 - 370 mA, RPM: 100 - 140.
 * Khi kẹt motor: Trục đứng yên (RPM < 15), dòng tăng cao (> 500 mA).
 * Thời gian trễ 1.2s: Đủ ghi nhận mẫu thử nghiệm, tự ngắt an toàn trước khi
 * nóng cháy cuộn dây/DRV8871. */
#define STALL_CURRENT_MA 1000.0f  /* Dòng kẹt trục tối thiểu (mA) */
#define STALL_RPM_THRESHOLD 15.0f /* Vòng/phút kẹt tải (gần như đứng yên) */
#define STALL_TIMEOUT_MS                                                       \
  1200 /* Thời gian kẹt tối đa cho phép (ms) trước khi tự ngắt */
#define STARTUP_GRACE_MS                                                       \
  1500 /* Thời gian bỏ qua kiểm tra RPM khi vừa khởi động (ms) */
#define STALL_RETRY_DELAY_MS                                                   \
  2000 /* Thời gian ngắt nghỉ trước khi tự khởi động lại (2 giây) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* --- Phân loại mã sự cố bảo vệ --- */
typedef enum {
  FAULT_NONE = 0,
  FAULT_SENSOR_LOST,   /* Mất kết nối cảm biến INA219 */
  FAULT_CRITICAL_OCUR, /* Quá dòng tức thời > 1.3A */
  FAULT_CRITICAL_LOWV, /* Sụt áp nguồn nặng < 8.5V */
  FAULT_MOTOR_STALL    /* Kẹt động cơ */
} FaultCode_t;

/* --- Trạng thái tự động thử lại sau kẹt tải (Auto-Retry Machine) --- */
typedef enum {
  STALL_STATE_NORMAL = 0, /* Bình thường */
  STALL_STATE_WAIT_RETRY, /* Tạm ngắt dừng 2 giây chờ tự động chạy lại */
  STALL_STATE_RETRYING,   /* Đang chạy lại để đánh giá tình trạng kẹt tải */
  STALL_STATE_LOCKED      /* Vẫn kẹt sau khi thử lại -> Ngắt khóa dừng hẳn */
} StallState_t;

StallState_t stall_state = STALL_STATE_NORMAL;
uint8_t stall_retry_count = 0; /* Số lần đã thử lại */
uint32_t stall_retry_tick = 0; /* Thời điểm bắt đầu chờ 2 giây */
uint32_t retry_eval_tick = 0;  /* Thời điểm bắt đầu chạy lại để đánh giá */

FaultCode_t current_fault = FAULT_NONE;
uint32_t stall_timer_start = 0; /* Mốc thời gian bắt đầu phát hiện kẹt tải */
uint32_t lowv_timer_start = 0;  /* Mốc thời gian bắt đầu phát hiện sụt áp */
uint32_t ocur_timer_start = 0;  /* Mốc thời gian bắt đầu phát hiện quá dòng */

/* --- Biến đo tốc độ động cơ --- */
uint16_t encoder_now = 0;
uint16_t encoder_last = 0;
int16_t delta_pulse = 0;
float rpm = 0.0f;

/* --- Biến INA219 --- */
float voltage_V = 0.0f;
float current_mA = 0.0f;
float power_mW = 0.0f;

/* --- Cờ bảo vệ --- */
uint8_t warning_flag = 0;
uint8_t lcd_ok = 0;    /* 1 = LCD kết nối OK */
uint8_t ina219_ok = 0; /* 1 = INA219 kết nối OK */
uint8_t critical_flag = 0;

/* --- Cờ điều khiển từ UART --- */
uint8_t motor_state = 0; /* 0: Dừng khi cấp nguồn (chờ nút nhấn PB12) */
uint8_t motor_dir = 1;   /* 1 = Thuận, 0 = Nghịch — tách khỏi motor_state */
uint8_t rx_data = 0;     /* Biến nhận 1 byte UART */

/* --- Bộ đệm LCD --- */
char lcd_buf[32];

/* --- Điều khiển thu thập dữ liệu CSV (M4 Pipeline) --- */
volatile uint8_t data_logging_enabled =
    0; /* 1 = Cho phép truyền CSV; bật/tắt bằng lệnh USB ('START'/'STOP' hoặc
          '1'/'0') */
uint32_t motor_start_tick =
    0; /* Thời điểm bật motor, dùng lọc bỏ 2 giây quá độ ban đầu */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Control(uint8_t dir, uint16_t duty);
void Motor_Stop(void);
void Read_Encoder(void);
void Read_INA219(void);
void Check_Protection(void);
void Update_LCD(void);
void Send_Data(void);
void Read_Buttons(void);
void Safety_Reset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Điều khiển motor qua DRV8871 (Sơ đồ: IN1 = PA0, IN2 = PA1)
 * @param  dir: 1 = quay thuận (IN2 = LOW), 0 = quay nghịch (IN2 = HIGH)
 * @param  duty: 0..PWM_PERIOD (tương ứng 0%..100%)
 * @note   DRV8871: IN1 = PWM (PA0 / TIM2_CH1), IN2 = GPIO (PA1)
 *         - Thuận: IN1 = PWM, IN2 = LOW
 *         - Nghịch: IN1 = PWM, IN2 = HIGH
 */
void Motor_Control(uint8_t dir, uint16_t duty) {
  if (duty > PWM_PERIOD)
    duty = PWM_PERIOD;

  if (dir == 1) {
    /* Quay thuận: IN1 = PWM, IN2 = LOW */
    HAL_GPIO_WritePin(motor_dir_GPIO_Port, motor_dir_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
  } else {
    /* Quay nghịch: IN1 = PWM, IN2 = HIGH */
    HAL_GPIO_WritePin(motor_dir_GPIO_Port, motor_dir_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
  }
}

/**
 * @brief  Dừng motor (coast)
 */
void Motor_Stop(void) {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(motor_dir_GPIO_Port, motor_dir_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Đọc encoder và tính RPM dựa trên thời gian thực tế
 * @note   Dùng HAL_GetTick() để đo dt_ms thực, không giả định 100 ms cố định.
 *         Xử lý tràn counter 16-bit bằng phép trừ uint16_t + ép kiểu int16_t.
 */
void Read_Encoder(void) {
  static uint32_t last_tick = 0;
  static uint8_t initialized = 0;

  uint32_t now_tick = HAL_GetTick();
  uint16_t now_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

  /* Lần đầu gọi: lưu trạng thái ban đầu, chưa tính RPM */
  if (!initialized) {
    encoder_last = now_count;
    last_tick = now_tick;
    initialized = 1;
    rpm = 0.0f;
    return;
  }

  uint32_t dt_ms = now_tick - last_tick;

  /* Bỏ qua nếu dt_ms = 0 để tránh chia 0 */
  if (dt_ms == 0)
    return;

  /* Đảo dấu: Do thứ tự dây pha A (PA6) và B (PA7) ngược với quy ước quay Thuận
   * của motor. Đảo dấu giúp: Quay Thuận -> delta_pulse dương (+), RPM dương (+)
   */
  delta_pulse = -(int16_t)(uint16_t)(now_count - encoder_last);
  encoder_last = now_count;
  last_tick = now_tick;

  /* RPM = (delta / PULSES_PER_REV) * (60000 / dt_ms) */
  rpm = ((float)delta_pulse * 60000.0f) / (PULSES_PER_REV * (float)dt_ms);
}

/**
 * @brief  Đọc INA219 (V, I, P) — Phát hiện mất kết nối phần cứng và tự phục hồi
 */
void Read_INA219(void) {
  /* Nếu chưa kết nối hoặc mất kết nối: thử kết nối lại định kỳ mỗi 1 giây */
  static uint32_t last_reconnect_tick = 0;
  if (!ina219_ok) {
    if (HAL_GetTick() - last_reconnect_tick >= 1000) {
      last_reconnect_tick = HAL_GetTick();
      if (HAL_I2C_IsDeviceReady(&hi2c1, 0x40 << 1, 2, 50) == HAL_OK) {
        if (INA219_Init(&hi2c1) == HAL_OK) {
          ina219_ok = 1;
        }
      }
    }
    return;
  }

  /* Đọc điện áp bus */
  float v = INA219_GetBusVoltage_V(&hi2c1);
  if (v < 0.0f) {
    /* Lỗi giao tiếp I2C trong khi đang vận hành -> chuyển sang trạng thái FAULT
     */
    ina219_ok = 0;
    return;
  }

  /* Đọc dòng điện và công suất */
  float i_a = INA219_GetCurrent_A(&hi2c1);
  float p_w = INA219_GetPower_W(&hi2c1);
  if (i_a < -10.0f || p_w < 0.0f) {
    /* Lỗi giao tiếp I2C */
    ina219_ok = 0;
    return;
  }

  voltage_V = v;
  current_mA = i_a * 1000.0f;
  power_mW = p_w * 1000.0f;

  /* Lọc giá trị rác âm nhỏ do offset quang/nhiệt quanh 0 */
  if (current_mA < 0.0f && current_mA > -5.0f)
    current_mA = 0.0f;
  if (power_mW < 0.0f)
    power_mW = 0.0f;
}

/**
 * @brief  Kiểm tra ngưỡng bảo vệ (Chống kẹt tải motor, Tự động thử lại 2s, Quá
 * dòng, Sụt áp)
 * @note   Quy trình bảo vệ kẹt tải:
 *         1. Khi phát hiện kẹt: Tạm dừng ngắt motor, đợi đúng 2 giây.
 *         2. Sau 2 giây: Tự động cho motor chạy lại để kiểm tra tải.
 *         3. Nếu vẫn tiếp tục kẹt -> NGẮT HẲN HOÀN TOÀN (LATCHED STOP, khóa
 * lỗi).
 *         4. Nếu hết kẹt và dòng ổn định -> Tiếp tục cho motor chạy bình
 * thường!
 */
void Check_Protection(void) {
  /* 1. Nếu hệ thống đang bị khóa lỗi nặng (LATCHED FAULT) -> Duy trì ngắt dừng
   * motor tuyệt đối */
  if (critical_flag) {
    Motor_Stop();
    motor_state = 0;
    return;
  }

  /* 2. Bảo vệ cảm biến: nếu mất kết nối INA219 -> ngắt motor an toàn */
  if (!ina219_ok) {
    current_fault = FAULT_SENSOR_LOST;
    critical_flag = 1;
    stall_state = STALL_STATE_LOCKED;
    motor_state = 0;
    Motor_Stop();
    return;
  }

  /* Bỏ qua 3 giây đầu sau khởi động MCU để nguồn và cảm biến ổn định */
  if (HAL_GetTick() < 3000) {
    warning_flag = 0;
    return;
  }

  /* 3. Bảo vệ quá dòng nặng (> 1.3A) hoặc sụt áp nguồn quá thấp (< 8.5V)
   * Có bộ đếm thời gian (debounce filter) để loại bỏ hoàn toàn các gai nhiễu tức thời khi thử tải */
  if (current_mA >= (CRITICAL_CURRENT_A * 1000.0f)) {
    if (ocur_timer_start == 0) {
      ocur_timer_start = HAL_GetTick();
    } else if (HAL_GetTick() - ocur_timer_start >= FAULT_OCUR_TIMEOUT_MS) {
      current_fault = FAULT_CRITICAL_OCUR;
      critical_flag = 1;
      stall_state = STALL_STATE_LOCKED;
      motor_state = 0;
      Motor_Stop();
      ocur_timer_start = 0;
      return;
    }
  } else {
    ocur_timer_start = 0;
  }

  if (voltage_V > 0.1f && voltage_V < CRITICAL_VOLTAGE_V) {
    if (lowv_timer_start == 0) {
      lowv_timer_start = HAL_GetTick();
    } else if (HAL_GetTick() - lowv_timer_start >= FAULT_LOWV_TIMEOUT_MS) {
      current_fault = FAULT_CRITICAL_LOWV;
      critical_flag = 1;
      stall_state = STALL_STATE_LOCKED;
      motor_state = 0;
      Motor_Stop();
      lowv_timer_start = 0;
      return;
    }
  } else {
    lowv_timer_start = 0;
  }

  /* 4. MÁY TRẠNG THÁI BẢO VỆ KẸT ĐỘNG CƠ & TỰ ĐỘNG THỬ LẠI (AUTO-RETRY): */

  /* 4A. Giai đoạn 1: Đang tạm ngắt nghỉ 2 giây chờ chạy lại */
  if (stall_state == STALL_STATE_WAIT_RETRY) {
    Motor_Stop();
    motor_state = 0;
    if (HAL_GetTick() - stall_retry_tick >= STALL_RETRY_DELAY_MS) {
      /* Đã đủ 2 giây ngắt nghỉ an toàn -> Tự động cho motor chạy lại để kiểm
       * tra! */
      stall_state = STALL_STATE_RETRYING;
      stall_retry_count = 1;
      motor_state = 1;
      motor_start_tick = HAL_GetTick();
      retry_eval_tick = HAL_GetTick();
      stall_timer_start = 0;
      char retry_msg[] =
          "[INFO] AUTO-RETRY: 2S ELAPSED, RESTARTING MOTOR TO TEST LOAD...\r\n";
      CDC_Transmit_FS((uint8_t *)retry_msg, sizeof(retry_msg) - 1);
    }
    return;
  }

  /* 4B. Giai đoạn 2: Đang thử chạy lại (RETRYING) để đánh giá tình trạng kẹt
   * tải */
  if (stall_state == STALL_STATE_RETRYING) {
    /* Đợi 1.0 giây sau khi bật lại để motor vượt qua dòng quá độ khởi động */
    if (HAL_GetTick() - retry_eval_tick >= 1000) {
      float abs_rpm = (rpm < 0.0f) ? -rpm : rpm;
      /* Nếu vẫn trong trạng thái kẹt (RPM < 15 & dòng >= 500mA) -> NGẮT HẲN */
      if (abs_rpm < STALL_RPM_THRESHOLD && current_mA >= STALL_CURRENT_MA) {
        current_fault = FAULT_MOTOR_STALL;
        stall_state = STALL_STATE_LOCKED;
        critical_flag = 1; /* Khóa lỗi dừng hẳn, không thử lại nữa */
        motor_state = 0;
        Motor_Stop();
        char lock_msg[] = "[FAULT] STILL STALLED ON RETRY! MOTOR PERMANENTLY "
                          "STOPPED. PRESS PB12 TO RESET.\r\n";
        CDC_Transmit_FS((uint8_t *)lock_msg, sizeof(lock_msg) - 1);
        return;
      }
      /* Nếu không còn ở trạng thái kẹt mà dòng ổn định motor hoạt động bình
       * thường */
      if (abs_rpm >= STALL_RPM_THRESHOLD &&
          current_mA < (WARNING_CURRENT_A * 1000.0f)) {
        stall_state = STALL_STATE_NORMAL;
        stall_retry_count = 0;
        current_fault = FAULT_NONE;
        char ok_msg[] = "[INFO] MOTOR RECOVERED NORMAL OPERATION! CONTINUING "
                        "RUNNING...\r\n";
        CDC_Transmit_FS((uint8_t *)ok_msg, sizeof(ok_msg) - 1);
      }
    }
    return;
  }

  /* 4C. Giai đoạn bình thường (STALL_STATE_NORMAL): Phát hiện kẹt tải lần đầu
   */
  if (motor_state == 1 &&
      (HAL_GetTick() - motor_start_tick >= STARTUP_GRACE_MS)) {
    float abs_rpm = (rpm < 0.0f) ? -rpm : rpm;
    if (abs_rpm < STALL_RPM_THRESHOLD && current_mA >= STALL_CURRENT_MA) {
      if (stall_timer_start == 0) {
        stall_timer_start = HAL_GetTick(); /* Bắt đầu bấm giờ kẹt tải */
      } else if (HAL_GetTick() - stall_timer_start >= STALL_TIMEOUT_MS) {
        /* Kẹt liên tục quá 1.2s -> TẠM NGẮT DỪNG MOTOR VÀ CHỜ 2S ĐỂ TỰ ĐỘNG
         * CHẠY LẠI */
        current_fault = FAULT_MOTOR_STALL;
        stall_state = STALL_STATE_WAIT_RETRY;
        stall_retry_tick = HAL_GetTick();
        motor_state = 0;
        Motor_Stop();
        stall_timer_start = 0;
        char wait_msg[] = "[WARN] MOTOR STALL DETECTED! MOTOR STOPPED. WAITING "
                          "2S BEFORE AUTO-RETRY...\r\n";
        CDC_Transmit_FS((uint8_t *)wait_msg, sizeof(wait_msg) - 1);
        return;
      }
    } else {
      stall_timer_start =
          0; /* Tốc độ quay tốt hoặc dòng hạ -> xóa bộ đếm kẹt */
    }
  } else {
    stall_timer_start = 0;
  }

  /* 5. Cảnh báo: Quá tải nhẹ -> Giảm 50% PWM */
  if (current_mA > (WARNING_CURRENT_A * 1000.0f) ||
      (voltage_V > 0.1f && warning_flag == 0 &&
       voltage_V < WARNING_VOLTAGE_V)) {
    warning_flag = 1;
  } else if (current_mA <= (WARNING_CURRENT_A * 1000.0f) &&
             (voltage_V >= WARNING_VOLTAGE_V)) {
    warning_flag = 0;
  }
}

/**
 * @brief  Cập nhật LCD 16x2
 */
void Update_LCD(void) {
  if (!lcd_ok)
    return; /* LCD không kết nối */

  /* Giới hạn tần số quét LCD: 250ms (4Hz) để tinh thể lỏng ổn định, chống
   * mờ/rung */
  static uint32_t last_lcd_tick = 0;
  if (HAL_GetTick() - last_lcd_tick < 250) {
    return;
  }
  last_lcd_tick = HAL_GetTick();

  /* ================================================================
   * Moving average N=4 cho hiển thị LCD
   * Chỉ áp dụng cho LCD — biến gốc rpm, current_mA KHÔNG bị thay đổi.
   * Dùng static ring buffer số nguyên, không tích lũy sai số float.
   * ================================================================ */
#define LCD_MA_N 4
  static int32_t rpm_buf[LCD_MA_N] = {0};
  static int32_t cur_buf[LCD_MA_N] = {0};
  static int32_t vol_buf[LCD_MA_N] = {0};
  static uint8_t ma_idx = 0;

  /* Nạp mẫu mới vào ring buffer */
  int32_t raw_rpm = (int32_t)rpm;
  if (raw_rpm < 0)
    raw_rpm = -raw_rpm; /* Hiển thị giá trị tuyệt đối */
  int32_t raw_cur = (int32_t)current_mA;
  if (raw_cur < 0)
    raw_cur = 0;
  int32_t raw_vol = (int32_t)(voltage_V * 1000.0f); /* Lọc theo mV */
  if (raw_vol < 0)
    raw_vol = 0;

  rpm_buf[ma_idx] = raw_rpm;
  cur_buf[ma_idx] = raw_cur;
  vol_buf[ma_idx] = raw_vol;
  ma_idx = (uint8_t)((ma_idx + 1) % LCD_MA_N);

  /* Tính trung bình trượt */
  int32_t sum_rpm = 0, sum_cur = 0, sum_vol = 0;
  for (uint8_t k = 0; k < LCD_MA_N; k++) {
    sum_rpm += rpm_buf[k];
    sum_cur += cur_buf[k];
    sum_vol += vol_buf[k];
  }
  int rpm_val = (int)(sum_rpm / LCD_MA_N);
  int i_mA = (int)(sum_cur / LCD_MA_N);
  int v_mV_avg = (int)(sum_vol / LCD_MA_N);

  /* Hàng 1 (chuẩn 16 ký tự): Điện áp V và Dòng điện I hoặc Báo lỗi cảm biến */
  if (!ina219_ok) {
    snprintf(lcd_buf, sizeof(lcd_buf), "INA219: FAULT   ");
  } else {
    /* Làm tròn số nguyên từ mV: (mV + 50) / 100
     * Ví dụ: 11956 mV -> (11956+50)/100 = 120 -> 12.0V
     *        11840 mV -> (11840+50)/100 = 118 -> 11.8V
     * Triệt tiêu 100% hiện tượng 11.95V bị nhảy thành 11.0V! */
    int v_total = (v_mV_avg + 50) / 100;
    int v_nguyen = v_total / 10;
    int v_thapphan = v_total % 10;

    snprintf(lcd_buf, sizeof(lcd_buf), "V:%2d.%1dV I:%4dmA", v_nguyen,
             v_thapphan, i_mA);
  }
  LCD_SetCursor(0, 0);
  LCD_Print(lcd_buf);

  /* Hàng 2 (chuẩn 16 ký tự): Tốc độ RPM và Công suất P / Trạng thái ngắt */
  int p_mW = (int)power_mW;
  if (p_mW < 0)
    p_mW = 0;

  if (!ina219_ok) {
    /* Mất kết nối cảm biến -> dừng motor, báo rõ lỗi */
    snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d  STOP:SENS", rpm_val);
  } else if (stall_state == STALL_STATE_WAIT_RETRY) {
    /* Đang trong 2 giây tạm dừng chờ tự động chạy lại */
    uint32_t elapsed = HAL_GetTick() - stall_retry_tick;
    uint32_t remain_s = (elapsed < STALL_RETRY_DELAY_MS)
                            ? ((STALL_RETRY_DELAY_MS - elapsed + 999) / 1000)
                            : 1;
    snprintf(lcd_buf, sizeof(lcd_buf), "R:   0  WAIT:%1lus",
             (unsigned long)remain_s);
  } else if (stall_state == STALL_STATE_RETRYING) {
    /* Đang thử chạy lại để đánh giá tình trạng kẹt */
    snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d  RETRYING", rpm_val);
  } else if (critical_flag) {
    /* Hiển thị rõ nguyên nhân ngắt bảo vệ khẩn cấp (chuẩn 16 ký tự) */
    if (current_fault == FAULT_MOTOR_STALL) {
      snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d STOP:STAL", rpm_val);
    } else if (current_fault == FAULT_CRITICAL_OCUR) {
      snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d STOP:OCUR", rpm_val);
    } else if (current_fault == FAULT_CRITICAL_LOWV) {
      snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d STOP:VOLT", rpm_val);
    } else {
      snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d STOP:CRIT", rpm_val);
    }
  } else {
    /* Hiển thị bình thường: thêm ký hiệu [L] nếu đang thực sự stream dữ liệu ổn
     * định */
    uint8_t is_logging = (data_logging_enabled && (motor_state != 0) &&
                          (HAL_GetTick() - motor_start_tick >= 2000));
    if (is_logging) {
      snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d P:%4d [L]", rpm_val, p_mW);
    } else {
      snprintf(lcd_buf, sizeof(lcd_buf), "R:%4d P:%5dmW", rpm_val, p_mW);
    }
  }
  LCD_SetCursor(0, 1);
  LCD_Print(lcd_buf);
}

/**
 * @brief  Gui du lieu giam sat qua USB Virtual COM Port (CSV format)
 *         Chỉ gửi khi:
 *         1. data_logging_enabled == 1 (cho phép thu thập qua lệnh máy tính
 * 'START'/'1'/'L')
 *         2. motor_state != 0 (động cơ đang chạy, không gửi dòng 0,0,0,0 khi
 * dừng)
 *         3. critical_flag == 0 (không ở trạng thái ngắt bảo vệ)
 *         4. HAL_GetTick() - motor_start_tick >= 2000 (đã chạy ổn định > 2
 * giây, loại bỏ dữ liệu quá độ)
 */
void Send_Data(void) {
  /* 1. Kiểm tra quyền thu thập */
  if (!data_logging_enabled) {
    return;
  }

  /* 2. Khi xảy ra sự cố ngắt khẩn cấp: Gửi thông báo sự cố một lần duy nhất qua
   * USB CDC */
  static uint8_t fault_notified = 0;
  if (critical_flag) {
    if (!fault_notified) {
      fault_notified = 1;
      char err_msg[64];
      int err_len = 0;
      if (current_fault == FAULT_MOTOR_STALL) {
        err_len = snprintf(err_msg, sizeof(err_msg),
                           "[FAULT] MOTOR STALL DETECTED! MOTOR STOPPED.\r\n");
      } else if (current_fault == FAULT_CRITICAL_OCUR) {
        err_len = snprintf(err_msg, sizeof(err_msg),
                           "[FAULT] OVERCURRENT DETECTED! MOTOR STOPPED.\r\n");
      } else if (current_fault == FAULT_CRITICAL_LOWV) {
        err_len = snprintf(err_msg, sizeof(err_msg),
                           "[FAULT] UNDERVOLTAGE DETECTED! MOTOR STOPPED.\r\n");
      } else {
        err_len = snprintf(err_msg, sizeof(err_msg),
                           "[FAULT] SENSOR LOST! MOTOR STOPPED.\r\n");
      }
      CDC_Transmit_FS((uint8_t *)err_msg, (uint16_t)err_len);
    }
    return;
  }
  fault_notified = 0;

  /* Không gửi dòng CSV khi động cơ dừng */
  if (motor_state == 0) {
    return;
  }

  /* 3. Bỏ qua 2 giây đầu khởi động để lọc bỏ dữ liệu quá độ không ổn định */
  if (HAL_GetTick() - motor_start_tick < 2000) {
    return;
  }

  char buf[80];
  int len;
  int rpm_send =
      (int)rpm; /* Quay Thuận -> số dương (+), Quay Nghịch -> số âm (-) */

  if (!ina219_ok) {
    len = snprintf(buf, sizeof(buf), "%lu,ERR,ERR,ERR,%d\r\n",
                   (unsigned long)HAL_GetTick(), rpm_send);
  } else {
    len = snprintf(buf, sizeof(buf), "%lu,%d,%d,%d,%d\r\n",
                   (unsigned long)HAL_GetTick(),
                   (int)(voltage_V * 1000.0f), /* mV  */
                   (int)current_mA,            /* mA  */
                   (int)power_mW,              /* mW  */
                   rpm_send);                  /* RPM */
  }

  /* Neu lan gui truoc chua xong (USBD_BUSY) thi thu lai toi da 10 ms */
  uint32_t t = HAL_GetTick();
  while (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len) == USBD_BUSY) {
    if (HAL_GetTick() - t > 10)
      break;
  }
}

/**
 * @brief  Khôi phục hệ thống về trạng thái sẵn sàng an toàn
 */
void Safety_Reset(void) {
  critical_flag = 0;
  stall_state = STALL_STATE_NORMAL;
  stall_retry_count = 0;
  current_fault = FAULT_NONE;
  stall_timer_start = 0;
  lowv_timer_start = 0;
  ocur_timer_start = 0;
  motor_state = 0;
  Motor_Stop();
}

/**
 * @brief  Đọc 2 nút nhấn — debounce cạnh xuống, blocking 50 ms
 *         PB13 = Đảo chiều (Thuận <-> Nghịch)
 *         PB12 = Bật / Tắt động cơ / Reset lỗi bảo vệ
 * @note   Phát hiện cạnh XUỐNG (nhấn), chờ blocking 50 ms rồi đọc lại.
 *         Đảo chiều phải dừng motor ngắn để DRV8871 không vào brake mode.
 */
void Read_Buttons(void) {
  static GPIO_PinState prev_pb13 = GPIO_PIN_SET;
  static GPIO_PinState prev_pb12 = GPIO_PIN_SET;

  GPIO_PinState now_pb13 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  GPIO_PinState now_pb12 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

  /* --- PB13: Đảo chiều --- */
  if (prev_pb13 == GPIO_PIN_SET && now_pb13 == GPIO_PIN_RESET) {
    HAL_Delay(50); /* Debounce blocking */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
      if (motor_state != 0) {
        Motor_Stop();
        HAL_Delay(10); /* Cho DRV8871 về trạng thái tắt trước khi đảo chiều */
      }
      motor_dir = (motor_dir == 1) ? 0 : 1;
    }
    prev_pb13 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
  } else {
    prev_pb13 = now_pb13;
  }

  /* --- PB12: Bật / Tắt motor / Reset lỗi bảo vệ --- */
  if (prev_pb12 == GPIO_PIN_SET && now_pb12 == GPIO_PIN_RESET) {
    HAL_Delay(50);
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
      /* Nếu đang bị khóa lỗi hoặc đang trong quá trình ngắt kẹt tải:
       * Nhấn PB12 sẽ giải phóng lỗi, đưa về trạng thái dừng sẵn sàng an toàn */
      if (critical_flag != 0 || stall_state != STALL_STATE_NORMAL) {
        Safety_Reset();
      } else {
        motor_state = (motor_state == 0) ? 1 : 0;
        if (motor_state == 1) {
          motor_start_tick =
              HAL_GetTick(); /* Ghi nhận thời điểm bắt đầu chạy */
          stall_timer_start = 0;
        } else {
          motor_start_tick = 0;
          Motor_Stop();
        }
      }
    }
    prev_pb12 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
  } else {
    prev_pb12 = now_pb12;
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Ep USB ngat ket noi ~50ms de PC nhan dien lai sau reset (chong loi Code 43
   * tren BluePill) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio_usb = {0};
  gpio_usb.Pin = GPIO_PIN_12;
  gpio_usb.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_usb.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio_usb);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(50);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* 1. Khởi động PWM và Encoder */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  encoder_last = 0;
  Motor_Stop();

  /* 2. Khởi tạo LCD — để hàm tự dò địa chỉ */
  lcd_ok = LCD_Init();
  if (lcd_ok) {
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("STARTING SYSTEM!");
    HAL_Delay(1000);
    LCD_Clear();
  } else {
    /* Không tìm thấy LCD — có thể bật LED báo lỗi */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  }

  /* 3. Khởi tạo INA219 */
  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x40 << 1, 3, 100) == HAL_OK) {
    if (INA219_Init(&hi2c1) == HAL_OK) {
      ina219_ok = 1;
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* ============================================
     * BUOC 0: DOC NUT NHAN (PB13 = Chieu, PB12 = Bat/Tat)
     * ============================================ */
    Read_Buttons();

    /* ============================================
     * BUOC 1: DIEU KHIEN MOTOR
     * motor_state: 0 = Dung, 1 = Chay
     * motor_dir  : 1 = Thuan, 0 = Nghich
     * ============================================ */
    if (critical_flag) {
      Motor_Stop();
    } else if (motor_state == 0) {
      Motor_Stop();
    } else if (warning_flag) {
      Motor_Control(motor_dir, PWM_DUTY_50); /* Giam 50% khi qua tai */
    } else {
      Motor_Control(motor_dir, PWM_DUTY_100); /* Chay binh thuong */
    }

    /* ============================================
     * BUOC 2: CHO LAY MAU
     * ============================================ */
    HAL_Delay(SAMPLE_PERIOD_MS);

    /* ============================================
     * BUOC 3: DOC ENCODER -> RPM
     * ============================================ */
    Read_Encoder();

    /* ============================================
     * BUOC 4: DOC INA219 -> V, I, P
     * ============================================ */
    Read_INA219();

    /* ============================================
     * BUOC 5: KIEM TRA BAO VE
     * ============================================ */
    Check_Protection();

    /* ============================================
     * BUOC 6: HIEN THI LCD
     * ============================================ */
    Update_LCD();

    /* ============================================
     * BUOC 7: TRUYEN DU LIEU QUA USB CDC
     * ============================================ */
    Send_Data();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motor_dir_GPIO_Port, motor_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : motor_dir_Pin */
  GPIO_InitStruct.Pin = motor_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motor_dir_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Cau hinh PB13 (Dao chieu) va PB12 (Bat/Tat) la Input voi Pull-up noi */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Ngắt nhận UART để xử lý lệnh từ C# WinForms
 */
/* Bỏ comment đoạn này sau khi đã bật USART1 trong CubeMX
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Kiểm tra xem có đúng là UART1 không
    if (huart->Instance == USART1)
    {
        if (rx_data == 'F') motor_state = 1;      // Quay Thuận
        else if (rx_data == 'R') motor_state = 2; // Quay Nghịch
        else if (rx_data == 'S') motor_state = 0; // Dừng (Stop)

        // Tiếp tục nhận byte tiếp theo
        HAL_UART_Receive_IT(huart, &rx_data, 1);
    }
}
*/

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
