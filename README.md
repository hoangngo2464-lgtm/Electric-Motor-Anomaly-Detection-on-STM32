# DriveGuard-AI: Smart Motor Monitoring & Anomaly Detection System

[![Platform](https://img.shields.io/badge/Platform-STM32F103-blue.svg)](https://www.st.com)
[![Algorithm](https://img.shields.io/badge/Algorithm-Isolation%20Forest-green.svg)](https://scikit-learn.org)
[![Performance](https://img.shields.io/badge/F1--Score-99.8%25-brightgreen.svg)](#performance-results)
[![Latency](https://img.shields.io/badge/Inference%20Latency-6.7ms-orange.svg)](#performance-results)

Hệ thống nhúng thời gian thực giám sát thông số vận hành động cơ điện (Điện áp, Dòng điện, Công suất, Tốc độ RPM), ứng dụng mô hình học máy không giám sát (**Isolation Forest**) kết hợp bộ lọc trượt nhân quả để phát hiện sớm các bất thường và bảo vệ động cơ, loại bỏ điểm mù của phương pháp ngưỡng tĩnh truyền thống.

---

## 📌 Tính năng nổi bật (Key Features)

- **Thu thập dữ liệu đa biến thời gian thực (10 Hz):**
  - Điện áp bus ($V$), dòng điện shunt ($I$), công suất ($P$) thông qua cảm biến **INA219** (chuẩn I2C).
  - Tốc độ vòng/phút (**RPM**) đọc từ bộ mã hóa xung (Quadrature Encoder) thông qua Timer phần cứng chế độ x4.
- **Phát hiện lỗi ẩn (Hidden Fault Detection):**
  - Khắc phục nhược điểm của rơ-le / cầu chì truyền thống (chỉ bảo vệ khi quá dòng cứng $I > 1000\text{ mA}$).
  - Nhận diện tức thời các lỗi mòn bạc đạn, bó trục nhẹ, suy giảm hiệu suất (ví dụ: $I = 600\text{ mA}$, $\text{RPM} = 20$).
- **Bộ lọc chống báo động giả (False Positive Filtering):**
  - Tích hợp bộ lọc trượt nhân quả ($k/N$ sliding window) và máy trạng thái (State Machine) để loại bỏ xung gai nhiễu do mặt đường và dao động nguồn.
- **Hiệu năng cao & Độ trễ thấp:**
  - **F1-score: 99.8%** (Precision: 100%, Recall: 99.5%).
  - Thời gian suy luận mô hình AI chỉ **6.7 ms/mẫu**.

---

## 🏗️ Kiến trúc thư mục (Repository Structure)

```text
DriveGuard-AI/
├── README.md                   # Giới thiệu dự án và hướng dẫn sử dụng
├── .gitignore                  # Bỏ qua file rác biên dịch (.o, .elf, Debug/)
│
├── firmware/                   # Dự án mã nguồn STM32CubeIDE
│   ├── motor.ioc               # File cấu hình vi điều khiển CubeMX
│   ├── STM32F103C8TX_FLASH.ld  # Linker script
│   ├── Core/                   # Mã nguồn C (main.c, ina219.c, lcd_i2c.c...)
│   ├── Drivers/                # Thư viện STM32 HAL & CMSIS
│   ├── USB_DEVICE/             # Cấu hình giao tiếp USB CDC (Virtual COM Port)
│   └── Middlewares/            # Thư viện USB Middleware
│
├── ai_pipeline/                # Pipeline Machine Learning (Python)
│   ├── requirements.txt        # Các thư viện phụ thuộc
│   ├── train_model.py          # Huấn luyện mô hình Isolation Forest (200 trees)
│   ├── alarm_filter.py         # Bộ lọc trượt sliding filter & state machine
│   └── evaluate_ai.py          # Kiểm thử 13 kịch bản vận hành & đánh giá độ trễ
│
└── docs/                       # Tài liệu báo cáo đồ án
    └── BaoCao_DoAn_Nhom1.pdf   # Báo cáo kỹ thuật chi tiết
```

---

## 🚀 Hướng dẫn cài đặt & Chạy thử nghiệm

### 1. Phần sụn STM32 (Firmware)
1. Mở phần mềm **STM32CubeIDE**.
2. Chọn `File` -> `Open Projects from File System...` và trỏ đến thư mục `firmware/`.
3. Nhấn `Build Project` (Ctrl + B) để biên dịch chương trình.
4. Nạp code vào kit STM32F103 qua mạch nạp ST-Link V2.

### 2. Mô hình AI (Python Pipeline)
Yêu cầu môi trường Python 3.8+:
```bash
# Di chuyển vào thư mục ai_pipeline
cd ai_pipeline

# Cài đặt thư viện
pip install -r requirements.txt

# Chạy huấn luyện mô hình Isolation Forest
python train_model.py

# Chạy kiểm thử 13 kịch bản lỗi và đo độ trễ suy luận
python evaluate_ai.py
```

---

## 📊 Kết quả thực nghiệm (Performance Results)

| Tiêu chí | Ngưỡng tĩnh truyền thống (Rơ-le / Cầu chì) | DriveGuard-AI (Isolation Forest + Filter) |
| :--- | :---: | :---: |
| **Kẹt trục cứng ($I > 1000\text{ mA}$)** | ✅ Phát hiện được | ✅ Phát hiện được |
| **Lỗi ẩn (Bó trục nhẹ, $I = 600\text{ mA}, \text{RPM} = 20$)** | ❌ **BỎ SÓT** | ✅ **Phát hiện ngay lập tức** |
| **F1-Score** | ~72.5% | **99.8%** |
| **Độ trễ phản hồi** | Chậm (theo nhiệt dòng điện) | **6.7 ms** (Suy luận AI) |
| **Báo động giả khi tải biến thiên** | Dễ báo sai | Triệt tiêu nhờ bộ lọc $3/5$ mẫu |

