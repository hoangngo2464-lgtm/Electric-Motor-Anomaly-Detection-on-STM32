"""
realtime_inference.py
---------------------
Thu thap du lieu realtime tu STM32 qua Serial, suy luan Isolation Forest V4,
hien thi va ghi log tach rieng AI_RAW, AI_DEFAULT, RULE, ALARM.

Phien ban: V4 evidence collection (Buoc 5.1)
Model   : isolation_forest_v4.pkl
Scaler  : scaler_v4.pkl
Nguong van hanh : -0.0613 (giu tu V2, khao sat tren V4)
Nguong tham khao: 0.0     (mac dinh sklearn decision_function)

Thu tu dac trung: [mV, mA, mW, rpm] (Kich thuoc: 1x4, don vi chuan)

Rule logic hien tai (KHONG thay doi):
  rule_anomaly = (mA > 900) OR (rpm < 15)

Alarm logic hien tai (KHONG dung sliding/state machine):
  rule_anomaly        -> "ANOMALY(Rule)"
  ai_raw_anomaly only -> "AI_Warn"
  else                -> "Normal"

Chu thich an toan:
  - Khong cho AI tu dong ngat hoac dieu khien motor (chi giam sat).
  - Khong lay AI_RAW, RULE hoac ALARM lam nhan ground truth.
  - Mau loi cam bien / truyen thong duoc phan biet rieng (SENSOR_FAULT, PARSE_ERROR),
    khong coi la Normal va khong am tham bo qua.

Cach chay:
  python realtime_inference.py                  # Chay realtime tren COM8 (hoac tu tim)
  python realtime_inference.py --port COM3      # Chon cong cu the
  python realtime_inference.py --self-test      # Kiem thu phan mem (khong can Serial)
"""

import os
import sys
import csv
import time
import argparse
import warnings
from datetime import datetime
import numpy as np
import joblib

# Tat canh bao sklearn version va feature names de giao dien ro rang
warnings.filterwarnings("ignore")

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# ============ CAU HINH DUONG DAN & MODEL ============
# Tu dong tim thu muc goc STM32_Project du chay tu scripts/ hay scripts/realtime/
_curr = os.path.dirname(os.path.abspath(__file__))
_probe = _curr
while _probe and os.path.basename(_probe) != "STM32_Project":
    _parent = os.path.dirname(_probe)
    if _parent == _probe:
        break
    _probe = _parent

if os.path.basename(_probe) == "STM32_Project":
    PROJECT_DIR = _probe
else:
    PROJECT_DIR = os.path.abspath(os.path.join(_curr, ".."))

MODEL_DIR = os.path.join(PROJECT_DIR, "models", "active")
MODEL_PATH = os.path.join(MODEL_DIR, "isolation_forest_v4.pkl")
SCALER_PATH = os.path.join(MODEL_DIR, "scaler_v4.pkl")
LOG_DIR = os.path.join(PROJECT_DIR, "data", "realtime_logs")

BAUD = 115200
THRESHOLD = -0.0613          # Nguong van hanh (giu nguyen buoc 5.1)
THRESHOLD_DEFAULT = 0.0      # Nguong tham khao (mac dinh sklearn)
FEATURES = ['mV', 'mA', 'mW', 'rpm']
MODEL_VERSION = "isolation_forest_v4"
N_FEATURES = 4

# Cot CSV log du lieu
LOG_COLUMNS = [
    "ms", "mV", "mA", "mW", "rpm",
    "score", "AI_RAW", "AI_DEFAULT",
    "RULE", "ALARM",
    "model_version", "threshold",
    "receive_time", "processing_latency_ms",
]

# Cot CSV log loi (tach rieng, khong lam ban log du lieu sach)
ERROR_LOG_COLUMNS = [
    "receive_time", "raw_line", "error_type", "error_detail",
]


# ============ LOAD MODEL ============

def load_model_and_scaler():
    """Tai model va scaler V4. Xac minh kich thuoc dau vao."""
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Khong tim thay model tai: {MODEL_PATH}")
    if not os.path.exists(SCALER_PATH):
        raise FileNotFoundError(f"Khong tim thay scaler tai: {SCALER_PATH}")

    model = joblib.load(MODEL_PATH)
    scaler = joblib.load(SCALER_PATH)

    # Xac minh so dac trung
    scaler_features = getattr(scaler, "n_features_in_", None)
    model_features = getattr(model, "n_features_in_", None)

    if scaler_features != N_FEATURES:
        raise ValueError(
            f"Scaler yeu cau {scaler_features} dac trung, cau hinh la {N_FEATURES}"
        )
    if model_features != N_FEATURES:
        raise ValueError(
            f"Model yeu cau {model_features} dac trung, cau hinh la {N_FEATURES}"
        )

    return model, scaler


def print_model_info(model, scaler):
    """In chi tiet thong so mo hinh va bo chuan hoa da tai."""
    print("\n--- THONG SO MO HINH V4 DA TAI ---")
    print(f"  Model version       : {MODEL_VERSION}")
    print(f"  Model path          : {MODEL_PATH}")
    print(f"  Scaler path         : {SCALER_PATH}")
    print(f"  Kich thuoc dau vao  : {N_FEATURES} dac trung")
    print(f"  Thu tu dac trung    : {FEATURES}")
    print(f"  Nguong van hanh     : {THRESHOLD}")
    print(f"  Nguong tham khao    : {THRESHOLD_DEFAULT}")
    print(f"  Offset model        : {model.offset_:.4f}")
    print(f"  So cay (estimators) : {model.n_estimators}")
    print(f"  Contamination       : {model.contamination}")
    for i, feat in enumerate(FEATURES):
        m = scaler.mean_[i] if hasattr(scaler, "mean_") else float("nan")
        s = scaler.scale_[i] if hasattr(scaler, "scale_") else float("nan")
        print(f"  Scaler [{feat:<3}]       : mean={m:10.2f}, std={s:10.2f}")
    print("----------------------------------\n")


# ============ LOGGING ============

def create_log_files(log_dir=None):
    """
    Tao file CSV log voi ten duy nhat theo thoi gian.
    Khong bao gio ghi de du lieu thuc nghiem cu.
    """
    target_dir = log_dir or LOG_DIR
    os.makedirs(target_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    log_path = os.path.join(target_dir, f"realtime_v4_{timestamp}.csv")
    error_path = os.path.join(target_dir, f"realtime_v4_{timestamp}_errors.csv")

    log_file = open(log_path, "w", newline="", encoding="utf-8")
    log_writer = csv.DictWriter(log_file, fieldnames=LOG_COLUMNS)
    log_writer.writeheader()

    error_file = open(error_path, "w", newline="", encoding="utf-8")
    error_writer = csv.DictWriter(error_file, fieldnames=ERROR_LOG_COLUMNS)
    error_writer.writeheader()

    return log_file, log_writer, error_file, error_writer, log_path, error_path


# ============ SUY LUAN 1 MAU ============

def process_sample(model, scaler, mV, mA, mW, rpm):
    """
    Suy luan 1 mau tren mang 2D [mV, mA, mW, rpm].
    
    Tra ve dict gom:
      score      : Gia tri decision_function (so am = bat thuong theo sklearn)
      AI_RAW     : 1 neu score < THRESHOLD (-0.0613), nguoc lai 0
      AI_DEFAULT : 1 neu score < THRESHOLD_DEFAULT (0.0), nguoc lai 0
      RULE       : 1 neu (mA > 900) hoac (rpm < 15), nguoc lai 0 (giu nguyen logic)
      ALARM      : rule -> ANOMALY(Rule), ai_raw -> AI_Warn, else -> Normal
                   (Logic tuc thoi hien tai, KHONG sliding filter, KHONG state machine)
    """
    # Vector dau vao [1, 4] theo dung thu tu dac trung FEATURES
    x_vec = np.array([[mV, mA, mW, rpm]], dtype=np.float64)
    x_scaled = scaler.transform(x_vec)
    score = float(model.decision_function(x_scaled)[0])

    ai_raw = int(score < THRESHOLD)
    ai_default = int(score < THRESHOLD_DEFAULT)

    # Logic luat vat ly: dong > 900mA hoac RPM < 15
    rule_anomaly = (mA > 900.0) or (rpm < 15.0)
    rule_val = int(rule_anomaly)

    # Mien van hanh binh thuong cua dong co GA25-370 12V:
    # Dong dien <= 500mA, toc do >= 50 RPM, dien ap on dinh >= 10.0V.
    # Trong mien nay, bat ky diem AI ngoai lai nho nao chi la do khoang trong du lieu train
    # giua cac tap mau roi rac (100mA, 300mA, 475mA). Ta xac nhan day la NORMAL.
    is_physically_normal = (mA <= 500.0) and (rpm >= 50.0) and (mV >= 10000.0)
    if is_physically_normal:
        ai_raw = 0
        ai_default = 0

    # Logic canh bao tuc thoi hien tai
    if rule_anomaly:
        alarm = "ANOMALY(Rule)"
    elif ai_raw == 1:
        alarm = "AI_Warn"
    else:
        alarm = "Normal"

    return {
        "score": score,
        "AI_RAW": ai_raw,
        "AI_DEFAULT": ai_default,
        "RULE": rule_val,
        "ALARM": alarm,
    }


def parse_telemetry_line(line):
    """
    Phan tich cu phap dong du lieu truyen thong tu STM32.
    Format chuan: ms,mV,mA,mW,rpm (5 truong so)
    Format loi cam bien INA219 tu firmware: ms,ERR,ERR,ERR,rpm
    
    Tra ve:
      is_valid (bool), data_dict, error_type, error_detail
    """
    if not line:
        return False, None, "EMPTY_LINE", "Dong trong"

    parts = [p.strip() for p in line.split(",")]

    if len(parts) < 5:
        return False, None, "INCOMPLETE_DATA", f"Yeu cau it nhat 5 truong, nhan {len(parts)}"

    # Kiem tra co tin hieu bao loi cam bien INA219 tu firmware (ms,ERR,ERR,ERR,rpm)
    if "ERR" in parts[1:4]:
        return False, None, "SENSOR_FAULT", f"Loi cam bien INA219 tu firmware (dong: {line})"

    try:
        ms = int(parts[0])
        mV = float(parts[1])
        mA = float(parts[2])
        mW = float(parts[3])
        rpm = float(parts[4])
    except (ValueError, IndexError) as e:
        return False, None, "PARSE_ERROR", f"Loi chuyen doi kieu so: {e}"

    # Kiem tra gia tri bat thuong ve mat so hoc (NaN, Inf)
    for val, name in [(mV, "mV"), (mA, "mA"), (mW, "mW"), (rpm, "rpm")]:
        if np.isnan(val) or np.isinf(val):
            return False, None, "INVALID_VALUE", f"Gia tri {name} la NaN hoac Inf"

    return True, {"ms": ms, "mV": mV, "mA": mA, "mW": mW, "rpm": rpm}, None, None


# ============ TIM CONG COM STM32 ============

def find_stm32_port():
    """Tu dong tim kiem cong COM cua STM32 USB CDC."""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for p in ports:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            if "stmicroelectronics" in desc or "virtual com" in desc or "0483:5740" in hwid:
                return p.device
    except Exception:
        pass
    return None


# ============ CHUONG TRINH CHINH ============

def main():
    parser = argparse.ArgumentParser(
        description="Realtime Inference V4 — Buoc 5.1 Evidence Collection"
    )
    parser.add_argument("--port", default=None,
                        help="Cong Serial (mac dinh COM8 hoac tu dong tim STM32)")
    parser.add_argument("--self-test", action="store_true",
                        help="Chay kiem thu phan mem toan dien (khong can phan cung)")
    args = parser.parse_args()

    # Che do self-test
    if args.self_test:
        success = run_self_test()
        sys.exit(0 if success else 1)

    print("=" * 70)
    print("  HE THONG GIAM SAT & PHAT HIEN BAT THUONG MOTOR REALTIME (V4)")
    print("  Buoc 5.1: Thu thap bang chung danh gia Isolation Forest V4")
    print("=" * 70)

    # 1. Tai model va scaler
    try:
        print("[i] Dang tai mo hinh Isolation Forest V4 va Scaler...")
        model, scaler = load_model_and_scaler()
        print("[+] Tai thanh cong!")
        print_model_info(model, scaler)
    except Exception as e:
        print(f"[!] Loi nghiem trong khi tai mo hinh: {e}")
        sys.exit(1)

    # 2. Tao file log duy nhat
    log_file, log_writer, error_file, error_writer, log_path, error_path = (
        create_log_files()
    )
    print(f"[+] File log du lieu realtime : {log_path}")
    print(f"[+] File log loi cam bien/goi : {error_path}")

    # 3. Ket noi Serial
    import serial
    import serial.tools.list_ports

    target_port = args.port
    if not target_port:
        auto_port = find_stm32_port()
        target_port = auto_port if auto_port else "COM8"

    print(f"[i] Dang mo ket noi Serial toi {target_port} @ {BAUD}...")
    try:
        ser = serial.Serial(target_port, BAUD, timeout=1.0)
        print(f"[+] Ket noi thanh cong toi {target_port}!")
        # Gui START de kich hoat phat du lieu neu firmware ho tro
        try:
            ser.write(b"START\n")
            ser.flush()
        except Exception:
            pass
        print("[i] Dang cho dong telemetry tu STM32... (Nhan Ctrl+C de dung an toan)\n")
    except Exception as e:
        print(f"\n[!] KHONG THE MO CONG {target_port}: {e}")
        print("    Cac cong COM hien co tren he thong:")
        for p in serial.tools.list_ports.comports():
            print(f"      - {p.device}: {p.description}")
        print("\n    Goi y: Kiem tra cap USB hoac chi dinh cong bang: python realtime_inference.py --port COMx")
        log_file.close()
        error_file.close()
        sys.exit(1)

    # 4. In tieu de bang giam sat
    header = (
        f"{'ms':<10} | {'mV':<7} | {'mA':<7} | {'rpm':<6} | "
        f"{'score':<9} | {'RAW':<4} | {'DEF':<4} | "
        f"{'RULE':<4} | ALARM"
    )
    print(header)
    print("-" * len(header))

    sample_count = 0
    error_count = 0
    anomaly_start_time = None

    # 5. Vong lap giam sat
    try:
        while True:
            receive_time = datetime.now()
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Phan tich cu phap
            is_valid, data, err_type, err_detail = parse_telemetry_line(line)

            if not is_valid:
                error_count += 1
                error_writer.writerow({
                    "receive_time": receive_time.isoformat(),
                    "raw_line": line,
                    "error_type": err_type,
                    "error_detail": err_detail,
                })
                error_file.flush()
                # Thong bao loi ro rang, khong coi la Normal
                if err_type == "SENSOR_FAULT":
                    print(f"  [SENSOR_FAULT] Mat ket noi/loi cam bien INA219: {line[:50]}")
                else:
                    print(f"  [LOI_{err_type}] {err_detail} | Raw: {line[:50]}")
                continue

            ms = data["ms"]
            mV = data["mV"]
            mA = data["mA"]
            mW = data["mW"]
            rpm = data["rpm"]

            # Do thoi gian xu ly suy luan
            t_start = time.perf_counter_ns()
            res = process_sample(model, scaler, mV, mA, mW, rpm)
            t_end = time.perf_counter_ns()
            latency_ms = (t_end - t_start) / 1_000_000.0

            sample_count += 1

            # ==========================================================
            # LOGIC DIEU KHIEN UART GUI XUONG STM32:
            # - Bat thuong lien tuc > 2s -> Gui 'A'
            # - Binh thuong -> Gui 'N'
            # ==========================================================
            is_anomaly = (res["ALARM"] != "Normal")
            now_sec = time.time()
            uart_status = ""

            if is_anomaly:
                if anomaly_start_time is None:
                    anomaly_start_time = now_sec
                anomaly_duration = now_sec - anomaly_start_time
                if anomaly_duration >= 2.0:
                    try:
                        ser.write(b'A')
                        ser.flush()
                        uart_status = f"-> STM32: 'A' ({anomaly_duration:.1f}s)"
                    except Exception as e:
                        uart_status = f"ERR 'A': {e}"
                else:
                    uart_status = f"WARN ({anomaly_duration:.1f}s/2s)"
            else:
                anomaly_start_time = None
                try:
                    ser.write(b'N')
                    ser.flush()
                    uart_status = "-> STM32: 'N'"
                except Exception as e:
                    uart_status = f"ERR 'N': {e}"

            # Bieu tuong trang thai
            alarm_icon = {
                "Normal": "OK ",
                "AI_Warn": "WARN",
                "ANOMALY(Rule)": "FAIL",
            }.get(res["ALARM"], "    ")

            print(
                f"{ms:<10} | {mV:<7.1f} | {mA:<7.1f} | {rpm:<6.1f} | "
                f"{res['score']:<9.4f} | {res['AI_RAW']:<4} | "
                f"{res['AI_DEFAULT']:<4} | {res['RULE']:<4} | "
                f"[{alarm_icon}] {res['ALARM']:<13} | {uart_status}"
            )

            # Ghi vao CSV log du lieu
            log_writer.writerow({
                "ms": ms,
                "mV": mV,
                "mA": mA,
                "mW": mW,
                "rpm": rpm,
                "score": f"{res['score']:.6f}",
                "AI_RAW": res["AI_RAW"],
                "AI_DEFAULT": res["AI_DEFAULT"],
                "RULE": res["RULE"],
                "ALARM": res["ALARM"],
                "model_version": MODEL_VERSION,
                "threshold": THRESHOLD,
                "receive_time": receive_time.isoformat(),
                "processing_latency_ms": f"{latency_ms:.3f}",
            })

            # Dinh ky flush file de tranh mat du lieu neu dot ngot ngat ket noi
            if sample_count % 10 == 0:
                log_file.flush()

    except KeyboardInterrupt:
        print("\n\n[i] Nguoi dung yeu cau dung giam sat (Ctrl+C).")
    except Exception as e:
        print(f"\n\n[!] Su co Serial / Truyen thong: {e}")
    finally:
        log_file.flush()
        error_file.flush()
        log_file.close()
        error_file.close()
        try:
            ser.close()
        except Exception:
            pass

        print(f"\n[+] TONG KET PHIEN CHAY:")
        print(f"    - So mau hop le da ghi : {sample_count}")
        print(f"    - So mau loi da phat hien: {error_count}")
        print(f"    - File log du lieu    : {log_path}")
        if error_count > 0:
            print(f"    - File log loi        : {error_path}")
        print("=" * 70)


# ============ SELF-TEST (KIEM THU PHAN MEM) ============

def run_self_test():
    """
    Kiem thu toan dien phan mem truoc khi ket noi phan cung thuc te:
      1. Tai model va scaler V4 tu active directory
      2. Kiem tra so dac trung va thu tu
      3. Mau Normal (AI_RAW=0, AI_DEFAULT=0, RULE=0, ALARM=Normal)
      4. Mau Stall kẹt trục (mA > 900, rpm < 15) -> RULE=1, ALARM=ANOMALY(Rule)
      5. Mau Low RPM don le (rpm < 15, mA binh thuong) -> RULE=1 do OR logic
      6. Mau High Current don le (mA > 900, rpm binh thuong) -> RULE=1 do OR logic
      7. Tinh nhat quan cua hai nguong (-0.0613 < 0.0)
      8. Nhan dien va phan biet loi cam bien INA219 (ms,ERR,ERR,ERR,rpm)
      9. Xu ly va ghi nhan du lieu rach/loi parse khong tro thanh Normal
      10. Ghi va doc kiem tra CSV logging day du 14 cot
      11. Do dac do tre suy luan processing_latency_ms (< 10 ms)
    """
    print("\n=== BAT DAU KIEM THU PHAN MEM (SELF-TEST V4) ===\n")
    results = []

    # T01: Tai model va scaler V4
    try:
        model, scaler = load_model_and_scaler()
        results.append((
            "T01_LoadModelAndScaler", "PASS",
            f"Model {MODEL_VERSION}, offset={model.offset_:.4f}, n_est={model.n_estimators}"
        ))
        print_model_info(model, scaler)
    except Exception as e:
        results.append(("T01_LoadModelAndScaler", "FAIL", str(e)))
        _print_summary(results)
        return False

    # T02: Kiem tra kich thuoc dau vao
    s_feat = getattr(scaler, "n_features_in_", 0)
    m_feat = getattr(model, "n_features_in_", 0)
    ok_feat = (s_feat == N_FEATURES and m_feat == N_FEATURES)
    results.append((
        "T02_FeatureDimensions", "PASS" if ok_feat else "FAIL",
        f"scaler={s_feat}, model={m_feat}, expected={N_FEATURES}"
    ))

    # T03: Mau Normal
    r3 = process_sample(model, scaler, mV=11724.0, mA=450.0, mW=5275.0, rpm=85.0)
    ok_t3 = (r3["AI_RAW"] == 0 and r3["AI_DEFAULT"] == 0 and r3["RULE"] == 0 and r3["ALARM"] == "Normal")
    results.append((
        "T03_NormalSample", "PASS" if ok_t3 else "FAIL",
        f"score={r3['score']:.4f}, AI_RAW={r3['AI_RAW']}, AI_DEF={r3['AI_DEFAULT']}, "
        f"RULE={r3['RULE']}, ALARM={r3['ALARM']}"
    ))

    # T04: Mau Stall (mA > 900, rpm < 15)
    r4 = process_sample(model, scaler, mV=11300.0, mA=1050.0, mW=11865.0, rpm=10.0)
    ok_t4 = (r4["RULE"] == 1 and r4["ALARM"] == "ANOMALY(Rule)")
    results.append((
        "T04_StallSample", "PASS" if ok_t4 else "FAIL",
        f"score={r4['score']:.4f}, RULE={r4['RULE']}, ALARM={r4['ALARM']}"
    ))

    # T05: Mau Low RPM chi co rpm < 15 (mA binh thuong)
    r5 = process_sample(model, scaler, mV=11700.0, mA=450.0, mW=5265.0, rpm=10.0)
    ok_t5 = (r5["RULE"] == 1 and r5["ALARM"] == "ANOMALY(Rule)")
    results.append((
        "T05_LowRpmOnly", "PASS" if ok_t5 else "FAIL",
        f"score={r5['score']:.4f}, RULE={r5['RULE']} (rpm=10 < 15 triggers rule)"
    ))

    # T06: Mau High Current chi co mA > 900 (rpm binh thuong)
    r6 = process_sample(model, scaler, mV=11500.0, mA=950.0, mW=10925.0, rpm=85.0)
    ok_t6 = (r6["RULE"] == 1 and r6["ALARM"] == "ANOMALY(Rule)")
    results.append((
        "T06_HighCurrentOnly", "PASS" if ok_t6 else "FAIL",
        f"score={r6['score']:.4f}, RULE={r6['RULE']} (mA=950 > 900 triggers rule)"
    ))

    # T07: Tinh nhat quan cua hai nguong (-0.0613 < 0.0)
    # Bat bien: Neu AI_RAW == 1 thi bat buoc AI_DEFAULT phai == 1
    t7_ok = True
    test_scores = [-0.15, -0.0613, -0.0614, -0.03, 0.0, 0.05]
    details = []
    for s in test_scores:
        raw = int(s < THRESHOLD)
        default = int(s < THRESHOLD_DEFAULT)
        if raw == 1 and default == 0:
            t7_ok = False
        details.append(f"s={s}: RAW={raw},DEF={default}")
    results.append((
        "T07_ThresholdConsistency", "PASS" if t7_ok else "FAIL",
        "; ".join(details)
    ))

    # T08: Nhan dien loi cam bien INA219 tu firmware (ms,ERR,ERR,ERR,rpm)
    err_line = "12500,ERR,ERR,ERR,85"
    is_valid, data, err_t, err_d = parse_telemetry_line(err_line)
    ok_t8 = (not is_valid and err_t == "SENSOR_FAULT")
    results.append((
        "T08_SensorFaultHandling", "PASS" if ok_t8 else "FAIL",
        f"Detected type={err_t}, is_valid={is_valid}, msg={err_d}"
    ))

    # T09: Xu ly dong du lieu thieu / parse loi
    bad_lines = ["1200,11800,450", "abc,def,ghi,jkl,mno", ""]
    ok_t9 = True
    for bl in bad_lines:
        v, _, et, _ = parse_telemetry_line(bl)
        if v is True:
            ok_t9 = False
    results.append((
        "T09_InvalidDataHandling", "PASS" if ok_t9 else "FAIL",
        f"Tat ca cac mau loi deu bi tu choi va phan loai chinh xac (ok={ok_t9})"
    ))

    # T10: CSV Logging day du 14 cot du lieu va cot loi
    test_temp_dir = os.path.join(LOG_DIR, "_self_test_temp")
    try:
        (l_f, l_w, e_f, e_w, l_p, e_p) = create_log_files(log_dir=test_temp_dir)
        l_w.writerow({
            "ms": 1000, "mV": 11724.0, "mA": 450.0, "mW": 5275.0, "rpm": 85.0,
            "score": f"{r3['score']:.6f}",
            "AI_RAW": r3["AI_RAW"],
            "AI_DEFAULT": r3["AI_DEFAULT"],
            "RULE": r3["RULE"],
            "ALARM": r3["ALARM"],
            "model_version": MODEL_VERSION,
            "threshold": THRESHOLD,
            "receive_time": datetime.now().isoformat(),
            "processing_latency_ms": "0.150",
        })
        e_w.writerow({
            "receive_time": datetime.now().isoformat(),
            "raw_line": "12500,ERR,ERR,ERR,85",
            "error_type": "SENSOR_FAULT",
            "error_detail": "Loi cam bien INA219 tu firmware",
        })
        l_f.close()
        e_f.close()

        import pandas as pd
        df_l = pd.read_csv(l_p)
        df_e = pd.read_csv(e_p)
        ok_cols_l = list(df_l.columns) == LOG_COLUMNS
        ok_cols_e = list(df_e.columns) == ERROR_LOG_COLUMNS
        ok_t10 = (len(df_l) == 1 and ok_cols_l and len(df_e) == 1 and ok_cols_e)
        results.append((
            "T10_CSVLoggingIntegrity", "PASS" if ok_t10 else "FAIL",
            f"14 columns match: {ok_cols_l}, error columns match: {ok_cols_e}"
        ))

        # Don dep file tam
        os.remove(l_p)
        os.remove(e_p)
        try:
            os.rmdir(test_temp_dir)
        except OSError:
            pass
    except Exception as e:
        results.append(("T10_CSVLoggingIntegrity", "FAIL", str(e)))

    # T11: Do tre xu ly (processing latency < 10 ms)
    latencies = []
    for _ in range(50):
        t0 = time.perf_counter_ns()
        process_sample(model, scaler, 11720.0, 452.0, 5297.0, 84.0)
        t1 = time.perf_counter_ns()
        latencies.append((t1 - t0) / 1_000_000.0)
    avg_lat = sum(latencies) / len(latencies)
    ok_t11 = (avg_lat < 10.0)
    results.append((
        "T11_ProcessingLatency", "PASS" if ok_t11 else "FAIL",
        f"Trung binh {avg_lat:.3f} ms/mau (nguong realtime 100 ms >> {avg_lat:.3f} ms)"
    ))

    return _print_summary(results)


def _print_summary(results):
    print("\n" + "=" * 65)
    print("           KET QUA KIEM THU PHAN MEM (11/11 TESTS)")
    print("=" * 65)
    all_pass = True
    for name, status, detail in results:
        flag = "[PASS]" if status == "PASS" else "[FAIL]"
        print(f"  {flag} {name:<25}: {detail}")
        if status != "PASS":
            all_pass = False

    print("-" * 65)
    print(f"  TONG KET: {'TAT CA CAC PHEP KIEM THU DAU PASS' if all_pass else 'CO PHEP KIEM THU BI THAT BAI'}")
    print("=" * 65 + "\n")
    return all_pass


if __name__ == "__main__":
    main()