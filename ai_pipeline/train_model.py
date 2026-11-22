import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
import joblib
import os
import sys

# ============ CẤU HÌNH ĐƯỜNG DẪN ============
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "data"))
DATAMAIN_DIR = os.path.join(DATA_DIR, "datamain")
MODEL_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "models", "active"))
os.makedirs(MODEL_DIR, exist_ok=True)

# Đường dẫn file dữ liệu chính trong thư mục datamain
TRAIN_FILE = os.path.join(DATAMAIN_DIR, "train_data.csv")
TEST_FILE = os.path.join(DATAMAIN_DIR, "test_data.csv")

# ============ 1. ĐỌC VÀ CHUẨN BỊ DỮ LIỆU ============
if not os.path.exists(TRAIN_FILE):
    print(f"[!] LỖI: Không tìm thấy file dữ liệu train tại:\n    {TRAIN_FILE}")
    sys.exit(1)

print(f"[i] Đang đọc dữ liệu Train từ: {TRAIN_FILE}")
df_train = pd.read_csv(TRAIN_FILE)

features = ['mV', 'mA', 'mW', 'rpm']
X_train_normal = df_train[features].values

# Đọc tập kiểm định Test nếu có
if os.path.exists(TEST_FILE):
    print(f"[i] Đang đọc dữ liệu Test từ: {TEST_FILE}")
    df_test = pd.read_csv(TEST_FILE)
    X_test = df_test[features].values
    y_test = df_test['label'].values
else:
    # Nếu không có file test riêng, chia từ train
    X_train_normal, X_test = train_test_split(X_train_normal, test_size=0.2, random_state=42)
    y_test = np.zeros(len(X_test))

print(f" - Số mẫu Train (chỉ Normal): {len(X_train_normal):,}")
print(f" - Số mẫu Test (Normal + Anomaly): {len(X_test):,}")

# ============ 2. CHUẨN HÓA DỮ LIỆU ============
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train_normal)
X_test_scaled = scaler.transform(X_test)

# ============ 3. HUẤN LUYỆN MÔ HÌNH V4 ============
print("\n[i] Đang huấn luyện mô hình Isolation Forest V4 trên dữ liệu datamain mới...")
model = IsolationForest(n_estimators=200, contamination=0.01, random_state=42, n_jobs=-1)
model.fit(X_train_scaled)

# ============ 4. ĐÁNH GIÁ MÔ HÌNH ============
scores = model.decision_function(X_test_scaled)

# Đánh giá với ngưỡng mặc định 0.0
y_pred_def = np.where(scores < 0.0, 1, 0)
print("\n=== KẾT QUẢ ĐÁNH GIÁ MÔ HÌNH V4 (Ngưỡng mặc định 0.0) ===")
print(classification_report(y_test, y_pred_def, target_names=['Normal (0)', 'Anomaly (1)']))

# Đánh giá với ngưỡng tối ưu vận hành -0.0613
y_pred_tuned = np.where(scores < -0.0613, 1, 0)
print("\n=== KẾT QUẢ ĐÁNH GIÁ MÔ HÌNH V4 (Ngưỡng vận hành -0.0613) ===")
print(classification_report(y_test, y_pred_tuned, target_names=['Normal (0)', 'Anomaly (1)']))

# ============ 5. LƯU MÔ HÌNH ============
model_path = os.path.join(MODEL_DIR, "isolation_forest_v4.pkl")
scaler_path = os.path.join(MODEL_DIR, "scaler_v4.pkl")

joblib.dump(model, model_path)
joblib.dump(scaler, scaler_path)

print(f"[+] Đã lưu mô hình V4 tại: {model_path}")
print(f"[+] Đã lưu scaler V4 tại: {scaler_path}")