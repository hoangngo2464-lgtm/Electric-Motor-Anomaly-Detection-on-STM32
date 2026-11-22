"""
alarm_filter.py
---------------
Cac ham loc canh bao va State Machine dung chung cho toan bo pipeline.

Chuc nang:
  - sliding_filter(raw, k, window)  : loc N/window causal, khong nhin tuong lai
  - state_machine(raw, on_k, off_n) : on khi >= on_k/5, off khi >= off_n lien tiep = 0
  - count_runs(values)              : dem so lan canh bao bat dau
  - alarm_stats(raw, filtered)      : tra ve dict thong ke

Khong co side effect ngoai tinh toan.
Khong phu thuoc vao file du lieu hay model.

Ket qua da xac minh (compare_v3_alarm_state_validation.py, PASS 2026-10-28):
  raw_total  = 187 mau FP tren 9000 mau Normal V3 val
  3/5 filter = 20  mau FP, 7 chuoi
  state_mach = 34  mau FP, 5 chuoi (giu canh bao qua khoang ngan)
"""

import numpy as np


# ============================================================
# 1. SLIDING / CAUSAL FILTER
# ============================================================

def sliding_filter(raw, k: int = 3, window: int = 5) -> np.ndarray:
    """
    Bo loc truot causal: tai vi tri i, neu tong window mau [i-window+1..i] >= k
    thi dat canh bao = 1.

    Params
    ------
    raw    : array-like int (0/1), dau vao tho tu AI
    k      : nguong so mau anomaly trong cua so (1 <= k <= window)
    window : do rong cua so (mac dinh 5 = 500 ms @ 10 Hz)

    Returns
    -------
    np.ndarray int (0/1)
    """
    raw = np.asarray(raw, dtype=int)
    output = np.zeros(len(raw), dtype=int)
    for i in range(len(raw)):
        start = max(0, i - window + 1)
        output[i] = int(raw[start:i + 1].sum() >= k)
    return output


# ============================================================
# 2. STATE MACHINE
# ============================================================

def state_machine(raw, on_k: int = 3, off_n: int = 3,
                  window: int = 5) -> np.ndarray:
    """
    State machine canh bao:
      - ON  : khi sliding_filter bat canh bao (tong >= on_k/window)
      - OFF : khi co off_n mau lien tiep = 0 (giam chatter khi phuc hoi)

    Params
    ------
    raw   : array-like int (0/1)
    on_k  : nguong bat canh bao (mac dinh 3)
    off_n : so mau Normal lien tiep de tat canh bao (mac dinh 3)
    window: do rong cua so bat canh bao (mac dinh 5)

    Returns
    -------
    np.ndarray int (0/1)
    """
    raw = np.asarray(raw, dtype=int)
    output = np.zeros(len(raw), dtype=int)
    alarm = False
    normal_count = 0

    for i, value in enumerate(raw):
        if not alarm:
            start = max(0, i - window + 1)
            if raw[start:i + 1].sum() >= on_k:
                alarm = True
                normal_count = 0
        else:
            if value == 0:
                normal_count += 1
            else:
                normal_count = 0
            if normal_count >= off_n:
                alarm = False
                normal_count = 0

        output[i] = int(alarm)

    return output


# ============================================================
# 3. UTILITIES
# ============================================================

def count_runs(values) -> int:
    """Dem so lan canh bao bat dau (0->1 transition)."""
    values = np.asarray(values, dtype=int)
    padded = np.r_[0, values, 0]
    return int((np.diff(padded) == 1).sum())


def alarm_stats(raw, filtered=None) -> dict:
    """
    Tra ve dict thong ke canh bao.

    Keys: active_samples, alarm_runs, transitions,
          filtered_active (neu filtered != None),
          filtered_runs, filtered_transitions
    """
    raw = np.asarray(raw, dtype=int)
    stats = {
        "active_samples": int(raw.sum()),
        "alarm_runs": count_runs(raw),
        "transitions": int(np.abs(np.diff(raw)).sum()),
    }
    if filtered is not None:
        filtered = np.asarray(filtered, dtype=int)
        stats["filtered_active"] = int(filtered.sum())
        stats["filtered_runs"] = count_runs(filtered)
        stats["filtered_transitions"] = int(
            np.abs(np.diff(filtered)).sum()
        )
    return stats


# ============================================================
# 4. SELF-TEST (chay truc tiep: python alarm_filter.py)
# ============================================================

if __name__ == "__main__":
    import pandas as pd

    # Ket qua tham khao da xac minh:
    # compare_v3_alarm_state_validation.py PASS 2026-10-28
    # sliding_3of5 total active = 20, runs = 7
    # state_machine total active = 34, runs = 5

    PATTERNS = {
        "continuous": [0]*5 + [1]*10 + [0]*5,
        "intermittent": [0]*5 + [1,0,1,0,1,1,0,1,1,1] + [0]*5,
        "short_noise": [0]*5 + [1,0,0,1,0,0] + [0]*5,
        "recovery_noise": [0]*5 + [1]*8 + [0,1,0,0,0,0]
    }

    rows = []
    for name, pattern in PATTERNS.items():
        raw = np.array(pattern, dtype=int)
        sl = sliding_filter(raw, k=3, window=5)
        sm = state_machine(raw, on_k=3, off_n=3, window=5)

        rows.append({
            "pattern": name,
            "raw_hits": int(raw.sum()),
            "sliding_active": int(sl.sum()),
            "state_active": int(sm.sum()),
            "sliding_runs": count_runs(sl),
            "state_runs": count_runs(sm),
        })

    df = pd.DataFrame(rows)
    print("\n=== alarm_filter.py SELF-TEST ===")
    print(df.to_string(index=False))

    # Kiem tra tuong thich voi ket qua da xac minh
    passed = (
        # continuous: sliding va state phai phat hien
        rows[0]["sliding_active"] > 0
        and rows[0]["state_active"] > 0
        # short_noise (1,0,0,1,0,0): 3/5 khong du -> sliding = 0
        and rows[2]["sliding_active"] == 0
    )
    print("\nKET QUA:", "PASS" if passed else "FAIL")
