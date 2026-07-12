"""
Moving Average Filter 자동 파라미터 튜닝
- Window size W 탐색 → RMSE 최소 W 탐색
- 트랙(1.8×1.8m 정사각형)을 Ground Truth로 사용
- 결과: RMSE vs W 그래프 + 최적 파라미터 JSON 저장
"""

import json
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.rcParams['font.family'] = 'Malgun Gothic'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
from collections import deque

# ── 설정 ──────────────────────────────────────
DATA_FILE    = "monitoring/plot_data3.xlsx"
OUTPUT_DIR   = "monitoring/result_img"
RESULT_IMG   = os.path.join(OUTPUT_DIR, "ma_tuning_result.png")
BEST_PARAMS  = os.path.join(OUTPUT_DIR, "best_params.json")

# 트랙 스펙 (포스터 기준, 단위: m)
TRACK_X0, TRACK_X1 = -0.9,  0.9
TRACK_Y0, TRACK_Y1 =  2.7,  4.5

# 탐색 범위: window size 1 ~ 50
W_VALUES = list(range(1, 51))


# ── Moving Average Filter ─────────────────────
class MovingAverage:
    def __init__(self, w):
        self.w = w
        self.buf = deque(maxlen=w)

    def update(self, z):
        self.buf.append(z)
        return sum(self.buf) / len(self.buf)


# ── Ground Truth: 정사각형 트랙 위 최근접 거리 ──
def dist_to_track(x, y):
    segments = [
        ((TRACK_X0, TRACK_Y0), (TRACK_X1, TRACK_Y0)),
        ((TRACK_X1, TRACK_Y0), (TRACK_X1, TRACK_Y1)),
        ((TRACK_X1, TRACK_Y1), (TRACK_X0, TRACK_Y1)),
        ((TRACK_X0, TRACK_Y1), (TRACK_X0, TRACK_Y0)),
    ]
    min_d = float('inf')
    for (ax, ay), (bx, by) in segments:
        dx, dy = bx - ax, by - ay
        t = max(0, min(1, ((x - ax)*dx + (y - ay)*dy) / (dx*dx + dy*dy)))
        px, py = ax + t*dx, ay + t*dy
        d = np.sqrt((x - px)**2 + (y - py)**2)
        min_d = min(min_d, d)
    return min_d


def calc_rmse(xs, ys):
    errors = [dist_to_track(x, y) for x, y in zip(xs, ys)]
    return np.sqrt(np.mean(np.array(errors)**2))


def apply_ma_and_rmse(raw_x, raw_y, w):
    ma_x = MovingAverage(w)
    ma_y = MovingAverage(w)
    fx = [ma_x.update(x) for x in raw_x]
    fy = [ma_y.update(y) for y in raw_y]
    return calc_rmse(fx, fy), fx, fy


# ── 메인 ─────────────────────────────────────
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    df = pd.read_excel(DATA_FILE)
    df.columns = [c.lower().strip() for c in df.columns]
    raw_x = df['x'].tolist()
    raw_y = df['y'].tolist()

    raw_rmse = calc_rmse(raw_x, raw_y)
    print(f"Raw RMSE     : {raw_rmse*100:.2f} cm")
    print(f"\n탐색 시작: W = 1 ~ {max(W_VALUES)}...")

    rmse_list = []
    best_rmse = float('inf')
    best_w = None
    best_fx, best_fy = None, None

    for w in W_VALUES:
        rmse, fx, fy = apply_ma_and_rmse(raw_x, raw_y, w)
        rmse_list.append(rmse)
        if rmse < best_rmse:
            best_rmse = rmse
            best_w = w
            best_fx, best_fy = fx, fy

    print(f"\n{'='*40}")
    print(f"최적 Window  : W = {best_w}")
    print(f"최적 RMSE    : {best_rmse*100:.2f} cm")
    print(f"개선율       : {(1 - best_rmse/raw_rmse)*100:.1f}%")
    print(f"{'='*40}")

    # ── 기존 best_params.json 로드 후 업데이트 ──
    if os.path.exists(BEST_PARAMS):
        with open(BEST_PARAMS) as f:
            params = json.load(f)
    else:
        params = {}

    params["moving_average"] = {
        "best_W": best_w,
        "best_rmse_cm": round(best_rmse * 100, 2),
        "raw_rmse_cm":  round(raw_rmse  * 100, 2),
        "improvement_pct": round((1 - best_rmse / raw_rmse) * 100, 1),
    }
    with open(BEST_PARAMS, 'w') as f:
        json.dump(params, f, indent=2, ensure_ascii=False)
    print(f"파라미터 저장: {BEST_PARAMS}")

    # ── 시각화 ──────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Moving Average Filter 자동 파라미터 튜닝 결과", fontsize=13)

    # 1) RMSE vs W
    ax = axes[0]
    ax.plot(W_VALUES, [r * 100 for r in rmse_list], color='#3498db', linewidth=2)
    ax.axvline(best_w, color='red', linestyle='--', linewidth=1.5,
               label=f'최적 W={best_w}')
    ax.axhline(raw_rmse * 100, color='gray', linestyle=':', linewidth=1.5,
               label=f'Raw ({raw_rmse*100:.1f}cm)')
    ax.set_xlabel('Window Size (W)')
    ax.set_ylabel('RMSE (cm)')
    ax.set_title('Window Size별 RMSE')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # 2) 궤적 비교
    ax = axes[1]
    track_x = [TRACK_X0, TRACK_X1, TRACK_X1, TRACK_X0, TRACK_X0]
    track_y = [TRACK_Y0, TRACK_Y0, TRACK_Y1, TRACK_Y1, TRACK_Y0]
    ax.plot(track_x, track_y, 'g--', linewidth=2, label='이상 트랙')
    ax.plot(raw_x, raw_y, color='#5dade2', alpha=0.5,
            linewidth=1, linestyle=':', label=f'Raw ({raw_rmse*100:.1f}cm)')
    ax.plot(best_fx, best_fy, color='#e74c3c',
            linewidth=2, label=f'MA W={best_w} ({best_rmse*100:.1f}cm)')
    ax.plot(0, 0, 'rv', markersize=10, label='Anchor')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('궤적 비교')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 3) 오차 분포
    ax = axes[2]
    raw_errors = [dist_to_track(x, y) * 100 for x, y in zip(raw_x, raw_y)]
    ma_errors  = [dist_to_track(x, y) * 100 for x, y in zip(best_fx, best_fy)]
    ax.hist(raw_errors, bins=20, alpha=0.6, color='#5dade2', label='Raw')
    ax.hist(ma_errors,  bins=20, alpha=0.6, color='#e74c3c', label=f'MA W={best_w}')
    ax.axvline(np.mean(raw_errors), color='#2980b9', linestyle='--', linewidth=1.5)
    ax.axvline(np.mean(ma_errors),  color='#c0392b', linestyle='--', linewidth=1.5)
    ax.set_xlabel('트랙 오차 (cm)')
    ax.set_ylabel('빈도')
    ax.set_title('오차 분포 비교')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(RESULT_IMG, dpi=150, bbox_inches='tight')
    print(f"결과 저장: {RESULT_IMG}")
    plt.show()


if __name__ == "__main__":
    main()
