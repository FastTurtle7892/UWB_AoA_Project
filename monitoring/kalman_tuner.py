"""
Kalman Filter 자동 파라미터 튜닝
- Q, R 그리드 탐색 → RMSE 최소 조합 탐색
- 트랙(1.8×1.8m 정사각형)을 Ground Truth로 사용
- 결과: 히트맵 + 최적 파라미터 출력
"""

import json
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.rcParams['font.family'] = 'Malgun Gothic'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from itertools import product

# ── 설정 ──────────────────────────────────────
DATA_FILE   = "monitoring/plot_data3.xlsx"
OUTPUT_DIR  = "monitoring/result_img"
RESULT_IMG  = os.path.join(OUTPUT_DIR, "tuning_result.png")
BEST_PARAMS = os.path.join(OUTPUT_DIR, "best_params.json")

# 트랙 스펙 (포스터 기준, 단위: m)
TRACK_X0, TRACK_X1 = -0.9,  0.9
TRACK_Y0, TRACK_Y1 =  2.7,  4.5

# 탐색 범위
Q_VALUES = np.logspace(-3, 0, 20)   # 0.001 ~ 1.0 (20단계)
R_VALUES = np.logspace(-2, 1, 20)   # 0.01  ~ 10.0 (20단계)


# ── Kalman Filter (1D) ────────────────────────
class KF1D:
    def __init__(self, q, r):
        self.q, self.r = q, r
        self.x, self.p = 0.0, 1.0

    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x


# ── Ground Truth: 정사각형 트랙 위 최근접 거리 ──
def dist_to_track(x, y):
    """
    점 (x, y)에서 정사각형 트랙 테두리까지 최단 거리
    """
    # 4개 선분에서 각각 최단 거리 계산 후 최솟값
    segments = [
        ((TRACK_X0, TRACK_Y0), (TRACK_X1, TRACK_Y0)),  # 아래
        ((TRACK_X1, TRACK_Y0), (TRACK_X1, TRACK_Y1)),  # 오른쪽
        ((TRACK_X1, TRACK_Y1), (TRACK_X0, TRACK_Y1)),  # 위
        ((TRACK_X0, TRACK_Y1), (TRACK_X0, TRACK_Y0)),  # 왼쪽
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


# ── 필터 적용 후 RMSE 계산 ────────────────────
def apply_kalman_and_rmse(raw_x, raw_y, q, r):
    kf_x = KF1D(q, r)
    kf_y = KF1D(q, r)
    fx = [kf_x.update(x) for x in raw_x]
    fy = [kf_y.update(y) for y in raw_y]
    return calc_rmse(fx, fy), fx, fy


# ── 메인 ─────────────────────────────────────
def main():
    # 데이터 로드
    df = pd.read_excel(DATA_FILE)
    df.columns = [c.lower().strip() for c in df.columns]
    raw_x = df['x'].tolist()
    raw_y = df['y'].tolist()

    raw_rmse = calc_rmse(raw_x, raw_y)
    print(f"Raw RMSE     : {raw_rmse*100:.2f} cm")

    # 그리드 탐색
    rmse_grid = np.zeros((len(Q_VALUES), len(R_VALUES)))
    best_rmse = float('inf')
    best_q, best_r = None, None
    best_fx, best_fy = None, None

    total = len(Q_VALUES) * len(R_VALUES)
    print(f"\n탐색 시작: {total}가지 조합...")

    for i, q in enumerate(Q_VALUES):
        for j, r in enumerate(R_VALUES):
            rmse, fx, fy = apply_kalman_and_rmse(raw_x, raw_y, q, r)
            rmse_grid[i, j] = rmse
            if rmse < best_rmse:
                best_rmse = rmse
                best_q, best_r = q, r
                best_fx, best_fy = fx, fy

    print(f"\n{'='*40}")
    print(f"최적 Q       : {best_q:.4f}")
    print(f"최적 R       : {best_r:.4f}")
    print(f"최적 RMSE    : {best_rmse*100:.2f} cm")
    print(f"개선율       : {(1 - best_rmse/raw_rmse)*100:.1f}%")
    print(f"{'='*40}")

    # ── best_params.json 저장 ────────────────
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    if os.path.exists(BEST_PARAMS):
        with open(BEST_PARAMS) as f:
            params = json.load(f)
    else:
        params = {}

    params["kalman"] = {
        "best_Q": round(best_q, 6),
        "best_R": round(best_r, 6),
        "best_rmse_cm": round(best_rmse * 100, 2),
        "raw_rmse_cm":  round(raw_rmse  * 100, 2),
        "improvement_pct": round((1 - best_rmse / raw_rmse) * 100, 1),
    }
    with open(BEST_PARAMS, 'w') as f:
        json.dump(params, f, indent=2, ensure_ascii=False)
    print(f"파라미터 저장: {BEST_PARAMS}")

    # ── 시각화 ──────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Kalman Filter 자동 파라미터 튜닝 결과", fontsize=13)

    # 1) RMSE 히트맵
    ax = axes[0]
    im = ax.imshow(
        rmse_grid * 100,   # cm 단위
        origin='lower',
        aspect='auto',
        cmap='RdYlGn_r',
        extent=[np.log10(R_VALUES[0]), np.log10(R_VALUES[-1]),
                np.log10(Q_VALUES[0]), np.log10(Q_VALUES[-1])]
    )
    plt.colorbar(im, ax=ax, label='RMSE (cm)')
    ax.set_xlabel('log₁₀(R)')
    ax.set_ylabel('log₁₀(Q)')
    ax.set_title('Q-R 조합별 RMSE 히트맵')

    # 최적점 표시
    ax.plot(np.log10(best_r), np.log10(best_q),
            'b*', markersize=14, label=f'최적\nQ={best_q:.3f}\nR={best_r:.3f}')
    ax.legend(fontsize=8)

    # 2) 궤적 비교 (Raw vs Kalman)
    ax = axes[1]
    # 트랙 테두리
    track_x = [TRACK_X0, TRACK_X1, TRACK_X1, TRACK_X0, TRACK_X0]
    track_y = [TRACK_Y0, TRACK_Y0, TRACK_Y1, TRACK_Y1, TRACK_Y0]
    ax.plot(track_x, track_y, 'g--', linewidth=2, label='이상 트랙')
    ax.plot(raw_x, raw_y, color='#5dade2', alpha=0.5,
            linewidth=1, linestyle=':', label=f'Raw ({raw_rmse*100:.1f}cm)')
    ax.plot(best_fx, best_fy, color='#f39c12',
            linewidth=2, label=f'Kalman ({best_rmse*100:.1f}cm)')
    ax.plot(0, 0, 'rv', markersize=10, label='Anchor')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('궤적 비교')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 3) 구간별 오차 분포
    ax = axes[2]
    raw_errors   = [dist_to_track(x, y)*100 for x, y in zip(raw_x, raw_y)]
    kf_errors    = [dist_to_track(x, y)*100 for x, y in zip(best_fx, best_fy)]
    ax.hist(raw_errors, bins=20, alpha=0.6, color='#5dade2', label='Raw')
    ax.hist(kf_errors,  bins=20, alpha=0.6, color='#f39c12', label='Kalman')
    ax.axvline(np.mean(raw_errors), color='#2980b9', linestyle='--', linewidth=1.5)
    ax.axvline(np.mean(kf_errors),  color='#e67e22', linestyle='--', linewidth=1.5)
    ax.set_xlabel('트랙 오차 (cm)')
    ax.set_ylabel('빈도')
    ax.set_title('오차 분포 비교')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(RESULT_IMG, dpi=150, bbox_inches='tight')
    print(f"\n결과 저장: {RESULT_IMG}")
    plt.show()


if __name__ == "__main__":
    main()
