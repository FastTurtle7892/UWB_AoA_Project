"""
UWB AoA RTLS 실시간 모니터링 시스템
- 데이터 재생(Playback) + 실시간 시리얼 수신 지원
- Kalman 필터 / 이동평균 / Raw 데이터 전환
- 궤적 표시, CSV 저장
"""

import sys
import csv
import math
import numpy as np
from collections import deque
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QSlider, QFileDialog,
    QRadioButton, QButtonGroup, QSpinBox, QStatusBar
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont
import pyqtgraph as pg
import pandas as pd
import serial
import serial.tools.list_ports


# ──────────────────────────────────────────────
# 1. Kalman Filter (1D, position-only)
# ──────────────────────────────────────────────
class KalmanFilter1D:
    def __init__(self, q=0.01, r=0.5):
        self.q = q   # process noise
        self.r = r   # measurement noise
        self.x = 0.0
        self.p = 1.0

    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x

    def reset(self):
        self.x = 0.0
        self.p = 1.0


# ──────────────────────────────────────────────
# 2. Serial Worker (QThread)
# ──────────────────────────────────────────────
class SerialWorker(QThread):
    """
    실시간 시리얼 수신 스레드
    데이터 형식: "D...I{dist}P{aoa}O..."  (MATLAB 코드 기준)
    """
    data_received = pyqtSignal(float, float)   # x, y
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baud=250000):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = False

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            self._running = True
            while self._running:
                if ser.in_waiting > 0:
                    raw = ser.readline().decode('utf-8', errors='ignore').strip()
                    x, y = self._parse(raw)
                    if x is not None:
                        self.data_received.emit(x, y)
            ser.close()
        except serial.SerialException as e:
            self.error_occurred.emit(str(e))

    def _parse(self, line):
        """
        "D...I{dist}P{aoa}O..." 형식 파싱 → (x, y) 변환
        """
        try:
            if 'I' not in line or 'P' not in line or 'O' not in line:
                return None, None
            dist_str = line.split('I')[1].split('P')[0]
            aoa_str  = line.split('P')[1].split('O')[0]
            dist = float(dist_str)
            aoa  = float(aoa_str)
            x = dist * math.sin(math.radians(aoa))
            y = dist * math.cos(math.radians(aoa))
            return x, y
        except Exception:
            return None, None

    def stop(self):
        self._running = False
        self.wait()


# ──────────────────────────────────────────────
# 3. Main Window
# ──────────────────────────────────────────────
class UWBMonitor(QMainWindow):
    # 트랙 스펙 (포스터 기준)
    TRACK_W  = 1.8   # m
    TRACK_H  = 1.8   # m
    ANCHOR_Y = 0.0   # 앵커 원점
    TRACK_Y0 = 2.7   # 트랙 근접 끝까지 거리

    TRAIL_LEN = 80   # 궤적 보존 포인트 수
    PLAYBACK_INTERVAL_MS = 50   # 재생 속도 기본값

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UWB AoA RTLS 모니터링")
        self.setMinimumSize(1100, 700)

        # 상태 변수
        self.raw_data   = []       # 전체 로드된 데이터
        self.play_idx   = 0
        self.is_playing = False
        self.serial_worker = None
        self.log_data   = []       # CSV 저장용

        # 필터
        self.kf_x = KalmanFilter1D(q=0.01, r=0.5)
        self.kf_y = KalmanFilter1D(q=0.01, r=0.5)
        self.trail_raw      = deque(maxlen=self.TRAIL_LEN)
        self.trail_filtered = deque(maxlen=self.TRAIL_LEN)
        self.ma_window      = deque(maxlen=8)   # 이동평균

        self._build_ui()
        self._setup_plot()

    # ── UI 구성 ──────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # 왼쪽: 플롯
        self.plot_widget = pg.PlotWidget(title="2D 위치 추적")
        self.plot_widget.setBackground('#1e1e1e')
        self.plot_widget.setMinimumWidth(600)
        root.addWidget(self.plot_widget, stretch=3)

        # 오른쪽: 컨트롤 패널
        panel = QVBoxLayout()
        panel.setSpacing(10)
        root.addLayout(panel, stretch=1)

        # ── 데이터 소스 ──
        src_box = QGroupBox("데이터 소스")
        src_lay = QVBoxLayout(src_box)

        self.btn_load = QPushButton("📂  CSV / Excel 불러오기")
        self.btn_load.clicked.connect(self._load_file)
        src_lay.addWidget(self.btn_load)

        port_row = QHBoxLayout()
        self.combo_port = QComboBox()
        self._refresh_ports()
        self.btn_refresh = QPushButton("↺")
        self.btn_refresh.setFixedWidth(32)
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["250000", "115200", "9600"])
        port_row.addWidget(QLabel("포트"))
        port_row.addWidget(self.combo_port)
        port_row.addWidget(self.btn_refresh)
        src_lay.addLayout(port_row)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("Baud"))
        baud_row.addWidget(self.combo_baud)
        src_lay.addLayout(baud_row)

        self.btn_serial = QPushButton("▶  시리얼 시작")
        self.btn_serial.clicked.connect(self._toggle_serial)
        src_lay.addWidget(self.btn_serial)
        panel.addWidget(src_box)

        # ── 재생 컨트롤 ──
        play_box = QGroupBox("재생 (Playback)")
        play_lay = QVBoxLayout(play_box)

        btn_row = QHBoxLayout()
        self.btn_play = QPushButton("▶  Play")
        self.btn_play.setEnabled(False)
        self.btn_play.clicked.connect(self._toggle_play)
        self.btn_reset = QPushButton("⏮  Reset")
        self.btn_reset.setEnabled(False)
        self.btn_reset.clicked.connect(self._reset_play)
        btn_row.addWidget(self.btn_play)
        btn_row.addWidget(self.btn_reset)
        play_lay.addLayout(btn_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("속도"))
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(1, 10)
        self.slider_speed.setValue(5)
        self.slider_speed.setTickInterval(1)
        self.slider_speed.valueChanged.connect(self._update_speed)
        self.label_speed = QLabel("×1.0")
        speed_row.addWidget(self.slider_speed)
        speed_row.addWidget(self.label_speed)
        play_lay.addLayout(speed_row)

        self.label_progress = QLabel("0 / 0")
        self.label_progress.setAlignment(Qt.AlignCenter)
        play_lay.addWidget(self.label_progress)
        panel.addWidget(play_box)

        # ── 필터 설정 ──
        filt_box = QGroupBox("필터")
        filt_lay = QVBoxLayout(filt_box)

        self.radio_raw  = QRadioButton("Raw")
        self.radio_ma   = QRadioButton("이동평균 (N=8)")
        self.radio_kf   = QRadioButton("Kalman Filter")
        self.radio_kf.setChecked(True)
        self.filt_group = QButtonGroup()
        for r in [self.radio_raw, self.radio_ma, self.radio_kf]:
            self.filt_group.addButton(r)
            filt_lay.addWidget(r)
            r.toggled.connect(self._reset_filters)

        kf_row = QHBoxLayout()
        kf_row.addWidget(QLabel("Q"))
        self.spin_q = QSpinBox()
        self.spin_q.setRange(1, 100)
        self.spin_q.setValue(1)   # ×0.01
        kf_row.addWidget(self.spin_q)
        kf_row.addWidget(QLabel("R"))
        self.spin_r = QSpinBox()
        self.spin_r.setRange(1, 100)
        self.spin_r.setValue(50)  # ×0.01
        kf_row.addWidget(self.spin_r)
        filt_lay.addLayout(kf_row)
        filt_lay.addWidget(QLabel("(Q·R: 값×0.01)"))
        panel.addWidget(filt_box)

        # ── 현재 수치 ──
        val_box = QGroupBox("현재 좌표")
        val_lay = QVBoxLayout(val_box)
        font_val = QFont("Courier", 11)
        self.lbl_x = QLabel("X :  —")
        self.lbl_y = QLabel("Y :  —")
        self.lbl_r = QLabel("R :  —")
        self.lbl_a = QLabel("θ :  —")
        for l in [self.lbl_x, self.lbl_y, self.lbl_r, self.lbl_a]:
            l.setFont(font_val)
            val_lay.addWidget(l)
        panel.addWidget(val_box)

        # ── 저장 ──
        save_box = QGroupBox("기록")
        save_lay = QVBoxLayout(save_box)
        self.btn_save = QPushButton("💾  CSV 저장")
        self.btn_save.clicked.connect(self._save_csv)
        save_lay.addWidget(self.btn_save)
        panel.addWidget(save_box)

        panel.addStretch()

        # 상태바
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("데이터를 불러오거나 시리얼 포트를 연결하세요.")

        # QTimer (재생용)
        self.timer = QTimer()
        self.timer.timeout.connect(self._step_playback)

    # ── 플롯 초기화 ───────────────────────────
    def _setup_plot(self):
        pw = self.plot_widget
        pw.setLabel('left',   '거리 (m)', color='#aaaaaa')
        pw.setLabel('bottom', 'X (m)',    color='#aaaaaa')
        pw.showGrid(x=True, y=True, alpha=0.3)
        pw.setXRange(-2.0, 2.0)
        pw.setYRange(-0.5, 5.5)
        pw.setAspectLocked(True)

        # 앵커 위치
        anchor_item = pg.ScatterPlotItem(
            [0], [0], size=14,
            brush=pg.mkBrush('#e74c3c'),
            symbol='t'
        )
        pw.addItem(anchor_item)
        pw.addItem(pg.TextItem("Anchor", color='#e74c3c', anchor=(0.5, 1.5)))

        # 트랙 테두리 (1.8×1.8m)
        x0 = -self.TRACK_W / 2
        y0 = self.TRACK_Y0
        track = pg.QtWidgets.QGraphicsRectItem(x0, y0, self.TRACK_W, self.TRACK_H)
        track.setPen(pg.mkPen('#27ae60', width=2, style=Qt.DashLine))
        pw.addItem(track)
        track_label = pg.TextItem("Track (1.8×1.8m)", color='#27ae60')
        track_label.setPos(x0, y0 - 0.1)
        pw.addItem(track_label)

        # 궤적: raw (반투명)
        self.curve_raw = pw.plot(
            pen=pg.mkPen('#5dade2', width=1, style=Qt.DotLine),
            name="Raw"
        )
        # 궤적: filtered
        self.curve_filt = pw.plot(
            pen=pg.mkPen('#f39c12', width=2),
            name="Filtered"
        )
        # 현재 포인트
        self.scatter_cur = pg.ScatterPlotItem(
            size=12, brush=pg.mkBrush('#f39c12'), symbol='o'
        )
        pw.addItem(self.scatter_cur)

        pw.addLegend()

    # ── 파일 불러오기 ─────────────────────────
    def _load_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "데이터 파일 선택", "",
            "Excel/CSV (*.xlsx *.xls *.csv)"
        )
        if not path:
            return
        try:
            if path.endswith('.csv'):
                df = pd.read_csv(path)
            else:
                df = pd.read_excel(path)

            # x, y 컬럼 확인
            df.columns = [c.lower().strip() for c in df.columns]
            if 'x' not in df.columns or 'y' not in df.columns:
                self.status.showMessage("오류: x, y 컬럼이 없습니다.")
                return

            self.raw_data = list(zip(df['x'].tolist(), df['y'].tolist()))
            self._reset_play()
            self.btn_play.setEnabled(True)
            self.btn_reset.setEnabled(True)
            self.status.showMessage(
                f"✅ {len(self.raw_data)}개 포인트 로드됨 — {path.split('/')[-1]}"
            )
        except Exception as e:
            self.status.showMessage(f"파일 오류: {e}")

    # ── 재생 제어 ─────────────────────────────
    def _toggle_play(self):
        if self.is_playing:
            self.timer.stop()
            self.is_playing = False
            self.btn_play.setText("▶  Play")
        else:
            if self.play_idx >= len(self.raw_data):
                self._reset_play()
            interval = max(10, self.PLAYBACK_INTERVAL_MS // self.slider_speed.value())
            self.timer.start(interval)
            self.is_playing = True
            self.btn_play.setText("⏸  Pause")

    def _reset_play(self):
        self.timer.stop()
        self.is_playing = False
        self.play_idx = 0
        self.btn_play.setText("▶  Play")
        self.trail_raw.clear()
        self.trail_filtered.clear()
        self.ma_window.clear()
        self.kf_x.reset()
        self.kf_y.reset()
        self.log_data.clear()
        self._refresh_plot([], [], [], [])
        self.label_progress.setText(f"0 / {len(self.raw_data)}")

    def _update_speed(self, val):
        speeds = {1: "×0.25", 2: "×0.5", 3: "×0.75", 4: "×1.0",
                  5: "×1.0", 6: "×1.5", 7: "×2.0", 8: "×3.0",
                  9: "×5.0", 10: "×10.0"}
        self.label_speed.setText(speeds.get(val, ""))
        if self.is_playing:
            interval = max(10, self.PLAYBACK_INTERVAL_MS // val)
            self.timer.setInterval(interval)

    def _step_playback(self):
        if self.play_idx >= len(self.raw_data):
            self.timer.stop()
            self.is_playing = False
            self.btn_play.setText("▶  Play")
            self.status.showMessage("재생 완료.")
            return
        x, y = self.raw_data[self.play_idx]
        self.play_idx += 1
        self._push_point(x, y)
        self.label_progress.setText(f"{self.play_idx} / {len(self.raw_data)}")

    # ── 포인트 처리 (재생 + 시리얼 공통) ──────
    def _push_point(self, x, y):
        self.trail_raw.append((x, y))

        # 필터 적용
        fx, fy = self._apply_filter(x, y)
        self.trail_filtered.append((fx, fy))

        # 로그
        r = math.sqrt(fx**2 + fy**2)
        angle = math.degrees(math.atan2(fx, fy))
        self.log_data.append({
            'timestamp': datetime.now().isoformat(timespec='milliseconds'),
            'raw_x': x, 'raw_y': y,
            'filtered_x': fx, 'filtered_y': fy,
            'distance_m': round(r, 4),
            'angle_deg': round(angle, 2)
        })

        # 수치 업데이트
        self.lbl_x.setText(f"X :  {fx:+.3f} m")
        self.lbl_y.setText(f"Y :  {fy:.3f} m")
        self.lbl_r.setText(f"R :  {r:.3f} m")
        self.lbl_a.setText(f"θ :  {angle:+.1f}°")

        # 플롯 업데이트
        rx = [p[0] for p in self.trail_raw]
        ry = [p[1] for p in self.trail_raw]
        fx_arr = [p[0] for p in self.trail_filtered]
        fy_arr = [p[1] for p in self.trail_filtered]
        self._refresh_plot(rx, ry, fx_arr, fy_arr)

    def _apply_filter(self, x, y):
        if self.radio_raw.isChecked():
            return x, y
        elif self.radio_ma.isChecked():
            self.ma_window.append((x, y))
            fx = sum(p[0] for p in self.ma_window) / len(self.ma_window)
            fy = sum(p[1] for p in self.ma_window) / len(self.ma_window)
            return fx, fy
        else:  # Kalman
            q = self.spin_q.value() * 0.01
            r = self.spin_r.value() * 0.01
            self.kf_x.q = q; self.kf_x.r = r
            self.kf_y.q = q; self.kf_y.r = r
            return self.kf_x.update(x), self.kf_y.update(y)

    def _reset_filters(self):
        self.kf_x.reset()
        self.kf_y.reset()
        self.ma_window.clear()
        self.trail_filtered.clear()

    def _refresh_plot(self, rx, ry, fx, fy):
        self.curve_raw.setData(rx, ry)
        self.curve_filt.setData(fx, fy)
        if fx:
            self.scatter_cur.setData([fx[-1]], [fy[-1]])

    # ── 시리얼 ───────────────────────────────
    def _refresh_ports(self):
        self.combo_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port.addItems(ports if ports else ["(없음)"])

    def _toggle_serial(self):
        if self.serial_worker and self.serial_worker.isRunning():
            self.serial_worker.stop()
            self.btn_serial.setText("▶  시리얼 시작")
            self.status.showMessage("시리얼 연결 해제.")
        else:
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            self.serial_worker = SerialWorker(port, baud)
            self.serial_worker.data_received.connect(self._push_point)
            self.serial_worker.error_occurred.connect(
                lambda e: self.status.showMessage(f"시리얼 오류: {e}")
            )
            self.serial_worker.start()
            self.btn_serial.setText("⏹  시리얼 중지")
            self.status.showMessage(f"시리얼 연결됨: {port} @ {baud}")

    # ── CSV 저장 ─────────────────────────────
    def _save_csv(self):
        if not self.log_data:
            self.status.showMessage("저장할 데이터가 없습니다.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "CSV 저장", f"uwb_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV (*.csv)"
        )
        if not path:
            return
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=self.log_data[0].keys())
            writer.writeheader()
            writer.writerows(self.log_data)
        self.status.showMessage(f"✅ 저장 완료: {path}")

    def closeEvent(self, event):
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()


# ──────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = UWBMonitor()
    win.show()
    sys.exit(app.exec_())
