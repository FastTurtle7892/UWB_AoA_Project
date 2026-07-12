"""
binary_receiver.py — UWB 바이너리 패킷 수신기
==============================================
AoA_rtls_rx_adj.c 에서 전송하는 11바이트 패킷을 수신·파싱합니다.

패킷 구조:
  [0]     0xAA         START 바이트
  [1..4]  float LE     distance (m)
  [5..8]  float LE     angle    (도)
  [9]     uint8        CRC8/SMBUS (payload = bytes [1..8])
  [10]    0x55         END 바이트
  Total: 11 바이트

CRC8 표준: CRC-8/SMBUS  poly=0x07  init=0x00
참고: embeddedrelated.com — "Help, My Serial Data Has Been Framed"
"""

import struct
import serial
import serial.tools.list_ports
import time
from collections import deque

# ── 설정 ─────────────────────────────────────────────────────────────────────
BAUD_RATE  = 115200
PKT_SIZE   = 11
PKT_START  = 0xAA
PKT_END    = 0x55

# 시뮬레이션 모드: True → COM 포트 없이 샘플 패킷으로 동작 테스트
SIMULATE   = False


# ── CRC8/SMBUS 구현 ───────────────────────────────────────────────────────────
def crc8(data: bytes) -> int:
    """
    CRC-8/SMBUS (poly=0x07, init=0x00)
    C 펌웨어 crc8() 함수와 동일한 로직
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


# ── 패킷 파서 ─────────────────────────────────────────────────────────────────
def parse_packet(raw: bytes):
    """
    11바이트 패킷을 검증하고 (distance, angle)을 반환합니다.

    Returns:
        (distance: float, angle: float)  — 정상 패킷
        None                             — CRC 오류 또는 포맷 불일치
    """
    if len(raw) != PKT_SIZE:
        return None
    if raw[0] != PKT_START or raw[10] != PKT_END:
        return None

    payload  = raw[1:9]       # 8바이트 페이로드
    received_crc = raw[9]
    computed_crc = crc8(payload)

    if received_crc != computed_crc:
        print(f"[CRC ERROR] received=0x{received_crc:02X}  computed=0x{computed_crc:02X}")
        return None

    # Little-Endian float 두 개 언팩
    distance, angle = struct.unpack('<ff', payload)
    return distance, angle


# ── 동기화 수신기 ─────────────────────────────────────────────────────────────
class BinaryReceiver:
    """
    스트림에서 헤더(0xAA)를 찾아 정렬한 뒤 11바이트 단위로 읽는 수신기.
    UART 스트림은 바이트 경계가 없으므로 헤더 탐색으로 동기화합니다.
    """

    def __init__(self, port: str):
        self.ser = serial.Serial(port, baudrate=BAUD_RATE, timeout=1.0)
        self._buf = deque()

    def _sync(self):
        """헤더 바이트 0xAA를 찾을 때까지 1바이트씩 읽어 버립니다."""
        while True:
            b = self.ser.read(1)
            if not b:
                return False
            if b[0] == PKT_START:
                return True

    def read_one(self):
        """
        동기화 후 패킷 1개를 파싱합니다.
        반환: (distance, angle) 또는 None (오류 시)
        """
        if not self._sync():
            return None
        rest = self.ser.read(PKT_SIZE - 1)
        if len(rest) < PKT_SIZE - 1:
            return None
        raw = bytes([PKT_START]) + rest
        return parse_packet(raw)

    def close(self):
        self.ser.close()


# ── 시뮬레이션 패킷 생성 ──────────────────────────────────────────────────────
def make_test_packet(distance: float, angle: float) -> bytes:
    """테스트용 패킷 생성 (C 펌웨어와 동일 로직)"""
    payload = struct.pack('<ff', distance, angle)
    checksum = crc8(payload)
    return bytes([PKT_START]) + payload + bytes([checksum, PKT_END])


# ── 포트 자동 탐색 ────────────────────────────────────────────────────────────
def find_port() -> str | None:
    """연결된 USB-UART 포트를 자동으로 찾습니다."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if 'USB' in p.description.upper() or 'UART' in p.description.upper() \
                or 'nRF' in p.description or 'JLink' in p.description:
            return p.device
    return ports[0].device if ports else None


# ── 메인 ─────────────────────────────────────────────────────────────────────
def main():
    if SIMULATE:
        # ── 시뮬레이션: 실제 포트 없이 로직 테스트 ──────────────────────────
        print("=== 시뮬레이션 모드 ===")
        test_cases = [
            (2.35, 14.7),
            (1.80, -8.3),
            (3.10,  0.0),
        ]
        for dist, ang in test_cases:
            pkt = make_test_packet(dist, ang)
            print(f"\n생성 패킷 ({len(pkt)}B): {pkt.hex(' ').upper()}")

            result = parse_packet(pkt)
            if result:
                d, a = result
                print(f"  ✓ distance={d:.3f} m  angle={a:.3f}°")
            else:
                print("  ✗ 파싱 실패")

        # CRC 오류 케이스 테스트
        print("\n=== CRC 오류 테스트 ===")
        bad_pkt = bytearray(make_test_packet(2.0, 10.0))
        bad_pkt[9] ^= 0xFF   # CRC 오염
        result = parse_packet(bytes(bad_pkt))
        print(f"  결과: {'오류 감지 ✓' if result is None else '감지 실패 ✗'}")
        return

    # ── 실제 수신 모드 ────────────────────────────────────────────────────────
    port = find_port()
    if not port:
        print("ERROR: UART 포트를 찾을 수 없습니다. 케이블을 확인하세요.")
        return

    print(f"포트: {port}  |  보레이트: {BAUD_RATE}")
    print(f"패킷 형식: [0xAA][float dist 4B][float angle 4B][CRC8][0x55] = 11B")
    print("수신 시작... (Ctrl+C 로 종료)\n")

    rx = BinaryReceiver(port)
    count = 0
    crc_errors = 0

    try:
        while True:
            result = rx.read_one()
            if result is None:
                crc_errors += 1
                print(f"[{count:05d}] CRC 오류 (누적 {crc_errors}회)")
                continue

            dist, angle = result
            count += 1
            print(f"[{count:05d}] distance={dist:6.3f} m  angle={angle:7.2f}°")

    except KeyboardInterrupt:
        print(f"\n종료. 총 수신: {count}패킷, CRC 오류: {crc_errors}회")
    finally:
        rx.close()


if __name__ == "__main__":
    main()
