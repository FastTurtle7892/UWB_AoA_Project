# AoA-Based UWB RTLS

> **Single Anchor + Single Tag 실시간 위치 추적 시스템**  
> DS-TWR 기반 거리(Ranging)와 AoA(Angle of Arrival) 기반 각도(Angle Estimation)를 융합해 **저복잡도·실시간 RTLS**를 구현

[![UWB](https://img.shields.io/badge/UWB-IEEE%20802.15.4z-blue)]()
[![Localization](https://img.shields.io/badge/Localization-AoA-green)]()
[![Award](https://img.shields.io/badge/Award-전자파학회%20동상-orange)]()

---

## 1. 개요
단일 앵커(Anchor)**와 **단일 태그(Tag)** 만으로 실시간 위치 추적(Real-Time Locating System, RTLS)을 구현한 프로젝트입니다.  
기존 삼각측량 방식이 최소 3개 이상의 앵커를 요구하는 것과 달리, **AoA(Angle of Arrival)**를 도입해 앵커 수를 줄이고 시스템 구조를 단순화**했습니다.

3인 팀 프로젝트로, 임베디드(DS-TWR·AoA 알고리즘·펌웨어) 1인, MATLAB 실시간 시각화 1인, PM 1인이 각각 담당했습니다.

---

## 2. 시스템 및 신호 흐름 비교
<div align="center">
  <table>
    <tr>
      <th align="center">삼각측량법 RTLS</th>
      <th align="center">AoA RTLS</th>
    </tr>
    <tr>
      <td align="center">
        <img src="./imgs/TRI_CAR.png" width="400" height="400" alt="삼각측량 RTLS 시스템">
      </td>
      <td align="center">
        <img src="./imgs/AOA_CAR.png" width="400" height="400" alt="AoA RTLS 시스템">
      </td>
    </tr>
    <tr>
      <td align="center">
        <img src="./imgs/TRI_MSG.png" width="400" height="700" alt="기존 RTLS 신호 흐름">
      </td>
      <td align="center">
        <img src="./imgs/AOA_MSG.png" width="400" height="700" alt="AoA RTLS 신호 흐름">
      </td>
    </tr>
  </table>
</div>

---
## 3. 핵심 기술 및 구현 방식

- **거리 측정 (Ranging)**
  - DS-TWR(Double-Sided Two-Way Ranging)을 사용하여 태그와 앵커 간의 정밀한 거리(R) 계산. 태그↔앵커 신호를 3회(Poll → Response → Final) 주고받은 왕복 시간으로 거리를 산출하며, 편도 방식(SS-TWR)과 달리 시계 오차(clock drift)를 상쇄해 더 정밀함
- **각도 측정 (Angle Estimation)**
  - UWB 안테나 배열의 위상차(Phase Difference, PDOA) 기반 AoA 계산
- **위치 추정**
  - 거리 R과 각도 $\theta$를 결합하여 2D 좌표(극좌표 → 직교좌표)를 실시간 산출
- **표준**: IEEE 802.15.4z HRP UWB
- **안테나**: XR-170 UWB 지향성 안테나

<div align="center">

  | DS-TWR (거리 측정) | AoA (각도 측정) |
  | :---: | :---: |
  | <img width="411" height="147" alt="AoA Diagram" src="https://github.com/user-attachments/assets/977d79b8-7ec6-4300-8c58-126f8e606e96"> | <img width="224" height="234" alt="DS-TWR Diagram" src="https://github.com/user-attachments/assets/e37c7578-ba79-491c-ba59-383611eb9a40">

</div>

### AoA (Angle of Arrival) 추정 원리

안테나 간의 거리($d$)와 수신 신호의 위상차($\Delta\phi$)를 이용하여 입사각($\Theta$)을 계산합니다.

$$
\lambda : 2\pi = \text{전파의 이동거리} : \Delta\phi
$$
$$
\lambda : 2\pi = d \cdot \sin\Theta : \Delta\phi
$$
$$
\sin\Theta = \frac{\lambda \cdot \Delta\phi}{2\pi \cdot d}
$$
$$
\sin\Theta = \frac{\Delta\phi}{\pi}
$$
$$
\theta = \arcsin\left(\frac{\Delta\phi}{\pi}\right)
$$

**구현 세부사항**
- 안테나 2개 간격(d): 17mm
- UWB 채널별 안테나 특성을 반영한 실측 보정 계수 적용 (채널5: 0.4321, 채널9: 0.3516)
- 위상차가 특정 각도를 넘으면 실제값과 다르게 wrap-around되는 현상이 있어, 위상차 임계값을 계산해 초과 시 ±2×임계값만큼 보정(언랩)한 뒤 각도를 산출 (자세한 내용은 5장 참고)

---

## 4. 하드웨어 구성

<div align="center">

<table align="center" width="90%">
  <tr>
    <th align="center">역할</th>
    <th align="center">모델명 / 칩셋</th>
    <th align="center">설명</th>
  </tr>
  <tr>
    <td align="center"><strong>Anchor</strong></td>
    <td align="center">Nordic nRF52840 + Qorvo DW3110</td>
    <td align="center">DS-TWR 기반 UWB 통신, AoA 계산용 신호 수집</td>
  </tr>
  <tr>
    <td align="center"><strong>Tag</strong></td>
    <td align="center">Nordic nRF52840 + Qorvo DW3000</td>
    <td align="center">이동 객체에 부착, UWB 송수신</td>
  </tr>
  <tr>
    <td align="center"><strong>Antenna</strong></td>
    <td align="center">XR-170 UWB Directional Antenna</td>
    <td align="center">고지향성으로 AoA 추정 성능 향상</td>
  </tr>
  <tr>
    <td align="center"><strong>시각화</strong></td>
    <td align="center">MATLAB</td>
    <td align="center">UART 데이터 수신 후 실시간 위치 시각화</td>
  </tr>
</table>

</div>

<p align="center">
  <img src="./imgs/EXPERIMENT.png" width="600" alt="Hardware Setup and Experiment Environment">
</p>

---

## 5. 주요 성과 및 분석

### ✅ 성과
- **단일 앵커 + 태그 구성**으로 DS-TWR + AoA 융합 실시간 RTLS 구현 (앵커 수 3→1개로 축소, 패킷 교환 횟수 11회→3회로 약 73% 감소)
- 1.8×1.8m 트랙 기준, 필터 적용 전 위치 추정 RMSE 22.45cm → **이동평균 필터(W=16) 적용 후 16.35cm로 27.2% 개선** (실사용 채택)
  - 참고: 동일 조건에서 중앙값 필터(N=29)는 15.85cm(29.4%↓)로 이동평균보다 더 낮은 오차를 보였으나, 실시간성과 구현 단순성을 고려해 이동평균 필터를 최종 채택
- MATLAB 기반 실시간 시각화 성공

### ⚠️ 문제점 및 해결

**1) PDOA 위상 언랩(Unwrap) 보정**
- 문제: 안테나 2개 간격(17mm)과 신호 파장 특성상, 위상차(PDOA) 측정값이 특정 각도를 넘으면 실제값과 다르게 wrap-around되어 각도 추정이 순간적으로 엉뚱한 값으로 튀는 현상 발생
- 해결: 안테나 간격·파장으로 위상차 임계값을 계산하고, 측정값이 임계값을 넘으면 ±2×임계값만큼 보정(언랩)해 실제 위상차로 복원한 뒤 각도 산출
- 결과: 안테나 배치 특성상 발생하는 위상 폴딩으로 인한 각도 이상치 제거

**2) UART 데이터 무결성 (CRC)**
- 문제: 초기엔 `" D I %.2f P %.2f O A "` 같은 ASCII 텍스트로 MCU→MATLAB 거리·각도 데이터를 전송했는데, 최대 25바이트의 가변 길이라 프레임 경계 동기화가 어렵고 CRC가 없어 값이 깨져도 검출할 수 없었음
- 해결: `[0xAA][거리 4B][각도 4B][CRC8][0x55]` 구조의 11바이트 고정 길이 바이너리 패킷으로 교체하고 CRC-8/SMBUS로 무결성 검증 (`monitoring/binary_receiver.py`)
- 결과: 패킷 크기 25B→11B(56%↓), 헤더/테일로 프레임 동기화 용이, CRC 오류 패킷은 즉시 폐기해 잘못된 좌표가 표시되는 것을 방지

**3) 방향에 따른 좌표 오차 편차 (Y축/각도 민감도)**
- 태그가 앵커 정면(거리) 방향으로 움직일 때는 각도 변화가 작아 각도 기반 추정의 민감도가 떨어지고, 그만큼 해당 방향 좌표 오차가 더 크게 나타나는 경향을 확인함 (개선 방향은 6장 참고)

<p align="center">
  <img src="./imgs/RESULT.png" width="400" alt="Experiment Results">
</p>

---

## 6. 개선 방향
- 중앙값·칼만 필터 등 다른 후처리 기법과의 정밀 비교 (칼만 필터는 본 프로젝트에서는 실제 적용하지 않음)
- 앵커 배치 최적화를 통한 방향별 오차 편차 추가 감소

---

## 7. 데모 영상


<p align="center">
  <a href="https://www.youtube.com/watch?v=SgOs7Dkw7NQ">
    <img src="https://img.youtube.com/vi/SgOs7Dkw7NQ/0.jpg" width="600" alt="Demo Video Thumbnail">
  </a>
</p>

---

## 8. 수상
🏆 **한국 전자파 학회 제 4회 대학생 창의설계 경진대회 우수논문상(동상) 수상**

---
