# AoA-Based UWB RTLS

> **Single Anchor + Single Tag 실시간 위치 추적 시스템**  
> DS-TWR 기반 거리(Ranging)와 AoA(Angle of Arrival) 기반 각도(Angle Estimation)를 융합해 **저복잡도·실시간 RTLS**를 구현

[![UWB](https://img.shields.io/badge/UWB-IEEE%20802.15.4z-blue)]()
[![Localization](https://img.shields.io/badge/Localization-AoA-green)]()
[![Award](https://img.shields.io/badge/Award-전자파학회%20동상-orange)]()

---

## 1. 개요
**단일 앵커(Anchor)**와 **단일 태그(Tag)** 만으로 실시간 위치 추적(Real-Time Locating System, RTLS)을 구현한 프로젝트입니다.  
기존 삼각측량 방식이 최소 3개 이상의 앵커를 요구하는 것과 달리, **AoA(Angle of Arrival)**를 도입해 **앵커 수를 줄이고 시스템 구조를 단순화**했습니다.

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
  - DS-TWR(Double-Sided Two-Way Ranging)을 사용하여 태그와 앵커 간의 정밀한 거리(R) 계산
- **각도 측정 (Angle Estimation)**
  - UWB 안테나 배열의 위상차(Phase Difference) 기반 AoA 계산
- **위치 추정**
  - 거리 R과 각도 $\theta$를 결합하여 2D 좌표를 실시간 산출
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
    <td align="center">Nordic nRF52840 + Qorvo DW3110</td>
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
- **단일 앵커 + 태그 구성**으로 DS-TWR + AoA 융합 실시간 RTLS 구현
- 실험 환경에서 **cm 단위 수준의 위치 추적 정확도** 확보
- MATLAB 기반 실시간 시각화 성공

### ⚠️ 문제점 및 해결
- **Y축 오차**
  - Y축 이동 시 각도 변화폭이 작아 X좌표 분산이 집중되는 현상
  - → 칼만 필터(Kalman Filter) 등 가중치 기반 필터 적용으로 정확도 향상 가능
- **실시간 시각화 문제**
  - MATLAB UART 통신 시 데이터 누락 → 버퍼 주기적 초기화로 해결

<p align="center">
  <img src="./imgs/RESULT.png" width="400" alt="Experiment Results">
</p>

---

## 6. 개선 방향
- 이동 방향 기반 **보정 로직** 추가(이동평균 필터, 중위값 필터, 칼만필터)로 안정적 추적
- 앵커 배치 최적화 및 보조 신호 활용으로 Y축 오차 감소

---

## 7. 데모 영상


<p align="center">
  <a href="https://www.youtube.com/watch?v=SgOs7Dkw7NQ">
    <img src="https://img.youtube.com/vi/SgOs7Dkw7NQ/0.jpg" width="600" alt="Demo Video Thumbnail">
  </a>
</p>

---

## 8. 수상
🏆 **한국 전자파 학회 제 4회 대학생 창의설계 경진대회 동상 수상**

---









