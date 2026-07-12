/*! ----------------------------------------------------------------------------
 *  @file    AoA_rtls_rx_adj.c
 *  @brief   AoA_rtls_rx.c 수정본 — ASCII sprintf 방식을 Binary Packet + CRC8로 교체
 *
 *  원본 ASCII 포맷:  " D I %3.2f P %3.2f O A "  → 최대 25바이트, 구분자 파싱 방식
 *  수정 Binary 포맷: [0xAA][dist:4B float LE][angle:4B float LE][CRC8:1B][0x55] = 11바이트
 *
 *  변경 이유:
 *    1) 고정 길이 → 동기화 용이 (헤더/테일로 프레임 경계 명확)
 *    2) CRC8 오류 검출 (ASCII는 데이터 오류 검출 불가)
 *    3) 크기 절감: 25B → 11B (56% 감소)
 *    4) float 원시값 전송 → 수신 측 파싱 오류 없음
 *
 *  참고: embeddedrelated.com - "Help, My Serial Data Has Been Framed"
 *        CRC-8/SMBUS (poly=0x07, init=0x00) 표준 적용
 * -------------------------------------------------------------------------- */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include <math.h>
#include <time.h>
#include <nrf_timer.h>
#include <nrf_drv_timer.h>
#include <string.h>

/* UART 관련 헤더 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

/* ── Binary Packet 정의 ─────────────────────────────────────────────────────
 *
 *  오프셋  크기  내용
 *  ──────  ────  ────────────────────────────────
 *  [0]     1B    START byte  = 0xAA
 *  [1..4]  4B    distance    (float, Little-Endian)
 *  [5..8]  4B    angle       (float, Little-Endian)
 *  [9]     1B    CRC8        (payload = bytes [1..8])
 *  [10]    1B    END byte    = 0x55
 *  ──────  ────  ────────────────────────────────
 *  Total: 11 bytes
 */
#define PKT_START    0xAA
#define PKT_END      0x55
#define PKT_SIZE     11     /* 고정 패킷 크기 */

/* CRC8/SMBUS: poly=0x07, init=0x00, refin=false, refout=false */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/*
 * @brief  거리·각도를 11바이트 바이너리 패킷으로 UART 전송
 *
 * @param  distance  거리 (m), float
 * @param  angle     AoA 각도 (도), float
 *
 * nRF52840 = ARM Cortex-M4 → float는 IEEE 754 single, Little-Endian 저장
 * memcpy로 바이트 배열에 복사하면 LE 바이트 순서 그대로 전송됨
 */
static void send_uwb_packet(float distance, float angle)
{
    uint8_t pkt[PKT_SIZE];

    pkt[0] = PKT_START;

    /* float → 4바이트 LE 복사 */
    memcpy(&pkt[1], &distance, sizeof(float));   /* bytes [1..4] */
    memcpy(&pkt[5], &angle,    sizeof(float));   /* bytes [5..8] */

    /* CRC8 계산 대상: payload bytes [1..8] (8바이트) */
    pkt[9]  = crc8(&pkt[1], 8);

    pkt[10] = PKT_END;

    /* 바이트 단위 전송 (기존 uart0_strsend 방식과 동일한 app_uart_put 사용) */
    for (uint8_t i = 0; i < PKT_SIZE; i++) {
        app_uart_put(pkt[i]);
    }
}

/* ── UART error handler ─────────────────────────────────────────────────── */
static void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
        APP_ERROR_HANDLER(p_event->data.error_communication);
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
        APP_ERROR_HANDLER(p_event->data.error_code);
}
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/* ── DW3000 설정 (원본과 동일) ──────────────────────────────────────────── */
#if defined(AoA_rtls_rx)

extern void test_run_info(unsigned char *data);
#define APP_NAME "AOA RTLS RX v1.1 (BIN)"

static dwt_config_t config = {
    5,
    DWT_PLEN_1024,
    DWT_PAC4,
    9,
    9,
    3,
    DWT_BR_850K,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (1024 + 1 + 8 - 4),
    DWT_STS_MODE_1 | DWT_STS_MODE_SDC,
    DWT_STS_LEN_256,
    DWT_PDOA_M1
};

#define DELAY_MS                    1
#define TX_ANT_DLY                  16385
#define RX_ANT_DLY                  16385

static uint8_t rx_poll_msg[]  = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0x21};
static uint8_t tx_resp_msg[]  = {0x41,0x88,0,0xCA,0xDE,'V','E','W','A',0x10,0x02,0,0};
static uint8_t rx_final_msg[] = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0x23,0,0,0,0,0,0,0,0,0,0,0,0};

#define ALL_MSG_COMMON_LEN      10
#define ALL_MSG_SN_IDX          2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

static uint8_t  frame_seq_nb = 0;
#define RX_BUF_LEN 24
static uint8_t  rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

#define POLL_RX_TO_RESP_TX_DLY_UUS  2200
#define RESP_TX_TO_FINAL_RX_DLY_UUS 770
#define FINAL_RX_TIMEOUT_UUS        1380
#define PRE_TIMEOUT                 15

static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

static double tof;
static double distance;
double AoA;
double prev = 0;

extern dwt_txconfig_t txconfig_options;
static dwt_rxdiag_t rx_diag;

double correctionPdoa(double pdoa, int channel);
double calculateAoA(double pdoa, int channel);
void   timer_init_uss(void);
void   TIMER4_IRQHandler(void);
float  TIMER4_OFTime(void);
uint32_t TIMER4_Check(uint32_t starttime, float clk_time);

/* ── 메인 함수 ──────────────────────────────────────────────────────────── */
int aoa_rtls_responder(void)
{
    int range_ok = 0;
    int16_t  pdoa;
    double   pdoa_deg;
    FILE    *file;

    test_run_info((unsigned char *)APP_NAME);
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);

    /* UART 초기화 (원본과 동일) */
    uint32_t err_code;
    const app_uart_comm_params_t comm_params = {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        UART_HWFC,
        false,
        NRF_UART_BAUDRATE_115200
    };
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);

    while (!dwt_checkidlerc()) {};

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) {};
    }

    if (dwt_configure(&config)) {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) {};
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_configciadiag(1);

    float    clk_time  = 0;
    uint32_t starttime = 0;

    timer_init_uss();
    clk_time = TIMER4_OFTime();
    NRF_TIMER4->TASKS_START = 1;

    file = fopen("newnormal_2m40.txt", "w");
    int i = 1;

    while (1)
    {
        dwt_setpreambledetecttimeout(0);
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                 (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;
            int16_t  stsqual;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            if (dwt_readstsquality(&stsqual))
            {
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                if (frame_len <= RX_BUF_LEN)
                    dwt_readrxdata(rx_buffer, frame_len, 0);

                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    poll_rx_ts   = get_rx_timestamp_u64();
                    resp_tx_time = (poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS) * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);
                    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_resp_msg) + FCS_LEN, 0, 1);
                    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
                    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                    if (ret == DWT_ERROR) {
                        printf("response fail ");
                        continue;
                    }

                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

                    frame_seq_nb++;

                    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                    {
                        dwt_write32bitreg(SYS_STATUS_ID,
                                         SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

                        if (dwt_readstsquality(&stsqual))
                        {
                            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
                            if (frame_len <= RX_BUF_LEN)
                                dwt_readrxdata(rx_buffer, frame_len, 0);

                            rx_buffer[ALL_MSG_SN_IDX] = 0;
                            if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                            {
                                uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                                uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                                double Ra, Rb, Da, Db;
                                int64_t tof_dtu;

                                resp_tx_ts  = get_tx_timestamp_u64();
                                final_rx_ts = get_rx_timestamp_u64();

                                final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],  &poll_tx_ts);
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],  &resp_rx_ts);
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                                poll_rx_ts_32  = (uint32_t)poll_rx_ts;
                                resp_tx_ts_32  = (uint32_t)resp_tx_ts;
                                final_rx_ts_32 = (uint32_t)final_rx_ts;
                                Ra = (double)(resp_rx_ts  - poll_tx_ts);
                                Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                                Da = (double)(final_tx_ts - resp_rx_ts);
                                Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                                tof_dtu  = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
                                tof      = tof_dtu * DWT_TIME_UNITS;
                                distance = tof * SPEED_OF_LIGHT;

                                printf("\nDIST: %3.2f m\n", distance);

                                range_ok = 1;

                                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                                memset(&rx_diag, 0, sizeof(rx_diag));
                                dwt_readdiagnostics(&rx_diag);

                                pdoa     = dwt_readpdoa();
                                pdoa_deg = ((float)pdoa / (1 << 11));
                                pdoa_deg = correctionPdoa(pdoa_deg, 9);
                                printf("pdoa_deg:%f\n", pdoa_deg);

                                prev = AoA;
                                AoA  = calculateAoA(pdoa_deg, 9);
                                printf("AoA :%g  ", AoA);

                                /* ── 기존 ASCII 방식 (주석 처리) ──────────────
                                 *   sprintf(dis_str, " D I %3.2f P %3.2f O A ", distance, AoA);
                                 *   uart0_strsend(dis_str);
                                 *   문제점: 최대 25바이트, CRC 없음, 파싱 fragile
                                 * ──────────────────────────────────────────── */

                                /* ── 바이너리 패킷 전송 (수정) ────────────────
                                 *   [0xAA][dist:4B LE][angle:4B LE][CRC8:1B][0x55]
                                 *   총 11바이트, CRC8/SMBUS 오류 검출 포함
                                 * ──────────────────────────────────────────── */
                                send_uwb_packet((float)distance, (float)AoA);

                                starttime = TIMER4_Check(starttime, clk_time);

                                if (i <= 500) {
                                    fprintf(file, "%f|%f\n", distance, AoA);
                                    fflush(file);
                                    i++;
                                } else {
                                    fclose(file);
                                    break;
                                }
                            }
                        }
                        else
                        {
                            printf("final fail\n");
                            dwt_write32bitreg(SYS_STATUS_ID,
                                             SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                        }
                    }
                }
            }
            else
            {
                dwt_write32bitreg(SYS_STATUS_ID,
                                 SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            }

            if (range_ok) {
                range_ok = 0;
                Sleep(DELAY_MS);
            }
        }
    }
    fclose(file);
    return 0;
}

/* ── PDoA 보정 / AoA 계산 (원본과 동일) ─────────────────────────────────── */
double correctionPdoa(double pdoa, int channel)
{
    double c = 3 * (10 ^ 8);
    double f;
    double d = 17.0 / 1000.0;

    if      (channel == 5) f = 6.5   * (10 ^ 9);
    else if (channel == 9) f = 7.9872 * (10 ^ 9);

    double threshold_pdoa = 1.0 / (c / (2 * 3.141592 * f * d));

    for (int k = 0; k < 2; k++) {
        if      (pdoa >  threshold_pdoa) pdoa -= 2 * threshold_pdoa;
        else if (pdoa < -threshold_pdoa) pdoa += 2 * threshold_pdoa;
    }
    return pdoa;
}

double calculateAoA(double pdoa, int channel)
{
    if      (channel == 5) AoA = (asin(pdoa * 0.4321)) * 180.0 / 3.141592;
    else if (channel == 9) AoA = (asin(pdoa * 0.3516)) * 180.0 / 3.141592;
    return AoA;
}

/* ── Timer (원본과 동일) ─────────────────────────────────────────────────── */
void timer_init_uss(void)
{
    NRF_TIMER4->TASKS_STOP  = 1;
    NRF_TIMER4->TASKS_CLEAR = 1;
    NRF_TIMER4->MODE        = TIMER_MODE_MODE_Timer;
    NRF_TIMER4->BITMODE     = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER4->PRESCALER   = 3;
    NRF_TIMER4->INTENSET    = 4294967295U;
    NRF_TIMER4->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NVIC_EnableIRQ(TIMER4_IRQn);
    NRF_TIMER4->TASKS_START = 1;
}

void TIMER4_IRQHandler(void)
{
    if (NRF_TIMER4->EVENTS_COMPARE[0] == 1) {
        NRF_TIMER4->EVENTS_COMPARE[0] = 0;
        NRF_TIMER4->TASKS_STOP  = 1;
        NRF_TIMER4->TASKS_CLEAR = 1;
    }
}

float TIMER4_OFTime(void)
{
    int   pre      = NRF_TIMER4->PRESCALER;
    float clk_time = 1.0f / ((16.0f) / (float)pow(2.0, pre));
    printf("prescaler = %d, clk time = %f us\n", pre, clk_time);
    printf("timer overflow time = %f second\n", pow(2.0, 32) * clk_time / 1000000.0f);
    return clk_time;
}

uint32_t TIMER4_Check(uint32_t starttime, float clk_time)
{
    NRF_TIMER4->TASKS_CAPTURE[1] = 1;
    uint32_t end_time = NRF_TIMER4->CC[1];
    uint32_t ticks    = end_time - starttime;
    printf("\nelapsed_time = %f us\n", ticks * clk_time);
    return end_time;
}

#endif /* AoA_rtls_rx */
