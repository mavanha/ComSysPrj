/**
 * morse_single.c
 *
 * One-file implementation for the Morse project:
 *  - Tier 1: IMU + buttons to generate Morse and send via USB.
 *  - Tier 2: Receive Morse from workstation and replay with RGB LED + buzzer.
 *
 * Protocol (new version):
 *   - Characters: '.' and '-'
 *   - One space ' ' between characters
 *   - Two spaces + '\n' to end message: "  \n"
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tkjhat/sdk.h"
#include "tkjhat/imu.h"
#include "tkjhat/buttons.h"
#include "tkjhat/led.h"
#include "tkjhat/buzzer.h"

/*==========================================================
 * CONFIGURATION
 *========================================================*/

#define MORSE_MAX_LEN       128   // tối đa ký tự trong một message

/* Ngưỡng cho IMU: Z hướng lên = dot, Y hướng lên = dash */
#define ACC_THR_MAIN        0.75f
#define ACC_THR_ORTHO       0.35f

/* Lọc nhiễu gia tốc */
#define ACC_EMA_ALPHA       0.2f

/* Chu kỳ đo IMU (ms) */
#define IMU_PERIOD_MS       40

/*==========================================================
 * TYPES & GLOBAL STATE
 *========================================================*/

typedef enum {
    APP_IDLE = 0,       // chưa ghi gì
    APP_RECORDING,      // đang ghi ký hiệu từ IMU
    APP_READY_TO_SEND,  // message kết thúc, chờ task TX gửi
    APP_TX_IN_PROGRESS, // đang gửi ra USB
    APP_RX_READY        // vừa nhận xong message từ PC
} app_state_t;

/* Dùng chung cho TX và RX */
typedef struct {
    char   buf[MORSE_MAX_LEN];
    size_t len;
} morse_msg_t;

/* Global messages + state */
static morse_msg_t g_tx_msg;
static morse_msg_t g_rx_msg;
static volatile app_state_t g_app_state = APP_IDLE;

/* Queue để truyền byte tới task TX (tùy thích, cho rõ ràng) */
static QueueHandle_t q_tx_bytes;

/*==========================================================
 * UTILS
 *========================================================*/

static inline float ema_update(float prev, float x)
{
    return ACC_EMA_ALPHA * x + (1.0f - ACC_EMA_ALPHA) * prev;
}

static void msg_clear(morse_msg_t *m)
{
    m->len    = 0;
    m->buf[0] = '\0';
}

static void msg_push_char(morse_msg_t *m, char c)
{
    if (m->len + 1 >= MORSE_MAX_LEN) {
        return; // tránh tràn
    }
    m->buf[m->len++] = c;
    m->buf[m->len]   = '\0';
}

/*==========================================================
 * INPUT TASK (IMU + BUTTONS) – TIER 1
 *========================================================*/

static void task_input_imu_buttons(void *arg)
{
    (void)arg;

    /* Khởi tạo IMU */
    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
    }

    /* Khởi tạo button từ TKJHAT SDK */
    init_button1();
    init_button2();

    float ax = 0.0f, ay = 0.0f, az = 1.0f;
    bool  btn1_prev = false;
    bool  btn2_prev = false;

    enum {POSE_NONE, POSE_DOT, POSE_DASH} last_pose = POSE_NONE;

    msg_clear(&g_tx_msg);
    g_app_state = APP_IDLE;

    for (;;)
    {
        /* ---- Đọc nút bấm (polling) ---- */
        bool btn1_now = !gpio_get(BUTTON1); // active low
        bool btn2_now = !gpio_get(BUTTON2);

        bool btn1_rise = (btn1_now && !btn1_prev);
        bool btn2_rise = (btn2_now && !btn2_prev);
        btn1_prev = btn1_now;
        btn2_prev = btn2_now;

        /* BTN1: Bắt đầu / hủy ghi message */
        if (btn1_rise) {
            if (g_app_state == APP_IDLE) {
                msg_clear(&g_tx_msg);
                last_pose  = POSE_NONE;
                g_app_state = APP_RECORDING;
                printf("__REC START__\r\n");
            } else if (g_app_state == APP_RECORDING) {
                /* Hủy message hiện tại */
                msg_clear(&g_tx_msg);
                last_pose  = POSE_NONE;
                g_app_state = APP_IDLE;
                printf("__REC CANCEL__\r\n");
            }
        }

        /* BTN2: thêm space hoặc kết thúc message */
        if (btn2_rise && g_app_state == APP_RECORDING) {
            if (g_tx_msg.len > 0 &&
                g_tx_msg.buf[g_tx_msg.len - 1] == ' ')
            {
                /* Đã có 1 space ở cuối -> thêm space nữa + '\n' => kết thúc */
                msg_push_char(&g_tx_msg, ' ');
                msg_push_char(&g_tx_msg, '\n');
                g_app_state = APP_READY_TO_SEND;
                last_pose   = POSE_NONE;
                printf("__REC END__\r\n");
            }
            else {
                /* Space giữa các ký tự / giữa các từ */
                msg_push_char(&g_tx_msg, ' ');
                printf("__ADD SPACE__\r\n");
            }
        }

        /* Nếu đang không ở trạng thái RECORDING thì bỏ qua IMU */
        if (g_app_state == APP_RECORDING)
        {
            /* ---- Đọc IMU ---- */
            float rx, ry, rz, gx, gy, gz, t;
            if (ICM42670_read_sensor_data(&rx, &ry, &rz, &gx, &gy, &gz, &t) == 0) {
                ax = ema_update(ax, rx);
                ay = ema_update(ay, ry);
                az = ema_update(az, rz);
            }

            /* Phân loại tư thế:
               - DOT  : Z lớn, Y nhỏ  -> đặt phẳng
               - DASH : Y lớn, Z nhỏ  -> dựng đứng */
            int pose_now = POSE_NONE;

            if (fabsf(az) > ACC_THR_MAIN && fabsf(ay) < ACC_THR_ORTHO) {
                pose_now = POSE_DOT;
            } else if (fabsf(ay) > ACC_THR_MAIN && fabsf(az) < ACC_THR_ORTHO) {
                pose_now = POSE_DASH;
            }

            /* Ghi ký hiệu khi chuyển sang tư thế mới (tránh spam) */
            if (pose_now != POSE_NONE && pose_now != last_pose) {
                char sym = (pose_now == POSE_DOT) ? '.' : '-';
                msg_push_char(&g_tx_msg, sym);
                last_pose = pose_now;
                printf("__SYM %c__\r\n", sym);
            }

            if (pose_now == POSE_NONE) {
                /* reset pose để lần sau quay lại tư thế mới vẫn ghi nhận */
                last_pose = POSE_NONE;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

/*==========================================================
 * USB TX TASK – gửi message hoàn chỉnh – TIER 1
 *========================================================*/

static void task_usb_tx(void *arg)
{
    (void)arg;

    /* đảm bảo USB CDC đã sẵn sàng */
    while (!stdio_usb_connected()) {
        tud_task();
        sleep_ms(10);
    }

    for (;;)
    {
        if (g_app_state == APP_READY_TO_SEND) {
            g_app_state = APP_TX_IN_PROGRESS;

            /* Gửi từng byte vào queue cho rõ ràng,
               nhưng thực tế có thể printf trực tiếp. */
            for (size_t i = 0; i < g_tx_msg.len; ++i) {
                char c = g_tx_msg.buf[i];
                xQueueSend(q_tx_bytes, &c, portMAX_DELAY);
            }

            /* Sau khi queue đầy đủ, thực sự ghi ra USB */
            char out;
            while (uxQueueMessagesWaiting(q_tx_bytes) > 0) {
                if (xQueueReceive(q_tx_bytes, &out, 0) == pdTRUE) {
                    putchar_raw(out);  // dùng stdio trên USB
                }
            }

            fflush(stdout);
            printf("__SENT__\r\n");

            msg_clear(&g_tx_msg);
            g_app_state = APP_IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/*==========================================================
 * USB RX TASK – nhận Morse từ workstation – TIER 2
 *========================================================*/

static void task_usb_rx(void *arg)
{
    (void)arg;

    msg_clear(&g_rx_msg);

    for (;;)
    {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\r') {
            /* bỏ CR, chờ LF */
            continue;
        }

        if (g_rx_msg.len + 1 >= MORSE_MAX_LEN) {
            /* tràn, reset đơn giản */
            msg_clear(&g_rx_msg);
        }

        msg_push_char(&g_rx_msg, (char)ch);

        if (ch == '\n') {
            /* Nhận xong message: buf chứa cả "  \n" */
            g_app_state = APP_RX_READY;
            printf("__RX %s__", g_rx_msg.buf);
        }
    }
}

/*==========================================================
 * FEEDBACK TASK – RGB + BUZZER cho message nhận – TIER 2
 *========================================================*/

static void play_dot(void)
{
    rgb_led_write(0, 0, 255);       // blue
    buzzer_play_tone(800, 150);
    vTaskDelay(pdMS_TO_TICKS(200));
    rgb_led_write(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(80));
}

static void play_dash(void)
{
    rgb_led_write(0, 255, 0);       // green
    buzzer_play_tone(800, 400);
    vTaskDelay(pdMS_TO_TICKS(450));
    rgb_led_write(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void play_space(void)
{
    rgb_led_write(255, 0, 0);       // red blink
    vTaskDelay(pdMS_TO_TICKS(150));
    rgb_led_write(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(150));
}

static void play_jingle(void)
{
    buzzer_play_tone(880, 120);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_play_tone(988, 120);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_play_tone(1175, 200);
    vTaskDelay(pdMS_TO_TICKS(250));
}

static void task_feedback(void *arg)
{
    (void)arg;

    init_rgb_led();
    init_buzzer();

    for (;;)
    {
        if (g_app_state == APP_RX_READY) {

            for (size_t i = 0; i < g_rx_msg.len; ++i) {
                char c = g_rx_msg.buf[i];

                if (c == '.')       play_dot();
                else if (c == '-')  play_dash();
                else if (c == ' ')  play_space();
                else if (c == '\n') { /* kết thúc message */
                    play_jingle();
                }
            }

            msg_clear(&g_rx_msg);
            g_app_state = APP_IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/*==========================================================
 * MAIN
 *========================================================*/

int main(void)
{
    stdio_init_all();
    tusb_init();                // TinyUSB cho stdio USB

    init_hat_sdk();             // bật I2C, GPIO… trên HAT
    sleep_ms(200);              // cho HAT ổn định

    init_led();                 // LED nhỏ trên Pico, dùng làm debug nếu cần

    /* Tạo queue */
    q_tx_bytes = xQueueCreate(256, sizeof(char));

    /* Tạo tasks */
    xTaskCreate(task_input_imu_buttons, "input", 1024, NULL, 2, NULL);
    xTaskCreate(task_usb_tx,            "tx",    1024, NULL, 1, NULL);
    xTaskCreate(task_usb_rx,            "rx",    1024, NULL, 1, NULL);
    xTaskCreate(task_feedback,          "fb",    1024, NULL, 1, NULL);

    /* Start FreeRTOS */
    vTaskStartScheduler();

    /* Không bao giờ tới đây */
    while (true) { }
}
