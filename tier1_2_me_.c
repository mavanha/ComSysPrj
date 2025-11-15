/**
 * morse_single.c
 * One-file Morse project: Tier 1 + Tier 2 (RX feedback) + Mic (optional)
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tkjhat/sdk.h"
#include "tkjhat/imu.h"
#include "tkjhat/buttons.h"
#include "tkjhat/led.h"
#include "tkjhat/buzzer.h"

/* =====================  CONFIGURATION  ===================== */

/* 1 = message ends with "  \\n"  (2 spaces + LF) – new serial client
   0 = message ends with "   "   (3 spaces)        – old style      */
#ifndef END_WITH_LF
#define END_WITH_LF 1
#endif

#if END_WITH_LF
  #define MSG_END_C1 ' '
  #define MSG_END_C2 ' '
  #define MSG_END_C3 '\n'
#else
  #define MSG_END_C1 ' '
  #define MSG_END_C2 ' '
  #define MSG_END_C3 ' '
#endif

#define MORSE_BUF_CAP   120

/* IMU position detection */
#define EMA_ALPHA       (0.20f)   /* low-pass filter coeff */
#define HOLD_MS         (200)     /* must hold pose >= 200 ms */
#define AFTER_MS        (150)     /* pause after each symbol */

/* Microphone detection (optional) */
#define USE_MIC         1         /* 1 = enable microphone task */
#define MIC_ADC_CH      0         /* ADC0 = GPIO26 on Pico */
#define MIC_RATE_HZ     8000      /* sampling rate */
#define MIC_ENV_ALPHA   0.1f      /* envelope smoothing */
#define MIC_ON_THR      0.15f
#define MIC_OFF_THR     0.08f
#define MIC_DOT_MAX_MS  220       /* shorter = dot, longer = dash */

/* =====================  TYPES & GLOBALS  ===================== */

typedef enum {
    SYM_DOT  = '.',
    SYM_DASH = '-',
    SYM_SP   = ' ',
    SYM_END  = 'E'  /* internal END event */
} morse_sym_t;

typedef enum {
    PS_IDLE = 0,
    PS_ARMING,
    PS_HAVE_CHAR,
    PS_SENDING,
    PS_RX_READY
} app_phase_t;

static QueueHandle_t q_sym_out;   /* IMU/Mic/Buttons -> encoder        */
static QueueHandle_t q_tx_bytes;  /* encoder -> USB TX task            */
static QueueHandle_t q_rx_bytes;  /* USB RX task -> feedback task      */

static volatile app_phase_t g_phase = PS_IDLE;

/* small EMA helper */
static inline float ema(float prev, float x) {
    return EMA_ALPHA * x + (1.0f - EMA_ALPHA) * prev;
}

/* =====================  INPUT: IMU + BUTTONS  ===================== */

typedef enum {
    BTN_EVT_START = 1,   /* arm IMU reading */
    BTN_EVT_END          /* send END event  */
} btn_evt_t;

static QueueHandle_t q_btn_evt;

/* interrupt for BUTTON1 (start), BUTTON2 (END) */
static void button_isr(uint gpio, uint32_t events) {
    (void)events;
    BaseType_t hpw = pdFALSE;
    btn_evt_t ev   = 0;

    if (gpio == BUTTON1) {
        ev = BTN_EVT_START;
    } else if (gpio == BUTTON2) {
        ev = BTN_EVT_END;
    } else {
        return;
    }

    xQueueSendFromISR(q_btn_evt, &ev, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void task_input_imu_btn(void *arg) {
    (void)arg;

    /* init IMU */
    if (init_ICM42670() == 0) {
        (void)ICM42670_start_with_default_values();
    }

    /* queue for button events from ISR */
    q_btn_evt = xQueueCreate(8, sizeof(btn_evt_t));

    /* attach interrupts */
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true,
                                       button_isr);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    float ax = 0, ay = 0, az = 1;
    TickType_t t_dot  = 0;
    TickType_t t_dash = 0;
    const TickType_t T_HOLD  = pdMS_TO_TICKS(HOLD_MS);
    const TickType_t T_AFTER = pdMS_TO_TICKS(AFTER_MS);

    for (;;) {
        /* handle button events first */
        btn_evt_t ev;
        if (xQueueReceive(q_btn_evt, &ev, 0) == pdTRUE) {
            if (ev == BTN_EVT_START) {
                g_phase = PS_ARMING;  /* enable IMU based input */
            } else if (ev == BTN_EVT_END) {
                morse_sym_t e = SYM_END;
                xQueueSend(q_sym_out, &e, 0);
            }
        }

        /* read IMU @ ~50 Hz */
        float rx, ry, rz, gx, gy, gz, temp;
        if (ICM42670_read_sensor_data(&rx, &ry, &rz,
                                      &gx, &gy, &gz, &temp) == 0) {
            ax = ema(ax, rx);
            ay = ema(ay, ry);
            az = ema(az, rz);
        }

        if (g_phase == PS_ARMING) {
            bool z_up = (fabsf(az) > 0.7f && fabsf(ay) < 0.3f);  /* dot pose  */
            bool y_up = (fabsf(ay) > 0.7f && fabsf(az) < 0.3f);  /* dash pose */

            TickType_t now = xTaskGetTickCount();

            if (z_up) {
                if (!t_dot) t_dot = now;
                if (now - t_dot >= T_HOLD) {
                    morse_sym_t s = SYM_DOT;
                    xQueueSend(q_sym_out, &s, 0);
                    g_phase = PS_HAVE_CHAR;
                    t_dot   = 0;
                    vTaskDelay(T_AFTER);
                }
            } else {
                t_dot = 0;
            }

            if (y_up) {
                if (!t_dash) t_dash = now;
                if (now - t_dash >= T_HOLD) {
                    morse_sym_t s = SYM_DASH;
                    xQueueSend(q_sym_out, &s, 0);
                    g_phase = PS_HAVE_CHAR;
                    t_dash  = 0;
                    vTaskDelay(T_AFTER);
                }
            } else {
                t_dash = 0;
            }
        }

        /* short press of BUTTON1 while already have char => SPACE */
        if (g_phase == PS_HAVE_CHAR && tkj_button1_pressed()) {
            morse_sym_t sp = SYM_SP;
            xQueueSend(q_sym_out, &sp, 0);
            vTaskDelay(pdMS_TO_TICKS(180));  /* debounce */
        }

        vTaskDelay(pdMS_TO_TICKS(20));  /* ~50 Hz loop */
    }
}

/* =====================  INPUT: MICROPHONE (OPTIONAL)  ===================== */

#if USE_MIC
static void task_input_mic(void *arg) {
    (void)arg;

    adc_init();
    adc_gpio_init(26 + MIC_ADC_CH);
    adc_select_input(MIC_ADC_CH);

    absolute_time_t next = get_absolute_time();
    float env = 0.0f;
    bool  on  = false;
    TickType_t t_on = 0;

    for (;;) {
        int samples_per_chunk = MIC_RATE_HZ / 200;  /* ~ 200 chunks /s */

        for (int i = 0; i < samples_per_chunk; ++i) {
            next = delayed_by_us(next, 1000000 / MIC_RATE_HZ);
            while (absolute_time_diff_us(get_absolute_time(), next) > 0) {
                tight_loop_contents();
            }

            uint16_t raw = adc_read();     /* 0..4095 */
            float v       = raw / 4095.0f; /* 0..1 */
            float x       = v - 0.5f;
            if (x < 0) x = -x;

            env = MIC_ENV_ALPHA * x + (1.0f - MIC_ENV_ALPHA) * env;

            TickType_t now = xTaskGetTickCount();
            if (!on && env > MIC_ON_THR) {
                on   = true;
                t_on = now;
            } else if (on && env < MIC_OFF_THR) {
                on = false;
                TickType_t dur = now - t_on;
                morse_sym_t s =
                    (dur <= pdMS_TO_TICKS(MIC_DOT_MAX_MS)) ? SYM_DOT : SYM_DASH;
                xQueueSend(q_sym_out, &s, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif /* USE_MIC */

/* =====================  ENCODER: SYMBOLS -> MORSE STREAM  ===================== */

static inline void tx_put_char(char c) {
    xQueueSend(q_tx_bytes, &c, portMAX_DELAY);
}

static void task_encoder(void *arg) {
    (void)arg;
    bool in_char = false;
    morse_sym_t s;

    for (;;) {
        if (xQueueReceive(q_sym_out, &s, portMAX_DELAY) == pdTRUE) {
            switch (s) {
            case SYM_DOT:
            case SYM_DASH:
                tx_put_char((char)s);
                in_char = true;
                g_phase = PS_HAVE_CHAR;
                break;

            case SYM_SP:
                if (in_char) {
                    tx_put_char(' ');  /* one space between characters */
                    in_char = false;
                }
                break;

            case SYM_END:
                /* end of message */
                tx_put_char(MSG_END_C1);
                tx_put_char(MSG_END_C2);
                tx_put_char(MSG_END_C3);
                g_phase = PS_SENDING;
                in_char = false;
                break;
            }
        }
    }
}

/* =====================  USB TX & RX TASKS  ===================== */

static void task_usb_tx(void *arg) {
    (void)arg;
    char c;

    for (;;) {
        if (xQueueReceive(q_tx_bytes, &c, portMAX_DELAY) == pdTRUE) {
            while (!tud_cdc_connected())
                vTaskDelay(pdMS_TO_TICKS(10));

            tud_task();
            tud_cdc_write_char((uint8_t)c);
            tud_cdc_write_flush();
        }
    }
}

static void task_usb_rx(void *arg) {
    (void)arg;

    for (;;) {
        tud_task();

        if (tud_cdc_available()) {
            char buf[64];
            uint32_t n = tud_cdc_read(buf, sizeof buf);
            for (uint32_t i = 0; i < n; ++i) {
                char ch = buf[i];
                if (ch == '.' || ch == '-' || ch == ' ' || ch == '\n') {
                    xQueueSend(q_rx_bytes, &ch, 0);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* =====================  FEEDBACK: BUZZER + RGB LED  ===================== */

static void fb_dot(void) {
    rgb_led_write(0, 0, 255);            /* blue */
    buzzer_play_tone(660, 150);
    vTaskDelay(pdMS_TO_TICKS(180));
    rgb_led_write(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(40));
}

static void fb_dash(void) {
    rgb_led_write(0, 255, 0);            /* green */
    buzzer_play_tone(660, 450);
    vTaskDelay(pdMS_TO_TICKS(480));
    rgb_led_write(255, 255, 255);
    vTaskDelay(pdMS_TO_TICKS(60));
}

static void fb_jingle(void) {
    buzzer_play_tone(784, 120);
    vTaskDelay(pdMS_TO_TICKS(140));
    buzzer_play_tone(988, 120);
    vTaskDelay(pdMS_TO_TICKS(140));
    buzzer_play_tone(1175, 160);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void task_feedback(void *arg) {
    (void)arg;

    for (;;) {
        char c;
        if (xQueueReceive(q_rx_bytes, &c, portMAX_DELAY) == pdTRUE) {
            if (c == '.') {
                fb_dot();
            } else if (c == '-') {
                fb_dash();
            } else if (c == ' ') {
                /* word/char gap */
                rgb_led_write(255, 0, 0);      /* red blink */
                vTaskDelay(pdMS_TO_TICKS(100));
                rgb_led_write(255, 255, 255);
            } else if (c == '\n') {
                /* end of message from host */
                fb_jingle();
                g_phase = PS_RX_READY;
            }
        }
    }
}

/* =====================  MAIN  ===================== */

int main(void) {
    stdio_init_all();
    tusb_init();

    init_hat_sdk();
    sleep_ms(200);

    init_led();
    init_rgb_led();
    init_buzzer();
    init_button1();
    init_button2();

    /* create queues */
    q_sym_out  = xQueueCreate(32,  sizeof(morse_sym_t));
    q_tx_bytes = xQueueCreate(256, sizeof(char));
    q_rx_bytes = xQueueCreate(256, sizeof(char));

    /* create tasks */
    xTaskCreate(task_input_imu_btn, "imu", 1024, NULL, 2, NULL);
#if USE_MIC
    xTaskCreate(task_input_mic,     "mic", 1024, NULL, 2, NULL);
#endif
    xTaskCreate(task_encoder,       "enc", 1024, NULL, 2, NULL);
    xTaskCreate(task_usb_tx,        "utx", 1024, NULL, 1, NULL);
    xTaskCreate(task_usb_rx,        "urx", 1024, NULL, 1, NULL);
    xTaskCreate(task_feedback,      "fbk", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    /* should never reach here */
    while (1) { }
    return 0;
}
