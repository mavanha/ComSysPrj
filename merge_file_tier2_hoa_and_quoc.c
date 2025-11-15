#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <tkjhat/sdk.h>   // TKJHAT SDK: IMU, MEMS mic, buzzer, LEDs, buttons

// ------------------- Button / mode config -------------------

// From pins.h:
//   SW1_PIN -> 2
//   SW2_PIN -> 22
#define BTN_MODE          SW1_PIN   // SW1: short = IMU/MIC toggle, long = send message
#define BTN_ACTION        SW2_PIN   // SW2: short = confirm symbol, long = space

#define BTN_LONG_PRESS_MS   800     // >= this → long press
#define BTN_DEBOUNCE_MS     150     // ignore changes faster than this

// ------------------- FreeRTOS task config -------------------

#define INPUT_TASK_STACK_SIZE   2048
#define COMM_TASK_STACK_SIZE    1024
#define BUZZER_TASK_STACK_SIZE  1024

#define INPUT_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)
#define COMM_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define BUZZER_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

// Queues
#define SYMBOL_QUEUE_LENGTH     64   // char: '.', '-', ' ', '\n'
#define EVENT_QUEUE_LENGTH      8

// ------------------- Types -------------------

typedef enum {
    INPUT_MODE_IMU = 0,
    INPUT_MODE_MIC = 1
} input_mode_t;

typedef enum {
    ORIENT_UNKNOWN = 0,
    ORIENT_DOT,
    ORIENT_DASH
} orientation_t;

typedef enum {
    APP_EVENT_MSG_SENT = 0
} app_event_t;

typedef enum {
    MIC_STATE_IDLE = 0,
    MIC_STATE_ACTIVE
} mic_state_t;

// ------------------- IMU thresholds -------------------

// We don’t assume sign; we just check which axis has the largest magnitude.
#define ORIENT_AXIS_THRESH_MAIN   0.6f   // required magnitude on main axis

// ------------------- Microphone thresholds -------------------

// MEMS_SAMPLING_FREQUENCY = 8000 Hz, MEMS_BUFFER_SIZE = 256 in SDK
#define MIC_AMPL_THRESHOLD   8000   // average absolute amplitude threshold (tune!)
#define MIC_DOT_MAX_MS       250    // <= this duration → dot '.', longer → dash '-'

// Input task loop rate
#define INPUT_TASK_PERIOD_MS 20

// ------------------- Globals -------------------

static QueueHandle_t xSymbolQueue = NULL;    // char
static QueueHandle_t xEventQueue  = NULL;    // app_event_t

static input_mode_t g_inputMode = INPUT_MODE_IMU;

// Microphone data
static volatile int g_mic_samples_ready = 0;
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE];

// ------------------- Microphone callback -------------------

static void on_pdm_samples_ready(void)
{
    int n = get_microphone_samples(g_mic_buffer, MEMS_BUFFER_SIZE);
    if (n > 0) {
        g_mic_samples_ready = n;
    }
}

// ------------------- IMU orientation helper -------------------

static orientation_t detect_orientation(void)
{
    float ax, ay, az, gx, gy, gz, t;
    if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) != 0) {
        return ORIENT_UNKNOWN;
    }

    float absx = fabsf(ax);
    float absy = fabsf(ay);
    float absz = fabsf(az);

    if (absz > absx && absz > absy && absz > ORIENT_AXIS_THRESH_MAIN) {
        return ORIENT_DOT;   // board “flat-ish”
    }

    if (absx > absz && absx > absy && absx > ORIENT_AXIS_THRESH_MAIN) {
        return ORIENT_DASH;  // board “on its side-ish”
    }

    return ORIENT_UNKNOWN;
}

// ------------------- Buzzer melody -------------------

static void play_message_sent_melody(void)
{
    buzzer_play_tone(880, 150);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(660, 150);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(440, 300);
}

// ------------------- Symbol helpers -------------------

static void send_symbol(char c)
{
    xQueueSend(xSymbolQueue, &c, 0);
}

static void send_space(void)
{
    char space = ' ';
    send_symbol(space);
}

static void send_message_end(void)
{
    char space = ' ';
    char nl    = '\n';

    send_symbol(space);
    send_symbol(space);
    send_symbol(nl);

    app_event_t evt = APP_EVENT_MSG_SENT;
    xQueueSend(xEventQueue, &evt, 0);
}

// ------------------- Input task -------------------

static void vInputTask(void *pvParameters)
{
    (void)pvParameters;

    bool       sw1_prev = false;
    bool       sw2_prev = false;
    TickType_t sw1_press_tick = 0;
    TickType_t sw2_press_tick = 0;
    TickType_t sw1_last_change = 0;
    TickType_t sw2_last_change = 0;

    init_led();
    init_buzzer();
    init_sw1();
    init_sw2();

    // IMU init
    if (init_ICM42670() == 0) {
        printf("__IMU INIT OK__\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("__IMU START DEFAULT FAILED__\n");
        }
    } else {
        printf("__IMU INIT FAILED__\n");
    }

    // Mic init
    if (init_pdm_microphone() == 0) {
        pdm_microphone_set_callback(on_pdm_samples_ready);
        if (init_microphone_sampling() == 0) {
            printf("__MIC INIT OK__\n");
        } else {
            printf("__MIC START FAILED__\n");
        }
    } else {
        printf("__MIC INIT FAILED__\n");
    }

    mic_state_t micState = MIC_STATE_IDLE;
    TickType_t  micStartTick = 0;

    g_inputMode = INPUT_MODE_IMU;
    printf("__IMU MODE__\n");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        bool sw1_now = gpio_get(BTN_MODE) ? true : false;
        bool sw2_now = gpio_get(BTN_ACTION) ? true : false;

        // -------- SW1 (MODE / MESSAGE) with debounce --------
        if (sw1_now != sw1_prev &&
            (now - sw1_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {

            sw1_last_change = now;

            if (sw1_now) {
                sw1_press_tick = now;
            } else {
                TickType_t dt = now - sw1_press_tick;
                uint32_t ms   = dt * portTICK_PERIOD_MS;

                if (ms >= BTN_LONG_PRESS_MS) {
                    printf("__MSG SEND__\n");
                    send_message_end();      // triggers buzzer melody via event queue
                } else {
                    if (g_inputMode == INPUT_MODE_IMU) {
                        g_inputMode = INPUT_MODE_MIC;
                        printf("__MIC MODE__\n");
                    } else {
                        g_inputMode = INPUT_MODE_IMU;
                        printf("__IMU MODE__\n");
                        micState = MIC_STATE_IDLE;
                    }
                }
            }
        }
        sw1_prev = sw1_now;

        // -------- SW2 (SYMBOL / SPACE) with debounce --------
        if (sw2_now != sw2_prev &&
            (now - sw2_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {

            sw2_last_change = now;

            if (sw2_now) {
                sw2_press_tick = now;
            } else {
                TickType_t dt = now - sw2_press_tick;
                uint32_t ms   = dt * portTICK_PERIOD_MS;

                if (ms >= BTN_LONG_PRESS_MS) {
                    send_space();
                    printf("__SPACE__\n");
                } else {
                    if (g_inputMode == INPUT_MODE_IMU) {
                        orientation_t o = detect_orientation();
                        char symbol;
                        if (o == ORIENT_DOT) {
                            symbol = '.';
                            send_symbol(symbol);
                            printf("__DOT FROM IMU__\n");
                        } else if (o == ORIENT_DASH) {
                            symbol = '-';
                            send_symbol(symbol);
                            printf("__DASH FROM IMU__\n");
                        } else {
                            printf("__UNKNOWN ORIENTATION__\n");
                        }
                    } else {
                        printf("__SW2 SHORT IN MIC MODE__\n");
                    }
                }
            }
        }
        sw2_prev = sw2_now;

        // -------- MIC processing (only in MIC mode) --------
        if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
            int sample_count = g_mic_samples_ready;
            g_mic_samples_ready = 0;

            if (sample_count > 0) {
                int64_t sumAbs = 0;
                for (int i = 0; i < sample_count; i++) {
                    int32_t s = g_mic_buffer[i];
                    if (s < 0) s = -s;
                    sumAbs += s;
                }
                uint32_t avgAbs = (uint32_t)(sumAbs / sample_count);

                switch (micState) {
                    case MIC_STATE_IDLE:
                        if (avgAbs > MIC_AMPL_THRESHOLD) {
                            micState = MIC_STATE_ACTIVE;
                            micStartTick = now;
                        }
                        break;

                    case MIC_STATE_ACTIVE:
                        if (avgAbs <= MIC_AMPL_THRESHOLD) {
                            TickType_t durationTicks = now - micStartTick;
                            uint32_t durationMs = durationTicks * portTICK_PERIOD_MS;

                            char symbol;
                            if (durationMs <= MIC_DOT_MAX_MS) {
                                symbol = '.';
                                printf("__MIC DOT (%lu ms)__\n",
                                       (unsigned long)durationMs);
                            } else {
                                symbol = '-';
                                printf("__MIC DASH (%lu ms)__\n",
                                       (unsigned long)durationMs);
                            }
                            send_symbol(symbol);
                            micState = MIC_STATE_IDLE;
                        }
                        break;

                    default:
                        micState = MIC_STATE_IDLE;
                        break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(INPUT_TASK_PERIOD_MS));
    }
}

// ------------------- Communication task -------------------

static void vCommTask(void *pvParameters)
{
    (void)pvParameters;
    char symbol;

    while (1) {
        if (xQueueReceive(xSymbolQueue, &symbol, portMAX_DELAY) == pdTRUE) {
            putchar(symbol);
            fflush(stdout);
        }
    }
}

// ------------------- Buzzer task -------------------

static void vBuzzerTask(void *pvParameters)
{
    (void)pvParameters;
    app_event_t evt;

    while (1) {
        if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
            if (evt == APP_EVENT_MSG_SENT) {
                play_message_sent_melody();   // <--- MUSIC/JINGLE HERE
            }
        }
    }
}

// ------------------- main -------------------

int main(void)
{
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    init_hat_sdk();
    sleep_ms(300);

    xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char));
    xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t));

    if (xSymbolQueue == NULL || xEventQueue == NULL) {
        while (1) { }
    }

    xTaskCreate(vInputTask,  "Input",  INPUT_TASK_STACK_SIZE,  NULL,
                INPUT_TASK_PRIORITY,  NULL);
    xTaskCreate(vCommTask,   "Comm",   COMM_TASK_STACK_SIZE,   NULL,
                COMM_TASK_PRIORITY,   NULL);
    xTaskCreate(vBuzzerTask, "Buzzer", BUZZER_TASK_STACK_SIZE, NULL,
                BUZZER_TASK_PRIORITY, NULL);

    vTaskStartScheduler();

    while (1) { }
    return 0;
}
