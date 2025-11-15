#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <tkjhat/sdk.h>     // TKJHAT SDK: IMU, MEMS mic, buzzer, LEDs, etc.

// ------------------- Morse / Project config -------------------

// Button mapping on JTKJ HAT
// From pins.h:
//  SW1_PIN -> 2
//  SW2_PIN -> 22
#define BTN_MODE          SW1_PIN   // Short press: toggle IMU/MIC
                                    // Long press: send message ("  \n")
#define BTN_ACTION        SW2_PIN   // Short press: confirm symbol
                                    // Long press: send space ' '

// Timing for buttons (in ms)
#define BTN_LONG_PRESS_MS       800   // >= this → long press

// FreeRTOS task config
#define INPUT_TASK_STACK_SIZE   2048
#define COMM_TASK_STACK_SIZE    1024
#define BUZZER_TASK_STACK_SIZE  1024

#define INPUT_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)
#define COMM_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define BUZZER_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

// Queue sizes
#define SYMBOL_QUEUE_LENGTH     64
#define EVENT_QUEUE_LENGTH      8

// Input mode: IMU orientation vs Microphone sound
typedef enum {
    INPUT_MODE_IMU = 0,
    INPUT_MODE_MIC = 1
} input_mode_t;

// IMU-based orientation → dot / dash
typedef enum {
    ORIENT_UNKNOWN = 0,
    ORIENT_DOT,
    ORIENT_DASH
} orientation_t;

// Simple app events for buzzer, etc.
typedef enum {
    APP_EVENT_MSG_SENT = 0
} app_event_t;

// Mic state machine
typedef enum {
    MIC_STATE_IDLE = 0,
    MIC_STATE_ACTIVE
} mic_state_t;

// ------------- IMU orientation thresholds -------------
// Values are in "g" units because TKJHAT SDK already scales.
// Typically when flat on table: az ≈ -1.0, ax≈0, ay≈0
// When on side: ax ≈ +1.0, others ≈ 0
#define ORIENT_AXIS_THRESH_MAIN   0.6f   // main axis must be larger than this
#define ORIENT_AXIS_THRESH_OTHER  0.5f   // other axes must be within ±this

// ------------- Microphone thresholds (tune if needed) -------------
// MEMS_SAMPLING_FREQUENCY = 8000 Hz, MEMS_BUFFER_SIZE = 256 → ~32 ms of audio per buffer
#define MIC_AMPL_THRESHOLD     8000   // average absolute amplitude threshold (tune!)
#define MIC_DOT_MAX_MS         250    // <= this duration → dot '.', longer → dash '-'

// Input task loop period
#define INPUT_TASK_PERIOD_MS   20


// ------------------- Globals -------------------

static QueueHandle_t xSymbolQueue = NULL;  // char: '.', '-', ' ', '\n'
static QueueHandle_t xEventQueue  = NULL;  // app_event_t

// Current input mode (only touched in InputTask, read-only elsewhere)
static input_mode_t g_inputMode = INPUT_MODE_IMU;

// Microphone sample buffer and flag (filled by callback)
static volatile int g_mic_samples_ready = 0;
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE];

// ------------------- Microphone callback -------------------

// Called from PDM library ISR when new samples are ready
static void on_pdm_samples_ready(void)
{
    // Read samples into our buffer; keep ISR short
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

    // DOT: board flat (Z ≈ -1)
    if ( (az < -ORIENT_AXIS_THRESH_MAIN) &&
         (fabsf(ax) < ORIENT_AXIS_THRESH_OTHER) &&
         (fabsf(ay) < ORIENT_AXIS_THRESH_OTHER) ) {
        return ORIENT_DOT;
    }

    // DASH: board on its side (X ≈ +1)
    if ( (ax > ORIENT_AXIS_THRESH_MAIN) &&
         (fabsf(ay) < ORIENT_AXIS_THRESH_OTHER) &&
         (fabsf(az) < ORIENT_AXIS_THRESH_OTHER) ) {
        return ORIENT_DASH;
    }

    return ORIENT_UNKNOWN;
}

// ------------------- Buzzer melody -------------------

static void play_message_sent_melody(void)
{
    // Simple 3-note indication using TKJHAT buzzer (blocking, but in separate task)
    buzzer_play_tone(880, 150);   // A5 short
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(660, 150);   // E5 short
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(440, 300);   // A4 longer
}

// ------------------- Helper: send symbols via queue -------------------

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
    // Protocol: two spaces + '\n'
    char space = ' ';
    char nl    = '\n';
    send_symbol(space);
    send_symbol(space);
    send_symbol(nl);

    // Notify buzzer task
    app_event_t evt = APP_EVENT_MSG_SENT;
    xQueueSend(xEventQueue, &evt, 0);
}

// ------------------- Input task -------------------

static void vInputTask(void *pvParameters)
{
    (void)pvParameters;

    // --- Local button state for edge & long-press detection ---
    bool sw1_prev = false;
    bool sw2_prev = false;
    TickType_t sw1_press_tick = 0;
    TickType_t sw2_press_tick = 0;

    // --- Init hardware that depends on SDK / RTOS ---
    init_led();       // red LED
    init_buzzer();    // buzzer pin
    init_sw1();
    init_sw2();

    // Init IMU
    if (init_ICM42670() == 0) {
        printf("__IMU INIT OK__\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("__IMU START DEFAULT FAILED__\n");
        }
    } else {
        printf("__IMU INIT FAILED__\n");
    }

    // Init microphone (PDM MEMS)
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

    printf("__IMU MODE__\n");
    g_inputMode = INPUT_MODE_IMU;

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // -------- Read buttons (active high) --------
        bool sw1_now = gpio_get(BTN_MODE) ? true : false;
        bool sw2_now = gpio_get(BTN_ACTION) ? true : false;

        // -------- SW1: MODE / SEND MESSAGE (short vs long) --------
        if (sw1_now && !sw1_prev) {
            // rising edge: store press time
            sw1_press_tick = now;
        }
        if (!sw1_now && sw1_prev) {
            // falling edge: released
            TickType_t dt = now - sw1_press_tick;
            uint32_t ms = dt * portTICK_PERIOD_MS;
            if (ms >= BTN_LONG_PRESS_MS) {
                // Long press → send message end
                printf("__MSG SEND__\n");
                send_message_end();
            } else {
                // Short press → toggle mode
                if (g_inputMode == INPUT_MODE_IMU) {
                    g_inputMode = INPUT_MODE_MIC;
                    printf("__MIC MODE__\n");
                } else {
                    g_inputMode = INPUT_MODE_IMU;
                    printf("__IMU MODE__\n");
                    // reset mic state when leaving MIC mode
                    micState = MIC_STATE_IDLE;
                }
            }
        }
        sw1_prev = sw1_now;

        // -------- SW2: CONFIRM SYMBOL / SPACE (short vs long) --------
        if (sw2_now && !sw2_prev) {
            sw2_press_tick = now;
        }
        if (!sw2_now && sw2_prev) {
            TickType_t dt = now - sw2_press_tick;
            uint32_t ms = dt * portTICK_PERIOD_MS;

            if (ms >= BTN_LONG_PRESS_MS) {
                // Long press SW2 → send SPACE (independent of mode)
                send_space();
                printf("__SPACE__\n");
            } else {
                // Short press SW2 → action depends on mode
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
                    // In MIC mode we don't use SW2 short; you could
                    // add extra functionality here if you like.
                    printf("__SW2 SHORT IN MIC MODE__\n");
                }
            }
        }
        sw2_prev = sw2_now;

        // -------- MIC processing (only when in MIC mode) --------
        if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
            // Take ownership of samples
            int sample_count = g_mic_samples_ready;
            g_mic_samples_ready = 0;

            if (sample_count > 0) {
                // Compute average absolute amplitude
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

        // Small delay to set task "sample rate" (~50 Hz)
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
            // Send symbol to Serial Client: '.', '-', ' ', '\n'
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
                play_message_sent_melody();
            }
        }
    }
}

// ------------------- main -------------------

int main(void)
{
    // Initialize stdio over USB (CDC) – used by Serial Client
    stdio_init_all();

    // OPTIONAL: wait for USB connection so you see prints immediately
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    // Initialize HAT basic stuff (I2C, stop RGB)
    init_hat_sdk();
    sleep_ms(300); // let USB & HAT settle

    // Create queues
    xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char));
    xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t));
    if (xSymbolQueue == NULL || xEventQueue == NULL) {
        // Failed to create queues; nothing we can do
        while (1) { }
    }

    // Create tasks
    xTaskCreate(vInputTask,  "Input",  INPUT_TASK_STACK_SIZE,  NULL,
                INPUT_TASK_PRIORITY,  NULL);
    xTaskCreate(vCommTask,   "Comm",   COMM_TASK_STACK_SIZE,   NULL,
                COMM_TASK_PRIORITY,   NULL);
    xTaskCreate(vBuzzerTask, "Buzzer", BUZZER_TASK_STACK_SIZE, NULL,
                BUZZER_TASK_PRIORITY, NULL);

    // Start FreeRTOS scheduler (never returns)
    vTaskStartScheduler();

    // Should never reach here
    while (1) { }
    return 0;
}
