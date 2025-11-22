#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h> // for abs()


// Pico SDK headers
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/cyw43_arch.h" // Added for Wi-Fi functionality


// FreeRTOS headers
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>


// TKJHAT SDK header
#include "tkjhat/sdk.h"


// --- MERGE START: From Group Code (Configuration) ---
// Button / mode config
#define BTN_MODE            BUTTON1 // SW1: short = IMU/MIC toggle, long = send 3 spaces (message)
#define BTN_ACTION          BUTTON2 // SW2: short = confirm symbol, long = space
#define BTN_LONG_PRESS_MS   800     // >= this -> long press
#define BTN_DEBOUNCE_MS     150     // ignore changes faster than this
#define MIC_AMPL_THRESHOLD  8000    // average absolute amplitude threshold (tune!)
#define MIC_DOT_MAX_MS      250     // <= this duration -> dot '.', longer -> dash '-'
#define INPUT_TASK_PERIOD_MS 20     // Input task loop rate


// Queues
#define SYMBOL_QUEUE_LENGTH     64   // char: '.', '-', ' ', '\n'
#define EVENT_QUEUE_LENGTH      8


// Types
typedef enum {
    INPUT_MODE_IMU = 0,
    INPUT_MODE_MIC = 1
} input_mode_t;


typedef enum {
    MIC_STATE_IDLE = 0,
    MIC_STATE_ACTIVE
} mic_state_t;


typedef enum {
    APP_EVENT_MSG_SENT = 0
} app_event_t;


// Globals
static input_mode_t g_inputMode = INPUT_MODE_IMU;
static volatile int g_mic_samples_ready = 0;
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE]; // MEMS_BUFFER_SIZE from sdk.h


static QueueHandle_t xSymbolQueue = NULL; // This will replace xSerialTxQueue for *internal* symbol passing
static QueueHandle_t xEventQueue  = NULL; // For internal app events


// Microphone callback
static void on_pdm_samples_ready(void) {
    int n = get_microphone_samples(g_mic_buffer, MEMS_BUFFER_SIZE);
    if (n > 0) {
        g_mic_samples_ready = n;
    }
}


// Function prototype for IMU/MIC input task
static void input_task(void *pvParameters);
static void vBuzzerTask(void *pvParameters);


// --- MERGE END ---


#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2)


// Queue lengths (existing, some may be adjusted or replaced)
#define SERIAL_TX_QUEUE_LENGTH 20 // This queue will now feed from xSymbolQueue
#define PLAYBACK_QUEUE_LENGTH 40
#define SERIAL_RX_BUFFER_SIZE 128


// --- IMU CONSTANTS ---
// Tier 1: Static Tilt Thresholds (approx 0.9g)
#define IMU_TILT_THRESHOLD      0.9f
#define IMU_NEUTRAL_THRESHOLD   0.5f
// Tier 2: Movement (Jerk) Thresholds (change in g)
// 0.4g is a good sensitivity for a hand shake
#define IMU_JERK_THRESHOLD      0.4f
#define IMU_DEBOUNCE_MS         300


// State machine for IMU symbol generation
volatile enum AppState {
  STATE_IDLE,  
  STATE_ARMED,  
  STATE_COOLDOWN
} g_appState = STATE_IDLE; // This state machine is primarily for IMU input.


// FreeRTOS Queues
// xSerialTxQueue will now *receive* from xSymbolQueue and then send to actual serial
QueueHandle_t xSerialTxQueue_actual; // Renamed to avoid conflict and clarify role
QueueHandle_t xPlaybackQueue;
QueueHandle_t i2cMutex;


// Morse-to-Text translation (unchanged)
struct MorseAlphabet {
  char morseCode[7];
  char letter;
};


struct MorseAlphabet morseCodes[40] = {
  {".-", 'a'}, {"-...", 'b'}, {"-.-.", 'c'}, {"-..", 'd'}, {".", 'e'},
  {"..-.", 'f'}, {"--.", 'g'}, {"....", 'h'}, {"..", 'i'}, {".---", 'j'},
  {"-.-", 'k'}, {".-..", 'l'}, {"--", 'm'}, {"-.", 'n'}, {"---", 'o'},
  {".--.", 'p'}, {"--.-", 'q'}, {".-.", 'r'}, {"...", 's'}, {"-", 't'},
  {"..-", 'u'}, {"...-", 'v'}, {"", 'w'}, {"-..-", 'x'}, {"-.--", 'y'},
  {"--..", 'z'}, {"-----", '0'}, {".----", '1'}, {"..---", '2'},
  {"...--", '3'}, {"....-", '4'}, {".....", '5'}, {"-....", '6'},
  {"--...", '7'}, {"---..", '8'}, {"----.", '9'}, {".-.-.-", '.'},
  {"--..--", ','}, {"..--..", '?'}, {"-.-.--", '!'}, {"", ' '}
};


/* =========================
* FUNCTION PROTOTYPES
* ========================= */
// static void imu_task(void *pvParameters); // Renamed to input_task
static void serial_tx_task(void *pvParameters);
static void serial_rx_task(void *pvParameters);
static void playback_task(void *pvParameters);
// static void button_isr_callback(uint gpio, uint32_t eventMask); // Replaced by input_task logic
char find_letter_from_morse_code(char *morseCode);
void process_received_line(char *line);


// --- MERGE START: Helper functions from Group Code ---
static void send_symbol(char c) {
    xQueueSend(xSymbolQueue, &c, 0); // Send to the internal symbol queue
}


static void send_space(void) {
    char space = ' ';
    send_symbol(space);
}


static void play_message_sent_melody(void) {
    buzzer_play_tone(880, 150);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(660, 150);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(440, 300);
}
// --- MERGE END ---


int main() {
    // 1. Initialize STDIO
    stdio_init_all();


    // --- FIX: Add a timeout to prevent "White Screen of Death" ---
    int i = 0;
    while (!stdio_usb_connected() && i < 30) { // Wait max 3 seconds
        sleep_ms(100);
        i++;
    }
    
    printf("__JTKJ Morse Communicator - Starting...__\n");


    // ========================================================================
    // --- MERGE SECTION: WI-FI INITIALIZATION ---
    // This block initializes the CYW43 chip and connects to the network
    // before the FreeRTOS scheduler takes over.
    // ========================================================================
    printf("Init the cyw43\n");
    if (cyw43_arch_init()) {
        printf("WiFi init failed! (Continuing without Wi-Fi)\n");
        // We continue instead of returning -1 so the Morse app still works offline
    } else {
        printf("Connecting to Wi-Fi\n");
        cyw43_arch_enable_sta_mode();
        printf("Set stable mode successfully -> connect to wifi\n");
        
        // Connecting to the open panoulu-network (no password needed)
        // We try to connect for 30 seconds
        if(cyw43_arch_wifi_connect_timeout_ms("panoulu", NULL, CYW43_AUTH_OPEN, 30 * 1000)) {
            printf("Failed to connect to Wi-Fi\n");
        } else {
            printf("Connected to hotspot\n");
        }
    }
    // ========================================================================


    // 2. Initialize Hardware (Sensors, Display, etc)
    init_hat_sdk(); 
    init_led();   
    init_rgb_led();
    init_buzzer();
    init_display();


    // --- MERGE START: IMU and MIC init from Group Code ---
    if (init_ICM42670() == 0) {
        printf("__IMU INIT OK__\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("__IMU START DEFAULT FAILED__\n");
        }
    } else {
        printf("__IMU INIT FAILED__\n");
    }


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
    // --- MERGE END ---


    // Display message
    clear_display();
    write_text_xy(0, 0, "Morse App Ready");
    write_text_xy(0, 10, "SW1=Mode, SW2=Action"); 
    write_text_xy(0, 30, "RX MSG:");


    // --- FreeRTOS Queues ---
    xSerialTxQueue_actual = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(char)); 
    xPlaybackQueue = xQueueCreate(PLAYBACK_QUEUE_LENGTH, sizeof(char));
    i2cMutex = xSemaphoreCreateMutex();
    
    // --- MERGE START: New queues from Group Code ---
    xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char)); // Internal queue for input symbols
    xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t)); // For app events (e.g., message sent)
    // --- MERGE END ---


    if (xSerialTxQueue_actual == NULL || xPlaybackQueue == NULL || i2cMutex == NULL || xSymbolQueue == NULL || xEventQueue == NULL) {
        printf("__CRITICAL ERROR: Could not create queues or mutex__\n");
        while (1) { blink_led(1); sleep_ms(100); } // Indicate error with LED
    }


    // --- STACK FIX: Increased stack sizes to prevent crash ---
    xTaskCreate(input_task, "InputTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_tx_task, "TxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_rx_task, "RxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(playback_task, "PlaybackTask", 4096, NULL, MAIN_TASK_PRIORITY, NULL);
    // --- MERGE START: Buzzer Task from Group Code ---
    xTaskCreate(vBuzzerTask, "BuzzerTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);
    // --- MERGE END ---


    printf("__Initialization complete. Starting scheduler.__\n");


    // --- Start FreeRTOS Scheduler ---
    vTaskStartScheduler();


    while (1); // Should not reach here
    return 0;
}


// Renamed and extended imu_task to input_task
static void input_task(void *pvParameters) {
    (void)pvParameters;


    // IMU variables
    float ax, ay, az, gx, gy, gz, t;
    float prev_ax = 0, prev_ay = 0, prev_az = 0;
    bool first_run_imu = true; 
    TickType_t last_action_time_imu = 0; 
    bool read_ok_imu = false; 


    // --- MERGE START: Button & MIC variables from Group Code ---
    bool       sw1_prev = false;
    bool       sw2_prev = false;
    TickType_t sw1_press_tick = 0;
    TickType_t sw2_press_tick = 0;
    TickType_t sw1_last_change = 0;
    TickType_t sw2_last_change = 0;


    mic_state_t micState = MIC_STATE_IDLE;
    TickType_t  micStartTick = 0;
    // --- MERGE END ---


    init_sw1(); // Initialize buttons
    init_sw2();


    g_inputMode = INPUT_MODE_IMU; // Start in IMU mode
    printf("__IMU MODE__\n");


    while (1) {
        TickType_t now = xTaskGetTickCount();


        // --- MERGE START: Button polling and long/short press logic from Group Code ---
        bool sw1_now = gpio_get(BTN_MODE) ? true : false;
        bool sw2_now = gpio_get(BTN_ACTION) ? true : false;


        // -------- SW1 (MODE / SEND 3 SPACES) with debounce --------
        if (sw1_now != sw1_prev &&
            (now - sw1_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw1_last_change = now;
            if (sw1_now) { // Button SW1 pressed down
                buzzer_play_tone(1000, 80); // Short beep
                sw1_press_tick = now;
            } else { // Button SW1 released
                TickType_t dt = now - sw1_press_tick;
                uint32_t ms   = dt * portTICK_PERIOD_MS;
                if (ms >= BTN_LONG_PRESS_MS) {
                    // Long press SW1: send 3 spaces = end of message
                    printf("__MSG SEND VIA 3 SPACES__\n");
                    send_space();
                    send_space();
                    send_space();
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0); // Notify buzzer task
                } else {
                    // Short press SW1: toggle mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        g_inputMode = INPUT_MODE_MIC;
                        printf("__MIC MODE__\n");
                        micState = MIC_STATE_IDLE; // Reset MIC state when switching
                    } else {
                        g_inputMode = INPUT_MODE_IMU;
                        printf("__IMU MODE__\n");
                        g_appState = STATE_IDLE; // Reset IMU state when switching
                        set_led_status(false); // Turn off LED if it was on from IMU armed state
                    }
                }
            }
        }
        sw1_prev = sw1_now;


        // -------- SW2 (SYMBOL / SPACE) with debounce --------
        if (sw2_now != sw2_prev &&
            (now - sw2_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw2_last_change = now;
            if (sw2_now) { // Button SW2 pressed down
                buzzer_play_tone(700, 80); // Different short beep
                sw2_press_tick = now;
            } else { // Button SW2 released
                TickType_t dt = now - sw2_press_tick;
                uint32_t ms   = dt * portTICK_PERIOD_MS;
                if (ms >= BTN_LONG_PRESS_MS) {
                    // Long press SW2 -> SPACE (for letters / words)
                    send_space();
                    printf("__SPACE__\n");
                } else {
                    // Short press SW2 -> action based on current mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        // Original "Arm" functionality of SW1 from functional code
                        if (g_appState == STATE_IDLE) {
                            g_appState = STATE_ARMED;
                            set_led_status(true);
                            printf("__IMU ARMED__\n");
                        } else {
                            // If already armed, use SW2 short press to generate a symbol based on current orientation
                            // This replaces the old IMU task's direct symbol generation.
                            // Now, the IMU task *only* generates symbols when explicitly triggered by SW2 short press.
                             if (g_appState == STATE_ARMED) {
                                 // Try to read IMU data
                                 read_ok_imu = false;
                                 if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                     if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                                         read_ok_imu = true;
                                     } else {
                                         printf("__IMU Task: Failed to read sensor data for SW2__\n");
                                     }
                                     xSemaphoreGive(i2cMutex);
                                 }


                                 if (read_ok_imu) {
                                     // Simplified orientation check based on IMU_TILT_THRESHOLD
                                     char symbol = '?'; // Default to unknown
                                     if (az > IMU_TILT_THRESHOLD) { // Tilted along Z axis (e.g., forward)
                                         symbol = '.';
                                     } else if (ay < -IMU_TILT_THRESHOLD) { // Tilted along Y axis (e.g., left)
                                         symbol = '-';
                                     }


                                     if (symbol != '?') {
                                         send_symbol(symbol);
                                         printf("__IMU SYMBOL '%c'__\n", symbol);
                                         set_led_status(false); // Indicate symbol sent
                                         g_appState = STATE_COOLDOWN; // Enter cooldown after symbol
                                         last_action_time_imu = now;
                                     } else {
                                         printf("__IMU: No clear orientation for symbol.__\n");
                                     }


                                      // Update History (essential for potential jerk detection)
                                     prev_ax = ax; prev_ay = ay; prev_az = az;
                                 }
                            }
                        }
                    } else { // g_inputMode == INPUT_MODE_MIC
                        printf("__SW2 SHORT IN MIC MODE - no direct action__\n");
                    }
                }
            }
        }
        sw2_prev = sw2_now;
        // --- MERGE END ---


        // --- MERGE START: Original IMU state machine and continuous processing (modified) ---
        // The original IMU task had continuous processing.
        // We'll keep the COOLDOWN logic, but symbol generation is now button-triggered in IMU mode.
        if (g_inputMode == INPUT_MODE_IMU) {
             if (g_appState == STATE_COOLDOWN) {
                 // Wait until device is neutral (No Tilt)
                 read_ok_imu = false;
                 if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                         read_ok_imu = true;
                    }
                    xSemaphoreGive(i2cMutex);
                 }


                 if (read_ok_imu) {
                     if (fabs(az) < IMU_NEUTRAL_THRESHOLD && fabs(ay) < IMU_NEUTRAL_THRESHOLD) {
                         g_appState = STATE_IDLE; // Return to IDLE after cooling down
                         printf("__IMU COOLED DOWN (IDLE)__\n");
                     }
                 }
             }
        }
        // --- MERGE END ---


        // --- MERGE START: MIC processing from Group Code ---
        if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
            int sample_count = g_mic_samples_ready;
            g_mic_samples_ready = 0; // Reset flag immediately


            if (sample_count > 0) {
                int64_t sumAbs = 0;
                for (int loop_i = 0; loop_i < sample_count; loop_i++) {
                    int32_t s = g_mic_buffer[loop_i];
                    if (s < 0) s = -s;
                    sumAbs += s;
                }
                uint32_t avgAbs = (uint32_t)(sumAbs / sample_count);


                switch (micState) {
                    case MIC_STATE_IDLE:
                        if (avgAbs > MIC_AMPL_THRESHOLD) {
                            micState = MIC_STATE_ACTIVE;
                            micStartTick = now;
                            printf("__MIC ACTIVE (Threshold hit)__\n");
                        }
                        break;
                    case MIC_STATE_ACTIVE:
                        if (avgAbs <= MIC_AMPL_THRESHOLD) { // Sound ended
                            TickType_t durationTicks = now - micStartTick;
                            uint32_t durationMs = durationTicks * portTICK_PERIOD_MS;
                            char symbol;
                            if (durationMs <= MIC_DOT_MAX_MS) {
                                symbol = '.';
                                printf("__MIC DOT (%lu ms)__\n", (unsigned long)durationMs);
                            } else {
                                symbol = '-';
                                printf("__MIC DASH (%lu ms)__\n", (unsigned long)durationMs);
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
        // --- MERGE END ---


        vTaskDelay(pdMS_TO_TICKS(INPUT_TASK_PERIOD_MS));
    }
}


static void serial_tx_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    int spaceCount = 0; // counter


    // This task now receives from xSymbolQueue (internal input),
    // then forwards to actual serial output and handles space logic.
    while (1) {
        if (xQueueReceive(xSymbolQueue, &symbol, portMAX_DELAY) == pdPASS) { // Receive from the input symbol queue
            if (symbol == ' ') {
                spaceCount++;
                putchar(' '); // Always send the space
                fflush(stdout);
                if (spaceCount == 3) {
                    // 3rd space triggers newline and feedback
                    putchar('\n');
                    printf("\n__[Morse Send OK]__\n");
                    fflush(stdout);
                    spaceCount = 0; // Reset
                    // Notify playback task that a message segment is complete
                    char nl_char = '\n';
                    xQueueSend(xPlaybackQueue, &nl_char, 0); // Send a newline to playback
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0); // Also notify event queue for buzzer
                }
            } else {
                // Any non-space symbol resets counter and sends
                spaceCount = 0;
                putchar(symbol); // Send '.' or '-'
                fflush(stdout);
                // Also send to playback queue for local feedback on display
                xQueueSend(xPlaybackQueue, &symbol, 0);
            }
        }
    }
}


static void serial_rx_task(void *pvParameters) {
    (void)pvParameters;
    char lineBuffer[SERIAL_RX_BUFFER_SIZE];
    int linePos = 0;
    while (1) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                if (linePos > 0) {
                    lineBuffer[linePos] = '\0';
                    process_received_line(lineBuffer);
                    linePos = 0;
                }
            } else if (linePos < (SERIAL_RX_BUFFER_SIZE - 1)) {
                lineBuffer[linePos++] = (char)c;
            } else {
                linePos = 0; // Overflow!!
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void process_received_line(char *line) {
    if (strncmp(line, ".clear", 6) == 0) {
        printf("\033[2J\033[H"); // ANSI clear screen
        fflush(stdout);
        return;
    }
    if (strncmp(line, ".boot", 5) == 0) {
        printf("__Rebooting to BOOTSEL mode...__\n");
        sleep_ms(100); // Give time for message to send
        reset_usb_boot(0, 0); // Jump to BOOTSEL mode
        return;
    }
    if (strncmp(line, ".exit", 5) == 0) {
        printf("\n__--- .exit command received. Halting. ---__\n");
        fflush(stdout);
        while (1) {
            blink_led(1);
            sleep_ms(200);
        }
    }


    // --- Parse for Morse, skipping debug blocks ---
    bool in_debug_block = false;
    for (int loop_i = 0; line[loop_i] != '\0'; loop_i++) { // Renamed loop variable
        if (line[loop_i] == '_' && line[loop_i+1] == '_') {
            in_debug_block = !in_debug_block;
            loop_i++;
            continue;
        }
        if (!in_debug_block) {
            char symbol = line[loop_i];
            if (symbol == '.' || symbol == '-' || symbol == ' ') {
                xQueueSend(xPlaybackQueue, &symbol, portMAX_DELAY);
            }
        }
    }
    // --- Send newline to playback task (This will trigger display update for RX message) ---
    char nl = '\n';
    xQueueSend(xPlaybackQueue, &nl, portMAX_DELAY);
}


static void playback_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    char morseSymbolBuffer[10] = {0};
    int morseSymbolPos = 0;
    char textMessageBuffer[22] = {0};
    int textMessagePos = 0;


    while (1) {
        if (xQueueReceive(xPlaybackQueue, &symbol, portMAX_DELAY) == pdPASS) {
            // --- Play symbol on Actuators ---
            if (symbol == '.') {
                rgb_led_write(255, 255, 0);
                buzzer_play_tone(880, 150);
                vTaskDelay(pdMS_TO_TICKS(150));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '.';
            }
            else if (symbol == '-') {
                rgb_led_write(255, 0, 255);
                buzzer_play_tone(660, 400);
                vTaskDelay(pdMS_TO_TICKS(400));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '-';
            }
            // --- Handle SPACE ---
            else if (symbol == ' ') {
                rgb_led_write(0, 255, 255);
                vTaskDelay(pdMS_TO_TICKS(200));
                morseSymbolBuffer[morseSymbolPos] = '\0';
                char letter = find_letter_from_morse_code(morseSymbolBuffer);
                if (textMessagePos < 20) {
                    textMessageBuffer[textMessagePos++] = letter;
                }
                morseSymbolPos = 0;
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            else if (symbol == '\n') {
                // Translate the FINAL letter (if any) from the last sequence
                morseSymbolBuffer[morseSymbolPos] = '\0';
                if (morseSymbolPos > 0) { // Only translate if there's a buffered Morse code
                    char letter = find_letter_from_morse_code(morseSymbolBuffer);
                    if (textMessagePos < 20) {
                        textMessageBuffer[textMessagePos++] = letter;
                    }
                }
                textMessageBuffer[textMessagePos] = '\0'; // Null-terminate the message
               
                // --- MERGE START: Protect display with I2C Mutex ---
                // Wait for mutex to update display
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    clear_display();
                    // Redraw the persistent UI
                    write_text_xy(0, 0, "Morse App Ready");
                    write_text_xy(0, 10, "SW1=Mode, SW2=Action"); // Updated instruction
                    write_text_xy(0, 30, "RX MSG:");
                    // Write the new message
                    if (textMessagePos > 0) {
                        write_text_xy(0, 40, textMessageBuffer);
                    }
                    xSemaphoreGive(i2cMutex);
                }
                // --- MERGE END ---


                // Reset for next message
                textMessagePos = 0;
                morseSymbolPos = 0;
                memset(textMessageBuffer, 0, sizeof(textMessageBuffer));
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            rgb_led_write(255, 255, 255); // White (or default) after playing symbol
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay for visual/auditory feedback
        }
    }
}


// --- MERGE START: New Buzzer Task from Group Code ---
static void vBuzzerTask(void *pvParameters) {
     (void)pvParameters;
     app_event_t evt;
     while (1) {
         if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
             if (evt == APP_EVENT_MSG_SENT) {
                 // Play 3-note melody when a message is sent (3 spaces)
                 play_message_sent_melody();
             }
         }
     }
}
// --- MERGE END ---


char find_letter_from_morse_code(char *morseCode) {
    if (strlen(morseCode) == 0) return ' '; // space was found
    for (int loop_i = 0; loop_i < 40; loop_i++){ // Renamed loop variable
        if (strcmp(morseCode, morseCodes[loop_i].morseCode) == 0){
            return (char)toupper(morseCodes[loop_i].letter);
        }
    }
    return '?'; // If not found
}



