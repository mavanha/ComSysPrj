#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h> // Included for abs() used in signal processing

// --- Pico SDK Headers ---
// These provide access to the specific hardware of the Raspberry Pi Pico
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/cyw43_arch.h" // Required for Wi-Fi chip control on Pico W

// --- FreeRTOS Headers ---
// We use a Real-Time Operating System to handle multiple tasks (Input, Output, Display) simultaneously
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

// --- Hardware Abstraction Layer ---
#include "tkjhat/sdk.h" // Custom library for the specific HAT hardware (IMU, Display, Buttons)

/* ==========================================
 * CONFIGURATION & MACROS
 * ========================================== */

// Button Configuration
#define BTN_MODE            BUTTON1 // SW1: Used to switch input modes (IMU <-> Mic) or send Message
#define BTN_ACTION          BUTTON2 // SW2: Used to input a Dot/Dash or a Space
#define BTN_LONG_PRESS_MS   800     // How long (ms) to hold a button to trigger a "Long Press"
#define BTN_DEBOUNCE_MS     150     // Time to ignore button noise after a press

// Microphone Tuning
#define MIC_AMPL_THRESHOLD  8000    // Audio volume threshold. Below this is silence, above is signal.
#define MIC_DOT_MAX_MS      250     // If sound is shorter than this = Dot (.). Longer = Dash (-).

// Task Timing
#define INPUT_TASK_PERIOD_MS 20     // How often the input task runs (50Hz). Good balance of responsiveness vs CPU usage.

// Queue Sizes
#define SYMBOL_QUEUE_LENGTH     64   // Holds raw symbols: '.', '-', ' '
#define EVENT_QUEUE_LENGTH      8    // Holds system events like "Message Sent"

// --- Enums for State Machines ---
// Tracks which input method we are currently using
typedef enum {
    INPUT_MODE_IMU = 0, // Tilting the board
    INPUT_MODE_MIC = 1  // Making noise
} input_mode_t;

// Tracks the internal state of the microphone logic
typedef enum {
    MIC_STATE_IDLE = 0, // Waiting for sound
    MIC_STATE_ACTIVE    // Currently recording a sound
} mic_state_t;

// Events meant for the Buzzer/System task
typedef enum {
    APP_EVENT_MSG_SENT = 0
} app_event_t;

/* ==========================================
 * GLOBAL VARIABLES
 * ========================================== */
static input_mode_t g_inputMode = INPUT_MODE_IMU; 
static volatile int g_mic_samples_ready = 0;     // Flag set by interrupt when mic data is available
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE]; // Buffer to store raw audio data

// --- FreeRTOS Handles ---
static QueueHandle_t xSymbolQueue = NULL; // Internal pipe: Input Task -> Serial Task
static QueueHandle_t xEventQueue  = NULL; // Internal pipe: Tasks -> Buzzer/Event handler

// --- Microphone Interrupt Callback ---
// This function runs automatically when the hardware fills the audio buffer.
// We keep it short because it blocks other code.
static void on_pdm_samples_ready(void) {
    int n = get_microphone_samples(g_mic_buffer, MEMS_BUFFER_SIZE);
    if (n > 0) {
        g_mic_samples_ready = n; // Signal the main loop that data is ready to process
    }
}

// Function prototypes
static void input_task(void *pvParameters);
static void vBuzzerTask(void *pvParameters);

// Priorities: IDLE + 2 ensures these tasks run over background cleanup jobs
#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

// Queue settings
#define SERIAL_TX_QUEUE_LENGTH 20 
#define PLAYBACK_QUEUE_LENGTH 40
#define SERIAL_RX_BUFFER_SIZE 128

// --- IMU Thresholds ---
// We define "Tilt" logic. 1.0f is full gravity (90 degrees). 
// 0.9f requires a significant tilt to prevent accidental inputs.
#define IMU_TILT_THRESHOLD      0.9f
#define IMU_NEUTRAL_THRESHOLD   0.5f
#define IMU_JERK_THRESHOLD      0.4f
#define IMU_DEBOUNCE_MS         300

// IMU State Machine
volatile enum AppState {
  STATE_IDLE,     // Waiting for user input
  STATE_ARMED,    // Ready to read tilt
  STATE_COOLDOWN  // Waiting for sensor to return to flat position
} g_appState = STATE_IDLE;

// --- FreeRTOS Queue Handles ---
QueueHandle_t xSerialTxQueue_actual; // Sends data to USB UART
QueueHandle_t xPlaybackQueue;        // Sends data to OLED and LED/Buzzer
QueueHandle_t i2cMutex;              // PROTECTS HARDWARE: Prevents IMU and OLED from using the wires at the same time

// Morse Code Lookup Table
struct MorseAlphabet {
  char morseCode[7]; // e.g., ".-"
  char letter;       // e.g., 'A'
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

/* ==========================================
 * FUNCTION PROTOTYPES
 * ========================================== */
static void serial_tx_task(void *pvParameters);
static void serial_rx_task(void *pvParameters);
static void playback_task(void *pvParameters);
char find_letter_from_morse_code(char *morseCode);
void process_received_line(char *line);

// Helper to push a character into the internal queue safely
static void send_symbol(char c) {
    xQueueSend(xSymbolQueue, &c, 0); 
}

static void send_space(void) {
    char space = ' ';
    send_symbol(space);
}

// Melody played when message is successfully sent
static void play_message_sent_melody(void) {
    buzzer_play_tone(880, 150);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(660, 150);
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(440, 300);
}

/* ==========================================
 * MAIN FUNCTION
 * ========================================== */
int main() {
    // 1. Initialize Standard I/O (USB Serial)
    stdio_init_all();

    // FIX: Wait for USB connection so we don't miss initial print statements.
    // Includes a timeout so the board works even if not plugged into a PC.
    int i = 0;
    while (!stdio_usb_connected() && i < 30) { 
        sleep_ms(100);
        i++;
    }
    
    printf("__JTKJ Morse Communicator - Starting...__\n");

    // 2. Initialize Wi-Fi (CYW43)
    // Even if we don't use internet features, initializing this is often required for the board's LED
    // or power management on the Pico W.
    printf("Init the cyw43\n");
    if (cyw43_arch_init()) {
        printf("WiFi init failed! (Continuing without Wi-Fi)\n");
    } else {
        printf("Connecting to Wi-Fi\n");
        cyw43_arch_enable_sta_mode();
        
        // Attempt connection to open network 'panoulu'
        if(cyw43_arch_wifi_connect_timeout_ms("panoulu", NULL, CYW43_AUTH_OPEN, 30 * 1000)) {
            printf("Failed to connect to Wi-Fi (Running Offline)\n");
        } else {
            printf("Connected to hotspot\n");
        }
    }

    // 3. Initialize HAT Hardware
    init_hat_sdk(); 
    init_led();   
    init_rgb_led();
    init_buzzer();
    init_display();

    // Initialize IMU (Accelerometer/Gyro)
    if (init_ICM42670() == 0) {
        printf("__IMU INIT OK__\n");
        ICM42670_start_with_default_values();
    } else {
        printf("__IMU INIT FAILED__\n");
    }

    // Initialize Microphone
    if (init_pdm_microphone() == 0) {
        pdm_microphone_set_callback(on_pdm_samples_ready);
        init_microphone_sampling();
        printf("__MIC INIT OK__\n");
    } else {
        printf("__MIC INIT FAILED__\n");
    }

    // Initial Display Setup
    clear_display();
    write_text_xy(0, 0, "Morse App Ready");
    write_text_xy(0, 10, "SW1=Mode, SW2=Action"); 
    write_text_xy(0, 30, "RX MSG:");

    // 4. Create FreeRTOS Queues
    // Queues allow tasks to talk to each other safely.
    xSerialTxQueue_actual = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(char)); 
    xPlaybackQueue = xQueueCreate(PLAYBACK_QUEUE_LENGTH, sizeof(char));
    xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char)); 
    xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t)); 
    
    // Create Mutex: A "Lock" for the I2C bus (Display and IMU are on same bus)
    i2cMutex = xSemaphoreCreateMutex();
    
    // Error check
    if (xSerialTxQueue_actual == NULL || xPlaybackQueue == NULL || i2cMutex == NULL || xSymbolQueue == NULL || xEventQueue == NULL) {
        printf("__CRITICAL ERROR: Could not create queues or mutex__\n");
        while (1) { blink_led(1); sleep_ms(100); }
    }

    // 5. Create Tasks
    // We create separate tasks for Logic, Serial comms, and Output.
    // Note: Stack sizes increased (2048/4096) to prevent stack overflow crashes.
    xTaskCreate(input_task, "InputTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_tx_task, "TxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_rx_task, "RxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(playback_task, "PlaybackTask", 4096, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(vBuzzerTask, "BuzzerTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);

    printf("__Initialization complete. Starting scheduler.__\n");

    // 6. Start the OS
    vTaskStartScheduler();

    // Code should never reach here unless OS fails
    while (1); 
    return 0;
}

/* ==========================================
 * TASK: INPUT HANDLING (Buttons, IMU, Mic)
 * ========================================== */
static void input_task(void *pvParameters) {
    (void)pvParameters;

    // IMU variables
    float ax, ay, az, gx, gy, gz, t;
    float prev_ax = 0, prev_ay = 0, prev_az = 0;
    bool read_ok_imu = false; 

    // Button Logic Variables
    bool       sw1_prev = false;
    bool       sw2_prev = false;
    TickType_t sw1_press_tick = 0; // When was SW1 pressed?
    TickType_t sw2_press_tick = 0; // When was SW2 pressed?
    TickType_t sw1_last_change = 0; // For debouncing
    TickType_t sw2_last_change = 0; // For debouncing

    // Mic Logic Variables
    mic_state_t micState = MIC_STATE_IDLE;
    TickType_t  micStartTick = 0;

    init_sw1(); 
    init_sw2();

    g_inputMode = INPUT_MODE_IMU; // Default to IMU mode
    printf("__IMU MODE__\n");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Read raw button states
        bool sw1_now = gpio_get(BTN_MODE) ? true : false;
        bool sw2_now = gpio_get(BTN_ACTION) ? true : false;

        // -------- BUTTON SW1 LOGIC (Mode Switch / Send Msg) --------
        // Check for change AND ensure enough time passed (Debounce)
        if (sw1_now != sw1_prev && (now - sw1_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw1_last_change = now;
            
            if (sw1_now) { // Button Just Pressed
                buzzer_play_tone(1000, 80); 
                sw1_press_tick = now; // Mark start time
            } else { // Button Just Released
                TickType_t dt = now - sw1_press_tick;
                uint32_t ms   = dt * portTICK_PERIOD_MS;
                
                if (ms >= BTN_LONG_PRESS_MS) {
                    // ACTION: Send Message (trigger by sending 3 spaces)
                    printf("__MSG SEND VIA 3 SPACES__\n");
                    send_space(); send_space(); send_space();
                    
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0); 
                } else {
                    // ACTION: Toggle Input Mode (IMU <-> Mic)
                    if (g_inputMode == INPUT_MODE_IMU) {
                        g_inputMode = INPUT_MODE_MIC;
                        micState = MIC_STATE_IDLE; 
                        printf("__MIC MODE__\n");
                    } else {
                        g_inputMode = INPUT_MODE_IMU;
                        g_appState = STATE_IDLE; 
                        set_led_status(false); 
                        printf("__IMU MODE__\n");
                    }
                }
            }
        }
        sw1_prev = sw1_now;

        // -------- BUTTON SW2 LOGIC (Input Symbol / Space) --------
        if (sw2_now != sw2_prev && (now - sw2_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw2_last_change = now;
            
            if (sw2_now) { 
                buzzer_play_tone(700, 80); 
                sw2_press_tick = now;
            } else { 
                TickType_t dt = now - sw2_press_tick;
                uint32_t ms   = dt * portTICK_PERIOD_MS;
                
                if (ms >= BTN_LONG_PRESS_MS) {
                    // ACTION: Long press = Space (Word break)
                    send_space();
                    printf("__SPACE__\n");
                } else {
                    // ACTION: Short press depends on Mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        // IMU Logic: Read sensor to determine Dot or Dash
                        if (g_appState == STATE_IDLE) {
                            g_appState = STATE_ARMED; // Arm the sensor
                            set_led_status(true);
                            printf("__IMU ARMED__\n");
                        } else if (g_appState == STATE_ARMED) {
                            // Read IMU Data protected by Mutex
                            read_ok_imu = false;
                            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                                    read_ok_imu = true;
                                }
                                xSemaphoreGive(i2cMutex); // Always give back mutex!
                            }

                            if (read_ok_imu) {
                                char symbol = '?';
                                // Check orientation:
                                if (az > IMU_TILT_THRESHOLD) symbol = '.'; // Board Tilted Forward
                                else if (ay < -IMU_TILT_THRESHOLD) symbol = '-'; // Board Tilted Left

                                if (symbol != '?') {
                                    send_symbol(symbol);
                                    printf("__IMU SYMBOL '%c'__\n", symbol);
                                    set_led_status(false);
                                    g_appState = STATE_COOLDOWN; 
                                }
                            }
                        }
                    } else {
                        // In Mic mode, SW2 short press does nothing (voice controls input)
                        printf("__SW2 SHORT IN MIC MODE - no direct action__\n");
                    }
                }
            }
        }
        sw2_prev = sw2_now;

        // -------- IMU STATE MACHINE (Cooldown) --------
        // Ensures user returns board to flat position before next input
        if (g_inputMode == INPUT_MODE_IMU && g_appState == STATE_COOLDOWN) {
             read_ok_imu = false;
             if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                     read_ok_imu = true;
                }
                xSemaphoreGive(i2cMutex);
             }

             if (read_ok_imu) {
                 // Check if board is roughly flat (neutral)
                 if (fabs(az) < IMU_NEUTRAL_THRESHOLD && fabs(ay) < IMU_NEUTRAL_THRESHOLD) {
                     g_appState = STATE_IDLE; 
                     printf("__IMU COOLED DOWN (IDLE)__\n");
                 }
             }
        }

        // -------- MICROPHONE PROCESSING --------
        if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
            int sample_count = g_mic_samples_ready;
            g_mic_samples_ready = 0; // Clear flag

            // Calculate Volume (Average Absolute Amplitude)
            int64_t sumAbs = 0;
            for (int loop_i = 0; loop_i < sample_count; loop_i++) {
                sumAbs += abs(g_mic_buffer[loop_i]);
            }
            uint32_t avgAbs = (uint32_t)(sumAbs / sample_count);

            // Mic State Machine
            switch (micState) {
                case MIC_STATE_IDLE:
                    if (avgAbs > MIC_AMPL_THRESHOLD) {
                        micState = MIC_STATE_ACTIVE; // Sound started
                        micStartTick = now;
                        printf("__MIC ACTIVE__\n");
                    }
                    break;
                case MIC_STATE_ACTIVE:
                    if (avgAbs <= MIC_AMPL_THRESHOLD) { // Sound stopped
                        // Calculate duration to decide Dot vs Dash
                        TickType_t durationTicks = now - micStartTick;
                        uint32_t durationMs = durationTicks * portTICK_PERIOD_MS;
                        char symbol = (durationMs <= MIC_DOT_MAX_MS) ? '.' : '-';
                        
                        printf("__MIC INPUT: %c (%lu ms)__\n", symbol, (unsigned long)durationMs);
                        send_symbol(symbol);
                        micState = MIC_STATE_IDLE;
                    }
                    break;
                default: micState = MIC_STATE_IDLE; break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(INPUT_TASK_PERIOD_MS)); // Sleep to yield CPU
    }
}

/* ==========================================
 * TASK: SERIAL TRANSMISSION (The "Bridge")
 * ========================================== */
static void serial_tx_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    int spaceCount = 0;

    // This task acts as a router. It takes internal symbols and sends them out via USB.
    while (1) {
        // Block indefinitely until a symbol arrives in the queue
        if (xQueueReceive(xSymbolQueue, &symbol, portMAX_DELAY) == pdPASS) { 
            if (symbol == ' ') {
                spaceCount++;
                putchar(' '); 
                fflush(stdout);
                // Logic: 3 consecutive spaces mean "End of Message"
                if (spaceCount == 3) {
                    putchar('\n'); // Send newline to serial
                    printf("\n__[Morse Send OK]__\n");
                    fflush(stdout);
                    
                    spaceCount = 0;
                    
                    // Tell the Playback/Display task that the message is done
                    char nl_char = '\n';
                    xQueueSend(xPlaybackQueue, &nl_char, 0); 
                    
                    // Tell the Buzzer task to play success sound
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0); 
                }
            } else {
                // If it's a dot or dash, reset space counter
                spaceCount = 0;
                putchar(symbol); 
                fflush(stdout);
                // Echo input to local display immediately
                xQueueSend(xPlaybackQueue, &symbol, 0);
            }
        }
    }
}

/* ==========================================
 * TASK: SERIAL RECEPTION (From PC/USB)
 * ========================================== */
static void serial_rx_task(void *pvParameters) {
    (void)pvParameters;
    char lineBuffer[SERIAL_RX_BUFFER_SIZE];
    int linePos = 0;

    while (1) {
        // Non-blocking read with 0 timeout
        int c = getchar_timeout_us(0);
        
        if (c != PICO_ERROR_TIMEOUT) {
            // If End of Line detected, process the full string
            if (c == '\n' || c == '\r') {
                if (linePos > 0) {
                    lineBuffer[linePos] = '\0';
                    process_received_line(lineBuffer);
                    linePos = 0;
                }
            } else if (linePos < (SERIAL_RX_BUFFER_SIZE - 1)) {
                lineBuffer[linePos++] = (char)c;
            } else {
                linePos = 0; // Buffer overflow protection: reset
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Parses commands (.clear, .exit) and incoming Morse strings
void process_received_line(char *line) {
    // Check for commands
    if (strncmp(line, ".clear", 6) == 0) {
        printf("\033[2J\033[H"); // ANSI Escape code to clear terminal
        fflush(stdout);
        return;
    }
    if (strncmp(line, ".boot", 5) == 0) {
        reset_usb_boot(0, 0); // Special Pico command to enter bootloader
        return;
    }

    // Parse Morse characters (ignoring debug tags like __DEBUG__)
    bool in_debug_block = false;
    for (int i = 0; line[i] != '\0'; i++) { 
        if (line[i] == '_' && line[i+1] == '_') {
            in_debug_block = !in_debug_block; // Toggle debug mode
            i++; continue;
        }
        if (!in_debug_block) {
            char symbol = line[i];
            // Forward valid morse to the playback task
            if (symbol == '.' || symbol == '-' || symbol == ' ') {
                xQueueSend(xPlaybackQueue, &symbol, portMAX_DELAY);
            }
        }
    }
    // Trigger display update
    char nl = '\n';
    xQueueSend(xPlaybackQueue, &nl, portMAX_DELAY);
}

/* ==========================================
 * TASK: PLAYBACK & DISPLAY
 * ========================================== */
static void playback_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    char morseSymbolBuffer[10] = {0}; // Holds current letter (e.g., "...")
    int morseSymbolPos = 0;
    char textMessageBuffer[22] = {0}; // Holds translated text
    int textMessagePos = 0;

    while (1) {
        // Wait for data from Input or RX tasks
        if (xQueueReceive(xPlaybackQueue, &symbol, portMAX_DELAY) == pdPASS) {
            
            // --- 1. Physical Feedback (LEDs/Buzzer) ---
            if (symbol == '.') {
                rgb_led_write(255, 255, 0); // Yellow
                buzzer_play_tone(880, 150);
                vTaskDelay(pdMS_TO_TICKS(150));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '.';
            }
            else if (symbol == '-') {
                rgb_led_write(255, 0, 255); // Magenta
                buzzer_play_tone(660, 400);
                vTaskDelay(pdMS_TO_TICKS(400));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '-';
            }
            // --- 2. Translation Logic (Space or Newline) ---
            else if (symbol == ' ') {
                rgb_led_write(0, 255, 255); // Cyan
                vTaskDelay(pdMS_TO_TICKS(200));
                
                // Translate accumulated Morse ("...") to Char ('S')
                morseSymbolBuffer[morseSymbolPos] = '\0';
                char letter = find_letter_from_morse_code(morseSymbolBuffer);
                if (textMessagePos < 20) textMessageBuffer[textMessagePos++] = letter;
                
                // Reset symbol buffer
                morseSymbolPos = 0;
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            else if (symbol == '\n') {
                // End of message: Translate any remaining symbols
                morseSymbolBuffer[morseSymbolPos] = '\0';
                if (morseSymbolPos > 0) {
                    char letter = find_letter_from_morse_code(morseSymbolBuffer);
                    if (textMessagePos < 20) textMessageBuffer[textMessagePos++] = letter;
                }
                textMessageBuffer[textMessagePos] = '\0'; 
               
                // --- 3. Update OLED Display ---
                // Critical: We must take the Mutex before using OLED because it shares I2C with IMU
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    clear_display();
                    // Draw UI
                    write_text_xy(0, 0, "Morse App Ready");
                    write_text_xy(0, 10, "SW1=Mode, SW2=Action");
                    write_text_xy(0, 30, "RX MSG:");
                    // Draw Translated Text
                    if (textMessagePos > 0) {
                        write_text_xy(0, 40, textMessageBuffer);
                    }
                    xSemaphoreGive(i2cMutex); // Release Mutex immediately
                }

                // Reset text buffer for next message
                textMessagePos = 0;
                morseSymbolPos = 0;
                memset(textMessageBuffer, 0, sizeof(textMessageBuffer));
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            
            rgb_led_write(255, 255, 255); // Reset LED to white
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ==========================================
 * TASK: BUZZER EVENTS
 * ========================================== */
static void vBuzzerTask(void *pvParameters) {
     (void)pvParameters;
     app_event_t evt;
     while (1) {
         // Wait for system events (like Message Sent)
         if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
             if (evt == APP_EVENT_MSG_SENT) {
                 play_message_sent_melody();
             }
         }
     }
}

// Helper: Translates ".-" to 'A'
char find_letter_from_morse_code(char *morseCode) {
    if (strlen(morseCode) == 0) return ' '; 
    for (int i = 0; i < 40; i++){ 
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0){
            return (char)toupper(morseCodes[i].letter);
        }
    }
    return '?'; // Unknown pattern
}
