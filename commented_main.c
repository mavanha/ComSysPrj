// Group members' names: Hoa Ly, Jonathan Tran, Kien Quoc Vu

//
// JTKJ Morse Communicator for Raspberry Pi Pico with TKJ HAT
//
// This application implements a Morse code communication device using a Raspberry Pi Pico
// and a custom TKJ HAT. It allows users to input Morse code through two different methods:
// 1.  An Inertial Measurement Unit (IMU) by tilting the device.
// 2.  A PDM microphone by making sounds of different durations.
//
// The application uses the FreeRTOS real-time operating system to manage multiple concurrent
// tasks, ensuring responsive handling of user input, serial communication, and feedback
// mechanisms like an OLED display, buzzer, and RGB LED. It also includes Wi-Fi
// connectivity to demonstrate network integration.
//

// --- Standard C Libraries ---
#include <stdio.h>      // Standard Input/Output operations (e.g., printf)
#include <string.h>     // String manipulation functions (e.g., strcmp, strlen)
#include <math.h>       // Mathematical functions (e.g., fabs for absolute float value)
#include <ctype.h>      // Character handling functions (e.g., toupper)
#include <stdlib.h>     // General utilities, including abs() for integer absolute value

// --- Pico SDK Headers ---
#include "pico/bootrom.h"   // For programmatic rebooting into BOOTSEL mode
#include "pico/stdlib.h"    // Core Pico functions and definitions
#include "pico/stdio.h"     // Standard I/O support for Pico (e.g., over USB)
#include "pico/cyw43_arch.h"// Architecture-specific functions for the CYW43 Wi-Fi/BT chip

// --- FreeRTOS Headers ---
#include <FreeRTOS.h>   // Core FreeRTOS definitions
#include <queue.h>      // FreeRTOS queue API for inter-task communication
#include <task.h>       // FreeRTOS task management API
#include <semphr.h>     // FreeRTOS semaphore and mutex API

// --- Custom Hardware Abstraction Layer ---
#include "tkjhat/sdk.h" // TKJHAT SDK for interacting with the HAT's components (display, sensors, etc.)

// =================================================================================
// --- APPLICATION CONFIGURATION CONSTANTS ---
// =================================================================================

// --- Button and Input Mode Configuration ---
#define BTN_MODE            BUTTON1 // SW1 on the HAT: Toggles input mode (IMU/MIC), long press sends message
#define BTN_ACTION          BUTTON2 // SW2 on the HAT: Confirms a symbol in IMU mode, long press sends a space
#define BTN_LONG_PRESS_MS   800     // Duration (in ms) to qualify a button press as a "long press"
#define BTN_DEBOUNCE_MS     150     // Time (in ms) to ignore further button changes to prevent bouncing
#define MIC_AMPL_THRESHOLD  8000    // The average absolute amplitude required for the microphone to detect sound
#define MIC_DOT_MAX_MS      250     // A sound duration less than or equal to this is a DOT; otherwise, it's a DASH
#define INPUT_TASK_PERIOD_MS 20     // The polling rate (in ms) for the main input task loop

// --- FreeRTOS Queue Configuration ---
#define SYMBOL_QUEUE_LENGTH     64   // Max number of Morse symbols ('.', '-', ' ') that can be buffered internally
#define EVENT_QUEUE_LENGTH      8    // Max number of application events (e.g., message sent) to buffer

// =================================================================================
// --- TYPE DEFINITIONS ---
// =================================================================================

// Defines the possible input methods for Morse code
typedef enum {
    INPUT_MODE_IMU = 0, // Input via the Inertial Measurement Unit (tilting)
    INPUT_MODE_MIC = 1  // Input via the PDM microphone (sound)
} input_mode_t;

// Defines the states for the microphone input logic
typedef enum {
    MIC_STATE_IDLE = 0,     // The microphone is waiting for a sound loud enough to cross the threshold
    MIC_STATE_ACTIVE        // The microphone has detected a sound and is measuring its duration
} mic_state_t;

// Defines internal application events for inter-task signaling
typedef enum {
    APP_EVENT_MSG_SENT = 0 // An event indicating that a message has been successfully sent (e.g., by 3 spaces)
} app_event_t;

// =================================================================================
// --- GLOBAL VARIABLES ---
// =================================================================================
static input_mode_t g_inputMode = INPUT_MODE_IMU; // Global variable to track the current input mode, starts with IMU
static volatile int g_mic_samples_ready = 0;      // A flag (set by an ISR) to indicate that new microphone samples are ready
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE]; // Buffer to store the raw audio samples from the microphone

// --- FreeRTOS Handles ---
// These are handles to the queues and semaphores created in main()
static QueueHandle_t xSymbolQueue = NULL; // Internal queue to pass Morse symbols from the input task to the TX/playback tasks
static QueueHandle_t xEventQueue  = NULL; // Queue for high-level application events, like signaling the buzzer task

// --- Microphone Interrupt Service Routine (ISR) Callback ---
// This function is called automatically by the PDM library when a buffer of samples is full.
static void on_pdm_samples_ready(void) {
    // Attempt to read the microphone samples into the global buffer
    int n = get_microphone_samples(g_mic_buffer, MEMS_BUFFER_SIZE);
    if (n > 0) {
        // If samples were read successfully, set the flag for the input task to process them
        g_mic_samples_ready = n;
    }
}

// --- Task Function Prototypes ---
// Forward declarations for the functions that will run as FreeRTOS tasks
static void input_task(void *pvParameters); // Handles all user input: buttons, IMU, and microphone
static void vBuzzerTask(void *pvParameters); // Plays sounds based on application events

// =================================================================================
// --- CONSTANTS AND DATA STRUCTURES ---
// =================================================================================

// --- Task Priority ---
#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2) // Sets a priority level for all application tasks

// --- Queue Lengths ---
#define SERIAL_TX_QUEUE_LENGTH 20 // This is legacy and not directly used for symbol buffering anymore
#define PLAYBACK_QUEUE_LENGTH 40  // Max number of symbols to buffer for local feedback (display, LED, buzzer)
#define SERIAL_RX_BUFFER_SIZE 128 // Max characters for the serial receive buffer

// --- IMU Thresholds ---
#define IMU_TILT_THRESHOLD      0.9f // g-force value (approx 0.9g) that must be exceeded to register a tilt
#define IMU_NEUTRAL_THRESHOLD   0.5f // g-force value below which the device is considered to be in a neutral (flat) position
#define IMU_DEBOUNCE_MS         300  // Debounce time for IMU actions to prevent multiple triggers from one movement

// --- IMU State Machine ---
// This state machine controls the logic for generating symbols from the IMU.
volatile enum AppState {
  STATE_IDLE,       // Waiting for the user to "arm" the IMU with a button press
  STATE_ARMED,      // IMU is armed. The next tilt will generate a symbol.
  STATE_COOLDOWN    // A symbol has just been generated. Waiting for the device to return to neutral.
} g_appState = STATE_IDLE;

// --- More FreeRTOS Handles ---
QueueHandle_t xSerialTxQueue_actual; // This name is legacy; the queue is not used for primary TX logic
QueueHandle_t xPlaybackQueue;       // Queue for symbols that need local feedback (display/sound)
QueueHandle_t i2cMutex;             // A mutex to prevent concurrent access to the I2C bus (used by IMU and display)

// --- Morse Code to Alphabet Translation Table ---
// This array maps Morse code strings to their corresponding letters and numbers.
struct MorseAlphabet {
  char morseCode[7]; // String for the Morse code (e.g., ".-")
  char letter;       // The corresponding character (e.g., 'a')
};

// The full Morse code alphabet used for translation
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


// =================================================================================
// --- FUNCTION PROTOTYPES ---
// =================================================================================
static void serial_tx_task(void *pvParameters); // Task to send Morse symbols over serial
static void serial_rx_task(void *pvParameters); // Task to receive and process data from serial
static void playback_task(void *pvParameters);  // Task to provide local feedback for Morse symbols
char find_letter_from_morse_code(char *morseCode); // Utility to translate Morse string to a character
void process_received_line(char *line);            // Utility to process a complete line received via serial


// --- Helper Functions ---
// A simple wrapper to send a character symbol to the internal symbol queue.
static void send_symbol(char c) {
    xQueueSend(xSymbolQueue, &c, 0); // Use 0 as the timeout (don't block)
}

// A helper to send a space symbol, used for separating letters and words.
static void send_space(void) {
    char space = ' ';
    send_symbol(space);
}

// Plays a simple 3-note melody on the buzzer to indicate a message was sent.
static void play_message_sent_melody(void) {
    buzzer_play_tone(880, 150); // A5 note
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(660, 150); // E5 note
    vTaskDelay(pdMS_TO_TICKS(50));
    buzzer_play_tone(440, 300); // A4 note
}

// =================================================================================
// --- MAIN FUNCTION ---
// =================================================================================
int main() {
    // 1. Initialize Standard I/O
    stdio_init_all();

    // Add a small delay to wait for the USB serial connection to be established.
    // This prevents the "White Screen of Death" where the program hangs if the
    // serial monitor is not open when the Pico starts.
    int i = 0;
    while (!stdio_usb_connected() && i < 30) { // Wait max 3 seconds
        sleep_ms(100);
        i++;
    }
    
    printf("__JTKJ Morse Communicator - Starting...__\n");

    // 2. Initialize Wi-Fi
    // This block initializes the CYW43 chip and attempts to connect to a Wi-Fi network.
    // It is done before starting the FreeRTOS scheduler.
    printf("Init the cyw43\n");
    if (cyw43_arch_init()) {
        printf("WiFi init failed! (Continuing without Wi-Fi)\n");
    } else {
        printf("Connecting to Wi-Fi\n");
        cyw43_arch_enable_sta_mode(); // Set the chip to station (client) mode
        printf("Set stable mode successfully -> connect to wifi\n");
        
        // Attempt to connect to the "panoulu" network, which is an open network.
        // This has a timeout of 30 seconds.
        if(cyw43_arch_wifi_connect_timeout_ms("panoulu", NULL, CYW43_AUTH_OPEN, 30 * 1000)) {
            printf("Failed to connect to Wi-Fi\n");
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

    // 4. Initialize On-board Sensors (IMU and Microphone)
    if (init_ICM42670() == 0) { // Initialize the IMU
        printf("__IMU INIT OK__\n");
        if (ICM42670_start_with_default_values() != 0) { // Start it with default settings
            printf("__IMU START DEFAULT FAILED__\n");
        }
    } else {
        printf("__IMU INIT FAILED__\n");
    }

    if (init_pdm_microphone() == 0) { // Initialize the microphone
        pdm_microphone_set_callback(on_pdm_samples_ready); // Register the ISR callback
        if (init_microphone_sampling() == 0) { // Start sampling
            printf("__MIC INIT OK__\n");
        } else {
            printf("__MIC START FAILED__\n");
        }
    } else {
        printf("__MIC INIT FAILED__\n");
    }

    // 5. Display a startup message on the OLED
    clear_display();
    write_text_xy(0, 0, "Morse App Ready");
    write_text_xy(0, 10, "SW1=Mode, SW2=Action"); 
    write_text_xy(0, 30, "RX MSG:");

    // 6. Create FreeRTOS Queues and Mutexes
    xSerialTxQueue_actual = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(char)); 
    xPlaybackQueue = xQueueCreate(PLAYBACK_QUEUE_LENGTH, sizeof(char));
    i2cMutex = xSemaphoreCreateMutex(); // To protect the I2C bus
    xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char)); // Internal symbol queue
    xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t)); // App event queue

    // Check if all queues and the mutex were created successfully. If not, halt.
    if (xSerialTxQueue_actual == NULL || xPlaybackQueue == NULL || i2cMutex == NULL || xSymbolQueue == NULL || xEventQueue == NULL) {
        printf("__CRITICAL ERROR: Could not create queues or mutex__\n");
        while (1) { blink_led(1); sleep_ms(100); } // Blink LED to indicate fatal error
    }

    // 7. Create FreeRTOS Tasks
    // Each task is a separate thread of execution. Stack sizes have been increased
    // to prevent stack overflow, which can be a common and hard-to-debug issue.
    // Tasks are created with MAIN_TASK_PRIORITY.
    xTaskCreate(input_task, "InputTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_tx_task, "TxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_rx_task, "RxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(playback_task, "PlaybackTask", 4096, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(vBuzzerTask, "BuzzerTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);

    printf("__Initialization complete. Starting scheduler.__\n");

    // 8. Start the FreeRTOS Scheduler
    // This function starts the real-time operating system and begins executing the created tasks.
    // Code execution should never proceed past this point.
    vTaskStartScheduler();

    while (1); // This line should be unreachable.
    return 0;
}

// =================================================================================
// --- INPUT TASK ---
// This is the most complex task. It handles all user inputs:
// - Polling the two buttons for short and long presses.
// - Reading the IMU sensor to detect tilts.
// - Processing microphone data to detect sounds.
// - Switching between IMU and Microphone input modes.
// - Sending generated Morse symbols to the internal `xSymbolQueue`.
// =================================================================================
static void input_task(void *pvParameters) {
    (void)pvParameters; // Unused parameter

    // --- Local variables for IMU state ---
    float ax, ay, az, gx, gy, gz, t; // To store sensor readings
    TickType_t last_action_time_imu = 0; // Timestamp of the last IMU action for debouncing
    bool read_ok_imu = false; // Flag to check if IMU read was successful

    // --- Local variables for Button state ---
    bool       sw1_prev = false, sw2_prev = false; // Previous state of the buttons
    TickType_t sw1_press_tick = 0, sw2_press_tick = 0; // Tick count when buttons were pressed
    TickType_t sw1_last_change = 0, sw2_last_change = 0; // Tick count of the last state change for debouncing

    // --- Local variables for Microphone state ---
    mic_state_t micState = MIC_STATE_IDLE; // Current state of the microphone logic
    TickType_t  micStartTick = 0; // Tick count when a sound was first detected

    // Initialize the GPIO for the buttons
    init_sw1();
    init_sw2();

    g_inputMode = INPUT_MODE_IMU; // Start in IMU mode by default
    printf("__IMU MODE__\n");

    // --- Main Task Loop ---
    while (1) {
        TickType_t now = xTaskGetTickCount(); // Get the current time in system ticks

        // Read the current state of the buttons
        bool sw1_now = gpio_get(BTN_MODE) ? true : false;
        bool sw2_now = gpio_get(BTN_ACTION) ? true : false;

        // --- SW1 (Mode Button) Logic with Debouncing ---
        if (sw1_now != sw1_prev && (now - sw1_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw1_last_change = now; // Update the last change time
            if (sw1_now) { // Button was just PRESSED
                buzzer_play_tone(1000, 80); // Short beep for feedback
                sw1_press_tick = now; // Record the press time
            } else { // Button was just RELEASED
                TickType_t press_duration = now - sw1_press_tick; // Calculate duration
                
                if (press_duration >= pdMS_TO_TICKS(BTN_LONG_PRESS_MS)) {
                    // LONG PRESS on SW1: Send message (3 spaces)
                    printf("__MSG SEND VIA 3 SPACES__\n");
                    send_space(); send_space(); send_space();
                    // Send an event to the buzzer task to play a confirmation melody
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0);
                } else {
                    // SHORT PRESS on SW1: Toggle input mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        g_inputMode = INPUT_MODE_MIC;
                        printf("__MIC MODE__\n");
                        micState = MIC_STATE_IDLE; // Reset mic state on mode switch
                    } else {
                        g_inputMode = INPUT_MODE_IMU;
                        printf("__IMU MODE__\n");
                        g_appState = STATE_IDLE; // Reset IMU state on mode switch
                        set_led_status(false);   // Turn off LED if it was on
                    }
                }
            }
        }
        sw1_prev = sw1_now; // Update previous state for next loop

        // --- SW2 (Action Button) Logic with Debouncing ---
        if (sw2_now != sw2_prev && (now - sw2_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
            sw2_last_change = now;
            if (sw2_now) { // Button was just PRESSED
                buzzer_play_tone(700, 80); // Different beep for SW2
                sw2_press_tick = now;
            } else { // Button was just RELEASED
                TickType_t press_duration = now - sw2_press_tick;

                if (press_duration >= pdMS_TO_TICKS(BTN_LONG_PRESS_MS)) {
                    // LONG PRESS on SW2: Send a space (for separating letters/words)
                    send_space();
                    printf("__SPACE__\n");
                } else {
                    // SHORT PRESS on SW2: Action depends on the current mode
                    if (g_inputMode == INPUT_MODE_IMU) {
                        if (g_appState == STATE_IDLE) {
                            // If idle, arm the IMU to prepare for a tilt gesture
                            g_appState = STATE_ARMED;
                            set_led_status(true); // Turn on LED to indicate armed state
                            printf("__IMU ARMED__\n");
                        } else if (g_appState == STATE_ARMED) {
                            // If already armed, generate a symbol based on current orientation
                             read_ok_imu = false;
                             if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                 if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                                     read_ok_imu = true;
                                 }
                                 xSemaphoreGive(i2cMutex);
                             }

                             if (read_ok_imu) {
                                 char symbol = '?'; // Default to unknown
                                 // Check for significant tilt along Z (forward/back) or Y (left/right) axes
                                 if (az > IMU_TILT_THRESHOLD)      symbol = '.'; // Tilt forward -> DOT
                                 else if (ay < -IMU_TILT_THRESHOLD) symbol = '-'; // Tilt left -> DASH
                                 
                                 if (symbol != '?') {
                                     send_symbol(symbol);
                                     printf("__IMU SYMBOL '%c'__\n", symbol);
                                     set_led_status(false); // Turn off LED
                                     g_appState = STATE_COOLDOWN; // Enter cooldown state
                                     last_action_time_imu = now;
                                 } else {
                                     printf("__IMU: No clear orientation for symbol.__\n");
                                 }
                             }
                        }
                    } else { // g_inputMode == INPUT_MODE_MIC
                        printf("__SW2 SHORT IN MIC MODE - no direct action__\n");
                        // In microphone mode, short press on SW2 currently does nothing.
                    }
                }
            }
        }
        sw2_prev = sw2_now;

        // --- IMU State Machine Processing ---
        // This part runs continuously when in IMU mode.
        if (g_inputMode == INPUT_MODE_IMU) {
             if (g_appState == STATE_COOLDOWN) {
                 // After a symbol is generated, wait for the device to return to a neutral (flat) position.
                 read_ok_imu = false;
                 if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                         read_ok_imu = true;
                    }
                    xSemaphoreGive(i2cMutex);
                 }
                 if (read_ok_imu) {
                     if (fabs(az) < IMU_NEUTRAL_THRESHOLD && fabs(ay) < IMU_NEUTRAL_THRESHOLD) {
                         g_appState = STATE_IDLE; // Ready for the next symbol
                         printf("__IMU COOLED DOWN (IDLE)__\n");
                     }
                 }
             }
        }

        // --- Microphone Input Processing ---
        // This part runs when in MIC mode and the ISR has signaled that new samples are available.
        if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
            int sample_count = g_mic_samples_ready;
            g_mic_samples_ready = 0; // Reset the flag immediately

            // Calculate the average absolute amplitude of the audio samples
            int64_t sumAbs = 0;
            for (int i = 0; i < sample_count; i++) {
                sumAbs += abs(g_mic_buffer[i]);
            }
            uint32_t avgAbs = (uint32_t)(sumAbs / sample_count);

            // Microphone State Machine
            switch (micState) {
                case MIC_STATE_IDLE:
                    // If idle and a loud sound is detected, switch to active state
                    if (avgAbs > MIC_AMPL_THRESHOLD) {
                        micState = MIC_STATE_ACTIVE;
                        micStartTick = now; // Record the start time of the sound
                        printf("__MIC ACTIVE (Threshold hit)__\n");
                    }
                    break;
                case MIC_STATE_ACTIVE:
                    // If active and the sound level drops, the sound has ended
                    if (avgAbs <= MIC_AMPL_THRESHOLD) {
                        uint32_t durationMs = (now - micStartTick) * portTICK_PERIOD_MS;
                        char symbol = (durationMs <= MIC_DOT_MAX_MS) ? '.' : '-'; // Determine if it was a dot or dash
                        
                        if(symbol == '.') printf("__MIC DOT (%lu ms)__\n", (unsigned long)durationMs);
                        else printf("__MIC DASH (%lu ms)__\n", (unsigned long)durationMs);

                        send_symbol(symbol);
                        micState = MIC_STATE_IDLE; // Return to idle state
                    }
                    break;
            }
        }

        // Delay to control the loop rate of this task
        vTaskDelay(pdMS_TO_TICKS(INPUT_TASK_PERIOD_MS));
    }
}

// =================================================================================
// --- SERIAL TX TASK ---
// This task waits for Morse symbols on the `xSymbolQueue`, sends them to the
// USB serial port, and handles the logic for message termination (3 spaces).
// =================================================================================
static void serial_tx_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    int spaceCount = 0; // Counter for consecutive spaces

    while (1) {
        // Block indefinitely until a symbol is received from the input task
        if (xQueueReceive(xSymbolQueue, &symbol, portMAX_DELAY) == pdPASS) {
            if (symbol == ' ') {
                spaceCount++;
                putchar(' '); // Always output the space
                if (spaceCount == 3) {
                    // Three consecutive spaces signifies the end of a message
                    putchar('\n'); // Send a newline character
                    printf("\n__[Morse Send OK]__\n"); // Print confirmation
                    spaceCount = 0; // Reset counter
                    
                    // Send a newline to the playback task to clear its display buffer
                    char nl_char = '\n';
                    xQueueSend(xPlaybackQueue, &nl_char, 0);
                    // Trigger the buzzer melody for message sent
                    app_event_t evt = APP_EVENT_MSG_SENT;
                    xQueueSend(xEventQueue, &evt, 0);
                }
            } else {
                // Any non-space symbol resets the space counter
                spaceCount = 0;
                putchar(symbol); // Output the '.' or '-'
            }
            fflush(stdout); // Ensure the character is sent immediately
            // Also send the symbol to the playback queue for local feedback
            xQueueSend(xPlaybackQueue, &symbol, 0);
        }
    }
}

// =================================================================================
// --- SERIAL RX TASK ---
// This task continuously polls the USB serial for incoming characters, assembles
// them into lines, and passes complete lines to `process_received_line`.
// =================================================================================
static void serial_rx_task(void *pvParameters) {
    (void)pvParameters;
    char lineBuffer[SERIAL_RX_BUFFER_SIZE];
    int linePos = 0;
    while (1) {
        int c = getchar_timeout_us(0); // Non-blocking read
        if (c != PICO_ERROR_TIMEOUT) {
            // If a newline is received, the line is complete
            if (c == '\n' || c == '\r') {
                if (linePos > 0) {
                    lineBuffer[linePos] = '\0'; // Null-terminate the string
                    process_received_line(lineBuffer);
                    linePos = 0; // Reset for the next line
                }
            } else if (linePos < (SERIAL_RX_BUFFER_SIZE - 1)) {
                // Add the character to the buffer
                lineBuffer[linePos++] = (char)c;
            } else {
                // Buffer overflow, discard the line
                linePos = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield the CPU
    }
}

// =================================================================================
// --- PROCESS RECEIVED LINE ---
// This utility function processes a line of text received from the serial port.
// It handles special commands and extracts Morse code for playback.
// =================================================================================
void process_received_line(char *line) {
    // --- Handle special commands ---
    if (strncmp(line, ".clear", 6) == 0) { // Clear the serial console
        printf("\033[2J\033[H");
        fflush(stdout);
        return;
    }
    if (strncmp(line, ".boot", 5) == 0) { // Reboot into BOOTSEL mode for programming
        printf("__Rebooting to BOOTSEL mode...__\n");
        sleep_ms(100);
        reset_usb_boot(0, 0);
        return;
    }
    if (strncmp(line, ".exit", 5) == 0) { // Halt the processor
        printf("\n__--- .exit command received. Halting. ---__\n");
        fflush(stdout);
        while (1) { blink_led(1); sleep_ms(200); }
    }

    // --- Parse the line for Morse symbols, skipping debug messages in "__" ---
    bool in_debug_block = false;
    for (int i = 0; line[i] != '\0'; i++) {
        if (line[i] == '_' && line[i+1] == '_') {
            in_debug_block = !in_debug_block; // Toggle flag
            i++;
            continue;
        }
        // If not in a debug block, send valid Morse symbols to the playback queue
        if (!in_debug_block) {
            char symbol = line[i];
            if (symbol == '.' || symbol == '-' || symbol == ' ') {
                xQueueSend(xPlaybackQueue, &symbol, portMAX_DELAY);
            }
        }
    }
    // Send a newline to the playback task to finalize the received message
    char nl = '\n';
    xQueueSend(xPlaybackQueue, &nl, portMAX_DELAY);
}

/*
I got the inspiration for this playback_task() from the “Morse-Code-Reader” FreeRTOS project (STM32 NUCLEO).
URL: https://github.com/carter-glynn/Morse-Code-Reader
This playback_task() follows a similar queue-driven symbol decode pattern.
*/
// =================================================================================
// --- PLAYBACK TASK ---
// This task provides local feedback for Morse symbols (both sent and received).
// It controls the RGB LED, the buzzer, and updates the OLED display.
// =================================================================================
static void playback_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    char morseSymbolBuffer[10] = {0}; // Buffer to build a Morse code string for one letter
    int morseSymbolPos = 0;
    char textMessageBuffer[22] = {0}; // Buffer to build the translated text message
    int textMessagePos = 0;

    while (1) {
        // Block until a symbol is received on the playback queue
        if (xQueueReceive(xPlaybackQueue, &symbol, portMAX_DELAY) == pdPASS) {
            // --- Provide feedback based on the symbol ---
            if (symbol == '.') {
                rgb_led_write(255, 255, 0); // Yellow for dot
                buzzer_play_tone(880, 150); // High pitch tone
                vTaskDelay(pdMS_TO_TICKS(150));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '.';
            }
            else if (symbol == '-') {
                rgb_led_write(255, 0, 255); // Magenta for dash
                buzzer_play_tone(660, 400); // Lower pitch tone
                vTaskDelay(pdMS_TO_TICKS(400));
                if (morseSymbolPos < 9) morseSymbolBuffer[morseSymbolPos++] = '-';
            }
            // --- Handle SPACE (end of a letter) ---
            else if (symbol == ' ') {
                rgb_led_write(0, 255, 255); // Cyan for space
                vTaskDelay(pdMS_TO_TICKS(200));
                morseSymbolBuffer[morseSymbolPos] = '\0'; // Null-terminate Morse string
                char letter = find_letter_from_morse_code(morseSymbolBuffer); // Translate
                if (textMessagePos < 20) {
                    textMessageBuffer[textMessagePos++] = letter; // Add to message
                }
                morseSymbolPos = 0; // Reset for next letter
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            // --- Handle NEWLINE (end of a message) ---
            else if (symbol == '\n') {
                // Translate the final letter of the message
                morseSymbolBuffer[morseSymbolPos] = '\0';
                if (morseSymbolPos > 0) {
                    char letter = find_letter_from_morse_code(morseSymbolBuffer);
                    if (textMessagePos < 20) {
                        textMessageBuffer[textMessagePos++] = letter;
                    }
                }
                textMessageBuffer[textMessagePos] = '\0'; // Null-terminate the final message
               
                // Update the OLED display with the received message
                // Use the I2C mutex to prevent conflicts with the IMU task
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    clear_display();
                    // Redraw the static UI elements
                    write_text_xy(0, 0, "Morse App Ready");
                    write_text_xy(0, 10, "SW1=Mode, SW2=Action");
                    write_text_xy(0, 30, "RX MSG:");
                    // Write the new message
                    if (textMessagePos > 0) {
                        write_text_xy(0, 40, textMessageBuffer);
                    }
                    xSemaphoreGive(i2cMutex); // Release the mutex
                }

                // Reset all buffers for the next message
                textMessagePos = 0;
                morseSymbolPos = 0;
                memset(textMessageBuffer, 0, sizeof(textMessageBuffer));
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
            }
            rgb_led_write(255, 255, 255); // Set LED to white after symbol feedback
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay between symbols
        }
    }
}

// =================================================================================
// --- BUZZER TASK ---
// A simple task that waits for application events and plays corresponding sounds.
// =================================================================================
static void vBuzzerTask(void *pvParameters) {
     (void)pvParameters;
     app_event_t evt;
     while (1) {
         // Block until an event is received
         if (xQueueReceive(xEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
             if (evt == APP_EVENT_MSG_SENT) {
                 // Play the "message sent" melody
                 play_message_sent_melody();
             }
         }
     }
}

// =================================================================================
// --- FIND LETTER FROM MORSE CODE ---
// A utility function to look up a Morse code string in the `morseCodes` table
// and return the corresponding character.
// =================================================================================
char find_letter_from_morse_code(char *morseCode) {
    if (strlen(morseCode) == 0) return ' '; // An empty Morse string is a space
    // Iterate through the translation table
    for (int i = 0; i < 40; i++){
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0){
            return (char)toupper(morseCodes[i].letter); // Return the uppercase letter
        }
    }
    return '?'; // Return '?' if the code is not found
}
