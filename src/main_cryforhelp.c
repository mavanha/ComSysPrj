#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

#include "tkjhat/sdk.h"

#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

// Queue lengths
#define SERIAL_TX_QUEUE_LENGTH 20
#define PLAYBACK_QUEUE_LENGTH 40 
#define SERIAL_RX_BUFFER_SIZE 128 

// IMU sensor thresholds
#define IMU_DOT_THRESHOLD 0.9f    
#define IMU_DASH_THRESHOLD -0.9f  
#define IMU_NEUTRAL_THRESHOLD 0.5f 

// State machine for IMU symbol generation
volatile enum AppState {
    STATE_IDLE,     
    STATE_ARMED,    
    STATE_COOLDOWN  
} g_appState = STATE_IDLE;

// FreeRTOS Queues
QueueHandle_t xSerialTxQueue; 
QueueHandle_t xPlaybackQueue; 

QueueHandle_t i2cMutex;

// Morse-to-Text translation
struct MorseAlphabet {
    char morseCode[7];
    char letter;
};

struct MorseAlphabet morseCodes[40] = {
    {".-", 'a'}, {"-...", 'b'}, {"-.-.", 'c'}, {"-..", 'd'}, {".", 'e'}, 
    {"..-.", 'f'}, {"--.", 'g'}, {"....", 'h'}, {"..", 'i'}, {".---", 'j'}, 
    {"-.-", 'k'}, {".-..", 'l'}, {"--", 'm'}, {"-.", 'n'}, {"---", 'o'}, 
    {".--.", 'p'}, {"--.-", 'q'}, {".-.", 'r'}, {"...", 's'}, {"-", 't'}, 
    {"..-", 'u'}, {"...-", 'v'}, {".--", 'w'}, {"-..-", 'x'}, {"-.--", 'y'}, 
    {"--..", 'z'}, {"-----", '0'}, {".----", '1'}, {"..---", '2'}, 
    {"...--", '3'}, {"....-", '4'}, {".....", '5'}, {"-....", '6'}, 
    {"--...", '7'}, {"---..", '8'}, {"----.", '9'}, {".-.-.-", '.'}, 
    {"--..--", ','}, {"..--..", '?'}, {"-.-.--", '!'}, {"", ' '}
};

/* =========================
 * FUNCTION PROTOTYPES
 * ========================= */

static void imu_task(void *pvParameters);
static void serial_tx_task(void *pvParameters);
static void serial_rx_task(void *pvParameters);
static void playback_task(void *pvParameters);
static void button_isr_callback(uint gpio, uint32_t eventMask);
char find_letter_from_morse_code(char *morseCode);
void process_received_line(char *line);

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100); 
    }
    printf("JTKJ Morse Communicator - Starting...\n");

    init_hat_sdk(); 
    init_led();     
    init_rgb_led(); 
    init_buzzer();  
    init_display(); 
    init_button1(); 
    init_button2(); 

    if (init_ICM42670() != 0) {
        printf("CRITICAL ERROR: ICM42670 initialization failed!\n");
        while (1) { blink_led(1); sleep_ms(100); } 
    } else {
        printf("ICM42670 initialized successfully.\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("ERROR: ICM42670 start failed!\n");
            while (1) { blink_led(2); sleep_ms(100); } 
        }
    }
    
    // Display message
    clear_display();
    write_text_xy(0, 0, "Morse App Ready");
    write_text_xy(0, 10, "SW1=Arm, SW2=Space");
    write_text_xy(0, 30, "RX MSG:");

    // --- FreeRTOS Queues ---
    xSerialTxQueue = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(char));
    xPlaybackQueue = xQueueCreate(PLAYBACK_QUEUE_LENGTH, sizeof(char));
    
    i2cMutex = xSemaphoreCreateMutex();

    if (xSerialTxQueue == NULL || xPlaybackQueue == NULL || i2cMutex == NULL) {
        printf("CRITICAL ERROR: Could not create queues or mutex\n");
        while (1);
    }

    // --- FreeRTOS Tasks ---
    xTaskCreate(imu_task, "IMUTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_tx_task, "TxTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_rx_task, "RxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(playback_task, "PlaybackTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);

    // --- Button Interrupts ---
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &button_isr_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    printf("Initialization complete. Starting scheduler.\n");
    
    // --- Start FreeRTOS Scheduler ---
    vTaskStartScheduler();

    while (1);
    return 0;
}

static void imu_task(void *pvParameters) {
    (void)pvParameters; 
    float ax, ay, az, gx, gy, gz, t;
    bool read_ok = false;

    while (1) {
        read_ok = false;
        if (g_appState == STATE_ARMED || g_appState == STATE_COOLDOWN) {
            
            // Wait 50ms for the lock.
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                    read_ok = true;
                } else {
                    printf("IMU Task: Failed to read sensor data.\n");
                }
                xSemaphoreGive(i2cMutex);
            }
        }

        // Process the data after giving the lock back
        if (read_ok) {
            if (g_appState == STATE_ARMED) {
                if (az > IMU_DOT_THRESHOLD) {
                    char symbol = '.';
                    xQueueSend(xSerialTxQueue, &symbol, 0);
                    set_led_status(false); 
                    g_appState = STATE_COOLDOWN;
                } else if (ay < IMU_DASH_THRESHOLD) {
                    char symbol = '-';
                    xQueueSend(xSerialTxQueue, &symbol, 0);
                    set_led_status(false); 
                    g_appState = STATE_COOLDOWN;
                }
            } else if (g_appState == STATE_COOLDOWN) {
                 if (fabs(az) < IMU_NEUTRAL_THRESHOLD && fabs(ay) < IMU_NEUTRAL_THRESHOLD) {
                    g_appState = STATE_IDLE;
                }
            }
        }
        
        if (g_appState == STATE_IDLE) {
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static void serial_tx_task(void *pvParameters) {
    (void)pvParameters; 
    char symbol;
    int spaceCount = 0; // counter

    while (1) {
        if (xQueueReceive(xSerialTxQueue, &symbol, portMAX_DELAY) == pdPASS) {
            
            // --- Logic to handle spaces ---
            if (symbol == ' ') {
                spaceCount++;
            } else {
                // reset the space count.
                putchar(symbol);
                fflush(stdout); 
                spaceCount = 0; 
            }

            // --- Space button presses ---
            if (spaceCount == 1) {
                putchar(' ');
                fflush(stdout);
            }
            else if (spaceCount == 2) {
                putchar(' ');
                fflush(stdout);
            }
            else if (spaceCount == 3) {
                putchar('\n');
                printf("\n[Morse Send OK]\n");
                fflush(stdout);
                spaceCount = 0; // Reset
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
            putchar(c);
            fflush(stdout);

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
    // --- Check for Commands ---
    if (strncmp(line, ".clear", 6) == 0) {
        printf("\033[2J\033[H");
        fflush(stdout);
        return;
    }

    if (strncmp(line, ".exit", 5) == 0) {
        printf("\n--- .exit command received. Halting. ---\n");
        fflush(stdout);
        while (1) {
            blink_led(1);
            sleep_ms(200);
        }
    }

    // --- Parse for Morse, skipping debug blocks ---
    bool in_debug_block = false;
    for (int i = 0; line[i] != '\0'; i++) {
        
        if (line[i] == '_' && line[i+1] == '_') {
            in_debug_block = !in_debug_block;
            i++; 
            continue;
        }

        if (!in_debug_block) {
            char symbol = line[i];
            if (symbol == '.' || symbol == '-' || symbol == ' ') {
                xQueueSend(xPlaybackQueue, &symbol, pdMS_TO_TICKS(10));
            }
        }
    }

    // --- Send newline to playback task ---
    char nl = '\n';
    xQueueSend(xPlaybackQueue, &nl, pdMS_TO_TICKS(10));
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
                // Translate the FINAL letter
                morseSymbolBuffer[morseSymbolPos] = '\0';
                if (morseSymbolPos > 0) { 
                    char letter = find_letter_from_morse_code(morseSymbolBuffer);
                    if (textMessagePos < 20) {
                        textMessageBuffer[textMessagePos++] = letter;
                    }
                }
                
                textMessageBuffer[textMessagePos] = '\0';
                
                // Wait 100ms for the lock.
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    
                    clear_display();
                    
                    // Redraw the persistent UI
                    write_text_xy(0, 0, "Morse App Ready");
                    write_text_xy(0, 10, "SW1=Arm, SW2=Space");
                    write_text_xy(0, 30, "RX MSG:");
                    
                    // Write the new message
                    if (textMessagePos > 0) { 
                        write_text_xy(0, 40, textMessageBuffer);
                    }
                    xSemaphoreGive(i2cMutex);
                }
                
                // Reset
                textMessagePos = 0;
                morseSymbolPos = 0; 
                memset(textMessageBuffer, 0, sizeof(textMessageBuffer));
                memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer)); 
            }

            rgb_led_write(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void button_isr_callback(uint gpio, uint32_t eventMask) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == BUTTON1) {
        if (g_appState == STATE_IDLE) {
            g_appState = STATE_ARMED;
            set_led_status(true); 
        }
    } 
    else if (gpio == BUTTON2) {
        char symbol = ' ';
        xQueueSendToBackFromISR(xSerialTxQueue, &symbol, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

char find_letter_from_morse_code(char *morseCode) {
    if (strlen(morseCode) == 0) return ' '; // space was found

    for (int i = 0; i < 40; i++){
        if (strcmp(morseCode, morseCodes[i].morseCode) == 0){
            return (char)toupper(morseCodes[i].letter); 
        }
    }
    return '?';
}


