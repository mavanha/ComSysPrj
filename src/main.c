#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include "pico/bootrom.h" // Required for reset_usb_boot
#include "pico/stdlib.h"
#include "pico/stdio.h"




// FreeRTOS headers
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>




// TKJHAT SDK header
#include "tkjhat/sdk.h"




#include <pico/cyw43_arch.h>




#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"




// --- CONFIGURATION ---
#define TEST_TCP_SERVER_IP "51.20.8.40" // UPDATE THIS TO YOUR SERVER IP
#define TCP_PORT 8080
#define WIFI_SSID "panoulu"                 // UPDATE THIS
#define WIFI_PASS NULL        // UPDATE THIS




#define DEBUG_printf printf
#define BUF_SIZE 502
#define POLL_TIME_S 5




// --- MACROS ---
#if 1
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
   unsigned int i = 0;
   printf("dump_bytes %d", len);
   for (i = 0; i < len;) {
       printf("%c", bptr[i++]);
   }
   printf("\n");
}
#define DUMP_BYTES dump_bytes
#else
#define DUMP_BYTES(A,B)
#endif




// --- TCP STRUCTS ---
typedef struct TCP_CLIENT_T_ {
   struct tcp_pcb *tcp_pcb;
   ip_addr_t remote_addr;
   uint8_t buffer[BUF_SIZE];
   int buffer_len;
   int sent_len;
   bool complete;
   int run_count;
   bool connected;
} TCP_CLIENT_T;




TCP_CLIENT_T *clientState = NULL;




// --- BUTTON / INPUT CONFIG ---
#define BTN_MODE            BUTTON1
#define BTN_ACTION          BUTTON2
#define BTN_LONG_PRESS_MS   800     
#define BTN_DEBOUNCE_MS     150     
#define MIC_AMPL_THRESHOLD  8000    
#define MIC_DOT_MAX_MS      250     
#define INPUT_TASK_PERIOD_MS 20     




// --- QUEUE CONFIG ---
#define SYMBOL_QUEUE_LENGTH     64  
#define EVENT_QUEUE_LENGTH      8
#define SERIAL_TX_QUEUE_LENGTH 20
#define PLAYBACK_QUEUE_LENGTH 40
#define SERIAL_RX_BUFFER_SIZE 128




// --- ENUMS ---
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




// State machine for IMU symbol generation
volatile enum AppState {
 STATE_IDLE, 
 STATE_ARMED, 
 STATE_COOLDOWN
} g_appState = STATE_IDLE;




// --- GLOBALS ---
static input_mode_t g_inputMode = INPUT_MODE_IMU;
static volatile int g_mic_samples_ready = 0;
static int16_t      g_mic_buffer[MEMS_BUFFER_SIZE];




static QueueHandle_t xSymbolQueue = NULL;
static QueueHandle_t xEventQueue  = NULL;
QueueHandle_t xSerialTxQueue_actual;
QueueHandle_t xPlaybackQueue;
QueueHandle_t i2cMutex;




// --- MORSE DATA ---
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




// --- IMU CONSTANTS ---
#define IMU_TILT_THRESHOLD      0.9f
#define IMU_NEUTRAL_THRESHOLD   0.5f
#define IMU_JERK_THRESHOLD      0.4f
#define IMU_DEBOUNCE_MS         300
#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2)




/* =========================
* FUNCTION PROTOTYPES
* ========================= */
static void input_task(void *pvParameters);
static void serial_tx_task(void *pvParameters);
static void serial_rx_task(void *pvParameters);
static void playback_task(void *pvParameters);
static void vBuzzerTask(void *pvParameters);
char find_letter_from_morse_code(char *morseCode);
void process_received_line(char *line);
void run_tcp_client_test(void);
void send_data_tcp(const char *msg);




// Microphone callback
static void on_pdm_samples_ready(void) {
   int n = get_microphone_samples(g_mic_buffer, MEMS_BUFFER_SIZE);
   if (n > 0) {
       g_mic_samples_ready = n;
   }
}




static void send_symbol(char c) {
   xQueueSend(xSymbolQueue, &c, 0);
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




/* =========================
* MAIN
* ========================= */
int main() {
   stdio_init_all();




   // Wait for USB (optional timeout)
   int i = 0;
   while (!stdio_usb_connected() && i < 30) {
      sleep_ms(100);
      i++;
   }
   printf("__JTKJ Morse Communicator - Starting...__\n");




   printf("Init the cyw43\n");
   if (cyw43_arch_init()) {
       printf("WiFi init failed!\n");
       return -1;
   }




   cyw43_arch_enable_sta_mode();
   printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
   if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_OPEN, 30 * 1000)) {
       printf("Failed to connect\n");
       return -1;
   } else {
       printf("Connected to hotspot\n");
   }




   printf("Starting TCP Connection...\n");
   run_tcp_client_test();




   init_hat_sdk();
   init_led();  
   init_rgb_led();
   init_buzzer();
   init_display();




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




   clear_display();
   write_text_xy(0, 0, "Morse App Ready");
   write_text_xy(0, 10, "SW1=Mode, SW2=Act");
   write_text_xy(0, 30, "RX MSG:");




   // Queues
   xSerialTxQueue_actual = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(char));
   xPlaybackQueue = xQueueCreate(PLAYBACK_QUEUE_LENGTH, sizeof(char));
   i2cMutex = xSemaphoreCreateMutex();
   xSymbolQueue = xQueueCreate(SYMBOL_QUEUE_LENGTH, sizeof(char));
   xEventQueue  = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(app_event_t));




   if (xSerialTxQueue_actual == NULL || xPlaybackQueue == NULL || i2cMutex == NULL || xSymbolQueue == NULL || xEventQueue == NULL) {
       printf("__CRITICAL ERROR: Could not create queues__\n");
       while (1) { blink_led(1); sleep_ms(100); }
   }




   // Tasks
   xTaskCreate(input_task, "InputTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
   xTaskCreate(serial_tx_task, "TxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
   xTaskCreate(serial_rx_task, "RxTask", 2048, NULL, MAIN_TASK_PRIORITY, NULL);
   xTaskCreate(playback_task, "PlaybackTask", 4096, NULL, MAIN_TASK_PRIORITY, NULL);
   xTaskCreate(vBuzzerTask, "BuzzerTask", 1024, NULL, MAIN_TASK_PRIORITY, NULL);




   printf("__Initialization complete. Starting scheduler.__\n");
   vTaskStartScheduler();




   while (1);
   return 0;
}




/* =========================
* TCP FUNCTIONS
* ========================= */




// 1. The Function to send data safely
void send_data_tcp(const char *msg) {
   if (clientState == NULL) {
       printf("__TCP: State NULL__\n");
       return;
   }
   if (!clientState->connected) {
       printf("__TCP: Not connected__\n");
       return;
   }
  
   printf("__Sending TCP: %s__\n", msg);
  
   // Lock lwIP core
   cyw43_arch_lwip_begin();
   err_t err = tcp_write(clientState->tcp_pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
   if(err == ERR_OK) {
       tcp_output(clientState->tcp_pcb); // Flush immediately
   } else {
       printf("__TCP Write Error: %d__\n", err);
   }
   // Unlock lwIP core
   cyw43_arch_lwip_end();
}




static err_t tcp_client_close(void *arg) {
   TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
   err_t err = ERR_OK;
   if (clientState->tcp_pcb != NULL) {
       tcp_arg(clientState->tcp_pcb, NULL);
       tcp_poll(clientState->tcp_pcb, NULL, 0);
       tcp_sent(clientState->tcp_pcb, NULL);
       tcp_recv(clientState->tcp_pcb, NULL);
       tcp_err(clientState->tcp_pcb, NULL);
       err = tcp_close(clientState->tcp_pcb);
       if (err != ERR_OK) {
           DEBUG_printf("close failed %d, calling abort\n", err);
           tcp_abort(clientState->tcp_pcb);
           err = ERR_ABRT;
       }
       clientState->tcp_pcb = NULL;
   }
   return err;
}




static err_t tcp_result(void *arg, int status) {
   TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
   if (status == 0) {
       DEBUG_printf("test success\n");
   } else {
       DEBUG_printf("test failed %d\n", status);
   }
   clientState->complete = true;
   return tcp_client_close(arg);
}




static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
   TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
   DEBUG_printf("tcp_client_sent %u\n", len);
   clientState->sent_len += len;
   return ERR_OK;
}




static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
   TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
   if (err != ERR_OK) {
       printf("connect failed %d\n", err);
       return tcp_result(arg, err);
   }
   clientState->connected = true;
   const char *msg = "Client Connected\n";
   tcp_write(tpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
   return ERR_OK;
}




static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
   // Keep alive polling
   return ERR_OK;
}




static void tcp_client_err(void *arg, err_t err) {
   if (err != ERR_ABRT) {
       DEBUG_printf("tcp_client_err %d\n", err);
       tcp_result(arg, err);
   }
}




err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
   TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
   if (!p) {
       return tcp_result(arg, -1);
   }
   cyw43_arch_lwip_check();
   if (p->tot_len > 0) {
       DEBUG_printf("recv %d err %d\n", p->tot_len, err);
       // Copy to buffer
       const uint16_t buffer_left = BUF_SIZE - clientState->buffer_len;
       clientState->buffer_len += pbuf_copy_partial(p, clientState->buffer + clientState->buffer_len,
                                              p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
       tcp_recved(tpcb, p->tot_len);
   }
   pbuf_free(p);




   // Process received data here if needed
   if (clientState->buffer_len > 0) {
       // Ensure null termination
       if(clientState->buffer_len < BUF_SIZE) clientState->buffer[clientState->buffer_len] = '\0';
       else clientState->buffer[BUF_SIZE-1] = '\0';
      
       printf("TCP RX: %s\n", clientState->buffer);
       // Clear buffer after processing
       clientState->buffer_len = 0;
   }
   return ERR_OK;
}




static bool tcp_client_open(void *arg) {
   TCP_CLIENT_T *clientState = (TCP_CLIENT_T*)arg;
   DEBUG_printf("Connecting to %s port %u\n", ip4addr_ntoa(&clientState->remote_addr), TCP_PORT);
   clientState->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&clientState->remote_addr));
   if (!clientState->tcp_pcb) {
       DEBUG_printf("failed to create pcb\n");
       return false;
   }




   tcp_arg(clientState->tcp_pcb, clientState);
   tcp_poll(clientState->tcp_pcb, tcp_client_poll, POLL_TIME_S * 2);
   tcp_sent(clientState->tcp_pcb, tcp_client_sent);
   tcp_recv(clientState->tcp_pcb, tcp_client_recv);
   tcp_err(clientState->tcp_pcb, tcp_client_err);




   clientState->buffer_len = 0;




   cyw43_arch_lwip_begin();
   err_t err = tcp_connect(clientState->tcp_pcb, &clientState->remote_addr, TCP_PORT, tcp_client_connected);
   cyw43_arch_lwip_end();




   return err == ERR_OK;
}




static TCP_CLIENT_T* tcp_client_init(void) {
   clientState = calloc(1, sizeof(TCP_CLIENT_T));
   if (!clientState) {
       DEBUG_printf("failed to allocate state\n");
       return NULL;
   }
   ip4addr_aton(TEST_TCP_SERVER_IP, &clientState->remote_addr);
   return clientState;
}




void run_tcp_client_test(void) {
   TCP_CLIENT_T *clientState = tcp_client_init();
   if (!clientState) {
       printf("__No state__\n");
       return;
   }
   if (!tcp_client_open(clientState)) {
       printf("__Cannot open__\n");
       tcp_result(clientState, -1);
       return;
   }
}




/* =========================
* TASKS
* ========================= */




static void input_task(void *pvParameters) {
   (void)pvParameters;
   float ax, ay, az, gx, gy, gz, t;
   float prev_ax = 0, prev_ay = 0, prev_az = 0;
   bool read_ok_imu = false;




   bool sw1_prev = false;
   bool sw2_prev = false;
   TickType_t sw1_press_tick = 0;
   TickType_t sw2_press_tick = 0;
   TickType_t sw1_last_change = 0;
   TickType_t sw2_last_change = 0;




   mic_state_t micState = MIC_STATE_IDLE;
   TickType_t  micStartTick = 0;




   init_sw1();
   init_sw2();




   g_inputMode = INPUT_MODE_IMU;
   printf("__IMU MODE__\n");




   while (1) {
       TickType_t now = xTaskGetTickCount();
       bool sw1_now = gpio_get(BTN_MODE) ? true : false;
       bool sw2_now = gpio_get(BTN_ACTION) ? true : false;




       // -------- SW1 (MODE / SEND 3 SPACES) --------
       if (sw1_now != sw1_prev && (now - sw1_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
           sw1_last_change = now;
           if (sw1_now) { // Pressed
               buzzer_play_tone(1000, 80);
               sw1_press_tick = now;
           } else { // Released
               TickType_t dt = now - sw1_press_tick;
               uint32_t ms   = dt * portTICK_PERIOD_MS;
               if (ms >= BTN_LONG_PRESS_MS) {
                   // Long press SW1: send 3 spaces
                   printf("__MSG SEND VIA 3 SPACES__\n");
                   send_space();
                   send_space();
                   send_space();
                   app_event_t evt = APP_EVENT_MSG_SENT;
                   xQueueSend(xEventQueue, &evt, 0);
               } else {
                   // Short press SW1: toggle mode
                   if (g_inputMode == INPUT_MODE_IMU) {
                       g_inputMode = INPUT_MODE_MIC;
                       printf("__MIC MODE__\n");
                       micState = MIC_STATE_IDLE;
                   } else {
                       g_inputMode = INPUT_MODE_IMU;
                       printf("__IMU MODE__\n");
                       g_appState = STATE_IDLE;
                       set_led_status(false);
                   }
               }
           }
       }
       sw1_prev = sw1_now;




       // -------- SW2 (SYMBOL / SPACE) --------
       if (sw2_now != sw2_prev && (now - sw2_last_change) > pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
           sw2_last_change = now;
           if (sw2_now) {
               buzzer_play_tone(700, 80);
               sw2_press_tick = now;
           } else {
               TickType_t dt = now - sw2_press_tick;
               uint32_t ms   = dt * portTICK_PERIOD_MS;
               if (ms >= BTN_LONG_PRESS_MS) {
                   send_space();
                   printf("__SPACE__\n");
               } else {
                   // Short press SW2
                   if (g_inputMode == INPUT_MODE_IMU) {
                       if (g_appState == STATE_IDLE) {
                           g_appState = STATE_ARMED;
                           set_led_status(true);
                           printf("__IMU ARMED__\n");
                       } else if (g_appState == STATE_ARMED) {
                            // Read IMU
                            read_ok_imu = false;
                            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                                     read_ok_imu = true;
                                }
                                xSemaphoreGive(i2cMutex);
                            }




                            if (read_ok_imu) {
                                char symbol = '?';
                                if (az > IMU_TILT_THRESHOLD) {
                                    symbol = '.';
                                } else if (ay < -IMU_TILT_THRESHOLD) {
                                    symbol = '-';
                                }




                                if (symbol != '?') {
                                    send_symbol(symbol);
                                    printf("__IMU SYMBOL '%c'__\n", symbol);
                                    set_led_status(false);
                                    g_appState = STATE_COOLDOWN;
                                } else {
                                    printf("__IMU: No clear orientation__\n");
                                }
                                prev_ax = ax; prev_ay = ay; prev_az = az;
                            }
                       }
                   }
               }
           }
       }
       sw2_prev = sw2_now;




       // Cooldown logic
       if (g_inputMode == INPUT_MODE_IMU && g_appState == STATE_COOLDOWN) {
            read_ok_imu = false;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) read_ok_imu = true;
                xSemaphoreGive(i2cMutex);
            }
            if (read_ok_imu) {
                if (fabs(az) < IMU_NEUTRAL_THRESHOLD && fabs(ay) < IMU_NEUTRAL_THRESHOLD) {
                    g_appState = STATE_IDLE;
                    printf("__IMU COOLED DOWN__\n");
                }
            }
       }




       // MIC Logic
       if (g_inputMode == INPUT_MODE_MIC && g_mic_samples_ready > 0) {
           int sample_count = g_mic_samples_ready;
           g_mic_samples_ready = 0;
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
                           printf("__MIC ACTIVE__\n");
                       }
                       break;
                   case MIC_STATE_ACTIVE:
                       if (avgAbs <= MIC_AMPL_THRESHOLD) {
                           TickType_t durationTicks = now - micStartTick;
                           uint32_t durationMs = durationTicks * portTICK_PERIOD_MS;
                           char symbol;
                           if (durationMs <= MIC_DOT_MAX_MS) {
                               symbol = '.';
                               printf("__MIC DOT__\n");
                           } else {
                               symbol = '-';
                               printf("__MIC DASH__\n");
                           }
                           send_symbol(symbol);
                           micState = MIC_STATE_IDLE;
                       }
                       break;
                   default: micState = MIC_STATE_IDLE; break;
               }
           }
       }
       vTaskDelay(pdMS_TO_TICKS(INPUT_TASK_PERIOD_MS));
   }
}




// --- MODIFIED SERIAL TX TASK WITH BUFFERING ---
static void serial_tx_task(void *pvParameters) {
   (void)pvParameters;
   char symbol;
   int spaceCount = 0;
  
   // Buffer to accumulate symbols
   char txBuffer[128] = {0};
   int txIndex = 0;




   while (1) {
       if (xQueueReceive(xSymbolQueue, &symbol, portMAX_DELAY) == pdPASS) {
          
           if (symbol == ' ') {
               spaceCount++;
              
               // Add to buffer
               if (txIndex < 127) txBuffer[txIndex++] = ' ';




               putchar(' ');
               fflush(stdout);




               // --- TRIGGER SEND ON 3 SPACES ---
               if (spaceCount == 3) {
                   // Null terminate
                   txBuffer[txIndex] = '\0';
                  
                   // 1. Send TCP
                   send_data_tcp(txBuffer);




                   // 2. Feedback
                   putchar('\n');
                   printf("\n__[Morse Send OK]__\n");
                   fflush(stdout);
                  
                   // 3. Notify local display and buzzer
                   char nl_char = '\n';
                   xQueueSend(xPlaybackQueue, &nl_char, 0);
                   app_event_t evt = APP_EVENT_MSG_SENT;
                   xQueueSend(xEventQueue, &evt, 0);




                   // 4. Reset
                   spaceCount = 0;
                   txIndex = 0;
                   memset(txBuffer, 0, sizeof(txBuffer));
               }
           } else {
               // Not a space, reset space counter
               spaceCount = 0;
              
               // Add to buffer
               if (txIndex < 127) txBuffer[txIndex++] = symbol;




               putchar(symbol);
               fflush(stdout);
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
               linePos = 0;
           }
       }
       vTaskDelay(pdMS_TO_TICKS(10));
   }
}




void process_received_line(char *line) {
   // --- COMMANDS ---
   if (strncmp(line, ".clear", 6) == 0) {
       printf("\033[2J\033[H");
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
  
   // --- MORSE PARSING ---
   bool in_debug_block = false;
   for (int loop_i = 0; line[loop_i] != '\0'; loop_i++) {
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
           else if (symbol == ' ') {
               rgb_led_write(0, 255, 255);
               vTaskDelay(pdMS_TO_TICKS(200));
               morseSymbolBuffer[morseSymbolPos] = '\0';
               char letter = find_letter_from_morse_code(morseSymbolBuffer);
               if (textMessagePos < 20) textMessageBuffer[textMessagePos++] = letter;
               morseSymbolPos = 0;
               memset(morseSymbolBuffer, 0, sizeof(morseSymbolBuffer));
           }
           else if (symbol == '\n') {
               morseSymbolBuffer[morseSymbolPos] = '\0';
               if (morseSymbolPos > 0) {
                   char letter = find_letter_from_morse_code(morseSymbolBuffer);
                   if (textMessagePos < 20) textMessageBuffer[textMessagePos++] = letter;
               }
               textMessageBuffer[textMessagePos] = '\0';
              
               if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                   clear_display();
                   write_text_xy(0, 0, "Morse App Ready");
                   write_text_xy(0, 10, "SW1=Mode, SW2=Act");
                   write_text_xy(0, 30, "RX MSG:");
                   if (textMessagePos > 0) write_text_xy(0, 40, textMessageBuffer);
                   xSemaphoreGive(i2cMutex);
               }




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




static void vBuzzerTask(void *pvParameters) {
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




char find_letter_from_morse_code(char *morseCode) {
   if (strlen(morseCode) == 0) return ' ';
   for (int loop_i = 0; loop_i < 40; loop_i++){
       if (strcmp(morseCode, morseCodes[loop_i].morseCode) == 0){
           return (char)toupper(morseCodes[loop_i].letter);
       }
   }
   return '?';
}













