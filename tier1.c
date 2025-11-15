#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

#include "tkjhat/sdk.h"

#define MAIN_TASK_PRIORITY (tskIDLE_PRIORITY + 2)

// Queue length for outgoing Morse symbols
#define SERIAL_TX_QUEUE_LENGTH 20

// IMU sensor thresholds
#define IMU_DOT_THRESHOLD      0.9f    // az > 0.9  → '.'
#define IMU_DASH_THRESHOLD    -0.9f    // ay < -0.9 → '-'
#define IMU_NEUTRAL_THRESHOLD  0.5f    // |az|,|ay| < 0.5 → neutral

// State machine for IMU symbol generation
volatile enum AppState {
    STATE_IDLE,      // waiting for user
    STATE_ARMED,     // ready to detect gesture
    STATE_COOLDOWN   // wait to return to neutral before re-arming
} g_appState = STATE_IDLE;

// FreeRTOS objects
QueueHandle_t xSerialTxQueue;   // queue of chars to send over USB serial
QueueHandle_t i2cMutex;         // protect I2C access to IMU

// Function prototypes (Tier 1)
static void imu_task(void *pvParameters);
static void serial_tx_task(void *pvParameters);
static void button_isr_callback(uint gpio, uint32_t eventMask);

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("JTKJ Morse Communicator - Tier 1\n");

    init_hat_sdk();
    init_led();
    init_button1();
    init_button2();

    // IMU init
    if (init_ICM42670() != 0) {
        printf("CRITICAL ERROR: ICM42670 initialization failed!\n");
        while (1) {
            blink_led(1);
            sleep_ms(100);
        }
    } else {
        printf("ICM42670 initialized successfully.\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("ERROR: ICM42670 start failed!\n");
            while (1) {
                blink_led(2);
                sleep_ms(100);
            }
        }
    }

    // Queues & mutex
    xSerialTxQueue = xQueueCreate(SERIAL_TX_QUEUE_LENGTH, sizeof(char));
    i2cMutex       = xSemaphoreCreateMutex();

    if (xSerialTxQueue == NULL || i2cMutex == NULL) {
        printf("CRITICAL ERROR: Could not create queue or mutex\n");
        while (1);
    }

    // Tasks: IMU producer + serial sender
    xTaskCreate(imu_task,      "IMUTask",  1024, NULL, MAIN_TASK_PRIORITY, NULL);
    xTaskCreate(serial_tx_task,"TxTask",   1024, NULL, MAIN_TASK_PRIORITY, NULL);

    // Button interrupts:
    //  - BUTTON1: arm IMU (start recording next '.' / '-')
    //  - BUTTON2: push a space symbol into the queue
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &button_isr_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    printf("Tier 1 init complete. Starting scheduler.\n");
    vTaskStartScheduler();

    while (1);
    return 0;
}

/* ========= IMU TASK: detect '.' / '-' from orientation ========= */

static void imu_task(void *pvParameters) {
    (void)pvParameters;
    float ax, ay, az, gx, gy, gz, t;
    bool read_ok = false;

    while (1) {
        read_ok = false;

        if (g_appState == STATE_ARMED || g_appState == STATE_COOLDOWN) {
            // Take mutex before I2C read
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                    read_ok = true;
                } else {
                    printf("IMU Task: Failed to read sensor data.\n");
                }
                xSemaphoreGive(i2cMutex);
            }
        }

        // Process data (after releasing I2C)
        if (read_ok) {
            if (g_appState == STATE_ARMED) {
                // Board face up → dot
                if (az > IMU_DOT_THRESHOLD) {
                    char symbol = '.';
                    xQueueSend(xSerialTxQueue, &symbol, 0);
                    set_led_status(false);
                    g_appState = STATE_COOLDOWN;
                }
                // Board tilted side (negative Y) → dash
                else if (ay < IMU_DASH_THRESHOLD) {
                    char symbol = '-';
                    xQueueSend(xSerialTxQueue, &symbol, 0);
                    set_led_status(false);
                    g_appState = STATE_COOLDOWN;
                }
            } else if (g_appState == STATE_COOLDOWN) {
                // Wait until board returns to “neutral” before allowing another symbol
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

/* ========= SERIAL TX TASK: send queued symbols over USB ========= */

static void serial_tx_task(void *pvParameters) {
    (void)pvParameters;
    char symbol;
    int spaceCount = 0;

    while (1) {
        if (xQueueReceive(xSerialTxQueue, &symbol, portMAX_DELAY) == pdPASS) {

            if (symbol == ' ') {
                // count how many times space button was pressed
                spaceCount++;
            } else {
                // normal symbol resets space counter
                putchar(symbol);
                fflush(stdout);
                spaceCount = 0;
            }

            // 1× space  → print one space
            if (spaceCount == 1) {
                putchar(' ');
                fflush(stdout);
            }
            // 2× space  → second space (between words)
            else if (spaceCount == 2) {
                putchar(' ');
                fflush(stdout);
            }
            // 3× space  → end of message (newline)
            else if (spaceCount == 3) {
                putchar('\n');
                printf("\n[Morse Send OK]\n");
                fflush(stdout);
                spaceCount = 0;
            }
        }
    }
}

/* ========= BUTTON ISR: arm IMU & send spaces ========= */

static void button_isr_callback(uint gpio, uint32_t eventMask) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == BUTTON1) {
        // Arm IMU: next orientation becomes '.' or '-'
        if (g_appState == STATE_IDLE) {
            g_appState = STATE_ARMED;
            set_led_status(true);   // LED ON while armed
        }
    } else if (gpio == BUTTON2) {
        // Space symbol sent to TX task
        char symbol = ' ';
        xQueueSendToBackFromISR(xSerialTxQueue, &symbol, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
