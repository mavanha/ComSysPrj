// Group members' names: Hoa Ly, Jonathan Tran, Kien Quoc Vu

/*
I get the inspiration for the format of this .h file from
the format the official sdk.h file of the course on Github
*/
 
 // =================================================================================
 // This file adds higher-level utilities on top of tkjhat/sdk.h:
 //  - Combined initialization (LED + RGB + buzzer + display)
 //  - Standard LED status patterns
 //  - Standard buzzer melodies
 // =================================================================================

#ifndef TKJHAT_UTILS_H
#define TKJHAT_UTILS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Standard LED/RGB status patterns.
 */
typedef enum {
    HAT_STATUS_OK = 0,       
    HAT_STATUS_ERROR,        
    HAT_STATUS_BUSY          
} hat_status_t;

/**
 * @brief Standard short melodies for the buzzer.
 */
typedef enum {
    HAT_MELODY_STARTUP = 0,  
    HAT_MELODY_SUCCESS,      
    HAT_MELODY_ERROR        
} hat_melody_t;

/**
 * @brief Convenience initialization for HAT peripherals commonly used together.
 * This function:
 *  - Calls init_hat_sdk() to set up I2C and basic state.
 *  - Initializes the red LED, RGB LED, buzzer and display.
 *  - Clears the display and briefly blinks the LED as a power-on indicator.
 * Call this once at startup, after stdio_init_all() / TinyUSB init.
 */
void hat_utils_init_all(void);

/**
 * @brief Show a two-line status message on the SSD1306 display.
 *
 * Line 1 is drawn near the top, line 2 below it.
 * If either pointer is NULL or empty string, that line is skipped.
 *
 * @param line1 First line of text (may be NULL).
 * @param line2 Second line of text (may be NULL).
 */
void hat_utils_show_status(const char *line1, const char *line2);

/**
 * @brief Display a short status pattern using LEDs.
 *
 * @param status Desired status pattern.
 */
void hat_utils_show_led_status(hat_status_t status);

/**
 * @brief Play one of the predefined buzzer melodies.
 *
 * @param melody Which melody to play.
 */
void hat_utils_play_melody(hat_melody_t melody);

/**
 * @brief Convenience combined status feedback:
 *  - Shows text on the display
 *  - Shows LED pattern
 *  - Plays the selected melody
 *
 * @param status Status pattern for LEDs.
 * @param melody Melody to play.
 * @param line1  Optional first line of text (may be NULL).
 * @param line2  Optional second line of text (may be NULL).
 */
void hat_utils_feedback(hat_status_t status,
                        hat_melody_t  melody,
                        const char   *line1,
                        const char   *line2);

#endif /* TKJHAT_UTILS_H */
