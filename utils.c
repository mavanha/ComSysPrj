// Group members' names: Hoa Ly, Jonathan Tran, Kien Quoc Vu

// =================================================================================
 // This file adds higher-level utilities on top of tkjhat/sdk.h:
 //  - Combined initialization (LED + RGB + buzzer + display)
 //  - Standard LED status patterns
 //  - Standard buzzer melodies
 // =================================================================================

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include <tkjhat/sdk.h>      
#include <tkjhat/utils.h>    

/**
 * @brief A possibly NULL/empty string.
 */
static bool is_nonempty(const char *s) {
    return (s != NULL) && (s[0] != '\0');
}

/**
 * @brief Clear display and optionally write up to 2 lines of text.
 */
static void display_two_lines(const char *line1, const char *line2) {
    clear_display();

    if (is_nonempty(line1)) {
        write_text_xy(0, 10, line1);
    }
    if (is_nonempty(line2)) {
        write_text_xy(0, 30, line2);
    }
}

// Public API implementation

void hat_utils_init_all(void) {
    init_hat_sdk();
    sleep_ms(300);

    init_red_led();
    init_rgb_led();
    init_buzzer();
    init_display();

    clear_display();
    write_text_xy(0, 0, "TKJHAT ready");
    write_text_xy(0, 16, "Utils init OK");

    set_red_led_status(true);
    rgb_led_write(0, 80, 0);  
    sleep_ms(150);
    set_red_led_status(false);
    rgb_led_write(0, 0, 0);   
}

void hat_utils_show_status(const char *line1, const char *line2) {
    display_two_lines(line1, line2);
}

void hat_utils_show_led_status(hat_status_t status) {
    switch (status) {
        case HAT_STATUS_OK:
            // Green flash + single red blink
            rgb_led_write(0, 120, 0);
            set_red_led_status(true);
            sleep_ms(150);
            set_red_led_status(false);
            rgb_led_write(0, 0, 0);
            break;

        case HAT_STATUS_ERROR:
            // Solid red + 3 quick blinks
            rgb_led_write(255, 0, 0);
            for (int i = 0; i < 3; ++i) {
                set_red_led_status(true);
                sleep_ms(120);
                set_red_led_status(false);
                sleep_ms(120);
            }
            rgb_led_write(0, 0, 0);
            break;

        case HAT_STATUS_BUSY:
            // Blue 
            for (int level = 0; level <= 255; level += 25) {
                rgb_led_write(0, 0, level);
                sleep_ms(40);
            }
            for (int level = 255; level >= 0; level -= 25) {
                rgb_led_write(0, 0, level);
                sleep_ms(40);
            }
            rgb_led_write(0, 0, 0);
            break;

        default:
            break;
    }
}

void hat_utils_play_melody(hat_melody_t melody) {
    switch (melody) {
        case HAT_MELODY_STARTUP:
            // Ascending 3-note melody
            buzzer_play_tone(440, 150); 
            sleep_ms(50);
            buzzer_play_tone(554, 150); 
            sleep_ms(50);
            buzzer_play_tone(659, 300); 
            break;

        case HAT_MELODY_SUCCESS:
            // Short success pattern
            buzzer_play_tone(880, 120); 
            sleep_ms(40);
            buzzer_play_tone(1175, 200); 
            break;

        case HAT_MELODY_ERROR:
            // Descending error pattern
            buzzer_play_tone(392, 250); 
            sleep_ms(80);
            buzzer_play_tone(262, 400); 
            break;

        default:
            break;
    }

    buzzer_turn_off();
}

void hat_utils_feedback(hat_status_t status,
                        hat_melody_t  melody,
                        const char   *line1,
                        const char   *line2)
{
    display_two_lines(line1, line2);
    hat_utils_show_led_status(status);
    hat_utils_play_melody(melody);
}
