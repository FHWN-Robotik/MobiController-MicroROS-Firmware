#ifndef LED_STRIP_H_
#define LED_STRIP_H_

#include "ARGB.h"
#include "mobi_interfaces/msg/color_rgbw.h"
#include "mobi_interfaces/srv/detail/set_led_strip__struct.h"
#include "stm32l4xx.h"

/*
  Structs
*/

typedef struct led_strip_s {
  bool do_animation;
  uint16_t count_to_10ms;
  uint8_t current_frame;

  mobi_interfaces__srv__SetLedStrip_Request animation_config;

} led_strip_t;

/*
  ARGB Overwrites
*/
// Init led strip
void led_strip_init(led_strip_t *led_strip);

/*
  Custom led functions
*/
// Set single LED by RGBW
void led_strip_set_rgbw(uint16_t index, mobi_interfaces__msg__ColorRGBW *color);

// Fill all leds with RGBW colorCore/Inc/led_strip.hpp
void led_strip_fill_rgbw(mobi_interfaces__msg__ColorRGBW *color);

// Fill a range of leds with RGBW color
void led_strip_fill_range_rgbw(uint16_t start, uint16_t end, mobi_interfaces__msg__ColorRGBW *color);

// Update the strip
void led_strip_update();

void led_strip_clear_and_update();

/*
  Interrupt
*/
void led_strip_handle_timer_interrupt(led_strip_t *led_strip);

/*
  Animations
*/

// Stop the animation
void led_strip_stop_animation(led_strip_t *led_strip);

// Start the animation, only needed if the animation was stopped by the stop_animation() function;
void led_strip_start_animation(led_strip_t *led_strip, mobi_interfaces__srv__SetLedStrip_Request animation);

// Car-like head- and taillight
void led_strip_driving_light();

// Lights which are used to indicate a critical battery charge
void led_strip_battery_warning_light(led_strip_t *led_strip);

// Animation thats played when starting up.
void led_strip_power_on_animation(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color);

// Beacon light with RGBW color
void led_strip_beacon_rgbw(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW color, uint8_t update_rate,
                           uint8_t frame_count, uint8_t line_length, uint8_t line_count, bool rotate_left);

// Blink the whole stip or sections in a color;
void led_strip_blink(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color, uint8_t update_rate,
                     uint8_t line_length, uint8_t line_count);

// Fill animation the whole strip (used on powerup)
void led_strip_fill(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color, uint8_t update_rate,
                    uint8_t frame_count);

#endif /* LED_STRIP_H_ */