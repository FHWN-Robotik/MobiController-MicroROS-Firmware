#include "led_strip.h"
#include "ARGB.h"
#include "main.h"
#include "tim.h"
#include "utils.h"

/*
  ARGB Overwrites
*/

void led_strip_init(led_strip_t *led_strip) {
  led_strip->do_animation = false;
  led_strip->current_frame = 0;
  led_strip->count_to_10ms = 0;

  ARGB_Init();
  HAL_TIM_Base_Start_IT(&htim6);
}

void led_strip_clear() { ARGB_Clear(); }

// ----------------------------------------------------------------------------

/*
  Custom led functions
*/

void led_strip_set_rgbw(uint16_t index, mobi_interfaces__msg__ColorRGBW *color) {
  ARGB_SetRGB(index, color->r, color->g, color->b);
  ARGB_SetWhite(index, color->w);
}

void led_strip_fill_rgbw(mobi_interfaces__msg__ColorRGBW *color) { led_strip_fill_range_rgbw(0, NUM_PIXELS, color); }

void led_strip_fill_range_rgbw(uint16_t start, uint16_t end, mobi_interfaces__msg__ColorRGBW *color) {
  for (volatile uint16_t i = start; i < end; i++)
    led_strip_set_rgbw(i, color);
}

void led_strip_update() {
  while (!ARGB_Show()) {
  }
}

void led_strip_clear_and_update() {
  led_strip_clear();
  led_strip_update();
}

// ----------------------------------------------------------------------------

/*
  Interrupt a.k.a main animation loop
*/

void led_strip_handle_timer_interrupt(led_strip_t *led_strip) {
  if (!led_strip->do_animation)
    return;

  if (led_strip->animation_config.update_rate == 0)
    led_strip_stop_animation(led_strip); // if update_rate is 0, animate the first frame then stop the animation

  if (led_strip->count_to_10ms < led_strip->animation_config.update_rate) {
    led_strip->count_to_10ms++;
    return;
  }

  led_strip->count_to_10ms = 0;

  // Do animation
  switch (led_strip->animation_config.type) {
  case mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_BLINK: {
    if (led_strip->current_frame == 0) {
      uint8_t offset = round_half_up_unscaled(NUM_PIXELS * 10 / led_strip->animation_config.line_count, 10);
      for (size_t i = 0; i < led_strip->animation_config.line_count; i++) {
        uint8_t start =
          (i * offset) + led_strip->current_frame *
                           round_half_up_unscaled(NUM_PIXELS * 10 / led_strip->animation_config.frame_count, 10);
        uint8_t end = start + led_strip->animation_config.line_length;
        led_strip_fill_range_rgbw(start, end, &led_strip->animation_config.color);
      }
    }
    if (led_strip->current_frame == 1)
      led_strip_clear();

    led_strip_update();
    break;
  }
  case mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_BEACON: {
    led_strip_clear();

    if (led_strip->animation_config.rotate_left) {
      uint8_t offset = round_half_up_unscaled(NUM_PIXELS * 10 / led_strip->animation_config.line_count, 10);
      for (size_t i = 0; i < led_strip->animation_config.line_count; i++) {
        uint8_t start = (i * offset) + (NUM_PIXELS - 1) -
                        led_strip->current_frame *
                          round_half_up_unscaled(NUM_PIXELS * 10 / led_strip->animation_config.frame_count, 10);
        uint8_t end = start + led_strip->animation_config.line_length;
        led_strip_fill_range_rgbw(start, end, &led_strip->animation_config.color);
      }
    } else {

      uint8_t offset = round_half_up_unscaled(NUM_PIXELS * 10 / led_strip->animation_config.line_count, 10);
      for (size_t i = 0; i < led_strip->animation_config.line_count; i++) {
        // offset between the lines + the current frame * NUM_PIXELS / frame_count
        // offset -> spacing
        // current frame -> movement
        // NUM_PIXELS / frame_count -> scaling to the whole led strip
        uint8_t start =
          (i * offset) + led_strip->current_frame *
                           round_half_up_unscaled(NUM_PIXELS * 10 / led_strip->animation_config.frame_count, 10);
        uint8_t end = start + led_strip->animation_config.line_length;
        led_strip_fill_range_rgbw(start, end, &led_strip->animation_config.color);
      }
    }
    led_strip_update();
    break;
  }

  case mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_FILL: {
    if (led_strip->current_frame >= led_strip->animation_config.frame_count - 1) {
      led_strip_stop_animation(led_strip);
      pwr_manager_set_power_led(false);
      break;
    }

    led_strip_clear();

    uint8_t start = 0;
    uint8_t end = NUM_PIXELS;
    uint8_t half_frames = led_strip->animation_config.frame_count / 2;
    uint8_t step_size = NUM_PIXELS / half_frames;

    if (led_strip->current_frame < half_frames) { // first half of the animation
      end = step_size * led_strip->current_frame;
    } else { // second half
      start = step_size * (led_strip->current_frame - half_frames);
    }

    led_strip_fill_range_rgbw(start, end, &led_strip->animation_config.color);
    led_strip_update();

    break;
  }
  default:
    break;
  }

  led_strip->current_frame++;
  if (led_strip->current_frame > led_strip->animation_config.frame_count - 1)
    led_strip->current_frame = 0;
}

// ----------------------------------------------------------------------------

/*
  Animations
*/

void led_strip_stop_animation(led_strip_t *led_strip) { led_strip->do_animation = false; }

void led_strip_start_animation(led_strip_t *led_strip, mobi_interfaces__srv__SetLedStrip_Request animation) {
  if ((animation.type == mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_BLINK &&
       animation.line_count * animation.line_length > NUM_PIXELS) ||
      (animation.type != mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_BLINK &&
       animation.line_count * animation.line_length >= NUM_PIXELS)) {
    led_strip_stop_animation(led_strip);
    led_strip_fill_rgbw(&animation.color);
    return;
  }

  led_strip->do_animation = true;
  led_strip->animation_config = animation;
}

void led_strip_driving_light() {
  uint8_t quater = NUM_PIXELS / 4;

  // Head light
  led_strip_fill_range_rgbw(0, quater, &(mobi_interfaces__msg__ColorRGBW){.r = 0, .g = 0, .b = 0, .w = 255});

  // Tail light
  led_strip_fill_range_rgbw(2 * quater + 1, 3 * quater + 1,
                            &(mobi_interfaces__msg__ColorRGBW){.r = 255, .g = 0, .b = 0, .w = 0});
  led_strip_update();
}

void led_strip_battery_warning_light(led_strip_t *led_strip) {
  led_strip_blink(led_strip, &(mobi_interfaces__msg__ColorRGBW){.r = 255, .g = 0, .b = 0, .w = 0}, 10, NUM_PIXELS, 1);
}

void led_strip_power_on_animation(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color) {
  led_strip_fill(led_strip, color, 1, NUM_PIXELS * 2);
}

void led_strip_beacon_rgbw(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color, uint8_t update_rate,
                           uint8_t frame_count, uint8_t line_length, uint8_t line_count, bool rotate_left) {
  mobi_interfaces__srv__SetLedStrip_Request animation;
  animation.type = mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_BEACON;
  animation.color = *color;
  animation.frame_count = frame_count;
  animation.update_rate = update_rate;
  animation.line_length = line_length;
  animation.line_count = line_count;
  animation.rotate_left = rotate_left;

  led_strip_start_animation(led_strip, animation);
}

void led_strip_blink(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color, uint8_t update_rate,
                     uint8_t line_length, uint8_t line_count) {
  mobi_interfaces__srv__SetLedStrip_Request animation;
  animation.type = mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_BLINK;
  animation.frame_count = 2;
  animation.color = *color;
  animation.update_rate = update_rate;
  animation.line_length = line_length;
  animation.line_count = line_count;

  led_strip_start_animation(led_strip, animation);
}

void led_strip_fill(led_strip_t *led_strip, mobi_interfaces__msg__ColorRGBW *color, uint8_t update_rate,
                    uint8_t frame_count) {
  mobi_interfaces__srv__SetLedStrip_Request animation = {
    .type = mobi_interfaces__srv__SetLedStrip_Request__LED_ANIMATION_FILL,
    .frame_count = frame_count,
    .color = *color,
    .update_rate = update_rate,
    .line_length = 0,
    .rotate_left = false,
    .line_count = 0,
  };

  led_strip_start_animation(led_strip, animation);
}