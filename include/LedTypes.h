#pragma once

enum LedState {
  LED_STATE_OFF,
  LED_STATE_WIFI_CONNECTING,  // green/red, alternating
  LED_STATE_AP_CONFIG,        // blue, blinking
  LED_STATE_WAITING_PRINTER,  // red, solid: printer off / not connected
  LED_STATE_IDLE_READY,       // green, solid
  LED_STATE_PRINTING,         // green, blinking
  LED_STATE_RESET_HELD,       // red, blinking
  LED_STATE_PRINTER_ERROR     // yellow, blinking: printer error OR unknown status
};
