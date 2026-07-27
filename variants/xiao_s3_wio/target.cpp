#include <Arduino.h>
#include "target.h"

XiaoS3WIOBoard board;

#if defined(P_LORA_SCLK)
  #if defined(USE_ETH_W5500)
    // the W5500 Ethernet module shares SCK/MISO/MOSI with the radio, and the
    // Arduino Ethernet library drives it via the global SPI object (FSPI).
    // Put the radio on the same bus so two SPI hosts don't fight over the pins.
    SPIClass radio_spi(FSPI);
  #else
    SPIClass radio_spi;
  #endif
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, radio_spi);
#else
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif

WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
#if ENV_INCLUDE_GPS
  #include <helpers/sensors/MicroNMEALocationProvider.h>
  MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
  EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
  EnvironmentSensorManager sensors;
#endif

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
#endif

bool radio_init() {
  fallback_clock.begin();
  rtc_clock.begin(Wire);
  pinMode(21, INPUT);
  pinMode(48, OUTPUT);

  #if defined(P_LORA_SCLK)
  radio_spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
  return radio.std_init(&radio_spi);
#else
  return radio.std_init();
#endif
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

