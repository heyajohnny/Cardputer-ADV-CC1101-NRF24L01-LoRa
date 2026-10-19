#ifndef __NRF_COMMON_H
#define __NRF_COMMON_H
#include <RF24.h>
#include <globals.h>

// Define the Macros case it hasn't been declared
#ifndef NRF24_CE_PIN
#define NRF24_CE_PIN -1
#endif
#ifndef NRF24_SS_PIN
#define NRF24_SS_PIN -1
#endif

enum NRF24_MODE {
    NRF_MODE_DISABLED, // 0b00
    NRF_MODE_SPI,      // 0b01
    NRF_MODE_UART,     // 0b10
    NRF_MODE_BOTH      // 0b11
};
#define CHECK_NRF_SPI(mode) (mode & NRF_MODE_SPI)
#define CHECK_NRF_UART(mode) (mode & NRF_MODE_UART)
#define CHECK_NRF_BOTH(mode) (mode == NRF_MODE_BOTH)

extern RF24 NRFradio;
extern HardwareSerial NRFSerial; // Uses UART2 for External NRF's
extern SPIClass *NRFSPI;

NRF24_MODE nrf_setMode();

bool nrf_start(NRF24_MODE mode);

// Pull CE (GPIO8/SDA) LOW only while CSN (GPIO9/SCL) is LOW so that
// SDA never falls while SCL is HIGH (which would generate an I2C START
// condition on the shared GPIO8/GPIO9 keyboard/sensor I2C bus).
static inline void nrfSafeCeLow() {
    digitalWrite(bruceConfigPins.NRF24_bus.cs, LOW);
    delayMicroseconds(2);
    digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
    delayMicroseconds(2);
    digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
    delayMicroseconds(2);
}

void nrfPrepareSharedPins(bool afterI2c);
void nrfDrainPendingKeyTraffic(bool stopOnEsc);

void nrf_info();
#endif
