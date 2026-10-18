#include "nrf_spectrum.h"
#include "core/bus_HAL.h"
#include "core/display.h"
#include "core/mykeyboard.h"

#define CHANNELS 80
#define RGB565(r, g, b) ((((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)))
uint8_t channel[CHANNELS];
static uint32_t lastSpectrumI2cSwitchCount = 0;

static void prepareSharedNrfPins() {
    if (bruceConfigPins.CC1101_bus.cs != GPIO_NUM_NC) {
        pinMode(bruceConfigPins.CC1101_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.CC1101_bus.cs, HIGH);
    }
#if !defined(LITE_VERSION)
    if (bruceConfigPins.LoRa_bus.cs != GPIO_NUM_NC) {
        pinMode(bruceConfigPins.LoRa_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.LoRa_bus.cs, HIGH);
    }
#endif
    if (bruceConfigPins.SDCARD_bus.cs != GPIO_NUM_NC) {
        pinMode(bruceConfigPins.SDCARD_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.SDCARD_bus.cs, HIGH);
    }

    // Set CE (GPIO8/SDA) LOW while CSN (GPIO9/SCL) is LOW so no I2C START condition is generated
    pinMode(bruceConfigPins.NRF24_bus.cs, OUTPUT);
    pinMode(bruceConfigPins.NRF24_bus.io0, OUTPUT);
    digitalWrite(bruceConfigPins.NRF24_bus.cs, LOW);
    delayMicroseconds(2);
    digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
    delayMicroseconds(2);
    digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
    delayMicroseconds(5);
}

static void configureSpectrumRadio(bool rebegin) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        prepareSharedNrfPins();

        if (rebegin || attempt > 0) {
            // Cleanly exit any continuous wave or active RX/TX state and give the PA+LNA /
            // Si24R1 crystal oscillator time to settle before re-initializing registers.
            NRFradio.stopConstCarrier();
            delayMicroseconds(500);
            NRFradio.powerDown();
            delay(2);
            NRFradio.begin(
                NRFSPI,
                rf24_gpio_pin_t(bruceConfigPins.NRF24_bus.io0),
                rf24_gpio_pin_t(bruceConfigPins.NRF24_bus.cs)
            );
        }
        NRFradio.stopConstCarrier();
        delayMicroseconds(500);
        NRFradio.powerUp();
        delayMicroseconds(200);
        NRFradio.setPALevel(RF24_PA_MAX, true);
        NRFradio.setAutoAck(false);
        NRFradio.disableCRC();       // accept any signal we find
        NRFradio.setAddressWidth(2); // a reverse engineering tactic (not typically recommended)
        const uint8_t noiseAddress[][2] = {
            {0x55, 0x55},
            {0xAA, 0xAA},
            {0xA0, 0xAA},
            {0xAB, 0xAA},
            {0xAC, 0xAA},
            {0xAD, 0xAA}
        };
        for (uint8_t i = 0; i < 6; ++i) { NRFradio.openReadingPipe(i, noiseAddress[i]); }
        NRFradio.setDataRate(RF24_1MBPS);
        NRFradio.startListening();
        NRFradio.stopListening();
        NRFradio.flush_rx();
        NRFradio.flush_tx();

        // Leave CE LOW while CSN is LOW so no I2C START condition is left on GPIO8/9
        digitalWrite(bruceConfigPins.NRF24_bus.cs, LOW);
        delayMicroseconds(2);
        digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
        delayMicroseconds(2);
        digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);

        // Verify that a non-zero register written above reads back intact
        // before marking the radio as configured.
        NRFradio.setChannel(42);
        if (NRFradio.getChannel() == 42 && NRFradio.getPALevel() == RF24_PA_MAX) {
            lastSpectrumI2cSwitchCount = getSysI2CPinsSwitchCount();
            return;
        }
        delay(2);
    }
}

// scanning channels
#define _BW tftWidth / CHANNELS
String scanChannels(bool web) {
    String result = "{";

    uint8_t rpdValues[CHANNELS] = {0};
    lockSysI2CBus();
    if (getSysI2CPinsSwitchCount() != lastSpectrumI2cSwitchCount || NRFradio.getPALevel() != RF24_PA_MAX) {
        configureSpectrumRadio(true);
    } else {
        pinMode(bruceConfigPins.NRF24_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
        pinMode(bruceConfigPins.NRF24_bus.io0, OUTPUT);
        digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
    }

    for (int i = 0; i < CHANNELS; i++) {
        NRFradio.setChannel(i);
        NRFradio.startListening();
        delayMicroseconds(160);
        bool foundSignal = NRFradio.testRPD();
        NRFradio.stopListening();

        int rpd = 0;
        if (foundSignal || NRFradio.testRPD() || NRFradio.available()) {
            NRFradio.flush_rx();
            rpd = 1;
        }
        channel[i] = (channel[i] * 3 + rpd * 125) / 4;
        rpdValues[i] = channel[i];
    }

    // Leave CE LOW while CSN is LOW so no I2C START condition is left on GPIO8/9
    digitalWrite(bruceConfigPins.NRF24_bus.cs, LOW);
    delayMicroseconds(2);
    digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
    delayMicroseconds(2);
    digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
    unlockSysI2CBus();

    for (int i = 0; i < CHANNELS; i++) {
        int level = rpdValues[i];
        int x = i * _BW;
        int c = i;

        tft.drawFastVLine(
            x, tftHeight - (10 + level), level, (i % 2 == 0) ? bruceConfig.priColor : TFT_DARKGREY
        ); // for level display

        tft.drawFastVLine(
            x, 0, tftHeight - (9 + level), (i % 8) ? TFT_BLACK : RGB565(25, 25, 25)
        );                                                    /// for clearing
        tft.drawFastVLine(x, 0, level, bruceConfig.secColor); /// for top display
        // show 5 channel gap only
        if (c % 5 == 0 && c != 0) { tft.drawCentreString(String(c).c_str(), x, tftHeight / 2, 1); }

        if (web) {
            if (i > 0) result += ",";
            result += String(level);
        }
    }

    if (web) result += "}";
    return result; // return a string in this format "{1,32,45,32,84,32 .... 12,54,65}" with 80 values to be
                   // used in the WebUI (Future)
}

void nrf_spectrum() {
    memset(channel, 0, sizeof(channel));
    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextSize(FP);
    tft.drawString("2.40Ghz", 0, tftHeight - LH);
    tft.drawCentreString("2.44Ghz", tftWidth / 2, tftHeight - LH, 1);
    tft.drawRightString("2.48Ghz", tftWidth, tftHeight - LH, 1);

    if (nrf_start(NRF_MODE_SPI)) { // This function only works on SPI
        lockSysI2CBus();
        configureSpectrumRadio(false);
        unlockSysI2CBus();

        while (!check(EscPress)) {
            if (AnyKeyPress && !EscPress) { AnyKeyPress = false; }
            scanChannels();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        lockSysI2CBus();
        prepareSharedNrfPins();
        NRFradio.stopListening();
        NRFradio.powerDown();
        digitalWrite(bruceConfigPins.NRF24_bus.cs, LOW);
        delayMicroseconds(2);
        digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
        delayMicroseconds(2);
        digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
        unlockSysI2CBus();
        delay(250);
        return;

    } else {
        Serial.println("Fail Starting radio");
        displayError("NRF24 not found");
        delay(500);
        return;
    }
}
