#include "nrf_common.h"
#include "../../core/bus_HAL.h"
#include "../../core/mykeyboard.h"

RF24 NRFradio(bruceConfigPins.NRF24_bus.io0, bruceConfigPins.NRF24_bus.cs);
HardwareSerial NRFSerial = HardwareSerial(2); // Uses UART2 for External NRF's
SPIClass *NRFSPI;

void nrf_info() {
    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextSize(FM);
    tft.setTextColor(TFT_RED, bruceConfig.bgColor);
    tft.drawCentreString("_Disclaimer_", tftWidth / 2, 10, 1);
    tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
    tft.setTextSize(FP);
    tft.setCursor(15, 33);
    padprintln("These functions were made to be used in a controlled environment for STUDY only.");
    padprintln("");
    padprintln("DO NOT use these functions to harm people or companies, you can go to jail!");
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    padprintln("");
    padprintln(
        "This device is VERY sensible to noise, so long wires or passing near VCC line can make "
        "things go wrong."
    );
    delay(1000);
    while (!check(AnyKeyPress)) { vTaskDelay(pdMS_TO_TICKS(1)); }
}

bool nrf_start(NRF24_MODE mode) {
    bool result = false;

    // CC1101, NRF24, LoRa and SD card share one SPI bus. Deselect the other devices
    // before any NRF24 traffic so they cannot answer on the bus.
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

    if (mode == NRF_MODE_DISABLED) return false;

    if (CHECK_NRF_UART(mode)) {
        if (USBserial.getSerialOutput() == &Serial1) {
            displayError("(E) UART already in use", true);
            result = false;
        }
        NRFSerial.begin(115200, SERIAL_8N1, bruceConfigPins.uart_bus.rx, bruceConfigPins.uart_bus.tx);
        Serial.println("NRF24 on Serial Started");
        result = true;
    };

    if (!CHECK_NRF_SPI(mode)) return result;

    // Wait so any key-release interrupt from pressing Enter on the menu is
    // drained by InputHandler before NRF24 takes over GPIO8/GPIO9.
    nrfDrainPendingKeyTraffic(false);

    lockSysI2CBus();

    // Always recover GPIO8/GPIO9 from I2C and re-assert CE LOW and CS HIGH before begin().
    nrfPrepareSharedPins(true);
    delay(10); // Let GPIO8/GPIO9 leave the I2C peripheral before SPI traffic

    NRFSPI =
        acquireSPIBus(bruceConfigPins.NRF24_bus.sck, bruceConfigPins.NRF24_bus.miso, bruceConfigPins.NRF24_bus.mosi);
    if (!NRFSPI) {
        Serial.println("No hardware SPI bus available for NRF24, falling back to default SPI");
        NRFSPI = &SPI;
    }
    NRFSPI->begin(
        (int8_t)bruceConfigPins.NRF24_bus.sck,
        (int8_t)bruceConfigPins.NRF24_bus.miso,
        (int8_t)bruceConfigPins.NRF24_bus.mosi,
        -1
    );
    delay(10);

    for (int attempt = 0; attempt < 3; ++attempt) {
        nrfSafeCeLow();
        NRFradio.stopConstCarrier();
        delayMicroseconds(500);
        NRFradio.powerDown();
        delay(2);
        if (NRFradio.begin(
                NRFSPI,
                rf24_gpio_pin_t(bruceConfigPins.NRF24_bus.io0),
                rf24_gpio_pin_t(bruceConfigPins.NRF24_bus.cs)
            )) {
            nrfSafeCeLow();
            NRFradio.stopConstCarrier();
            delayMicroseconds(500);
            NRFradio.powerUp();
            delayMicroseconds(200);
            NRFradio.setPALevel(RF24_PA_MAX, true);
            result = true;
            break;
        }
        delay(10);
    }
    if (!result) {
        unlockSysI2CBus();
        return false;
    }
    unlockSysI2CBus();
    return result;
}

void nrfPrepareSharedPins(bool afterI2c) {
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

    if (afterI2c) {
        // If any I2C slave on GPIO8(SDA)/GPIO9(SCL) (TCA8418, ES8311, BMI270)
        // is holding SDA (GPIO8 = NRF24 CE) LOW mid-byte, clock SCL (GPIO9)
        // up to 9 times until the slave releases SDA HIGH.
        pinMode(bruceConfigPins.NRF24_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
        pinMode(bruceConfigPins.NRF24_bus.io0, INPUT_PULLUP);
        delay(1);
        for (int clk = 0; clk < 18 && digitalRead(bruceConfigPins.NRF24_bus.io0) == LOW; ++clk) {
            digitalWrite(bruceConfigPins.NRF24_bus.cs, LOW);
            delayMicroseconds(50);
            digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
            delayMicroseconds(50);
        }
    }

    pinMode(bruceConfigPins.NRF24_bus.cs, OUTPUT);
    pinMode(bruceConfigPins.NRF24_bus.io0, OUTPUT);

    if (afterI2c) {
        // Issue a clean I2C STOP condition (SDA/io0 LOW -> HIGH while SCL/cs is HIGH)
        // to terminate any spurious I2C START condition created when setI2cPinsTooutput()
        // switched GPIO8/GPIO9 back from Wire1.
        digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
        delayMicroseconds(50);
        digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
        delayMicroseconds(50);
        digitalWrite(bruceConfigPins.NRF24_bus.io0, HIGH);
        delayMicroseconds(50);
    }

    // Set CE (GPIO8/SDA) LOW while CSN (GPIO9/SCL) is LOW so no I2C START condition is generated.
    nrfSafeCeLow();
    delay(2);
}

void nrfDrainPendingKeyTraffic(bool stopOnEsc) {
    unsigned long waitStart = millis();
    while (millis() - waitStart < 350) {
        if (stopOnEsc && EscPress) break;
        if (stopOnEsc && AnyKeyPress && !EscPress) {
            // Clear AnyKeyPress so taskInputHandler is not throttled by its 75 ms gate
            // and can immediately service the key-release interrupt on GPIO11.
            AnyKeyPress = false;
        }
        uint32_t switchSnap = getSysI2CPinsSwitchCount();
        vTaskDelay(pdMS_TO_TICKS(20));
        if (stopOnEsc && EscPress) break;
        if (stopOnEsc && AnyKeyPress && !EscPress) {
            AnyKeyPress = false;
        }
        if (digitalRead(11) == HIGH && (!AnyKeyPress || (stopOnEsc && EscPress)) && !SelPress) {
            vTaskDelay(pdMS_TO_TICKS(60));
            if (digitalRead(11) == HIGH && getSysI2CPinsSwitchCount() == switchSnap) {
                break;
            }
        }
    }
}

NRF24_MODE nrf_setMode() {
    NRF24_MODE mode = NRF_MODE_DISABLED;
    bool nrfSPI = true;
    bool nrfUART = true;
    if (bruceConfigPins.NRF24_bus.checkConflict(GPIO_NUM_NC)) {
        displayError("NRF24 pins not configured", true);
        nrfSPI = false;
    }
    // Serial UART oly display errors on Serial Monitor
    if (bruceConfigPins.NRF24_bus.checkConflict(bruceConfigPins.uart_bus.rx) ||
        bruceConfigPins.NRF24_bus.checkConflict(bruceConfigPins.uart_bus.tx)) {
        Serial.println("NRF24 pins conflict with UART pins");
        nrfUART = false;
    }
    if (bruceConfigPins.uart_bus.checkConflict(GPIO_NUM_NC)) {
        Serial.println("UART pins not configured");
        nrfUART = false;
    }
    if (nrfSPI && nrfUART) {
        // FIX: having both valid SPI pins AND valid UART pins is not an error.
        // A UART-NRF can't be reliably auto-detected from pin config, so when the
        // SPI pins are configured, default to SPI (the common case — e.g. the
        // M5Stick RF Pack S3 with an SPI nRF24). Previously this branch wrongly
        // raised "NRF24 pins undefined", blocking every SPI-only NRF24 device.
        mode = NRF_MODE_SPI;
    } else if (nrfSPI) {
        mode = NRF_MODE_SPI;
    } else if (nrfUART) {
        mode = NRF_MODE_UART;
    }
    return mode;
}
