#include "kbI2cBase.h"
#include "configuration.h"
#include "detect/ScanI2C.h"
#include "detect/ScanI2CTwoWire.h"

#if defined(T_DECK_PRO)
#include "TDeckProKeyboard.h"
#elif defined(T_LORA_PAGER)
#include "TLoraPagerKeyboard.h"
#elif defined(HACKADAY_COMMUNICATOR)
#include "HackadayCommunicatorKeyboard.h"
#else
#include "TCA8418Keyboard.h"
#endif

extern ScanI2C::DeviceAddress cardkb_found;
extern uint8_t kb_model;

KbI2cBase::KbI2cBase(const char *name)
    : concurrency::OSThread(name),
#if defined(T_DECK_PRO)
      TCAKeyboard(*(new TDeckProKeyboard()))
#elif defined(T_LORA_PAGER)
      TCAKeyboard(*(new TLoraPagerKeyboard()))
#elif defined(HACKADAY_COMMUNICATOR)
      TCAKeyboard(*(new HackadayCommunicatorKeyboard()))
#else
      TCAKeyboard(*(new TCA8418Keyboard()))
#endif
{
    this->_originName = name;
}

uint8_t read_from_14004(TwoWire *i2cBus, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t readflag = 0;
    i2cBus->beginTransmission(CARDKB_ADDR);
    i2cBus->write(reg);
    i2cBus->endTransmission(); // stop transmitting
    delay(20);
    i2cBus->requestFrom(CARDKB_ADDR, (int)length);
    int i = 0;
    while (i2cBus->available()) // slave may send less than requested
    {
        data[i++] = i2cBus->read(); // receive a byte as a proper uint8_t
        readflag = 1;
    }
    return readflag;
}

void KbI2cBase::setKbBacklight(bool on)
{
    // Regular T-Deck: keyboard is an I2C device at TDECK_KB_ADDR (0x55).
    // The backlight appears to be handled by the keyboard MCU, so we attempt a best-effort I2C command.
    // If the command is unsupported, it will simply be ignored.
    if (kb_model == 0x10 && i2cBus && cardkb_found.address == TDECK_KB_ADDR) {
        // Empirical attempt: write single byte 0x01 (on) / 0x00 (off)
        i2cBus->beginTransmission((int)cardkb_found.address);
        i2cBus->write(on ? 0x01 : 0x00);
        i2cBus->endTransmission();
        kbBlOn = on;
        // Some T-Deck keyboards seem to echo state/bytes back into the read stream.
        // Briefly ignore reads so auto mode doesn't immediately re-trigger.
        kbBlIgnoreUntilMs = millis() + 1000;
        return;
    }

#if defined(KB_BL_PIN)
    // Other devices (or custom wiring) might expose a GPIO backlight pin.
    if (!kbBlInit) {
        pinMode(KB_BL_PIN, OUTPUT);
        kbBlInit = true;
    }
    digitalWrite(KB_BL_PIN, on ? HIGH : LOW);
    kbBlOn = on;
#else
    (void)on;
#endif
}

void KbI2cBase::toggleKbBacklightAuto()
{
    kbBlAuto = !kbBlAuto;
    kbBlLastOffAttemptMs = 0;
    if (kbBlAuto) {
        kbBlLastActivityMs = millis();
        setKbBacklight(true);
    } else {
        kbBlLastActivityMs = 0;
        setKbBacklight(false);
    }
}

int32_t KbI2cBase::runOnce()
{
    if (!i2cBus) {
        switch (cardkb_found.port) {
        case ScanI2C::WIRE1:
#if WIRE_INTERFACES_COUNT == 2
            LOG_DEBUG("Use I2C Bus 1 (the second one)");
            i2cBus = &Wire1;
            if (cardkb_found.address == BBQ10_KB_ADDR) {
                Q10keyboard.begin(BBQ10_KB_ADDR, &Wire1);
                Q10keyboard.setBacklight(0);
            }
            if (cardkb_found.address == MPR121_KB_ADDR) {
                MPRkeyboard.begin(MPR121_KB_ADDR, &Wire1);
            }
            if (cardkb_found.address == TCA8418_KB_ADDR) {
                TCAKeyboard.begin(TCA8418_KB_ADDR, &Wire1);
            }
            break;
#endif
        case ScanI2C::WIRE:
            LOG_DEBUG("Use I2C Bus 0 (the first one)");
            i2cBus = &Wire;
            if (cardkb_found.address == BBQ10_KB_ADDR) {
                Q10keyboard.begin(BBQ10_KB_ADDR, &Wire);
                Q10keyboard.setBacklight(0);
            }
            if (cardkb_found.address == MPR121_KB_ADDR) {
                MPRkeyboard.begin(MPR121_KB_ADDR, &Wire);
            }
            if (cardkb_found.address == TCA8418_KB_ADDR) {
                TCAKeyboard.begin(TCA8418_KB_ADDR, &Wire);
            }
            break;
        case ScanI2C::NO_I2C:
        default:
            i2cBus = 0;
        }
    }

    // Regular T-Deck keyboard backlight auto timeout
    if (kbBlAuto && kbBlOn) {
        uint32_t now = millis();
        if (kbBlLastActivityMs != 0 && (now - kbBlLastActivityMs) > kbBlAutoTimeoutMs) {
            // Rate limit OFF attempts. If the keyboard MCU ignores our I2C command,
            // repeatedly sending it can cause visible flicker.
            if (kbBlLastOffAttemptMs == 0 || (now - kbBlLastOffAttemptMs) > kbBlOffRetryCooldownMs) {
                kbBlLastOffAttemptMs = now;
                setKbBacklight(false);
                // Treat as "off" from firmware perspective to avoid spamming OFF every loop.
                kbBlOn = false;
            }
        }
    }

    switch (kb_model) {
    case 0x11: { // BB Q10
        int keyCount = Q10keyboard.keyCount();
        while (keyCount--) {
            const BBQ10Keyboard::KeyEvent key = Q10keyboard.keyEvent();
            if ((key.key != 0x00) && (key.state == BBQ10Keyboard::StateRelease)) {
                InputEvent e = {};
                e.inputEvent = INPUT_BROKER_NONE;
                e.source = this->_originName;
                switch (key.key) {
                case 'p': // TAB
                case 't': // TAB as well
                    if (is_sym) {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = 0x09; // TAB Scancode
                        is_sym = false;  // reset sym state after second keypress
                    } else {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = key.key;
                    }
                    break;
                case 'q': // ESC
                    if (is_sym) {
                        e.inputEvent = INPUT_BROKER_CANCEL;
                        e.kbchar = 0;
                        is_sym = false; // reset sym state after second keypress
                    } else {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = key.key;
                    }
                    break;
                case 0x08: // Back
                    e.inputEvent = INPUT_BROKER_BACK;
                    e.kbchar = key.key;
                    break;
                case 'e': // sym e
                    if (is_sym) {
                        e.inputEvent = INPUT_BROKER_UP;
                        e.kbchar = INPUT_BROKER_UP;
                        is_sym = false; // reset sym state after second keypress
                    } else {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = key.key;
                    }
                    break;
                case 'x': // sym x
                    if (is_sym) {
                        e.inputEvent = INPUT_BROKER_DOWN;
                        e.kbchar = 0;
                        is_sym = false; // reset sym state after second keypress
                    } else {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = key.key;
                    }
                    break;
                case 's': // sym s
                    if (is_sym) {
                        e.inputEvent = INPUT_BROKER_LEFT;
                        e.kbchar = 0x00; // tweak for destSelect
                        is_sym = false;  // reset sym state after second keypress
                    } else {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = key.key;
                    }
                    break;
                case 'f': // sym f
                    if (is_sym) {
                        e.inputEvent = INPUT_BROKER_RIGHT;
                        e.kbchar = 0x00; // tweak for destSelect
                        is_sym = false;  // reset sym state after second keypress
                    } else {
                        e.inputEvent = INPUT_BROKER_ANYKEY;
                        e.kbchar = key.key;
                    }
                    break;
                case 0x13: // Code scanner says the SYM key is 0x13
                    is_sym = !is_sym;
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = is_sym ? INPUT_BROKER_MSG_FN_SYMBOL_ON   // send 0xf1 to tell CannedMessages to display that
                                      : INPUT_BROKER_MSG_FN_SYMBOL_OFF; // the modifier key is active
                    break;
                case 0x0a: // apparently Enter on Q10 is a line feed instead of carriage return
                    e.inputEvent = INPUT_BROKER_SELECT;
                    break;
                case 0x00: // nopress
                    e.inputEvent = INPUT_BROKER_NONE;
                    break;
                default: // all other keys
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = key.key;
                    is_sym = false; // reset sym state after second keypress
                    break;
                }

                if (e.inputEvent != INPUT_BROKER_NONE) {
                    this->notifyObservers(&e);
                }
            }
        }
        break;
    }
    case 0x37: { // MPR121
        MPRkeyboard.trigger();
        InputEvent e = {};

        while (MPRkeyboard.hasEvent()) {
            char nextEvent = MPRkeyboard.dequeueEvent();
            e.inputEvent = INPUT_BROKER_ANYKEY;
            e.kbchar = 0x00;
            e.source = this->_originName;
            switch (nextEvent) {
            case 0x00: // MPR121_NONE
                e.inputEvent = INPUT_BROKER_NONE;
                e.kbchar = 0x00;
                break;
            case 0x90: // MPR121_REBOOT
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_MSG_REBOOT;
                break;
            case 0xb4: // MPR121_LEFT
                e.inputEvent = INPUT_BROKER_LEFT;
                e.kbchar = 0x00;
                break;
            case 0xb5: // MPR121_UP
                e.inputEvent = INPUT_BROKER_UP;
                e.kbchar = 0x00;
                break;
            case 0xb6: // MPR121_DOWN
                e.inputEvent = INPUT_BROKER_DOWN;
                e.kbchar = 0x00;
                break;
            case 0xb7: // MPR121_RIGHT
                e.inputEvent = INPUT_BROKER_RIGHT;
                e.kbchar = 0x00;
                break;
            case 0x1b: // MPR121_ESC
                e.inputEvent = INPUT_BROKER_CANCEL;
                e.kbchar = 0;
                break;
            case 0x08: // MPR121_BSP
                e.inputEvent = INPUT_BROKER_BACK;
                e.kbchar = 0x08;
                break;
            case 0x0d: // MPR121_SELECT
                e.inputEvent = INPUT_BROKER_SELECT;
                e.kbchar = 0x00;
                break;
            default:
                if (nextEvent > 127) {
                    e.inputEvent = INPUT_BROKER_NONE;
                    e.kbchar = 0x00;
                    break;
                }
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = nextEvent;
                break;
            }
            if (e.inputEvent != INPUT_BROKER_NONE) {
                LOG_DEBUG("MP121 Notifying: %i Char: %i", e.inputEvent, e.kbchar);
                this->notifyObservers(&e);
            }
        }
        break;
    }
    case 0x84: { // Adafruit TCA8418
        TCAKeyboard.trigger();
        InputEvent e = {};
        while (TCAKeyboard.hasEvent()) {
            char nextEvent = TCAKeyboard.dequeueEvent();
            e.inputEvent = INPUT_BROKER_ANYKEY;
            e.kbchar = 0x00;
            e.source = this->_originName;
            switch (nextEvent) {
            case TCA8418KeyboardBase::NONE:
                e.inputEvent = INPUT_BROKER_NONE;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::REBOOT:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_MSG_REBOOT;
                break;
            case TCA8418KeyboardBase::LEFT:
                e.inputEvent = INPUT_BROKER_LEFT;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::UP:
                e.inputEvent = INPUT_BROKER_UP;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::DOWN:
                e.inputEvent = INPUT_BROKER_DOWN;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::RIGHT:
                e.inputEvent = INPUT_BROKER_RIGHT;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::BSP:
                e.inputEvent = INPUT_BROKER_BACK;
                e.kbchar = 0x08;
                break;
            case TCA8418KeyboardBase::SELECT:
                e.inputEvent = INPUT_BROKER_SELECT;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::ESC:
                e.inputEvent = INPUT_BROKER_CANCEL;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::GPS_TOGGLE:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_GPS_TOGGLE;
                break;
            case TCA8418KeyboardBase::SEND_PING:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_SEND_PING;
                break;
            case TCA8418KeyboardBase::MUTE_TOGGLE:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_MSG_MUTE_TOGGLE;
                break;
            case TCA8418KeyboardBase::BT_TOGGLE:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_MSG_BLUETOOTH_TOGGLE;
                break;
            case TCA8418KeyboardBase::BL_TOGGLE:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_MSG_BLUETOOTH_TOGGLE;
                break;
            case TCA8418KeyboardBase::TAB:
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = INPUT_BROKER_MSG_TAB;
                break;
            case TCA8418KeyboardBase::FUNCTION_F1:
                e.inputEvent = INPUT_BROKER_FN_F1;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::FUNCTION_F2:
                e.inputEvent = INPUT_BROKER_FN_F2;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::FUNCTION_F3:
                e.inputEvent = INPUT_BROKER_FN_F3;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::FUNCTION_F4:
                e.inputEvent = INPUT_BROKER_FN_F4;
                e.kbchar = 0x00;
                break;
            case TCA8418KeyboardBase::FUNCTION_F5:
                e.inputEvent = INPUT_BROKER_FN_F5;
                e.kbchar = 0x00;
                break;
            default:
                if (nextEvent > 127) {
                    e.inputEvent = INPUT_BROKER_NONE;
                    e.kbchar = 0x00;
                    break;
                }
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = nextEvent;
                break;
            }
            if (e.inputEvent != INPUT_BROKER_NONE) {
                // LOG_DEBUG("TCA8418 Notifying: %i Char: %c", e.inputEvent, e.kbchar);
                this->notifyObservers(&e);
            }
            TCAKeyboard.trigger();
        }
        TCAKeyboard.clearInt();
        break;
    }
    case 0x02: {
        // RAK14004
        uint8_t rDataBuf[8] = {0};
        uint8_t PrintDataBuf = 0;
        if (read_from_14004(i2cBus, 0x01, rDataBuf, 0x04) == 1) {
            for (uint8_t aCount = 0; aCount < 0x04; aCount++) {
                for (uint8_t bCount = 0; bCount < 0x04; bCount++) {
                    if (((rDataBuf[aCount] >> bCount) & 0x01) == 0x01) {
                        PrintDataBuf = aCount * 0x04 + bCount + 1;
                    }
                }
            }
        }
        if (PrintDataBuf != 0) {
            LOG_DEBUG("RAK14004 key 0x%x pressed", PrintDataBuf);
            InputEvent e = {};
            e.inputEvent = INPUT_BROKER_MATRIXKEY;
            e.source = this->_originName;
            e.kbchar = PrintDataBuf;
            this->notifyObservers(&e);
        }
        break;
    }
    case 0x00:   // CARDKB
    case 0x10: { // T-DECK

        i2cBus->requestFrom((int)cardkb_found.address, 1);

        if (i2cBus->available()) {
            char c = i2cBus->read();

            // DEBUG: log raw keycodes for T-Deck keyboard combos (helps diagnose ALT+? behavior)
            // Log only for interesting keys to avoid spamming.
            if ((uint8_t)c == 0x0c || (uint8_t)c == 0xAA || (uint8_t)c == 'v' || (uint8_t)c == 'b') {
                LOG_DEBUG("TDECKKB key=0x%02X ('%c') is_sym=%d", (uint8_t)c, (c >= 32 && c <= 126) ? c : '.', is_sym);
            }

            InputEvent e = {};
            e.inputEvent = INPUT_BROKER_NONE;
            e.source = this->_originName;
            switch (c) {
            case 0x71: // This is the button q. If modifier and q pressed, it cancels the input
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_CANCEL;
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x74: // letter t. if modifier and t pressed call 'tab'
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = 0x09; // TAB Scancode
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x76: // letter v. Modifier makes it toggle KB backlight auto mode
                if (is_sym) {
                    is_sym = false;
                    toggleKbBacklightAuto();
                    e.inputEvent = INPUT_BROKER_NONE; // consume
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x6d: // letter m. Modifier makes it mute notifications
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = INPUT_BROKER_MSG_MUTE_TOGGLE; // mute notifications
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x6f: // letter o(+). Modifier makes screen increase in brightness
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = INPUT_BROKER_MSG_BRIGHTNESS_UP; // Increase Brightness code
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x69: // letter i(-).  Modifier makes screen decrease in brightness
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = INPUT_BROKER_MSG_BRIGHTNESS_DOWN; // Decrease Brightness code
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x20: // Space. Send network ping like double press does
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = INPUT_BROKER_SEND_PING; // (fn + space)
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x67: // letter g. toggle gps
                if (is_sym) {
                    is_sym = false;
                    e.inputEvent = INPUT_BROKER_GPS_TOGGLE;
                    e.kbchar = INPUT_BROKER_GPS_TOGGLE;
                } else {
                    e.inputEvent = INPUT_BROKER_ANYKEY;
                    e.kbchar = c;
                }
                break;
            case 0x1b: // ESC
                e.inputEvent = INPUT_BROKER_CANCEL;
                break;
            case 0x08: // Back
                e.inputEvent = INPUT_BROKER_BACK;
                e.kbchar = 0;
                break;
            case 0xb5: // Up
                e.inputEvent = INPUT_BROKER_UP;
                e.kbchar = 0;
                break;
            case 0xb6: // Down
                e.inputEvent = INPUT_BROKER_DOWN;
                e.kbchar = 0;
                break;
            case 0xb4: // Left
                e.inputEvent = INPUT_BROKER_LEFT;
                e.kbchar = 0;
                break;
            case 0xb7: // Right
                e.inputEvent = INPUT_BROKER_RIGHT;
                e.kbchar = 0;
                break;
            case 0xc: // Modifier key: 0xc is alt+c (Other options could be: 0xea = shift+mic button or 0x4 shift+$(speaker))
                // toggle moddifiers button.
                is_sym = !is_sym;
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = is_sym ? INPUT_BROKER_MSG_FN_SYMBOL_ON   // send 0xf1 to tell CannedMessages to display that the
                                  : INPUT_BROKER_MSG_FN_SYMBOL_OFF; // modifier key is active
                break;
            case 0x9e: // fn+g      INPUT_BROKER_GPS_TOGGLE
                e.inputEvent = INPUT_BROKER_GPS_TOGGLE;
                e.kbchar = c;
                break;
            case 0xaf: // fn+space  INPUT_BROKER_SEND_PING
                e.inputEvent = INPUT_BROKER_SEND_PING;
                e.kbchar = c;
                break;
            case 0x9b: // fn+s      INPUT_BROKER_MSG_SHUTDOWN
                e.inputEvent = INPUT_BROKER_SHUTDOWN;
                e.kbchar = c;
                break;

            case 0x90: // fn+r      INPUT_BROKER_MSG_REBOOT
            case 0x91: // fn+t
            case 0xac: // fn+m      INPUT_BROKER_MSG_MUTE_TOGGLE
            case 0xAA: // fn+b      INPUT_BROKER_MSG_BLUETOOTH_TOGGLE
            case 0x8F: // fn+e      INPUT_BROKER_MSG_EMOTE_LIST
                // just pass those unmodified
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = c;
                break;
            case 0x0d: // Enter
                e.inputEvent = INPUT_BROKER_SELECT;
                break;
            case 0x00: // nopress
                e.inputEvent = INPUT_BROKER_NONE;
                break;
            default:           // all other keys
                if (c > 127) { // bogus key value
                    e.inputEvent = INPUT_BROKER_NONE;
                    break;
                }
                e.inputEvent = INPUT_BROKER_ANYKEY;
                e.kbchar = c;
                is_sym = false;
                break;
            }

            if (e.inputEvent != INPUT_BROKER_NONE) {
                // Auto mode: only treat meaningful user keys as activity.
                // Also ignore a short window after we send backlight I2C commands.
                if (kbBlAuto) {
                    uint32_t now = millis();
                    if (now >= kbBlIgnoreUntilMs) {
                        // Prefer decoded kbchar, but for nav keys kbchar is 0 so use raw byte.
                        uint8_t raw = (uint8_t)c;
                        uint8_t activityCode = e.kbchar ? (uint8_t)e.kbchar : raw;

                        bool meaningful = false;
                        if (activityCode >= 32 && activityCode <= 126) {
                            meaningful = true; // printable typing
                        } else if (activityCode == 0x0d || activityCode == 0x08 || activityCode == 0x1b) {
                            meaningful = true; // enter/back/esc
                        } else if (activityCode == 0xb4 || activityCode == 0xb5 || activityCode == 0xb6 || activityCode == 0xb7) {
                            meaningful = true; // arrows
                        }

                        if (meaningful) {
                            kbBlLastActivityMs = now;
                            if (!kbBlOn)
                                setKbBacklight(true);
                        }
                    }
                }

                this->notifyObservers(&e);
            }
        }
        break;
    }
    default:
        LOG_WARN("Unknown kb_model 0x%02x", kb_model);
    }
    return 300;
}

void KbI2cBase::toggleBacklight(bool on)
{
#if defined(T_LORA_PAGER)
    TCAKeyboard.setBacklight(on);
#endif
}
