#pragma once

#include "BBQ10Keyboard.h"
#include "InputBroker.h"
#include "MPR121Keyboard.h"
#include "Wire.h"
#include "concurrency/OSThread.h"

class TCA8418KeyboardBase;

class KbI2cBase : public Observable<const InputEvent *>, public concurrency::OSThread
{
  public:
    explicit KbI2cBase(const char *name);
    void toggleBacklight(bool on);

    // Regular T-Deck keyboard backlight auto mode (uses KB_BL_PIN)
    void setKbBacklight(bool on);
    void toggleKbBacklightAuto();

  protected:
    virtual int32_t runOnce() override;

  private:
    const char *_originName;

    TwoWire *i2cBus = 0;

    BBQ10Keyboard Q10keyboard;
    MPR121Keyboard MPRkeyboard;
    TCA8418KeyboardBase &TCAKeyboard;
    bool is_sym = false;

    // Regular T-Deck keyboard backlight (GPIO)
    bool kbBlInit = false;
    bool kbBlOn = false;
    bool kbBlAuto = false;
    uint32_t kbBlLastActivityMs = 0;
    uint32_t kbBlIgnoreUntilMs = 0; // ignore self-induced I2C artifacts
    static constexpr uint32_t kbBlAutoTimeoutMs = 10000;
};