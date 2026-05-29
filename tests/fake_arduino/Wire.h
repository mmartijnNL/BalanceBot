#pragma once

#include <cstdint>

class TwoWire {
   public:
    explicit TwoWire(uint8_t index = 0) : index_(index) {}

    void setSDA(uint8_t pin) { sda_pin_ = pin; }
    void setSCL(uint8_t pin) { scl_pin_ = pin; }
    void begin() { begun_ = true; }

    uint8_t index() const { return index_; }
    bool begun() const { return begun_; }
    uint8_t sda_pin() const { return sda_pin_; }
    uint8_t scl_pin() const { return scl_pin_; }

   private:
    uint8_t index_ = 0;
    bool begun_ = false;
    uint8_t sda_pin_ = 0;
    uint8_t scl_pin_ = 0;
};

extern TwoWire Wire;
extern TwoWire Wire1;

// Expose second bus macros so sketch code can select Wire1 in tests.
#define PIN_WIRE1_SDA 6
#define PIN_WIRE1_SCL 7
