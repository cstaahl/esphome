#pragma once
#include <cstdint>
#include <string>

namespace esphome {

#define LOG_PIN(prefix, pin) \
  if ((pin) != nullptr) { \
    ESP_LOGCONFIG(TAG, prefix "%s", (pin)->dump_summary().c_str()); \
  }

// put GPIO flags in a namespace to not pollute esphome namespace
namespace gpio {

enum Flags : uint8_t {
  // Can't name these just INPUT because of Arduino defines :(
  FLAG_NONE = 0x00,
  FLAG_INPUT = 0x01,
  FLAG_OUTPUT = 0x02,
  FLAG_OPEN_DRAIN = 0x04,
  FLAG_PULLUP = 0x08,
  FLAG_PULLDOWN = 0x10,
};

class FlagsHelper {
 public:
  constexpr FlagsHelper(Flags val) : val_(val) {}
  constexpr operator Flags() const { return val_; }

 protected:
  Flags val_;
};
constexpr FlagsHelper operator&(Flags lhs, Flags rhs) {
  return static_cast<Flags>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}
constexpr FlagsHelper operator|(Flags lhs, Flags rhs) {
  return static_cast<Flags>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

enum InterruptType : uint8_t {
  INTERRUPT_RISING_EDGE = 1,
  INTERRUPT_FALLING_EDGE = 2,
  INTERRUPT_ANY_EDGE = 3,
  INTERRUPT_LOW_LEVEL = 4,
  INTERRUPT_HIGH_LEVEL = 5,
};

}  // namespace gpio


/// Copy of GPIOPin that is safe to use from ISRs (with no virtual functions)
class ISRInternalGPIOPin {
 public:
  //ISRInternalGPIOPin() = default;
  //ISRInternalGPIOPin(void *arg) : arg_(arg) {}
  bool digital_read();
  void digital_write(bool value);
  void clear_interrupt();
  void pin_mode(gpio::Flags flags);

 protected:
  void *arg_{nullptr};
};

class InternalGPIOPin {
 public:
  template<typename T> void attach_interrupt(void (*func)(T *), T *arg, gpio::InterruptType type) const {
    this->attach_interrupt(reinterpret_cast<void (*)(void *)>(func), arg, type);
  }

  ISRInternalGPIOPin to_isr() const;
  void setup();
 std::string dump_summary();

 protected:
  void attach_interrupt(void (*func)(void *), void *arg, gpio::InterruptType type) const;
};

}  // namespace esphome