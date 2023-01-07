#define CATCH_CONFIG_MAIN

#include <cassert>
#include <cstdio>
#include "catch2/catch.hpp"
#define protected public
#include "esphome/components/pulse_meter/pulse_meter_sensor.h"
#include "esphome/core/log.h"

const bool HIGH = true;
const bool LOW = false;

void (*isr_callback)(void *);
void * call_back_arg;


bool pin_value;
uint32_t time_us;

uint32_t pulse_width_ms;
uint32_t total_pulses;

namespace esphome {

void esp_log_printf_(int level, const char *tag, int line, const char *format, ...) {
    	//(int level, const char *tag, int line, const char *format, va_list args) 
    va_list argList;

    printf("level: %d, ", level);
    printf("%s, ", tag);
    printf("line %d, ", line);
    
    va_start(argList, format);
    vprintf(format, argList);
    va_end(argList);
    printf("\n");
    	
    return;
}

bool ISRInternalGPIOPin::digital_read(){
	return pin_value;
}

ISRInternalGPIOPin InternalGPIOPin::to_isr() const {
	return ISRInternalGPIOPin();
}

void InternalGPIOPin::setup() {}
std::string InternalGPIOPin::dump_summary(){
	return std::string("pin summary\n");
}

void InternalGPIOPin::attach_interrupt(void (*func)(void *), void *arg, gpio::InterruptType type) const {
	isr_callback = func;
	call_back_arg = arg;
}

uint32_t micros() {
	return time_us;
}

float Component::get_setup_priority() const { 
	return setup_priority::DATA; 
}

void Component::setup() {}

void Component::loop() {}

void Component::dump_config() {}

namespace setup_priority {

const float DATA = 600.0f;

}  // namespace setup_priority

namespace sensor {
	void Sensor::publish_state(float val) {
    ESP_LOGD("PulseMeterSensor Unit Test", "Publishing state %.2f @ t = %u us", val, time_us);
    pulse_width_ms = (uint32_t)( 60000.0f/val);
		return;
	}
} // namespace sensor

namespace pulse_meter {

}  // namespace pulse_meter
}  // namespace esphome

using namespace esphome;
using namespace esphome::pulse_meter;

void step_time_us(uint32_t delta_t_us) {
  time_us += delta_t_us;
}

void step_time_ms(uint32_t delta_t_ms) {
  if (delta_t_ms > 4294967) {
    ESP_LOGW("PulseMeterSensor Unit Test", "%u ms is larger than maximum possible increment (%u us). Capping to %u", delta_t_ms, 0xffffffff, 4294967);
    delta_t_ms = 4294967;
  }
  time_us += delta_t_ms*1000;
}

void trigger_interrupt(const bool pin_val_) {
  pin_value = pin_val_;
  isr_callback(call_back_arg);
}

class TotalSensor : public sensor::Sensor {
  void publish_state(float val) override {
    ESP_LOGD("PulseMeterSensor Unit Test", "Publishing state %u @ t = %u us", (uint32_t)val, time_us);
    total_pulses = (uint32_t)val;
    return;
  }
};

PulseMeterSensor init_sensor(
                 const uint32_t filter_us,
                 const uint32_t timeout_us,
                 TotalSensor * const total_sensor,
                 const PulseMeterSensor::InternalFilterMode filter_mode = PulseMeterSensor::FILTER_PULSE,
                 const uint32_t init_time_us=0, 
                 const bool init_pin_value=0,
                 const uint32_t init_total_pulses=0,
                 const uint32_t init_pulse_width_ms=0){
  
  time_us = init_time_us;
  pin_value = init_pin_value;
  total_pulses = init_total_pulses;
  pulse_width_ms = init_pulse_width_ms;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(filter_mode);
  pulse_meter_sensor.set_total_sensor(total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();

  return pulse_meter_sensor;
}

// void probe_sensor_state(PulseMeterSensor const * const sensor){
  
//   pulse_width_us = sensor->pulse_width_us_;
//   total_pulses = sensor->total_pulses_;
  
//   ESP_LOGD("PulseMeterSensor Unit Test", "t=%u us, pulse_width=%u us, total_pulses=%u", time_us, pulse_width_us, total_pulses);
//   return;
// }

// Test Case 01: Verify that a pulse is registered only after the filter_us time has passed starting with a low value
// Test Case 02: Verify that a pulse is registered only after the filter_us time has passed starting wiht a high value
// Test Case 03: Verify that the correct pulse_width is registered in EDGE MODE
// Test Case 04: Verify that the correct pulse_width is registered in PULSE MODE
// Test Case 05: Verify that a disturbance in the low pulse is filtered out in PULSE MODE
// Test Case 06: Verify that a disturbance in the high pulse is filtered out in PULSE MODE
// Test Case 07: Verify that jitter a the rising edge of a pulse is handled correctly in PULSE MODE
// Test Case 08: Verify that jitter a the falling edge of a pulse is handled correctly in PULSE MODE
// Test Case 09: Verify that a phantom interrupt a the rising edge is handled correctly in PULSE MODE
// Test Case 10: Verify that a phantom interrupt a the falling edge is handled correctly in PULSE MODE
// Test Case TDB: Verify that ISR triggered in the loop function are handled correctly
// Test Case TDB: Verify that resume after timeout is correct
// Test Case TDB: Verify that results are independent of starting pin value
// Test Case XX: Verify that pulses are counted correctly before and after the first loop has triggered
// First EDGE registered at a time < filter_us (EDGE mode) . With and w/o triggering an ISR beforehand to set values of last_intr etc.
//


TEST_CASE("01: Verify that a pulse is registered only after the filter_us time has passed. Starting with a low pin value.") {
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();


  // First Pulse 
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us);
  trigger_interrupt(LOW);

  // Allow for a high interrupt and loop call to process
  step_time_us(filter_us);
  trigger_interrupt(HIGH);

  pulse_meter_sensor.loop();
  REQUIRE(total_pulses == 0);

  // Second Pulse
  step_time_us(1);
  trigger_interrupt(LOW);
  step_time_us(filter_us+1);
  
  trigger_interrupt(HIGH);
  step_time_us(filter_us+1);

  // Allow for a sufficient long low pulse and then a rising edge and loop call for processing
  trigger_interrupt(LOW);
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);

  pulse_meter_sensor.loop();
  REQUIRE(total_pulses == 1);
}

TEST_CASE("02: Verify that a pulse is registered only after the filter_us time has passed. Starting with a high pin value.") {
  time_us = 0;
  pin_value = 1;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  

  step_time_ms(pulse_interval_ms);
  trigger_interrupt(LOW);

  // First Pulse 
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us);
  trigger_interrupt(LOW);

  // Allow for a high interrupt and loop call to process
  step_time_us(filter_us);
  trigger_interrupt(HIGH);

  pulse_meter_sensor.loop();
  REQUIRE(total_pulses == 0);

  // Second Pulse
  step_time_us(1);
  trigger_interrupt(LOW);
  step_time_us(filter_us+1);
  
  trigger_interrupt(HIGH);
  step_time_us(filter_us+1);

  // Allow for a sufficient long low pulse and then a rising edge and loop call for processing
  trigger_interrupt(LOW);
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);

  pulse_meter_sensor.loop();
  REQUIRE(total_pulses == 1); 
}


TEST_CASE("03: Verify that the correct pulse_width is registered in EDGE MODE"){
  const uint32_t filter_us = 49999;
  const uint32_t timeout_us = 60000000;

  TotalSensor total_sensor = TotalSensor();
  PulseMeterSensor pulse_meter_sensor = init_sensor(filter_us, timeout_us, &total_sensor, PulseMeterSensor::FILTER_EDGE);
  
  // Settling time
  step_time_us(filter_us+1);
  
  // First pulse
  trigger_interrupt(HIGH);
  step_time_us(1);
  
  trigger_interrupt(LOW);
  step_time_us(filter_us);

  // Second Pulse (Allow for loop calls in between for processing)
  pulse_meter_sensor.loop();
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  CHECK(total_pulses == 2);
  CHECK(pulse_width_ms == (filter_us+1)/1000);
}

TEST_CASE("04: Verify that the correct pulse_width is registered in PULSE MODE"){
  const uint32_t filter_us = 49999;
  const uint32_t timeout_us = 60000000;

  TotalSensor total_sensor = TotalSensor();
  PulseMeterSensor pulse_meter_sensor = init_sensor(filter_us, timeout_us, &total_sensor);

  // Settling time
  step_time_us(filter_us+1);
  
  // Pulse 1
  trigger_interrupt(HIGH);
  step_time_us(filter_us+1);
  
  trigger_interrupt(LOW);
  step_time_us(filter_us+1);
  
  // Pulse 2
  trigger_interrupt(HIGH);
  step_time_us(filter_us+1);

  trigger_interrupt(LOW);
  step_time_us(filter_us+1);

  // Allow for two calls two loop with rising edge in between for processing
  pulse_meter_sensor.loop();
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  CHECK(total_pulses == 2);
  CHECK(pulse_width_ms == ((2*(filter_us+1))/1000));
}

TEST_CASE("05: Verify that a disturbance in the low pulse is filtered out in PULSE MODE"){
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;
  const uint32_t disturbance_pulse_length_us = 10;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  
  // First Pulse
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  // Disturbance
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(LOW);

  // Second Pulse
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  

  // Allow for a cycle to process the edge
  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();


  REQUIRE(total_pulses == 2);
  REQUIRE(pulse_width_ms == ((3*(filter_us+1)+disturbance_pulse_length_us)/1000));
}

TEST_CASE("06: Verify that a disturbance in the high pulse is filtered out in PULSE MODE"){
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;
  const uint32_t disturbance_pulse_length_us = 10;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  
  // First Pulse
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  // Second Pulse
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  // Disturbance
  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(HIGH);
  
  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 2);
  REQUIRE(pulse_width_ms == (2*(filter_us+1)/1000));
}

TEST_CASE("07: Verify that jitter a the rising edge of a pulse is handled correctly in PULSE MODE"){
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;
  const uint32_t disturbance_pulse_length_us = 40000;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  
  // First Pulse
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  // Second Pulse
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop(); // Make sure the sensor is initialized

  // Disturbance
  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(LOW);

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(HIGH);
  
  step_time_us(3*filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(3*filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 2);
  REQUIRE(pulse_width_ms == 2*(filter_us+disturbance_pulse_length_us+1)/1000);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 3);
  REQUIRE(pulse_width_ms == 2*(3*filter_us+1)/1000);
}

TEST_CASE("08: Verify that jitter a the falling edge of a pulse is handled correctly in PULSE MODE"){
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;
  const uint32_t disturbance_pulse_length_us = 40000;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  
  // First Pulse
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  // Second Pulse
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop(); // Make sure the sensor is initialized

  // Disturbance
  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(HIGH);

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(LOW);
  
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 2);
  REQUIRE(pulse_width_ms == 2*(filter_us+1)/1000);

  step_time_us(3*filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(3*filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 3);
  REQUIRE(pulse_width_ms == 2*(filter_us+disturbance_pulse_length_us+1)/1000);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 4);
  REQUIRE(pulse_width_ms == 2*(3*filter_us+1)/1000);
}

TEST_CASE("09: Verify that a phantom interrupt at the rising edge of a pulse is handled correctly in PULSE MODE"){
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;
  const uint32_t disturbance_pulse_length_us = 40000;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  
  // First Pulse
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  // Second Pulse
  step_time_us(filter_us+1);
  // trigger_interrupt(HIGH); Phantom interrupt, so this one is missed

  // Disturbance
  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(LOW); 

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop(); // Make sure the sensor is initialized
  
  step_time_us(3*filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(3*filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 2);
  REQUIRE(pulse_width_ms == 2*(filter_us+disturbance_pulse_length_us+1)/1000);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 3);
  REQUIRE(pulse_width_ms == 2*(3*filter_us+1)/1000);
}

TEST_CASE("10: Verify that a phantom interrupt at the falling edge of a pulse is handled correctly in PULSE MODE"){
  time_us = 0;
  pin_value = 0;
  total_pulses = 0;
  pulse_width_ms = 0;

  const uint32_t filter_us = 50000;
  const uint32_t timeout_us = 60000000;
  const uint32_t pulse_interval_ms = 100;
  const uint32_t disturbance_pulse_length_us = 40000;

  PulseMeterSensor pulse_meter_sensor = PulseMeterSensor();
  InternalGPIOPin pin = InternalGPIOPin();

  pulse_meter_sensor.set_timeout_us(timeout_us);    
  pulse_meter_sensor.set_filter_us(filter_us);  
  pulse_meter_sensor.set_pin(&pin);
  pulse_meter_sensor.set_filter_mode(PulseMeterSensor::FILTER_PULSE);
  
  TotalSensor total_sensor = TotalSensor();
  pulse_meter_sensor.set_total_sensor(&total_sensor);

  pulse_meter_sensor.dump_config();

  pulse_meter_sensor.setup();
  pulse_meter_sensor.loop();
  
  // First Pulse
  step_time_ms(pulse_interval_ms);
  trigger_interrupt(HIGH);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  // Second Pulse
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop(); // Make sure the sensor is initialized

  // Disturbance
  step_time_us(filter_us+1);
  // trigger_interrupt(LOW); Phantom interrupt, so this one is missed

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(HIGH);

  step_time_us(disturbance_pulse_length_us);
  trigger_interrupt(LOW);
  
  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 2);
  REQUIRE(pulse_width_ms == 2*(filter_us+1)/1000);

  step_time_us(3*filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(3*filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 3);
  REQUIRE(pulse_width_ms == 2*(filter_us+disturbance_pulse_length_us+1)/1000);

  step_time_us(filter_us+1);
  trigger_interrupt(LOW);

  step_time_us(filter_us+1);
  trigger_interrupt(HIGH);
  pulse_meter_sensor.loop();

  REQUIRE(total_pulses == 4);
  REQUIRE(pulse_width_ms == 2*(3*filter_us+1)/1000);
}
