#pragma once

//#include <string>
//#include <functional>
//#include <cmath>

namespace esphome {

/** Default setup priorities for components of different types.
 *
 * Components should return one of these setup priorities in get_setup_priority.
 */
namespace setup_priority {

/// For communication buses like i2c/spi
extern const float BUS;
/// For components that represent GPIO pins like PCF8573
extern const float IO;
/// For components that deal with hardware and are very important like GPIO switch
extern const float HARDWARE;
/// For components that import data from directly connected sensors like DHT.
extern const float DATA;
/// Alias for DATA (here for compatibility reasons)
extern const float HARDWARE_LATE;
/// For components that use data from sensors like displays
extern const float PROCESSOR;
extern const float BLUETOOTH;
extern const float AFTER_BLUETOOTH;
extern const float WIFI;
extern const float ETHERNET;
/// For components that should be initialized after WiFi and before API is connected.
extern const float BEFORE_CONNECTION;
/// For components that should be initialized after WiFi is connected.
extern const float AFTER_WIFI;
/// For components that should be initialized after a data connection (API/MQTT) is connected.
extern const float AFTER_CONNECTION;
/// For components that should be initialized at the very end of the setup process.
extern const float LATE;

}  // namespace setup_priority

class Component {
 public:
  /** Where the component's initialization should happen.
   *
   * Analogous to Arduino's setup(). This method is guaranteed to only be called once.
   * Defaults to doing nothing.
   */
  virtual void setup();

  /** This method will be called repeatedly.
   *
   * Analogous to Arduino's loop(). setup() is guaranteed to be called before this.
   * Defaults to doing nothing.
   */
  virtual void loop();

  virtual void dump_config();

  /** priority of setup(). higher -> executed earlier
   *
   * Defaults to 0.
   *
   * @return The setup priority of this component
   */
  virtual float get_setup_priority() const;
};
}  // namespace esphome