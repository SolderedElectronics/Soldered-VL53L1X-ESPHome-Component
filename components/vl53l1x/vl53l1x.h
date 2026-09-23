/**
 * @file vl53l1x.h
 * @brief Public API for the vl53l1x ESPHome component
 * @author Soldered Electronics
 *
 * The ranging logic is ported from the Pololu VL53L1X Arduino library (used by the Soldered VL53L1X Arduino library),
 * which is in turn based on the VL53L1X API from ST (STSW-IMG007). See LICENSE.txt in this directory.
 */

#pragma once

#include <vector>

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace vl53l1x {

/// Distance mode, trades maximum range against immunity to ambient light (see the VL53L1X datasheet).
enum DistanceMode : uint8_t {
  DISTANCE_MODE_SHORT = 0,   ///< up to ~1.3 m, best ambient light immunity
  DISTANCE_MODE_MEDIUM = 1,  ///< up to ~3 m
  DISTANCE_MODE_LONG = 2,    ///< up to ~4 m in the dark
};

/// Range status of a measurement, same values as VL53L1X::RangeStatus in the Pololu library.
enum RangeStatus : uint8_t {
  RANGE_STATUS_VALID = 0,
  RANGE_STATUS_SIGMA_FAIL = 1,
  RANGE_STATUS_SIGNAL_FAIL = 2,
  RANGE_STATUS_VALID_MIN_RANGE_CLIPPED = 3,
  RANGE_STATUS_OUT_OF_BOUNDS_FAIL = 4,
  RANGE_STATUS_HARDWARE_FAIL = 5,
  RANGE_STATUS_VALID_NO_WRAP_CHECK_FAIL = 6,
  RANGE_STATUS_WRAP_TARGET_FAIL = 7,
  RANGE_STATUS_XTALK_SIGNAL_FAIL = 9,
  RANGE_STATUS_SYNCHRONIZATION_INT = 10,
  RANGE_STATUS_MIN_RANGE_FAIL = 13,
  RANGE_STATUS_NONE = 255,
};

/// Human readable name of a RangeStatus, same strings as VL53L1X::rangeStatusToString().
const char *range_status_to_string(RangeStatus status);

/**
 * @brief VL53L1X time-of-flight distance sensor.
 *
 * The sensor ranges continuously on its own, with the inter-measurement period equal to the timing budget. On every
 * update() the latched result is discarded and the next fresh measurement is read from loop() as soon as the sensor
 * reports it (at most one timing budget later), so a published value is never older than one measurement, whatever
 * the update interval.
 *
 * Several sensors can share one I2C bus if each has its XSHUT pin wired to a GPIO (enable_pin): on boot all of them
 * are held in reset, then brought up one by one at the default address 0x29 and moved to their configured address.
 */
class VL53L1XComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  VL53L1XComponent();

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_enable_pin(GPIOPin *enable_pin) { this->enable_pin_ = enable_pin; }
  void set_distance_mode(DistanceMode distance_mode) { this->distance_mode_ = distance_mode; }
  void set_timing_budget_us(uint32_t timing_budget_us) { this->timing_budget_us_ = timing_budget_us; }
  void set_timeout_ms(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
  /// Region of interest size in SPADs, 4 - 16 each. 16 x 16 (the default) is the full ~27 degree field of view.
  void set_roi_size(uint8_t width, uint8_t height) {
    this->roi_width_ = width;
    this->roi_height_ = height;
  }
  /// Center SPAD of the region of interest, see the SPAD table in UM2555. 199 is the optical center.
  void set_roi_center(uint8_t roi_center) { this->roi_center_ = roi_center; }

#ifdef USE_SENSOR
  void set_distance_sensor(sensor::Sensor *sensor) { this->distance_sensor_ = sensor; }
  void set_signal_rate_sensor(sensor::Sensor *sensor) { this->signal_rate_sensor_ = sensor; }
  void set_ambient_rate_sensor(sensor::Sensor *sensor) { this->ambient_rate_sensor_ = sensor; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_range_status_text_sensor(text_sensor::TextSensor *sensor) { this->range_status_text_sensor_ = sensor; }
#endif

 protected:
  /// Raw result block read from RESULT__RANGE_STATUS (0x0089) onwards, only the fields the ranging math needs.
  struct ResultBuffer {
    uint8_t range_status;
    uint8_t stream_count;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
  };

  enum SetupError : uint8_t {
    SETUP_OK = 0,
    SETUP_WRONG_MODEL_ID,
    SETUP_BOOT_TIMEOUT,
    SETUP_COMMUNICATION_FAILED,
    SETUP_TIMING_BUDGET_REJECTED,
  };

  // Register access. Errors are sticky in i2c_failed_ so a multi-register sequence can be checked once at the end.
  void write_reg_(uint16_t reg, uint8_t value);
  void write_reg16_(uint16_t reg, uint16_t value);
  void write_reg32_(uint16_t reg, uint32_t value);
  uint8_t read_reg_(uint16_t reg);
  uint16_t read_reg16_(uint16_t reg);

  SetupError init_sensor_();
  void apply_distance_mode_(DistanceMode mode);
  bool apply_timing_budget_(uint32_t budget_us);
  uint32_t read_timing_budget_();
  void start_continuous_(uint32_t period_ms);
  bool data_ready_(bool &ready);
  bool read_measurement_();
  void setup_manual_calibration_();
  void update_dss_();
  void publish_measurement_();
  void publish_failure_();

  uint32_t calc_macro_period_(uint8_t vcsel_period) const;
  static uint32_t decode_timeout_(uint16_t reg_val);
  static uint16_t encode_timeout_(uint32_t timeout_mclks);
  static uint32_t timeout_mclks_to_microseconds_(uint32_t timeout_mclks, uint32_t macro_period_us);
  static uint32_t timeout_microseconds_to_mclks_(uint32_t timeout_us, uint32_t macro_period_us);

  GPIOPin *enable_pin_{nullptr};
  DistanceMode distance_mode_{DISTANCE_MODE_LONG};
  uint32_t timing_budget_us_{50000};
  uint32_t timeout_ms_{500};
  uint8_t roi_width_{16};
  uint8_t roi_height_{16};
  int16_t roi_center_{-1};

#ifdef USE_SENSOR
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *signal_rate_sensor_{nullptr};
  sensor::Sensor *ambient_rate_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *range_status_text_sensor_{nullptr};
#endif

  bool i2c_failed_{false};
  SetupError setup_error_{SETUP_OK};

  uint16_t fast_osc_frequency_{0};
  uint16_t osc_calibrate_val_{0};
  bool calibrated_{false};
  uint8_t saved_vhv_init_{0};
  uint8_t saved_vhv_timeout_{0};

  ResultBuffer results_{};
  uint16_t range_mm_{0};
  RangeStatus range_status_{RANGE_STATUS_NONE};
  float peak_signal_count_rate_mcps_{0.0f};
  float ambient_count_rate_mcps_{0.0f};

  bool waiting_for_data_{false};
  uint32_t wait_start_ms_{0};

  static std::vector<VL53L1XComponent *> vl53l1x_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
  static bool enable_pin_setup_complete;                   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
};

}  // namespace vl53l1x
}  // namespace esphome
