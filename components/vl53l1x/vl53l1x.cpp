/**
 * @file vl53l1x.cpp
 * @brief VL53L1X time-of-flight distance sensor ESPHome component
 * @author Soldered Electronics
 *
 * The register sequences and ranging math below are ported from the Pololu VL53L1X Arduino library (used by the
 * Soldered VL53L1X Arduino library), which is in turn based on the VL53L1X API from ST (STSW-IMG007). Only the I2C
 * layer (Wire -> i2c::I2CDevice) and the non-blocking read flow are new. See LICENSE.txt in this directory.
 */

#include "vl53l1x.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace vl53l1x {

static const char *const TAG = "vl53l1x";

// Register addresses, from vl53l1x_register_map.h in the ST API (via the Pololu library)
static const uint16_t SOFT_RESET = 0x0000;
static const uint16_t I2C_SLAVE__DEVICE_ADDRESS = 0x0001;
static const uint16_t OSC_MEASURED__FAST_OSC__FREQUENCY = 0x0006;
static const uint16_t VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
static const uint16_t VHV_CONFIG__INIT = 0x000B;
static const uint16_t ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E;
static const uint16_t MM_CONFIG__OUTER_OFFSET_MM = 0x0022;
static const uint16_t DSS_CONFIG__TARGET_TOTAL_RATE_MCPS = 0x0024;
static const uint16_t PAD_I2C_HV__EXTSUP_CONFIG = 0x002E;
static const uint16_t GPIO__TIO_HV_STATUS = 0x0031;
static const uint16_t SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS = 0x0036;
static const uint16_t SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS = 0x0037;
static const uint16_t ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM = 0x0039;
static const uint16_t ALGO__RANGE_IGNORE_VALID_HEIGHT_MM = 0x003E;
static const uint16_t ALGO__RANGE_MIN_CLIP = 0x003F;
static const uint16_t ALGO__CONSISTENCY_CHECK__TOLERANCE = 0x0040;
static const uint16_t CAL_CONFIG__VCSEL_START = 0x0047;
static const uint16_t PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B;
static const uint16_t PHASECAL_CONFIG__OVERRIDE = 0x004D;
static const uint16_t DSS_CONFIG__ROI_MODE_CONTROL = 0x004F;
static const uint16_t SYSTEM__THRESH_RATE_HIGH = 0x0050;
static const uint16_t SYSTEM__THRESH_RATE_LOW = 0x0052;
static const uint16_t DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT = 0x0054;
static const uint16_t DSS_CONFIG__APERTURE_ATTENUATION = 0x0057;
static const uint16_t MM_CONFIG__TIMEOUT_MACROP_A = 0x005A;
static const uint16_t MM_CONFIG__TIMEOUT_MACROP_B = 0x005C;
static const uint16_t RANGE_CONFIG__TIMEOUT_MACROP_A = 0x005E;
static const uint16_t RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060;
static const uint16_t RANGE_CONFIG__TIMEOUT_MACROP_B = 0x0061;
static const uint16_t RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063;
static const uint16_t RANGE_CONFIG__SIGMA_THRESH = 0x0064;
static const uint16_t RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
static const uint16_t RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069;
static const uint16_t SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C;
static const uint16_t SYSTEM__GROUPED_PARAMETER_HOLD_0 = 0x0071;
static const uint16_t SYSTEM__SEED_CONFIG = 0x0077;
static const uint16_t SD_CONFIG__WOI_SD0 = 0x0078;
static const uint16_t SD_CONFIG__WOI_SD1 = 0x0079;
static const uint16_t SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A;
static const uint16_t SD_CONFIG__INITIAL_PHASE_SD1 = 0x007B;
static const uint16_t SYSTEM__GROUPED_PARAMETER_HOLD_1 = 0x007C;
static const uint16_t SD_CONFIG__QUANTIFIER = 0x007E;
static const uint16_t ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x007F;
static const uint16_t ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080;
static const uint16_t SYSTEM__SEQUENCE_CONFIG = 0x0081;
static const uint16_t SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082;
static const uint16_t SYSTEM__INTERRUPT_CLEAR = 0x0086;
static const uint16_t SYSTEM__MODE_START = 0x0087;
static const uint16_t RESULT__RANGE_STATUS = 0x0089;
static const uint16_t PHASECAL_RESULT__VCSEL_START = 0x00D8;
static const uint16_t RESULT__OSC_CALIBRATE_VAL = 0x00DE;
static const uint16_t FIRMWARE__SYSTEM_STATUS = 0x00E5;
static const uint16_t IDENTIFICATION__MODEL_ID = 0x010F;

static const uint8_t VL53L1X_DEFAULT_ADDRESS = 0x29;
static const uint16_t VL53L1X_MODEL_ID = 0xEACC;

// Measurement timing budget overhead, assumes PresetMode is LOWPOWER_AUTONOMOUS (see the Pololu library for the
// derivation)
static const uint32_t TIMING_GUARD = 4528;
// Value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, used in DSS calculations
static const uint16_t TARGET_RATE = 0x0A00;
// Number of bytes read from RESULT__RANGE_STATUS through RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0
static const uint8_t RESULT_BLOCK_SIZE = 17;

std::vector<VL53L1XComponent *> VL53L1XComponent::vl53l1x_sensors;  // NOLINT
bool VL53L1XComponent::enable_pin_setup_complete = false;           // NOLINT

VL53L1XComponent::VL53L1XComponent() { VL53L1XComponent::vl53l1x_sensors.push_back(this); }

const char *range_status_to_string(RangeStatus status) {
  switch (status) {
    case RANGE_STATUS_VALID:
      return "range valid";
    case RANGE_STATUS_SIGMA_FAIL:
      return "sigma fail";
    case RANGE_STATUS_SIGNAL_FAIL:
      return "signal fail";
    case RANGE_STATUS_VALID_MIN_RANGE_CLIPPED:
      return "range valid, min range clipped";
    case RANGE_STATUS_OUT_OF_BOUNDS_FAIL:
      return "out of bounds fail";
    case RANGE_STATUS_HARDWARE_FAIL:
      return "hardware fail";
    case RANGE_STATUS_VALID_NO_WRAP_CHECK_FAIL:
      return "range valid, no wrap check fail";
    case RANGE_STATUS_WRAP_TARGET_FAIL:
      return "wrap target fail";
    case RANGE_STATUS_XTALK_SIGNAL_FAIL:
      return "xtalk signal fail";
    case RANGE_STATUS_SYNCHRONIZATION_INT:
      return "synchronization int";
    case RANGE_STATUS_MIN_RANGE_FAIL:
      return "min range fail";
    case RANGE_STATUS_NONE:
      return "no update";
    default:
      return "unknown status";
  }
}

static const char *distance_mode_to_string(DistanceMode mode) {
  switch (mode) {
    case DISTANCE_MODE_SHORT:
      return "short";
    case DISTANCE_MODE_MEDIUM:
      return "medium";
    case DISTANCE_MODE_LONG:
      return "long";
    default:
      return "unknown";
  }
}

void VL53L1XComponent::setup() {
  if (!VL53L1XComponent::enable_pin_setup_complete) {
    // Hold every sensor that has an XSHUT pin in reset, so the one being set up is the only one answering at 0x29
    for (auto *sensor : VL53L1XComponent::vl53l1x_sensors) {
      if (sensor->enable_pin_ != nullptr) {
        sensor->enable_pin_->setup();
        sensor->enable_pin_->digital_write(false);
      }
    }
    VL53L1XComponent::enable_pin_setup_complete = true;
  }

  if (this->enable_pin_ != nullptr) {
    // Release XSHUT, the sensor boots back at the default address. tBOOT is 1.2 ms max.
    this->enable_pin_->digital_write(true);
    delay(2);
  }

  // Talk to the sensor at the default address until it has been moved to the configured one
  uint8_t final_address = this->address_;
  this->set_i2c_address(VL53L1X_DEFAULT_ADDRESS);
  this->i2c_failed_ = false;

  this->setup_error_ = this->init_sensor_();
  if (this->setup_error_ != SETUP_OK) {
    this->set_i2c_address(final_address);
    this->mark_failed();
    return;
  }

  if (final_address != VL53L1X_DEFAULT_ADDRESS) {
    this->write_reg_(I2C_SLAVE__DEVICE_ADDRESS, final_address & 0x7F);
  }
  this->set_i2c_address(final_address);

  this->apply_distance_mode_(this->distance_mode_);
  if (!this->apply_timing_budget_(this->timing_budget_us_)) {
    this->setup_error_ = SETUP_TIMING_BUDGET_REJECTED;
    this->mark_failed();
    return;
  }

  // from VL53L1X::setROISize(): force the ROI to be centered if width or height > 10, matching the ULD API
  if (this->roi_width_ > 10 || this->roi_height_ > 10) {
    this->write_reg_(ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199);
  }
  this->write_reg_(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                   (this->roi_height_ - 1) << 4 | (this->roi_width_ - 1));
  if (this->roi_center_ >= 0) {
    this->write_reg_(ROI_CONFIG__USER_ROI_CENTRE_SPAD, static_cast<uint8_t>(this->roi_center_));
  }

  // Inter-measurement period equal to the timing budget, the shortest period the sensor supports
  this->start_continuous_((this->timing_budget_us_ + 999) / 1000);

  if (this->i2c_failed_) {
    this->setup_error_ = SETUP_COMMUNICATION_FAILED;
    this->mark_failed();
  }
}

void VL53L1XComponent::update() {
  if (this->waiting_for_data_) {
    // Previous measurement still pending, loop() will time it out
    return;
  }
  // Drop the latched result (it can be up to an update interval old) and wait for the next one in loop()
  this->i2c_failed_ = false;
  this->write_reg_(SYSTEM__INTERRUPT_CLEAR, 0x01);  // sys_interrupt_clear_range
  if (this->i2c_failed_) {
    ESP_LOGW(TAG, "Failed to start a measurement");
    this->publish_failure_();
    return;
  }
  this->waiting_for_data_ = true;
  this->wait_start_ms_ = millis();
}

void VL53L1XComponent::loop() {
  if (!this->waiting_for_data_)
    return;

  bool ready = false;
  if (!this->data_ready_(ready)) {
    this->waiting_for_data_ = false;
    ESP_LOGW(TAG, "Failed to read the data ready flag");
    this->publish_failure_();
    return;
  }
  if (!ready) {
    if (millis() - this->wait_start_ms_ > this->timeout_ms_) {
      this->waiting_for_data_ = false;
      ESP_LOGW(TAG, "Timed out waiting for a measurement");
      this->publish_failure_();
    }
    return;
  }

  this->waiting_for_data_ = false;
  if (!this->read_measurement_()) {
    ESP_LOGW(TAG, "Failed to read the measurement");
    this->publish_failure_();
    return;
  }
  this->status_clear_warning();
  this->publish_measurement_();
}

void VL53L1XComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X:");
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  switch (this->setup_error_) {
    case SETUP_WRONG_MODEL_ID:
      ESP_LOGE(TAG, "  Wrong model ID, is this a VL53L1X?");
      break;
    case SETUP_BOOT_TIMEOUT:
      ESP_LOGE(TAG, "  Timed out waiting for the sensor to boot");
      break;
    case SETUP_COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "  Communication with the sensor failed");
      break;
    case SETUP_TIMING_BUDGET_REJECTED:
      ESP_LOGE(TAG, "  Timing budget rejected by the sensor");
      break;
    default:
      break;
  }
  ESP_LOGCONFIG(TAG,
                "  Distance Mode: %s\n"
                "  Timing Budget: %" PRIu32 " us\n"
                "  Timeout: %" PRIu32 " ms\n"
                "  ROI: %ux%u",
                distance_mode_to_string(this->distance_mode_), this->timing_budget_us_, this->timeout_ms_,
                this->roi_width_, this->roi_height_);
  if (this->roi_center_ >= 0) {
    ESP_LOGCONFIG(TAG, "  ROI Center SPAD: %d", this->roi_center_);
  }
  LOG_UPDATE_INTERVAL(this);
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Signal Rate", this->signal_rate_sensor_);
  LOG_SENSOR("  ", "Ambient Rate", this->ambient_rate_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Range Status", this->range_status_text_sensor_);
#endif
}

void VL53L1XComponent::write_reg_(uint16_t reg, uint8_t value) {
  if (this->write_register16(reg, &value, 1) != i2c::ERROR_OK)
    this->i2c_failed_ = true;
}

void VL53L1XComponent::write_reg16_(uint16_t reg, uint16_t value) {
  uint8_t data[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  if (this->write_register16(reg, data, sizeof(data)) != i2c::ERROR_OK)
    this->i2c_failed_ = true;
}

void VL53L1XComponent::write_reg32_(uint16_t reg, uint32_t value) {
  uint8_t data[4] = {static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
                     static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  if (this->write_register16(reg, data, sizeof(data)) != i2c::ERROR_OK)
    this->i2c_failed_ = true;
}

uint8_t VL53L1XComponent::read_reg_(uint16_t reg) {
  uint8_t value = 0;
  if (this->read_register16(reg, &value, 1) != i2c::ERROR_OK)
    this->i2c_failed_ = true;
  return value;
}

uint16_t VL53L1XComponent::read_reg16_(uint16_t reg) {
  uint8_t data[2] = {0, 0};
  if (this->read_register16(reg, data, sizeof(data)) != i2c::ERROR_OK)
    this->i2c_failed_ = true;
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

// Initialize the sensor using settings taken mostly from VL53L1_DataInit() and VL53L1_StaticInit(), in 2V8 I/O mode.
// from VL53L1X::init()
VL53L1XComponent::SetupError VL53L1XComponent::init_sensor_() {
  // check model ID and module type registers (values specified in datasheet)
  uint16_t model_id = this->read_reg16_(IDENTIFICATION__MODEL_ID);
  if (this->i2c_failed_)
    return SETUP_COMMUNICATION_FAILED;
  if (model_id != VL53L1X_MODEL_ID) {
    ESP_LOGE(TAG, "Unexpected model ID 0x%04X", model_id);
    return SETUP_WRONG_MODEL_ID;
  }

  // VL53L1_software_reset()
  this->write_reg_(SOFT_RESET, 0x00);
  delayMicroseconds(100);
  this->write_reg_(SOFT_RESET, 0x01);

  // give it some time to boot, otherwise the sensor NACKs the first status read below
  delay(1);

  // VL53L1_poll_for_boot_completion(), a NACK here only means the sensor is still booting
  uint32_t start = millis();
  while (true) {
    uint8_t status = 0;
    if (this->read_register16(FIRMWARE__SYSTEM_STATUS, &status, 1) == i2c::ERROR_OK && (status & 0x01) != 0)
      break;
    if (millis() - start > this->timeout_ms_)
      return SETUP_BOOT_TIMEOUT;
    delay(1);
  }

  // VL53L1_DataInit()
  this->i2c_failed_ = false;

  // sensor uses 1V8 mode for I/O by default, switch to 2V8 mode
  this->write_reg_(PAD_I2C_HV__EXTSUP_CONFIG, this->read_reg_(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);

  // store oscillator info for later use
  this->fast_osc_frequency_ = this->read_reg16_(OSC_MEASURED__FAST_OSC__FREQUENCY);
  this->osc_calibrate_val_ = this->read_reg16_(RESULT__OSC_CALIBRATE_VAL);
  if (this->i2c_failed_)
    return SETUP_COMMUNICATION_FAILED;
  if (this->fast_osc_frequency_ == 0) {
    // calc_macro_period_() divides by it
    ESP_LOGE(TAG, "Sensor reported a fast oscillator frequency of 0");
    return SETUP_COMMUNICATION_FAILED;
  }

  // VL53L1_StaticInit(): the API only keeps these in memory until a measurement is started, the Pololu library (and
  // this component) writes them straight away. Values labeled "tuning parm default" are from
  // vl53l1_tuning_parm_defaults.h

  // static config
  this->write_reg16_(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TARGET_RATE);  // should already be this value after reset
  this->write_reg_(GPIO__TIO_HV_STATUS, 0x02);
  this->write_reg_(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);     // tuning parm default
  this->write_reg_(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);  // tuning parm default
  this->write_reg_(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  this->write_reg_(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  this->write_reg_(ALGO__RANGE_MIN_CLIP, 0);                // tuning parm default
  this->write_reg_(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);  // tuning parm default

  // general config
  this->write_reg16_(SYSTEM__THRESH_RATE_HIGH, 0x0000);
  this->write_reg16_(SYSTEM__THRESH_RATE_LOW, 0x0000);
  this->write_reg_(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config, most of it is set later by the distance mode and timing budget
  this->write_reg16_(RANGE_CONFIG__SIGMA_THRESH, 360);                   // tuning parm default
  this->write_reg16_(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);  // tuning parm default

  // dynamic config
  this->write_reg_(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  this->write_reg_(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  this->write_reg_(SD_CONFIG__QUANTIFIER, 2);  // tuning parm default

  // from VL53L1_preset_mode_timed_ranging_*: writing GPH0 and GPH1 above sets GPH to 1, ranging does not work unless
  // it is set back to 0
  this->write_reg_(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  this->write_reg_(SYSTEM__SEED_CONFIG, 1);  // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  this->write_reg_(SYSTEM__SEQUENCE_CONFIG, 0x8B);  // VHV, PHASECAL, DSS1, RANGE
  this->write_reg16_(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  this->write_reg_(DSS_CONFIG__ROI_MODE_CONTROL, 2);  // REQUESTED_EFFFECTIVE_SPADS

  // long range, 50 ms timing budget (Pololu defaults), the configured values are applied by setup() afterwards
  this->apply_distance_mode_(DISTANCE_MODE_LONG);
  this->apply_timing_budget_(50000);

  // the API does this in VL53L1_init_and_start_range() once a measurement is started, assumes MM1 and MM2 are
  // disabled
  this->write_reg16_(ALGO__PART_TO_PART_RANGE_OFFSET_MM, this->read_reg16_(MM_CONFIG__OUTER_OFFSET_MM) * 4);

  return this->i2c_failed_ ? SETUP_COMMUNICATION_FAILED : SETUP_OK;
}

// from VL53L1X::setDistanceMode(), based on VL53L1_SetDistanceMode()
void VL53L1XComponent::apply_distance_mode_(DistanceMode mode) {
  // save existing timing budget
  uint32_t budget_us = this->read_timing_budget_();

  switch (mode) {
    case DISTANCE_MODE_SHORT:
      // from VL53L1_preset_mode_standard_ranging_short_range()
      this->write_reg_(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      this->write_reg_(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      this->write_reg_(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
      this->write_reg_(SD_CONFIG__WOI_SD0, 0x07);
      this->write_reg_(SD_CONFIG__WOI_SD1, 0x05);
      this->write_reg_(SD_CONFIG__INITIAL_PHASE_SD0, 6);  // tuning parm default
      this->write_reg_(SD_CONFIG__INITIAL_PHASE_SD1, 6);  // tuning parm default
      break;

    case DISTANCE_MODE_MEDIUM:
      // from VL53L1_preset_mode_standard_ranging()
      this->write_reg_(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      this->write_reg_(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      this->write_reg_(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
      this->write_reg_(SD_CONFIG__WOI_SD0, 0x0B);
      this->write_reg_(SD_CONFIG__WOI_SD1, 0x09);
      this->write_reg_(SD_CONFIG__INITIAL_PHASE_SD0, 10);  // tuning parm default
      this->write_reg_(SD_CONFIG__INITIAL_PHASE_SD1, 10);  // tuning parm default
      break;

    case DISTANCE_MODE_LONG:
    default:
      // from VL53L1_preset_mode_standard_ranging_long_range()
      this->write_reg_(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      this->write_reg_(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      this->write_reg_(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
      this->write_reg_(SD_CONFIG__WOI_SD0, 0x0F);
      this->write_reg_(SD_CONFIG__WOI_SD1, 0x0D);
      this->write_reg_(SD_CONFIG__INITIAL_PHASE_SD0, 14);  // tuning parm default
      this->write_reg_(SD_CONFIG__INITIAL_PHASE_SD1, 14);  // tuning parm default
      break;
  }

  // reapply timing budget, the timeout registers depend on the VCSEL periods
  this->apply_timing_budget_(budget_us);
}

// Set the measurement timing budget in microseconds, the time allowed for one measurement.
// from VL53L1X::setMeasurementTimingBudget(), based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool VL53L1XComponent::apply_timing_budget_(uint32_t budget_us) {
  // assumes PresetMode is LOWPOWER_AUTONOMOUS
  if (budget_us <= TIMING_GUARD)
    return false;

  uint32_t range_config_timeout_us = budget_us - TIMING_GUARD;
  if (range_config_timeout_us > 1100000)  // FDA_MAX_TIMING_BUDGET_US * 2
    return false;

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values()

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = this->calc_macro_period_(this->read_reg_(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A", 1000 us is TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT
  uint32_t phasecal_timeout_mclks = timeout_microseconds_to_mclks_(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF)
    phasecal_timeout_mclks = 0xFF;
  this->write_reg_(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout", 1 us is LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT
  this->write_reg16_(MM_CONFIG__TIMEOUT_MACROP_A, encode_timeout_(timeout_microseconds_to_mclks_(1, macro_period_us)));

  // "Update Range Timing A timeout"
  this->write_reg16_(RANGE_CONFIG__TIMEOUT_MACROP_A,
                     encode_timeout_(timeout_microseconds_to_mclks_(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = this->calc_macro_period_(this->read_reg_(RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  this->write_reg16_(MM_CONFIG__TIMEOUT_MACROP_B, encode_timeout_(timeout_microseconds_to_mclks_(1, macro_period_us)));

  // "Update Range Timing B timeout"
  this->write_reg16_(RANGE_CONFIG__TIMEOUT_MACROP_B,
                     encode_timeout_(timeout_microseconds_to_mclks_(range_config_timeout_us, macro_period_us)));

  return true;
}

// from VL53L1X::getMeasurementTimingBudget(), assumes PresetMode is LOWPOWER_AUTONOMOUS and the VHV, PHASECAL, DSS1
// and RANGE sequence steps are enabled
uint32_t VL53L1XComponent::read_timing_budget_() {
  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = this->calc_macro_period_(this->read_reg_(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"
  uint32_t range_config_timeout_us = timeout_mclks_to_microseconds_(
      decode_timeout_(this->read_reg16_(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  return 2 * range_config_timeout_us + TIMING_GUARD;
}

// from VL53L1X::startContinuous()
void VL53L1XComponent::start_continuous_(uint32_t period_ms) {
  // from VL53L1_set_inter_measurement_period_ms()
  this->write_reg32_(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * this->osc_calibrate_val_);

  this->write_reg_(SYSTEM__INTERRUPT_CLEAR, 0x01);  // sys_interrupt_clear_range
  this->write_reg_(SYSTEM__MODE_START, 0x40);       // mode_range__timed
}

// from VL53L1X::dataReady(), assumes the interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1, the reset default)
bool VL53L1XComponent::data_ready_(bool &ready) {
  uint8_t status = 0;
  if (this->read_register16(GPIO__TIO_HV_STATUS, &status, 1) != i2c::ERROR_OK)
    return false;
  ready = (status & 0x01) == 0;
  return true;
}

// Read the finished measurement, run the per-measurement calibration steps and clear the interrupt.
// from VL53L1X::read(), VL53L1X::readResults() and VL53L1X::getRangingData()
bool VL53L1XComponent::read_measurement_() {
  uint8_t buf[RESULT_BLOCK_SIZE];
  if (this->read_register16(RESULT__RANGE_STATUS, buf, sizeof(buf)) != i2c::ERROR_OK)
    return false;

  // buf[1] is report_status, peak_signal_count_rate_mcps_sd0 (buf[5..6]), sigma_sd0 (buf[9..10]) and phase_sd0
  // (buf[11..12]) are not used
  this->results_.range_status = buf[0];
  this->results_.stream_count = buf[2];
  this->results_.dss_actual_effective_spads_sd0 = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
  this->results_.ambient_count_rate_mcps_sd0 = (static_cast<uint16_t>(buf[7]) << 8) | buf[8];
  this->results_.final_crosstalk_corrected_range_mm_sd0 = (static_cast<uint16_t>(buf[13]) << 8) | buf[14];
  this->results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = (static_cast<uint16_t>(buf[15]) << 8) | buf[16];

  this->i2c_failed_ = false;

  if (!this->calibrated_) {
    this->setup_manual_calibration_();
    this->calibrated_ = true;
  }

  this->update_dss_();

  // VL53L1_copy_sys_and_core_results_to_range_results(): "apply correction gain", 2011 is
  // VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT, scales the result by 2011/2048 with rounding
  uint16_t range = this->results_.final_crosstalk_corrected_range_mm_sd0;
  this->range_mm_ = (static_cast<uint32_t>(range) * 2011 + 0x0400) / 0x0800;

  // mostly based on ConvertStatusLite()
  switch (this->results_.range_status) {
    case 17:  // MULTCLIPFAIL
    case 2:   // VCSELWATCHDOGTESTFAILURE
    case 1:   // VCSELCONTINUITYTESTFAILURE
    case 3:   // NOVHVVALUEFOUND
      this->range_status_ = RANGE_STATUS_HARDWARE_FAIL;
      break;
    case 13:  // USERROICLIP
      this->range_status_ = RANGE_STATUS_MIN_RANGE_FAIL;
      break;
    case 18:  // GPHSTREAMCOUNT0READY
      this->range_status_ = RANGE_STATUS_SYNCHRONIZATION_INT;
      break;
    case 5:  // RANGEPHASECHECK
      this->range_status_ = RANGE_STATUS_OUT_OF_BOUNDS_FAIL;
      break;
    case 4:  // MSRCNOTARGET
      this->range_status_ = RANGE_STATUS_SIGNAL_FAIL;
      break;
    case 6:  // SIGMATHRESHOLDCHECK
      this->range_status_ = RANGE_STATUS_SIGMA_FAIL;
      break;
    case 7:  // PHASECONSISTENCY
      this->range_status_ = RANGE_STATUS_WRAP_TARGET_FAIL;
      break;
    case 12:  // RANGEIGNORETHRESHOLD
      this->range_status_ = RANGE_STATUS_XTALK_SIGNAL_FAIL;
      break;
    case 8:  // MINCLIP
      this->range_status_ = RANGE_STATUS_VALID_MIN_RANGE_CLIPPED;
      break;
    case 9:  // RANGECOMPLETE
      this->range_status_ =
          this->results_.stream_count == 0 ? RANGE_STATUS_VALID_NO_WRAP_CHECK_FAIL : RANGE_STATUS_VALID;
      break;
    default:
      this->range_status_ = RANGE_STATUS_NONE;
      break;
  }

  // count rates are in fixed point 9.7 format
  this->peak_signal_count_rate_mcps_ =
      static_cast<float>(this->results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0) / (1 << 7);
  this->ambient_count_rate_mcps_ = static_cast<float>(this->results_.ambient_count_rate_mcps_sd0) / (1 << 7);

  this->write_reg_(SYSTEM__INTERRUPT_CLEAR, 0x01);  // sys_interrupt_clear_range

  return !this->i2c_failed_;
}

// "Setup ranges after the first one in low power auto mode by turning off FW calibration steps and programming static
// values"
// from VL53L1X::setupManualCalibration(), based on VL53L1_low_power_auto_setup_manual_calibration()
void VL53L1XComponent::setup_manual_calibration_() {
  // "save original vhv configs"
  this->saved_vhv_init_ = this->read_reg_(VHV_CONFIG__INIT);
  this->saved_vhv_timeout_ = this->read_reg_(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  // "disable VHV init"
  this->write_reg_(VHV_CONFIG__INIT, this->saved_vhv_init_ & 0x7F);

  // "set loop bound to tuning param", LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT
  this->write_reg_(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (this->saved_vhv_timeout_ & 0x03) + (3 << 2));

  // "override phasecal"
  this->write_reg_(PHASECAL_CONFIG__OVERRIDE, 0x01);
  this->write_reg_(CAL_CONFIG__VCSEL_START, this->read_reg_(PHASECAL_RESULT__VCSEL_START));
}

// Dynamic SPAD selection
// from VL53L1X::updateDSS(), based on VL53L1_low_power_auto_update_DSS()
void VL53L1XComponent::update_dss_() {
  uint16_t spad_count = this->results_.dss_actual_effective_spads_sd0;

  if (spad_count != 0) {
    // "Calc total rate per spad"
    uint32_t total_rate_per_spad =
        static_cast<uint32_t>(this->results_.peak_signal_count_rate_crosstalk_corrected_mcps_sd0) +
        this->results_.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (total_rate_per_spad > 0xFFFF)
      total_rate_per_spad = 0xFFFF;

    // "shift up to take advantage of 32 bits"
    total_rate_per_spad <<= 16;

    total_rate_per_spad /= spad_count;

    if (total_rate_per_spad != 0) {
      // "get the target rate and shift up by 16"
      uint32_t required_spads = (static_cast<uint32_t>(TARGET_RATE) << 16) / total_rate_per_spad;

      // "clip to 16 bit"
      if (required_spads > 0xFFFF)
        required_spads = 0xFFFF;

      // "override DSS config", DSS_CONFIG__ROI_MODE_CONTROL is already set to REQUESTED_EFFFECTIVE_SPADS
      this->write_reg16_(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, required_spads);
      return;
    }
  }

  // Anything above would have divided by zero. "We want to gracefully set a spad target, not just exit with an error",
  // "set target to mid point"
  this->write_reg16_(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

void VL53L1XComponent::publish_measurement_() {
  bool valid = this->range_status_ == RANGE_STATUS_VALID;
  ESP_LOGV(TAG, "Range %u mm, status '%s', signal %.2f Mcps, ambient %.2f Mcps", this->range_mm_,
           range_status_to_string(this->range_status_), this->peak_signal_count_rate_mcps_,
           this->ambient_count_rate_mcps_);
  if (!valid) {
    ESP_LOGD(TAG, "No valid distance: %s", range_status_to_string(this->range_status_));
  }
#ifdef USE_SENSOR
  if (this->distance_sensor_ != nullptr)
    this->distance_sensor_->publish_state(valid ? this->range_mm_ / 1000.0f : NAN);
  if (this->signal_rate_sensor_ != nullptr)
    this->signal_rate_sensor_->publish_state(this->peak_signal_count_rate_mcps_);
  if (this->ambient_rate_sensor_ != nullptr)
    this->ambient_rate_sensor_->publish_state(this->ambient_count_rate_mcps_);
#endif
#ifdef USE_TEXT_SENSOR
  if (this->range_status_text_sensor_ != nullptr)
    this->range_status_text_sensor_->publish_state(range_status_to_string(this->range_status_));
#endif
}

void VL53L1XComponent::publish_failure_() {
  this->status_set_warning();
#ifdef USE_SENSOR
  if (this->distance_sensor_ != nullptr)
    this->distance_sensor_->publish_state(NAN);
  if (this->signal_rate_sensor_ != nullptr)
    this->signal_rate_sensor_->publish_state(NAN);
  if (this->ambient_rate_sensor_ != nullptr)
    this->ambient_rate_sensor_->publish_state(NAN);
#endif
}

// Macro period in microseconds (12.12 format) for the given VCSEL period, assumes fast_osc_frequency_ has been read.
// from VL53L1X::calcMacroPeriod(), based on VL53L1_calc_macro_period_us()
uint32_t VL53L1XComponent::calc_macro_period_(uint8_t vcsel_period) const {
  // from VL53L1_calc_pll_period_us(): fast osc frequency in 4.12 format, PLL period in 0.24 format
  uint32_t pll_period_us = (static_cast<uint32_t>(0x01) << 30) / this->fast_osc_frequency_;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = static_cast<uint32_t>(2304) * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

// from VL53L1X::decodeTimeout(), based on VL53L1_decode_timeout()
uint32_t VL53L1XComponent::decode_timeout_(uint16_t reg_val) {
  return (static_cast<uint32_t>(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encoded format is "(LSByte * 2^MSByte) + 1"
// from VL53L1X::encodeTimeout(), based on VL53L1_encode_timeout()
uint16_t VL53L1XComponent::encode_timeout_(uint32_t timeout_mclks) {
  if (timeout_mclks == 0)
    return 0;

  uint32_t ls_byte = timeout_mclks - 1;
  uint16_t ms_byte = 0;
  while ((ls_byte & 0xFFFFFF00) > 0) {
    ls_byte >>= 1;
    ms_byte++;
  }
  return (ms_byte << 8) | (ls_byte & 0xFF);
}

// from VL53L1X::timeoutMclksToMicroseconds(), based on VL53L1_calc_timeout_us()
uint32_t VL53L1XComponent::timeout_mclks_to_microseconds_(uint32_t timeout_mclks, uint32_t macro_period_us) {
  return (static_cast<uint64_t>(timeout_mclks) * macro_period_us + 0x800) >> 12;
}

// from VL53L1X::timeoutMicrosecondsToMclks(), based on VL53L1_calc_timeout_mclks()
uint32_t VL53L1XComponent::timeout_microseconds_to_mclks_(uint32_t timeout_us, uint32_t macro_period_us) {
  return ((timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

}  // namespace vl53l1x
}  // namespace esphome
