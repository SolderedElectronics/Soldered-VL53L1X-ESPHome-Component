/**
 * @file soldered_esphome_component_template.cpp
 * @brief Implementation for the soldered_esphome_component_template ESPHome component
 * @author Soldered Electronics
 */

#include "soldered_esphome_component_template.h"
#include "esphome/core/log.h"

namespace esphome {
namespace soldered_esphome_component_template {

static const char *const TAG = "soldered_esphome_component_template";

void SolderedEsphomeComponentTemplate::setup() {
  // Replace with real setup logic.
}

void SolderedEsphomeComponentTemplate::loop() {
  // Replace with real loop logic, or delete this override (and the one in the
  // header) if the component doesn't need to run every loop iteration.
}

void SolderedEsphomeComponentTemplate::dump_config() { ESP_LOGCONFIG(TAG, "Soldered ESPHome Component Template:"); }

}  // namespace soldered_esphome_component_template
}  // namespace esphome
