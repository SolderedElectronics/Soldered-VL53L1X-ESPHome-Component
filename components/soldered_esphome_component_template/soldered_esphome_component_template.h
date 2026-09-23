/**
 * @file soldered_esphome_component_template.h
 * @brief Public API for the soldered_esphome_component_template ESPHome component
 * @author Soldered Electronics
 */

#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace soldered_esphome_component_template {

/**
 * @brief Example component, replace with the real component API.
 */
class SolderedEsphomeComponentTemplate : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
};

}  // namespace soldered_esphome_component_template
}  // namespace esphome
