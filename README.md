# Soldered VL53L1X ESPHome Component

| ![VL53L1X ToF Laser Distance Sensor](https://cms.soldered.com/products/333064/media/333064_featured-photo_9c57c0.jpg) |
| :--------------------------------------------------------------------------------------------------------------------: |
|                           [VL53L1X ToF Laser Distance Sensor](https://www.solde.red/333064)                           |

Time-of-flight laser distance sensor breakout board built around the ST VL53L1X. It measures distance from 30 mm to
4 m using an invisible 940 nm Class 1 laser, at up to 50 Hz, and has a programmable Region of Interest (15° - 27°
field of view). It communicates over I2C (default address 0x29, programmable) and is part of the
[Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem).

External ESPHome component for the Soldered VL53L1X breakout board. It is a port of the
[Soldered VL53L1X Arduino library](https://github.com/SolderedElectronics/Soldered-VL53L1X-Laser-Distance-Sensor-Arduino-Library)
(which is built on the [Pololu VL53L1X library](https://github.com/pololu/vl53l1x-arduino)) and exposes the distance,
the signal and ambient count rates and the range status of every measurement as ESPHome sensors.

## Repository Contents

- **components/** - the ESPHome external component (Python config + C++ implementation)
- **examples/** - example YAML configs showing how to use the component

## Usage

Reference this repo directly from your own ESPHome YAML (no need to clone it locally):

```yaml
external_components:
  - source: github://SolderedElectronics/Soldered-VL53L1X-ESPHome-Component
    components: [vl53l1x]

i2c:
  sda: GPIO21
  scl: GPIO22

vl53l1x:
  id: tof
  update_interval: 1s

sensor:
  - platform: vl53l1x
    distance:
      name: "Distance"

text_sensor:
  - platform: vl53l1x
    range_status:
      name: "Range Status"
```

The sensor ranges continuously on its own, one measurement per timing budget. On every update the component discards
the latched result and publishes the next fresh measurement (at most one timing budget later), so published values
are never stale, whatever the `update_interval`.

A distance is only published when the range status is `range valid`. Any other status (nothing in range, too much
ambient light, target too close, ...) publishes `NAN` (shown as *unknown* in Home Assistant); the signal rate, ambient
rate and range status are still published so the cause can be seen.

See [`examples/basic.yaml`](examples/basic.yaml) for a full working example.

### Configuration variables

**`vl53l1x:`** (the sensor itself, can be given more than once for several sensors)

- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#config-id)): ID of the sensor, needed by the
  `sensor` / `text_sensor` platforms if there is more than one VL53L1X.
- **address** (*Optional*, int): I2C address. Defaults to `0x29`. Any other address requires `enable_pin`: the chip
  always powers up at `0x29` and is moved to the configured address on every boot.
- **enable_pin** (*Optional*, [Pin](https://esphome.io/guides/configuration-types#pin)): GPIO wired to the sensor's
  XSHUT pin. Needed to run several VL53L1X on one I2C bus: on boot all sensors with an `enable_pin` are held in reset
  and then brought up one at a time.
- **distance_mode** (*Optional*, string): `short` (up to ~1.3 m, best ambient light immunity), `medium` (up to ~3 m)
  or `long` (up to ~4 m in the dark). Defaults to `long`.
- **timing_budget** (*Optional*, [Time](https://esphome.io/guides/configuration-types#config-time)): time allowed for
  one measurement, longer is more accurate. `20ms` - `1000ms`, and at least `33ms` in `medium` and `long` mode.
  Defaults to `50ms`. The sensor also measures once per timing budget.
- **timeout** (*Optional*, [Time](https://esphome.io/guides/configuration-types#config-time)): how long to wait for
  the sensor to boot and for a measurement to finish before giving up. Defaults to `500ms`.
- **roi** (*Optional*): Region of Interest, see ST's [UM2555](https://www.st.com/resource/en/user_manual/um2555-vl53l1x-ultra-lite-driver-multiple-zone-implementation-stmicroelectronics.pdf).
  - **width** (*Optional*, int): `4` - `16` SPADs. Defaults to `16`.
  - **height** (*Optional*, int): `4` - `16` SPADs. Defaults to `16`.
  - **center** (*Optional*, int): center SPAD number, `0` - `255`. `199` is the optical center, and is forced if
    width or height is above 10 and no center is given.
- **update_interval** (*Optional*, [Time](https://esphome.io/guides/configuration-types#config-time)): how often to
  publish a measurement. Defaults to `60s`.

**`sensor:`** (`platform: vl53l1x`)

- **vl53l1x_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#config-id)): the `vl53l1x` to read
  from.
- **distance** (*Optional*): distance in meters (`NAN` if the range status is not valid). All options from
  [Sensor](https://esphome.io/components/sensor/#config-sensor).
- **signal_rate** (*Optional*): peak signal count rate in Mcps. All options from
  [Sensor](https://esphome.io/components/sensor/#config-sensor).
- **ambient_rate** (*Optional*): ambient count rate in Mcps, useful to see how much sunlight or other IR the sensor
  is seeing. All options from [Sensor](https://esphome.io/components/sensor/#config-sensor).

**`text_sensor:`** (`platform: vl53l1x`)

- **vl53l1x_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#config-id)): the `vl53l1x` to read
  from.
- **range_status** (*Optional*): range status of the last measurement (`range valid`, `sigma fail`, `signal fail`,
  `out of bounds fail`, ...), same strings as `rangeStatusToString()` in the Arduino library. All options from
  [Text Sensor](https://esphome.io/components/text_sensor/#config-text-sensor).

### Several sensors on one bus

```yaml
vl53l1x:
  - id: tof_left
    address: 0x30
    enable_pin: GPIO4
  - id: tof_right
    address: 0x31
    enable_pin: GPIO5

sensor:
  - platform: vl53l1x
    vl53l1x_id: tof_left
    distance:
      name: "Left Distance"
  - platform: vl53l1x
    vl53l1x_id: tof_right
    distance:
      name: "Right Distance"
```

### Hardware design

TODO: link the VL53L1X ToF Laser Distance Sensor hardware design repository once it is published.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Original source

The ranging code is ported from the [Pololu VL53L1X Arduino library](https://github.com/pololu/vl53l1x-arduino), which is based on ST's VL53L1X API (STSW-IMG007). Its BSD 3-clause license is included in [`components/vl53l1x/LICENSE.txt`](components/vl53l1x/LICENSE.txt). Thank you, Pololu.

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
