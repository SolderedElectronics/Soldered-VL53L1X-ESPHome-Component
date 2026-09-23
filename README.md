# Soldered NAZIV PROIZVODA ESPHome Component

| ![Product name](https://upload.wikimedia.org/wikipedia/commons/8/8f/Example_image.svg) |
| :------------------------------------------------------------------------------------: |
|                      [NAZIV PROIZVODA](https://www.solde.red/SKU)                      |

OPIS PROIZVODA + LINK NA [Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem).

External ESPHome component for NAZIV PROIZVODA.

### Using the template

Before publishing a new component make sure to replace:

- `NAZIV PROIZVODA`, `OPIS PROIZVODA`, product image, and SKU link in this README
- the `components/soldered_esphome_component_template/` directory name with the real component name
- `soldered_esphome_component_template` namespace, `SolderedEsphomeComponentTemplate` class name, and `CODEOWNERS` in `__init__.py`, matching names in the `.h`/`.cpp` files and their `#include`
- `CONFIG_SCHEMA` and `to_code()` in `__init__.py` with the real config options and codegen
- the `TAG` string and `dump_config()` output in the `.cpp` file
- `github://SolderedElectronics/<repo>` source path and the sample config in the "Usage" section below
- `examples/basic.yaml` (rename/add examples as needed, keep `external_components.source.path` pointing at `../components`)
- `@file`, `@brief`, `@author` Doxygen comments in the `.h`/`.cpp` files to describe the real API

Also make sure to add more examples if the component supports multiple boards/modes (see `Soldered-Inkplate-ESPHome` for a repo with several board variants).

Run `pip install clang-format==13.0.1 && find components -name "*.cpp" -o -name "*.h" | xargs clang-format -i` before committing to auto-format the component against ESPHome's own style (`.clang-format`, copied from the ESPHome core repo). CI runs the same check on every push/PR via `.github/workflows/format_check.yml` and fails on unformatted code. `.github/workflows/build.yml` compiles every YAML under `examples/` on every push/PR.

**Remove this section of README after everything is done!**

## Repository Contents

- **components/** - the ESPHome external component (Python config + C++ implementation)
- **examples/** - example YAML configs showing how to use the component

## Usage

Reference this repo directly from your own ESPHome YAML (no need to clone it locally):

```yaml
external_components:
  - source: github://SolderedElectronics/<repo>
    components: [soldered_esphome_component_template]

soldered_esphome_component_template:
```

See [`examples/basic.yaml`](examples/basic.yaml) for a full working example.

### Hardware design

You can find hardware design for this board in the _NAZIV PROIZVODA_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

<img src="https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png" alt="soldered-logo" width="500"/>

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
