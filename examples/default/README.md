# I2C bus scanner

## What it does

The example scans the I2C bus in a loop and prints a table with the addresses
that answer. Every second it probes the addresses `0x03`..`0x77`, one
`i2c_dev_check_present()` call per address, and marks the ones that ACK.

The whole exchange goes through `i2cdev`, so the example also shows the
shortest useful setup of the library: `i2cdev_init()` once, a descriptor with
the pins and the clock speed, and `i2c_dev_probe()` for the probe itself. No
device mutex is created, because a probe does not touch a device handle and
nothing else runs on this bus.

## Requirements

* An ESP32 series board and an I2C bus with at least one device on it (or none:
  the table will be empty, which is a valid result too).
* Pull-up resistors on SDA and SCL, or `CONFIG_I2CDEV_AUTO_ENABLE_PULLUPS` if
  the board has none and the bus is short.

## Wiring

| Signal | Default pin | Where it comes from |
| --- | --- | --- |
| SDA | 18 | `CONFIG_EXAMPLE_I2C_MASTER_SDA` |
| SCL | 19 | `CONFIG_EXAMPLE_I2C_MASTER_SCL` |

The defaults depend on the target: 18/19 on ESP32, ESP32-S2 and ESP32-S3, 5/6
on ESP32-C2, C3, C5, C6 and C61, 3/4 on ESP32-H2 and ESP32-P4. Connect SDA and
SCL to those pins, ground to ground, and power the devices as their datasheets
require.

## Configure, build, flash

```sh
idf.py set-target esp32
idf.py menuconfig
```

The pins and the clock frequency are in `Example configuration`:

* `Example I2C master SCL` — SCL pin (`CONFIG_EXAMPLE_I2C_MASTER_SCL`)
* `Example I2C master SDA` — SDA pin (`CONFIG_EXAMPLE_I2C_MASTER_SDA`)
* `I2C clock frequency, Hz` — clock of the bus, 100000 by default
  (`CONFIG_EXAMPLE_I2C_CLOCK_HZ`)

Then:

```sh
idf.py build flash monitor
```

## Example output

Three devices on the bus, at `0x38`, `0x60` and `0x77`:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- 38 -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: 60 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 77
```

The first row is the low nibble, the left column is the high nibble, so `38`
in row `30` means address `0x38`.

## Notes

* The addresses are 7-bit, as everywhere in ESP-IDF. An 8-bit address from a
  datasheet has to be shifted right by one before it is compared with the
  table.
* A probe is a bare START/STOP pair and always runs at 100 kHz, no matter what
  the clock frequency is set to; the clock matters for the transfers of real
  drivers.
* A device that does not answer may be wired wrong, unpowered, or hold SDA low.
  The scan keeps going, so the rest of the bus is still shown.
