# Firmware for a SnapperGPS with pressure-sensor daughter-board

*Author: Jonas Beuchert*

[The daughter-board](https://github.com/SnapperGPS/snappergps-pressure-sensor-daughterboard) is built around a [MS5837-30BA high-resolution pressure sensor](https://www.te.com/en/product-CAT-BLPS0017.html).
It is soldered onto the five pads of the SnapperGPS main board.
The daughter-board is orientated such that the two pads labeled `VDD` connect.
The two boards communicate via I2C.

The pressure sensor points upwards (away from the PCB) and its gel-filled cap must be exposed to the medium in which pressure is measured (air/water).

To use the pressure sensor, flash the `SnapperGPS-PressureSensor` firmware using [https://snappergps.info/flash](https://snappergps.info/flash).

You can configure the SnapperGPS as normal.
In addition to GPS snapshots, it will then also capture pressures in a rate that is twenty times higher than the configured GPS snapshot rate.

The pressures are read out via [https://snappergps.info/pressure-sensor](https://snappergps.info/pressure-sensor) or [https://snappergps.github.io/snappergps-app-lite/pressure-sensor](https://snappergps.github.io/snappergps-app-lite/pressure-sensor).
The data is returned as JSON file and as CSV file, both of which contain the same data.
The measurements are in units of `mbar` (millibar).

**LED patterns**

If the pressure-sensor daughter-board is correctly connected and the `SnapperGPS-PressureSensor` firmware is flashed, then the green LED will blink when the receiver is powered via USB.
If the pressure-sensor daughter-board is incorrectly or not connected and the `SnapperGPS-PressureSensor` firmware is flashed, then the red LED will blink when the receiver is powered via USB.
