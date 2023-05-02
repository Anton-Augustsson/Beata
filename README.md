# Beata
A zephyr driver for a node sensor with capabilities of sensing noise, motion,
temperature, and humidity. Using a Raspberry Pi Pico (rp2040) as the base
station, running zephyr, and another Pico for the sensor node.

## Install

### base-station
To compile the base-station,`cd` into the folder and use:
```zsh
west build base-station -b rpi_pico
```
The command above will generate a build folder in the same directory. Copy over
the `.uf2` file found in `build/zephyr/zephyr.uf2`.

### Sensor Node
To compile the sensor node use:
```zsh
rm -r build ; cmake -B build -S . ; cd build ; make
```
This will generate a `.uf2` file in `sensor-node/build/src/`.

If you just want to test the sensor node and not upload the `.uf2` file to the
Pico board,
then you can use:
```zsh
rm -r build ; cmake -B build -S . -D BUILD_TESTS=ON ; cd build ; make
```
Which will generale a executable file in `sensor-node/build/tests/UnitTests`.
