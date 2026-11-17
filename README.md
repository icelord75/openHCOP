# openHCOP

Coil-on-plug controller for HONDA H- F- D- B-series OBD1 Engines.

Piggyback ignition controller, for replace distributor internals: igniter unit, ignition coil, distributor cap and coil wires with Coil-on-plug.

Direct connection to CYL angle sensor - Orange(signal) and White(ground) wires (OBD1 distributers),
Solder directly to sensor wires, do not cut and/or twist over wires!

Support positive triggering 3-wire Coil-On-Plugs.

## Proof of work

[![Proof of Work](https://img.youtube.com/vi/j7przlhhsT8/0.jpg)](https://www.youtube.com/watch?v=j7przlhhsT8)

## Build and flash

Firmware is for ATtiny44 (internal 8 MHz RC oscillator), built with [PlatformIO](https://platformio.org/):

    pio run                 # build
    pio run -t fuses        # write fuses (lfuse 0x52, hfuse 0xDD, efuse 0xFF), once
    pio run -t upload       # flash (USBtiny programmer by default, see platformio.ini)

Without PlatformIO: `cd src && make && make install` (needs avr-gcc and avrdude).

## Tests

Simulation tests of the firmware (simavr) are in [sim/](sim/README.md): `cd sim && make`.

## Order board

Order from [OSHPark](https://oshpark.com/shared_projects/Hhu0kevI)

## License

**openHCOP** is available under GPL License.
