# Errata

## dev-r1 (2025-03-08)

### Major

#### Regulator is hot

Input voltage from rotator = 17.3V, causes more heat dissipation in the regulator than expected.

Workaround: Small TO-220 heatsink added, being careful of clearance with C2. This heatsink is still uncomfortable to touch.

Next revision: Use TSR1-2450 DC-DC, with appropriate Class 1 filtering to keep Martin happy.

### DIN connnector pinout

DIN connector pinout slightly off.

Workaround: Use DIN connector patch pads to fix pinout.

Next revision: Use BC817s or similar discrete devices with low leakage current.

### G-5500 Controller Relays Sticking On

Leakage current from ULN2803A darlington array is about 200uA per channel, this is enough to cause the switch relays to 'stick' in the G-5500 controller.

Workaround: 15K PTH resistor is installed in series on the DIN connector patch pads for each relay channel to limit the leakage current.

Next revision: Use BC817s or similar discrete devices with low leakage current.

### Minor

#### LED Current is low

LED Current: 2.23V / 180R = 12.4mA
Better would be 120R.

Workaround: None for this revision, 120R to be used on next revision.

#### DIN connector very tight to insert

Look for replacement part next time?

Workaround: None for this revision.

#### SWD/RUN connectors not required.

RPi debug probe FTW.

Workaround: Don't fit SWD/Run connectors, use SWD port on Pi Pico daughterboard.
