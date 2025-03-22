# G-5500 Hamlib Adaptor

Network-connected physical adaptor providing a Hamlib interface for the G-5500 Az+El rotator. Primarily for Gpredict.

A Wiznet Pi Pico board ("W5500-EVB-Pico") is used to serve a partial implementation of the 'rotctld' protocol on TCP sockets, and controls the rotator through the G-5500's GS-232 DIN port.

The board is powered from the DIN port.

## Firmware

This firmware is implemented using Embassy framework, written in Rust.

See firmware/README.md for more information.

## PCB

Currently only a development PCB has been built.

See pcb/README.md for more information.

## Licensing

Unless otherwise specified, all materials of this project are licensed under the BSD 3-clause License (New BSD License).

Copyright (c) Phil Crump 2025. All rights reserved.
