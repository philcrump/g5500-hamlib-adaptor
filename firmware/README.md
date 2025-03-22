# G-5500 Hamlib Adaptor - Firmware

## Dependencies

Rust toolchain: https://doc.rust-lang.org/cargo/getting-started/installation.html

This project uses latest stable Rust (1.85.1 as of time of writing).

Probe-rs (for SWD flashing and debugging): `cargo install probe-rs --features=cli`

NB this firmware is only compatible with the "W5500-EVB-Pico" daughterboard. Do NOT use "W55RP20-EVB-PICO", this requires a PIO SPI driver to talk to the W5500 on hardwired non-standard pins inside the chip package. This PIO SPI driver does not currently exist for embassy.

## Usage

Build: `cargo build`

Build, flash with Pi Pico debug probe, and run: `cargo run`

## rotctld protocol

Reference: https://hamlib.sourceforge.net/html/rotctld.1.html

## Notes

`cargo install elf2uf2-rs` - Might allow flashing over USB serial or UFS?
