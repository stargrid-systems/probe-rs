# Source provenance of the avr-call-chain fixture

`../avr-call-chain` and `../avr-call-chain.bin` are the DWARF and flash
fixtures used by the AVR stack-scan unwinder tests in
`src/exception_handling/avr.rs`.

The binary predates this branch. It is byte-identical to
`scratch/avr/q11/q11.elf`, a test program of four functions, `level1` through
`level4`, calling each other in a chain, built for the AVR128DA64 with the
avr-gcc toolchain (its DWARF carries avr-libc 2.2.1 and libgcc paths). The
original source project is not available, so the binary cannot be rebuilt and
the tests pin their expectations to it:

- `MEASURED_STACK` in `src/exception_handling/avr.rs` holds stack offsets
  measured from this exact binary.
- The test that finds a `DW_AT_frame_base` of `[DW_OP_REG28]` depends on the
  code shape this toolchain produced.

A replacement fixture with available sources exists as
`scratch/avr/da64/` (a Rust program with the same call-chain shape). Swapping
the fixture over to it means regenerating the binary, updating the measured
offsets, and re-checking every test in `avr.rs`.
