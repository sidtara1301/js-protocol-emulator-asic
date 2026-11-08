---
title: Protocol Emulator ASIC
author: Your Name
---

## How it works

This is a cycle-accurate, programmable hardware protocol emulator (ASIC) designed for the Jane Street protocol emulator competition. It features a custom 16-bit Instruction Set Architecture (ISA) with concurrent side-set functionality, an execution engine with a 3-phase deterministic protocol-tick, hardware TX/RX FIFO buffers, and a command-based SPI host bridge. 

The CPU fetches packed 16-bit instructions from an on-chip IHP `RM_IHPSG13_1P_1024x32_c2_bm_bist` SRAM macro. By utilizing precise delay opcodes, conditional branching, and MSB/LSB-first shift registers, the engine can deterministically emulate various serial protocols (UART, SPI, I2C) purely through software microcode without requiring dedicated hardware MACs.

## How to test

The ASIC is controlled by an external host microcontroller via a dedicated SPI command interface.

1. **Power & Reset:** Assert active-low reset (`rst_n = 0`) and pull the tile enable high (`ena = 1`).
2. **Load Firmware:** Hold `CS_N` low and send the `CMD_WRITE_IMEM (0x01)` command followed by the 16-bit microcode instructions to program the SRAM.
3. **Execution:** Send the `CMD_CTRL (0x04)` command to issue a `RESET_CPU` and `START` signal. The CPU will begin executing from address 0.
4. **Data I/O:** During runtime, the host can stream data into the TX FIFO using `CMD_WRITE_TX (0x02)` and read received bytes from the RX FIFO using `CMD_READ_RX (0x03)`.
5. **Observation:** Monitor the bidirectional `uio` pins with a logic analyzer to verify cycle-accurate UART, SPI, or I2C protocol waveforms.