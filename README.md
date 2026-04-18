![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Digital STDP Learning Controller

A digital spike-timing-dependent plasticity (STDP) learning engine for neuromorphic systems. Captures pre- and post-synaptic spike timestamps, computes the signed time difference, and looks up the corresponding weight change from an 8-entry programmable STDP curve LUT. Supports both single-shot and continuous learning modes. All parameters are configurable via SPI.

- [Detailed documentation](docs/info.md)

## Architecture

An external timestamp clock drives an 8-bit counter. Pre- and post-synaptic spike edges are detected via 2-stage synchronizers, and the current timestamp is captured for each. When both timestamps are valid and learning is enabled, a 4-state FSM (IDLE, COMPUTE, UPDATE, DONE) computes delta_t, classifies the result as LTP or LTD, performs LUT lookup, and scales the weight update by a configurable learning rate.

## Key Features

- 8-entry STDP curve LUT (0-3: LTP/potentiation, 4-7: LTD/depression)
- 8-bit timestamp counter with external clock input
- Configurable learning rate (2-bit shift control)
- Configurable time window for STDP computation
- Single-shot and continuous operation modes
- SPI Mode 0 interface for register access
- Debug outputs for FSM state and valid flags

## Pin Summary

| Pin | Direction | Function |
|-----|-----------|----------|
| `ui_in[0]` | Input | Pre-synaptic spike |
| `ui_in[1]` | Input | Post-synaptic spike |
| `ui_in[2]` | Input | Timestamp clock |
| `ui_in[3]` | Input | Learning enable |
| `uo_out[0]` | Output | Update ready |
| `uo_out[1]` | Output | Potentiation (LTP) flag |
| `uo_out[2]` | Output | Depression (LTD) flag |
| `uo_out[3]` | Output | Weight overflow (reserved) |
| `uo_out[7:4]` | Output | Weight delta [3:0] |
| `uio[0]` | Input | SPI CS |
| `uio[1]` | Input | SPI MOSI |
| `uio[2]` | Output | SPI MISO |
| `uio[3]` | Input | SPI SCK |
| `uio[5:4]` | Output | Debug: pre/post valid |
| `uio[7:6]` | Output | Debug: FSM state |

## Simulation

```bash
cd test
make
```

Requires cocotb and Icarus Verilog.

## Target

Tiny Tapeout [TTSKY26a](https://tinytapeout.com) shuttle, 1x1 tile, SkyWater 130 nm.
