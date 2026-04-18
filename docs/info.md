<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This design implements a digital spike-timing-dependent plasticity (STDP) learning engine. It captures pre- and post-synaptic spike timestamps using an external timestamp clock, computes the signed time difference (deltaT = post_ts - pre_ts), looks up the corresponding weight change from an 8-entry programmable STDP curve LUT (entries 0-3 for potentiation/LTP, 4-7 for depression/LTD), and scales the result by a configurable learning rate. The `weight_overflow` output (uo_out[3]) flags when the LUT produces a saturated value (0xFF). The design supports both single-shot and continuous learning modes, all configurable via SPI.

**Note:** The learning rate register (0x01) uses only bits [1:0] to select the scaling factor: 0→÷8, 1→÷4, 2→÷2, 3→full scale.

## How to test

1. Apply reset and configure via SPI (CS=uio_in[0], MOSI=uio_in[1], SCK=uio_in[3], MISO=uio_out[2]).
2. Write learning rate (reg 0x01), time window (0x02), and optionally program the STDP LUT (0x08-0x0F).
3. Enable learning via CTRL[0] (reg 0x00) and ui_in[3] (learn_enable).
4. Provide a timestamp clock on ui_in[2] and pre/post spikes on ui_in[0]/ui_in[1].
5. After both spikes are captured, the STDP FSM computes the weight update.
6. Monitor update_ready on uo_out[0], potentiation on uo_out[1], depression on uo_out[2], and weight delta on uo_out[7:4].
7. Read computed values via SPI: delta_t (0x05), weight_update (0x06), status (0x07).

## External hardware

SPI master, external timestamp clock source on ui_in[2], pre-synaptic and post-synaptic spike sources on ui_in[0] and ui_in[1].
