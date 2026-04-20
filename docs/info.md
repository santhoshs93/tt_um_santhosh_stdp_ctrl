<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This design is a standalone digital spike-timing-dependent plasticity (STDP) learning controller. It captures the timestamps of pre- and post-synaptic spikes on an independent timestamp clock, computes the signed time difference `deltaT = post_ts − pre_ts`, looks up the magnitude of the weight change from an 8-entry programmable STDP curve LUT (entries 0–3 = LTP/potentiation, entries 4–7 = LTD/depression, indexed by `|deltaT|`), and scales the result by a configurable learning rate. It also adds three biologically-motivated features beyond plain pair-based STDP: an **R-STDP reward gate** that only releases weight updates when an external reward signal is asserted, an **anti-Hebbian mode** that inverts the LTP/LTD polarity (useful for inhibitory synapses and predictive-coding models), and an **eligibility trace** — a leaky counter that remembers recent plasticity events so that a delayed reward can still credit the correct update.

Pre- and post-synaptic spikes arrive on `ui_in[0]` and `ui_in[1]`, the independent timestamp clock on `ui_in[2]`, a hard learning enable on `ui_in[3]`, and the R-STDP reward on `ui_in[4]`. A 16-bit SPI slave (Mode 0, MSB-first; CS=`uio_in[0]`, MOSI=`uio_in[1]`, MISO=`uio_out[2]`, SCK=`uio_in[3]`) exposes the configuration: `reg_ctrl` at 0x00 (`[0]=enable`, `[1]=reset`, `[2]=single_shot`, `[3]=anti_hebbian`, `[4]=reward_gate_en`, `[5]=trace_en`), `reg_learn_rate` at 0x01 (`[3:0]=learning rate`, `[7:4]=trace decay`), `reg_time_window` at 0x02 (`[3:0]` = `|deltaT|` window width in LUT index units), read-only `reg_pre_ts`/`reg_post_ts`/`reg_delta_t`/`reg_weight_update`/`reg_status` at 0x03–0x07, and the 8-entry STDP curve LUT at 0x08–0x0F. The reset defaults install an exponential curve (LTP: 127/63/31/15, LTD: 100/50/25/12).

On each post-synaptic spike the FSM classifies `deltaT` as LTP or LTD, reads the matching LUT entry, optionally inverts polarity if anti-Hebbian is enabled, right-shifts by the learning-rate field, and applies the reward gate if enabled. `update_ready` (`uo_out[0]`) pulses high when a (reward-gated) update is available, alongside `potentiation` (`uo_out[1]`), `depression` (`uo_out[2]`), a `weight_overflow` flag (`uo_out[3]`) when the LUT saturates at 0xFF, and the low 4 bits of the (reward-gated) weight delta on `uo_out[7:4]`. Four debug taps — `pre_valid`, `post_valid`, and FSM state bits — are exposed on `uio_out[7:4]` for bring-up and silicon debug.

## How to test

1. Apply reset (`rst_n` low, then high) and, over SPI, write the desired `reg_ctrl` (e.g. `0x01` for basic enable, `0x11` to also gate updates on the reward, `0x31` to also enable the eligibility trace, `0x09` for anti-Hebbian + enable), the learning-rate and trace-decay register 0x01, and the time window 0x02.
2. (Optional) Overwrite the STDP LUT at 0x08–0x0F with a custom curve.
3. Assert `learn_enable` (`ui_in[3]`) and drive `ui_in[2]` with the timestamp clock. Issue a pre-synaptic spike on `ui_in[0]`, then a post-synaptic spike on `ui_in[1]` (or vice-versa for pure LTD).
4. When a computation completes, `update_ready` pulses on `uo_out[0]`; the polarity is on `uo_out[2:1]` (LTP/LTD) and the low 4 bits of the gated weight delta appear on `uo_out[7:4]`. Reads of SPI registers 0x05 (`delta_t`), 0x06 (`weight_update`), 0x07 (`status`) return the last computed values.
5. For R-STDP, set `reg_ctrl[4]=1` and drive `ui_in[4]` high during the qualifying spike pair — without reward the output is suppressed but the internal update is still computed (visible via SPI reads).
6. For anti-Hebbian, set `reg_ctrl[3]=1` — the LUT output polarity swaps (what would have been LTP becomes LTD and vice-versa).
7. For the eligibility trace, set `reg_ctrl[5]=1`; the trace increments on each qualifying pair and decays at a rate set by `reg_learn_rate[7:4]`.

A cocotb suite (22 tests in `test/test.py`) covers reset, SPI register roundtrip, LUT programming, single-shot vs continuous, weight overflow, R-STDP gating, anti-Hebbian inversion, eligibility-trace decay, and `uio_oe` direction control. All tests pass in RTL and post-synthesis gate-level simulation.

## External hardware

An SPI master (microcontroller or FPGA) on `uio[0:3]` is required to configure the learning rate, time window, STDP LUT, and control bits. Pre- and post-synaptic spike sources drive `ui_in[0]` and `ui_in[1]`; an independent timestamp clock on `ui_in[2]` allows biological timescales (kHz range) to be used together with the system clock (50 MHz). An external reward signal on `ui_in[4]` is required only when R-STDP (`reg_ctrl[4]=1`) is used. The four output weight-delta bits (`uo_out[7:4]`) feed whatever weight-storage element the user wires up (memristor driver, skyrmion track, SRAM etc.).
