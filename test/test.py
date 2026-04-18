# SPDX-FileCopyrightText: © 2026 Prof. Santhosh Sivasubramani, IIT Delhi
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


async def reset_dut(dut):
    """Apply reset and release."""
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0b00001  # CS high
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def spi_write(dut, addr, data):
    """SPI write: 0 + 7-bit addr + 8-bit data, MSB first, Mode 0."""
    cs_bit = 0
    mosi_bit = 1
    sck_bit = 3
    word = ((addr & 0x7F) << 8) | (data & 0xFF)

    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 4)

    for i in range(16):
        bit_val = (word >> (15 - i)) & 1
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 4)
        dut.uio_in.value = (bit_val << mosi_bit) | (1 << sck_bit)
        await ClockCycles(dut.clk, 4)
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 2)

    dut.uio_in.value = (1 << cs_bit)
    await ClockCycles(dut.clk, 4)


async def spi_read(dut, addr):
    """SPI read: 1 + 7-bit addr + 8 clocks to read data."""
    cs_bit = 0
    mosi_bit = 1
    sck_bit = 3
    word = (1 << 15) | ((addr & 0x7F) << 8)

    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 4)

    read_data = 0
    for i in range(16):
        bit_val = (word >> (15 - i)) & 1
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 4)
        dut.uio_in.value = (bit_val << mosi_bit) | (1 << sck_bit)
        await ClockCycles(dut.clk, 2)
        if i >= 8:
            miso = (int(dut.uio_out.value) >> 2) & 1
            read_data = (read_data << 1) | miso
        await ClockCycles(dut.clk, 2)
        dut.uio_in.value = (bit_val << mosi_bit)
        await ClockCycles(dut.clk, 2)

    dut.uio_in.value = (1 << cs_bit)
    await ClockCycles(dut.clk, 4)
    return read_data


async def pulse_ts_clk(dut, n):
    """Pulse the timestamp clock n times via ui_in[2]."""
    for _ in range(n):
        old = int(dut.ui_in.value)
        dut.ui_in.value = old | 0x04   # ts_clk high
        await ClockCycles(dut.clk, 4)
        dut.ui_in.value = old & ~0x04  # ts_clk low
        await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_reset_state(dut):
    """After reset, update_ready should be 0."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    assert (int(dut.uo_out.value) & 1) == 0, "update_ready should be 0 after reset"


@cocotb.test()
async def test_spi_register_access(dut):
    """Write and read back learning rate and time window."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await spi_write(dut, 0x01, 0x0A)  # learn_rate
    await spi_write(dut, 0x02, 0x0F)  # time_window
    await ClockCycles(dut.clk, 10)

    lr = await spi_read(dut, 0x01)
    tw = await spi_read(dut, 0x02)
    dut._log.info(f"learn_rate={lr}, time_window={tw}")
    assert lr == 0x0A, f"Expected learn_rate=0x0A, got 0x{lr:02x}"
    assert tw == 0x0F, f"Expected time_window=0x0F, got 0x{tw:02x}"


@cocotb.test()
async def test_timestamp_capture(dut):
    """Provide pre and post spikes and verify timestamps are captured."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Advance timestamp counter
    await pulse_ts_clk(dut, 5)

    # Send pre-spike (rising edge on ui_in[0])
    dut.ui_in.value = 0b00001  # pre_spike=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b00000  # pre_spike=0
    await ClockCycles(dut.clk, 4)

    # Advance more timestamps
    await pulse_ts_clk(dut, 3)

    # Send post-spike
    dut.ui_in.value = 0b00010  # post_spike=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b00000
    await ClockCycles(dut.clk, 4)

    # Read captured timestamps
    pre_ts = await spi_read(dut, 0x03)
    post_ts = await spi_read(dut, 0x04)
    dut._log.info(f"pre_ts={pre_ts}, post_ts={post_ts}")

    # Pre should be ~5, post should be ~8
    # Exact values depend on sync pipeline latency
    assert post_ts > pre_ts, "Post timestamp should be after pre timestamp"


@cocotb.test()
async def test_stdp_computation(dut):
    """Trigger STDP with pre-before-post (LTP) and verify update_ready."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Configure
    await spi_write(dut, 0x00, 0x01)  # enable
    await spi_write(dut, 0x01, 0x04)  # learn_rate=4
    await spi_write(dut, 0x02, 0x0F)  # time_window=15
    await ClockCycles(dut.clk, 10)

    # Advance timestamps
    await pulse_ts_clk(dut, 3)

    # Pre spike
    dut.ui_in.value = 0b01001  # pre_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000  # learn_en stays on
    await ClockCycles(dut.clk, 4)

    # More timestamps
    await pulse_ts_clk(dut, 2)

    # Post spike
    dut.ui_in.value = 0b01010  # post_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000

    # Poll for update_ready (pulse flag, only HIGH for 2 cycles per FSM pass)
    update_ready = 0
    for _ in range(40):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        if uo & 1:
            update_ready = 1
            break

    potentiation = (uo >> 1) & 1
    depression = (uo >> 2) & 1
    dut._log.info(f"update_ready={update_ready}, LTP={potentiation}, LTD={depression}")
    dut._log.info(f"weight_delta={uo >> 4}")
    assert update_ready == 1, "update_ready should be asserted after STDP computation"
    assert potentiation == 1, "Pre-before-post should produce LTP (potentiation)"
    assert depression == 0, "Should not flag LTD for pre-before-post ordering"


@cocotb.test()
async def test_lut_programming(dut):
    """Write and read back STDP LUT entries."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Write LUT entry 0 (LTP peak) and entry 4 (LTD peak)
    await spi_write(dut, 0x08, 0xFF)  # LUT[0] = 255
    await spi_write(dut, 0x0C, 0xCC)  # LUT[4] = 204
    await ClockCycles(dut.clk, 10)

    val0 = await spi_read(dut, 0x08)
    val4 = await spi_read(dut, 0x0C)
    dut._log.info(f"LUT[0]=0x{val0:02x}, LUT[4]=0x{val4:02x}")
    assert val0 == 0xFF, f"Expected 0xFF, got 0x{val0:02x}"
    assert val4 == 0xCC, f"Expected 0xCC, got 0x{val4:02x}"


@cocotb.test()
async def test_uio_oe(dut):
    """Verify OE direction bits: dynamic MISO, debug outputs."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await ClockCycles(dut.clk, 1)
    # CS=1 (inactive): MISO tri-stated → uio_oe=0xF0
    assert int(dut.uio_oe.value) == 0b11110000, \
        f"Expected uio_oe=0xF0 (CS inactive), got 0x{int(dut.uio_oe.value):02x}"

    # Assert CS=0 (active): MISO enabled → uio_oe=0xF4
    dut.uio_in.value = 0  # CS=0
    await ClockCycles(dut.clk, 1)
    assert int(dut.uio_oe.value) == 0b11110100, \
        f"Expected uio_oe=0xF4 (CS active), got 0x{int(dut.uio_oe.value):02x}"
    dut.uio_in.value = 0b00001  # restore CS=1
    await ClockCycles(dut.clk, 1)


@cocotb.test()
async def test_ltd_depression(dut):
    """Post-before-pre should result in LTD (depression flag)."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Configure
    await spi_write(dut, 0x00, 0x01)  # enable
    await spi_write(dut, 0x01, 0x04)  # learn_rate=4
    await spi_write(dut, 0x02, 0x0F)  # time_window=15

    # Advance timestamps
    await pulse_ts_clk(dut, 3)

    # Post spike FIRST (for LTD)
    dut.ui_in.value = 0b01010  # post_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000  # learn_en stays
    await ClockCycles(dut.clk, 4)

    # More timestamps
    await pulse_ts_clk(dut, 2)

    # Then pre spike (post before pre → negative delta → LTD)
    dut.ui_in.value = 0b01001  # pre_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000

    # Poll for update_ready (pulse flag)
    update_ready = 0
    for _ in range(40):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        if uo & 1:
            update_ready = 1
            break

    potentiation = (uo >> 1) & 1
    depression = (uo >> 2) & 1
    dut._log.info(f"LTD: update_ready={update_ready}, LTP={potentiation}, LTD={depression}")
    assert update_ready == 1, "update_ready should be asserted after STDP computation"
    assert depression == 1, "Post-before-pre should produce LTD (depression)"
    assert potentiation == 0, "Should NOT be LTP in LTD path"


@cocotb.test()
async def test_all_lut_entries(dut):
    """Write distinct values to all 8 STDP LUT entries and read back."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    test_vals = [0xA0, 0xB1, 0xC2, 0xD3, 0xE4, 0xF5, 0x16, 0x27]
    for i, v in enumerate(test_vals):
        await spi_write(dut, 0x08 + i, v)
    await ClockCycles(dut.clk, 10)

    for i, v in enumerate(test_vals):
        rb = await spi_read(dut, 0x08 + i)
        assert rb == v, f"LUT[{i}]: expected 0x{v:02x}, got 0x{rb:02x}"
    dut._log.info("All 8 STDP LUT entries verified")


@cocotb.test()
async def test_single_shot_mode(dut):
    """In single-shot mode, FSM should stay in DONE after one computation."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Enable single-shot: ctrl[2]=1, ctrl[0]=1
    await spi_write(dut, 0x00, 0x05)
    await spi_write(dut, 0x01, 0x04)  # learn_rate
    await spi_write(dut, 0x02, 0x0F)  # time_window

    # Provide pre then post spike
    await pulse_ts_clk(dut, 3)
    dut.ui_in.value = 0b01001  # pre_spike + learn_en
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000
    await pulse_ts_clk(dut, 2)
    dut.ui_in.value = 0b01010  # post_spike + learn_en
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000
    await ClockCycles(dut.clk, 30)

    # Read FSM state from debug pins: uio_out[7:6] = stdp_state
    state1 = (int(dut.uio_out.value) >> 6) & 3
    await ClockCycles(dut.clk, 50)
    state2 = (int(dut.uio_out.value) >> 6) & 3
    dut._log.info(f"Single-shot FSM state: {state1} -> {state2}")
    assert state1 == 3, f"FSM should be in DONE state (3) after computation, got {state1}"
    assert state1 == state2, "FSM should remain in DONE state in single-shot mode"


@cocotb.test()
async def test_timestamp_wraparound(dut):
    """Verify STDP handles timestamp counter wraparound (255→0 boundary)."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Configure
    await spi_write(dut, 0x00, 0x01)  # enable
    await spi_write(dut, 0x01, 0x03)  # learn_rate=3 (full strength)
    await spi_write(dut, 0x02, 0x0F)  # time_window=15

    # Advance timestamp counter close to overflow (250 pulses)
    await pulse_ts_clk(dut, 250)

    # Pre spike at ts~250
    dut.ui_in.value = 0b01001  # pre_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000
    await ClockCycles(dut.clk, 4)

    # Advance 10 more timestamps (wraps from 255 → 0 → ...)
    await pulse_ts_clk(dut, 10)

    # Post spike at ts~4 (wrapped)
    dut.ui_in.value = 0b01010  # post_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000

    # Poll for update_ready
    update_ready = 0
    for _ in range(40):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        if uo & 1:
            update_ready = 1
            break

    potentiation = (uo >> 1) & 1
    depression = (uo >> 2) & 1
    dut._log.info(f"Wraparound: update_ready={update_ready}, LTP={potentiation}, LTD={depression}")
    # FSM should still compute an update (delta wraps but computation still runs)
    assert update_ready == 1, "STDP should still compute even when timestamps wrap around"


@cocotb.test()
async def test_wt_overflow(dut):
    """Program LUT[0]=0xFF and verify wt_overflow output (C2 fix)."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Program ALL LTP entries (0-3) to 0xFF to trigger wt_overflow regardless of exact deltaT
    for i in range(4):
        await spi_write(dut, 0x08 + i, 0xFF)
    await spi_write(dut, 0x00, 0x01)  # enable
    await spi_write(dut, 0x01, 0x03)  # learn_rate=3 (full)
    await spi_write(dut, 0x02, 0x0F)  # time_window=15

    # Pre then post spike (LTP path → uses LUT[0])
    await pulse_ts_clk(dut, 3)
    dut.ui_in.value = 0b01001  # pre_spike + learn_en
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000
    await ClockCycles(dut.clk, 4)
    await pulse_ts_clk(dut, 1)
    dut.ui_in.value = 0b01010  # post_spike + learn_en
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000

    # Poll for update_ready
    wt_overflow = 0
    for _ in range(40):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        if uo & 1:  # update_ready
            wt_overflow = (uo >> 3) & 1
            break

    dut._log.info(f"wt_overflow with LUT[0]=0xFF: {wt_overflow}")
    assert wt_overflow == 1, "wt_overflow should be set when LUT value is 0xFF"


@cocotb.test()
async def test_simultaneous_spikes(dut):
    """Both pre and post spikes arriving on the same cycle."""
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await spi_write(dut, 0x00, 0x01)  # enable
    await spi_write(dut, 0x01, 0x03)  # learn_rate
    await spi_write(dut, 0x02, 0x0F)  # time_window

    await pulse_ts_clk(dut, 5)

    # Both spikes simultaneously
    dut.ui_in.value = 0b01011  # pre_spike=1, post_spike=1, learn_en=1
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = 0b01000  # learn_en only

    # Wait for FSM to process
    update_ready = 0
    for _ in range(40):
        await ClockCycles(dut.clk, 1)
        uo = int(dut.uo_out.value)
        if uo & 1:
            update_ready = 1
            break

    dut._log.info(f"Simultaneous spikes: update_ready={update_ready}")
    # deltaT should be 0 → within window → should still produce an update
    assert update_ready == 1, "Simultaneous spikes should still trigger STDP computation"
