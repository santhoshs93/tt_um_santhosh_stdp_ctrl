/*
 * Copyright (c) 2026 Prof. Santhosh Sivasubramani, IIT Delhi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Digital STDP Learning Controller
 * - Captures pre/post-synaptic spike timestamps
 * - Computes deltaT and applies programmable STDP curve via LUT
 * - Outputs signed weight update (potentiation/depression)
 * - SPI-configurable learning rate, time window, and STDP curve shape
 * - R-STDP reward gate: weight updates gated by external reward signal
 * - Anti-Hebbian mode: inverts LTP/LTD polarity for inhibitory learning
 * - Eligibility trace: leaky counter for delayed reward credit assignment
 * - Standalone IP for any synapse technology (memristor, skyrmion, etc.)
 */

`default_nettype none

module tt_um_santhosh_stdp_ctrl (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ============================================================
    // Input assignments
    // ============================================================
    wire pre_spike   = ui_in[0];   // Pre-synaptic spike input
    wire post_spike  = ui_in[1];   // Post-synaptic spike input
    wire ts_clk      = ui_in[2];   // Timestamp clock (can differ from system clock)
    wire learn_en    = ui_in[3];   // Learning enable
    wire reward      = ui_in[4];   // R-STDP reward signal

    // SPI signals
    wire spi_cs_n = uio_in[0];
    wire spi_mosi = uio_in[1];
    wire spi_miso;
    wire spi_sck  = uio_in[3];

    // ============================================================
    // SPI Slave & Register File
    // ============================================================
    wire        wr_en;
    wire [7:0]  wr_addr, wr_data, rd_addr;
    reg  [7:0]  rd_data;

    spi_slave #(.NUM_REGS(16)) u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .spi_cs_n (spi_cs_n),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso),
        .spi_sck  (spi_sck),
        .wr_en    (wr_en),
        .wr_addr  (wr_addr),
        .wr_data  (wr_data),
        .rd_addr  (rd_addr),
        .rd_data  (rd_data)
    );

    // Configuration registers
    reg [7:0] reg_ctrl;        // 0x00: [0]=enable, [1]=reset, [2]=single_shot,
                               //       [3]=anti_hebbian, [4]=reward_gate_en, [5]=trace_en
    reg [7:0] reg_learn_rate;  // 0x01: [3:0]=learning rate, [7:4]=trace decay rate
    reg [7:0] reg_time_window; // 0x02: [3:0]=STDP time window width
    wire [7:0] reg_pre_ts;     // 0x03: last pre timestamp (read-only)
    wire [7:0] reg_post_ts;    // 0x04: last post timestamp (read-only)
    wire [7:0] reg_delta_t;    // 0x05: computed deltaT (read-only, signed)
    wire [7:0] reg_weight_upd; // 0x06: computed weight update (read-only, signed)
    wire [7:0] reg_status;     // 0x07: status

    // STDP LUT: 8 entries x 8 bits (0x08-0x0F)
    // Entries 0-3: LTP (potentiation), Entries 4-7: LTD (depression)
    reg [7:0] lut_mem [0:7];

    // Register write
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl        <= 8'h01;
            reg_learn_rate  <= 8'h04;
            reg_time_window <= 8'h03;
            // Default STDP curve: exponential decay
            // LTP entries 0-3 (potentiation)
            lut_mem[0] <= 8'd127;  // abs_delta=0: peak potentiation
            lut_mem[1] <= 8'd63;   // abs_delta=1
            lut_mem[2] <= 8'd31;   // abs_delta=2
            lut_mem[3] <= 8'd15;   // abs_delta=3
            // LTD entries 4-7 (depression)
            lut_mem[4] <= 8'd100;  // abs_delta=0: peak depression
            lut_mem[5] <= 8'd50;   // abs_delta=1
            lut_mem[6] <= 8'd25;   // abs_delta=2
            lut_mem[7] <= 8'd12;   // abs_delta=3
        end else if (wr_en) begin
            case (wr_addr)
                8'h00: reg_ctrl        <= wr_data;
                8'h01: reg_learn_rate  <= wr_data;
                8'h02: reg_time_window <= wr_data;
                default: begin
                    if (wr_addr >= 8'h08 && wr_addr <= 8'h0F)
                        lut_mem[wr_addr[2:0]] <= wr_data;
                end
            endcase
        end
    end

    // Register read mux
    always @(*) begin
        case (rd_addr)
            8'h00: rd_data = reg_ctrl;
            8'h01: rd_data = reg_learn_rate;
            8'h02: rd_data = reg_time_window;
            8'h03: rd_data = reg_pre_ts;
            8'h04: rd_data = reg_post_ts;
            8'h05: rd_data = reg_delta_t;
            8'h06: rd_data = reg_weight_upd;
            8'h07: rd_data = reg_status;
            default: begin
                if (rd_addr >= 8'h08 && rd_addr <= 8'h0F)
                    rd_data = lut_mem[rd_addr[2:0]];
                else
                    rd_data = 8'h00;
            end
        endcase
    end

    // ============================================================
    // Timestamp counter (driven by ts_clk, synced to clk domain)
    // ============================================================
    reg [2:0] ts_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ts_sync <= 3'b0;
        else        ts_sync <= {ts_sync[1:0], ts_clk};
    end
    wire ts_rising = (ts_sync[2:1] == 2'b01);

    reg [7:0] ts_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ts_counter <= 8'd0;
        else if (reg_ctrl[1])
            ts_counter <= 8'd0;
        else if (ts_rising)
            ts_counter <= ts_counter + 8'd1;
    end

    // ============================================================
    // Spike detection (edge detection for pre and post spikes)
    // ============================================================
    reg [1:0] pre_sync, post_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre_sync  <= 2'b0;
            post_sync <= 2'b0;
        end else begin
            pre_sync  <= {pre_sync[0], pre_spike};
            post_sync <= {post_sync[0], post_spike};
        end
    end
    wire pre_edge  = (pre_sync  == 2'b01);
    wire post_edge = (post_sync == 2'b01);

    // ============================================================
    // Timestamp capture
    // ============================================================
    reg [7:0] pre_timestamp;
    reg [7:0] post_timestamp;
    reg       pre_valid, post_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre_timestamp  <= 8'd0;
            post_timestamp <= 8'd0;
            pre_valid      <= 1'b0;
            post_valid     <= 1'b0;
        end else if (reg_ctrl[1]) begin
            pre_valid  <= 1'b0;
            post_valid <= 1'b0;
        end else if (clear_valid) begin
            // STDP FSM consumed the spike pair — clear flags (single always block owns pre/post_valid)
            pre_valid  <= 1'b0;
            post_valid <= 1'b0;
        end else begin
            if (pre_edge) begin
                pre_timestamp <= ts_counter;
                pre_valid     <= 1'b1;
            end
            if (post_edge) begin
                post_timestamp <= ts_counter;
                post_valid     <= 1'b1;
            end
        end
    end

    assign reg_pre_ts  = pre_timestamp;
    assign reg_post_ts = post_timestamp;

    // ============================================================
    // STDP Computation FSM
    // ============================================================
    localparam STDP_IDLE    = 2'd0;
    localparam STDP_COMPUTE = 2'd1;
    localparam STDP_UPDATE  = 2'd2;
    localparam STDP_DONE    = 2'd3;

    reg [1:0]  stdp_state;
    reg [7:0]  delta_t;       // Signed: post_ts - pre_ts
    reg        is_ltp;        // 1 if potentiation (pre before post)
    reg [7:0]  abs_delta;
    reg [7:0]  lut_value;
    reg [7:0]  weight_update; // Signed result
    reg        update_ready;
    reg        potentiation;
    reg        depression;
    reg        wt_overflow;
    reg        clear_valid;   // Pulse to consume spike pair (clears pre/post_valid)

    wire [3:0] learn_rate = reg_learn_rate[3:0];
    wire [3:0] time_window = reg_time_window[3:0];
    wire [3:0] trace_decay = reg_learn_rate[7:4];
    wire       anti_hebbian   = reg_ctrl[3];
    wire       reward_gate_en = reg_ctrl[4];
    wire       trace_en       = reg_ctrl[5];

    wire both_valid = pre_valid & post_valid;
    wire stdp_en = learn_en & reg_ctrl[0] & both_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stdp_state    <= STDP_IDLE;
            delta_t       <= 8'd0;
            is_ltp        <= 1'b0;
            abs_delta     <= 8'd0;
            lut_value     <= 8'd0;
            weight_update <= 8'd0;
            update_ready  <= 1'b0;
            potentiation  <= 1'b0;
            depression    <= 1'b0;
            wt_overflow   <= 1'b0;
            clear_valid   <= 1'b0;
        end else if (reg_ctrl[1]) begin
            stdp_state   <= STDP_IDLE;
            update_ready <= 1'b0;
            potentiation <= 1'b0;
            depression   <= 1'b0;
            clear_valid  <= 1'b0;
        end else begin
            case (stdp_state)
                STDP_IDLE: begin
                    update_ready <= 1'b0;
                    clear_valid  <= 1'b0;
                    if (stdp_en) begin
                        delta_t    <= post_timestamp - pre_timestamp;
                        stdp_state <= STDP_COMPUTE;
                    end
                end

                STDP_COMPUTE: begin
                    // Signal timestamp block to consume spike pair
                    clear_valid <= 1'b1;
                    // Determine sign and magnitude
                    // Anti-Hebbian mode inverts LTP/LTD polarity
                    if (delta_t[7]) begin
                        // Negative: post before pre -> LTD normally, LTP in anti-Hebbian
                        is_ltp    <= anti_hebbian;
                        abs_delta <= (~delta_t) + 8'd1; // twos complement
                    end else begin
                        // Positive: pre before post -> LTP normally, LTD in anti-Hebbian
                        is_ltp    <= ~anti_hebbian;
                        abs_delta <= delta_t;
                    end
                    stdp_state <= STDP_UPDATE;
                end

                STDP_UPDATE: begin
                    clear_valid <= 1'b0;
                    // Check if within learning window
                    if (abs_delta <= {4'b0, time_window}) begin
                        // LUT lookup: LTP uses entries 0-3, LTD uses 4-7
                        if (is_ltp) begin
                            lut_value <= lut_mem[abs_delta[1:0]];
                            potentiation <= 1'b1;
                            depression   <= 1'b0;
                            wt_overflow <= (lut_mem[abs_delta[1:0]] == 8'hFF);
                            // Read LUT directly (not through lut_value NBA) for correct pipeline
                            weight_update <= lut_mem[abs_delta[1:0]] >> (4'd3 - learn_rate[1:0]);
                        end else begin
                            lut_value <= lut_mem[abs_delta[1:0] + 3'd4];
                            potentiation <= 1'b0;
                            depression   <= 1'b1;
                            wt_overflow <= (lut_mem[abs_delta[1:0] + 3'd4] == 8'hFF);
                            weight_update <= lut_mem[abs_delta[1:0] + 3'd4] >> (4'd3 - learn_rate[1:0]);
                        end
                    end else begin
                        // Outside window: no update
                        weight_update <= 8'd0;
                        potentiation  <= 1'b0;
                        depression    <= 1'b0;
                        wt_overflow   <= 1'b0;
                    end

                    update_ready <= 1'b1;
                    stdp_state   <= STDP_DONE;
                end

                STDP_DONE: begin
                    if (reg_ctrl[2]) begin
                        // Single-shot mode: stay done until reset
                        stdp_state <= STDP_DONE;
                    end else begin
                        // Continuous mode: go back to idle
                        stdp_state <= STDP_IDLE;
                    end
                end

                default: stdp_state <= STDP_IDLE;
            endcase
        end
    end

    assign reg_delta_t    = delta_t;
    assign reg_weight_upd = weight_update;
    assign reg_status     = {2'b0, trace_nonzero, reward, wt_overflow, depression, potentiation, update_ready};

    // ============================================================
    // Eligibility Trace: leaky counter for delayed reward
    // ============================================================
    reg [7:0] elig_trace;
    wire      trace_nonzero = (elig_trace != 8'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            elig_trace <= 8'd0;
        end else if (reg_ctrl[1]) begin
            elig_trace <= 8'd0;
        end else if (trace_en) begin
            if (update_ready) begin
                // Increment trace on STDP event (saturate at 255)
                if (elig_trace < 8'hFF)
                    elig_trace <= elig_trace + 8'd1;
            end else begin
                // Decay trace when no STDP event this cycle
                if (elig_trace > {4'b0, trace_decay})
                    elig_trace <= elig_trace - {4'b0, trace_decay};
                else
                    elig_trace <= 8'd0;
            end
        end
    end

    // Reward gating logic
    wire reward_ok = ~reward_gate_en | reward;
    // When trace_en: also require trace_nonzero for gated output
    wire update_gated = update_ready & (reward_ok | ~reward_gate_en) &
                        (~trace_en | trace_nonzero | ~reward_gate_en);

    // ============================================================
    // Output assignments
    // ============================================================
    assign uo_out[0]   = update_gated;                 // Weight update ready (reward-gated)
    assign uo_out[1]   = potentiation;                 // LTP
    assign uo_out[2]   = depression;                   // LTD
    assign uo_out[3]   = wt_overflow;                  // Weight overflow
    assign uo_out[7:4] = reward_ok ? weight_update[3:0] : 4'b0000; // Gated weight delta

    assign uio_out[0]  = 1'b0;                     // CS is input
    assign uio_out[1]  = 1'b0;                     // MOSI is input
    assign uio_out[2]  = spi_miso;                 // MISO is output
    assign uio_out[3]  = 1'b0;                     // SCK is input
    assign uio_out[4]  = pre_valid;                // Debug: pre timestamp captured
    assign uio_out[5]  = post_valid;               // Debug: post timestamp captured
    assign uio_out[6]  = stdp_state[0];            // Debug: FSM state bit 0
    assign uio_out[7]  = stdp_state[1];            // Debug: FSM state bit 1

    assign uio_oe = {4'b1111, 1'b0, ~spi_cs_n, 2'b00};
    // uio[7:4]=debug(1), uio[3]=SCK in(0), uio[2]=MISO(dynamic), uio[1]=MOSI in(0), uio[0]=CS in(0)

    // Unused inputs
    wire _unused = &{ena, ui_in[7:5], uio_in[2], uio_in[7:4], 1'b0};

endmodule
