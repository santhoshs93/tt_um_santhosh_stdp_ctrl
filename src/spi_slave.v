/*
 * Copyright (c) 2026 Prof. Santhosh Sivasubramani, IIT Delhi
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI Slave Module - Reusable across all neuromorphic IP blocks
 * Mode 0 (CPOL=0, CPHA=0), MSB first
 * 8-bit address + 8-bit data, active-low CS
 */

`default_nettype none

module spi_slave #(
    parameter NUM_REGS = 16
) (
    input  wire       clk,
    input  wire       rst_n,

    // SPI interface
    input  wire       spi_cs_n,
    input  wire       spi_mosi,
    output wire       spi_miso,
    input  wire       spi_sck,

    // Register interface
    output reg                      wr_en,
    output reg  [7:0]               wr_addr,
    output reg  [7:0]               wr_data,
    output reg  [7:0]               rd_addr,
    input  wire [7:0]               rd_data
);

    // Synchronize SPI signals to clk domain
    reg [2:0] sck_sync;
    reg [1:0] cs_sync;
    reg [1:0] mosi_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_sync  <= 3'b0;
            cs_sync   <= 2'b11;
            mosi_sync <= 2'b0;
        end else begin
            sck_sync  <= {sck_sync[1:0], spi_sck};
            cs_sync   <= {cs_sync[0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
        end
    end

    wire sck_rising  = (sck_sync[2:1] == 2'b01);
    wire sck_falling = (sck_sync[2:1] == 2'b10);
    wire cs_active   = ~cs_sync[1];
    wire mosi_bit    = mosi_sync[1];

    // Bit counter & shift registers
    reg [4:0] bit_cnt;      // 0-15: 8 addr bits + 8 data bits
    reg [7:0] shift_in;
    reg [7:0] shift_out;
    reg       rw_bit;       // 1=read, 0=write (MSB of first byte)
    reg [6:0] addr_latched;

    // MISO output (directly from shift register)
    reg       miso_reg;
    assign spi_miso = cs_active ? miso_reg : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt     <= 5'd0;
            shift_in    <= 8'd0;
            shift_out   <= 8'd0;
            rw_bit      <= 1'b0;
            addr_latched<= 7'd0;
            miso_reg    <= 1'b0;
            wr_en       <= 1'b0;
            wr_addr     <= 8'd0;
            wr_data     <= 8'd0;
            rd_addr     <= 8'd0;
        end else if (!cs_active) begin
            bit_cnt     <= 5'd0;
            wr_en       <= 1'b0;
            miso_reg    <= 1'b0;
        end else begin
            wr_en <= 1'b0;

            if (sck_rising) begin
                // Sample MOSI
                shift_in <= {shift_in[6:0], mosi_bit};
                bit_cnt  <= bit_cnt + 5'd1;

                // After 8 bits: latch address
                if (bit_cnt == 5'd7) begin
                    rw_bit       <= shift_in[6]; // MSB was first bit shifted in
                    addr_latched <= {shift_in[5:0], mosi_bit};
                    rd_addr      <= {1'b0, shift_in[5:0], mosi_bit};
                end

                // After 16 bits: complete transaction
                if (bit_cnt == 5'd15) begin
                    if (!rw_bit) begin
                        // Write transaction
                        wr_en   <= 1'b1;
                        wr_addr <= {1'b0, addr_latched};
                        wr_data <= {shift_in[6:0], mosi_bit};
                    end
                end
            end

            if (sck_falling) begin
                // Update MISO
                if (bit_cnt == 5'd8) begin
                    // Load read data for output
                    shift_out <= rd_data;
                    miso_reg  <= rd_data[7];
                end else if (bit_cnt > 5'd8) begin
                    shift_out <= {shift_out[6:0], 1'b0};
                    miso_reg  <= shift_out[6];
                end
            end
        end
    end

endmodule
