// ============================================================================
// TLP Completion Delay — LFSR Jitter for ASM1061 SATA timing emulation
// Real ASM1061 responds in ~200-800ns. FPGA raw is ~10-30ns.
// LFSR provides non-repeating jitter pattern.
// Target: Captain 75T, 125MHz clock (8ns per tick)
// ============================================================================
`timescale 1ns / 1ps

module tlp_completion_delay #(
    parameter CLK_FREQ_MHZ = 125,
    parameter MIN_DELAY_NS = 200,
    parameter MAX_DELAY_NS = 700
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        cpl_valid_in,
    input  logic [95:0] cpl_data_in,
    output logic        cpl_valid_out,
    output logic [95:0] cpl_data_out
);

    // Clock ticks per nanosecond (integer approx)
    localparam integer TICKS_PER_US = CLK_FREQ_MHZ;
    localparam integer MIN_TICKS   = (MIN_DELAY_NS * CLK_FREQ_MHZ) / 1000;
    localparam integer MAX_TICKS   = (MAX_DELAY_NS * CLK_FREQ_MHZ) / 1000;
    localparam integer RANGE       = MAX_TICKS - MIN_TICKS;

    logic [15:0] counter;
    logic [15:0] target_delay;
    logic        pending;
    logic [95:0] held_data;

    // 16-bit LFSR (taps: 16,14,13,11 — maximal length)
    logic [15:0] lfsr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lfsr <= 16'hACE1;
        else
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
    end

    wire [15:0] jitter = (RANGE > 0) ? (lfsr % RANGE[15:0]) : 16'd0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter       <= '0;
            pending       <= 1'b0;
            cpl_valid_out <= 1'b0;
            target_delay  <= '0;
            held_data     <= '0;
            cpl_data_out  <= '0;
        end else begin
            cpl_valid_out <= 1'b0;

            if (cpl_valid_in && !pending) begin
                held_data    <= cpl_data_in;
                target_delay <= MIN_TICKS[15:0] + jitter;
                counter      <= '0;
                pending      <= 1'b1;
            end

            if (pending) begin
                counter <= counter + 16'd1;
                if (counter >= target_delay) begin
                    cpl_valid_out <= 1'b1;
                    cpl_data_out  <= held_data;
                    pending       <= 1'b0;
                end
            end
        end
    end

endmodule
