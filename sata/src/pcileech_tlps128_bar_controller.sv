//
// PCILeech FPGA.
//
// PCIe BAR PIO controller.
//
// The PCILeech BAR PIO controller allows for easy user-implementation on top
// of the PCILeech AXIS128 PCIe TLP streaming interface.
// The controller consists of a read engine and a write engine and pluggable
// user-implemented PCIe BAR implementations (found at bottom of the file).
//
// Considerations:
// - The core handles 1 DWORD read + 1 DWORD write per CLK max. If a lot of
//   data is written / read from the TLP streaming interface the core may
//   drop packet silently.
// - The core reads 1 DWORD of data (without byte enable) per CLK.
// - The core writes 1 DWORD of data (with byte enable) per CLK.
// - All user-implemented cores must have the same latency in CLKs for the
//   returned read data or else undefined behavior will take place.
// - 32-bit addresses are passed for read/writes. Larger BARs than 4GB are
//   not supported due to addressing constraints. Lower bits (LSBs) are the
//   BAR offset, Higher bits (MSBs) are the 32-bit base address of the BAR.
// - DO NOT edit read/write engines.
// - DO edit pcileech_tlps128_bar_controller (to swap bar implementations).
// - DO edit the bar implementations (at bottom of the file, if neccessary).
//
// Example implementations exists below, swap out any of the example cores
// against a core of your use case, or modify existing cores.
// Following test cores exist (see below in this file):
// - pcileech_bar_impl_zerowrite4k = zero-initialized read/write BAR.
//     It's possible to modify contents by use of .coe file.
// - pcileech_bar_impl_loopaddr = test core that loops back the 32-bit
//     address of the current read. Does not support writes.
// - pcileech_bar_impl_none = core without any reply.
// 
// (c) Ulf Frisk, 2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    input [31:0]            base_address_register,
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out
);
    
    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx[7];
    wire [31:0] bar_rsp_data[7];
    wire        bar_rsp_valid[7];
    
    assign rd_rsp_ctx = bar_rsp_valid[0] ? bar_rsp_ctx[0] :
                        bar_rsp_valid[1] ? bar_rsp_ctx[1] :
                        bar_rsp_valid[2] ? bar_rsp_ctx[2] :
                        bar_rsp_valid[3] ? bar_rsp_ctx[3] :
                        bar_rsp_valid[4] ? bar_rsp_ctx[4] :
                        bar_rsp_valid[5] ? bar_rsp_ctx[5] :
                        bar_rsp_valid[6] ? bar_rsp_ctx[6] : 0;
    assign rd_rsp_data = bar_rsp_valid[0] ? bar_rsp_data[0] :
                        bar_rsp_valid[1] ? bar_rsp_data[1] :
                        bar_rsp_valid[2] ? bar_rsp_data[2] :
                        bar_rsp_valid[3] ? bar_rsp_data[3] :
                        bar_rsp_valid[4] ? bar_rsp_data[4] :
                        bar_rsp_valid[5] ? bar_rsp_data[5] :
                        bar_rsp_valid[6] ? bar_rsp_data[6] : 0;
    assign rd_rsp_valid = bar_rsp_valid[0] || bar_rsp_valid[1] || bar_rsp_valid[2] || bar_rsp_valid[3] || bar_rsp_valid[4] || bar_rsp_valid[5] || bar_rsp_valid[6];
    
    pcileech_bar_impl_none i_bar0(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[0] ),
        .base_address_register ( base_address_register[0] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[0]                ),
        .rd_rsp_data    ( bar_rsp_data[0]               ),
        .rd_rsp_valid   ( bar_rsp_valid[0]              )
    );
    
    pcileech_bar_impl_none i_bar1(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[1] ),
        .base_address_register ( base_address_register[1] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[1]                ),
        .rd_rsp_data    ( bar_rsp_data[1]               ),
        .rd_rsp_valid   ( bar_rsp_valid[1]              )
    );
    
    pcileech_bar_impl_none i_bar2(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[2]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[2] ),
        .base_address_register ( base_address_register[2] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[2]                ),
        .rd_rsp_data    ( bar_rsp_data[2]               ),
        .rd_rsp_valid   ( bar_rsp_valid[2]              )
    );
    
    pcileech_bar_impl_none i_bar3(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[3]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[3] ),
        .base_address_register ( base_address_register[3] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[3]                ),
        .rd_rsp_data    ( bar_rsp_data[3]               ),
        .rd_rsp_valid   ( bar_rsp_valid[3]              )
    );
    
    pcileech_bar_impl_none i_bar4(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[4]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[4] ),
        .base_address_register ( base_address_register[4] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_SATA i_bar5(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[5] ),
        .base_address_register ( base_address_register[5]      ),
        .rd_rsp_ctx            ( bar_rsp_ctx[5]                ),
        .rd_rsp_data           ( bar_rsp_data[5]               ),
        .rd_rsp_valid          ( bar_rsp_valid[5]              )
    );
    
    pcileech_bar_impl_none i_bar6_optrom(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[6]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[6] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[6]                ),
        .rd_rsp_data    ( bar_rsp_data[6]               ),
        .rd_rsp_valid   ( bar_rsp_valid[6]              )
    );


endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule



// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);

    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;
    
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation that does nothing but drop any read/writes
// silently without generating a response.
// This is only recommended for placeholder designs.
// Latency = N/A.
// ------------------------------------------------------------------------
module pcileech_bar_impl_none(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input [31:0] base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    initial rd_rsp_ctx = 0;
    initial rd_rsp_data = 0;
    initial rd_rsp_valid = 0;

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of "address loopback" which can be useful
// for testing. Any read to a specific BAR address will result in the
// address as response.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_loopaddr(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input [87:0]        rd_req_ctx,
    input [31:0]        rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      rd_req_ctx_1;
    bit [31:0]      rd_req_addr_1;
    bit             rd_req_valid_1;
    
    always @ ( posedge clk ) begin
        rd_req_ctx_1    <= rd_req_ctx;
        rd_req_addr_1   <= rd_req_addr;
        rd_req_valid_1  <= rd_req_valid;
        rd_rsp_ctx      <= rd_req_ctx_1;
        rd_rsp_data     <= rd_req_addr_1;
        rd_rsp_valid    <= rd_req_valid_1;
    end    

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of a 4kB writable initial-zero BAR.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_zerowrite4k(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]  drd_req_ctx;
    bit         drd_req_valid;
    wire [31:0] doutb;
    
    always @ ( posedge clk ) begin
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        rd_rsp_data     <= doutb; 
    end
    
    bram_bar_zero4k i_bram_bar_zero4k(
        // Port A - write:
        .addra  ( wr_addr[11:2]     ),
        .clka   ( clk               ),
        .dina   ( wr_data           ),
        .ena    ( wr_valid          ),
        .wea    ( wr_be             ),
        // Port A - read (2 CLK latency):
        .addrb  ( rd_req_addr[11:2] ),
        .clkb   ( clk               ),
        .doutb  ( doutb             ),
        .enb    ( rd_req_valid      )
    );

endmodule

module pcileech_bar_impl_io0(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);
    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit             drd_req_valid;

    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;

    bit [31:0]      data_32;

    always @ (posedge clk) begin
        if (rst)
            
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h57) 
          2'h00 : rd_rsp_data <= 32'h0000E00A;
        default : rd_rsp_data <= 32'h00000000; 
            endcase
       end
    end

endmodule

module pcileech_bar_impl_io1(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);
    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit             drd_req_valid;

    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;

    bit [31:0]      data_32;


    always @ (posedge clk) begin
        if (rst)
          
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h43) 
          2'h00 : rd_rsp_data <= 32'h0000E008;
        default : rd_rsp_data <= 32'h00000000; 
            endcase
       end
    end

endmodule

module pcileech_bar_impl_io2(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);
    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit             drd_req_valid;

    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;

    bit [31:0]      data_32;


    always @ (posedge clk) begin
        if (rst)
            
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h37) 
          2'h00 : rd_rsp_data <= 32'h0000E006;
        default : rd_rsp_data <= 32'h00000000; 
            endcase
       end
    end

endmodule

module pcileech_bar_impl_io3(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);
    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit             drd_req_valid;

    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;

    bit [31:0]      data_32;


    always @ (posedge clk) begin
        if (rst)
            
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h23) 
          2'h00 : rd_rsp_data <= 32'h0000E004;
        default : rd_rsp_data <= 32'h00000000; 
            endcase
       end
    end

endmodule

module pcileech_bar_impl_io4(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);
    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit             drd_req_valid;

    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;

    bit [31:0]      data_32;


    always @ (posedge clk) begin
        if (rst)
         
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h1F) 
          2'h00 : rd_rsp_data <= 32'h0000E000;
          2'h08 : rd_rsp_data <= 32'h0000E001;
          2'h10 : rd_rsp_data <= 32'h0000E002;
          2'h18 : rd_rsp_data <= 32'h0000E003;
        default : rd_rsp_data <= 32'h00000000; 
            endcase
       end
    end

endmodule

// ============================================================================
// pcileech_bar_impl_SATA — ASM1061 AHCI SATA Controller Emulation
// Full AHCI HBA + IDENTIFY DEVICE + Virtual Disk + SMART
// Target: Captain 75T, Donor: ASM1061 (VID:1B21 DID:0612)
// ============================================================================
module pcileech_bar_impl_SATA(
    input               rst,
    input               clk,
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

// Pipeline registers
bit [87:0]  drd_req_ctx;
bit [31:0]  drd_req_addr;
bit         drd_req_valid;
bit [31:0]  dwr_addr;
bit [3:0]   dwr_be;
bit [31:0]  dwr_data;
bit         dwr_valid;

// BSY/DRQ timing — simulates real AHCI command latency
bit [7:0]   busy_cnt0;

// ======================== AHCI HBA Registers ========================
// Global
bit [31:0]  ghc_reg;
bit [31:0]  global_is;       // Interrupt Status (0x08)

// Port 0 only — single port ASM1061 config (PI=0x01)
bit [31:0]  port0_clb;
bit [31:0]  port0_clbu;
bit [31:0]  port0_fb;
bit [31:0]  port0_fbu;
bit [31:0]  port0_is;
bit [31:0]  port0_ie;
bit [31:0]  port0_cmd;
bit [31:0]  port0_tfd;
bit [31:0]  port0_sig;
bit [31:0]  port0_ssts;
bit [31:0]  port0_sctl;
bit [31:0]  port0_serr;
bit [31:0]  port0_sact;
bit [31:0]  port0_ci;

// Address decode helper
wire [31:0] bar_offset = (drd_req_addr - (base_address_register & 32'hFFFFF000));
wire [31:0] wr_offset  = (dwr_addr     - (base_address_register & 32'hFFFFF000));

// ======================== IDENTIFY DEVICE (512 bytes) ========================
// WDC WD2500AAKX-001CA0 — 250GB WD Blue, 7200RPM
reg [15:0] identify_data [0:255];

initial begin : init_identify
    integer i;
    for (i = 0; i < 256; i = i + 1) identify_data[i] = 16'h0000;

    identify_data[0]   = 16'h0040;        // general config: fixed, non-removable
    identify_data[1]   = 16'd16383;       // logical cylinders
    identify_data[3]   = 16'd16;          // heads
    identify_data[6]   = 16'd63;          // sectors per track

    // Serial number (words 10-19): "WD-WMAZ1234567890" (20 chars, byte-swapped)
    identify_data[10]  = "W" | ("D" << 8);
    identify_data[11]  = "-" | ("W" << 8);
    identify_data[12]  = "M" | ("A" << 8);
    identify_data[13]  = "Z" | ("1" << 8);
    identify_data[14]  = "2" | ("3" << 8);
    identify_data[15]  = "4" | ("5" << 8);
    identify_data[16]  = "6" | ("7" << 8);
    identify_data[17]  = "8" | ("9" << 8);
    identify_data[18]  = "0" | (" " << 8);
    identify_data[19]  = " " | (" " << 8);

    // Firmware revision (words 23-26): "01.00A01"
    identify_data[23]  = "0" | ("1" << 8);
    identify_data[24]  = "." | ("0" << 8);
    identify_data[25]  = "0" | ("A" << 8);
    identify_data[26]  = "0" | ("1" << 8);

    // Model number (words 27-46): "WDC WD2500AAKX-001CA0"
    identify_data[27]  = "W" | ("D" << 8);
    identify_data[28]  = "C" | (" " << 8);
    identify_data[29]  = "W" | ("D" << 8);
    identify_data[30]  = "2" | ("5" << 8);
    identify_data[31]  = "0" | ("0" << 8);
    identify_data[32]  = "A" | ("A" << 8);
    identify_data[33]  = "K" | ("X" << 8);
    identify_data[34]  = "-" | ("0" << 8);
    identify_data[35]  = "0" | ("1" << 8);
    identify_data[36]  = "C" | ("A" << 8);
    identify_data[37]  = "0" | (" " << 8);
    for (i = 38; i <= 46; i = i + 1) identify_data[i] = " " | (" " << 8);

    identify_data[47]  = 16'h8000;        // max sectors per R/W multiple: 128
    identify_data[49]  = 16'h2F00;        // capabilities: LBA, DMA, IORDY
    identify_data[50]  = 16'h4001;        // capabilities2
    identify_data[53]  = 16'h0007;        // translation fields valid
    identify_data[59]  = 16'h0110;        // current sectors per multiple
    identify_data[60]  = 16'hE6B0;        // total LBA28 sectors (low)  — 250GB
    identify_data[61]  = 16'h1D1C;        // total LBA28 sectors (high) — 488397168
    identify_data[63]  = 16'h0007;        // multiword DMA modes
    identify_data[64]  = 16'h0003;        // PIO modes supported (3,4)
    identify_data[65]  = 16'h0078;        // min multiword DMA transfer time
    identify_data[66]  = 16'h0078;        // recommended multiword DMA time
    identify_data[67]  = 16'h0078;        // min PIO transfer time (no IORDY)
    identify_data[68]  = 16'h0078;        // min PIO transfer time (IORDY)
    identify_data[75]  = 16'h001F;        // queue depth: 32
    identify_data[76]  = 16'h0E00;        // SATA capabilities (Gen1,Gen2,Gen3)
    identify_data[77]  = 16'h0040;        // SATA additional
    identify_data[78]  = 16'h004C;        // SATA features supported
    identify_data[79]  = 16'h0040;        // SATA features enabled
    identify_data[80]  = 16'h01F0;        // major version: ATA/ATAPI-5,6,7,8
    identify_data[81]  = 16'h0000;        // minor version
    identify_data[82]  = 16'h746B;        // command set supported 1
    identify_data[83]  = 16'h7400;        // command set supported 2: 48bit LBA, FLUSH
    identify_data[84]  = 16'h4100;        // command set supported ext
    identify_data[85]  = 16'h746B;        // command set enabled 1
    identify_data[86]  = 16'h7400;        // command set enabled 2
    identify_data[87]  = 16'h4100;        // command set default
    identify_data[88]  = 16'h407F;        // UDMA modes (0-6, mode 6 active)
    identify_data[93]  = 16'h0000;        // hardware reset result
    identify_data[100] = 16'hE6B0;        // total LBA48 sectors (word 0)  — 250GB
    identify_data[101] = 16'h1D1C;        // total LBA48 sectors (word 1)
    identify_data[102] = 16'h0000;        // total LBA48 sectors (word 2)
    identify_data[103] = 16'h0000;        // total LBA48 sectors (word 3)
    identify_data[106] = 16'h6003;        // physical/logical sector: 4K phys, 512 logical
    identify_data[169] = 16'h0000;        // TRIM not supported (HDD)
    identify_data[217] = 16'd7200;        // nominal rotation rate: 7200 RPM
end

// ======================== SMART Data (512 bytes) ========================
// Minimal SMART: temperature, power-on hours, reallocated, power cycles
reg [15:0] smart_data [0:255];

initial begin : init_smart
    integer j;
    for (j = 0; j < 256; j = j + 1) smart_data[j] = 16'h0000;
    // SMART structure version
    smart_data[0]   = 16'h0010;        // version 16
    // Attribute: Reallocated Sector Count (ID=5)
    smart_data[1]   = {8'd0, 8'd5};    // flags, id
    smart_data[2]   = 16'd200;         // current value
    smart_data[3]   = 16'd200;         // worst value
    smart_data[4]   = 16'h0000;        // raw value low (0 reallocated)
    // Attribute: Power-On Hours (ID=9)
    smart_data[7]   = {8'd0, 8'd9};
    smart_data[8]   = 16'd98;          // current
    smart_data[9]   = 16'd98;          // worst
    smart_data[10]  = 16'd2847;        // raw: 2847 hours (~4 months)
    // Attribute: Power Cycle Count (ID=12)
    smart_data[13]  = {8'd0, 8'd12};
    smart_data[14]  = 16'd100;
    smart_data[15]  = 16'd100;
    smart_data[16]  = 16'd347;         // raw: 347 power cycles
    // Attribute: Temperature (ID=194)
    smart_data[19]  = {8'd0, 8'd194};
    smart_data[20]  = 16'd119;         // current (normalized)
    smart_data[21]  = 16'd107;         // worst
    smart_data[22]  = 16'd33;          // raw: 33°C
end

// ======================== Data Area (8KB in BRAM) ========================
localparam DATA_BASE     = 32'h2000;
localparam DATA_SIZE     = 32'd8192;
localparam IDENTIFY_BASE = 32'h3000;
localparam SMART_BASE    = 32'h3200;

reg [31:0] disk_mem [0:(DATA_SIZE/4)-1];

initial begin : init_disk_mem
    integer k;
    for (k = 0; k < DATA_SIZE/4; k = k + 1) disk_mem[k] = 32'h00000000;
end

// ======================== Main Logic ========================
always @(posedge clk) begin
    if (rst) begin
        // AHCI Global
        ghc_reg     <= 32'h80000001;    // AE=1, AHCI enabled
        global_is   <= 32'h00000000;

        // Port 0 — disk present, link active, ready
        port0_clb   <= 32'h00000000;
        port0_clbu  <= 32'h00000000;
        port0_fb    <= 32'h00000000;
        port0_fbu   <= 32'h00000000;
        port0_is    <= 32'h00000000;
        port0_ie    <= 32'h00000000;
        port0_cmd   <= 32'h00000016;    // FRE=1, FR=1, POD=1, SUD=1
        port0_tfd   <= 32'h00000050;    // BSY=0, DRQ=0, status=0x50 (DRDY+DSC)
        port0_sig   <= 32'h00000101;    // ATA device signature
        port0_ssts  <= 32'h00000113;    // DET=3 (present+comm), SPD=1 (Gen1), IPM=1 (active)
        port0_sctl  <= 32'h00000000;
        port0_serr  <= 32'h00000000;
        port0_sact  <= 32'h00000000;
        port0_ci    <= 32'h00000000;
        busy_cnt0   <= 8'd0;
    end else begin
        // Pipeline inputs
        drd_req_ctx   <= rd_req_ctx;
        drd_req_valid <= rd_req_valid;
        drd_req_addr  <= rd_req_addr;
        dwr_valid     <= wr_valid;
        dwr_addr      <= wr_addr;
        dwr_be        <= wr_be;
        dwr_data      <= wr_data;

        // ============ Write handling ============
        if (dwr_valid) begin
            case (wr_offset[11:0])
                // Global
                12'h004 : ghc_reg    <= dwr_data;     // GHC
                12'h008 : global_is  <= global_is & ~dwr_data;  // IS: RW1C

                // Port 0
                12'h100 : port0_clb  <= dwr_data;
                12'h104 : port0_clbu <= dwr_data;
                12'h108 : port0_fb   <= dwr_data;
                12'h10C : port0_fbu  <= dwr_data;
                12'h110 : port0_is   <= port0_is & ~dwr_data;  // PxIS: RW1C
                12'h114 : port0_ie   <= dwr_data;
                12'h118 : begin
                    port0_cmd <= dwr_data;
                    // CMD write triggers BSY simulation
                    if (dwr_data[0]) begin  // ST bit set
                        port0_tfd[7] <= 1'b1;   // BSY
                        port0_tfd[3] <= 1'b0;   // clear DRQ
                        busy_cnt0    <= 8'd15;   // 15 cycle delay
                    end
                end
                12'h12C : port0_sctl <= dwr_data;
                12'h130 : port0_serr <= port0_serr & ~dwr_data; // PxSERR: RW1C
                12'h134 : port0_sact <= dwr_data;
                12'h138 : begin
                    port0_ci <= dwr_data;
                    // CI write also triggers BSY
                    if (|dwr_data) begin
                        port0_tfd[7] <= 1'b1;
                        port0_tfd[3] <= 1'b0;
                        busy_cnt0    <= 8'd15;
                    end
                end
                default : ; // ignore unknown writes
            endcase

            // Data area writes
            if (wr_offset >= DATA_BASE && wr_offset < (DATA_BASE + DATA_SIZE)) begin
                disk_mem[(wr_offset - DATA_BASE) >> 2] <= dwr_data;
            end
        end

        // ============ BSY/DRQ state machine ============
        if (busy_cnt0 != 0) begin
            busy_cnt0 <= busy_cnt0 - 1;
            if (busy_cnt0 == 8'd1) begin
                port0_tfd[7] <= 1'b0;    // clear BSY
                port0_tfd[3] <= 1'b0;    // DRQ stays clear (command complete)
                port0_tfd[4] <= 1'b1;    // DRDY
                port0_ci     <= 32'h0;   // clear CI — command done
                port0_is[0]  <= 1'b1;    // DHRS — D2H Register FIS Interrupt
            end
        end
    end

    // ============ Read handling ============
    rd_rsp_ctx   <= drd_req_ctx;
    rd_rsp_valid <= drd_req_valid;

    if (drd_req_valid) begin
        // Identify Device area (0x3000-0x31FF)
        if (bar_offset >= IDENTIFY_BASE && bar_offset < (IDENTIFY_BASE + 32'd512)) begin
            rd_rsp_data <= {identify_data[((bar_offset - IDENTIFY_BASE) >> 1) | 1],
                           identify_data[((bar_offset - IDENTIFY_BASE) >> 1) & ~1]};
        end
        // SMART data area (0x3200-0x33FF)
        else if (bar_offset >= SMART_BASE && bar_offset < (SMART_BASE + 32'd512)) begin
            rd_rsp_data <= {smart_data[((bar_offset - SMART_BASE) >> 1) | 1],
                           smart_data[((bar_offset - SMART_BASE) >> 1) & ~1]};
        end
        // Disk data area (0x2000-0x3FFF)
        else if (bar_offset >= DATA_BASE && bar_offset < (DATA_BASE + DATA_SIZE)) begin
            rd_rsp_data <= disk_mem[(bar_offset - DATA_BASE) >> 2];
        end
        // AHCI registers
        else begin
            case (bar_offset[11:0])
                // ---- Global Host Control ----
                12'h000 : rd_rsp_data <= 32'h00011F01;  // CAP: NP=1, NCS=31, SSS=1, ISS=Gen2
                12'h004 : rd_rsp_data <= ghc_reg;
                12'h008 : rd_rsp_data <= global_is;
                12'h00C : rd_rsp_data <= 32'h00000001;  // PI: port 0 only
                12'h010 : rd_rsp_data <= 32'h00010301;  // VS: AHCI 1.3.1
                12'h014 : rd_rsp_data <= 32'h00000000;  // CCC_CTL
                12'h018 : rd_rsp_data <= 32'h00000000;  // CCC_PORTS
                12'h01C : rd_rsp_data <= 32'h00000000;  // EM_LOC
                12'h020 : rd_rsp_data <= 32'h00000000;  // EM_CTL
                12'h024 : rd_rsp_data <= 32'h00000000;  // CAP2
                12'h028 : rd_rsp_data <= 32'h00000000;  // BOHC

                // ---- Port 0 Registers (offset 0x100) ----
                12'h100 : rd_rsp_data <= port0_clb;
                12'h104 : rd_rsp_data <= port0_clbu;
                12'h108 : rd_rsp_data <= port0_fb;
                12'h10C : rd_rsp_data <= port0_fbu;
                12'h110 : rd_rsp_data <= port0_is;
                12'h114 : rd_rsp_data <= port0_ie;
                12'h118 : rd_rsp_data <= port0_cmd;
                12'h11C : rd_rsp_data <= 32'h00000000;  // reserved
                12'h120 : rd_rsp_data <= port0_tfd;
                12'h124 : rd_rsp_data <= port0_sig;
                12'h128 : rd_rsp_data <= port0_ssts;
                12'h12C : rd_rsp_data <= port0_sctl;
                12'h130 : rd_rsp_data <= port0_serr;
                12'h134 : rd_rsp_data <= port0_sact;
                12'h138 : rd_rsp_data <= port0_ci;
                12'h13C : rd_rsp_data <= 32'h00000000;  // PxSNTF
                12'h140 : rd_rsp_data <= 32'h00000000;  // PxFBS

                default : rd_rsp_data <= 32'h00000000;
            endcase
        end
    end else begin
        rd_rsp_data <= 32'h00000000;
    end
end

endmodule
