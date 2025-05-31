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
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out,
    output                  int_enable,
    input                   msix_vaild,
    output                  msix_send_done,
    output [31:0]           msix_address,
    output [31:0]           msix_vector,
    input [31:0]            base_address_register,
    input [31:0]            base_address_register_1,
    input [31:0]            base_address_register_2,
    input [31:0]            base_address_register_3,
    input [31:0]            base_address_register_4,
    input [31:0]            base_address_register_5
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
    wire [3:0]  rd_req_be;
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
        .rd_req_be      ( rd_req_be                     ),
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
    
    
    
    // 这是一个模拟
    assign int_enable = 1;
    
    pcileech_bar_impl_bar0 i_bar0(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .base_address_register (base_address_register_0 ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[0] ),
        .rd_rsp_ctx            ( bar_rsp_ctx[0]                ),
        .rd_rsp_data           ( bar_rsp_data[0]               ),
        .rd_rsp_valid          ( bar_rsp_valid[0]              )
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
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_none i_bar5(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[5] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[5]                ),
        .rd_rsp_data    ( bar_rsp_data[5]               ),
        .rd_rsp_valid   ( bar_rsp_valid[5]              )
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
    output [3:0]            rd_req_be,        
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
    wire [3:0]  rd1_in_be       = tlps_in.tdata[35:32];
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;

    
    wire [3:0]  rd1_out_be;
    wire        rd1_out_be_valid;
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
    fifo_4_4_clk1_bar_rd1 i_fifo_4_4_clk1_bar_rd1 (
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_be                     ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_be                    ),
        .empty          (                               ),
        .valid          ( rd1_out_be_valid              )

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
    wire [3:0]  rd2_in_be       = rd1_out_be;
    wire        rd2_in_be_valid = rd1_out_valid;

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    bit  [3:0]  rd2_out_be;
    bit         rd2_out_be_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
        rd2_out_be       <= rd2_in_be_valid ? rd2_in_be : rd2_out_data;
        rd2_out_be_valid <= rd2_in_be_valid && !rst;  
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit [3:0]   rd3_process_be;
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    assign rd_req_be    = rd3_process_be;
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
                    if ( rd2_out_be_valid ) begin
                        rd3_process_be <= rd2_out_be;
                    end else begin
                        rd3_process_be <= 4'hf;
                    end
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

// 这是一个bar0的tlp回应实现
module pcileech_bar_impl_bar0(
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
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);
                     
    reg [87:0]      drd_req_ctx;
    reg [31:0]      drd_req_addr;
    reg             drd_req_valid;
                  
    reg [31:0]      dwr_addr;
    reg [31:0]      dwr_data;
    reg             dwr_valid;
               
    reg [31:0]      data_32;
              
    time number = 0;
                  
    always @ (posedge clk) begin
        if (rst)
            number <= 0;
               
        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        
        

        if (drd_req_valid) begin
            // 52104004 - 52104000 = 00000004 & FFFF = 0004
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h0FFF)
        16'h0000 : rd_rsp_data <= 32'hB6187FDC;
        16'h0004 : rd_rsp_data <= 32'h638FFBC3;
        16'h0008 : rd_rsp_data <= 32'h31FFB83E;
        16'h000C : rd_rsp_data <= 32'h1FF787EF;
        16'h0010 : rd_rsp_data <= 32'hFE707DFA;
        16'h0014 : rd_rsp_data <= 32'hEE0FDFAB;
        16'h0018 : rd_rsp_data <= 32'hE1FBF5BF;
        16'h001C : rd_rsp_data <= 32'h1FBE57F5;
        16'h0020 : rd_rsp_data <= 32'hF7EA7E5F;
        16'h0024 : rd_rsp_data <= 32'h7DAFEBFC;
        16'h0028 : rd_rsp_data <= 32'hD5FDBFC6;
        16'h002C : rd_rsp_data <= 32'h5FD7F86D;
        16'h0030 : rd_rsp_data <= 32'hFA7F8DDF;
        16'h0034 : rd_rsp_data <= 32'hAFF1DBFD;
        16'h0038 : rd_rsp_data <= 32'hFE1BBFDB;
        16'h003C : rd_rsp_data <= 32'hE3B7FBB0;
        16'h0040 : rd_rsp_data <= 32'h367FB606;
        16'h0044 : rd_rsp_data <= 32'h6FF66062;
        16'h0048 : rd_rsp_data <= 32'hFE6C0C21;
        16'h004C : rd_rsp_data <= 32'hEDC1C414;
        16'h0050 : rd_rsp_data <= 32'hD8184242;
        16'h0054 : rd_rsp_data <= 32'h83882826;
        16'h0058 : rd_rsp_data <= 32'h3185846F;
        16'h005C : rd_rsp_data <= 32'h10504DFB;
        16'h0060 : rd_rsp_data <= 32'h0A09DFB4;
        16'h0064 : rd_rsp_data <= 32'hA19BF643;
        16'h0068 : rd_rsp_data <= 32'h13BE683B;
        16'h006C : rd_rsp_data <= 32'h37ED87BF;
        16'h0070 : rd_rsp_data <= 32'h7DD077FD;
        16'h0074 : rd_rsp_data <= 32'hDA0E7FD4;
        16'h0078 : rd_rsp_data <= 32'hA1EFFA49;
        16'h007C : rd_rsp_data <= 32'h1DFFA99C;
        16'h0080 : rd_rsp_data <= 32'hDFF593CF;
        16'h0084 : rd_rsp_data <= 32'hFE5239FD;
        16'h0088 : rd_rsp_data <= 32'hEA279FD3;
        16'h008C : rd_rsp_data <= 32'hA473FA3C;
        16'h0090 : rd_rsp_data <= 32'h4E3FA7CE;
        16'h0094 : rd_rsp_data <= 32'hE7F479E9;
        16'h0098 : rd_rsp_data <= 32'h7E4F9D9E;
        16'h009C : rd_rsp_data <= 32'hE9F3D3E0;
        16'h00A0 : rd_rsp_data <= 32'h9E3A3C00;
        16'h00A4 : rd_rsp_data <= 32'hE7A7C008;
        16'h00A8 : rd_rsp_data <= 32'h7478018C;
        16'h00AC : rd_rsp_data <= 32'h4F8011CF;
        16'h00B0 : rd_rsp_data <= 32'hF00219F3;
        16'h00B4 : rd_rsp_data <= 32'h00239E3A;
        16'h00B8 : rd_rsp_data <= 32'h0433E7A6;
        16'h00BC : rd_rsp_data <= 32'h463C746F;
        16'h00C0 : rd_rsp_data <= 32'h67CE4DF4;
        16'h00C4 : rd_rsp_data <= 32'h79E9DE46;
        16'h00C8 : rd_rsp_data <= 32'h9D9BE86E;
        16'h00CC : rd_rsp_data <= 32'hD3BD8DE8;
        16'h00D0 : rd_rsp_data <= 32'h37D1DD8D;
        16'h00D4 : rd_rsp_data <= 32'h7A1BD1DF;
        16'h00D8 : rd_rsp_data <= 32'hA3BA1BF2;
        16'h00DC : rd_rsp_data <= 32'h37A3BE2B;
        16'h00E0 : rd_rsp_data <= 32'h7437E5BE;
        16'h00E4 : rd_rsp_data <= 32'h467C57EA;
        16'h00E8 : rd_rsp_data <= 32'h6FCA7DA1;
        16'h00EC : rd_rsp_data <= 32'hF9AFD41D;
        16'h00F0 : rd_rsp_data <= 32'h95FA43DF;
        16'h00F4 : rd_rsp_data <= 32'h5FA83BF6;
        16'h00F8 : rd_rsp_data <= 32'hF587BE60;
        16'h00FC : rd_rsp_data <= 32'h5077EC02;
        16'h0100 : rd_rsp_data <= 32'h0E7DC021;
        16'h0104 : rd_rsp_data <= 32'hEFD80411;
        16'h0108 : rd_rsp_data <= 32'hFB804210;
        16'h010C : rd_rsp_data <= 32'hB0082207;
        16'h0110 : rd_rsp_data <= 32'h0184207F;
        16'h0114 : rd_rsp_data <= 32'h10440FFE;
        16'h0118 : rd_rsp_data <= 32'h0841FFEB;
        16'h011C : rd_rsp_data <= 32'h881FFDB3;
        16'h0120 : rd_rsp_data <= 32'h83FFD636;
        16'h0124 : rd_rsp_data <= 32'h3FFA6665;
        16'h0128 : rd_rsp_data <= 32'hFFAC6C52;
        16'h012C : rd_rsp_data <= 32'hF5CDCA27;
        16'h0130 : rd_rsp_data <= 32'h59D9A474;
        16'h0134 : rd_rsp_data <= 32'h9B944E42;
        16'h0138 : rd_rsp_data <= 32'hB249E82E;
        16'h013C : rd_rsp_data <= 32'h299D85E9;
        16'h0140 : rd_rsp_data <= 32'h93D05D92;
        16'h0144 : rd_rsp_data <= 32'h3A0BD220;
        16'h0148 : rd_rsp_data <= 32'hA1BA2403;
        16'h014C : rd_rsp_data <= 32'h17A4403D;
        16'h0150 : rd_rsp_data <= 32'h744807D3;
        16'h0154 : rd_rsp_data <= 32'h49807A33;
        16'h0158 : rd_rsp_data <= 32'h900FA633;
        16'h015C : rd_rsp_data <= 32'h01F4663A;
        16'h0160 : rd_rsp_data <= 32'h1E4C67A7;
        16'h0164 : rd_rsp_data <= 32'hE9CC7470;
        16'h0168 : rd_rsp_data <= 32'h99CE4E09;
        16'h016C : rd_rsp_data <= 32'h99E9E19A;
        16'h0170 : rd_rsp_data <= 32'h9D9C13A7;
        16'h0174 : rd_rsp_data <= 32'hD3C23478;
        16'h0178 : rd_rsp_data <= 32'h38264F89;
        16'h017C : rd_rsp_data <= 32'h8469F196;
        16'h0180 : rd_rsp_data <= 32'h4D9E126C;
        16'h0184 : rd_rsp_data <= 32'hD3E22DCC;
        16'h0188 : rd_rsp_data <= 32'h3C25D9C4;
        16'h018C : rd_rsp_data <= 32'hC45B984D;
        16'h0190 : rd_rsp_data <= 32'h4BB389D8;
        16'h0194 : rd_rsp_data <= 32'hB6319B82;
        16'h0198 : rd_rsp_data <= 32'h6613B023;
        16'h019C : rd_rsp_data <= 32'h62360432;
        16'h01A0 : rd_rsp_data <= 32'h26604624;
        16'h01A4 : rd_rsp_data <= 32'h6C08644E;
        16'h01A8 : rd_rsp_data <= 32'hC18C49EB;
        16'h01AC : rd_rsp_data <= 32'h11C99DB1;
        16'h01B0 : rd_rsp_data <= 32'h1993D61D;
        16'h01B4 : rd_rsp_data <= 32'h923A63D4;
        16'h01B8 : rd_rsp_data <= 32'h27AC3A40;
        16'h01BC : rd_rsp_data <= 32'h75C7A80D;
        16'h01C0 : rd_rsp_data <= 32'h587581D6;
        16'h01C4 : rd_rsp_data <= 32'h8E501A61;
        16'h01C8 : rd_rsp_data <= 32'hEA03AC13;
        16'h01CC : rd_rsp_data <= 32'hA035C237;
        16'h01D0 : rd_rsp_data <= 32'h0658267E;
        16'h01D4 : rd_rsp_data <= 32'h6B846FEC;
        16'h01D8 : rd_rsp_data <= 32'hB04DFDCD;
        16'h01DC : rd_rsp_data <= 32'h09DFD9D7;
        16'h01E0 : rd_rsp_data <= 32'h9BFB9A79;
        16'h01E4 : rd_rsp_data <= 32'hBFB3AF93;
        16'h01E8 : rd_rsp_data <= 32'hF635F23B;
        16'h01EC : rd_rsp_data <= 32'h665E27B0;
        16'h01F0 : rd_rsp_data <= 32'h6BE4760F;
        16'h01F4 : rd_rsp_data <= 32'hBC4E61F5;
        16'h01F8 : rd_rsp_data <= 32'hC9EC1E57;
        16'h01FC : rd_rsp_data <= 32'h9DC3EA7A;
        16'h0200 : rd_rsp_data <= 32'hD83DAFAA;
        16'h0204 : rd_rsp_data <= 32'h87D5F5AD;
        16'h0208 : rd_rsp_data <= 32'h7A5E55DE;
        16'h020C : rd_rsp_data <= 32'hABEA5BEA;
        16'h0210 : rd_rsp_data <= 32'hBDABBDAC;
        16'h0214 : rd_rsp_data <= 32'hD5B7D5C9;
        16'h0218 : rd_rsp_data <= 32'h567A5990;
        16'h021C : rd_rsp_data <= 32'h6FAB920A;
        16'h0220 : rd_rsp_data <= 32'hF5B221AD;
        16'h0224 : rd_rsp_data <= 32'h562415DE;
        16'h0228 : rd_rsp_data <= 32'h64425BE6;
        16'h022C : rd_rsp_data <= 32'h482BBC65;
        16'h0230 : rd_rsp_data <= 32'h85B7CC53;
        16'h0234 : rd_rsp_data <= 32'h5679CA35;
        16'h0238 : rd_rsp_data <= 32'h6F99A65F;
        16'h023C : rd_rsp_data <= 32'hF3946BF4;
        16'h0240 : rd_rsp_data <= 32'h324DBE46;
        16'h0244 : rd_rsp_data <= 32'h29D7E863;
        16'h0248 : rd_rsp_data <= 32'h9A7D8C39;
        16'h024C : rd_rsp_data <= 32'hAFD1C798;
        16'h0250 : rd_rsp_data <= 32'hFA187388;
        16'h0254 : rd_rsp_data <= 32'hA38E3186;
        16'h0258 : rd_rsp_data <= 32'h31E61069;
        16'h025C : rd_rsp_data <= 32'h1C620D95;
        16'h0260 : rd_rsp_data <= 32'hCC21D25B;
        16'h0264 : rd_rsp_data <= 32'hC41A2BB7;
        16'h0268 : rd_rsp_data <= 32'h43A5B67B;
        16'h026C : rd_rsp_data <= 32'h34566FB5;
        16'h0270 : rd_rsp_data <= 32'h4A6DF651;
        16'h0274 : rd_rsp_data <= 32'hADDE6A1B;
        16'h0278 : rd_rsp_data <= 32'hDBEDA3B0;
        16'h027C : rd_rsp_data <= 32'hBDD4360F;
        16'h0280 : rd_rsp_data <= 32'hDA4661F3;
        16'h0284 : rd_rsp_data <= 32'hA86C1E34;
        16'h0288 : rd_rsp_data <= 32'h8DC3E64A;
        16'h028C : rd_rsp_data <= 32'hD83C69A1;
        16'h0290 : rd_rsp_data <= 32'h87CD9418;
        16'h0294 : rd_rsp_data <= 32'h79D24384;
        16'h0298 : rd_rsp_data <= 32'h9A28304A;
        16'h029C : rd_rsp_data <= 32'hA58609A7;
        16'h02A0 : rd_rsp_data <= 32'h50619479;
        16'h02A4 : rd_rsp_data <= 32'h0C124F98;
        16'h02A8 : rd_rsp_data <= 32'hC229F38F;
        16'h02AC : rd_rsp_data <= 32'h259E31F4;
        16'h02B0 : rd_rsp_data <= 32'h53E61E41;
        16'h02B4 : rd_rsp_data <= 32'h3C63E815;
        16'h02B8 : rd_rsp_data <= 32'hCC3D8250;
        16'h02BC : rd_rsp_data <= 32'hC7D02A06;
        16'h02C0 : rd_rsp_data <= 32'h7A05A06B;
        16'h02C4 : rd_rsp_data <= 32'hA0540DBB;
        16'h02C8 : rd_rsp_data <= 32'h0A41D7B8;
        16'h02CC : rd_rsp_data <= 32'hA81A7780;
        16'h02D0 : rd_rsp_data <= 32'h83AE7000;
        16'h02D4 : rd_rsp_data <= 32'h35EE0009;
        16'h02D8 : rd_rsp_data <= 32'h5DE00193;
        16'h02DC : rd_rsp_data <= 32'hDC00123B;
        16'h02E0 : rd_rsp_data <= 32'hC00227B9;
        16'h02E4 : rd_rsp_data <= 32'h00247794;
        16'h02E8 : rd_rsp_data <= 32'h044E724E;
        16'h02EC : rd_rsp_data <= 32'h49EE29EB;
        16'h02F0 : rd_rsp_data <= 32'h9DE59DBE;
        16'h02F4 : rd_rsp_data <= 32'hDC53D7E9;
        16'h02F8 : rd_rsp_data <= 32'hCA3A7D9E;
        16'h02FC : rd_rsp_data <= 32'hA7AFD3E0;
        16'h0300 : rd_rsp_data <= 32'h75FA3C02;
        16'h0304 : rd_rsp_data <= 32'h5FA7C022;
        16'h0308 : rd_rsp_data <= 32'hF478042B;
        16'h030C : rd_rsp_data <= 32'h4F8045BD;
        16'h0310 : rd_rsp_data <= 32'hF00857DE;
        16'h0314 : rd_rsp_data <= 32'h018A7BE1;
        16'h0318 : rd_rsp_data <= 32'h11AFBC1E;
        16'h031C : rd_rsp_data <= 32'h15F7C3E3;
        16'h0320 : rd_rsp_data <= 32'h5E783C3C;
        16'h0324 : rd_rsp_data <= 32'hEF87C7C7;
        16'h0328 : rd_rsp_data <= 32'hF0787874;
        16'h032C : rd_rsp_data <= 32'h0F8F8E4E;
        16'h0330 : rd_rsp_data <= 32'hF1F1E9E8;
        16'h0334 : rd_rsp_data <= 32'h1E1D9D88;
        16'h0338 : rd_rsp_data <= 32'hE3D3D189;
        16'h033C : rd_rsp_data <= 32'h3A3A1193;
        16'h0340 : rd_rsp_data <= 32'hA7A2123F;
        16'h0344 : rd_rsp_data <= 32'h742227FC;
        16'h0348 : rd_rsp_data <= 32'h44247FC5;
        16'h034C : rd_rsp_data <= 32'h444FF850;
        16'h0350 : rd_rsp_data <= 32'h49FF8A0B;
        16'h0354 : rd_rsp_data <= 32'h9FF1A1B2;
        16'h0358 : rd_rsp_data <= 32'hFE141628;
        16'h035C : rd_rsp_data <= 32'hE2426583;
        16'h0360 : rd_rsp_data <= 32'h282C5039;
        16'h0364 : rd_rsp_data <= 32'h85CA0793;
        16'h0368 : rd_rsp_data <= 32'h59A0723A;
        16'h036C : rd_rsp_data <= 32'h940E27A3;
        16'h0370 : rd_rsp_data <= 32'h41E47439;
        16'h0374 : rd_rsp_data <= 32'h1C4E4790;
        16'h0378 : rd_rsp_data <= 32'hC9E8720B;
        16'h037C : rd_rsp_data <= 32'h9D8E21BC;
        16'h0380 : rd_rsp_data <= 32'hD1E417C1;
        16'h0384 : rd_rsp_data <= 32'h1C427811;
        16'h0388 : rd_rsp_data <= 32'hC82F8213;
        16'h038C : rd_rsp_data <= 32'h85F02237;
        16'h0390 : rd_rsp_data <= 32'h5E042673;
        16'h0394 : rd_rsp_data <= 32'hE0446E36;
        16'h0398 : rd_rsp_data <= 32'h084DE664;
        16'h039C : rd_rsp_data <= 32'h89DC6C4D;
        16'h03A0 : rd_rsp_data <= 32'h9BCDC9D0;
        16'h03A4 : rd_rsp_data <= 32'hB9D99A0A;
        16'h03A8 : rd_rsp_data <= 32'h9B93A1A5;
        16'h03AC : rd_rsp_data <= 32'hB2341453;
        16'h03B0 : rd_rsp_data <= 32'h26424A36;
        16'h03B4 : rd_rsp_data <= 32'h6829A66E;
        16'h03B8 : rd_rsp_data <= 32'h85946DE8;
        16'h03BC : rd_rsp_data <= 32'h524DDD86;
        16'h03C0 : rd_rsp_data <= 32'h29DBD06B;
        16'h03C4 : rd_rsp_data <= 32'h9BBA0DB2;
        16'h03C8 : rd_rsp_data <= 32'hB7A1D625;
        16'h03CC : rd_rsp_data <= 32'h741A6457;
        16'h03D0 : rd_rsp_data <= 32'h43AC4A72;
        16'h03D4 : rd_rsp_data <= 32'h35C9AE2B;
        16'h03D8 : rd_rsp_data <= 32'h5995E5B0;
        16'h03DC : rd_rsp_data <= 32'h925C5609;
        16'h03E0 : rd_rsp_data <= 32'h2BCA6193;
        16'h03E4 : rd_rsp_data <= 32'hB9AC123E;
        16'h03E8 : rd_rsp_data <= 32'h95C227E4;
        16'h03EC : rd_rsp_data <= 32'h58247C4F;
        16'h03F0 : rd_rsp_data <= 32'h844FC9F6;
        16'h03F4 : rd_rsp_data <= 32'h49F99E64;
        16'h03F8 : rd_rsp_data <= 32'h9F93EC49;
        16'h03FC : rd_rsp_data <= 32'hF23DC99B;
        16'h0400 : rd_rsp_data <= 32'h27D993B6;
        16'h0404 : rd_rsp_data <= 32'h7B92366F;
        16'h0408 : rd_rsp_data <= 32'hB2266DFC;
        16'h040C : rd_rsp_data <= 32'h246DDFCA;
        16'h0410 : rd_rsp_data <= 32'h4DDBF9AD;
        16'h0414 : rd_rsp_data <= 32'hDBBF95D7;
        16'h0418 : rd_rsp_data <= 32'hB7F25A75;
        16'h041C : rd_rsp_data <= 32'h7E2BAE5D;
        16'h0420 : rd_rsp_data <= 32'hE5B5EBDD;
        16'h0424 : rd_rsp_data <= 32'h565DBBDD;
        16'h0428 : rd_rsp_data <= 32'h6BD7BBD7;
        16'h042C : rd_rsp_data <= 32'hBA77BA72;
        16'h0430 : rd_rsp_data <= 32'hAE77AE23;
        16'h0434 : rd_rsp_data <= 32'hEE75E436;
        16'h0438 : rd_rsp_data <= 32'hEE5C466E;
        16'h043C : rd_rsp_data <= 32'hEBC86DE0;
        16'h0440 : rd_rsp_data <= 32'hB98DDC01;
        16'h0444 : rd_rsp_data <= 32'h91DBC01C;
        16'h0448 : rd_rsp_data <= 32'h1BB803CE;
        16'h044C : rd_rsp_data <= 32'hB78039E1;
        16'h0450 : rd_rsp_data <= 32'h70079C14;
        16'h0454 : rd_rsp_data <= 32'h0073C245;
        16'h0458 : rd_rsp_data <= 32'h0E382857;
        16'h045C : rd_rsp_data <= 32'hE7858A74;
        16'h0460 : rd_rsp_data <= 32'h7051AE4C;
        16'h0464 : rd_rsp_data <= 32'h0A15E9CC;
        16'h0468 : rd_rsp_data <= 32'hA25D99C9;
        16'h046C : rd_rsp_data <= 32'h2BD3999D;
        16'h0470 : rd_rsp_data <= 32'hBA3393D7;
        16'h0474 : rd_rsp_data <= 32'hA6323A73;
        16'h0478 : rd_rsp_data <= 32'h6627AE30;
        16'h047C : rd_rsp_data <= 32'h6475E605;
        16'h0480 : rd_rsp_data <= 32'h4E5C6054;
        16'h0484 : rd_rsp_data <= 32'hEBCC0A4C;
        16'h0488 : rd_rsp_data <= 32'hB9C1A9C7;
        16'h048C : rd_rsp_data <= 32'h98159877;
        16'h0490 : rd_rsp_data <= 32'h82538E73;
        16'h0494 : rd_rsp_data <= 32'h2A31EE33;
        16'h0498 : rd_rsp_data <= 32'hA61DE634;
        16'h049C : rd_rsp_data <= 32'h63DC6646;
        16'h04A0 : rd_rsp_data <= 32'h3BCC6861;
        16'h04A4 : rd_rsp_data <= 32'hB9CD8C1B;
        16'h04A8 : rd_rsp_data <= 32'h99D1C3B9;
        16'h04AC : rd_rsp_data <= 32'h9A183795;
        16'h04B0 : rd_rsp_data <= 32'hA3867258;
        16'h04B4 : rd_rsp_data <= 32'h306E2B8F;
        16'h04B8 : rd_rsp_data <= 32'h0DE5B1F5;
        16'h04BC : rd_rsp_data <= 32'hDC561E56;
        16'h04C0 : rd_rsp_data <= 32'hCA63EA6B;
        16'h04C4 : rd_rsp_data <= 32'hAC3DADBC;
        16'h04C8 : rd_rsp_data <= 32'hC7D5D7CE;
        16'h04CC : rd_rsp_data <= 32'h7A5A79E0;
        16'h04D0 : rd_rsp_data <= 32'hABAF9C0A;
        16'h04D4 : rd_rsp_data <= 32'hB5F3C1AA;
        16'h04D8 : rd_rsp_data <= 32'h5E3815A3;
        16'h04DC : rd_rsp_data <= 32'hE7825431;
        16'h04E0 : rd_rsp_data <= 32'h702A4619;
        16'h04E4 : rd_rsp_data <= 32'h05A86393;
        16'h04E8 : rd_rsp_data <= 32'h558C3230;
        16'h04EC : rd_rsp_data <= 32'h51C6260D;
        16'h04F0 : rd_rsp_data <= 32'h186461DE;
        16'h04F4 : rd_rsp_data <= 32'h8C4C1BE6;
        16'h04F8 : rd_rsp_data <= 32'hC9C3BC69;
        16'h04FC : rd_rsp_data <= 32'h9837CD91;
        16'h0500 : rd_rsp_data <= 32'h8679D215;
        16'h0504 : rd_rsp_data <= 32'h6F9A225D;
        16'h0508 : rd_rsp_data <= 32'hF3A42BD7;
        16'h050C : rd_rsp_data <= 32'h3445BA72;
        16'h0510 : rd_rsp_data <= 32'h4857AE23;
        16'h0514 : rd_rsp_data <= 32'h8A75E434;
        16'h0518 : rd_rsp_data <= 32'hAE5C4648;
        16'h051C : rd_rsp_data <= 32'hEBC86986;
        16'h0520 : rd_rsp_data <= 32'hB98D9061;
        16'h0524 : rd_rsp_data <= 32'h91D20C1C;
        16'h0528 : rd_rsp_data <= 32'h1A21C3CE;
        16'h052C : rd_rsp_data <= 32'hA41839ED;
        16'h0530 : rd_rsp_data <= 32'h43879DD8;
        16'h0534 : rd_rsp_data <= 32'h3073DB8B;
        16'h0538 : rd_rsp_data <= 32'h0E3BB1BF;
        16'h053C : rd_rsp_data <= 32'hE7B617F1;
        16'h0540 : rd_rsp_data <= 32'h76627E15;
        16'h0544 : rd_rsp_data <= 32'h6C2FE255;
        16'h0548 : rd_rsp_data <= 32'hC5FC2A50;
        16'h054C : rd_rsp_data <= 32'h5FC5AA03;
        16'h0550 : rd_rsp_data <= 32'hF855A030;
        16'h0554 : rd_rsp_data <= 32'h8A540608;
        16'h0558 : rd_rsp_data <= 32'hAA40618E;
        16'h055C : rd_rsp_data <= 32'hA80C11EA;
        16'h0560 : rd_rsp_data <= 32'h81C21DAA;
        16'h0564 : rd_rsp_data <= 32'h1823D5A1;
        16'h0568 : rd_rsp_data <= 32'h843A5418;
        16'h056C : rd_rsp_data <= 32'h47AA4380;
        16'h0570 : rd_rsp_data <= 32'h75A8300E;
        16'h0574 : rd_rsp_data <= 32'h558601EE;
        16'h0578 : rd_rsp_data <= 32'h50601DE8;
        16'h057C : rd_rsp_data <= 32'h0C03DD8B;
        16'h0580 : rd_rsp_data <= 32'hC03BD1B8;
        16'h0584 : rd_rsp_data <= 32'h07BA1783;
        16'h0588 : rd_rsp_data <= 32'h77A27034;
        16'h058C : rd_rsp_data <= 32'h742E064E;
        16'h0590 : rd_rsp_data <= 32'h45E069E6;
        16'h0594 : rd_rsp_data <= 32'h5C0D9C6D;
        16'h0598 : rd_rsp_data <= 32'hC1D3CDD6;
        16'h059C : rd_rsp_data <= 32'h1A39DA68;
        16'h05A0 : rd_rsp_data <= 32'hA79BAD8F;
        16'h05A4 : rd_rsp_data <= 32'h73B5D1F8;
        16'h05A8 : rd_rsp_data <= 32'h365A1F89;
        16'h05AC : rd_rsp_data <= 32'h6BA3F194;
        16'h05B0 : rd_rsp_data <= 32'hB43E124B;
        16'h05B4 : rd_rsp_data <= 32'h47E229BE;
        16'h05B8 : rd_rsp_data <= 32'h7C2597E0;
        16'h05BC : rd_rsp_data <= 32'hC4527C0E;
        16'h05C0 : rd_rsp_data <= 32'h4A2FC1E3;
        16'h05C4 : rd_rsp_data <= 32'hA5F81C38;
        16'h05C8 : rd_rsp_data <= 32'h5F83C785;
        16'h05CC : rd_rsp_data <= 32'hF0387050;
        16'h05D0 : rd_rsp_data <= 32'h078E0A0D;
        16'h05D4 : rd_rsp_data <= 32'h71E1A1DE;
        16'h05D8 : rd_rsp_data <= 32'h1C141BE5;
        16'h05DC : rd_rsp_data <= 32'hC243BC52;
        16'h05E0 : rd_rsp_data <= 32'h2837CA29;
        16'h05E4 : rd_rsp_data <= 32'h8679A594;
        16'h05E8 : rd_rsp_data <= 32'h6F945243;
        16'h05EC : rd_rsp_data <= 32'hF24A283A;
        16'h05F0 : rd_rsp_data <= 32'h29A587A1;
        16'h05F4 : rd_rsp_data <= 32'h9450741D;
        16'h05F8 : rd_rsp_data <= 32'h4A0E43DB;
        16'h05FC : rd_rsp_data <= 32'hA1E83BB5;
        16'h0600 : rd_rsp_data <= 32'h1D87B65F;
        16'h0604 : rd_rsp_data <= 32'hD0766BFD;
        16'h0608 : rd_rsp_data <= 32'h0E6DBFDD;
        16'h060C : rd_rsp_data <= 32'hEDD7FBD0;
        16'h0610 : rd_rsp_data <= 32'hDA7FBA0B;
        16'h0614 : rd_rsp_data <= 32'hAFF7A1B8;
        16'h0618 : rd_rsp_data <= 32'hFE741786;
        16'h061C : rd_rsp_data <= 32'hEE42706E;
        16'h0620 : rd_rsp_data <= 32'hE82E0DE2;
        16'h0624 : rd_rsp_data <= 32'h85E1DC24;
        16'h0628 : rd_rsp_data <= 32'h5C1BC442;
        16'h062C : rd_rsp_data <= 32'hC3B8482C;
        16'h0630 : rd_rsp_data <= 32'h378985C0;
        16'h0634 : rd_rsp_data <= 32'h71905800;
        16'h0638 : rd_rsp_data <= 32'h120B8009;
        16'h063C : rd_rsp_data <= 32'h21B00193;
        16'h0640 : rd_rsp_data <= 32'h1600123A;
        16'h0644 : rd_rsp_data <= 32'h600227AB;
        16'h0648 : rd_rsp_data <= 32'h002475B5;
        16'h064C : rd_rsp_data <= 32'h044E5653;
        16'h0650 : rd_rsp_data <= 32'h49EA6A30;
        16'h0654 : rd_rsp_data <= 32'h9DADA608;
        16'h0658 : rd_rsp_data <= 32'hD5D46182;
        16'h065C : rd_rsp_data <= 32'h5A4C1025;
        16'h0660 : rd_rsp_data <= 32'hA9C20453;
        16'h0664 : rd_rsp_data <= 32'h98204A32;
        16'h0668 : rd_rsp_data <= 32'h8409A629;
        16'h066C : rd_rsp_data <= 32'h41946599;
        16'h0670 : rd_rsp_data <= 32'h124C5397;
        16'h0674 : rd_rsp_data <= 32'h29CA327D;
        16'h0678 : rd_rsp_data <= 32'h99A62FD3;
        16'h067C : rd_rsp_data <= 32'h9465FA3F;
        16'h0680 : rd_rsp_data <= 32'h4C5FA7F1;
        16'h0684 : rd_rsp_data <= 32'hCBF47E1F;
        16'h0688 : rd_rsp_data <= 32'hBE4FE3F3;
        16'h068C : rd_rsp_data <= 32'hE9FC3E3C;
        16'h0690 : rd_rsp_data <= 32'h9FC7E7CD;
        16'h0694 : rd_rsp_data <= 32'hF87C79DA;
        16'h0698 : rd_rsp_data <= 32'h8FCF9BAB;
        16'h069C : rd_rsp_data <= 32'hF9F3B5B4;
        16'h06A0 : rd_rsp_data <= 32'h9E365648;
        16'h06A4 : rd_rsp_data <= 32'hE66A698B;
        16'h06A8 : rd_rsp_data <= 32'h6DAD91B1;
        16'h06AC : rd_rsp_data <= 32'hD5D2161E;
        16'h06B0 : rd_rsp_data <= 32'h5A2263ED;
        16'h06B4 : rd_rsp_data <= 32'hA42C3DD6;
        16'h06B8 : rd_rsp_data <= 32'h45C7DA61;
        16'h06BC : rd_rsp_data <= 32'h587BAC16;
        16'h06C0 : rd_rsp_data <= 32'h8FB5C264;
        16'h06C4 : rd_rsp_data <= 32'hF6582C40;
        16'h06C8 : rd_rsp_data <= 32'h6B85C806;
        16'h06CC : rd_rsp_data <= 32'hB0598068;
        16'h06D0 : rd_rsp_data <= 32'h0B900D8D;
        16'h06D4 : rd_rsp_data <= 32'hB201D1D2;
        16'h06D8 : rd_rsp_data <= 32'h201A1A2C;
        16'h06DC : rd_rsp_data <= 32'h03A3A5C4;
        16'h06E0 : rd_rsp_data <= 32'h34345848;
        16'h06E4 : rd_rsp_data <= 32'h464B8982;
        16'h06E8 : rd_rsp_data <= 32'h69B1902E;
        16'h06EC : rd_rsp_data <= 32'h961205EF;
        16'h06F0 : rd_rsp_data <= 32'h62205DFD;
        16'h06F4 : rd_rsp_data <= 32'h240BDFDB;
        16'h06F8 : rd_rsp_data <= 32'h41BBFBBD;
        16'h06FC : rd_rsp_data <= 32'h17BFB7D2;
        16'h0700 : rd_rsp_data <= 32'h77F67A2F;
        16'h0704 : rd_rsp_data <= 32'h7E6FA5FC;
        16'h0708 : rd_rsp_data <= 32'hEDF45FCD;
        16'h070C : rd_rsp_data <= 32'hDE4BF9DB;
        16'h0710 : rd_rsp_data <= 32'hE9BF9BBA;
        16'h0714 : rd_rsp_data <= 32'h97F3B7AD;
        16'h0718 : rd_rsp_data <= 32'h7E3675D8;
        16'h071C : rd_rsp_data <= 32'hE66E5B81;
        16'h0720 : rd_rsp_data <= 32'h6DEBB017;
        16'h0724 : rd_rsp_data <= 32'hDDB60275;
        16'h0728 : rd_rsp_data <= 32'hD6602E5E;
        16'h072C : rd_rsp_data <= 32'h6C05EBE7;
        16'h0730 : rd_rsp_data <= 32'hC05DBC78;
        16'h0734 : rd_rsp_data <= 32'h0BD7CF8D;
        16'h0738 : rd_rsp_data <= 32'hBA79F1DC;
        16'h073C : rd_rsp_data <= 32'hAF9E1BC5;
        16'h0740 : rd_rsp_data <= 32'hF3E3B853;
        16'h0744 : rd_rsp_data <= 32'h3C378A39;
        16'h0748 : rd_rsp_data <= 32'hC671A791;
        16'h074C : rd_rsp_data <= 32'h6E14721B;
        16'h0750 : rd_rsp_data <= 32'hE24E23BA;
        16'h0754 : rd_rsp_data <= 32'h29E437A1;
        16'h0758 : rd_rsp_data <= 32'h9C467418;
        16'h075C : rd_rsp_data <= 32'hC86E4381;
        16'h0760 : rd_rsp_data <= 32'h8DE8301C;
        16'h0764 : rd_rsp_data <= 32'hDD8603C4;
        16'h0768 : rd_rsp_data <= 32'hD060384C;
        16'h076C : rd_rsp_data <= 32'h0C0789CA;
        16'h0770 : rd_rsp_data <= 32'hC07199A3;
        16'h0774 : rd_rsp_data <= 32'h0E139433;
        16'h0778 : rd_rsp_data <= 32'hE232463A;
        16'h077C : rd_rsp_data <= 32'h262867AE;
        16'h0780 : rd_rsp_data <= 32'h658C75E5;
        16'h0784 : rd_rsp_data <= 32'h51CE5C50;
        16'h0788 : rd_rsp_data <= 32'h19EBCA00;
        16'h078C : rd_rsp_data <= 32'h9DB9A006;
        16'h0790 : rd_rsp_data <= 32'hD7940060;
        16'h0794 : rd_rsp_data <= 32'h72400C0E;
        16'h0798 : rd_rsp_data <= 32'h2801C1E5;
        16'h079C : rd_rsp_data <= 32'h80181C53;
        16'h07A0 : rd_rsp_data <= 32'h0383CA30;
        16'h07A4 : rd_rsp_data <= 32'h3039A60D;
        16'h07A8 : rd_rsp_data <= 32'h079461D9;
        16'h07AC : rd_rsp_data <= 32'h724C1B92;
        16'h07B0 : rd_rsp_data <= 32'h29C3B22B;
        16'h07B4 : rd_rsp_data <= 32'h983625B5;
        16'h07B8 : rd_rsp_data <= 32'h8664565E;
        16'h07BC : rd_rsp_data <= 32'h6C4A6BEA;
        16'h07C0 : rd_rsp_data <= 32'hC9ADBDAB;
        16'h07C4 : rd_rsp_data <= 32'h95D7D5B2;
        16'h07C8 : rd_rsp_data <= 32'h5A7A562B;
        16'h07CC : rd_rsp_data <= 32'hAFAA65B1;
        16'h07D0 : rd_rsp_data <= 32'hF5AC561A;
        16'h07D4 : rd_rsp_data <= 32'h55CA63A4;
        16'h07D8 : rd_rsp_data <= 32'h59AC3443;
        16'h07DC : rd_rsp_data <= 32'h95C64836;
        16'h07E0 : rd_rsp_data <= 32'h58698669;
        16'h07E4 : rd_rsp_data <= 32'h8D906D9D;
        16'h07E8 : rd_rsp_data <= 32'hD20DD3D8;
        16'h07EC : rd_rsp_data <= 32'h21DA3B81;
        16'h07F0 : rd_rsp_data <= 32'h1BA7B01F;
        16'h07F4 : rd_rsp_data <= 32'hB47603FE;
        16'h07F8 : rd_rsp_data <= 32'h4E603FEB;
        16'h07FC : rd_rsp_data <= 32'hEC07FDB6;
        16'h0800 : rd_rsp_data <= 32'hC07FD661;
        16'h0804 : rd_rsp_data <= 32'h0FFA6C15;
        16'h0808 : rd_rsp_data <= 32'hFFADC25F;
        16'h080C : rd_rsp_data <= 32'hF5D82BF9;
        16'h0810 : rd_rsp_data <= 32'h5B85BF98;
        16'h0814 : rd_rsp_data <= 32'hB057F385;
        16'h0818 : rd_rsp_data <= 32'h0A7E3053;
        16'h081C : rd_rsp_data <= 32'hAFE60A37;
        16'h0820 : rd_rsp_data <= 32'hFC61A67F;
        16'h0824 : rd_rsp_data <= 32'hCC146FF4;
        16'h0828 : rd_rsp_data <= 32'hC24DFE48;
        16'h082C : rd_rsp_data <= 32'h29DFE98A;
        16'h0830 : rd_rsp_data <= 32'h9BFD91AA;
        16'h0834 : rd_rsp_data <= 32'hBFD215A3;
        16'h0838 : rd_rsp_data <= 32'hFA22543E;
        16'h083C : rd_rsp_data <= 32'hA42A47EA;
        16'h0840 : rd_rsp_data <= 32'h45A87DA7;
        16'h0844 : rd_rsp_data <= 32'h558FD47E;
        16'h0848 : rd_rsp_data <= 32'h51FA4FED;
        16'h084C : rd_rsp_data <= 32'h1FA9FDDC;
        16'h0850 : rd_rsp_data <= 32'hF59FDBC0;
        16'h0854 : rd_rsp_data <= 32'h53FBB80E;
        16'h0858 : rd_rsp_data <= 32'h3FB781EC;
        16'h085C : rd_rsp_data <= 32'hF6701DC1;
        16'h0860 : rd_rsp_data <= 32'h6E03D81B;
        16'h0864 : rd_rsp_data <= 32'hE03B83BE;
        16'h0868 : rd_rsp_data <= 32'h07B037E5;
        16'h086C : rd_rsp_data <= 32'h76067C5F;
        16'h0870 : rd_rsp_data <= 32'h606FCBF6;
        16'h0874 : rd_rsp_data <= 32'h0DF9BE60;
        16'h0878 : rd_rsp_data <= 32'hDF97EC02;
        16'h087C : rd_rsp_data <= 32'hF27DC020;
        16'h0880 : rd_rsp_data <= 32'h2FD8040B;
        16'h0884 : rd_rsp_data <= 32'hFB8041B4;
        16'h0888 : rd_rsp_data <= 32'hB008164A;
        16'h088C : rd_rsp_data <= 32'h018269A4;
        16'h0890 : rd_rsp_data <= 32'h102D9448;
        16'h0894 : rd_rsp_data <= 32'h05D24983;
        16'h0898 : rd_rsp_data <= 32'h5A29903F;
        16'h089C : rd_rsp_data <= 32'hA59207F5;
        16'h08A0 : rd_rsp_data <= 32'h52207E58;
        16'h08A4 : rd_rsp_data <= 32'h240FEB85;
        16'h08A8 : rd_rsp_data <= 32'h41FDB055;
        16'h08AC : rd_rsp_data <= 32'h1FD60A54;
        16'h08B0 : rd_rsp_data <= 32'hFA61AA47;
        16'h08B4 : rd_rsp_data <= 32'hAC15A871;
        16'h08B8 : rd_rsp_data <= 32'hC2558E13;
        16'h08BC : rd_rsp_data <= 32'h2A51E236;
        16'h08C0 : rd_rsp_data <= 32'hAA1C2667;
        16'h08C4 : rd_rsp_data <= 32'hA3C46C70;
        16'h08C8 : rd_rsp_data <= 32'h384DCE0C;
        16'h08CC : rd_rsp_data <= 32'h89D9E1C0;
        16'h08D0 : rd_rsp_data <= 32'h9B9C180E;
        16'h08D4 : rd_rsp_data <= 32'hB3C381E0;
        16'h08D8 : rd_rsp_data <= 32'h38301C09;
        16'h08DC : rd_rsp_data <= 32'h8603C19C;
        16'h08E0 : rd_rsp_data <= 32'h603813C4;
        16'h08E4 : rd_rsp_data <= 32'h07823842;
        16'h08E8 : rd_rsp_data <= 32'h7027882B;
        16'h08EC : rd_rsp_data <= 32'h047185B5;
        16'h08F0 : rd_rsp_data <= 32'h4E105657;
        16'h08F4 : rd_rsp_data <= 32'hE20A6A77;
        16'h08F8 : rd_rsp_data <= 32'h21ADAE7A;
        16'h08FC : rd_rsp_data <= 32'h15D5EFA8;
        16'h0900 : rd_rsp_data <= 32'h5A5DF58C;
        16'h0904 : rd_rsp_data <= 32'hABDE51C4;
        16'h0908 : rd_rsp_data <= 32'hBBEA1845;
        16'h090C : rd_rsp_data <= 32'hBDA38854;
        16'h0910 : rd_rsp_data <= 32'hD4318A4A;
        16'h0914 : rd_rsp_data <= 32'h4611A9AB;
        16'h0918 : rd_rsp_data <= 32'h621595B9;
        16'h091C : rd_rsp_data <= 32'h22525797;
        16'h0920 : rd_rsp_data <= 32'h2A2A727A;
        16'h0924 : rd_rsp_data <= 32'hA5AE2FA8;
        16'h0928 : rd_rsp_data <= 32'h55E5F589;
        16'h092C : rd_rsp_data <= 32'h5C5E5195;
        16'h0930 : rd_rsp_data <= 32'hCBEA1252;
        16'h0934 : rd_rsp_data <= 32'hBDA22A2A;
        16'h0938 : rd_rsp_data <= 32'hD425A5AF;
        16'h093C : rd_rsp_data <= 32'h445455F1;
        16'h0940 : rd_rsp_data <= 32'h4A4A5E12;
        16'h0944 : rd_rsp_data <= 32'hA9ABE226;
        16'h0948 : rd_rsp_data <= 32'h95BC246F;
        16'h094C : rd_rsp_data <= 32'h57C44DFE;
        16'h0950 : rd_rsp_data <= 32'h7849DFE5;
        16'h0954 : rd_rsp_data <= 32'h899BFC53;
        16'h0958 : rd_rsp_data <= 32'h93BFCA33;
        16'h095C : rd_rsp_data <= 32'h37F9A638;
        16'h0960 : rd_rsp_data <= 32'h7F94678A;
        16'h0964 : rd_rsp_data <= 32'hF24C71A4;
        16'h0968 : rd_rsp_data <= 32'h29CE144F;
        16'h096C : rd_rsp_data <= 32'h99E249F5;
        16'h0970 : rd_rsp_data <= 32'h9C299E54;
        16'h0974 : rd_rsp_data <= 32'hC593EA41;
        16'h0978 : rd_rsp_data <= 32'h523DA815;
        16'h097C : rd_rsp_data <= 32'h27D58254;
        16'h0980 : rd_rsp_data <= 32'h7A502A44;
        16'h0984 : rd_rsp_data <= 32'hAA05A841;
        16'h0988 : rd_rsp_data <= 32'hA0558812;
        16'h098C : rd_rsp_data <= 32'h0A51822D;
        16'h0990 : rd_rsp_data <= 32'hAA1025DA;
        16'h0994 : rd_rsp_data <= 32'hA2045BAD;
        16'h0998 : rd_rsp_data <= 32'h204BB5DC;
        16'h099C : rd_rsp_data <= 32'h09B65BC0;
        16'h09A0 : rd_rsp_data <= 32'h966BB809;
        16'h09A4 : rd_rsp_data <= 32'h6DB7819F;
        16'h09A8 : rd_rsp_data <= 32'hD67013FA;
        16'h09AC : rd_rsp_data <= 32'h6E023FA3;
        16'h09B0 : rd_rsp_data <= 32'hE027F438;
        16'h09B4 : rd_rsp_data <= 32'h047E4789;
        16'h09B8 : rd_rsp_data <= 32'h4FE87194;
        16'h09BC : rd_rsp_data <= 32'hFD8E1248;
        16'h09C0 : rd_rsp_data <= 32'hD1E22981;
        16'h09C4 : rd_rsp_data <= 32'h1C25901A;
        16'h09C8 : rd_rsp_data <= 32'hC45203A0;
        16'h09CC : rd_rsp_data <= 32'h4A203405;
        16'h09D0 : rd_rsp_data <= 32'hA4064055;
        16'h09D4 : rd_rsp_data <= 32'h40680A57;
        16'h09D8 : rd_rsp_data <= 32'h0D81AA75;
        16'h09DC : rd_rsp_data <= 32'hD015AE5B;
        16'h09E0 : rd_rsp_data <= 32'h0255EBB6;
        16'h09E4 : rd_rsp_data <= 32'h2A5DB664;
        16'h09E8 : rd_rsp_data <= 32'hABD66C47;
        16'h09EC : rd_rsp_data <= 32'hBA6DC87B;
        16'h09F0 : rd_rsp_data <= 32'hADD98FB4;
        16'h09F4 : rd_rsp_data <= 32'hDB91F64E;
        16'h09F8 : rd_rsp_data <= 32'hB21E69E6;
        16'h09FC : rd_rsp_data <= 32'h23ED9C63;
        16'h0A00 : rd_rsp_data <= 32'h3DD3CC3B;
        16'h0A04 : rd_rsp_data <= 32'hDA39C7BA;
        16'h0A08 : rd_rsp_data <= 32'hA79877AB;
        16'h0A0C : rd_rsp_data <= 32'h738E75B5;
        16'h0A10 : rd_rsp_data <= 32'h31EE5650;
        16'h0A14 : rd_rsp_data <= 32'h1DEA6A00;
        16'h0A18 : rd_rsp_data <= 32'hDDADA00D;
        16'h0A1C : rd_rsp_data <= 32'hD5D401D7;
        16'h0A20 : rd_rsp_data <= 32'h5A401A73;
        16'h0A24 : rd_rsp_data <= 32'hA803AE3E;
        16'h0A28 : rd_rsp_data <= 32'h8035E7E2;
        16'h0A2C : rd_rsp_data <= 32'h065C7C28;
        16'h0A30 : rd_rsp_data <= 32'h6BCFC587;
        16'h0A34 : rd_rsp_data <= 32'hB9F8507D;
        16'h0A38 : rd_rsp_data <= 32'h9F8A0FD8;
        16'h0A3C : rd_rsp_data <= 32'hF1A1FB89;
        16'h0A40 : rd_rsp_data <= 32'h141FB197;
        16'h0A44 : rd_rsp_data <= 32'h43F6127F;
        16'h0A48 : rd_rsp_data <= 32'h3E622FFC;
        16'h0A4C : rd_rsp_data <= 32'hEC25FFC8;
        16'h0A50 : rd_rsp_data <= 32'hC45FF987;
        16'h0A54 : rd_rsp_data <= 32'h4BFF907B;
        16'h0A58 : rd_rsp_data <= 32'hBFF20FB2;
        16'h0A5C : rd_rsp_data <= 32'hFE21F628;
        16'h0A60 : rd_rsp_data <= 32'hE41E6587;
        16'h0A64 : rd_rsp_data <= 32'h43EC507E;
        16'h0A68 : rd_rsp_data <= 32'h3DCA0FEE;
        16'h0A6C : rd_rsp_data <= 32'hD9A1FDEB;
        16'h0A70 : rd_rsp_data <= 32'h941FDDBC;
        16'h0A74 : rd_rsp_data <= 32'h43FBD7CE;
        16'h0A78 : rd_rsp_data <= 32'h3FBA79E7;
        16'h0A7C : rd_rsp_data <= 32'hF7AF9C74;
        16'h0A80 : rd_rsp_data <= 32'h75F3CE4A;
        16'h0A84 : rd_rsp_data <= 32'h5E39E9AC;
        16'h0A88 : rd_rsp_data <= 32'hE79D95CA;
        16'h0A8C : rd_rsp_data <= 32'h73D259AE;
        16'h0A90 : rd_rsp_data <= 32'h3A2B95EC;
        16'h0A94 : rd_rsp_data <= 32'hA5B25DCB;
        16'h0A98 : rd_rsp_data <= 32'h562BD9BD;
        16'h0A9C : rd_rsp_data <= 32'h65BB97DE;
        16'h0AA0 : rd_rsp_data <= 32'h57B27BEC;
        16'h0AA4 : rd_rsp_data <= 32'h762FBDC4;
        16'h0AA8 : rd_rsp_data <= 32'h65F7D84D;
        16'h0AAC : rd_rsp_data <= 32'h5E7B89DA;
        16'h0AB0 : rd_rsp_data <= 32'hEFB19BA2;
        16'h0AB4 : rd_rsp_data <= 32'hF613B42D;
        16'h0AB8 : rd_rsp_data <= 32'h623645D0;
        16'h0ABC : rd_rsp_data <= 32'h26685A0A;
        16'h0AC0 : rd_rsp_data <= 32'h6D8BA1A3;
        16'h0AC4 : rd_rsp_data <= 32'hD1B4143D;
        16'h0AC8 : rd_rsp_data <= 32'h164247DE;
        16'h0ACC : rd_rsp_data <= 32'h68287BE5;
        16'h0AD0 : rd_rsp_data <= 32'h858FBC5B;
        16'h0AD4 : rd_rsp_data <= 32'h51F7CBB1;
        16'h0AD8 : rd_rsp_data <= 32'h1E79B612;
        16'h0ADC : rd_rsp_data <= 32'hEF966227;
        16'h0AE0 : rd_rsp_data <= 32'hF26C2478;
        16'h0AE4 : rd_rsp_data <= 32'h2DC44F8F;
        16'h0AE8 : rd_rsp_data <= 32'hD849F1F5;
        16'h0AEC : rd_rsp_data <= 32'h899E1E5A;
        16'h0AF0 : rd_rsp_data <= 32'h93E3EBAE;
        16'h0AF4 : rd_rsp_data <= 32'h3C3DB5E4;
        16'h0AF8 : rd_rsp_data <= 32'hC7D65C41;
        16'h0AFC : rd_rsp_data <= 32'h7A6BC815;
        16'h0B00 : rd_rsp_data <= 32'hADB98253;
        16'h0B04 : rd_rsp_data <= 32'hD7902A30;
        16'h0B08 : rd_rsp_data <= 32'h7205A60B;
        16'h0B0C : rd_rsp_data <= 32'h205461B8;
        16'h0B10 : rd_rsp_data <= 32'h0A4C178F;
        16'h0B14 : rd_rsp_data <= 32'hA9C271F6;
        16'h0B18 : rd_rsp_data <= 32'h982E1E67;
        16'h0B1C : rd_rsp_data <= 32'h85E3EC72;
        16'h0B20 : rd_rsp_data <= 32'h5C3DCE27;
        16'h0B24 : rd_rsp_data <= 32'hC7D9E474;
        16'h0B28 : rd_rsp_data <= 32'h7B9C4E45;
        16'h0B2C : rd_rsp_data <= 32'hB3C9E85A;
        16'h0B30 : rd_rsp_data <= 32'h399D8BAF;
        16'h0B34 : rd_rsp_data <= 32'h93D1B5FF;
        16'h0B38 : rd_rsp_data <= 32'h3A165FF8;
        16'h0B3C : rd_rsp_data <= 32'hA26BFF82;
        16'h0B40 : rd_rsp_data <= 32'h2DBFF02C;
        16'h0B44 : rd_rsp_data <= 32'hD7FE05C9;
        16'h0B48 : rd_rsp_data <= 32'h7FE05995;
        16'h0B4C : rd_rsp_data <= 32'hFC0B925D;
        16'h0B50 : rd_rsp_data <= 32'hC1B22BD2;
        16'h0B54 : rd_rsp_data <= 32'h1625BA2B;
        16'h0B58 : rd_rsp_data <= 32'h6457A5B8;
        16'h0B5C : rd_rsp_data <= 32'h4A745784;
        16'h0B60 : rd_rsp_data <= 32'hAE4A704F;
        16'h0B64 : rd_rsp_data <= 32'hE9AE09FC;
        16'h0B68 : rd_rsp_data <= 32'h95E19FC9;
        16'h0B6C : rd_rsp_data <= 32'h5C13F999;
        16'h0B70 : rd_rsp_data <= 32'hC23F9399;
        16'h0B74 : rd_rsp_data <= 32'h27F23396;
        16'h0B78 : rd_rsp_data <= 32'h7E263269;
        16'h0B7C : rd_rsp_data <= 32'hE4662D9E;
        16'h0B80 : rd_rsp_data <= 32'h4C65D3E1;
        16'h0B84 : rd_rsp_data <= 32'hCC5A3C15;
        16'h0B88 : rd_rsp_data <= 32'hCBA7C250;
        16'h0B8C : rd_rsp_data <= 32'hB4782A02;
        16'h0B90 : rd_rsp_data <= 32'h4F85A025;
        16'h0B94 : rd_rsp_data <= 32'hF0540453;
        16'h0B98 : rd_rsp_data <= 32'h0A404A3D;
        16'h0B9C : rd_rsp_data <= 32'hA809A7D5;
        16'h0BA0 : rd_rsp_data <= 32'h81947A5A;
        16'h0BA4 : rd_rsp_data <= 32'h124FABA6;
        16'h0BA8 : rd_rsp_data <= 32'h29F5B46B;
        16'h0BAC : rd_rsp_data <= 32'h9E564DBC;
        16'h0BB0 : rd_rsp_data <= 32'hEA69D7C0;
        16'h0BB4 : rd_rsp_data <= 32'hAD9A7807;
        16'h0BB8 : rd_rsp_data <= 32'hD3AF8070;
        16'h0BBC : rd_rsp_data <= 32'h35F00E09;
        16'h0BC0 : rd_rsp_data <= 32'h5E01E19C;
        16'h0BC4 : rd_rsp_data <= 32'hE01C13C5;
        16'h0BC8 : rd_rsp_data <= 32'h03C23855;
        16'h0BCC : rd_rsp_data <= 32'h38278A5B;
        16'h0BD0 : rd_rsp_data <= 32'h8471ABB5;
        16'h0BD4 : rd_rsp_data <= 32'h4E15B65B;
        16'h0BD8 : rd_rsp_data <= 32'hE2566BB7;
        16'h0BDC : rd_rsp_data <= 32'h2A6DB67D;
        16'h0BE0 : rd_rsp_data <= 32'hADD66FD5;
        16'h0BE4 : rd_rsp_data <= 32'hDA6DFA56;
        16'h0BE8 : rd_rsp_data <= 32'hADDFAA6F;
        16'h0BEC : rd_rsp_data <= 32'hDBF5ADF3;
        16'h0BF0 : rd_rsp_data <= 32'hBE55DE38;
        16'h0BF4 : rd_rsp_data <= 32'hEA5BE788;
        16'h0BF8 : rd_rsp_data <= 32'hABBC7183;
        16'h0BFC : rd_rsp_data <= 32'hB7CE1033;
        16'h0C00 : rd_rsp_data <= 32'h79E2063A;
        16'h0C04 : rd_rsp_data <= 32'h9C2067A0;
        16'h0C08 : rd_rsp_data <= 32'hC40C740A;
        16'h0C0C : rd_rsp_data <= 32'h41CE41AF;
        16'h0C10 : rd_rsp_data <= 32'h19E815F0;
        16'h0C14 : rd_rsp_data <= 32'h9D825E05;
        16'h0C18 : rd_rsp_data <= 32'hD02BE054;
        16'h0C1C : rd_rsp_data <= 32'h05BC0A41;
        16'h0C20 : rd_rsp_data <= 32'h57C1A81A;
        16'h0C24 : rd_rsp_data <= 32'h781583A6;
        16'h0C28 : rd_rsp_data <= 32'h82503462;
        16'h0C2C : rd_rsp_data <= 32'h2A064C23;
        16'h0C30 : rd_rsp_data <= 32'hA069C436;
        16'h0C34 : rd_rsp_data <= 32'h0D98466F;
        16'h0C38 : rd_rsp_data <= 32'hD3886DFC;
        16'h0C3C : rd_rsp_data <= 32'h318DDFCC;
        16'h0C40 : rd_rsp_data <= 32'h11DBF9C3;
        16'h0C44 : rd_rsp_data <= 32'h1BBF9839;
        16'h0C48 : rd_rsp_data <= 32'hB7F38797;
        16'h0C4C : rd_rsp_data <= 32'h7E30727D;
        16'h0C50 : rd_rsp_data <= 32'hE60E2FDC;
        16'h0C54 : rd_rsp_data <= 32'h61E5FBC9;
        16'h0C58 : rd_rsp_data <= 32'h1C5FB998;
        16'h0C5C : rd_rsp_data <= 32'hCBF7938C;
        16'h0C60 : rd_rsp_data <= 32'hBE7231CB;
        16'h0C64 : rd_rsp_data <= 32'hEE2619BE;
        16'h0C68 : rd_rsp_data <= 32'hE46397EA;
        16'h0C6C : rd_rsp_data <= 32'h4C327DA2;
        16'h0C70 : rd_rsp_data <= 32'hC62FD424;
        16'h0C74 : rd_rsp_data <= 32'h65FA444F;
        16'h0C78 : rd_rsp_data <= 32'h5FA849F2;
        16'h0C7C : rd_rsp_data <= 32'hF5899E28;
        16'h0C80 : rd_rsp_data <= 32'h5193E582;
        16'h0C84 : rd_rsp_data <= 32'h123C5029;
        16'h0C88 : rd_rsp_data <= 32'h27CA0597;
        16'h0C8C : rd_rsp_data <= 32'h79A0527E;
        16'h0C90 : rd_rsp_data <= 32'h940A2FE5;
        16'h0C94 : rd_rsp_data <= 32'h41A5FC52;
        16'h0C98 : rd_rsp_data <= 32'h145FCA20;
        16'h0C9C : rd_rsp_data <= 32'h4BF9A40A;
        16'h0CA0 : rd_rsp_data <= 32'hBF9441A7;
        16'h0CA4 : rd_rsp_data <= 32'hF2481476;
        16'h0CA8 : rd_rsp_data <= 32'h29824E6F;
        16'h0CAC : rd_rsp_data <= 32'h9029EDF3;
        16'h0CB0 : rd_rsp_data <= 32'h059DDE32;
        16'h0CB4 : rd_rsp_data <= 32'h53DBE622;
        16'h0CB8 : rd_rsp_data <= 32'h3BBC6427;
        16'h0CBC : rd_rsp_data <= 32'hB7CC4477;
        16'h0CC0 : rd_rsp_data <= 32'h79C84E79;
        16'h0CC4 : rd_rsp_data <= 32'h9989EF95;
        16'h0CC8 : rd_rsp_data <= 32'h919DF25F;
        16'h0CCC : rd_rsp_data <= 32'h13DE2BF8;
        16'h0CD0 : rd_rsp_data <= 32'h3BE5BF89;
        16'h0CD4 : rd_rsp_data <= 32'hBC57F190;
        16'h0CD8 : rd_rsp_data <= 32'hCA7E1203;
        16'h0CDC : rd_rsp_data <= 32'hAFE22036;
        16'h0CE0 : rd_rsp_data <= 32'hFC240669;
        16'h0CE4 : rd_rsp_data <= 32'hC4406D9F;
        16'h0CE8 : rd_rsp_data <= 32'h480DD3F9;
        16'h0CEC : rd_rsp_data <= 32'h81DA3F96;
        16'h0CF0 : rd_rsp_data <= 32'h1BA7F26B;
        16'h0CF4 : rd_rsp_data <= 32'hB47E2DB8;
        16'h0CF8 : rd_rsp_data <= 32'h4FE5D786;
        16'h0CFC : rd_rsp_data <= 32'hFC5A7060;
        16'h0D00 : rd_rsp_data <= 32'hCBAE0C0B;
        16'h0D04 : rd_rsp_data <= 32'hB5E1C1B1;
        16'h0D08 : rd_rsp_data <= 32'h5C18161F;
        16'h0D0C : rd_rsp_data <= 32'hC38263F2;
        16'h0D10 : rd_rsp_data <= 32'h302C3E2F;
        16'h0D14 : rd_rsp_data <= 32'h05C7E5F8;
        16'h0D18 : rd_rsp_data <= 32'h587C5F8E;
        16'h0D1C : rd_rsp_data <= 32'h8FCBF1E9;
        16'h0D20 : rd_rsp_data <= 32'hF9BE1D9F;
        16'h0D24 : rd_rsp_data <= 32'h97E3D3F8;
        16'h0D28 : rd_rsp_data <= 32'h7C3A3F81;
        16'h0D2C : rd_rsp_data <= 32'hC7A7F01A;
        16'h0D30 : rd_rsp_data <= 32'h747E03AA;
        16'h0D34 : rd_rsp_data <= 32'h4FE035A4;
        16'h0D38 : rd_rsp_data <= 32'hFC065440;
        16'h0D3C : rd_rsp_data <= 32'hC06A480C;
        16'h0D40 : rd_rsp_data <= 32'h0DA981CC;
        16'h0D44 : rd_rsp_data <= 32'hD59019C3;
        16'h0D48 : rd_rsp_data <= 32'h5203983B;
        16'h0D4C : rd_rsp_data <= 32'h203387B8;
        16'h0D50 : rd_rsp_data <= 32'h0630778C;
        16'h0D54 : rd_rsp_data <= 32'h660E71C4;
        16'h0D58 : rd_rsp_data <= 32'h61EE1840;
        16'h0D5C : rd_rsp_data <= 32'h1DE38803;
        16'h0D60 : rd_rsp_data <= 32'hDC318035;
        16'h0D64 : rd_rsp_data <= 32'hC610065B;
        16'h0D68 : rd_rsp_data <= 32'h62006BB8;
        16'h0D6C : rd_rsp_data <= 32'h200DB78D;
        16'h0D70 : rd_rsp_data <= 32'h01D671D0;
        16'h0D74 : rd_rsp_data <= 32'h1A6E1A0C;
        16'h0D78 : rd_rsp_data <= 32'hADE3A1C5;
        16'h0D7C : rd_rsp_data <= 32'hDC341851;
        16'h0D80 : rd_rsp_data <= 32'hC6438A1E;
        16'h0D84 : rd_rsp_data <= 32'h6831A3E4;
        16'h0D88 : rd_rsp_data <= 32'h86143C4F;
        16'h0D8C : rd_rsp_data <= 32'h6247C9FD;
        16'h0D90 : rd_rsp_data <= 32'h28799FDD;
        16'h0D94 : rd_rsp_data <= 32'h8F93FBD4;
        16'h0D98 : rd_rsp_data <= 32'hF23FBA4B;
        16'h0D9C : rd_rsp_data <= 32'h27F7A9B5;
        16'h0DA0 : rd_rsp_data <= 32'h7E75965A;
        16'h0DA4 : rd_rsp_data <= 32'hEE526BAF;
        16'h0DA8 : rd_rsp_data <= 32'hEA2DB5F8;
        16'h0DAC : rd_rsp_data <= 32'hA5D65F89;
        16'h0DB0 : rd_rsp_data <= 32'h5A6BF19B;
        16'h0DB4 : rd_rsp_data <= 32'hADBE13B5;
        16'h0DB8 : rd_rsp_data <= 32'hD7E2365B;
        16'h0DBC : rd_rsp_data <= 32'h7C266BB9;
        16'h0DC0 : rd_rsp_data <= 32'hC46DB796;
        16'h0DC4 : rd_rsp_data <= 32'h4DD67261;
        16'h0DC8 : rd_rsp_data <= 32'hDA6E2C1C;
        16'h0DCC : rd_rsp_data <= 32'hADE5C3C9;
        16'h0DD0 : rd_rsp_data <= 32'hDC58399C;
        16'h0DD4 : rd_rsp_data <= 32'hCB8793C0;
        16'h0DD8 : rd_rsp_data <= 32'hB0723802;
        16'h0DDC : rd_rsp_data <= 32'h0E278025;
        16'h0DE0 : rd_rsp_data <= 32'hE470045D;
        16'h0DE4 : rd_rsp_data <= 32'h4E004BDE;
        16'h0DE8 : rd_rsp_data <= 32'hE009BBE6;
        16'h0DEC : rd_rsp_data <= 32'h0197BC61;
        16'h0DF0 : rd_rsp_data <= 32'h1277CC1C;
        16'h0DF4 : rd_rsp_data <= 32'h2E79C3C4;
        16'h0DF8 : rd_rsp_data <= 32'hEF983847;
        16'h0DFC : rd_rsp_data <= 32'hF387887D;
        16'h0E00 : rd_rsp_data <= 32'h30718FDC;
        16'h0E04 : rd_rsp_data <= 32'h0E11FBCC;
        16'h0E08 : rd_rsp_data <= 32'hE21FB9C4;
        16'h0E0C : rd_rsp_data <= 32'h23F79840;
        16'h0E10 : rd_rsp_data <= 32'h3E738802;
        16'h0E14 : rd_rsp_data <= 32'hEE31802F;
        16'h0E18 : rd_rsp_data <= 32'hE61005F0;
        16'h0E1C : rd_rsp_data <= 32'h62005E0B;
        16'h0E20 : rd_rsp_data <= 32'h200BE1BD;
        16'h0E24 : rd_rsp_data <= 32'h01BC17D0;
        16'h0E28 : rd_rsp_data <= 32'h17C27A09;
        16'h0E2C : rd_rsp_data <= 32'h782FA190;
        16'h0E30 : rd_rsp_data <= 32'h85F41200;
        16'h0E34 : rd_rsp_data <= 32'h5E422000;
        16'h0E38 : rd_rsp_data <= 32'hE8240006;
        16'h0E3C : rd_rsp_data <= 32'h84400061;
        16'h0E40 : rd_rsp_data <= 32'h48000C17;
        16'h0E44 : rd_rsp_data <= 32'h8001C276;
        16'h0E48 : rd_rsp_data <= 32'h00182E61;
        16'h0E4C : rd_rsp_data <= 32'h0385EC1A;
        16'h0E50 : rd_rsp_data <= 32'h305DC3A0;
        16'h0E54 : rd_rsp_data <= 32'h0BD83407;
        16'h0E58 : rd_rsp_data <= 32'hBB864079;
        16'h0E5C : rd_rsp_data <= 32'hB0680F97;
        16'h0E60 : rd_rsp_data <= 32'h0D81F277;
        16'h0E64 : rd_rsp_data <= 32'hD01E2E7E;
        16'h0E68 : rd_rsp_data <= 32'h03E5EFED;
        16'h0E6C : rd_rsp_data <= 32'h3C5DFDDD;
        16'h0E70 : rd_rsp_data <= 32'hCBDFDBDC;
        16'h0E74 : rd_rsp_data <= 32'hBBFBBBCE;
        16'h0E78 : rd_rsp_data <= 32'hBFB7B9E8;
        16'h0E7C : rd_rsp_data <= 32'hF6779D8B;
        16'h0E80 : rd_rsp_data <= 32'h6E73D1B6;
        16'h0E84 : rd_rsp_data <= 32'hEE3A1661;
        16'h0E88 : rd_rsp_data <= 32'hE7A26C13;
        16'h0E8C : rd_rsp_data <= 32'h742DC232;
        16'h0E90 : rd_rsp_data <= 32'h45D82625;
        16'h0E94 : rd_rsp_data <= 32'h5B846459;
        16'h0E98 : rd_rsp_data <= 32'hB04C4B9B;
        16'h0E9C : rd_rsp_data <= 32'h09C9B3BF;
        16'h0EA0 : rd_rsp_data <= 32'h999637F8;
        16'h0EA4 : rd_rsp_data <= 32'h92667F8D;
        16'h0EA8 : rd_rsp_data <= 32'h2C6FF1DC;
        16'h0EAC : rd_rsp_data <= 32'hCDFE1BC6;
        16'h0EB0 : rd_rsp_data <= 32'hDFE3B863;
        16'h0EB4 : rd_rsp_data <= 32'hFC378C32;
        16'h0EB8 : rd_rsp_data <= 32'hC671C620;
        16'h0EBC : rd_rsp_data <= 32'h6E18640D;
        16'h0EC0 : rd_rsp_data <= 32'hE38C41D7;
        16'h0EC4 : rd_rsp_data <= 32'h31C81A71;
        16'h0EC8 : rd_rsp_data <= 32'h1983AE1B;
        16'h0ECC : rd_rsp_data <= 32'h9035E3BE;
        16'h0ED0 : rd_rsp_data <= 32'h065C37ED;
        16'h0ED4 : rd_rsp_data <= 32'h6BC67DDF;
        16'h0ED8 : rd_rsp_data <= 32'hB86FDBFD;
        16'h0EDC : rd_rsp_data <= 32'h8DFBBFD4;
        16'h0EE0 : rd_rsp_data <= 32'hDFB7FA4D;
        16'h0EE4 : rd_rsp_data <= 32'hF67FA9D5;
        16'h0EE8 : rd_rsp_data <= 32'h6FF59A50;
        16'h0EEC : rd_rsp_data <= 32'hFE53AA01;
        16'h0EF0 : rd_rsp_data <= 32'hEA35A016;
        16'h0EF4 : rd_rsp_data <= 32'hA6540266;
        16'h0EF8 : rd_rsp_data <= 32'h6A402C60;
        16'h0EFC : rd_rsp_data <= 32'hA805CC00;
        16'h0F00 : rd_rsp_data <= 32'h8059C00A;
        16'h0F04 : rd_rsp_data <= 32'h0B9801AD;
        16'h0F08 : rd_rsp_data <= 32'hB38015D7;
        16'h0F0C : rd_rsp_data <= 32'h30025A7A;
        16'h0F10 : rd_rsp_data <= 32'h002BAFA8;
        16'h0F14 : rd_rsp_data <= 32'h05B5F586;
        16'h0F18 : rd_rsp_data <= 32'h565E5062;
        16'h0F1C : rd_rsp_data <= 32'h6BEA0C2A;
        16'h0F20 : rd_rsp_data <= 32'hBDA1C5AB;
        16'h0F24 : rd_rsp_data <= 32'hD41855B2;
        16'h0F28 : rd_rsp_data <= 32'h438A5628;
        16'h0F2C : rd_rsp_data <= 32'h31AA6583;
        16'h0F30 : rd_rsp_data <= 32'h15AC5035;
        16'h0F34 : rd_rsp_data <= 32'h55CA0650;
        16'h0F38 : rd_rsp_data <= 32'h59A06A08;
        16'h0F3C : rd_rsp_data <= 32'h940DA180;
        16'h0F40 : rd_rsp_data <= 32'h41D4100F;
        16'h0F44 : rd_rsp_data <= 32'h1A4201FF;
        16'h0F48 : rd_rsp_data <= 32'hA8201FFD;
        16'h0F4C : rd_rsp_data <= 32'h8403FFDA;
        16'h0F50 : rd_rsp_data <= 32'h403FFBAA;
        16'h0F54 : rd_rsp_data <= 32'h07FFB5AF;
        16'h0F58 : rd_rsp_data <= 32'h7FF655F2;
        16'h0F5C : rd_rsp_data <= 32'hFE6A5E24;
        16'h0F60 : rd_rsp_data <= 32'hEDABE44C;
        16'h0F64 : rd_rsp_data <= 32'hD5BC49C7;
        16'h0F68 : rd_rsp_data <= 32'h57C99873;
        16'h0F6C : rd_rsp_data <= 32'h79938E33;
        16'h0F70 : rd_rsp_data <= 32'h9231E634;
        16'h0F74 : rd_rsp_data <= 32'h261C664B;
        16'h0F78 : rd_rsp_data <= 32'h63CC69B4;
        16'h0F7C : rd_rsp_data <= 32'h39CD964D;
        16'h0F80 : rd_rsp_data <= 32'h99D269DB;
        16'h0F84 : rd_rsp_data <= 32'h9A2D9BB8;
        16'h0F88 : rd_rsp_data <= 32'hA5D3B781;
        16'h0F8C : rd_rsp_data <= 32'h5A367013;
        16'h0F90 : rd_rsp_data <= 32'hA66E0232;
        16'h0F94 : rd_rsp_data <= 32'h6DE02627;
        16'h0F98 : rd_rsp_data <= 32'hDC046478;
        16'h0F9C : rd_rsp_data <= 32'hC04C4F8A;
        16'h0FA0 : rd_rsp_data <= 32'h09C9F1A4;
        16'h0FA4 : rd_rsp_data <= 32'h999E1446;
        16'h0FA8 : rd_rsp_data <= 32'h93E24860;
        16'h0FAC : rd_rsp_data <= 32'h3C298C0A;
        16'h0FB0 : rd_rsp_data <= 32'hC591C1AD;
        16'h0FB4 : rd_rsp_data <= 32'h521815D3;
        16'h0FB8 : rd_rsp_data <= 32'h23825A3A;
        16'h0FBC : rd_rsp_data <= 32'h302BA7AE;
        16'h0FC0 : rd_rsp_data <= 32'h05B475E8;
        16'h0FC4 : rd_rsp_data <= 32'h564E5D8A;
        16'h0FC8 : rd_rsp_data <= 32'h69EBD1AB;
        16'h0FCC : rd_rsp_data <= 32'h9DBA15B0;
        16'h0FD0 : rd_rsp_data <= 32'hD7A25605;
        16'h0FD4 : rd_rsp_data <= 32'h742A6057;
        16'h0FD8 : rd_rsp_data <= 32'h45AC0A7B;
        16'h0FDC : rd_rsp_data <= 32'h55C1AFB0;
        16'h0FE0 : rd_rsp_data <= 32'h5815F606;
        16'h0FE4 : rd_rsp_data <= 32'h825E6062;
        16'h0FE8 : rd_rsp_data <= 32'h2BEC0C25;
        16'h0FEC : rd_rsp_data <= 32'hBDC1C453;
        16'h0FF0 : rd_rsp_data <= 32'hD8184A3A;
        16'h0FF4 : rd_rsp_data <= 32'h8389A7A8;
        16'h0FF8 : rd_rsp_data <= 32'h31947582;
        16'h0FFC : rd_rsp_data <= 32'h124E5021;
                default: rd_rsp_data <= 32'h00000000;
            endcase
        end else if (dwr_valid) begin
            case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'hFFFF)
                //Dont be scared
            endcase
        end else begin
            rd_rsp_data <= 32'h00000000;
        end
    end
            
endmodule

// 这是一个bar2的tlp回应实现
module pcileech_bar_impl_bar2(
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
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);
                     
    reg [87:0]      drd_req_ctx;
    reg [31:0]      drd_req_addr;
    reg             drd_req_valid;
                  
    reg [31:0]      dwr_addr;
    reg [31:0]      dwr_data;
    reg             dwr_valid;
               
    reg [31:0]      data_32;
              
    time number = 0;
                  
    always @ (posedge clk) begin
        if (rst)
            number <= 0;
               
        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        
        

        if (drd_req_valid) begin
            // 52104004 - 52104000 = 00000004 & FFFF = 0004
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'h0FFF)
            16'h0000 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0008 : rd_rsp_data <= 32'h000049A7;
            16'h0010 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0018 : rd_rsp_data <= 32'h00004997;
            16'h0020 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0028 : rd_rsp_data <= 32'h00004987;
            16'h0030 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0038 : rd_rsp_data <= 32'h00004977;
            16'h0040 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0048 : rd_rsp_data <= 32'h00004967;
            16'h0050 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0058 : rd_rsp_data <= 32'h00004957;
            16'h0060 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0068 : rd_rsp_data <= 32'h000049B8;
            16'h0070 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0078 : rd_rsp_data <= 32'h000049A8;
            16'h0080 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0088 : rd_rsp_data <= 32'h00004998;
            16'h0090 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0098 : rd_rsp_data <= 32'h00004988;
            16'h00A0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h00A8 : rd_rsp_data <= 32'h00004978;
            16'h00B0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h00B8 : rd_rsp_data <= 32'h00004968;
            16'h00C0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h00C8 : rd_rsp_data <= 32'h00004958;
            16'h00D0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h00D8 : rd_rsp_data <= 32'h000049B9;
            16'h00E0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h00E8 : rd_rsp_data <= 32'h000049A9;
            16'h00F0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h00F8 : rd_rsp_data <= 32'h00004999;
            16'h0100 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0108 : rd_rsp_data <= 32'h00004989;
            16'h0110 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0118 : rd_rsp_data <= 32'h00004979;
            16'h0120 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0128 : rd_rsp_data <= 32'h00004969;
            16'h0130 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0138 : rd_rsp_data <= 32'h000049A7;
            16'h0140 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0148 : rd_rsp_data <= 32'h000049A7;
            16'h0150 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0158 : rd_rsp_data <= 32'h000049A7;
            16'h0160 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0168 : rd_rsp_data <= 32'h000049A7;
            16'h0170 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0178 : rd_rsp_data <= 32'h000049A7;
            16'h0180 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0188 : rd_rsp_data <= 32'h000049A7;
            16'h0190 : rd_rsp_data <= 32'hFEE3F00C;
            16'h0198 : rd_rsp_data <= 32'h000049A7;
            16'h01A0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h01A8 : rd_rsp_data <= 32'h000049A7;
            16'h01B0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h01B8 : rd_rsp_data <= 32'h000049A7;
            16'h01C0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h01C8 : rd_rsp_data <= 32'h000049A7;
            16'h01D0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h01D8 : rd_rsp_data <= 32'h000049A7;
            16'h01E0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h01E8 : rd_rsp_data <= 32'h000049A7;
            16'h01F0 : rd_rsp_data <= 32'hFEE3F00C;
            16'h01F8 : rd_rsp_data <= 32'h000049A7;
                default: rd_rsp_data <= 32'h00000000;
            endcase
        end else if (dwr_valid) begin
            case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'hFFFF)
                //Dont be scared
            endcase
        end else begin
            rd_rsp_data <= 32'h00000000;
        end
    end
            
endmodule



// 这是bar4的tlp回应实现
module pcileech_bar_impl_bar4(
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
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);
                     
    reg [87:0]      drd_req_ctx;
    reg [31:0]      drd_req_addr;
    reg             drd_req_valid;
                  
    reg [31:0]      dwr_addr;
    reg [31:0]      dwr_data;
    reg             dwr_valid;
               
    reg [31:0]      data_32;
              
    time number = 0;
                  
    always @ (posedge clk) begin
        if (rst)
            number <= 0;
               
        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'hFFFF)
            16'h0000 : rd_rsp_data <= 32'h20C0504E;
            16'h0004 : rd_rsp_data <= 32'h0002C000;
            16'h0008 : rd_rsp_data <= 32'h00000D00;
            16'h000C : rd_rsp_data <= 32'h0000000A;
            16'h0018 : rd_rsp_data <= 32'h00000280;
            16'h0020 : rd_rsp_data <= 32'h00C00800;
            16'h0024 : rd_rsp_data <= 32'h006C0900;
            16'h0028 : rd_rsp_data <= 32'h00C00A00;
            16'h002C : rd_rsp_data <= 32'hF8008000;
            16'h0030 : rd_rsp_data <= 32'h480002C5;
            16'h0034 : rd_rsp_data <= 32'h01519003;
            16'h0038 : rd_rsp_data <= 32'h0BDD0000;
            16'h003C : rd_rsp_data <= 32'h00001400;
            16'h0040 : rd_rsp_data <= 32'h6004A000;
            16'h0044 : rd_rsp_data <= 32'hB8000312;
            16'h0048 : rd_rsp_data <= 32'h01543007;
            16'h0050 : rd_rsp_data <= 32'hD7840000;
            16'h0054 : rd_rsp_data <= 32'h00091000;
            16'h0058 : rd_rsp_data <= 32'hB5E8010A;
            16'h005C : rd_rsp_data <= 32'h000A2000;
            16'h0064 : rd_rsp_data <= 32'h00000D00;
            16'h0068 : rd_rsp_data <= 32'h00120C00;
            16'h006C : rd_rsp_data <= 32'h00180B00;
            16'h0800 : rd_rsp_data <= 32'h41000882;
            16'h0804 : rd_rsp_data <= 32'h6E616C74;
            16'h0808 : rd_rsp_data <= 32'h90636974;
            16'h080C : rd_rsp_data <= 32'h4E50005F;
            16'h0810 : rd_rsp_data <= 32'h3932330A;
            16'h0814 : rd_rsp_data <= 32'h35393430;
            16'h0818 : rd_rsp_data <= 32'h45353930;
            16'h081C : rd_rsp_data <= 32'h46300143;
            16'h0820 : rd_rsp_data <= 32'h62610347;
            16'h0824 : rd_rsp_data <= 32'h03434C63;
            16'h0828 : rd_rsp_data <= 32'h4D666564;
            16'h082C : rd_rsp_data <= 32'h46410C4E;
            16'h0830 : rd_rsp_data <= 32'h45575344;
            16'h0834 : rd_rsp_data <= 32'h53424557;
            16'h0838 : rd_rsp_data <= 32'h47504446;
            16'h083C : rd_rsp_data <= 32'h49494903;
            16'h0840 : rd_rsp_data <= 32'h430C4E53;
            16'h0844 : rd_rsp_data <= 32'h39354C50;
            16'h0848 : rd_rsp_data <= 32'h4C543833;
            16'h084C : rd_rsp_data <= 32'h56594D4B;
            16'h0850 : rd_rsp_data <= 32'h66770630;
            16'h0854 : rd_rsp_data <= 32'h65667765;
            16'h0858 : rd_rsp_data <= 32'h66063156;
            16'h085C : rd_rsp_data <= 32'h66776577;
            16'h0860 : rd_rsp_data <= 32'h05325665;
            16'h0864 : rd_rsp_data <= 32'h57464453;
            16'h0868 : rd_rsp_data <= 32'h01565249;
            16'h086C : rd_rsp_data <= 32'h004F9116;
            16'h0870 : rd_rsp_data <= 32'h39074159;
            16'h0874 : rd_rsp_data <= 32'h38353934;
            16'h0878 : rd_rsp_data <= 32'h30563932;
            16'h087C : rd_rsp_data <= 32'h34336609;
            16'h0880 : rd_rsp_data <= 32'h72346567;
            16'h0884 : rd_rsp_data <= 32'h31566773;
            16'h0888 : rd_rsp_data <= 32'h72656710;
            16'h088C : rd_rsp_data <= 32'h35673533;
            16'h0890 : rd_rsp_data <= 32'h67687472;
            16'h0894 : rd_rsp_data <= 32'h61736768;
            16'h0898 : rd_rsp_data <= 32'h09305933;
            16'h089C : rd_rsp_data <= 32'h66647362;
            16'h08A0 : rd_rsp_data <= 32'h63786276;
            16'h08A4 : rd_rsp_data <= 32'h0931597A;
            16'h08A8 : rd_rsp_data <= 32'h66657766;
            16'h08AC : rd_rsp_data <= 32'h66777765;
            16'h08B0 : rd_rsp_data <= 32'h0B575265;
            16'h08BC : rd_rsp_data <= 32'h78000000;
            16'h0900 : rd_rsp_data <= 32'h5519050A;
            16'h0904 : rd_rsp_data <= 32'hEFDA0001;
            16'h0908 : rd_rsp_data <= 32'h07B11D6A;
            16'h090C : rd_rsp_data <= 32'h02000002;
            16'h0910 : rd_rsp_data <= 32'h00011D6A;
            16'h0918 : rd_rsp_data <= 32'h00040001;
            16'h091C : rd_rsp_data <= 32'hFFFFFFFF;
            16'h0920 : rd_rsp_data <= 32'hF004FFFF;
            16'h0924 : rd_rsp_data <= 32'hFFFFFFFF;
            16'h0928 : rd_rsp_data <= 32'h0004FFFF;
            16'h092C : rd_rsp_data <= 32'hFFFFFFC0;
            16'h0930 : rd_rsp_data <= 32'h0000FFFF;
            16'h0934 : rd_rsp_data <= 32'h0700FFFC;
            16'h0938 : rd_rsp_data <= 32'h02FF0000;
            16'h0940 : rd_rsp_data <= 32'h40014000;
            16'h0944 : rd_rsp_data <= 32'h00007F00;
            16'h0950 : rd_rsp_data <= 32'h00008038;
            16'h0960 : rd_rsp_data <= 32'h00114100;
            16'h0968 : rd_rsp_data <= 32'h000058B4;
            16'h0A00 : rd_rsp_data <= 32'h000B0001;
            16'h0A04 : rd_rsp_data <= 32'h000B0002;
            16'h0A08 : rd_rsp_data <= 32'h000B0003;
            16'h0A0C : rd_rsp_data <= 32'h000B0004;
            16'h0A10 : rd_rsp_data <= 32'h000B0005;
            16'h0A14 : rd_rsp_data <= 32'h000B0006;
            16'h0A18 : rd_rsp_data <= 32'h000B0007;
            16'h0A1C : rd_rsp_data <= 32'h000B0008;
            16'h0A20 : rd_rsp_data <= 32'h000B0009;
            16'h0A24 : rd_rsp_data <= 32'h000B000A;
            16'h0A28 : rd_rsp_data <= 32'h000B000B;
            16'h0A2C : rd_rsp_data <= 32'h000B000C;
            16'h0A30 : rd_rsp_data <= 32'h000B000D;
            16'h0A34 : rd_rsp_data <= 32'h000B000E;
            16'h0A38 : rd_rsp_data <= 32'h000B000F;
            16'h0A3C : rd_rsp_data <= 32'h000B0010;
            16'h0A40 : rd_rsp_data <= 32'h000B0011;
            16'h0A44 : rd_rsp_data <= 32'h000B0012;
            16'h0A48 : rd_rsp_data <= 32'h000B0013;
            16'h0A4C : rd_rsp_data <= 32'h000B0014;
            16'h0A50 : rd_rsp_data <= 32'h000B0015;
            16'h0A54 : rd_rsp_data <= 32'h000B0016;
            16'h0A58 : rd_rsp_data <= 32'h000B0017;
            16'h0A5C : rd_rsp_data <= 32'h000B0018;
            16'h0A60 : rd_rsp_data <= 32'h000B0019;
            16'h0A64 : rd_rsp_data <= 32'h000B001A;
            16'h0A68 : rd_rsp_data <= 32'h000B001B;
            16'h0A6C : rd_rsp_data <= 32'h000B001C;
            16'h0A70 : rd_rsp_data <= 32'h000B001D;
            16'h0A74 : rd_rsp_data <= 32'h000B001E;
            16'h0A78 : rd_rsp_data <= 32'h000B001F;
            16'h0A7C : rd_rsp_data <= 32'h000B0020;
            16'h0A80 : rd_rsp_data <= 32'h000B0021;
            16'h0A84 : rd_rsp_data <= 32'h000B0022;
            16'h0A88 : rd_rsp_data <= 32'h000B0023;
            16'h0A8C : rd_rsp_data <= 32'h000B0024;
            16'h0A90 : rd_rsp_data <= 32'h000B0025;
            16'h0A94 : rd_rsp_data <= 32'h000B0026;
            16'h0A98 : rd_rsp_data <= 32'h000B0027;
            16'h0A9C : rd_rsp_data <= 32'h000B0028;
            16'h0AA0 : rd_rsp_data <= 32'h000B0029;
            16'h0AA4 : rd_rsp_data <= 32'h000B002A;
            16'h0AA8 : rd_rsp_data <= 32'h000B002B;
            16'h0AAC : rd_rsp_data <= 32'h000B002C;
            16'h0AB0 : rd_rsp_data <= 32'h000B002D;
            16'h0AB4 : rd_rsp_data <= 32'h000B002E;
            16'h0AB8 : rd_rsp_data <= 32'h000B002F;
            16'h0ABC : rd_rsp_data <= 32'h000B0030;
            16'h0B00 : rd_rsp_data <= 32'h00000028;
            16'h0B04 : rd_rsp_data <= 32'hFFFFFFFF;
            16'h0B08 : rd_rsp_data <= 32'h0017B600;
            16'h0B0C : rd_rsp_data <= 32'h00000029;
            16'h0B10 : rd_rsp_data <= 32'hFFFF0000;
            16'h0C00 : rd_rsp_data <= 32'h01040103;
            16'h0C04 : rd_rsp_data <= 32'hAB0EC41D;
            16'h0C08 : rd_rsp_data <= 32'hC41EFFFF;
            16'h0C0C : rd_rsp_data <= 32'hFFFF0792;
                default: rd_rsp_data <= 32'h00000000;
            endcase
        end else if (dwr_valid) begin
            case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'hFFFF)
                //Dont be scared
            endcase
        end else begin
            rd_rsp_data <= 32'h00000000;
        end
    end
            
endmodule
