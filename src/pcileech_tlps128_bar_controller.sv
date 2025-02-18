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
    output  wire            int_enable,
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
    
    pcileech_bar_impl_CSI2 i_bar0(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_be             ( rd_req_be                     ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[0] ),
        .int_enable            ( int_enable                    ),
        .base_address_register ( base_address_register         ),
        .rd_rsp_ctx            ( bar_rsp_ctx[0]                ),
        .rd_rsp_data           ( bar_rsp_data[0]               ),
        .rd_rsp_valid          ( bar_rsp_valid[0]              )
    );

    pcileech_bar_impl_none i_bar1(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[1] ),
        .rd_rsp_ctx            ( bar_rsp_ctx[1]                ),
        .rd_rsp_data           ( bar_rsp_data[1]               ),
        .rd_rsp_valid          ( bar_rsp_valid[1]              )
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
    
    // pcileech_bar_impl_none i_bar0(
    //     .rst            ( rst                           ),
    //     .clk            ( clk                           ),
    //     .wr_addr        ( wr_addr                       ),
    //     .wr_be          ( wr_be                         ),
    //     .wr_data        ( wr_data                       ),
    //     .wr_valid       ( wr_valid && wr_bar[0]         ),
    //     .rd_req_ctx     ( rd_req_ctx                    ),
    //     .rd_req_addr    ( rd_req_addr                   ),
    //     .rd_req_valid   ( rd_req_valid && rd_req_bar[0] ),
    //     .rd_rsp_ctx     ( bar_rsp_ctx[0]                ),
    //     .rd_rsp_data    ( bar_rsp_data[0]               ),
    //     .rd_rsp_valid   ( bar_rsp_valid[0]              )
    // );
    
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
        rd_rsp_data     <= 32'h0;
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


// ------------------------------------------------------------------------
// 
// 
// ------------------------------------------------------------------------
module pcileech_bar_impl_CSI2(
    input               rst,
    input               clk,
    // interrupt enable
    output wire          int_enable,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input  [3:0]        rd_req_be,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      drd_req_ctx;
    bit [31:0]      drd_req_addr;
    bit [3:0]       drd_req_be;
    bit             drd_req_valid;
    
    bit [3:0]       dwr_be;
    bit [31:0]      dwr_addr;
    bit [31:0]      dwr_data;
    bit             dwr_valid;

    bit [31:0]      data_32;

    bit [31:0]      number = 32'h51;
    bit [31:0]      random_number = 32'h49;
    bit stop = 0;
    logic toggle;
 

    always @ ( posedge clk ) begin
        if (rst) begin
            number <= 32'h51;
            data_32 <= 0;
            stop <= 1'b0;
            random_number <= 32'h49;
        end
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        drd_req_be      <= rd_req_be;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;
        dwr_be         <= wr_be;
        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'hFFFF)
                16'h0004: begin
                    rd_rsp_data <= 32'hEEAB6897;
                end
                16'h0008: begin
                    rd_rsp_data <= 32'h001E0000;
                    stop <= 1'b1;
                end
                // 16'h0008 : rd_rsp_data <= 32'h001E0000;
                16'h000C : rd_rsp_data <= 32'h101E0000;
                16'h0010 : rd_rsp_data <= 32'h39060000;
                16'h0014 : rd_rsp_data <= 32'h49240000;
                16'h0018 : rd_rsp_data <= 32'h047B4000;
                16'h001C : rd_rsp_data <= 32'h4D9F4000;
                16'h0020 : rd_rsp_data <= 32'h00042000;
                16'h0024 : rd_rsp_data <= 32'h4DA36000;
                16'h0028 : rd_rsp_data <= 32'h044D9000;
                16'h002C : rd_rsp_data <= 32'h51F0F000;
                16'h0030 : rd_rsp_data <= 32'h0001D000;
                16'h0034 : rd_rsp_data <= 32'h51F2C000;
                16'h0038 : rd_rsp_data <= 32'h00011000;
                16'h003C : rd_rsp_data <= 32'h51F3D000;
                16'h0040 : rd_rsp_data <= 32'h0000E000;
                16'h0044 : rd_rsp_data <= 32'h51F4B000;
                16'h0048 : rd_rsp_data <= 32'h0001E000;
                16'h004C : rd_rsp_data <= 32'h51F69000;
                16'h0050 : rd_rsp_data <= 32'h00003000;
                16'h0054 : rd_rsp_data <= 32'h51F6C000;
                16'h0058 : rd_rsp_data <= 32'h00013000;
                16'h005C : rd_rsp_data <= 32'h51F7F000;
                16'h0060 : rd_rsp_data <= 32'h51F80000;
                16'h0064 : rd_rsp_data <= 32'h51F91000;
                16'h0068 : rd_rsp_data <= 32'h00009000;
                16'h006C : rd_rsp_data <= 32'h51F9A000;
                16'h0070 : rd_rsp_data <= 32'h00026000;
                16'h0074 : rd_rsp_data <= 32'h51FC0000;
                16'h0078 : rd_rsp_data <= 32'h51FDD000;
                16'h007C : rd_rsp_data <= 32'h0005E000;
                16'h0080 : rd_rsp_data <= 32'h5203B000;
                16'h0084 : rd_rsp_data <= 32'h5203D000;
                16'h0088 : rd_rsp_data <= 32'h00005000;
                16'h008C : rd_rsp_data <= 32'h52042000;
                16'h0090 : rd_rsp_data <= 32'h52044000;
                16'h0094 : rd_rsp_data <= 32'h52049000;
                16'h0098 : rd_rsp_data <= 32'h00004000;
                16'h009C : rd_rsp_data <= 32'h5204D000;
                16'h00A0 : rd_rsp_data <= 32'h00008000;
                16'h00A4 : rd_rsp_data <= 32'h52055000;
                16'h00A8 : rd_rsp_data <= 32'h52057000;
                16'h00AC : rd_rsp_data <= 32'h00006000;
                16'h00B0 : rd_rsp_data <= 32'h5205D000;
                16'h00B4 : rd_rsp_data <= 32'h5205E000;
                16'h00B8 : rd_rsp_data <= 32'h0002B000;
                16'h00BC : rd_rsp_data <= 32'h52089000;
                16'h00C0 : rd_rsp_data <= 32'h5208D000;
                16'h00C4 : rd_rsp_data <= 32'h52095000;
                16'h00C8 : rd_rsp_data <= 32'h5209A000;
                16'h00CC : rd_rsp_data <= 32'h00080000;
                16'h00D0 : rd_rsp_data <= 32'h5211A000;
                16'h00D4 : rd_rsp_data <= 32'h00044000;
                16'h00D8 : rd_rsp_data <= 32'h5215E000;
                16'h00DC : rd_rsp_data <= 32'h0003E000;
                16'h00E0 : rd_rsp_data <= 32'h5219C000;
                16'h00E4 : rd_rsp_data <= 32'h5219E000;
                16'h00E8 : rd_rsp_data <= 32'h521B0000;
                16'h00EC : rd_rsp_data <= 32'h521CE000;
                16'h00F0 : rd_rsp_data <= 32'h00156000;
                16'h00F4 : rd_rsp_data <= 32'h52324000;
                16'h00F8 : rd_rsp_data <= 32'h00007000;
                16'h00FC : rd_rsp_data <= 32'h5232B000;
                16'h0100 : rd_rsp_data <= 32'h0000B000;
                16'h0104 : rd_rsp_data <= 32'h52336000;
                16'h0108 : rd_rsp_data <= 32'h5233B000;
                16'h010C : rd_rsp_data <= 32'h023DD000;
                16'h0110 : rd_rsp_data <= 32'h54718000;
                16'h0114 : rd_rsp_data <= 32'h00023000;
                16'h0118 : rd_rsp_data <= 32'h5473B000;
                16'h011C : rd_rsp_data <= 32'h5473D000;
                16'h0120 : rd_rsp_data <= 32'h0000C000;
                16'h0124 : rd_rsp_data <= 32'h54749000;
                16'h0128 : rd_rsp_data <= 32'h54759000;
                16'h012C : rd_rsp_data <= 32'h00900000;
                16'h0130 : rd_rsp_data <= 32'h55059000;
                16'h0134 : rd_rsp_data <= 32'h55060000;
                16'h0138 : rd_rsp_data <= 32'h0000F000;
                16'h013C : rd_rsp_data <= 32'h5506F000;
                16'h0140 : rd_rsp_data <= 32'h003F3000;
                16'h0144 : rd_rsp_data <= 32'h55462000;
                16'h0148 : rd_rsp_data <= 32'h01A0D000;
                16'h014C : rd_rsp_data <= 32'h56E6F000;
                16'h0150 : rd_rsp_data <= 32'h003D0000;
                16'h0154 : rd_rsp_data <= 32'h5723F000;
                16'h0158 : rd_rsp_data <= 32'h00CD0000;
                16'h015C : rd_rsp_data <= 32'h57F0F000;
                16'h0160 : rd_rsp_data <= 32'h58F0F000;
                16'h0164 : rd_rsp_data <= 32'h02FB0000;
                16'h0168 : rd_rsp_data <= 32'h00000004;
                16'h016C : rd_rsp_data <= 32'h5BEBF000;
                16'h0170 : rd_rsp_data <= 32'h00140000;
                16'h0174 : rd_rsp_data <= 32'h00000003;
                16'h0178 : rd_rsp_data <= 32'h5BFFF000;
                16'h017C : rd_rsp_data <= 32'h97800000;
                16'h0180 : rd_rsp_data <= 32'h00000007;
                16'h0184 : rd_rsp_data <= 32'h000A0000;
                16'h0188 : rd_rsp_data <= 32'h00060000;
                16'h018C : rd_rsp_data <= 32'h5C000000;
                16'h0190 : rd_rsp_data <= 32'h04400000;
                16'h0194 : rd_rsp_data <= 32'h60E00000;
                16'h0198 : rd_rsp_data <= 32'h01200000;
                16'h019C : rd_rsp_data <= 32'h62000000;
                16'h01A0 : rd_rsp_data <= 32'h5BEFD228;
                16'h01A4 : rd_rsp_data <= 32'h00000059;
                16'h01A8 : rd_rsp_data <= 32'h00001000;
                16'h01AC : rd_rsp_data <= 32'h00000001;
                16'h01B0 : rd_rsp_data <= 32'h00002000;
                16'h01B4 : rd_rsp_data <= 32'h00085000;
                16'h01B8 : rd_rsp_data <= 32'h00087000;
                16'h01BC : rd_rsp_data <= 32'h00088000;
                16'h01C0 : rd_rsp_data <= 32'h00017000;
                16'h01C4 : rd_rsp_data <= 32'h0009F000;
                16'h01C8 : rd_rsp_data <= 32'h00000002;
                16'h01CC : rd_rsp_data <= 32'h00100000;
                16'h01D0 : rd_rsp_data <= 32'h00102000;
                16'h01D4 : rd_rsp_data <= 32'h00103000;
                16'h01D8 : rd_rsp_data <= 32'h00157000;
                16'h01DC : rd_rsp_data <= 32'h0025A000;
                16'h01E0 : rd_rsp_data <= 32'h0002D000;
                16'h01E4 : rd_rsp_data <= 32'h00287000;
                16'h01E8 : rd_rsp_data <= 32'h00288000;
                16'h01EC : rd_rsp_data <= 32'h00289000;
                16'h01F0 : rd_rsp_data <= 32'h00012000;
                16'h01F4 : rd_rsp_data <= 32'h0029B000;
                16'h01F8 : rd_rsp_data <= 32'h0029C000;
                16'h01FC : rd_rsp_data <= 32'h00010000;
                16'h0200 : rd_rsp_data <= 32'h002AC000;
                16'h0204 : rd_rsp_data <= 32'h00022000;
                16'h0208 : rd_rsp_data <= 32'h002CE000;
                16'h020C : rd_rsp_data <= 32'h000CB000;
                16'h0210 : rd_rsp_data <= 32'h00399000;
                16'h0214 : rd_rsp_data <= 32'h00067000;
                16'h0218 : rd_rsp_data <= 32'h00400000;
                16'h021C : rd_rsp_data <= 32'h004C6000;
                16'h0220 : rd_rsp_data <= 32'h008C6000;
                16'h0224 : rd_rsp_data <= 32'h0015B000;
                16'h0228 : rd_rsp_data <= 32'h00A21000;
                16'h022C : rd_rsp_data <= 32'h00A21000;
                16'h0230 : rd_rsp_data <= 32'h00A21000;
                16'h0234 : rd_rsp_data <= 32'h00A21000;
                16'h0238 : rd_rsp_data <= 32'h00A21000;
                16'h023C : rd_rsp_data <= 32'h00A21000;
                16'h0240 : rd_rsp_data <= 32'h00A21000;
                16'h0244 : rd_rsp_data <= 32'h00A21000;
                16'h0248 : rd_rsp_data <= 32'h00A21000;
                16'h024C : rd_rsp_data <= 32'h00A21000;
                16'h0250 : rd_rsp_data <= 32'h00A21000;
                16'h0254 : rd_rsp_data <= 32'h00A21000;
                16'h0258 : rd_rsp_data <= 32'h00A21000;
                16'h025C : rd_rsp_data <= 32'h00A21000;
                16'h0260 : rd_rsp_data <= 32'h00A21000;
                16'h0264 : rd_rsp_data <= 32'h00A21000;
                16'h0268 : rd_rsp_data <= 32'h00A21000;
                16'h026C : rd_rsp_data <= 32'h00A21000;
                16'h0270 : rd_rsp_data <= 32'h00A21000;
                16'h0274 : rd_rsp_data <= 32'h00A21000;
                16'h0278 : rd_rsp_data <= 32'h00A21000;
                16'h027C : rd_rsp_data <= 32'h00A21000;
                16'h0280 : rd_rsp_data <= 32'h00A21000;
                16'h0284 : rd_rsp_data <= 32'h00A21000;
                16'h0288 : rd_rsp_data <= 32'h00A21000;
                16'h028C : rd_rsp_data <= 32'h00A21000;
                16'h0290 : rd_rsp_data <= 32'h00A21000;
                16'h0294 : rd_rsp_data <= 32'h00A21000;
                16'h0298 : rd_rsp_data <= 32'h00A21000;
                16'h029C : rd_rsp_data <= 32'h00A21000;
                16'h02A0 : rd_rsp_data <= 32'h00A21000;
                16'h02A4 : rd_rsp_data <= 32'h00A21000;
                16'h02A8 : rd_rsp_data <= 32'h00A21000;
                16'h02AC : rd_rsp_data <= 32'h0029B000;
                16'h02B0 : rd_rsp_data <= 32'h0029C000;
                16'h02B4 : rd_rsp_data <= 32'h00010000;
                16'h02B8 : rd_rsp_data <= 32'h00010000;
                16'h02BC : rd_rsp_data <= 32'h00010000;
                16'h02C0 : rd_rsp_data <= 32'h00010000;
                16'h02C4 : rd_rsp_data <= 32'h00010000;
                16'h02C8 : rd_rsp_data <= 32'h00010000;
                16'h02CC : rd_rsp_data <= 32'h00010000;
                16'h02D0 : rd_rsp_data <= 32'h00010000;
                16'h02D4 : rd_rsp_data <= 32'h00010000;
                16'h02D8 : rd_rsp_data <= 32'h00010000;
                16'h02DC : rd_rsp_data <= 32'h00010000;
                16'h02E0 : rd_rsp_data <= 32'h00010000;
                16'h02E4 : rd_rsp_data <= 32'h00010000;
                16'h02E8 : rd_rsp_data <= 32'h00010000;
                16'h02EC : rd_rsp_data <= 32'h00010000;
                16'h02F0 : rd_rsp_data <= 32'h00010000;
                16'h02F4 : rd_rsp_data <= 32'h00010000;
                16'h02F8 : rd_rsp_data <= 32'h00010000;
                16'h02FC : rd_rsp_data <= 32'h00010000;
                16'h0300 : rd_rsp_data <= 32'h00010000;
                16'h0304 : rd_rsp_data <= 32'h00010000;
                16'h0308 : rd_rsp_data <= 32'h00010000;
                16'h030C : rd_rsp_data <= 32'h00010000;
                16'h0310 : rd_rsp_data <= 32'h00010000;
                16'h0314 : rd_rsp_data <= 32'h00010000;
                16'h0318 : rd_rsp_data <= 32'h00010000;
                16'h031C : rd_rsp_data <= 32'h00010000;
                16'h0320 : rd_rsp_data <= 32'h00010000;
                16'h0324 : rd_rsp_data <= 32'h00010000;
                16'h0328 : rd_rsp_data <= 32'h00010000;
                16'h032C : rd_rsp_data <= 32'h00010000;
                16'h0330 : rd_rsp_data <= 32'h00010000;
                16'h0334 : rd_rsp_data <= 32'h00010000;
                16'h0338 : rd_rsp_data <= 32'h00010000;
                16'h033C : rd_rsp_data <= 32'h00010000;
                16'h0340 : rd_rsp_data <= 32'h00010000;
                16'h0344 : rd_rsp_data <= 32'h00010000;
                16'h0348 : rd_rsp_data <= 32'h00010000;
                16'h034C : rd_rsp_data <= 32'h00010000;
                16'h0350 : rd_rsp_data <= 32'h00010000;
                16'h0354 : rd_rsp_data <= 32'h00010000;
                16'h0358 : rd_rsp_data <= 32'h00010000;
                16'h035C : rd_rsp_data <= 32'h00010000;
                16'h0360 : rd_rsp_data <= 32'h00010000;
                16'h0364 : rd_rsp_data <= 32'h00010000;
                16'h0368 : rd_rsp_data <= 32'h00010000;
                16'h036C : rd_rsp_data <= 32'h00010000;
                16'h0370 : rd_rsp_data <= 32'h00010000;
                16'h0374 : rd_rsp_data <= 32'h00010000;
                16'h0378 : rd_rsp_data <= 32'h00010000;
                16'h037C : rd_rsp_data <= 32'h00010000;
                16'h0380 : rd_rsp_data <= 32'h00010000;
                16'h0384 : rd_rsp_data <= 32'h00010000;
                16'h0388 : rd_rsp_data <= 32'h00010000;
                16'h038C : rd_rsp_data <= 32'h00010000;
                16'h0390 : rd_rsp_data <= 32'h00010000;
                16'h0394 : rd_rsp_data <= 32'h00010000;
                16'h0398 : rd_rsp_data <= 32'h00010000;
                16'h039C : rd_rsp_data <= 32'h00010000;
                16'h03A0 : rd_rsp_data <= 32'h00010000;
                16'h03A4 : rd_rsp_data <= 32'h00010000;
                16'h03A8 : rd_rsp_data <= 32'h00010000;
                16'h03AC : rd_rsp_data <= 32'h00010000;
                16'h03B0 : rd_rsp_data <= 32'h00010000;
                16'h03B4 : rd_rsp_data <= 32'h00010000;
                16'h03B8 : rd_rsp_data <= 32'h00010000;
                16'h03BC : rd_rsp_data <= 32'h00010000;
                16'h03C0 : rd_rsp_data <= 32'h00010000;
                16'h03C4 : rd_rsp_data <= 32'h00010000;
                16'h03C8 : rd_rsp_data <= 32'h00010000;
                16'h03CC : rd_rsp_data <= 32'h00010000;
                16'h03D0 : rd_rsp_data <= 32'h00010000;
                16'h03D4 : rd_rsp_data <= 32'h00010000;
                16'h03D8 : rd_rsp_data <= 32'h00010000;
                16'h03DC : rd_rsp_data <= 32'h00010000;
                16'h03E0 : rd_rsp_data <= 32'h00010000;
                16'h03E4 : rd_rsp_data <= 32'h00A21000;
                16'h03E8 : rd_rsp_data <= 32'h00A21000;
                16'h03EC : rd_rsp_data <= 32'h00A21000;
                16'h03F0 : rd_rsp_data <= 32'h00A21000;
                16'h03F4 : rd_rsp_data <= 32'h00A21000;
                16'h03F8 : rd_rsp_data <= 32'h00A21000;
                16'h03FC : rd_rsp_data <= 32'h00A21000;
                16'h0400 : rd_rsp_data <= 32'h00A21000;
                default: begin
                    random_number <= {random_number[30:0], random_number[31] ^ random_number[21] ^ random_number[1] ^ random_number[0]};
                    rd_rsp_data <= random_number;
                end
            endcase
        end else if (dwr_valid) begin
            case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & 32'hFFFFFFF0)) & 32'hFFFF)
                //No need to write bar;
            endcase
        end else begin
            rd_rsp_data[7:0]   <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
            rd_rsp_data[15:8]  <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
            rd_rsp_data[23:16] <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
            rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
        end
        // number <= (stop) ? number : number + 1'b1;
        number <= (stop) ? number : {number[30:0], number[31] ^ number[21] ^ number[1] ^ number[0]};
    end

    bit[31:0] int_start_cnt = 0;
    always @ (posedge clk) begin
        if (rst) begin
            int_start_cnt <= 0;
        end else if (int_start_cnt <= 32'd500000000)begin
            int_start_cnt <= int_start_cnt + 1'b1;
        end
    end
    assign int_enable = int_start_cnt >= 32'd500000000;
    // assign int_enable = 0;
endmodule
