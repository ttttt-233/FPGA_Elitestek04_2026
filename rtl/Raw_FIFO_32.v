`timescale 1ns/1ps
// Resource-bounded symmetric configuration of the SAME generated Efinix FIFO
// implementation included by ip/W0_FIFO_32/W0_FIFO_32.sv. 512 x 32 uses two
// RAM10s. A RAW burst consumes 64 words; 448 words remain for DDR contention.
// Keep the generated implementation UUID in sync when regenerating that IP.
module Raw_FIFO_32(
    input wire reset,wr_clk,rd_clk,write,read,
    input wire [31:0] data,
    output wire [31:0] q,
    output wire full,empty,pempty,rst_busy
);
    efx_fifo_top_7e7fd22cad5340c1b4bd6217399af02c #(
        .SYNC_CLK(0),.SYNC_STAGE(3),.DATA_WIDTH(32),.DEPTH(512),
        .MODE("FWFT"),.OUTPUT_REG(0),.PIPELINE_REG(1),.ASYM_WIDTH_RATIO(4),
        .PROGRAMMABLE_FULL("STATIC_SINGLE"),.PROG_FULL_ASSERT(448),.PROG_FULL_NEGATE(448),
        .PROGRAMMABLE_EMPTY("STATIC_SINGLE"),.PROG_EMPTY_ASSERT(63),.PROG_EMPTY_NEGATE(64),
        .OPTIONAL_FLAGS(0),.FAMILY("TITANIUM"),.BYPASS_RESET_SYNC(0),.ENDIANESS(0),
        .RAM_STYLE("block_ram"),.OVERFLOW_PROTECT(1),.UNDERFLOW_PROTECT(1)
    ) fifo (
        .wr_clk_i(wr_clk),.rd_clk_i(rd_clk),.a_rst_i(reset),
        .wr_en_i(write),.rd_en_i(read),.wdata(data),.rdata(q),
        .full_o(full),.empty_o(empty),.prog_empty_o(pempty),.rst_busy(rst_busy),
        .prog_full_o(),.wr_datacount_o(),.rd_datacount_o(),.underflow_o(),.overflow_o());
endmodule
