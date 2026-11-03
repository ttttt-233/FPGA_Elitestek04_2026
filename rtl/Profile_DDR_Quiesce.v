`timescale 1ns/1ps
// Counts commands actually accepted by the DDR port and their completions.
// The Core prevents the AW/AR mux from starting new commands while requested;
// a command already selected by that mux still completes normally. Read data,
// write data and responses remain connected throughout the drain.
module Profile_DDR_Quiesce #(
    parameter integer QUIET_CYCLES=16,
    parameter integer COUNT_BITS=8
)(
    input wire clk,rst_n,request,tracking_enable,
    input wire avalid,aready,atype,
    input wire bvalid,rvalid,rready,rlast,
    input wire awready,arready,
    output wire idle,
    output reg error
);
    localparam integer QUIET_BITS=$clog2(QUIET_CYCLES+1);
    reg [COUNT_BITS-1:0] reads,writes;
    reg [QUIET_BITS-1:0] quiet_count;
    wire add_read=avalid && aready && !atype;
    wire add_write=avalid && aready && atype;
    wire end_read=rvalid && rready && rlast;
    wire quiet=request && !avalid && !awready && !arready &&
        reads==0 && writes==0 && !bvalid && !rvalid;
    assign idle=tracking_enable && request && quiet_count==QUIET_CYCLES && quiet && !error;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin reads<=0;writes<=0;quiet_count<=0;error<=0;end
        // PHY/controller response outputs before successful calibration are
        // not accepted AXI transactions. A calibration loss also invalidates
        // the old accounting epoch; never report that epoch as safely drained.
        else if(!tracking_enable) begin reads<=0;writes<=0;quiet_count<=0;error<=0;end
        else begin
            case({add_read,end_read})
                2'b10: if(&reads) error<=1;else reads<=reads+1'b1;
                2'b01: if(reads==0) error<=1;else reads<=reads-1'b1;
                default: ;
            endcase
            case({add_write,bvalid})
                2'b10: if(&writes) error<=1;else writes<=writes+1'b1;
                2'b01: if(writes==0) error<=1;else writes<=writes-1'b1;
                default: ;
            endcase
            if(!quiet) quiet_count<=0;
            else if(quiet_count!=QUIET_CYCLES) quiet_count<=quiet_count+1'b1;
        end
    end
endmodule
