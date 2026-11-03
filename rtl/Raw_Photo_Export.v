`timescale 1ns/1ps
// A lease protects one physical SNAP slot while the host reads checked blocks.
// VIDEO/PRE stay RGB565 and never enter this path. A single DDR beat
// is accepted without RREADY throttling, as required by the board controller.
module Raw_Photo_Export #(parameter PIXELS=1280*720,CLOCK_HZ=96000000)(
    input wire clk,rst_n,open,close,request,
    input wire [31:0] selected_base,
    input wire [15:0] block_index,
    output reg locked,
    output reg [31:0] locked_base,
    output wire busy,
    output reg done,error,
    output wire [127:0] block_data,
    output wire [31:0] araddr,
    output wire [7:0] arlen,
    output wire arvalid,rready,
    input wire arready,rvalid,rlast,
    input wire [127:0] rdata,
    input wire [1:0] rresp
);
    localparam RAW_BYTES=((PIXELS+2047)/2048)*2048;
    localparam IDLE=0,AR=1,READ=2;
    reg [1:0] state;
    reg [2:0] beat;
    reg [31:0] address,idle_clocks;
    reg [127:0] data;
    assign block_data=data;
    assign busy=state!=IDLE;
    assign araddr=address;
    assign arlen=0;
    assign arvalid=state==AR;
    assign rready=1;
    always @(posedge clk) if(rvalid && (state==READ || (state==AR && arready)))
        data<=rdata;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;beat<=0;address<=0;locked<=0;locked_base<=0;
            done<=0;error<=0;idle_clocks<=0;
        end else begin
            done<=0;
            if(open && state==IDLE) begin locked<=1;locked_base<=selected_base;idle_clocks<=0;end
            if(close && state==IDLE) locked<=0;
            if(request || !locked || state!=IDLE) idle_clocks<=0;
            else if(idle_clocks==CLOCK_HZ*30-1) locked<=0;
            else idle_clocks<=idle_clocks+1'b1;
            if(state==IDLE && request) begin
                address<=locked_base+PIXELS*2+
                    (block_index>=16'hfff8 ? RAW_BYTES+{25'd0,block_index[2:0],4'd0}:
                     block_index<PIXELS/16?block_index*16:RAW_BYTES+(block_index-PIXELS/16)*16);
                error<=0;beat<=0;state<=AR;
            end
            if(state==AR && arready) state<=READ;
            if(rvalid && (state==READ || (state==AR && arready))) begin
                beat<=beat+1'b1;
                if(rresp!=0) error<=1;
                if(rlast) begin error<=error || rresp!=0 || beat!=0;done<=1;state<=IDLE;end
            end
        end
    end
endmodule
