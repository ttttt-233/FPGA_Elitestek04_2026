`timescale 1ns/1ps
// A single, bounded burst buffer shared by PRE sampling, SNAP and clip copies.
// Source RGB565 has pixel 0 in bits 127:112, as in the existing display FIFO.
// All PHOTO/VIDEO/PRE copies preserve every RGB565 bit. No frame-sized RAM.
module Media_Burst_DMA #(parameter BEATS=128)(
    input wire clk,rst_n,start,
    input wire [31:0] source_address,destination_address,
    output wire busy,
    output reg done,error,
    output wire [31:0] araddr,awaddr,
    output wire [7:0] arlen,awlen,
    output wire arvalid,rready,awvalid,wvalid,wlast,
    output reg [127:0] wdata,
    input wire arready,rvalid,rlast,awready,wready,bvalid,
    input wire [127:0] rdata,
    input wire [1:0] rresp,bresp
);
    localparam IDLE=0,AR=1,READ=2,AW=3,LOAD=4,WRITE=5,RESP=6;
    reg [2:0] state;
    reg read_error;
    reg [31:0] src,dst;
    reg [8:0] read_count;
    reg [7:0] send_count;
    reg [127:0] buffer[0:BEATS-1];
    localparam AB=$clog2(BEATS);
    // One explicit address per port is essential: separate indexed writes in
    // two branches can be synthesized as a register file rather than RAM.
    wire [AB-1:0] write_address=read_count;
    wire [AB-1:0] read_address=state==LOAD?0:send_count+1'b1;
    wire write_enable=state==READ && rvalid && rready;
    wire read_enable=state==LOAD || (state==WRITE && wready && !wlast);
    always @(posedge clk) if(write_enable) buffer[write_address]<=rdata;
    always @(posedge clk) if(read_enable) wdata<=buffer[read_address];
    assign busy=state!=IDLE;
    assign araddr=src;
    assign awaddr=dst;
    assign arlen=BEATS-1;
    assign awlen=BEATS-1;
    assign arvalid=state==AR;
    assign rready=state==READ;
    assign awvalid=state==AW;
    assign wvalid=state==WRITE;
    assign wlast=send_count==BEATS-1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;read_error<=0;src<=0;dst<=0;
            read_count<=0;send_count<=0;done<=0;error<=0;
        end else begin
            done<=0;
            case(state)
                IDLE:if(start) begin
                    src<=source_address;dst<=destination_address;
                    read_count<=0;send_count<=0;read_error<=0;error<=0;state<=AR;
                end
                AR:if(arready) state<=READ;
                READ:if(rvalid && rready) begin
                    if(rresp!=0) read_error<=1;
                    read_count<=read_count+1'b1;
                    if(rlast) begin
                        if(read_error || rresp!=0 || read_count!=BEATS-1) begin
                            error<=1;done<=1;state<=IDLE;
                        end else state<=AW;
                    end
                end
                AW:if(awready) state<=LOAD;
                LOAD:state<=WRITE;
                WRITE:if(wready) begin
                    if(wlast) begin
                        if(bvalid) begin error<=bresp!=0;done<=1;state<=IDLE;end
                        else state<=RESP;
                    end else send_count<=send_count+1'b1;
                end
                RESP:if(bvalid) begin error<=bresp!=0;done<=1;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
