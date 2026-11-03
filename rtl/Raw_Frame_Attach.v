`timescale 1ns/1ps
// Append a lossless Bayer plane and immutable replay header before publishing
// the display slot. The caller retains its OUT token and input bank until done.
// One-beat register buffer; no full-frame on-chip storage or extra wide RAM.
module Raw_Frame_Attach #(parameter PIXELS=1280*720,SLOT_BITS=22)(
    input wire clk,rst_n,start,
    input wire [31:0] source_base,
    input wire [2:0] slot,
    input wire [1023:0] header,
    output wire busy,
    output reg done,error,
    output wire [31:0] araddr,awaddr,
    output wire [7:0] arlen,awlen,
    output wire arvalid,rready,awvalid,wvalid,wlast,
    output wire [127:0] wdata,
    input wire arready,rvalid,rlast,awready,wready,bvalid,
    input wire [127:0] rdata,
    input wire [1:0] rresp,bresp
);
    localparam RAW_BYTES=((PIXELS+2047)/2048)*2048;
    localparam IDLE=0,AR=1,READ=2,AW=3,WRITE=4,RESP=5,HEADER_AW=6,HEADER_W=7,HEADER_B=8;
    reg [3:0] state;
    reg [2:0] beat;
    reg [31:0] offset,source,destination;
    // One explicit register word avoids inferring eight wide RAM10s for a
    // tiny burst buffer. Reads remain continuous; only WVALID can have gaps.
    reg [127:0] data0;
    wire header_phase=state>=HEADER_AW;
    assign busy=state!=IDLE;
    assign araddr=source+offset;
    assign arlen=0;
    assign arvalid=state==AR;
    assign rready=1;
    assign awaddr=destination+(header_phase?RAW_BYTES:offset);
    assign awlen=header_phase?8'd7:8'd0;
    assign awvalid=state==AW || state==HEADER_AW;
    assign wvalid=state==WRITE || state==HEADER_W;
    assign wlast=beat==(header_phase?7:0);
    wire [127:0] copy_data=data0;
    assign wdata=header_phase?header[beat*128 +: 128]:copy_data;
    always @(posedge clk) if(rvalid && (state==READ || state==AR && arready)) begin
        data0<=rdata;
    end
    task next_copy;
        begin
            beat<=0;
            if(bresp!=0) begin error<=1;done<=1;state<=IDLE;end
            else if(offset+16==RAW_BYTES) state<=HEADER_AW;
            else begin offset<=offset+16;state<=AR;end
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;beat<=0;offset<=0;source<=0;destination<=0;done<=0;error<=0;
        end else begin
            done<=0;
            case(state)
                IDLE:if(start) begin
                    source<=source_base;destination<=({29'd0,slot}<<SLOT_BITS)+PIXELS*2;
                    offset<=0;beat<=0;error<=0;state<=AR;
                end
                AR:if(arready) state<=READ;
                READ:begin end
                AW:if(awready) begin beat<=0;state<=WRITE;end
                WRITE:if(wready) begin
                    if(beat==0) begin
                        if(bvalid) next_copy();else state<=RESP;
                    end else beat<=beat+1'b1;
                end
                RESP:if(bvalid) next_copy();
                HEADER_AW:if(awready) begin beat<=0;state<=HEADER_W;end
                HEADER_W:if(wready) begin
                    if(beat==7) begin
                        if(bvalid) begin error<=bresp!=0;done<=1;state<=IDLE;end
                        else state<=HEADER_B;
                    end else beat<=beat+1'b1;
                end
                HEADER_B:if(bvalid) begin error<=bresp!=0;done<=1;state<=IDLE;end
                default:state<=IDLE;
            endcase
            if(rvalid && (state==READ || state==AR && arready)) begin
                beat<=beat+1'b1;
                if(rresp!=0) error<=1;
                if(rlast) begin
                    if(error || rresp!=0 || beat!=0) begin error<=1;done<=1;state<=IDLE;end
                    else state<=AW;
                end
            end
        end
    end
endmodule
