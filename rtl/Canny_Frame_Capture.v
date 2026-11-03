`timescale 1ns/1ps
// Capture one complete frontend frame in a caller-owned bank. Requests made
// while busy are ignored; the base is latched until all writes have completed.
// 32-bit records are written in raster order, first record in low AXI word.
// WIDTH*HEIGHT must be a multiple of 512 (one 128-beat burst).
module Canny_Frame_Capture #(
    parameter integer PIXELS=1280*720,DYNAMIC_BASE=0,METADATA_WIDTH=13,
    parameter integer RAW8=0,FRAME_GATE=0,
    parameter [31:0] BASE=32'h02000000
)(
    input wire clk,rst_n,start,pclk,vs,de,
    input wire frame_gate,
    output wire capture_active,
    output wire [1:0] pixel_diagnostic,
    input wire [31:0] record_data,
    input wire [METADATA_WIDTH-1:0] frame_metadata,
    output reg [METADATA_WIDTH-1:0] captured_metadata,
    input wire [31:0] base_address,
    output wire busy,
    output reg done,error,
    output wire frame_started,
    output wire [31:0] awaddr,
    output wire [7:0] awlen,
    output wire awvalid,wvalid,wlast,
    output wire [127:0] wdata,
    input wire awready,wready,bvalid,
    input wire [1:0] bresp
);
    localparam IDLE=0,RESET_FIFO=1,SETTLE=2,WAIT_DATA=3,AW=4,WD=5,B=6;
    reg [2:0] state;
    reg [7:0] delay_count;
    reg request_toggle;
    reg [31:0] written;
    reg [31:0] frame_base;
    reg [6:0] beat;
    // Never decode a multi-bit FSM directly onto an asynchronous FIFO reset:
    // unrelated state transitions can glitch that decode after placement.
    reg fifo_reset_request;
    wire fifo_reset=!rst_n || fifo_reset_request;
    reg [2:0] req_sync,ack_sync;
    reg ack_toggle,armed,capturing,vs_d,pixel_error;
    reg begin_toggle,begin_seen;
    (* async_reg = "true" *) reg [2:0] begin_sync;
    assign frame_started=begin_sync[2]!=begin_seen;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin begin_sync<=0;begin_seen<=0;end
        else begin begin_sync<={begin_sync[1:0],begin_toggle};begin_seen<=begin_sync[2];end
    end
    reg [31:0] pixel_count;
    // Held across the synchronized end acknowledgement; sample only at DONE.
    assign pixel_diagnostic={pixel_count!=PIXELS,pixel_error};
    localparam FRAME_BYTES=RAW8?((PIXELS+2047)/2048)*2048:PIXELS*4;
    reg [23:0] raw_pack;
    reg [8:0] pad_words;
    reg padding;
    assign capture_active=capturing;
    // Bundled CDC: held from actual VS start until the next capture request.
    // System sampling is after the synchronized end acknowledgement + 32 clocks.
    reg [METADATA_WIDTH-1:0] metadata_hold;
    wire fifo_full,fifo_empty,fifo_pempty,fifo_rst_busy;
    wire [127:0] fifo_data;
    wire frame_begin=armed && vs && !vs_d && (!FRAME_GATE || frame_gate);
    wire taking_pixel=(capturing || frame_begin) && vs && de;
    wire fifo_write=((taking_pixel && (!RAW8 || pixel_count[1:0]==3)) ||
                     (RAW8 && padding && pad_words!=0)) && !fifo_full;
    wire [31:0] fifo_input=RAW8?(padding?32'd0:{record_data[7:0],raw_pack}):record_data;
    always @(posedge pclk or negedge rst_n) begin
        if(!rst_n) begin
            req_sync<=0;ack_toggle<=0;armed<=0;capturing<=0;vs_d<=0;
            pixel_error<=0;pixel_count<=0;begin_toggle<=0;
            metadata_hold<=0;raw_pack<=0;pad_words<=0;padding<=0;
        end else begin
            req_sync<={req_sync[1:0],request_toggle};vs_d<=vs;
            if(req_sync[2]!=ack_toggle && !armed && !capturing && !padding) begin
                armed<=1;pixel_error<=0;pixel_count<=0;
            end
            if(frame_begin) begin
                armed<=0;capturing<=1;begin_toggle<=req_sync[2];
                metadata_hold<=frame_metadata;
            end
            if(taking_pixel) begin
                raw_pack<={record_data[7:0],raw_pack[23:8]};
                pixel_count<=pixel_count+1'b1;
                if(fifo_full || pixel_count>=PIXELS) pixel_error<=1;
            end
            if(capturing && !vs && vs_d) begin
                capturing<=0;
                if(RAW8 && FRAME_BYTES!=PIXELS) begin
                    padding<=1;pad_words<=(FRAME_BYTES-PIXELS)/4;
                end else ack_toggle<=req_sync[2];
                if(pixel_count!=PIXELS) pixel_error<=1;
            end
            if(padding && !fifo_full) begin
                pad_words<=pad_words-1'b1;
                if(pad_words==1) begin padding<=0;ack_toggle<=req_sync[2];end
            end
        end
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n) ack_sync<=0;else ack_sync<={ack_sync[1:0],ack_toggle};
    reg [127:0] raw_output;
    reg [1:0] raw_word;
    reg raw_valid;
    wire [31:0] raw_q;
    wire raw_pop=state==WD && !raw_valid && !fifo_empty;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin raw_output<=0;raw_word<=0;raw_valid<=0;end
        else if(fifo_reset) begin raw_word<=0;raw_valid<=0;end
        else begin
            if(raw_pop) begin
                raw_output<={raw_q,raw_output[127:32]};raw_word<=raw_word+1'b1;
                if(raw_word==3) raw_valid<=1;
            end
            if(wvalid && wready) raw_valid<=0;
        end
    generate if(RAW8) begin: g_raw_fifo
        Raw_FIFO_32 capture_fifo (
            .reset(fifo_reset),.wr_clk(pclk),.rd_clk(clk),.write(fifo_write),.read(raw_pop),
            .data(fifo_input),.q(raw_q),.full(fifo_full),.empty(fifo_empty),.pempty(fifo_pempty),.rst_busy(fifo_rst_busy));
        assign fifo_data=raw_output;
    end else begin: g_record_fifo
    W0_FIFO_32 capture_fifo (
        .a_rst_i(fifo_reset),.wr_clk_i(pclk),.rd_clk_i(clk),
        .wr_en_i(fifo_write),.wdata(fifo_input),.full_o(fifo_full),
        .rd_en_i(wvalid && wready),.rdata(fifo_data),
        .empty_o(fifo_empty),.prog_empty_o(fifo_pempty),.rst_busy(fifo_rst_busy)
    );
    end endgenerate
    assign busy=state!=IDLE;
    assign awaddr=(DYNAMIC_BASE?frame_base:BASE)+written;
    assign awlen=RAW8?8'd15:8'd127;
    assign awvalid=state==AW;
    assign wvalid=state==WD && (RAW8?raw_valid:!fifo_empty);
    assign wlast=beat==(RAW8?15:127);
    assign wdata=RAW8?fifo_data:{fifo_data[31:0],fifo_data[63:32],fifo_data[95:64],fifo_data[127:96]};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;delay_count<=0;request_toggle<=0;written<=0;
            frame_base<=BASE;
            beat<=0;done<=0;error<=0;fifo_reset_request<=1;
            captured_metadata<=0;
        end else begin
            done<=0;
            case(state)
                IDLE: if(start) begin
                    frame_base<=base_address;
                    error<=0;written<=0;delay_count<=31;state<=RESET_FIFO;fifo_reset_request<=1;
                end
                RESET_FIFO: if(delay_count==0) begin delay_count<=63;state<=SETTLE;fifo_reset_request<=0;end
                            else delay_count<=delay_count-1'b1;
                SETTLE: if(delay_count==0 && !fifo_rst_busy) begin
                    request_toggle<=!request_toggle;delay_count<=0;state<=WAIT_DATA;
                end else if(delay_count!=0) delay_count<=delay_count-1'b1;
                WAIT_DATA: begin
                    // Multi-bit pixel_error is held from before the synchronized
                    // acknowledgement until the next request; wait 32 more clocks.
                    if(ack_sync[2]==request_toggle && delay_count<32) delay_count<=delay_count+1'b1;
                    if(ack_sync[2]==request_toggle && delay_count==32 && (error || pixel_error)) begin
                        error<=1;done<=1;state<=IDLE;
                    end else if(!fifo_pempty && written<FRAME_BYTES) begin beat<=0;state<=AW;end
                    else if(ack_sync[2]==request_toggle && delay_count==32) begin
                        captured_metadata<=metadata_hold;
                        error<=error || written!=FRAME_BYTES || !fifo_empty;
                        done<=1;state<=IDLE;
                    end
                end
                AW: if(awready) state<=WD;
                WD: if(wvalid && wready) begin
                    beat<=beat+1'b1;if(wlast) state<=B;
                end
                B: if(bvalid) begin
                    if(bresp!=0) error<=1;
                    written<=written+(RAW8?32'd256:32'd2048);state<=WAIT_DATA;
                end
                default: state<=IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial if(PIXELS<512 || PIXELS%512!=0) $fatal(1,"Capture requires a whole burst frame");
    // synthesis translate_on
endmodule
