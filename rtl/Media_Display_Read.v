`timescale 1ns/1ps
// Drain offered bursts before applying a frame-boundary source change. The
// existing display FIFO is shared; live and media RGB565 use the same unchanged pixel format.
module Media_Display_Read #(parameter PIXELS=1280*720,BEATS=128,
    RAW_PHOTOS=0,PHOTO_FRAMES=8,TARGETS=6,
    parameter [31:0] PHOTO_BASE=32'h04000000)(
    input wire clk,rst_n,fifo_reset,boundary,blanking,fifo_full,
    input wire live_valid,media_enable,
    input wire [31:0] live_base,media_base,
    output reg media_active,
    output reg [31:0] active_base,
    // Same immutable RAW1 record used by the UART/DNG exporter. Header beats
    // never enter the pixel FIFO; VIDEO/PRE retain their RGB565-only layout.
    output reg photo_metadata_valid,
    output reg [19:0] photo_word,
    output reg [4:0] photo_word_index,
    output wire photo_word_valid,
    input wire photo_word_ready,
    output reg fifo_push,
    output reg [127:0] fifo_data,
    output wire [31:0] araddr,
    output wire [7:0] arlen,
    output wire arvalid,rready,
    input wire arready,rvalid,rlast,
    input wire [127:0] rdata,
    input wire [1:0] rresp
);
    localparam IDLE=0,ADDRESS=1,DATA=2,WORD=3;
    reg [1:0] state;
    reg restart,allowed,drop_data,valid;
    reg [2:0] settle;
    reg [31:0] offset,command_address;
    localparam RAW_BYTES=((PIXELS+2047)/2048)*2048;
    localparam PHOTO_BYTES=PIXELS*2+RAW_BYTES+2048;
    localparam META_BITS=TARGETS*65+15,META_WORDS=(META_BITS+19)/20;
    reg header_pending,command_header,command_word,header_ok,word_pending;
    reg [7:0] word_lanes;
    reg [3:0] header_beat;
    wire selected_photo=RAW_PHOTOS && media_enable && media_base>=PHOTO_BASE &&
        media_base<PHOTO_BASE+PHOTO_FRAMES*PHOTO_BYTES;
    assign araddr=command_address;
    assign arlen=command_header?(command_word?8'd0:8'd7):BEATS-1;
    assign arvalid=state==ADDRESS;
    // The board DDR IP advances burst data even when RREADY is low.
    // Receive every beat continuously; live/media RGB565 need no conversion.
    assign rready=1'b1;
    assign photo_word_valid=state==WORD && !restart && !fifo_reset && !drop_data;
    // Map a compact metadata bit back to the immutable little-endian RAW1
    // header. All calls below have constant arguments: no runtime bit shifter.
    function integer header_bit;
        input integer n;
        begin
            if(n<12) header_bit=192+n;
            else if(n==12) header_bit=176;
            else if(n==13) header_bit=184;
            else if(n<14+TARGETS*51) header_bit=320+n-14;
            else if(n<14+TARGETS*55) header_bit=640+n-14-TARGETS*51;
            else if(n<14+TARGETS*57) header_bit=672+n-14-TARGETS*55;
            else if(n<14+TARGETS*65) header_bit=704+n-14-TARGETS*57;
            else header_bit=296;
        end
    endfunction
    function [2:0] first_lane;
        input [7:0] lanes;
        integer k;
        begin first_lane=0;for(k=7;k>=0;k=k-1) if(lanes[k]) first_lane=k;end
    endfunction
    wire [2:0] word_lane=first_lane(word_lanes);
    wire [7:0] remaining_lanes=word_lanes & ~(8'b1<<word_lane);
    wire [19:0] fragments[0:META_WORDS-1];
    wire [7:0] lane_masks[0:META_WORDS-1];
    genvar w,b,lane;
    generate for(w=0;w<META_WORDS;w=w+1) begin: g_word
        wire [19:0] masks[0:7];
        for(b=0;b<20;b=b+1) begin: g_bit
            if(w*20+b<META_BITS) begin: g_used
                localparam HB=header_bit(w*20+b);
                assign fragments[w][b]=word_lane==HB/128?rdata[HB%128]:1'b0;
                for(lane=0;lane<8;lane=lane+1) begin: g_lane
                    assign masks[lane][b]=lane==HB/128;
                end
            end else begin: g_padding
                assign fragments[w][b]=0;
                for(lane=0;lane<8;lane=lane+1) begin: g_lane
                    assign masks[lane][b]=0;
                end
            end
        end
        for(lane=0;lane<8;lane=lane+1) begin: g_mask
            assign lane_masks[w][lane]=|masks[lane];
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;restart<=1;allowed<=0;drop_data<=1;valid<=0;settle<=0;
            offset<=0;command_address<=0;media_active<=0;active_base<=0;
            fifo_push<=0;fifo_data<=0;
            header_pending<=0;command_header<=0;command_word<=0;header_ok<=0;header_beat<=0;
            word_pending<=0;word_lanes<=0;photo_word<=0;photo_word_index<=0;
            photo_metadata_valid<=0;
        end else begin
            fifo_push<=0;
            if(fifo_reset) begin restart<=1;drop_data<=1;allowed<=0;end
            if(boundary) begin settle<=3;allowed<=0;end
            else if(settle!=0) begin settle<=settle-1'b1;if(settle==1) allowed<=1;end
            if(!blanking) allowed<=0;
            if(state==IDLE) begin
                if(restart && allowed && blanking && !fifo_reset) begin
                    active_base<=media_enable?media_base:live_base;
                    media_active<=media_enable;valid<=media_enable || live_valid;
                    offset<=0;restart<=0;drop_data<=0;allowed<=0;
                    header_pending<=selected_photo &&
                        (!media_active || media_base!=active_base || !photo_metadata_valid);
                    word_pending<=0;
                    if(!selected_photo || !media_active || media_base!=active_base)
                        photo_metadata_valid<=0;
                end else if(!restart && !fifo_reset && valid && header_pending) begin
                    command_address<=active_base+PIXELS*2+RAW_BYTES;
                    command_header<=1;command_word<=0;header_pending<=0;header_beat<=0;header_ok<=1;
                    photo_metadata_valid<=0;state<=ADDRESS;
                    photo_word_index<=0;photo_word<=0;word_lanes<=lane_masks[0];
                end else if(!restart && !fifo_reset && valid && word_pending) begin
                    command_address<=active_base+PIXELS*2+RAW_BYTES+{25'd0,word_lane,4'd0};
                    command_header<=1;command_word<=1;header_beat<=0;state<=ADDRESS;
                end else if(!restart && !fifo_reset && valid && !fifo_full && !fifo_push &&
                            offset<PIXELS*2) begin
                    command_address<=active_base+offset;command_header<=0;state<=ADDRESS;
                end
            end
            // One compact word at a time enters the existing metadata RAM.
            // Live publication may temporarily own its write port. No DDR
            // response is ever throttled, and no wide header bank is added.
            if(state==WORD) begin
                if(restart || fifo_reset || drop_data) begin
                    state<=IDLE;word_pending<=0;photo_metadata_valid<=0;
                end else if(photo_word_ready) begin
                    state<=IDLE;photo_word<=0;
                    if(photo_word_index==META_WORDS-1) begin
                        word_pending<=0;photo_metadata_valid<=header_ok;
                    end else begin
                        photo_word_index<=photo_word_index+1'b1;
                        word_lanes<=lane_masks[photo_word_index+1'b1];
                    end
                end
            end
            if(state==ADDRESS && arready) begin
                if(!command_header) offset<=offset+BEATS*16;
                state<=DATA;
            end
            if(rvalid && (state==DATA || (state==ADDRESS && arready))) begin
                if(command_header) begin
                    if(command_word) begin
                        photo_word<=photo_word | fragments[photo_word_index];
                        word_lanes<=remaining_lanes;
                    end
                    if(!command_word && header_beat==0 && (rdata[31:0]!=32'h31574152 || rdata[47:32]!=1 ||
                        rdata[95:88]!=8)) header_ok<=0;
                    if(!command_word && header_beat==2 && rdata[39:32]!=TARGETS) header_ok<=0;
                    if(!command_word && header_beat==6 && rdata[31:0]!=PIXELS) header_ok<=0;
                    if(rresp!=0 || fifo_reset || drop_data || restart) header_ok<=0;
                    header_beat<=header_beat+1'b1;
                end else if(!fifo_reset && !drop_data && !restart && rresp==0 && valid) begin
                    fifo_push<=1;
                    fifo_data<=rdata;
                end
                if(rresp!=0 && !command_header) begin valid<=0;drop_data<=1;end
                if(rlast) begin
                    state<=IDLE;
                    if(command_header) begin
                        if(header_ok && header_beat==(command_word?0:7) && rresp==0 &&
                            !fifo_reset && !drop_data && !restart) begin
                            word_pending<=1;
                            if(command_word && remaining_lanes==0) state<=WORD;
                        end else begin header_ok<=0;word_pending<=0;end
                    end
                end
            end
        end
    end
endmodule
