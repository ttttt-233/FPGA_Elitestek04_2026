`timescale 1ns/1ps
// Eight centers per physical track bank, plus two coherent display snapshots
// share ONE 256x20 RAM10. Coordinates are quantized to 2x2 pixels (10+9 bits).
// This module consumes the existing matcher's bank mapping; it never matches.
module Shape_Track_Trail #(parameter TARGETS=6,WIDTH=1280,HEIGHT=720,
    parameter IB=TARGETS<2?1:$clog2(TARGETS))(
    input wire clk,rst_n,publish,
    input wire [2:0] frame_slot,
    input wire [TARGETS*51-1:0] boxes,
    input wire [TARGETS*8-1:0] ids,
    input wire [TARGETS-1:0] reset_history,
    input wire [TARGETS*IB-1:0] banks,
    input wire [TARGETS-1:0] observed,visible,
    output wire ready,
    input wire pixel_clk,pixel_rst_n,frame_blanking,
    input wire [11:0] x,y,
    input wire [2:0] display_slot,
    input wire [TARGETS*8-1:0] display_ids,
    input wire live_enable,
    output wire dot,outline
);
    reg [19:0] points[0:255];
    reg [19:0] source_q,pixel_q;
    reg [2:0] heads[0:TARGETS-1];
    reg [3:0] counts[0:TARGETS-1];
    reg [TARGETS*IB-1:0] banks_hold;
    reg [TARGETS-1:0] seen_hold,new_hold,visible_hold;
    reg [TARGETS*8-1:0] ids_hold;
    reg [2:0] slot_hold;
    reg [2:0] state;
    localparam IDLE=0,APPEND=1,COPY_READ=2,COPY_WRITE=3,EXPORT=4,BRIDGE_READ=5,BRIDGE_SEND=6;
    // All point-RAM accesses use clk. The pixel rasterizer requests one held
    // word at a time, so no third RAM port / replicated memory is inferred.
    reg word_request,word_reply;
    reg [7:0] word_address;
    reg [19:0] word_payload;
    (* async_reg="true" *) reg [1:0] word_request_sync;
    (* async_reg="true" *) reg [2:0] word_reply_sync;
    reg [IB-1:0] target;
    reg [2:0] point_index;
    reg request_toggle,pixel_ack;
    (* async_reg="true" *) reg [1:0] ack_sync;
    (* async_reg="true" *) reg [2:0] request_sync;
    reg [TARGETS*8-1:0] snapshot_ids;
    reg [TARGETS-1:0] snapshot_visible;
    reg [2:0] snapshot_slot;
    reg [TARGETS*8-1:0] pixel_ids;
    reg [TARGETS-1:0] pixel_visible;
    reg [2:0] pixel_slot;
    reg active_bank,snapshot_valid;
    wire [IB-1:0] selected_bank=banks_hold[target*IB +: IB];
    // The existing tracker holds boxes until this short transaction finishes.
    // Share ONE pair of center adders instead of 12 adders + a center snapshot.
    // Caller must hold boxes until ready returns; all other inputs snapshot.
    wire [50:0] selected_box=boxes[target*51 +: 51];
    wire [12:0] cx={1'b0,selected_box[47:36]}+selected_box[23:12];
    wire [12:0] cy={1'b0,selected_box[35:24]}+selected_box[11:0];
    wire [2:0] append_index=new_hold[target]?3'd0:heads[selected_bank];
    // Export newest first; temporal weight is implicit in snapshot address.
    wire [2:0] history_index=heads[selected_bank]-3'd1-point_index;
    wire [7:0] master_address={2'b00,selected_bank,history_index};
    wire [7:0] append_address={2'b00,selected_bank,append_index};
    wire [7:0] copy_address={1'b0,!request_toggle,6'b0}+target*8+point_index+8'd64;
    // copy_address: bank 1 -> 128..175; bank 0 -> 64..111.
    assign ready=state==IDLE;
    wire source_write=state==APPEND && seen_hold[target] || state==COPY_WRITE;
    wire [7:0] source_address=state==APPEND?append_address:
        state==COPY_WRITE?copy_address:
        (state==BRIDGE_READ || state==BRIDGE_SEND)?word_address:master_address;
    wire [19:0] source_data=state==APPEND?{1'b1,cy[10:2],cx[11:2]}:
        {visible_hold[target] && point_index<counts[selected_bank],source_q[18:0]};
    wire [5:0] pixel_point;
    wire [7:0] pixel_address=(active_bank?8'd128:8'd64)+pixel_point;
    always @(posedge clk) begin
        if(source_write) points[source_address]<=source_data;
        source_q<=points[source_address];
    end
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;target<=0;point_index<=0;banks_hold<=0;
            seen_hold<=0;new_hold<=0;visible_hold<=0;ids_hold<=0;slot_hold<=0;
            snapshot_ids<=0;snapshot_visible<=0;snapshot_slot<=0;
            request_toggle<=0;ack_sync<=0;
            word_reply<=0;word_payload<=0;word_request_sync<=0;
            for(k=0;k<TARGETS;k=k+1) begin heads[k]<=0;counts[k]<=0;end
        end else begin
            ack_sync<={ack_sync[0],pixel_ack};
            word_request_sync<={word_request_sync[0],word_request};
            case(state)
                IDLE:if(publish) begin
                    banks_hold<=banks;seen_hold<=observed;visible_hold<=visible;
                    ids_hold<=ids;slot_hold<=frame_slot;
                    for(k=0;k<TARGETS;k=k+1) begin
                        new_hold[k]<=reset_history[k];
                    end
                    target<=0;state<=APPEND;
                end else if(word_request_sync[1]!=word_reply) state<=BRIDGE_READ;
                APPEND:begin
                    if(seen_hold[target]) begin
                        heads[selected_bank]<=append_index+1'b1;
                        counts[selected_bank]<=new_hold[target]?4'd1:
                            counts[selected_bank]==8?4'd8:counts[selected_bank]+1'b1;
                    end
                    if(target==TARGETS-1) begin
                        target<=0;point_index<=0;
                        // A paused pixel clock can skip snapshots, never track
                        // history updates or video publication.
                        state<=request_toggle==ack_sync[1]?COPY_READ:IDLE;
                    end else target<=target+1'b1;
                end
                COPY_READ:state<=COPY_WRITE;
                COPY_WRITE:begin
                    if(point_index==7) begin
                        point_index<=0;
                        if(target==TARGETS-1) state<=EXPORT;
                        else begin target<=target+1'b1;state<=COPY_READ;end
                    end else begin point_index<=point_index+1'b1;state<=COPY_READ;end
                end
                EXPORT:begin
                    snapshot_ids<=ids_hold;snapshot_visible<=visible_hold;snapshot_slot<=slot_hold;
                    request_toggle<=!request_toggle;state<=IDLE;
                end
                BRIDGE_READ:state<=BRIDGE_SEND;
                BRIDGE_SEND:begin
                    word_payload<=source_q;word_reply<=word_request_sync[1];state<=IDLE;
                end
                default:state<=IDLE;
            endcase
        end
    end
    // No point memory reset: count/valid bits hide old tracks and RAM contents.
    // Destination acknowledges only in vblank; inactive snapshot RAM is then
    // free for source writes throughout the next active frame.
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            request_sync<=0;pixel_ack<=0;active_bank<=0;snapshot_valid<=0;
            pixel_ids<=0;pixel_visible<=0;pixel_slot<=0;
        end else begin
            request_sync<={request_sync[1:0],request_toggle};
            if(frame_blanking && request_sync[2]!=pixel_ack) begin
                active_bank<=request_sync[2];pixel_ack<=request_sync[2];snapshot_valid<=1;
                pixel_ids<=snapshot_ids;pixel_visible<=snapshot_visible;pixel_slot<=snapshot_slot;
            end
        end
    end
    // Prepare the next two pixel rows. Each 2x2 cell carries four foreground
    // and four footprint bits. Two 1024x8 masks ping-pong; the inactive RAM's
    // read port serves OR writes while the active RAM serves video.
    // 640 clear cycles + 48 bounded word handshakes + <=180 cell OR writes.
    localparam COLS=(WIDTH+1)/2,AB=$clog2(COLS);
    localparam P_IDLE=0,P_CLEAR=1,P_READ=2,P_WAIT=3,P_TEST=4,P_MASK_READ=5,P_MASK_WRITE=6;
    reg [2:0] pstate;
    reg [AB-1:0] clear_x;
    reg [8:0] prepare_y;
    reg [IB-1:0] scan_target;
    reg [2:0] scan_point,stamp_index;
    reg blank_previous,ack_previous;
    reg [1:0] line_valid;
    reg [7:0] mask0[0:(1<<AB)-1],mask1[0:(1<<AB)-1];
    reg [7:0] q0,q1;
    assign pixel_point=scan_target*8+scan_point;
    wire eligible=snapshot_valid && pixel_visible[scan_target] &&
        pixel_ids[scan_target*8 +: 8]==display_ids[scan_target*8 +: 8] && pixel_q[19];
    // Constant glyphs: no per-pixel point comparisons or line multipliers.
    // Newest: radius-3 diamond with radius-4 footprint (9x9 overall).
    // Ages 1..4: 5x5 with outline; ages 5..7: 3x3 with outline.
    // Each byte covers a 2x2 cell, low nibble foreground, high footprint.
    // Five cells span dx=-4..5; row_delta is in pairs of pixel rows.
    function [39:0] glyph;
        input [1:0] family;
        input [9:0] row_delta;
        begin
            case(family)
                0: case(row_delta)
                    10'd1022: glyph = 40'h0000d48000;
                    10'd1023: glyph = 40'h00d4fffe80;
                    10'd0: glyph = 40'h10f7ffffb2;
                    10'd1: glyph = 40'h0010f7b200;
                    10'd2: glyph = 40'h0000100000;
                    default: glyph=0;
                endcase
                1: case(row_delta)
                    10'd1022: glyph = 40'h00c0c0c080;
                    10'd1023: glyph = 40'h00f5ffffa0;
                    10'd0: glyph = 40'h00f5ffffa0;
                    10'd1: glyph = 40'h00f1f3f3a0;
                    default: glyph=0;
                endcase
                2: case(row_delta)
                    10'd1023: glyph = 40'h0050fcf800;
                    10'd0: glyph = 40'h0050fffa00;
                    10'd1: glyph = 40'h0010303000;
                    default: glyph=0;
                endcase
                default: glyph=0;
            endcase
        end
    endfunction
    wire [1:0] family=scan_point==0?2'd0:scan_point<5?2'd1:2'd2;
    wire [39:0] glyph_bits=glyph(family,{1'b0,prepare_y}-{1'b0,pixel_q[18:10]});
    wire [7:0] stamp_bits=glyph_bits[stamp_index*8 +: 8];
    // Extra unsigned bit catches left underflow before address truncation.
    wire [10:0] cell_x={1'b0,pixel_q[9:0]}+{8'd0,stamp_index}-11'd2;
    wire [AB-1:0] stamp_x=cell_x[AB-1:0];
    wire last_cell=stamp_index==(scan_point==0?3'd4:3'd3);
    wire writing=pstate==P_CLEAR || pstate==P_MASK_WRITE && cell_x<COLS;
    wire [AB-1:0] write_x=pstate==P_CLEAR?clear_x:stamp_x;
    wire [7:0] merged=(prepare_y[0]?q1:q0)|stamp_bits;
    always @(posedge pixel_clk) begin
        if(writing && !prepare_y[0]) mask0[write_x]<=pstate==P_CLEAR?8'b0:merged;
        if(writing && prepare_y[0]) mask1[write_x]<=pstate==P_CLEAR?8'b0:merged;
        q0<=mask0[!prepare_y[0] && pstate!=P_IDLE?stamp_x:x[AB:1]];
        q1<=mask1[prepare_y[0] && pstate!=P_IDLE?stamp_x:x[AB:1]];
    end
    reg dot_valid,display_bank;
    reg [1:0] display_bit;
    assign dot=(display_bank?q1[display_bit]:q0[display_bit]) && dot_valid;
    // Foreground wins across overlaps, so an outline never erases a center.
    // Both layers have the same one-pixel delay as the existing video OSD.
    assign outline=(display_bank?q1[{1'b1,display_bit}]:q0[{1'b1,display_bit}]) && dot_valid && !dot;
    wire advance=pstate==P_TEST && (!eligible || glyph_bits==0 || pixel_q[9:0]>=COLS) ||
        pstate==P_MASK_WRITE && last_cell;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            pstate<=P_IDLE;clear_x<=0;prepare_y<=0;scan_target<=0;scan_point<=0;
            blank_previous<=0;ack_previous<=0;line_valid<=0;dot_valid<=0;
            stamp_index<=0;display_bank<=0;display_bit<=0;
            word_request<=0;word_address<=0;word_reply_sync<=0;pixel_q<=0;
        end else begin
            blank_previous<=frame_blanking;ack_previous<=pixel_ack;
            word_reply_sync<={word_reply_sync[1:0],word_reply};
            display_bank<=y[1];display_bit<={y[0],x[0]};
            dot_valid<=live_enable && !frame_blanking && x<WIDTH && y<HEIGHT &&
                snapshot_valid && pixel_slot==display_slot && line_valid[y[1]];
            if(frame_blanking && (!blank_previous || ack_previous!=pixel_ack)) begin
                prepare_y<=0;clear_x<=0;line_valid<=0;pstate<=P_CLEAR;
            end else if(!frame_blanking && x==0 && !y[0]) begin
                prepare_y<=y[9:1]+1'b1;clear_x<=0;line_valid[!y[1]]<=0;pstate<=P_CLEAR;
            end else if(advance) begin
                if(scan_point==7) begin
                    scan_point<=0;
                    if(scan_target==TARGETS-1) begin line_valid[prepare_y[0]]<=1;pstate<=P_IDLE;end
                    else begin scan_target<=scan_target+1'b1;pstate<=P_READ;end
                end else begin scan_point<=scan_point+1'b1;pstate<=P_READ;end
            end else case(pstate)
                P_CLEAR:if(clear_x==COLS-1) begin
                    scan_target<=0;scan_point<=0;pstate<=P_READ;
                end else clear_x<=clear_x+1'b1;
                P_READ:if(word_reply_sync[2]==word_request) begin
                    word_address<=pixel_address;word_request<=!word_request;pstate<=P_WAIT;
                end
                P_WAIT:if(word_reply_sync[2]==word_request) begin pixel_q<=word_payload;pstate<=P_TEST;end
                P_TEST:begin
                    stamp_index<=scan_point<5?3'd0:3'd1;pstate<=P_MASK_READ;
                end
                P_MASK_READ:pstate<=P_MASK_WRITE;
                P_MASK_WRITE:begin
                    stamp_index<=stamp_index+1'b1;pstate<=P_MASK_READ;
                end
                default:pstate<=P_IDLE;
            endcase
        end
    end
    // synthesis translate_off
    initial if(TARGETS>8 || WIDTH>2048 || HEIGHT>1024) $fatal(1,"Unsupported trail geometry");
    always @(posedge clk) if(rst_n && publish && !ready) $fatal(1,"Trail publication while busy");
    // synthesis translate_on
endmodule
