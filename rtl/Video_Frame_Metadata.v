`timescale 1ns/1ps
// Eight display-slot records, serialized into a synchronous 20-bit RAM.
// publish is accepted only while publish_ready is high. Snapshot the entire
// input on that edge; the producer may change its data immediately afterwards.
// The allocator protects the reader and media slots. Even a same-slot overwrite
// cancels an in-progress read, so no partial record can cross to pixel_clk.
// Read selection occurs AFTER VSYNC: update throughout vertical blanking,
// including the back porch, and hold both outputs during active video.
module Video_Frame_Metadata #(
    parameter integer WIDTH=13, EXTERNAL_READ=0,
    parameter [WIDTH-1:0] DEFAULT_DATA={1'b1,12'd96}
)(
    input wire clk,rst_n,publish,
    input wire [2:0] write_index,read_index,
    input wire read_valid,
    input wire [WIDTH-1:0] published_data,
    // Optional immutable photo record, loaded by the shared DDR reader.
    // Reuse the live RAM reader's bundled CDC and pixel holding register.
    input wire external_select,external_valid,
    input wire external_word_valid,
    input wire [4:0] external_word_index,
    input wire [19:0] external_word_data,
    output wire external_word_ready,
    input wire [31:0] external_tag,
    input wire pixel_clk,frame_blanking,
    output wire publish_ready,
    output reg [WIDTH-1:0] frame_data,
    output reg frame_valid,
    output reg frame_external,
    output reg [31:0] frame_tag
);
    localparam integer WORD_BITS=20;
    localparam integer WORDS=(WIDTH+WORD_BITS-1)/WORD_BITS;
    localparam integer WORD_ADDR_BITS=(WORDS<=1)?1:$clog2(WORDS);
    localparam integer PAD_BITS=WORDS*WORD_BITS;
    localparam integer RAM_ADDR_BITS=WORD_ADDR_BITS+(EXTERNAL_READ?4:3);
    // A ninth record uses spare depth in the same RAM10 in the six-target
    // configuration (512x20). Live allocation still has exactly eight slots.
    reg [WORD_BITS-1:0] slot_words[0:((EXTERNAL_READ?16:8)<<WORD_ADDR_BITS)-1];
    reg [WORD_BITS-1:0] ram_q;
    reg [7:0] slot_valid;
    reg write_busy;
    reg [2:0] write_slot;
    reg [WORD_ADDR_BITS-1:0] write_word;
    reg [PAD_BITS-1:0] write_shift;
    reg read_busy,read_phase,snapshot_current;
    reg [3:0] read_slot;
    reg [WORD_ADDR_BITS-1:0] read_word;
    assign publish_ready=!write_busy;
    assign external_word_ready=EXTERNAL_READ && !write_busy;
    wire ram_write_enable=rst_n && (write_busy || EXTERNAL_READ && external_word_valid);
    wire [RAM_ADDR_BITS-1:0] ram_write_address=write_busy?{write_slot,write_word}:
        (8<<WORD_ADDR_BITS)+external_word_index;
    wire [RAM_ADDR_BITS-1:0] ram_read_address={read_slot,read_word};
    wire [WORD_BITS-1:0] ram_write_data=write_busy?write_shift[WORD_BITS-1:0]:external_word_data;

    // The RAM itself has no reset. Valid bits hide old/uninitialized words.
    always @(posedge clk) begin
        if(ram_write_enable)
            slot_words[ram_write_address]<=ram_write_data;
        if(read_busy && !read_phase)
            ram_q<=slot_words[ram_read_address];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            slot_valid<=0;
            write_busy<=0;write_slot<=0;write_word<=0;write_shift<=0;
        end else begin
            if(write_busy) begin
                write_shift<=write_shift>>WORD_BITS;
                if(write_word==WORDS-1) begin
                    write_busy<=0;slot_valid[write_slot]<=1;
                end else write_word<=write_word+1'b1;
            end
            if(publish && publish_ready) begin
                write_busy<=1;write_slot<=write_index;write_word<=0;
                write_shift<=published_data;slot_valid[write_index]<=0;
            end
        end
    end

    // Assemble directly into the bundled-data handshake holding register.
    // Do not modify it between request and acknowledgement. This avoids a
    // second wide source register while retaining the existing CDC protocol.
    reg [PAD_BITS-1:0] payload_hold;
    reg payload_valid,request_toggle,acknowledge_toggle;
    reg payload_external;
    reg [31:0] payload_tag;
    wire use_external=EXTERNAL_READ && external_select;
    (* async_reg = "true" *) reg [1:0] acknowledge_sync;
    (* async_reg = "true" *) reg [2:0] request_sync;
    wire [3:0] selected_slot=use_external?4'd8:{1'b0,read_index};
    wire selected_ready=use_external?external_valid:read_valid && slot_valid[read_index] &&
        !(write_busy && write_slot==read_index) &&
        !(publish && write_index==read_index);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            payload_hold<=DEFAULT_DATA;payload_valid<=0;request_toggle<=0;
            payload_external<=0;payload_tag<=0;
            acknowledge_sync<=0;read_busy<=0;read_phase<=0;read_slot<=0;read_word<=0;snapshot_current<=0;
        end else begin
            acknowledge_sync<={acknowledge_sync[0],acknowledge_toggle};
            if(publish && write_index==read_slot && !payload_external) snapshot_current<=0;
            if(read_busy) begin
                if(!selected_ready || selected_slot!=read_slot || use_external!=payload_external ||
                    (use_external && external_tag!=payload_tag)) begin
                    read_busy<=0;read_phase<=0;
                end else if(!read_phase) read_phase<=1;
                else begin
                    payload_hold<=(payload_hold>>WORD_BITS) |
                        ({{(PAD_BITS-WORD_BITS){1'b0}},ram_q}<<(PAD_BITS-WORD_BITS));
                    read_phase<=0;
                    if(read_word==WORDS-1) begin
                        read_busy<=0;payload_valid<=1;request_toggle<=~request_toggle;snapshot_current<=1;
                    end else read_word<=read_word+1'b1;
                end
            end else if(request_toggle==acknowledge_sync[1]) begin
                if(selected_ready) begin
                    if(!snapshot_current || read_slot!=selected_slot || payload_external!=use_external ||
                        (use_external && payload_tag!=external_tag)) begin
                        read_busy<=1;read_phase<=0;read_slot<=selected_slot;read_word<=0;snapshot_current<=0;
                        payload_external<=use_external;payload_tag<=use_external?external_tag:32'd0;
                    end
                end else if(payload_valid || payload_external!=use_external ||
                    (use_external && payload_tag!=external_tag)) begin
                    payload_hold<=DEFAULT_DATA;payload_valid<=0;request_toggle<=~request_toggle;snapshot_current<=0;
                    payload_external<=use_external;payload_tag<=use_external?external_tag:32'd0;
                end
            end
        end
    end
    // Accept the bundled payload directly into the frame holding register in
    // blanking. The source already holds it until acknowledgement: a second
    // WIDTH-bit pixel staging copy is unnecessary. RAM publications continue
    // independently while an active display defers this acknowledgement.
    always @(posedge pixel_clk or negedge rst_n) begin
        if(!rst_n) begin
            request_sync<=0;acknowledge_toggle<=0;frame_data<=DEFAULT_DATA;frame_valid<=0;
            frame_external<=0;frame_tag<=0;
        end else begin
            request_sync<={request_sync[1:0],request_toggle};
            if(frame_blanking && request_sync[2]!=acknowledge_toggle) begin
                frame_data<=payload_hold[WIDTH-1:0];frame_valid<=payload_valid;
                frame_external<=payload_external;frame_tag<=payload_tag;
                acknowledge_toggle<=request_sync[2];
            end
        end
    end
    // synthesis translate_off
    initial if(WIDTH<1) $fatal(1,"Metadata WIDTH must be positive");
    always @(posedge clk) if(rst_n && publish && !publish_ready)
        $fatal(1,"Metadata publication while storage busy");
    // synthesis translate_on
endmodule
