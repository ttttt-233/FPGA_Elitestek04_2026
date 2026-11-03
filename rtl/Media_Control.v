`timescale 1ns/1ps
// PHOTO / VIDEO / PRE metadata and burst scheduler. All time is frame/30.
// A PRE sample leases a complete live slot; clip copies consume PRE oldest first.
// No video pixels are stored on chip. A new VIDEO is published only after its
// final successful write response; overwritten or failed sources never succeed.
module Media_Control #(
    parameter integer PIXELS=1280*720,CLOCK_HZ=96_000_000,BEATS=128,
    parameter integer PHOTO_FRAMES=8,RING_FRAMES=128,VIDEO_FRAMES=112,RAW_PHOTOS=0,
    parameter [31:0] PHOTO_BASE=32'h04000000,
    parameter [31:0] VIDEO_BASE=PHOTO_BASE+PHOTO_FRAMES*(PIXELS*2+(RAW_PHOTOS?((PIXELS+2047)/2048)*2048+2048:0)),
    parameter [31:0] PRE_BASE=VIDEO_BASE+VIDEO_FRAMES*PIXELS*2
)(
    input wire clk,rst_n,enable,
    input wire [2:0] control_group,live_index,
    input wire live_valid,display_media_active,
    input wire [31:0] display_media_base,
    input wire export_lock,
    input wire [31:0] export_base,
    output wire photo_capture_request,
    input wire photo_capture_ready,photo_capture_error,
    input wire [2:0] photo_capture_slot,
    input wire [8:0] operation_request,
    output wire [7:0] live_pin_mask,
    output wire playback_enable,
    output wire [31:0] playback_address,
    output wire [54:0] media_status,
    output reg [2:0] feedback_code,
    output reg feedback_toggle,
    output reg dma_start,
    output reg [31:0] dma_source,dma_destination,
    input wire dma_done,dma_error,
    output reg source_overrun,transfer_error,tick_overrun,
    output wire photo_busy,pre_ready,record_ready
);
    localparam integer TICK=CLOCK_HZ/30,CHUNK=BEATS*16,FRAME_BYTES=PIXELS*2;
    localparam integer PHOTO_BYTES=FRAME_BYTES+(RAW_PHOTOS?((PIXELS+2047)/2048)*2048+2048:0);
    localparam integer TICK_BITS=$clog2(TICK+1);
    localparam [2:0] JOB_RING=1,JOB_PHOTO=2,JOB_VIDEO=3;
    reg [TICK_BITS-1:0] ring_clock,play_clock;
    reg tick_pending,ring_active,photo_pending,photo_active,clip_pending,clip_active,outstanding;
    localparam integer PHOTO_BITS=$clog2(PHOTO_FRAMES);
    reg [2:0] job,ring_source,photo_source;
    reg [PHOTO_BITS-1:0] photo_slot,photo_next;
    reg [8:0] ring_slot,clip_slot,pending_slot;
    reg [31:0] ring_sequence,in_sequence,clip_first,pending_first;
    reg [8:0] ring_count,clip_count,clip_frame,pending_count,video_count,play_frame;
    reg [31:0] ring_offset,photo_offset,clip_offset;
    reg [4:0] photo_count,media_index;
    reg [PHOTO_BITS-1:0] photo_order[0:PHOTO_FRAMES-1];
    reg recording,video_valid,playing;
    reg [2:0] clip_reason,pending_reason;
    wire [8:0] available=ring_count-((ring_active && ring_count==RING_FRAMES)?9'd1:9'd0);
    wire [31:0] oldest=ring_sequence-{23'd0,available};
    wire [31:0] clip_sequence=clip_first+{23'd0,clip_frame};
    wire [31:0] clip_age=ring_sequence-clip_sequence;
    wire clip_readable=clip_age!=0 && clip_age<={23'd0,available} &&
        !(ring_active && clip_slot==ring_slot);
    wire [4:0] media_count=photo_count+{4'd0,video_valid};
    wire selected_valid=media_index!=0 && media_index<=media_count;
    wire selected_video=selected_valid && video_valid && media_index>photo_count;
    wire [8:0] display_frame=play_frame>=video_count && video_count!=0?video_count-1'b1:play_frame;
    wire [31:0] recorded_frames=ring_sequence-in_sequence;
    wire [8:0] rec_frames=recorded_frames>VIDEO_FRAMES?VIDEO_FRAMES:recorded_frames[8:0];
    assign photo_busy=photo_active || photo_pending;
    assign pre_ready=available!=0;
    assign record_ready=recording && recorded_frames!=0;
    function [31:0] frame_address;
        input [31:0] base;
        input [8:0] frame;
        integer b;
        begin
            frame_address=base;
            for(b=0;b<9;b=b+1) if(frame[b]) frame_address=frame_address+(FRAME_BYTES<<b);
        end
    endfunction
    function [31:0] photo_address;
        input [PHOTO_BITS-1:0] frame;
        begin photo_address=PHOTO_BASE+frame*PHOTO_BYTES;end
    endfunction
    function [11:0] time_bcd;
        input [8:0] frames;
        reg [8:0] remainder;
        reg [3:0] seconds,tenths;
        integer i;
        begin
            remainder=frames;seconds=0;
            for(i=0;i<8;i=i+1) if(remainder>=30) begin remainder=remainder-30;seconds=seconds+1'b1;end
            tenths=0;
            for(i=0;i<9;i=i+1) if(remainder>=3) begin remainder=remainder-3;tenths=tenths+1'b1;end
            time_bcd={4'd0,seconds,tenths};
        end
    endfunction
    assign live_pin_mask=(RAW_PHOTOS && photo_capture_ready?(8'b1<<photo_capture_slot):8'd0) |
                         (ring_active?(8'b1<<ring_source):8'd0) |
                         (photo_active?(8'b1<<photo_source):8'd0);
    wire [PHOTO_BITS-1:0] selected_photo_slot=photo_order[(media_index-1'b1)%PHOTO_FRAMES];
    wire photo_display_conflict=(display_media_active && display_media_base==photo_address(photo_next)) ||
        (RAW_PHOTOS && export_lock && export_base==photo_address(photo_next));
    wire video_display_conflict=display_media_active && display_media_base>=VIDEO_BASE && display_media_base<PRE_BASE;
    assign photo_capture_request=RAW_PHOTOS && photo_pending && !photo_display_conflict;
    assign playback_enable=control_group==1 && selected_valid &&
        !(!selected_video && photo_pending && selected_photo_slot==photo_next);
    assign playback_address=selected_video?frame_address(VIDEO_BASE,display_frame):
        photo_address(selected_photo_slot);
    // Preserve the legacy low 52 bits; append high count/index bits for 16 photos + one clip.
    assign media_status={photo_count[4],media_count[4],media_index[4],
        selected_valid,time_bcd(video_count),time_bcd(play_frame),
        selected_video && playing,selected_video,media_count[3:0],media_index[3:0],
        time_bcd(rec_frames),recording,photo_count[3:0]};
    task feedback;
        input [2:0] code;
        begin feedback_code<=code;feedback_toggle<=~feedback_toggle;end
    endtask
    task queue_clip;
        input [31:0] first;
        input [8:0] count;
        input [2:0] reason;
        reg [8:0] age;
        begin
            if(count!=0) begin
                // first is at most RING_FRAMES samples behind the next write.
                // Explicit physical cursors also work for a non-power-of-two ring.
                age=ring_sequence-first;
                pending_slot<=ring_slot>=age ? ring_slot-age : RING_FRAMES+ring_slot-age;
                pending_first<=first;pending_count<=count;pending_reason<=reason;
                clip_pending<=1;clip_active<=0;video_valid<=0;video_count<=0;
                playing<=0;play_frame<=0;
                if(media_index>photo_count) media_index<=photo_count!=0?4'd1:4'd0;
            end
        end
    endtask
    task issue;
        input [2:0] kind;
        input [31:0] src,dst;
        begin
            job<=kind;dma_source<=src;dma_destination<=dst;
            dma_start<=1;outstanding<=1;
        end
    endtask
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ring_clock<=0;play_clock<=0;tick_pending<=0;ring_active<=0;
            photo_pending<=0;photo_active<=0;clip_pending<=0;clip_active<=0;outstanding<=0;
            ring_source<=0;photo_source<=0;photo_slot<=0;photo_next<=0;job<=0;
            ring_sequence<=0;in_sequence<=0;ring_count<=0;
            ring_slot<=0;clip_slot<=0;pending_slot<=0;
            ring_offset<=0;photo_offset<=0;clip_offset<=0;
            clip_first<=0;pending_first<=0;clip_count<=0;pending_count<=0;clip_frame<=0;
            clip_reason<=0;pending_reason<=0;video_count<=0;play_frame<=0;
            photo_count<=0;media_index<=0;recording<=0;video_valid<=0;playing<=0;
            feedback_code<=0;feedback_toggle<=0;dma_start<=0;
            dma_source<=0;dma_destination<=0;source_overrun<=0;transfer_error<=0;tick_overrun<=0;
            for(i=0;i<PHOTO_FRAMES;i=i+1) photo_order[i]<=0;
        end else begin
            dma_start<=0;
            if(!enable || !live_valid) ring_clock<=0;
            else if(ring_clock==TICK-1) begin
                ring_clock<=0;tick_pending<=1;
                if(ring_active || tick_pending) tick_overrun<=1;
            end else ring_clock<=ring_clock+1'b1;

            if(control_group!=1 || !selected_video) begin playing<=0;play_clock<=0;end
            else if(playing) begin
                if(play_clock==TICK-1) begin
                    play_clock<=0;
                    if(play_frame+1'b1>=video_count) begin play_frame<=video_count;playing<=0;feedback(6);end
                    else play_frame<=play_frame+1'b1;
                end else play_clock<=play_clock+1'b1;
            end

            if(dma_done && outstanding) begin
                outstanding<=0;
                if(dma_error) begin
                    transfer_error<=1;
                    case(job)
                        JOB_RING:begin ring_active<=0;if(ring_count==RING_FRAMES) ring_count<=ring_count-1'b1;end
                        JOB_PHOTO:photo_active<=0;
                        JOB_VIDEO:clip_active<=0;
                    endcase
                end else case(job)
                    JOB_RING:if(ring_offset+CHUNK==FRAME_BYTES) begin
                        ring_active<=0;ring_sequence<=ring_sequence+1'b1;
                        ring_slot<=ring_slot==RING_FRAMES-1?9'd0:ring_slot+1'b1;
                        if(ring_count<RING_FRAMES) ring_count<=ring_count+1'b1;
                    end else ring_offset<=ring_offset+CHUNK;
                    JOB_PHOTO:if(photo_offset+CHUNK==PHOTO_BYTES) begin
                        photo_active<=0;photo_order[photo_count]<=photo_slot;
                        photo_count<=photo_count+1'b1;
                        photo_next<=photo_next==PHOTO_FRAMES-1?0:photo_next+1'b1;
                        if(media_index==0) media_index<=1;
                        else if(selected_video) media_index<=media_index+1'b1;
                        feedback(1);
                    end else photo_offset<=photo_offset+CHUNK;
                    JOB_VIDEO:if(clip_active) begin
                        if(clip_offset+CHUNK==FRAME_BYTES) begin
                            clip_offset<=0;
                            if(clip_frame+1'b1==clip_count) begin
                                clip_active<=0;video_valid<=1;video_count<=clip_count;
                                media_index<=photo_count+5'd1;play_frame<=0;feedback(clip_reason);
                            end else begin
                                clip_frame<=clip_frame+1'b1;
                                clip_slot<=clip_slot==RING_FRAMES-1?9'd0:clip_slot+1'b1;
                            end
                        end else clip_offset<=clip_offset+CHUNK;
                    end
                endcase
            end

            // Acquire the live display slot before issuing any source read.
            // The existing frame controller already excludes its old reader
            // on this edge; live_pin_mask preserves that exclusion afterwards.
            if(tick_pending && !ring_active && live_valid && enable) begin
                ring_active<=1;ring_source<=live_index;ring_offset<=0;tick_pending<=0;
            end
            if(photo_capture_error && RAW_PHOTOS) begin photo_pending<=0;transfer_error<=1;end
            if(photo_pending && !photo_active && !photo_display_conflict && live_valid &&
               (!RAW_PHOTOS || photo_capture_ready)) begin
                photo_pending<=0;photo_active<=1;
                photo_source<=RAW_PHOTOS?photo_capture_slot:live_index;photo_offset<=0;photo_slot<=photo_next;
                if(photo_count==PHOTO_FRAMES) begin
                    for(i=0;i<PHOTO_FRAMES-1;i=i+1) photo_order[i]<=photo_order[i+1];
                    photo_count<=PHOTO_FRAMES-1;
                    if(media_index>1) media_index<=media_index-1'b1;
                end
            end
            if(clip_pending && !clip_active && !video_display_conflict && !outstanding) begin
                clip_pending<=0;clip_active<=1;clip_first<=pending_first;
                clip_count<=pending_count;clip_reason<=pending_reason;clip_frame<=0;clip_offset<=0;
                clip_slot<=pending_slot;
            end
            if(!outstanding && !dma_start && enable) begin
                if(ring_active) issue(JOB_RING,
                    {7'd0,ring_source,22'd0}+ring_offset,
                    frame_address(PRE_BASE,ring_slot)+ring_offset);
                else if(photo_active) issue(JOB_PHOTO,
                    {7'd0,photo_source,22'd0}+photo_offset,photo_address(photo_slot)+photo_offset);
                else if(clip_active) begin
                    if(!clip_readable) begin clip_active<=0;source_overrun<=1;end
                    else issue(JOB_VIDEO,frame_address(PRE_BASE,clip_slot)+clip_offset,
                        frame_address(VIDEO_BASE,clip_frame)+clip_offset);
                end
            end
            if(recording && recorded_frames>=VIDEO_FRAMES) begin
                recording<=0;queue_clip(in_sequence,VIDEO_FRAMES,3);
            end
            // Commands are applied last: a newer OUT/PRE cancels an unfinished
            // save at a burst boundary and replaces the sole VIDEO descriptor.
            if(control_group==0) begin
                if(operation_request[0]) begin recording<=1;in_sequence<=ring_count!=0?ring_sequence-1'b1:ring_sequence;end
                if(operation_request[1] && recording) begin
                    recording<=0;
                    if(recorded_frames!=0) begin
                        if(recorded_frames>{23'd0,available}) queue_clip(oldest,available>VIDEO_FRAMES?VIDEO_FRAMES:available,3);
                        else queue_clip(in_sequence,rec_frames,3);
                    end
                end
                if(operation_request[2]) queue_clip(ring_sequence-(available>VIDEO_FRAMES?VIDEO_FRAMES:available),
                    available>VIDEO_FRAMES?VIDEO_FRAMES:available,2);
                if(operation_request[3] && !photo_active && !photo_pending) photo_pending<=1;
            end else if(control_group==1) begin
                if((operation_request[4] || operation_request[5]) && selected_valid) begin
                    if(operation_request[4]) media_index<=media_index==1?media_count:media_index-1'b1;
                    else media_index<=media_index>=media_count?4'd1:media_index+1'b1;
                    playing<=0;play_clock<=0;play_frame<=0;
                end else if(operation_request[6] && selected_video) begin
                    if(!playing && play_frame>=video_count) play_frame<=0;
                    playing<=~playing;play_clock<=0;feedback(playing?3'd5:3'd4);
                end
            end
        end
    end
`ifndef SYNTHESIS
    initial begin
        if(FRAME_BYTES%CHUNK!=0 || PHOTO_FRAMES<2 || PHOTO_FRAMES>16 ||
           RING_FRAMES>511 || VIDEO_FRAMES<1 || VIDEO_FRAMES>256 || VIDEO_FRAMES>=RING_FRAMES)
            $fatal(1,"Invalid media geometry");
        if(PRE_BASE+RING_FRAMES*FRAME_BYTES>32'h20000000) $fatal(1,"Media exceeds conservative 512 MiB range");
    end
`endif
endmodule
