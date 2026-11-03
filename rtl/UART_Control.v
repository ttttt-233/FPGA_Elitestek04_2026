`timescale 1ns/1ps
// Fixed command: A5 01 sequence command value_lo value_hi XOR.
// This module decodes protocol only. Existing debug/Media/Frame controllers
// own every setting and operation. No CNN payload or video backpressure.
module UART_Control #(parameter CLOCK_HZ=96000000,BAUD=2000000,TARGETS=6,
    WIDTH=1280,HEIGHT=720,FPS=30,BUILD_ID=16'h0307,PROFILE_ID=0,RAW_EXPORT_ENABLE=0,
    PROFILE_SWITCH_ENABLE=0,
    FRAME_STREAM_ENABLE=0,THRESHOLD_MIN=16,THRESHOLD_MAX=1024)(
    input wire clk,rst_n,rx,
    output wire tx,
    input wire publish,
    input wire [TARGETS*51-1:0] boxes,
    input wire [TARGETS*4-1:0] digits,
    input wire [TARGETS*2-1:0] qualities,
    input wire [TARGETS*8-1:0] ids,ages,lost,
    input wire [TARGETS*13-1:0] vx,vy,
    input wire [TARGETS-1:0] alive,observed,
    input wire [9:0] fps_x10,
    input wire [15:0] failed_frames,
    input wire [15:0] control_state,
    input wire [11:0] manual_threshold,
    input wire [13:0] applied_algorithm,
    input wire [7:0] device_flags,
    input wire [54:0] media_status,
    input wire live_ready,photo_busy,pre_ready,record_ready,source_safe,source_tag,
    input wire key_event,
    input wire profile_switch_busy,
    input wire raw_available,raw_locked,raw_busy,raw_done,raw_error,
    input wire [127:0] raw_data,
    input wire [15:0] raw_diagnostics,
    output wire raw_request,
    output wire [15:0] raw_index,
    input wire [11:0] scan_ms,out_ms,gap_ms,
    input wire profile_valid,
    output wire control_write,
    output reg [3:0] control_address,
    output reg [15:0] control_value
);
    wire [7:0] byte_data;
    wire byte_valid,byte_error,bit_tick;
    UART_RX #(.CLOCK_HZ(CLOCK_HZ),.BAUD(BAUD)) receiver(
        .clk(clk),.rst_n(rst_n),.rx(rx),.data(byte_data),.valid(byte_valid),.error(byte_error),.bit_tick(bit_tick));
    reg [2:0] phase;
    reg [4:0] idle_bits;
    reg [7:0] packet_sequence,command,checksum;
    reg [15:0] value;
    reg packet_valid,bad_checksum;
    reg [7:0] rx_errors,checksum_errors,command_count;
    wire timeout=phase!=0 && bit_tick && idle_bits==31;
    // Packet payload is meaningful only after all six preceding bytes arrived.
    // Reset parser validity, not the unused payload datapath.
    always @(posedge clk) if(byte_valid) begin
        if(phase==0 || phase==6 || phase==1 && byte_data!=1) checksum<=8'ha5;
        else checksum<=checksum^byte_data;
        case(phase)
            2:packet_sequence<=byte_data;
            3:command<=byte_data;
            4:value[7:0]<=byte_data;
            5:value[15:8]<=byte_data;
            6:bad_checksum<=checksum!=byte_data;
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            phase<=0;idle_bits<=0;
            packet_valid<=0;rx_errors<=0;checksum_errors<=0;command_count<=0;
        end else begin
            packet_valid<=0;
            if(phase==0 || byte_valid) idle_bits<=0;
            else if(bit_tick) idle_bits<=idle_bits+1'b1;
            if(byte_error || timeout) begin phase<=0;rx_errors<=rx_errors+1'b1;end
            else if(byte_valid) begin
                if(phase==0) begin
                    if(byte_data==8'ha5) begin phase<=1;end
                end else if(phase==1) begin
                    if(byte_data==1) begin phase<=2;end
                    else begin phase<=byte_data==8'ha5 ? 3'd1:3'd0;end
                end else if(phase==6) begin
                    packet_valid<=1;
                    if(checksum!=byte_data) checksum_errors<=checksum_errors+1'b1;
                    else command_count<=command_count+1'b1;
                    phase<=checksum!=byte_data && byte_data==8'ha5 ? 3'd1:3'd0;
                end else begin
                    phase<=phase+1'b1;
                end
            end
        end
    end
    localparam OK=0,BAD_CHECKSUM=1,UNKNOWN=2,BAD_ARGUMENT=3,BUSY=4,UNSUPPORTED=5;
    reg [2:0] status;
    reg [1:0] response_kind;
    wire response_ready;
    reg raw_pending,raw_deliver,raw_failed;
    reg [7:0] raw_sequence;
    reg [15:0] raw_block;
    reg stream_active;
    reg [15:0] stream_index;
    reg [7:0] stream_sequence;
    wire stream_start=RAW_EXPORT_ENABLE && packet_valid && !bad_checksum &&
        command==8'h53 && status==OK && response_ready;
    wire auto_read=RAW_EXPORT_ENABLE && stream_active && !raw_pending && !raw_busy &&
        response_ready && !packet_valid;
    assign raw_index=stream_active?stream_index:value;
    assign raw_request=auto_read || (RAW_EXPORT_ENABLE && packet_valid && !bad_checksum &&
        command==8'h52 && status==OK && response_ready);
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            raw_pending<=0;raw_deliver<=0;raw_failed<=0;raw_sequence<=0;raw_block<=0;
            stream_active<=0;stream_index<=0;stream_sequence<=0;
        end
        else begin
            if(stream_start) begin stream_active<=1;stream_index<=0;stream_sequence<=packet_sequence;end
            if(raw_request) begin
                raw_pending<=1;raw_sequence<=stream_active?stream_sequence:packet_sequence;raw_block<=raw_index;
                if(auto_read) begin
                    stream_index<=stream_index+1'b1;
                    if(stream_index==WIDTH*HEIGHT/16-1) stream_active<=0;
                end
            end
            if(raw_pending && raw_done) begin
                raw_deliver<=1;raw_failed<=raw_error;
                if(raw_error) stream_active<=0;
            end
            if(raw_deliver && response_ready) begin raw_pending<=0;raw_deliver<=0;end
        end
    reg [15:0] response_value;
    reg write_setting;
    wire [7:0] tx_overruns;
    reg [7:0] target_count;
    integer k;
    always @(*) begin
        target_count=0;
        for(k=0;k<TARGETS;k=k+1) target_count=target_count+observed[k];
    end
    // One flat read mux; share page decoding between the three read commands.
    reg [15:0] read_value;
    always @(*) begin
        read_value=0;
        case({command[1:0],value[2:0]})
            5'h00:read_value=3;
            5'h01:read_value=BUILD_ID;
            5'h02:read_value=WIDTH;
            5'h03:read_value=HEIGHT;
            5'h04:read_value=FPS;
            5'h08:read_value={6'd0,fps_x10};
            5'h09:read_value=failed_frames;
            5'h0a:read_value={device_flags,target_count};
            5'h0b:read_value={checksum_errors,rx_errors};
            5'h0c:read_value={tx_overruns,command_count};
            5'h0d:read_value={media_status[51],media_status[26:25],media_status[4],media_status[24:17],media_status[3:0]};
            5'h0e:read_value={applied_algorithm[12],applied_algorithm[13],2'd0,applied_algorithm[11:0]};
            5'h0f:read_value={5'd0,applied_algorithm[11:1]};
            5'h10:read_value=control_state;
            5'h11:read_value={4'd0,manual_threshold};
            5'h12:read_value=RAW_EXPORT_ENABLE?raw_diagnostics:16'd0;
            // Complete five-bit counts from one sample; legacy status page 5
            // retains its original layout for older clients.
            5'h13:read_value={1'b0,media_status[53],media_status[24:21],
                media_status[52],media_status[20:17],media_status[54],media_status[3:0]};
        endcase
    end
    always @(*) begin
        status=OK;response_kind=0;response_value=0;write_setting=0;
        control_address=0;control_value=value;
        case(command)
            8'h00,8'h01,8'h02:begin
                response_kind=1;response_value=read_value;
                if(value[15:3]!=0 || command==0 && value[2:0]>4 || command==2 && value[2:0]>3)
                    status=BAD_ARGUMENT;
                else if(command==1 && value[2:1]==3 && applied_algorithm[11:0]==0) status=BUSY;
            end
            8'h03:begin response_kind=2;response_value=value;
                if(value>=TARGETS) status=BAD_ARGUMENT;end
            8'h10:begin control_address=0;write_setting=1;
                if(value<THRESHOLD_MIN || value>THRESHOLD_MAX) status=BAD_ARGUMENT;end
            8'h11:begin control_address=1;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h12:begin control_address=2;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h13:begin control_address=3;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h14:begin control_address=9;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h15:begin control_address=7;write_setting=1;if(value>6) status=BAD_ARGUMENT;end
            8'h16:begin control_address=8;write_setting=1;
                if(value>31 || value[2:0]>5) status=BAD_ARGUMENT;end
            8'h20:begin // IN, OUT, PRE, SNAP, media previous/next, play/pause
                control_address=11;write_setting=1;
                if(value>7) status=BAD_ARGUMENT;
                else if(!live_ready) status=BUSY;
                else case(value[2:0])
                    0:begin end // IN is an event: repeated IN resets the existing recording origin.
                    1:if(media_status[4] && !record_ready) status=BUSY;
                    2:if(!pre_ready) status=BUSY;
                    3:if(photo_busy) status=BUSY;
                    4,5:if(!media_status[51]) status=BUSY;
                    6,7:begin
                        control_value=6;
                        if(!media_status[51] || !media_status[25]) status=BAD_ARGUMENT;
                        else if(media_status[26]==(value==6)) write_setting=0;
                    end
                endcase
            end
            8'h30:begin control_address=4;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h31:begin control_address=5;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h32:begin control_address=6;write_setting=1;if(value>7) status=BAD_ARGUMENT;end
            8'h33:begin control_address=10;write_setting=1;if(value>1) status=BAD_ARGUMENT;end
            8'h40:begin
                response_kind=1;
                response_value=(PROFILE_ID<<8)|FPS;
                if(value>=4) status=BAD_ARGUMENT;
                else if(value!=0) status=UNSUPPORTED; // Existing profiler remains on HDMI HUD.
            end
            8'h41:begin
                control_address=14;
                if(value>1) status=BAD_ARGUMENT;
                else if(PROFILE_SWITCH_ENABLE && profile_switch_busy) status=BUSY;
                else if(value!=PROFILE_ID) begin
                    if(!PROFILE_SWITCH_ENABLE || PROFILE_ID>1) status=UNSUPPORTED;
                    else if(RAW_EXPORT_ENABLE && raw_locked) status=BUSY;
                    else write_setting=1;
                end
            end
            8'h50:begin
                control_address=12;write_setting=1;
                if(!RAW_EXPORT_ENABLE) status=UNSUPPORTED;
                else if(value!=0) status=BAD_ARGUMENT;
                else if(!raw_available || raw_locked || raw_busy) status=BUSY;
            end
            8'h51:begin
                control_address=13;write_setting=1;
                if(!RAW_EXPORT_ENABLE) status=UNSUPPORTED;
                else if(value!=0) status=BAD_ARGUMENT;
                else if(raw_busy) status=BUSY;
            end
            8'h52:begin
                response_kind=3;
                if(!RAW_EXPORT_ENABLE) status=UNSUPPORTED;
                else if(value>=(WIDTH*HEIGHT+128)/16 && value<16'hfff8) status=BAD_ARGUMENT;
                else if(!raw_locked || raw_busy) status=BUSY;
            end
            8'h53:begin
                if(!RAW_EXPORT_ENABLE) status=UNSUPPORTED;
                else if(value!=0) status=BAD_ARGUMENT;
                else if(!raw_locked || raw_busy) status=BUSY;
            end
            default:status=UNKNOWN;
        endcase
        if(RAW_EXPORT_ENABLE && (raw_pending || stream_active)) status=BUSY;
        if(write_setting && key_event) status=BUSY;
        if(bad_checksum) status=BAD_CHECKSUM;
    end
    assign control_write=packet_valid && !bad_checksum && status==OK && write_setting && response_ready;
    Track_Status_UART #(.TARGETS(TARGETS),.CLOCK_HZ(CLOCK_HZ),.BAUD(BAUD),.REMOTE_ENABLE(1),.FRAME_STREAM_ENABLE(FRAME_STREAM_ENABLE)) transmitter(
        .clk(clk),.rst_n(rst_n),.publish(publish),.boxes(boxes),.digits(digits),.qualities(qualities),
        .lost(lost),.ids(ids),.ages(ages),.vx(vx),.vy(vy),.alive(alive),.observed(observed),
        .fps_x10(fps_x10),.errors(failed_frames),.tx(tx),.busy(),
        .source_safe(source_safe),.source_tag(source_tag),
        .raw_data(raw_data),
        .request_valid((raw_deliver && response_ready) || (packet_valid && !raw_request)),
        .request_sequence(raw_deliver?raw_sequence:packet_sequence),.request_command(raw_deliver?8'h52:command),
        .request_status(raw_deliver?(raw_failed?3'd6:3'd0):status),
        .request_kind(raw_deliver?2'd3:response_kind),.request_value(raw_deliver?raw_block:response_value),
        .response_ready(response_ready),.overruns(tx_overruns));
endmodule
